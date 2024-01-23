#include <iostream>
#include <chrono>
#include <thread>
#include <filesystem>

#include <lccv.hpp>
#include "opencv2/opencv.hpp"
#include <opencv2/videoio.hpp>

#include <rodos.h>

#include "PoseEstimator.h"
#include "DataLogger.h"

#include "Datalink.h"

//Below are settings for debugging and runtime
#define RUN_ESTIMATOR
#define CAMERA_ISO -0.1
#define CAMERA_EV  0
#define CAMERA_SS  10 //Shutter speed in microseconds
//#define SHOW_DEBUG_IMAGE
#define COMMS_BUFFERLIMIT 256
//#define WRITE_DEBUG_VIDEO
//#define WRITE_DEBUG_IMAGE
#define PRINT_LEDS
#define PRINT_EST
#define TIME_LIMIT 20
#define FORCE_ESTIMATION_START
//#define RUN_ONCE
#define FPS 20

#define FLOATSAT //Use LED pattern for floatsat instead of CubeSat

using namespace cv;
using namespace std;


#ifdef FLOATSAT

vector<LED> modelPoints = {
    LED(cv::Point3f(-129.6, 38.7, 0), 0b01011110),
    LED(cv::Point3f(-141.3, -43, 0), 0b10101111),
    LED(cv::Point3f(-82.6, 16.3, 0), 0b11010111),
    LED(cv::Point3f(-49.4, -20.2, 0), 0b01011011),
    //LED(cv::Point3f(-22.5, 27.6, 0), 5),
    //LED(cv::Point3f(25.8, 2.7, 0), 6),
    LED(cv::Point3f(44.8, -25, 0), 0b11011010),
    LED(cv::Point3f(80, 34.5, 0), 0b11010000),
    LED(cv::Point3f(104.9, -33, 0), 0b10010110),
    LED(cv::Point3f(136.3, 0, 0), 0b11110101)};

#else

vector<LED> modelPoints = {
    LED(cv::Point3f(0, 0, 0), 0b01011110),
    LED(cv::Point3f(0, 86, 0), 0b10101111),
    LED(cv::Point3f(100, 0, 0), 0b11010111),
    LED(cv::Point3f(100, 86, 0), 0b01011011),
    LED(cv::Point3f(200, 0, 0), 0b11011010),
    LED(cv::Point3f(200, 86, 0), 0b11010000),
    LED(cv::Point3f(307, 0, 0), 0b10010110),
    LED(cv::Point3f(307, 86, 0), 0b11110101)};

#endif

Mat cameraMatrix = (cv::Mat_<float>(3, 3) << 1.716e3, 0, 5.9937e2, 0, 1.719e3, 3.9096e2, 0, 0, 1); // 720p
Mat cameraDistorsionMatrix = (cv::Mat_<float>(5, 1) << 2.3672e-1, -1.2357, -2.7905e-3, -6.1804e-3, 6.5872); //720p

Mat imageRead;
Mat imageWrite;

volatile bool dataReady = false;

std::mutex imReadMut;
std::mutex imWriteMut;
std::mutex dataReadyMut;

volatile bool killThreads = false;
volatile bool imageReadError = false;

void setDataReady() {

    dataReadyMut.lock();
    dataReady = true;
    dataReadyMut.unlock();

}

bool getDataReady() {

    dataReadyMut.lock();
    bool ready = dataReady;
    dataReady = false;
    dataReadyMut.unlock();
    return ready;

}

// Object to control communication with flight software
//Comms comPort;

//lccv::PiCamera cameraObj;

/// @brief Reads the next image into the global imageRead mat. Used for multithreading
/// @param capture
void imageReadFunc()
{
    //imageRead = imageRead;
    lccv::PiCamera cam;
	
    cam.options->video_width=1280;
    cam.options->video_height=720;
    cam.options->framerate=FPS;
    cam.options->verbose=true;
    cam.options->shutter=CAMERA_SS;
    cam.options->gain=CAMERA_ISO;
    cam.options->ev=CAMERA_EV;
    cam.startVideo();

    bool leave = false;
    imageReadError = false;
    Mat image;

    if (!cam.getVideoFrame(image, 5000)) {
	cout << "Failed to init camera" << endl;
	imReadMut.lock();
	imageReadError = true;
	imReadMut.unlock();
	setDataReady();
	imageReadError = true;
    }

    while (!leave && !imageReadError) {
    	
	if (cam.getVideoFrame(image, 1000)) {

	    imReadMut.lock();
	    image.copyTo(imageRead);
	    leave = killThreads;
	    imReadMut.unlock();
	    setDataReady();

	} else {
	
	    imReadMut.lock();
	    imageReadError = true;
	    imReadMut.unlock();
	
	}

    }

    cam.stopVideo();
    imageReadError = true;
    setDataReady(); //Get main loop out of waiting;

}

/// @brief Writes the global imageWrite mat. Used for multithreading
/// @param capture
void imageWriteFunc(cv::VideoWriter &writer)
{

    writer.write(imageWrite);
}

std::string gstreamer_pipeline(int capture_width, int capture_height, int framerate, int display_width, int display_height) {
    return
            " libcamerasrc ! video/x-raw, "
            " width=(int)" + std::to_string(capture_width) + ","
            " height=(int)" + std::to_string(capture_height) + ","
            " framerate=(fraction)" + std::to_string(framerate) +"/1 !"
            " videoconvert ! videoscale !"
            " video/x-raw,"
            " width=(int)" + std::to_string(display_width) + ","
            " height=(int)" + std::to_string(display_height) + " ! appsink";
}

/**
 * @brief Runs the PoseEstimator with a name. If output video is enabled then the output video is given the videoFile parameter as name.
 *
 * @param videoFile
 */
void runEstimatorWithName(std::string videoFile)
{
    //getBuildInformation();
    // Project file path
    std::string directoryPath = "/home/pi/orpeLCCV/ORPE"; // ########### THIS MUST BE CHANGED. Path upto the project home.

    cout << "Estimator output in: " << videoFile << endl;
    //while(1);
    PoseEstimator estimator(modelPoints, cameraMatrix, cameraDistorsionMatrix);
    // DataLogger logger(directoryPath, videoFile);
    //while(1);
    // cout << "Opening input video: " << (directoryPath + "/data/Videos/" + videoFile + ".mp4") << endl;
    //  cv::VideoCapture capture(directoryPath + "/data/Videos/" + videoFile + ".mp4");

    //std::string ss = "nvarguscamerasrc !  video/x-raw(memory:NVMM), width=1920, height=1080, format=NV12, framerate=10/1 ! nvvidconv flip-method=2 ! video/x-raw, width=480, height=680, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink";

    //lccv::PiCamera capture;
    //capture.set(CAP_PROP_FORMAT, CV_8UC1 );

    //cv::VideoCapture capture(fifoPath);
    //capture.set(CAP_PROP_FRAME_HEIGHT, 960);
    //capture.set(CAP_PROP_FRAME_WIDTH, 1280);
    //capture.set(CAP_PROP_FPS, FPS);
    //capture.set(CAP_PROP_GAIN, CAMERA_ISO); // Set camera ISO to 100
    //capture.set(CAP_PROP_EXPOSURE, CAMERA_SS);
    //while(1);
    
    //while(1);
    cv::Point3f rVec, tVec; // Declared here to keep old values for iterative pnp-solvers
    rVec = tVec = cv::Point3f(1, 1, 1);

    float estTime = 100000;

// int ex = static_cast<int>(capture.get(CAP_PROP_FOURCC));
// cout << "Opening output video" << endl;
#ifdef WRITE_DEBUG_VIDEO
    cv::VideoWriter writer;
    writer.open(directoryPath + "/data/video/" + videoFile + ".mp4", cv::VideoWriter::fourcc('m', 'p', '4', 'v'), FPS, Size(1280, 720));
#endif
	
    killThreads = false;
    imageReadError = false;
    getDataReady(); //Sets ready to false;

    Mat image;
    //thread imageReadThread(imageReadFunc);

    lccv::PiCamera cam;
	
    cam.options->video_width=1280;
    cam.options->video_height=720;
    cam.options->framerate=FPS;
    cam.options->verbose=true;
    cam.options->shutter=CAMERA_SS;
    cam.options->gain=CAMERA_ISO;
    cam.options->ev=CAMERA_EV;
    cam.startVideo();

    //if (!capture.getVideoFrame(image,5000))
    //{
    //    cout << "Failed to open input video!" << endl;
        //capture.stopVideo();
        //return;
    //}

#ifdef WRITE_DEBUG_VIDEO
    if (!writer.isOpened())
    {
        cout << "Failed to open output video!" << endl;
        return;
    }
#endif
    //while(1);

#ifdef WRITE_DEBUG_VIDEO
    thread imageWriteThread;
#endif

    auto progBegin = chrono::steady_clock::now();
    uint32_t frameCounter = 0;
	
    bool leaveLoop = false;
    while (!leaveLoop)
    {
	//continue;
	int64_t time_ms = chrono::duration_cast<std::chrono::milliseconds>(chrono::steady_clock::now() - progBegin).count();
        int64_t nextLoop = time_ms - time_ms%(1000/FPS) + 1000/FPS;
        //while(chrono::duration_cast<std::chrono::milliseconds>(chrono::steady_clock::now() - progBegin).count() >= nextLoop) //Wait to limit FPS.
	auto loopBeginTimestamp = chrono::steady_clock::now();
        time_ms = chrono::duration_cast<std::chrono::milliseconds>(chrono::steady_clock::now() - progBegin).count();

        std::cout << "Time: " << time_ms << std::endl;

        //if (imageReadThread.joinable());
        //    imageReadThread.join(); // Wait for thread to finish reading
        auto failedRead = !cam.getVideoFrame(image, 1000);
	                                   // Get new image
        //imageReadThread = thread(imageReadFunc, std::ref(capture)); // Restart thread;
	//capture.grab();
	//capture.retrieve(image);

	if (failedRead) {
	    std::cout << "Failed to read image! Exiting!" << std::endl;
	    leaveLoop = true;
	}

        if (image.empty())
        {
            std::cout << "Image was empty! Skipping frame!" << std::endl;
            continue;
        }

#ifdef RUN_ESTIMATOR
        estimator.giveEstimatorNextImage(image);
#endif

        auto poseEstBegin = chrono::steady_clock::now();
        estimator.estimatePose();
        uint32_t poseEstTime = std::chrono::duration_cast<std::chrono::microseconds>(chrono::steady_clock::now() - poseEstBegin).count();

        estTime = estTime * 0.9 + poseEstTime * 0.1;

        bool estimationGood = estimator.getPoseEstimation(rVec, tVec);
        // |= estimationGood;
        if (estimationGood)
        {
#ifdef PRINT_EST
	    cout << "Rot: " << rVec << ", Pos: " << tVec << endl;
#endif
            cv::drawFrameAxes(image, cameraMatrix, cameraDistorsionMatrix, cv::Mat(rVec), cv::Mat(tVec), 20);
            frameCounter++;
        }

	//comPort.sendEstimation(tVec.x, tVec.y, tVec.z, rVec.x, rVec.y, rVec.z, frameCounter);

#ifdef WRITE_DEBUG_VIDEO
#ifdef PRINT_LEDS
	for (auto const& p: estimator.getCurrentPoints()) {
            p.drawLED(image);
        }
#endif
#endif

#ifdef PRINT_LEDS
	cout << "Current LEDs: " << endl;
	for (auto const& p: estimator.getCurrentPoints()) {
	    cout << " - " << p.getId();
	    if (p.getId() > 0) cout << " Identified";
	    cout << endl;
	}
#endif

        // cout << tVec << endl;

        // logger.logData(cv::Mat(rVec), cv::Mat(tVec), estimationGood, estimator.getCurrentFrameNumber(), poseEstTime);

        //uint32_t loopTime = std::chrono::duration_cast<std::chrono::microseconds>(loopEndTimestamp - loopBeginTimestamp).count();

#ifdef WRITE_DEBUG_VIDEO
        //std::ostringstream text;
        //text << "Loop time: " << (loopTime / 1000) << "ms. Estimation time avg: " << float(uint32_t(estTime / 10)) / 100 << "ms";

        //cv::putText(image, text.str(), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255));

        cv::resize(image, image, cv::Size(1280, 720));

        if (imageWriteThread.joinable())
            imageWriteThread.join();
        image.copyTo(imageWrite);
        imageWriteThread = thread(imageWriteFunc, std::ref(writer));
#endif

#ifdef WRITE_DEBUG_IMAGE
        cv::imwrite(directoryPath + "/data/video/outputs/" + videoFile + ".jpg", image);
#endif

#ifdef SHOW_DEBUG_IMAGE
        cv::imshow("Estimation", image);
#endif

        std::cout << "Estimation Time: " << float(uint32_t(estTime / 10)) / 100.0f << std::endl;

        // int32_t waitTime = int32_t(1000/videoFPS) - std::chrono::duration_cast<std::chrono::microseconds>(chrono::steady_clock::now() - lastFrameTimestamp).count()/1000;
        // lastFrameTimestamp = chrono::steady_clock::now();
        // if (videoFPS < 0) waitTime = 1;
        // if (waitTime < 1) waitTime = 1;

        // if (waitKey(waitTime) == 27) {
        //     std::cout << "User Exiting program" << std::endl;
        //    break; //Wait long enough for next frame to be timed correctly. If ESC pressed, then exit loop.
        //}

        //comPort.update();

        /*if (comPort.powerCommandIsNew())
        {

            bool powerCommand;
            comPort.getPowerCommand(powerCommand);

            if (!powerCommand)
                break; // Turn off command.
        }*/

        bool powerCommand = false;
        if (orpePowerCommandBuf.getOnlyIfNewData(powerCommand)) {

            if (!powerCommand) 
                break;
            
        }

	//orpeTestingTopic.publish(RODOS::SECONDS_NOW());
	//cout << "Time sec: " << RODOS::SECONDS_NOW() << endl;

#ifdef TIME_LIMIT

        if (std::chrono::duration_cast<std::chrono::milliseconds>(chrono::steady_clock::now() - progBegin).count() > 1000 * TIME_LIMIT)
            break;

#endif
    }

    std::cout << "Estimator stopping!" << std::endl;
	
    //imReadMut.lock();
    //killThreads = true;
    //imReadMut.unlock();
    cam.stopVideo();

#ifdef WRITE_DEBUG_VIDEO
    if (imageWriteThread.joinable())
        imageWriteThread.join();
#endif

    //capture.stopVideo();

#ifdef WRITE_DEBUG_VIDEO
    writer.release();
#endif

    destroyAllWindows();
}


void threadTest() {

    while (1) {
	
	cout << "Time: " << SECONDS_NOW() << endl;
        auto begin = NOW();
        while (NOW() - begin < 1*SECONDS);

    }	

}


class ORPEMain : public RODOS::StaticThread<> {
public:

    ORPEMain() : StaticThread<>("ORPEMain Thread") {}

    void init() override {

    }

    void run() override {

        std::cout << "Starting Estimator software.\n Communication initialising..." << std::endl;

        //comPort.initialiseUart();

        std::cout << "Communication initialised. Starting loop. Will wait for power on command." << std::endl;
	
	//auto t1 = thread(threadTest);

	//while(t1.joinable()) 
	//    suspendCallerUntil(NOW() + 100*MILLISECONDS);

        while (1)
        {
	    //orpeTestingTopic.publish(RODOS::SECONDS_NOW());
	    //cout << "Time sec: " << RODOS::SECONDS_NOW() << endl;
            //suspendCallerUntil(NOW() + 100*MILLISECONDS);
	    //continue;
	    //comPort.update();
	    bool powerCommand = false;
            if (orpePowerCommandBuf.getOnlyIfNewData(powerCommand)) {

                if (powerCommand) {
                    //runEstimatorInThread("PowerCommandRun");
		    auto t = thread(runEstimatorWithName, "Commanded");
		    while (t.joinable())
			suspendCallerUntil(NOW() + 100*MILLISECONDS);
                }
                
            }

    #ifdef FORCE_ESTIMATION_START
            std::cout << "Forcing estimator to start. This is a debug setting!" << std::endl;
            auto t = thread(runEstimatorWithName, "Forced");
	    while (t.joinable())
	        suspendCallerUntil(NOW() + 100*MILLISECONDS);
    #endif

    #ifdef RUN_ONCE
            std::cout << "Exiting due to RUN_ONCE setting!" << std::endl;
            break;
    #endif

            suspendCallerUntil(RODOS::NOW() + 100*RODOS::MILLISECONDS);
        }

        //comPort.closeUart();
        
    }


};

ORPEMain orpeApp;
