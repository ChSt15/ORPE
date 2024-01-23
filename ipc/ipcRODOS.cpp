#include <stddef.h>

#include "Datastruct.h"
#include "ipcRODOS.h"


#define EST_PIPE "/var/lock/poseEst"
#define PWR_PIPE "/var/lock/pwrCmd"


int estPipeID;
int pwrPipeID;


void sendORPEPowerCommand(bool powerOn) {

    auto ret = write(pwrPipeID, &powerOn, sizeof(bool));

    if (ret < 0) {
        printf("Error when writing pwr pipe. %s\n", strerror(errno));
    }

}

bool getORPEEstimation(OrpePoseEst& poseEst) {

    auto ret = read(estPipeID, &poseEst, sizeof(OrpePoseEst));

    if (ret < 0) {
        printf("Error when reading est pipe. %s\n", strerror(errno));
        return false;
    }

    return true;
    
}

void ipcInit() {

    int ret = mkfifo(EST_PIPE, 0666);
    if (ret < 0)
        printf("Error when creating est pipe. %s\n", strerror(errno));


    estPipeID = open(EST_PIPE, O_WRONLY);

    ret = mkfifo(PWR_PIPE, 0666);
    if (ret < 0)
        printf("Error when creating est pipe. %s\n", strerror(errno));


    pwrPipeID = open(PWR_PIPE, O_WRONLY);
    
}

void ipcDeinit() {
    
    close(estPipeID);
    close(pwrPipeID);

    unlink(estPipeID);
    unlink(pwrPipeID);

}