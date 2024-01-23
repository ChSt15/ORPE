
The most simple way to start using RODOS is to use
a virtual mashine where every thing (and compilers) is already installed.

Setting up the Rodos Virtual Machine
-------------------------------------

We provide an Ubuntu 20.4 Linux Image, which includes all required compilers and libraries and all RODOS sources.

First, install Virtualbox and the matching VirtualBox Extension Pack from oracle.
                    
Download the RODOS image (3.4 Gbytes to download, ca 10 Gbytes disk space required)  
    > https://oc.informatik.uni-wuerzburg.de/s/p7J6HHMTGdtdXRJ   
    Download Password: rodosVM64Bit

Here you find Information how create a Virtual Machine from the RODOS Image:  
    > https://www.informatik.uni-wuerzburg.de/aerospaceinfo/forschung-und-entwicklung-prof-dr-sergio-montenegro/projekte/rodos/rodos-vm-installation-english-version/

Problems with the VM?  
There is a VM Troubleshooting guide at the end of the howto.

To Login:  
   > user: rodos  
   Pass: rodos2020


To work on an embedded system: Discovery Board/STM32f4
------------------------------------------------------

Download the eclipse project from gitlab

Connect your discovery board to an USB port

From Shell:  
     https://youtu.be/cwXLvjuV8Wk 

Make the rodos lib, compile your program, load and execute on discoevery board  

>
    cd eclipse-workspace  
    cd rodosTemplate    
    cd rodos  
    source setenvs.sh   
    rodos-lib.sh discovery   
    cd MyFiles/  
    rodos-executable.sh discovery *.cpp    
    arm-none-eabi-objcopy -S -O binary tst  myExecutable.bin
    cp myExecutable.bin /media/rodos/DIS_F407
    # now press reset on the discovery board


Using an IDE (on Windows and Ubuntu):  
    https://www.youtube.com/watch?v=vuDY7197kiE&list=PL7o5WShpbf0wEInq8lV_SR1qt84WxjuSb

    1. Download and install the "STM32Cube IDE", 
        which is a Eclipse derivative from the STM32 producer.
        https://www.st.com/en/development-tools/stm32cubeide.html
    2. Start the IDE
    3. Clone RODOS from gitlab (eclipse-project branch only)
        https://gitlab.com/rodos/rodos
    4. Import RODOS-Project 
    5. Import HelloWorld-Project (in „rodos/tutorials/00-empty-rodos-project/“)
    6. Compile and debug (with Debug_STM32F407 for Discovery Board).
        when flashing for the first time please select „STM32 Cortex-M C/C++ Application“


