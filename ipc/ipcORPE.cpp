#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <cerrno>

#include "../include/Datastruct.h"
#include "ipcORPE.h"

#define EST_PIPE "/var/lock/poseEst"
#define PWR_PIPE "/var/lock/pwrCmd"


int estPipeID;
int pwrPipeID;


void sendORPEEstimation(OrpePoseEst& poseEst) {

    auto ret = write(estPipeID, &poseEst, sizeof(OrpePoseEst));

    if (ret < 0) {
        printf("Error when writing est pipe. %s\n", strerror(errno));
    }

}

bool getORPEPowerCommand(bool& powerOn) {

    auto ret = read(pwrPipeID, &powerOn, sizeof(bool));

    if (ret < 0) {
        printf("Error when reading pwr pipe. %s\n", strerror(errno));
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

    unlink(EST_PIPE);
    unlink(PWR_PIPE);

}