
#pragma once
#include <stdint.h>

const int32_t  udpPortNr          = -50000; // < 0 means allow multiple receivers
const uint32_t topicIdLinux2Rodos = 1002;
const uint32_t topicIdRodos2Linux = 1003;


// data from linux to rodos
struct Position {
  char name[20];
  int32_t cnt;
  double x, y, z;
};

// data rodos to linux
struct MyTime {
  int64_t timeNow;
  int32_t msgIndex;
};


