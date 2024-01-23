
#pragma once

#include  "rodos.h"
#include "reliable-topic.h"
  
struct Vector3D {
    double x, y, z;
};
  
extern ReliableTopic <Vector3D>  position;

