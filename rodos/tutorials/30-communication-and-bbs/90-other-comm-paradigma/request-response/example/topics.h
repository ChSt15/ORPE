
#pragma once

#include  "rodos.h"
#include "reqresp.h"

struct Vector3D { 
    double x, y, z;
};

struct Vector6D { 
    double x, y, z;
    double deltaX, deltaY, deltaZ;
};


extern ReqResp<Vector3D, Vector6D>  estimator;

