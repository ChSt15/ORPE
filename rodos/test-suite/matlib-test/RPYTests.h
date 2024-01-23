//Tests the functions of RPY except the rotations (see rotationTests.h)
#include "random.h"
#define NUMBER_OF_TESTS     100
#define RANGE               5

#define FAIL {PRINTF("FAILED at line %d in file %s\n", __LINE__, __FILE__); failed++;};  

int RPYTests() {
    int failed = 0;

    for (int i = 0; i < NUMBER_OF_TESTS; i++) {
        YPR r1, r2, rtot;
        Vector3D start, end;
        
        start.x = drand(RANGE);
        start.y = drand(RANGE);
        start.z = drand(RANGE);
        
        r1.yaw   = abs(drand() * 2 * M_PI);
        r1.pitch = abs(drand() * 2 * M_PI);
        r1.roll  = abs(drand() * 2 * M_PI);
        
        r2.yaw   = abs(drand() * 2 * M_PI);
        r2.pitch = abs(drand() * 2 * M_PI);
        r2.roll  = abs(drand() * 2 * M_PI);
        
        rtot = r1.accumulateRotation(r2);
        end = start.yprRotate(r2).yprRotate(r1);
        
        if (!end.equals(start.yprRotate(rtot))) FAIL;
        
    }
    
    return failed;
}
