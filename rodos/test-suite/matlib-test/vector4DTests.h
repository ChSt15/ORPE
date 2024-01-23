//Tests the functions of Vector4D

#include "random.h"

#define FAIL {PRINTF("FAILED at line %d in file %s\n", __LINE__, __FILE__); failed++;};  

#define NUMBER_OF_TESTS     100
#define RANGE               5

int vector4DTests() {
    int failed = 0;
    
    double elements[16];    
    HTransform m4d;
    HCoord v4d, res, res2;
    
    //Identity matrix
    for (int i = 0; i < 16; i++) {
        if ((i % 5) == 0) {
            elements[i] = 1;
        } else {
            elements[i] = 0;
        }
    }
    m4d = HTransform(elements);
    
    for (int i = 0; i < NUMBER_OF_TESTS; i++) {
        v4d = HCoord(drand(RANGE), drand(RANGE), drand(RANGE));
        
        res = m4d * v4d;	

        if (!v4d.equals(res)) FAIL;
    }
    
    //Arbitrary matrix
    for (int i = 0; i < NUMBER_OF_TESTS; i++) {
        v4d = HCoord(drand(RANGE), drand(RANGE), drand(RANGE));
        
        for (int i = 0; i < 12; i++) {
            elements[i] = drand(RANGE);
        }
        elements[12] = 0;
        elements[13] = 0;
        elements[14] = 0;
        elements[15] = 1;
        m4d = HTransform(elements);
        
        res = m4d * v4d;	

        res2.r[0][0] = elements[0]*v4d.r[0][0] + elements[1]*v4d.r[1][0] + elements[2]*v4d.r[2][0] + elements[3]*v4d.r[3][0];
        res2.r[1][0] = elements[4]*v4d.r[0][0] + elements[5]*v4d.r[1][0] + elements[6]*v4d.r[2][0] + elements[7]*v4d.r[3][0];
        res2.r[2][0] = elements[8]*v4d.r[0][0] + elements[9]*v4d.r[1][0] + elements[10]*v4d.r[2][0] + elements[11]*v4d.r[3][0];
        res2.r[3][0] = elements[12]*v4d.r[0][0] + elements[13]*v4d.r[1][0] + elements[14]*v4d.r[2][0] + elements[15]*v4d.r[3][0];
        
        if (!res2.equals(res)) FAIL;
    }
    
    return failed;
}
