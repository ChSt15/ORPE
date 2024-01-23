#ifndef DATASTRUCT_H
#define DATASTRUCT_H

#include <stdint.h>


struct OrpePoseEst {

    // Received pose data.
    int16_t px_, py_, pz_, ax_, ay_, az_;
    // Received pose frame number.
    uint32_t frameNum_;

};


#endif