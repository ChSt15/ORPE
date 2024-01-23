/** READ THIS!!!! **/
/** To compute this value please use tutorials/development-tests/cpuspeedtest.cpp **/

#include "hostinfos.h"
#include "stream-bytesex.h"

namespace RODOS {

bool getIsHostBigEndian()
{
    return isHostBigEndian;
}

/* see .../development-tests/cspuspeedtest.cpp
 * !!! speed value depends on compiler optimization !!!
 */
// long  getSpeedKiloLoopsPerSecond()  { return 2434; } // -00
// long  getMaxIdleCntPerSecond()      { return 80154; } // -O0
// long  getSpeedKiloLoopsPerSecond()  { return 5197; } // -03
// long  getMaxIdleCntPerSecond()      { return 327627; } // -O3
long getSpeedKiloLoopsPerSecond() { return 4872; } // see .../development-tests/cspuspeedtest.cpp
long getMaxIdleCntPerSecond() { return 279547; } // see rodos-tutorials/development-tests/cpuspeedtest.cpp
long getYieldTimeOverhead() { return 12500; } // see tutorials/core/yieldtime.cpp
const char* getHostCpuArch() { return "cortex-M4 (VA41620)"; }
const char* getHostBasisOS() { return "baremetal"; }

}
