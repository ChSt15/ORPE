
/** To compute this value please use tutorials/development-tests/cpuspeedtest.cpp **/

/**
* @file hostinfos.cpp
* @date 2008/04/24 16:13
* @author Marcelo Alves, Jorge Oliveira
*
*
* @brief configuration parameters for the current platform
*
*/

#include "rodos.h"

namespace RODOS {


bool  		getIsHostBigEndian()          	{ return isHostBigEndian; }  	// Yes! Actually getIsHostBigEndian is host independent
long  		getSpeedKiloLoopsPerSecond()  	{ return 1032;  } 		// see tutorials/20-core/compute-hostinfos.cpp
long  		getMaxIdleCntPerSecond()      	{ return 24635; } 		// see tutorials/20-core/compute-hostinfos.cpp
long  		getYieldTimeOverhead()        	{ return 39843; } 		// see tutorials/20-core/compute-hostinfos.cpp
const char* 	getHostCpuArch()        	{ return "Cortex-M4 (EFR32FG1P) ";    }
const char* 	getHostBasisOS()        	{ return "baremetal"; }
}
