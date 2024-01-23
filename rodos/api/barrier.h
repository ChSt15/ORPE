#pragma once

#include "thread.h"

namespace RODOS {

/**
* @class Barrier
* @brief a synchronisation primitive
*
* To Synchronize only one waiting Thread! 
* One thread will be waiting until it gets reactivated whit unblock
*
*/

class Barrier {

private:
  Thread* volatile owner;   ///< A pointer to the thread that is waiting to be reactivated
public:
  Barrier();

  /** caller will be blocked until some one else gives the signal to continue  or timeout 
   * Returns false in case of time out, else true
   * It waits only if the condition is true
   */
  bool waitForSignal(bool condition = true, int64_t timeOutAt = END_OF_TIME);

  /** caller does not block. Resumes one waiting thread , in no one is waiting returns false, else true */
  bool unblock();
};

} // namespace

