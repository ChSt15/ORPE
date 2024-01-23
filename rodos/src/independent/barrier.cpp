
/**
 * @file barrier
 * @date 2022/05/11 
 * @author Sergio Montenegro
 *
 *
 * @brief Barrier: to synchronize a lot of tasks, They should wait until all of them have reached a certain “barrier.”
 *
 *
 */

#include "barrier.h"
#include "rodos.h"


namespace RODOS {

Barrier::Barrier() : owner(0)  { }

/** caller will be blocked until it gets the unblock signal */
bool Barrier::waitForSignal(bool condition, int64_t timeOutAt) {
  if(owner != 0) return false; // mmm..? Some one else is waiting? Shall never be!!
  bool ok = true;
  {
     PRIORITY_CEILER_IN_SCOPE();

     if(condition) {
        owner =  Thread::getCurrentThread();

        // Sleep until wake up by leave
        Thread::suspendCallerUntil(timeOutAt, this);
        ok = (owner == 0);
     }
  }
  owner = 0;
  return ok;
} 

/**
 *  caller does not block. resumes one waiting thread (enter)
 */
bool Barrier::unblock() {
  if (owner == 0) return false;
  bool ok;
  {
    PRIORITY_CEILER_IN_SCOPE();
    if(owner != 0) owner->resume();
    ok = (owner != 0);
    owner = 0;
  } // end of PRIORITY_CEILER, 
  return ok;
}

} // namespace
