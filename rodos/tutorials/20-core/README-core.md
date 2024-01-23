# RODOS

# Tutorial README Core

### Author: S. Montenegro

## 1. How To begin

### 1.1. RODOS directory structure

|              |                                                               |
|-------------:|:--------------------------------------------------------------|
|        `api` | API and implementations for many different hardware platforms |
|      `build` | Compiled libraries                                            |
|        `doc` | Documentation                                                 |
|    `scripts` | Scripts for compilations on many different hardware platforms |
|        `src` | Implementations for various platforms                         |
|    `support` | Programs and libs often used in space applications           |
| `test-suite` | test suite for rodos functions                                |
|  `tutorials` | current directory, tutorials                                  |

### 1.2. Steps to compile and execute a RODOS program

1. Open a Terminal using the **bash** shell
2. Enter the **RODOS root** directory
3. Set some shell variables that are needed by the compilation scripts

   ```
   source setenvs.sh
   ```

   It has to be executed **every time** when **opening a new terminal!**
4. Compile the RODOS library for a Linux x86 PC

   ```
   rodos-lib.sh linux-makecontext
   ```

    - shell completion with *tab* shows the alternatives
    - Has to be done **only once** for every RODOS version, unless something in folder src or api has been modified.
5. Enter the folder with the user program

   ```
   cd tutorials/20-core
   ```

6. You may use different ports and target architectures. In these examples we will use
   linux-make-context (`linux-makecontext`). To see the list of alternatives, see in the build-scripts directory the
   files in set-vars.
7. Compile the user program

   ```
   rodos-executable.sh linux-makecontext usercode1.cpp usercode2.cpp ...
   ```

8. Execute the binary

   ```
   ./tst
   ```
9. Exit the program with Ctrl+C. If you want to terminate the execution automatically, add the
   program `terminate-test.cpp` to your source code list, then it will be terminated after 20 seconds.

For each example programme please first read and understand the code, then compile and execute and see if it acts as
expected. Then modify and continue trying. Try to combine different programs together.

### Useful to know:

- Time unit = nanoseconds, stored in 64 bits (int64_t)
- Threads implement co-operative switch (yield) and preemption
- Priorities begin with 1 (lowest) up to 2^31, stored in long
- Be aware of the printouts when you execute an application. You will see there the configuration of the system and the
  list of loaded modules. Then take a look to the sources and try to modify them to add more functionality. We have very
  few macros and only for operations which are used very often. We recommend using such macros: (to find examples use
  for example `%grep PROTECT_WITH *`)

|                                             |                                                                                                                                                                 |
|:--------------------------------------------|:----------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `NOW()`                                     | Returns the current time (in nanoseconds)                                                                                                                       |
| `SECONDS_NOW()`                             | Returns the current time in seconds                                                                                                                             |
| `AT(time)`                                  | Suspends (interrupts) the thread that has called this method, until the given point in time is reached. AT is a short cut for `suspendCallerUntil(time)`.       |
| `TIME_LOOP(firstExecution, Period) { ... }` | Almost each control loop has a start time and a period. This macro provides this loop with no end.                                                              |
| `PROTECT_WITH_SEMAPHORE(semaphore) { ... }` | Critical blocks which shall be protected by semaphores, have almost always such structure. This macro uses the given semaphore to protect the region in { ... } |
| `PRIORITY_CEILING { ... }`                  | To deactivate scheduling for a very short time to protect very short critical section.                                                                          |

## 2. Programs

If not mentioned otherwise, you are expected to compile and execute the addressed `.cpp` file as described
in [section 1.2.](#12-steps-to-compile-and-execute-a-rodos-program)

### 2.1. Threads and Time

#### timewaitfor.cpp

This is a thread which waits for a time point. It loops and waits 2 seconds in each loop. Check to see if the time
difference of each execution is more than 2 seconds, because of the time consumed in the thread execution.

#### timewaitbeats.cpp

Periodic threads. It loops and prints in beats of 2 seconds. The beat shall have a minimal delay and no shifts as in

Compile and execute `timewaitbeats.cpp`. Also compile and execute `timewaitfor.cpp` and `timewaitbeats.cpp` together.
Observe the time shift. For exact beats use `waitUntilNextBeat()`.

#### timewaitbeats_shortcut.cpp

A shortcut for timebeats using macros.

Compile and execute `timewaitbeats_shortcut.cpp`. Also Compare with `timewaitbeats.cpp`.

#### timeevents.cpp

Reaction to pre-defined time points without using threads. Instead it uses event handlers. The event handler can react
to internal events, interrupts and time events.

Compile and execute `timeevents.cpp`. Combine it with other threads such as `timewaitfor.cpp`.

#### timeevent_resumer01.cpp & timeevent_resumer02.cpp

`timeevent_resumer.cpp` shows how a time event resumes a waiting thread. This other thread waits in a time-loop. Despite
extra resumes, the beat does not get lost.

Compile and execute `timeevent_resumer01.cpp` together `timeevent_resumer02.cpp`.

#### utctest.cpp

Can be found in `tutorials-with-acceptable-warnings` Shows how to use UTC time (time 0 is 1th of jan 2000 at 0:00) and
calendar time.
(Calendar time is useless for internal computations but nice to communicate with people)

#### timepoints.cpp

Used to manage a (small) log file of time points and print them. Usable for debug and time tuning.

### 2.2. Priorities

#### preemptiontest.cpp

Two threads, one with a high priority which is executed very shortly every 3 seconds and one with a low priority which
is executed constantly. The high priority thread (printing `*`) shall assume the CPU when it needs it even if the low
priority thread (Printing `.`) does not suspend or yield.

See the application definition, which now uses an application ID (2000) to use the application discovery protocol with
other applications (see `ceiler`).

#### ceiler.cpp

This is a demonstrator of priority ceiling to keep the CPU for a critical block. It will search for preemption
application (using the application ID 2000). If not found it reports a warning.

Compile and execute `ceiler.cpp`. Compile and execute different combinations, such as `ceiler.cpp` together
with `preemptiontest.cpp` or `preemptiontest.cpp` together with `timeevents.cpp`.

Even if `ceiler` has a low priority it keeps the CPU when it is inside the critical block in `PRIORITY_CEILING ( ... )`.
It will not be interrupted by other threads when it is inside this block. But time events and other events can not be
blocked. They always run.

### 2.3. Local Communication

Local communication uses shared memory. To provide consistency in a concurrent system some means of synchronisation has
to be used or else objects which can guarantee concurrent access without blocking threads. These objects are the means
of communication between event-handlers, and between event-handlers and threads, because there we cannot suspend events.

#### commbufftest.cpp

Is a double buffer with only one writer and only one reader. Both can work concurrently. The writer may write at any
time. The reader gets the newest consistent data (e.g. The last complete written record)

#### commbufftest-struct.cpp

Like `commbufftest.cpp`, but using more complex structures than just integers.

#### fifotest.cpp

For synchronous communication from one single writer to one single reader. Neither will be suspended. Writing to a full
FiFo has no effect and returns 0. Reading from an empty FiFo returns 0.

### 2.4. Thread synchronisation

Thread synchronisation deals with suspending and resuming threads to synchronise their activities. To avoid concurrent
access of critical sections semaphores must be used. To synchronise threads on data transfer use SyncFiFos.

#### syncfifotest.cpp

A sender and a receiver which wait until data is ready.

#### semaphoretest.cpp

Many threads try to access a protected area. Only one is allowed to enter at a time.

#### semaphore_worm.cpp

A (semi) graphical demonstration of semaphores. Several threads move with random speeds each a letter in a screen area (
30x30 Characters). The right part of the screen is protected by a semaphore. Only one thread may move its letter in the
right part, all other which want to enter have to wait until the one which occupies the semaphores exits the protected
area.

Note: The terminal size has to be at least 30x30 characters (check the lower limit).

### 2.5. Events/Interrupts

With events, you may react to interrupts like timers and signals from devices. You may need this when programming I/O
drivers.

#### resumefromevent.cpp

A thread can be suspended and will wait until another thread that knows him resumes him. For example an event handler.

#### Following examples for interrupts: see [tutorials/70-network-and-io/30-interrupts](../70-network-and-io)

#### externalevents01.cpp

It reacts to external interrupts. In the Linux simulation we have only one external interrupt, and it is simulated by
the Linux command “kill” or “killall”. Using kill we can generate a signal number 15 (Linux stuff).

Use two Linux windows. In one compile and execute `externalevents01.cpp`. Execute tst. In the other window call

```
killall tst
```

#### externalevents02.cpp

Like `externalevents01.cpp`, but it uses the default event::handle, which searches for a thread waiting for this signal
and resumes it.

#### 2.6. Test programmes

#### yieldtime.cpp

Example how to use co-operative threads. It measure the time for thread switches in nanoseconds.

## 10. Other files

|                               |                                      |
|:------------------------------|:-------------------------------------|
| `gets-async-minimal.cpp`      | Receives data from interrupt         |
| `gets-async.cpp`              | Receives data                        |
| `getsnowait.cpp`              | Also receives data                   |
| `helloworld-minimalistic.cpp` | Prints ‘Hello World!’                |
| `preemption*`                 | Several examples about preemption    |
| `resumefromevent.cpp`         | Thread gets resumed after short time |
| `resumer.cpp`                 | Similar to above                     |

All other file names should be self explaining.
