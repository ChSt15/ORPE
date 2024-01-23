# RODOS

# How to get started with RODOS in 9 easy steps

### Author: S. Montenegro

## 1. Before you start

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
   cd tutorials/10-first-steps
   ```
6. You may use different ports and target architectures. In these examples we will use
   linux-make-context (`linux-makecontext`). To see the list of alternatives, see in the build-scripts directory the
   files in set-vars.
7. Compile the user program

   ```
   rodos-executable.sh linux-makecontext usercode1.cpp usercode2.cpp ...
   ```

   As a **shortcut**, a bash-script has been created for every example that compiles the necessary code-files and
   executes it (e.g.
   `execute-example-01.sh` for the example in chapter 1). Attention:
   Don't forget to do step 1 to 5 beforehand. At the end of the paper, there is a short summary about all programmes.
8. Execute the binary

   ```
   ./tst
   ```
9. Exit the program with Ctrl+C

## 2. Hello World

The Hello World tutorial is the simplest RODOS program. It only prints the string “Hello World!” in one thread.

### 2.1. Used RODOS functions

`PRINTF()`

> basically the same as the standard C printf() function – prints
> characters and numbers to terminal

### 2.2. Program helloworld.cpp

Compile `helloworld.cpp` in [tutorials/10-first-steps](../10-first-steps) (current directory) as described in
[section 1.2.](#12-steps-to-compile-and-execute-a-rodos-program).

### 2.3. Console output

After some RODOS Information:

```
--------------- application running ------------
Hello World! 
```

- printout of run() method

### 2.4. Several Threads

Try now `helloworld-multiple.cpp`. Do you see the difference?

### 2.5. Annotation

For several programmes exist shell scripts that compile and execute their program. Just have a look into the scripts.

## 3. Basic Structure

The Basic structure tutorial is an extension to the Hello World tutorial. It prints the string “*Hello World!*” and
implements the basic structure of a RODOS program consistent of one application and one thread.

### 3.1. Used RODOS functions

`PRINTF()`

- basically the same as the standard C printf() function – prints characters and numbers to terminal

### 3.2. Program basic.cpp

Compile `basic.cpp`  in [tutorials/10-first-steps](../10-first-steps) as described in
[section 1.2.](#12-steps-to-compile-and-execute-a-rodos-program).  
Modify the `run()` method in `basic.cpp` and repeat step 6 to 8 of
[section 1.2.](#12-steps-to-compile-and-execute-a-rodos-program) to see the difference.

### 3.3. Console Output

- RODOS version

```
RODOS RODOS-100.0 OS Version RODOS-linux-8
```

- all applications in this program

```
Loaded Applications:
         10 -> 'Topics & Middleware'
       1000 -> 'HelloWorld'
```

```
Calling Initiators
Distribute Subscribers to Topics
List of Middleware Topics:
CharInput Id = 28449 len = 12.
 -- Subscribers:
SigTermInterrupt Id = 16716 len = 4.
 -- Subscribers:
UartInterrupt Id = 15678 len = 4.
 -- Subscribers:
TimerInterrupt Id = 25697 len = 4.
 -- Subscribers:
gatewayTopic Id = 0 len = 12.
 -- Subscribers:
```

- shows defined threads and printout of init method

```
Event servers:
Threads in System:
    Prio =      0 Stack = 32000 IdleThread: yields all the time
    Prio =    100 Stack = 32000 HelloWorld: Printing Hello World
```

```
BigEndianity = 0, cpu-Arc = x86, Basis-Os = baremetal, Cpu-Speed
(K-Loops/sec) = 350000
-----------------------------------------------------
Default internal MAIN
--------------- application running ------------
```

- printout of run() method

```
Hello World!
```

## 4. Time

This tutorial shows how time dependent processes can be modelled in RODOS. It demonstrates how to do something at a
specific **point in time**, after a defined **amount of time** and **periodically**. While the thread waits for the
defined time, other threads can be executed.
**Time in RODOS** is defined with a long long type “TTime” and represents the number of **nanoseconds elapsed since
start-up**.

### 4.1. RODOS time functions

|                                             |                                                                                                        |
|:--------------------------------------------|:-------------------------------------------------------------------------------------------------------|
| `NOW()`                                     | Returns the current time (in nanoseconds)                                                              |
| `SECONDS_NOW()`                             | Returns the current time in seconds                                                                    |
| `AT(time)`                                  | Suspends (interrupts) the thread that has called this method, until the given point in time is reached |
| `TIME_LOOP(firstExecution, Period) { ... }` | Almost each control loop has a start time and a period. This macro provides this loop with no end.     |

### 4.2. Time macros

In order to use the time functions comfortably there are some time macros defined:  
`NANOSECONDS` , `MICROSECONDS` , `MILLISECONDS` , `SECONDS` , `MINUTES`,
`HOURS` , `DAYS` , `WEEKS` , `END_OF_TIME`  
To use them, just multiply them to the amount of time, e.g.
`AT(3*SECONDS)`.  
`END_OF_TIME` is the highest time possible (about 293 years).

### 4.3. Program time.cpp

Compile `time.cpp` in tutorials/10-first-steps as described in section
[section 1.2.](#12-steps-to-compile-and-execute-a-rodos-program) and execute it. The output should be the following:

```
waiting until 3rd second after start
after 3rd second
waiting until 1 second has pased
1 second has pased
print every 2 seconds, start at 5 seconds
current time:
 5.000003995
current time:
 7.000004191 ...
```

## 5. Thread Communication

The communication between two threads can bee realized via a CommBuffer or a FiFo (First in first out). For that a
CommBuffer or a FiFo has to be defined outside a thread so that both threads can access it.

## 5.1. CommBuffer

A CommBuffer is a double buffer with **only one writer** and **only one reader**. Both can work concurrently. The writer
may write at any time. The reader gets the newest consistent data (e.g. the last complete written record). The type of
the CommBuffer can be defined. Not using a CommBuffer is risky, because maybe the data is half written in the shared
variable while the thread is interrupted. In this case the receiver thread gets inconsistent data.

Compile `commbuffer.cpp` in [tutorials/10-first-steps](../10-first-steps) as described
in [section 1.2.](#12-steps-to-compile-and-execute-a-rodos-program) and execute it.

## 5.2. FiFo

A FiFo is used for synchronous communication from one single writer to one single reader. Writing to a full FiFo has no
effect and returns 0. Reading from an empty FiFo returns 0. The first value inserted into the FiFo will be the first
value to be read.

Compile `fifo.cpp` in [tutorials/10-first-steps](../10-first-steps) as described
in [section 1.2.](#12-steps-to-compile-and-execute-a-rodos-program) and execute it.

## 5.3. Synchronous FiFo

A SyncFiFo is basically the same as FiFo, but in this case the sender will be suspended if the FiFo is full and the
receiver will be **suspended until data is ready**.

Compile `fifo-sync.cpp` in [tutorials/10-first-steps](../10-first-steps) as described
in [section 1.2.](#12-steps-to-compile-and-execute-a-rodos-program) and execute it.

## 5.4. Which is best for what?

If the receiver needs only the latest data, a CommBuffer should be used. If the receiver needs all the data from the
sender and in the right order, a FiFo is the way to do it. A SyncFiFo is a good option if the data has to be processed
short times after sending it, but take notice that the thread cannot do anything until new data is available.

# 6. Critical sections

To avoid concurrent access of critical sections semaphores have to be used. To enter a semaphore use sema.enter() and to
leave use sema.leave().

## 6.1. RODOS functions

|                                   |                                                                                                                                                                                                             |
|:----------------------------------|:------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `Semaphore::enter()`              | Makes a thread enter a semaphore. All other threads trying to enter the same semaphore will wait until it has been left again.                                                                              |
| `Semaphore::leave()`              | Leaves the semaphore and allows other threads entering it.                                                                                                                                                  |
| `{ PROTECT_IN_SCOPE(sema); ... }` | A macro entering the semaphore "sema" when entering in the current scope (between surrounded { and } ). and leaving it when leaving the scope, It is only a shortcut, which may be useful ... or maybe not. |
| `yield()`                         | Interrupts the current thread and calls the scheduler that looks for a thread to execute. If no other thread wants to be executed, the thread continues.                                                    |

## 6.2. Program semaphore.cpp

Compile `semaphore.cpp` in [tutorials/10-first-steps](../10-first-steps) as described
in [section 1.2.](#12-steps-to-compile-and-execute-a-rodos-program) and execute it. The output should be the following:

```
only one, I am -- A -- , time    3.000056382
only one, I am -- B -- , time    3.000077366
only one, I am -- C -- , time    3.000094338
only one, I am -- D -- , time    3.000111110
only one, I am -- E -- , time    3.000128005
only one, I am -- F -- , time    3.000157338
only one, I am -- G -- , time    3.000180353
...
```

Remove the protection in semaphore.cpp, compile again and see the difference.

## 6.3. Program semaphore-macro.cpp

Compile `semaphore-macro.cpp` in [tutorials/10-first-steps](../10-first-steps) as described
in [section 1.2.](#12-steps-to-compile-and-execute-a-rodos-program) and execute it.

## 6.4. Attention: A deadlock may occur!

Have a look at the tutorial `semaphore-deadlock.cpp`. The programme will stop when a deadlock has occurred.

Compile `semaphore-deadlock.cpp` in [tutorials/10-first-steps](../10-first-steps) as described
in [section 1.2.](#12-steps-to-compile-and-execute-a-rodos-program) and execute it.

## 7. Events

Events can be used to react to interrupts from timers and signals from devices. Do not use them for complex actions,
because they cannot be interrupted. Just use them to trigger threads that handle the interrupts. Implement them as short
as possible.

An event has basically two methods: The init() method similar to threads and the handle() method in which the code is
defined that handles the event.

### 7.1. RODOS functions

|                                       |                                                                          |
|:--------------------------------------|:-------------------------------------------------------------------------|
| `activatePeriodic(startTime, period)` | Activates an event periodically after the first activation at startTime. |
| `activateAt(time)`                    | Activates an event at the given point in time.                           |
| `thread.resume()`                     | Resumes a thread that is suspended.                                      |

### 7.1. Program event.cpp

Compile `event.cpp` in [tutorials/10-first-steps](../10-first-steps) as described
in [section 1.2.](#12-steps-to-compile-and-execute-a-rodos-program) and execute it. The output should be the following:

```
Suspend and wait until some one resumes me
Time Event at
 5.000107974
Testwaiter resumed from me
testwaiter running again at
 5.000135475
Suspend and wait until some one resumes me
Time Event at
 8.000168306
Testwaiter resumed from me
...
```

Try the example with `activateAt(time)` instead of `activatePeriodic(startTime, period)`.

## 8. Middleware

Up to here, we had "normal" programming. Now assume we are in a big team with a big project. You do not know the details
of what others are programming, just the format of the data you need from them or you produce for them. Now you have to
get and distribute this data without notion of the other side of this generic interface which we call the middleware.

The middleware is used to communicate between tasks and even between tasks of different RODOS nodes. This communication
is based on a `publisher/subscriber protocol` and there is no connection from a sender to a receiver.

Any thread can publish messages under a given topic, while subscribers of the same topic receive the published data.

There can be 0, 1 or many publishers for one topic. The same goes for subscribers.

### 8.1. Required files

For this example you will need flowing files:

|                        |                                                          |
|:-----------------------|:---------------------------------------------------------|
| `sender.cpp`           | The one who sends test message: a publisher              |
| `topics.h, topics.cpp` | communication channels to send and receive data          |
| `receiver-*`           | different methods to subscribe and get data: Subscribers |

You will need to compile several source files together. Every compilation needs to include the file `topics.cpp`

For example:

```
rodos-executable.sh linux-makecontext topics.cpp sender.cpp
receiver-commbuff.cpp
```

or another receiver:

```
rodos-executable.sh linux-makecontext topics.cpp sender.cpp
receiver-putter.cpp
```

or all receivers together:

```
rodos-executable.sh linux-makecontext topics.cpp sender.cpp
receiver-*.cpp
```

### 8.2. Topic, program topics.cpp

A topic is a pair of a data-type (`long`) and the topic id (`10`) and labeled by a name (`"counter1"`), e.g.:

```
Topic<long> counter1(10, "counter1");
```

If the topic id is `-1` the id will be generated.

Some sample topics are defined in [topics.cpp](topics.cpp). To use these topics in a RODOS program include `topics.h`.

### 8.3. Publisher, program sender.cpp

A publisher is easy to implement. To publish data under the topic `“counter”` just use
`counter.publish(data)` in any thread, as shown in [sender.cpp](sender.cpp).

### 8.4. Subscriber

There are many possibilities to implement a receiver of middleware data.

#### 8.4.1 Subscriber put() method, program receiver-simple.cpp

[receiver-simple.cpp](receiver-simple.cpp) defines a new subscriber by inheriting from `Subscriber`:

Compile `receiver-simple.cpp` in [tutorials/10-first-steps](../10-first-steps) as described
in [section 1.2.](#12-steps-to-compile-and-execute-a-rodos-program) and execute it. Do not forget to compile it
with `topics.cpp` and `sender.cpp`:

```
rodos-executable.sh linux-makecontext topics.cpp sender.cpp receiver-simple.cpp
```

#### 8.4.2 Subscriber and a CommBuffer, program receiver-commbuf.cpp

[receiver-commbuf.cpp](receiver-commbuf.cpp) defines a CommBuffer that is going to be filled by a Subscriber. The thread
gets periodically the latest data from the CommBuffer.

Compile `receiver-commbuff.cpp` in [tutorials/10-first-steps](../10-first-steps) as described
in [section 1.2.](#12-steps-to-compile-and-execute-a-rodos-program) and execute it. Do not forget to compile it
with `topics.cpp` and `sender.cpp`:

```
rodos-executable.sh linux-makecontext topics.cpp sender.cpp receiver-commbuff.cpp
```

To get synchronised data transfer use a SyncFiFo like in tutorial receiver-sync.cpp

#### 8.4.3 Putter, program receiver-putter.cpp

[receiver-putter.cpp](receiver-putter.cpp) defines a new putter by inheriting from `Putter`.

Compile `receiver-putter.cpp` in [tutorials/10-first-steps](../10-first-steps) as described
in [section 1.2.](#12-steps-to-compile-and-execute-a-rodos-program) and execute it. Do not forget to compile it
with `topics.cpp` and `sender.cpp`:

```
rodos-executable.sh linux-makecontext topics.cpp sender.cpp receiver-putter.cpp
```

To receive two counters, implement a sender of the second counter.

#### 8.4.4 Which subscriber is best for what?

If the receiver needs only the latest data and has to be executed periodically, the CommBuffer solution should be used.
For synchronized communication the subscriber put method in combination with resuming a thread is the way to do it. A
SyncFiFo is also good for this. To receive from multiple topics with one method a Putter should be used.

For more information and tutorials about the middleware check out the folders
[tutorials/30-communication/10-alice_bob_charly](../30-communication-and-bbs/10-alice-bob-charly) and
[tutorials/30-communication-and-bbs20-/middleware](../30-communication-and-bbs/20-middleware)

## 9. More Middleware

To see a little more about using the middleware and multicasting, please have a look at the example in the directory
[tutorials/10-first-steps/middleware](middleware). Here, we have an example of topics with more than one subscriber and
of subscribers of more than one topic. A position sensor measures and publishes data of the position (3D) of a flying
object. A speed calculator receives those data and calculates and publishes the object's speed. Finally, a display
subscribing both topics, position and speed, and prints the data.

|                      |                                                         |
|:---------------------|:--------------------------------------------------------|
| `executeit.sh`       | shell script to compile and execute the whole example   |
| `topics.h`           | interface of the topics                                 |
| `topics.cpp`         | definition of the topics position and speed             |
| `positionsensor.cpp` | generates and publishes random position data            |
| `speedcalc.cpp`      | subscribes position topic and publishes speed data      |
| `display.cpp`        | subscribes position and speed topic and prints the data |

Compile all `*cpp` and see the execution.

Then try to compile without `speedcalc.cpp`. Do you see the difference?

## 10. Other files

|                        |                                                    |
|:-----------------------|:---------------------------------------------------|
| `mem-test.cpp`         | Prints memory location of some variables           |
| `priority.cpp`         | 2 threads print at the same time                   |
| `priority-ceiling.cpp` | 2 threads print at the same time (a bit different) |
