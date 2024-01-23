# RODOS

# Tutorial Middleware

### Author: S. Montenegro

## 1. How To begin (Steps to compile and execute a RODOS program)

Read and understand first the RODOS-Tutorial and then the RODOS-Middleware tutorial.

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
   cd tutorials/30-communication-and-bbs/20-middleware
   ```

6. You may use different ports and target architectures. In these examples we will use
   linux-make-context (`linux-makecontext`). To see the list of alternatives, see in the build-scripts directory the
   files in set-vars.
7. Compile the user program

   ```
   rodos-executable.sh linux-makecontext <list of sources>
   ```

8. Execute the binary

   ```
   ./tst
   ```

For each example programme please first read and understand the code, then compile and execute and see if it acts as
expected. Then modify and continue trying. Try to combine different programs together.

## 2. Useful to know

For output in masks we use the [support/support_programs/maskgenerator](../../../support/support-programs/maskgenerator)
. Please first take a look to mask.txt, you will find the same macro names `[NAME ]` in all programs.

The middleware communication is based on a publisher/subscriber protocol. This is a multicast protocol. There is no
connection from a sender to a receiver (connectionless). Publishers make messages public under a given topic.
Subscribers to a given topic get all messages which are published under the given topic. To establish a transfer path,
both the publisher and subscriber have to share the same topic. A Topic is a pair - data-type and an integer
representing a topic identifier. To match a communication pair, both data-type and topic identifier must match. For a
topic there may be zero, one or several subscribers. A message which is published under a topic will be distributed to 0
or more subscribers. Each subscriber has a reference to the associated topic and a “putter” to store messages. All
subscribers are registered in a list. Each time a message is published the list of all subscribers will be searched and
for each subscriber where the topic matches the associated putter will be called to store a copy of the published
message. Using a network interface and the corresponding gateways, the middleware may make the node borders transparent.
Applications running on different computers may communicate as if they were on the same computer.

## 3. Stabilising common topics

**demo_topics.h** declares all topics and data structures used for communication for the tutorial. These topics and data
structures shall be known by all communicating applications. Therefore it has to be declared globally and all
applications must include this file.

**demo_topics.cpp** allocates all topics. For each topic declaration the corresponding (exactly one) object has to be
created. There may not be replications of topic objects.

Try to replicate topic objects or topic identifiers. You shall get an error message.

Compile and execute `demo_topics.cpp`.

This programme does nothing, but you can get error messages if you replicate topics.

### Publisher / Sender

There are some examples of senders:

**sender01.cpp** is a simple publisher with a thread to send simple messages.

**sender02.cpp-antiexample.cpp** is a publisher which sends simple messages from an event handler. No thread is needed.

**senderPosition.cpp** sends more complex messages using an extra data structure. You may compile one or more of them
together. Each compilation has to have the `demo_topics.cpp`. You will see the reports of each sender each time it
publishes in a topic. There is no subscriber, therefore no message will be received.

### Subscribers/Receivers

There are several examples of subscribers. You may compile one or more together, but as for the publisher, each
compilation has to have the `demo_topics.cpp`.

**receiverthread.cpp** is the most simple receiver, a thread waits on a topic until something arrives. But his method
can lose messages if the thread is not waiting when the message arrives.

**receiverputter.cpp** is subscriber which creates an own putter to handle messages to topics `counter1` and `counter2`.
You man try to add or remove topics. No Thread is required to access the messages.

**receivercommbuf.cpp** has a CommBuffer as a putter, to store the last published message. It provides a thread which
look periodic to the buffer.

**receiverasync.cpp** has a asynchronous FiFo to store the messages. It provides a Thread which look periodic to the
FiFo.

**receiversync.cpp** has a synchronous FiFo to store the messages. It provides a thread which is activated each time
someone (a publisher) has written data to the FiFo (as a putter).

Try to compile one or more of them together. There is no publisher, therefore no subscriber will get data.

### Publishing and receiving

Now we may combine publishers and receivers. See which publisher publishes which topic and see which subscriber receives
which topic. Try different combinations or all senders and all receivers.

For example:

```
rodos-executable.sh linux-makecontext demo_topics.cpp receiverputter.cpp sender01.cpp
tst
```

or

```
rodos-executable.sh linux-makecontext *.cpp
tst
```
