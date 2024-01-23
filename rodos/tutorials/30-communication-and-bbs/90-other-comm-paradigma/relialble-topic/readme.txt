
Reliable transmission on the top of topics, using acknowledgments
    Quality of Service: "Exact once"



author Sergio Montenegro
date 2022/05/21 




RelialbeTopic is a superclass of topic. It is a topic.
We use the same topic to send the message and to receiver the ack.

The senders publishes data and waits until it gets the acknowledgment from the receiver.
After a time out of 100 ms it will retry the send the same message.
until a maximal time out, then  it gives up and returns a failure.

I use as ack the msgInfo (contines time and senderId) of the original message.

It is intended to be point to point: pro reliable topic only one
sender and only one receiver. If you have more than one receiver the
sender will be happy with the first acknowledgment for each message.
We know only that at least one receiver got the data.

To test, compile sender and receiver and start them in different windows
then abort the receiver and watch the sender. Then restart the receiver.


