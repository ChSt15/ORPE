
set -e

rodos-executable.sh  linux-x86  -I..  publisher.cpp gateway.cpp topics.cpp
mv tst tst-publisher
xterm -bg white  -fg black -title publisher -e tst-publisher &


rodos-executable.sh  linux-x86  -I..  subscriber.cpp gateway.cpp topics.cpp
mv tst tst-subscriber
xterm -bg white  -fg black -title subscriber -e tst-subscriber &


