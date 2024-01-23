
set -e

rodos-executable.sh  linux-x86  -I..  client.cpp gateway.cpp topics.cpp
mv tst tst-client
xterm -bg white  -fg black -title client -e tst-client &


rodos-executable.sh  linux-x86  -I..  server.cpp gateway.cpp topics.cpp
mv tst tst-server
xterm -bg white  -fg black -title server -e tst-server &


