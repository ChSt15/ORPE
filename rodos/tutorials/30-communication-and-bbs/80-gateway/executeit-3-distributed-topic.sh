#!/bin/bash

GATEWAY=gateway-distributed-topic.cpp

RUN_XTERM=true
if ! command -v xterm &> /dev/null ; then
    # xterm not found => user has to run it manually
    RUN_XTERM=false
fi

set -e

rodos-executable.sh linux-x86 ${GATEWAY} demo_topics.cpp sender.cpp
mv tst tst-sender
if ${RUN_XTERM} ; then
    xterm -bg white -fg black -title senders -e tst-sender &
fi

rodos-executable.sh linux-x86 ${GATEWAY} demo_topics.cpp receiver1.cpp
mv tst tst-rec1
if ${RUN_XTERM} ; then
    xterm -geometry 130x23 -bg gray -fg black -title receiver1 -e tst-rec1 &
fi

rodos-executable.sh linux-x86 ${GATEWAY} demo_topics.cpp receiver2.cpp
mv tst tst-rec2
if ${RUN_XTERM} ; then
    xterm -bg gray -fg black -title receiver2 -e tst-rec2 &
fi

if ${RUN_XTERM} ; then
    echo "CR to close all"
    read JA
    kill $(jobs -p)
fi
