#!/bin/bash
if [ $# -ne 1 ]
then
  echo "Usage: `basename $0` source"
  exit 1
fi

\rm -f a.out
g++ -I ../../api/ -I ../../src/linux/ \
    ../../src/linux/hw_udp.cpp  ../../src/linux/hal/*.cpp ../../src/independent/*.cpp \
    $1

a.out

