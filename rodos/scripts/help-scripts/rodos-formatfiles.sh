#!/usr/bin/env bash

if [ $# -lt 1 ]
then
  echo "Usage: `basename $0` <lsit-of-sources-to-reformat>"
  exit 1
fi

#astyle -O  -o  $*
clang-format   $* -i
\rm -f *.orig
