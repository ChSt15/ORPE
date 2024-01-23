#!/usr/bin/env bash

echo "::: Running $1 against $2"
set -e
# Kill test executable (with SIGKILL) after 25 seconds.  Necessary
# because sometimes tests might deadlock.
if timeout -s 9 25 $1 > "$1.output"
then
    true
else
    echo ":: KILLED BY TIMEOUT" >> "$1.output"
fi

diff -ruh $1.output $2 > $1.diff || true
