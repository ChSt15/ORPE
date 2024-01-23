
rodos-executable.sh  on-posix print-results.cpp  top1.cpp gateway.cpp
mv tst tst1

rodos-executable.sh  on-posix print-results.cpp  top2.cpp gateway.cpp
mv tst tst2

rodos-executable.sh  on-posix print-results.cpp  top3.cpp gateway.cpp
mv tst tst3

rodos-executable.sh  on-posix print-results.cpp  top4.cpp gateway.cpp
mv tst tst4

rodos-executable.sh  on-posix print-results.cpp  top5.cpp gateway.cpp
mv tst tst5


cat  << EOT

Start some tst* in different windowns.
See the bitmap of subscriber
Kill some and see how the bitmap changes in 10 seconds
Start again and see.
Rec -1 means more than one reciever
-2 no reciever
else receiver index

EOT





