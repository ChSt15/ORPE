
\rm -f tst
rodos-executable.sh discovery  send_receive_discovery_uart.cpp
mv tst tst-on-discovery.elf
rodos-executable.sh linux-x86  send_receive_UDP.cpp
tst

