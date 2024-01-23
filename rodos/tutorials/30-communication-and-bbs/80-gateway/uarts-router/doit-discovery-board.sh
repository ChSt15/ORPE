
rodos-executable.sh discovery src/uart_router.cpp src/router.cpp
arm-none-eabi-objcopy -O binary tst router.bin

rodos-executable.sh discovery src/demo_topics.cpp src/gateway1.cpp src/sender.cpp
arm-none-eabi-objcopy -O binary tst sender.bin

rodos-executable.sh discovery src/demo_topics.cpp src/gateway1.cpp src/receiver1.cpp
arm-none-eabi-objcopy -O binary tst receiver1.bin

rodos-executable.sh discovery src/demo_topics.cpp src/gateway1.cpp src/receiver2.cpp
arm-none-eabi-objcopy -O binary tst receiver2.bin


echo "______________________"
echo "to program each board, please use:"
echo "st-flash write XXXXX.bin 0x08000000"
