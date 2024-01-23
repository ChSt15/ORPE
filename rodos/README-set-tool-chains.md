Setup Toolchains in Ubuntu
==========================

Only in case you do not have the gcc, gcc++, clang, cmake, and ARM crosscompiler tools, then follow these instructions.

First try the simple way following the instructions the shell will show you if the program is not found. If this do not help, then follow
these instructions here.



Install prerequisites 
---------------------

```
sudo apt update
sudo apt upgrade
```


Install toolchain for CLang
-----------------------

```
sudo apt install -y apt-utils
sudo apt install -y clang clang-format clang-tools gdb
sudo apt install -y gcc-multilib g++-multilib
sudo apt install -y gcc-arm-none-eabi binutils-arm-none-eabi libnewlib-arm-none-eabi
sudo apt install -y cmake

```
If this fails, try
https://askubuntu.com/questions/1243252/how-to-install-arm-none-eabi-gdb-on-ubuntu-20-04-lts-focal-fossa/1243405#1243405



To check your camake version run:

```
cmake --version
```

at the time of writing, version 3.13 was not yet part of the canonical
repository of earlier Ubuntu versions. Replace commands with latest version 

```
sudo apt remove cmake
wget https://github.com/Kitware/CMake/releases/download/v3.13.2/cmake-3.13.2-Linux-x86_64.sh
chmod +x cmake-3.13.2-Linux-x86_64.sh
sudo ./cmake-3.13.2-Linux-x86_64.sh
sudo ln -s /opt/cmake-3.13.2-Linux-x86_64/bin/* /usr/local/bin
```


For Smartfusion 2 (SF2)
======================

Setting up FlashPro 5 on newer Linux versions (to upload boot images to SF2)
---------------------------------------------------------------------------

The reason why the FlashPro 5 doesn't work out of the box with newer
Linux is that it advertises a USB device type that is registered
and handled by the FTDI serial I/O driver called ftdi_sio.
In order to make it work, we need to hook into the udev system
to ask it to unbind the ftdi_sio driver if the device type is "FlashPro5". 

```
/etc/udev/rules.d/70-microsemi.rules

# FlashPro5
SUBSYSTEM=="usb", ATTR{idVendor}=="1514", ATTR{idProduct}=="2008", MODE="0776", GROUP="plugdev", RUN+="unbind_ftdi.sh"
SUBSYSTEM=="usb", ATTR{idVendor}=="0403", ATTR{idProduct}=="6001", MODE="0776", GROUP="plugdev", RUN+="unbind_ftdi.sh"
/lib/udev/unbind_ftdi.sh
  GNU nano 2.9.8                                                                                                               /lib/udev/unbind_ftdi.sh                                                                                                                Modified  
 
#!/usr/bin/env bash
 
# Find FTDI child USB device with interface FlashPro5
for DEVICE in /sys/bus/usb/drivers/ftdi_sio/*/
do
 
if grep -q FlashPro5 "${DEVICE}interface"; then
# Match found, unbind it from the FTDI driver
echo -n "$(basename $DEVICE)" > /sys/bus/usb/drivers/ftdi_sio/unbind
break
fi
done
```

Now set execution permissions for both 

```
/etc/udev/rules.d/70-microsemi.rules 
/lib/udev/unbind_ftdi.sh 
```

And set owner to root:root.

To be able to access the device,  add yourself to the plugdev user group (after adding yourself to the group, you need to restart your computer or run "su - $USER" to propagate the group change) :


sudo usermod -aG plugdev $USER


Reload the udev rules:

sudo udevadm control --reload-rules

Plug in the FlashPro 5 and dmesg should look like:

```
[135290.142184] usb 1-6.2: new high-speed USB device number 59 using xhci_hcd
[135290.250893] usb 1-6.2: New USB device found, idVendor=1514, idProduct=2008, bcdDevice= 8.00
[135290.250895] usb 1-6.2: New USB device strings: Mfr=1, Product=2, SerialNumber=3
[135290.250897] usb 1-6.2: Product: FlashPro5
[135290.250898] usb 1-6.2: Manufacturer: Microsemi
[135290.250899] usb 1-6.2: SerialNumber: 01QVQXR
[135290.257579] ftdi_sio 1-6.2:1.2: FTDI USB Serial Device converter detected
[135290.257606] usb 1-6.2: Detected FT4232H
[135290.257774] usb 1-6.2: FTDI USB Serial Device converter now attached to ttyUSB1
[135290.283949] ftdi_sio ttyUSB1: FTDI USB Serial Device converter now disconnected from ttyUSB1
[135290.283969] ftdi_sio 1-6.2:1.2: device disconnected
```

The important line is the last one which says "device disconnected",
which means the ftdi_sio driver has given up controlling the device.
OpenOCD should be working now

