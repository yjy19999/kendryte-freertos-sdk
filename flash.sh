#!/bin/bash
echo "avaiable port are:" 
ls /dev/ttyUSB* | grep tty

read -p "Enter port: " usb_port

kflash -p /dev/ttyUSB$usb_port -t ./build/hello_world.bin