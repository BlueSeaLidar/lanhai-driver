# usb-lidar
linux demo programs for lanhai 2d lidar

# compile
souce ./build.sh #compile the demo programs

# plug lidar usb port, make sure /dev/ttyUSBx existed, add read / write attribution
sudo chmod 666 /dev/ttyUSB0


# for LDS-50C-2
./bin/uart-demo params/LDS-50C-2.txt


# for LDS-50C-2
./bin/uart-demo params/LDS-50C-2.txt


# for LDS-50C-C20E
./bin/udp-demo params/LDS-50C-C20E.txt
