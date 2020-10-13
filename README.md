# usb-lidar
linux demo programs for lanhai 2d lidar

# compile
souce ./build.sh #compile the demo programs

# plug lidar usb port, make sure /dev/ttyUSBx existed, add read / write attribution
sudo chmod 666 /dev/ttyUSB0

# if your lidar model is LDS-25C or LDS-50C:
./lidar /dev/ttyUSB0 230400 0 1

# if your lidar model is LDS-50C-2 :
./lidar /dev/ttyUSB0 500000 1 1
