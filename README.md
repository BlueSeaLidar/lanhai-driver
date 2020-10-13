# usb-lidar
linux demo programs for lanhai 2d lidar

# compile
run ./build.sh to compile the demo programs

# if your lidar model is LDS-25C or LDS-50C:
./lidar /dev/ttyUSB0 230400 0 1

# if your lidar model is LDS-50C-2 :
./lidar /dev/ttyUSB0 500000 1 1
