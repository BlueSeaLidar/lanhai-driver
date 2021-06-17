# usb-lidar
linux demo programs for lanhai 2d lidar

# compile
souce ./build.sh #compile the demo programs

# plug lidar usb port, make sure /dev/ttyUSBx existed, add read / write attribution
sudo chmod 666 /dev/ttyUSB0

# for LDS-40D-B20R 
./bin/uart-demo /dev/ttyUSB0 1000000 1 1 200 1 1

# if your lidar model is LDS-15D-B25R or LDS-25D-B25R:
./bin/uart-demo /dev/ttyUSB0 768000 1 1 200 1 1

# for LDS-50C-B40R 
./bin/uart-demo /dev/ttyUSB0 1000000 1 1 400 1 1

# if your lidar model is LDS-15BDM or LDS-25BDM:
./bin/uart-demo /dev/ttyUSB0 230400 0 1 0 0 0

# if your lidar model is LDS-50C-2 :
./bin/uart-demo /dev/ttyUSB0 500000 1 1 0 1 1

# if your lidar model is LDS-U50C-S :
./bin/udp-demo 192.168.158.91 5000 5100 1 1 1 1

# if your lidar model is LSS-40S :
./bin/udp-demo 192.168.158.98 6543 6668 1 1 1 1 200
