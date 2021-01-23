#!/bin/bash

echo ""
echo "This script copies OMO R1mini udev rules to /etc/udev/rules.d/"
echo ""

echo "Motor Driver (USB Serial from RS232) : /dev/ttyUSBx to /dev/ttyMotor :"
if [ -f "/etc/udev/rules.d/98-omo-r1mini.rules" ]; then
    echo "98-omo-r1mini.rules file already exist."
else 
    echo 'KERNEL=="ttyTHS1", MODE:="0666", GROUP:="dialout"' > /etc/udev/rules.d/98-omo-r1mini.rules
    
    echo '98-omo-r1mini.rules created'
fi

echo ""
echo "YD LiDAR (USB Serial) : /dev/ttyUSBx to /dev/ttyLiDAR :"
if [ -f "/etc/udev/rules.d/ydlidar.rules" ]; then
    echo "ydlidar.rules file already exist."
else 
    echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout",  SYMLINK+="ttyLiDAR"' >/etc/udev/rules.d/97-ydlidar.rules
    
    echo 'ydlidar.rules created'
fi

echo ""
echo "USB0 setting"
if [ -f "/etc/udev/rules.d/USB0_setting.rules" ]; then
    echo "USB0_setting.rules file already exist."
else 
    echo 'KERNEL=="ttyUSB0", MODE:="0666", GROUP:="dialout"' >/etc/udev/rules.d/96-USB0_setting.rules
    
    echo 'USB0_setting.rules created'
fi


# if [ -f "/etc/udev/rules.d/ydlidar-V2.rules" ]; then
#     echo "ydlidar-V2.rules file already exist."
# else 
#     echo 'KERNEL=="ttyACM*", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE:="0666", GROUP:="dialout",  SYMLINK+="ttyLiDAR"' >/etc/udev/rules.d/ydlidar-V2.rules
    
#     echo 'ydlidar-V2.rules created'
# fi

# if [ -f "/etc/udev/rules.d/ydlidar-2303.rules" ]; then
#     echo "ydlidar-2303.rules file already exist."
# else 
#     echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="2303", MODE:="0666", GROUP:="dialout",  SYMLINK+="ttyLiDAR"' >/etc/udev/rules.d/ydlidar-2303.rules
    
#     echo 'ydlidar-2303.rules created'
# fi

echo ""
echo "Reload rules"
echo ""
sudo udevadm control --reload-rules
sudo udevadm trigger
