flexforce_adapter
=================

ROS node for utilizing InterfaceKit with FlexiForce Analog sensors via usb on beaglebone black.

Phidgets libraries and udev rules must be installed first.

Phidgets libraries HOWTO:
-sudo apt-get install libusb-1.0-0-dev
-wget http://www.phidgets.com/downloads/libraries/libphidget.tar.gz
-tar -zxvf libphidget.tar.gz
-cd libphidget
-./configure
-make
-sudo make install

Phidgets udev HOWTO:
-sudo nano /etc/udev/rules.d/80_phidget.rules
-add the content:
	SUBSYSTEMS=="usb", ACTION=="add", ATTRS{idVendor}=="06c2", ATTRS{idProduct}=="00[3-a][0-f]", 		MODE="666"
-export USB_DEVFS_PATH=/dev/bus/usb
-service udev restart

-sudo apt-get install libusb-dev
