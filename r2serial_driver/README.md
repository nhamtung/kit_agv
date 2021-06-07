# r2serial_driver
# package giao tiếp với driver motor (USB to RS485)


- Connect RS485
- Check serial port: $ls /dev/ttyUSB*
- Allows access: $sudo chmod 777 /dev/ttyUSBx
- Test: $roslaunch r2serial_driver r2serial_driver.launch port:=/dev/ttyUSBx

