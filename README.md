# kit_agv - Test Navigation with a Microscan3

- Connect Microscan3:
+ Source: https://github.com/SICKAG/sick_safetyscanners
+ Connect Microscan3 to PC via ethernet
+ Check IP of Microscan3 (ex: 192.168.1.3)
+ Change IP of PC fix 192.168.1.x
+ Test connect: $ping 192.168.1.3
+ Test for microscan3: $roslaunch sick_safetyscanners sick_safetyscanners.launch sensor_ip:=192.168.1.3 host_ip:=192.168.1.5 frame_id:=base_laser
+ Open Rviz: $rosrun rviz rviz
+ Change in Rviz: fix_frame to base_laser, add Laser_scan
+ Enjoy!

- Run WebServer:
+ $roslanch webServer kit_agv_webServer.launch
+ Open browser: localhost:5000/navigation
+ Connect to IP of roscore

- Enable: USB to RS485:
+ sudo chmod 777 /dev/ttyUSB0

- Navigation:
+ $roslaunch kit_agv_navigation kit_agv_navigation.launch