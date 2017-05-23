# pi_roombot(Under Developing)
------
This is a ROS based SLAM robot project comes out with a very low price, and it works fine with the Raspberry Pi.

## Components
1. Raspberry Pi or other ARM or x86 based computer.
2. Arduino UNO R3(recommended, used for wheel and IMU controll, you can do it the harder way which recoding to suit just using the Raspberry Pi)
3. Motor Driver: PololuMC33926 or DFRobotL298PShield(other L298N component should works too) 
4. IMU: Sparkfun SEN-10724/Generic GY-85/Generic GY-80
5. Lidar: XV-11/rplidar etc.
6. Kinect for xbox 360
7. Robot Chassis with Power Supply and two wheels (both 12V and 5V needed)

## Software Requirement
1. Ubuntu Mate 16.04.2 LTS(Raspbian shold work too, if someone test under this env plz let me know)
2. ROS Indigo(For PI, this may need to compile from the source code)
3. ROS Packages: rosserial, ros_arduino, freenect_stack(or openni_camera, openni_launch), rtabmap, rtabmap_ros

## Installation (incomplete)
1. just solve the deps
2. catkin_make
3. ready to go

## Trouble Shooting
1. rosserial lose sync: #define USE_USBCON before include ros.h 
2. rosserial still got problem with sync: try use hydro-devel instead of indigo-devel
