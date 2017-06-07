# pi_roombot(Under Developing)
------
This is a ROS based SLAM robot project comes out with a very low price(less than $300), and it works fine with the Raspberry Pi.

## Components
1. Raspberry Pi or other ARM or x86 based computer.
2. Arduino Mega 2560(recommended, used for wheel and IMU controll, you can do it the harder way which recoding to suit just using the Raspberry Pi)
3. Motor Driver: PololuMC33926 or DFRobotL298PShield(other L298N component should works too, but need to transform the controll pins from 6 pin to 4 pin with a NOT gate) 
4. IMU: Sparkfun SEN-10724/Generic GY-85/Generic GY-80
5. Lidar: XV-11/rplidar etc.(Optional, could replace this with fake lidar from the depth camera with package "depthimage_to_laserscan, but it is not precisely and reduce performace")
6. Kinect for Xbox 360(Optional)
7. Speaker for audio support(Optional)
7. Robot Chassis with Power Supply and two wheels(both 12V and 5V needed)

## Software Requirement
1. Ubuntu Mate 16.04.2 LTS(Raspbian shold work too, if someone test under this env plz let me know)
2. ROS Indigo(For PI, this may need to compile from the source code)
3. ROS Packages: navigation, rosserial, ros_arduino, imu_tools, freenect_stack(or openni_camera, openni_launch), rtabmap, rtabmap_ros, audio_common(optional)

## Installation
1. Install ROS From Tutoral, For Pi: http://wiki.ros.org/indigo/Installation/UbuntuARM, if this not working try build from source following: http://wiki.ros.org/indigo/Installation/Source
2. Install ROS Packages listed from software requirement. (if your ROS is build from source, then try build these package from source code also, just clone them inside your workspace following the steps below, then it will build when call the catkin_make)
3. Create your own catkin workspace following: http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment
4. Clone the source code in side the src folder.
5. Just cd to your catkin workspace folder, run "catkin_make" or "catkin_make_isolated".
6. ready to go

## Trouble Shooting
1. rosserial lose sync: #define USE_USBCON before include ros.h 
2. rosserial only works at the first launch or first reset on arduino: maybe the sram not enough for the buffering, make sure you are using Mega version arduino instead of Uno or something else.
3. rosserial still got problem with sync: replace "ros::nodeHandle" inside the ino file with "ros::nodeHandle_<ArduinoHardware, 6, 6, 150, 150>" to lower the buffer size, or just try use the jade-devel branch instead of indigo-devel
4. hector_slam consumes too much CPU or got error like "SearchDir angle change too large": rebuild catkin work space using "catkin_make -DCMAKE_BUILD_TYPE=Release", the debug version will use more CPU more than release version, and if you are using a real-life slam, try not move it too fase.
5. imu_tools got error while compiling: "usr/include/boost/type_traits/detail/has_binary_operator.hp:50: Parse error at BOOST_JOIN", it seems the qt moc parse error because of the multiple reference of the headers, just add "#ifndef Q_MOC_RUN" and "#endif" amoung the #include <...> from the spesific source file
6. audio_common dependence the gstreamer-0.10, this may not easy to obtain on the Raspberry Pi. First, you can compile it from the script: https://gist.github.com/sphaero/02717b0b35501ad94863, but something may come up. like: v4l2_calls.c:58:26: "error: 'V4L2_CID_HCENTER_DEPRECATED' undeclared", to fix this you need to patch the source code with: "cd ~/src/gstreamer/gst-plugins-good && sed -i -e "/input:/d" sys/v4l2/gstv4l2bufferpool.c && sed -i -e "/case V4L2_CID_HCENTER/d" -e "/case V4L2_CID_VCENTER/d" sys/v4l2/v4l2_calls.c"; second, if the gst-python package compiled fail such as int-conversion error or what, try comment the python3 specifition in the script; third, there should be some stdafx.h related issues you should surf the Internet and fix them mansualy.

