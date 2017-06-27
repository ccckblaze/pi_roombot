# pi_roombot(Under Developing)  
------
This is a ROS based SLAM robot project comes out with a very low price(less than $300), and it works perfect along with the Raspberry Pi.  

## Components  
1. Raspberry Pi or other ARM or x86 based computer  
2. Arduino Mega 2560(Recommended, used for wheel and IMU controll, you can do it the harder way which recoding it to suit just using the Raspberry Pi's GPIO)  
3. Motor Driver: PololuMC33926, DFRobotL298PShield or L298N component  
4. Robot Chassis with Power Supply and two wheels(Both 12V and 5V needed)  
5. IMU: Sparkfun SEN-10724/Generic GY-85/Generic GY-80  
6. Lidar: XV-11/rplidar etc.(Optional, could replace this with fake lidar from the depth camera with package "depthimage_to_laserscan, but it is not precisely and reduce performace")  
7. Kinect for RGBD-SLAM(Optional)  
8. Microphone and Speaker for audio support(Optional, you can use "kinect-audio-setup" package to use Kinect as microphone)  

## Software Requirement
1. Ubuntu Mate 16.04.2 LTS(Raspbian shold work too, if someone test under this env plz let me know)  
2. ROS Indigo(For PI, this may need to compile from the source code)  
3. ROS Packages: navigation, rosserial, ros_arduino, imu_tools, freenect_stack(or openni_camera, openni_launch), rtabmap, rtabmap_ros, audio_common(optional), pocketsphinx(optional)

## Installation
1. Install ROS From Tutoral, For Pi: http://wiki.ros.org/indigo/Installation/UbuntuARM, if this not working try build from source following: http://wiki.ros.org/indigo/Installation/Source  
2. Install ROS Packages listed from software requirement. (if your ROS is build from source, then try build these package from source code also, just clone them inside your workspace following the steps below, then it will build when call the catkin_make)  
3. Create your own catkin workspace following: http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment  
4. Clone the source code in side the src folder.  
5. Just cd to your catkin workspace folder, run "catkin_make" or "catkin_make_isolated".  
6. ready to go  

## Trouble Shooting
1. rosserial lose sync: #define USE_USBCON before include ros.h  
2. rosserial only works at the first launch or first reset on arduino: maybe the sram not enough for the buffering, make sure you are using Mega version arduino instead of Uno or something else  
3. rosserial still got problem with sync: replace "ros::nodeHandle" inside the ino file with "ros::nodeHandle_<ArduinoHardware, 6, 6, 150, 150>" to lower the buffer size, or just try use the jade-devel branch instead of indigo-devel  
4. hector_slam consumes too much CPU or got error like "SearchDir angle change too large": rebuild catkin work space using "catkin_make -DCMAKE_BUILD_TYPE=Release", the debug version will use more CPU more than release version, and if you are using a real-life slam, try not move it too fast  
5. imu_tools got error while compiling: "usr/include/boost/type_traits/detail/has_binary_operator.hp:50: Parse error at BOOST_JOIN", it seems the qt moc parse error because of the multiple reference of the headers, just add "#ifndef Q_MOC_RUN" and "#endif" amoung the #include <...> from the spesific source file  
6. audio_common not working or get message "jack server is not running or cannot be started": run the command "jack_control start" in shell, if you still got "Cannot lock down 82278944 byte memory area (Cannot allocate memory)": run the command "sudo mv /etc/security/limits.d/audio.conf.disable /etc/security/limits.d/audio.conf" to unlock the mem limit, if you got "Inbound TCP/IP connection failed" following this post: http://tianb03.blogspot.com/2016/08/ros-issues-problems-during-install.html  
7. pocketsphinx and needed gstreamer-0.10 instead of gstreamer-1.0, and it need the plugin gstreamer-0.10-pocketsphinx. If there are not in the apt-cache, you should compile them by yourself, if you do so, use should this script: https://gist.github.com/ccckblaze/61770b2f4311b6475a6e41cac604ba4f , if you got "error: ‘V4L2_CID_HCENTER_DEPRECATED’ undeclared": try apply this patch: https://github.com/LeSpocky/ptxdist/blob/master/patches/gst-plugins-good-0.10.31/0003-remove-V4L2_CID_HCENTER-V4L2_CID_VCENTER-usage.patch. After that, when you try launch ros if you got: "glib.GError: no element 'vader'", try run "export GST_PLUGIN_PATH=/usr/local/lib/gstreamer-0.10"  
8. ros package got error "'/usr/include/pcl-1.7' does not exist", try "export PCL_DIR=/usr/local/share/pcl-1.8/PCLConfig" or whatever the PCLConfig.cmake should be, this also deal with the issues that libs installed but not found, last sometimes delete the CMakeCache.txt will help the cmake to relocate to the right directory  

