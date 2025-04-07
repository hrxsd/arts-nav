# arts-nav
Hardware setup of Unitree-GO1
Function: The robot can receive /cmd_vel commands published by ROS on NUC and move them.
文希负责完善这部分内容
Mapping & Navigation
Install the mapping and navigation modules
sudo apt-get install ros-noetic-serial
sudo apt-get install libpcap-dev

cd ~
mkdir -p arts-nav-ws/src && cd arts-nav/src
git clone https://github.com/hrxsd/arts-nav.git
cd ..
catkin_make
If the error is related to <mapping/Pose6D.h>, run catkin_make again until there is no error.
Note: Due to the large number of packages and their dependency on each other, there will be a false error. You can run catkin_make several times to solve the problem. If you run catkin_make three times, there is the same error. That is the real error message.
If the error message is related to move_base_msgs, you can install the corresponding package.
Bind IMU usb port to /dev/imu 
sudo vim /etc/udev/rules.d/imu.rules 
# add code below
KERNEL=="ttyUSB*",  ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0777", SYMLINK+="imu"
# Save and run the following code
sudo service udev reload
sudo service udev restart
# Re-plugging the IMU usb cable
ls -l /dev/imu # View mapping relationships
Testing the mapping module
Test mapping and chassis function package communication
Create a new folder named PCD under the path /arts-nav/Mapping to save the 3D point cloud.
- Connect the host computer to the radar network cable and make sure the radar is powered on.
- Connect the host computer to the IMU usb cable.
- Manually change the ipv4 address of the host network cable to the same network segment as the ip address of the laser radar, with mask 255.255.255.0.
cd ~/arts-nav-ws
source devel/setup.bash
roslaunch mapping mapping.launch
# rviz has pointcloud data and press <j> can rotate robot to test ok
Lidar scanned 3D point clouds are saved in arts-nav/Mapping/PCD
1. Map preservation
Before saving the map, change the absolute path in the file /arts-nav/Mapping/src/pcd2pgm.cpp to the correct path.
[图片]
roslaunch mapping saver.launch
The location of the saved map is arts-nav/hdl_localization/config/map.
2. Navigation
Test navigation
roslaunch navigation nav_test.launch
