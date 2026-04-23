# 底盘
## 安装底盘模块
```bash
sudo apt install -y libasio-dev
sudo apt install -y ros-$ROS_DISTRO-teleop-twist-keyboard

cd ~
mkdir -p arts_nav_ws/src && cd arts_nav_ws/src
git clone https://github.com/agilexrobotics/ugv_sdk.git 
git clone https://github.com/agilexrobotics/scout_ros.git
cd ..
catkin_make
```
## 测试底盘模块
1. 首次上电后使能usb2can
```bash
# 使能gs_usb内核模块
sudo modprobe gs_usb
# 设置500k波特率和使能can2usb适配器
sudo ip link set can0 up type can bitrate 500000
# 安装并使用can-utils来测试硬件
sudo apt install -y can-utils
# 测试can通信
candump can0 # 有数据说明通信成功
```
2. 之后上电只需设置波特率
```bash
sudo ip link set can0 up type can bitrate 500000
```
3. 测试底盘
```bash
candump can0  #有数据则正常
#没有数据则插拔can2usb（或者重启底盘）并重新运行：
sudo ip link set can0 up type can bitrate 500000
candump can0

cd ~/arts_nav_ws
source devel/setup.bash
roslaunch scout_bringup scout_miniomni_robot_base.launch
# create a new terminal
rosrun teleop_twist_keyboard teleop_twist_keyboard.py 
# 必须选中上述终端才能控制
# press <j> make car rot ， press <k> stop  
# 注： 如果没有远程连接，还有线接外设或其他设备，小心遥控！
```
# Mapping & Navigation
## 安装mapping和navigation模块
```bash
sudo apt-get install ros-$ROS_DISTRO-serial
sudo apt-get install libpcap-dev

cd ~
mkdir -p arts_nav_ws/src && cd arts_nav_ws/src
git clone https://github.com/hrxsd/arts-nav.git
cd ..
catkin_make
```
报错与`<mapping/Pose6D.h>`相关，重新再运行一遍`catkin_make`直到无报错
注：由于功能包较多并且互相依赖，所以会出现假报错情况，可以重复运行多次catkin_make即可解决问题，如果运行三次catkin_make都是同样的报错，那才为真实的错误信息。
cd另外，如果报错和move_base_msgs有关，安装相应功能包即可。
## 绑定IMU usb端口为 /dev/imu 
```bash
sudo gedit /etc/udev/rules.d/imu.rules 
# add code below
KERNEL=="ttyUSB*",  ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0777", SYMLINK+="imu"
# 保存后运行下面代码
sudo service udev reload
sudo service udev restart
#重新插拔IMU usb线
ls -l /dev/imu #查看映射关系
```
## 测试mapping模块
### 测试mapping和底盘功能包通信
在/arts_nav/Mapping路径下新建名为PCD的文件夹，用于保存三维点云
- 主机连接雷达网线，并确认雷达电源接通
- 主机连接IMU usb线
- 将主机网线ipv4地址手动修改为192.168.5.1，掩码255.255.255.0
```bash
cd ~/arts_nav_ws
source devel/setup.bash
roslaunch mapping mapping.launch
# rviz无雷达数据，修改rslidar_sdk的config,修改对应的雷达型号lidar_type: RSHELIOS_16P 
# rviz有雷达数据且press <j> 可旋转小车即测试正常
```
雷达扫描的3D点云保存在arts_nav/Mapping/PCD
### 保存地图
保存地图前，先将/arts_nav/Mapping/src/pcd2pgm.cpp文件中的绝对路径改为正确的路径
<img width="1134" height="54" alt="image" src="https://github.com/user-attachments/assets/a6e22dd3-5949-40e6-902c-557ef690e63c" />
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py 
roslaunch mapping saver.launch
```
保存的地图位置是arts_nav/hdl_localization/config/map
### 测试navigation模块
```bash
cd ~/arts_nav_ws
source devel/setup.bash
roslaunch navigation nav_test.launch # 使用2D nav_goal发布目标点
```
# 可视化app
## 安装依赖
```bash
sudo apt-get update
sudo apt-get install qtbase5-private-dev libqt5svg5-dev libsdl-image1.2-dev libsdl1.2-dev -y
```
## Cmake升级
ubuntu 18.04 默认安装的是 3.10.2 版本，需要升级到 3.16+ 版本, Ubuntu20.04及以上可跳过此步骤
```bash
wget https://cmake.org/files/v3.16/cmake-3.16.4-Linux-x86_64.sh -O cmake-install.sh
chmod +x cmake-install.sh
sudo ./cmake-install.sh --prefix=/usr/local --skip-license
# 运行
cd ~/arts_nav_ws/src/arts-nav/app
./ros_qt5_app
```
## 配置说明
配置文件 config.json 在首次运行软件后会自动生成在可执行程序同级目录下。修改配置后需要重启软件生效。
配置项说明
### ros话题配置
```json
{
  "topics": {
    "map": {
      "display_name": "Map",
      "topic": "/map",
      "enable": true
    },
    "laser": {
      "display_name": "LaserScan", 
      "topic": "/scan",
      "enable": true
    },
    "odom": {
      "display_name": "Odometry",
      "topic": "/odom", 
      "enable": true
    },
    "velocity": {
      "display_name": "Speed",
      "topic": "/cmd_vel",
      "enable": true
    },
    "initialpose": {
      "display_name": "Reloc",
      "topic": "/initialpose",
      "enable": true
    },
    "move_base_simple": {
      "display_name": "NavGoal",
      "topic": "/move_base_simple/goal",
      "enable": true
    },
    "battery": {
      "display_name": "Battery",
      "topic": "/battery",
      "enable": true
    }
  }
}
```
### 机器人外形配置
```json
{
  "robot_shape_config": {
    "shaped_points": [
      {
        "x": 0.5,
        "y": 0.5
      },
      {
        "x": 0.5,
        "y": -0.5
      },
      {
        "x": -0.5,
        "y": -0.5
      },
      {
        "x": -0.5,
        "y": 0.5
      }
    ],
    "is_ellipse": false,
    "color": "0x00000FF",
    "opacity": 0.5
  }
}
```
### 相机配置（目前有bug）
```json
{
  "images": [
    {
      "location": "front",
      "topic": "/camera/rgb/image_raw",
      "enable": true
    },
    {
      "location": "front/depth",
      "topic": "/camera/depth/image_raw", 
      "enable": true
    }
  ]
}
```
## 功能使用指南
### 地图显示与编辑
#### 地图显示
软件支持显示全局地图和局部地图，地图数据来自 ROS 话题。
要启用地图显示，请确保 config.json 中有以下配置：
```json
{
  "topics": {
    "map": {
      "display_name": "Map",
      "topic": "/map",
      "enable": true
    }
  }
}
```
#### 地图编辑
提供以下编辑功能：
<img width="1280" height="834" alt="image" src="https://github.com/user-attachments/assets/779f831c-58bc-4e1e-8984-26cf6a056e5b" />

#### 拓扑地图
支持拖拽设置机器人导航目标点
注意：如果导航点发布无响应，请检查以下配置：
```json
{
  "move_base_simple": {
    "display_name": "NavGoal",
    "topic": "/move_base_simple/goal",
    "enable": true
  }
}
```
#### 橡皮擦工具
点击橡皮擦可以擦除地图中的障碍物：
#### 画笔工具
使用画笔绘制障碍物：
#### 线段绘制
在地图上绘制直线：
#### 地图保存
编辑完成后，点击保存按钮保存：
- *.pgm - 图像数据
- *.yaml - 地图描述文件
- *.topology - 拓扑数据（导航点信息）
#### 机器人控制

##### 手动控制
使用键盘或界面按钮控制机器人
注意：请检查手动控制的配置：
```json
{
  "velocity": {
    "display_name": "Speed",
    "topic": "/cmd_vel",
    "enable": true
  }
}
```
#### 机器人重定位
左键按住拖动设置位置，右键旋转方向：
注意：确保正确配置：
```json
{
  "initialpose": {
    "display_name": "Reloc",
    "topic": "/initialpose",
    "enable": true
  }
}
```
#### 速度仪表盘（暂时不可用）
#### 实时显示机器人速度：
需要的配置：
```json
{
  "odom": {
    "display_name": "Odometry",
    "topic": "/odom",
    "enable": true
  }
}
```
#### 电池显示（暂时不可用）
#### 显示实时电池状态：
配置（使用 sensor_msgs::BatteryState）：
```json
{
  "battery": {
    "display_name": "Battery",
    "topic": "/battery",
    "enable": true
  }
}
```
#### 导航功能
##### 多点导航
设置多个导航点按顺序执行：
使用步骤：
1. 添加导航点
2. 设置执行顺序
3. 点击"开始任务"按钮
4. 监控任务进度
#### 相机显示（暂时不可用）
支持多路相机图像：
- RGB和深度图像
- 压缩传输
- 移植自 rqt_image_view
配置示例：
```json
{
  "images": [
    {
      "location": "front",
      "topic": "/camera/rgb/image_raw",
      "enable": true
    },
    {
      "location": "front/depth",
      "topic": "/camera/depth/image_raw",
      "enable": true
    }
  ]
}
```
#### 机器人车身显示
支持多种车身形状：
- 矩形
- 圆形
- 自定义形状
配置示例：
```json
{
  "robot_shape_config": {
    "shaped_points": [
      {"x": 0.5, "y": 0.5},
      {"x": 0.5, "y": -0.5},
      {"x": -0.5, "y": -0.5},
      {"x": -0.5, "y": 0.5}
    ],
    "is_ellipse": false,
    "color": "0x00000FF",
    "opacity": 0.5
  }
}
```
