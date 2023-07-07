# 目前状态

melodic 版本可以完全运行, 机器人可以选择 kobuki 或者 先锋。

noetic 版本可以生成全局和局部 rrt 并找到 frontier 但是不能进行聚类，应该是 python 环境的问题，noetic 是 python3，所以 python2 的 meanshift() 函数（==python 文件 filter.py 有调用这个函数==）应该用起来有问题, 可以在 noetic 安装 python2 环境。机器人可以选择 kobuki 或者 先锋。

# 安装先锋机器人

按照 p3dx 的 readme 安装即可

# 安装 kobuki 和 RRT_Exploration

1 **安装依赖：**

```bash
sudo apt-get install ros-<版本>-kobuki-*
sudo apt-get install ros-<版本>-ecl-streams
sudo apt-get install libusb-dev
sudo apt-get install libspnav-dev
sudo apt-get install bluetooth
sudo apt-get install libbluetooth-dev
sudo apt-get install libcwiid-dev
```

2 **安装 bfl：**

```bash
sudo apt-get install liborocos-bfl-dev
```

3 下载 kobuki_package 到工作空间 src 下**编译**：

```bash
catkin_make
```

4 安装 gmapping 和 navigation

```bash
sudo apt-get install ros-<版本>-gmapping ros-<版本>-navigation
```

5 编译所有包

# **ros noetic 版本 melodic 版本**需要修改的地方：

rrt_exploration_tutorials/launch/includes/urdf/kobuki.urdf.xacro

```
<kobuki_sim/>   melodic 版本写成这样

<xacro:kobuki_sim/>  noetic 版本写成这样
```

rrt_exploration_tutorials/launch/includes/urdf/kobuki_standalone.urdf.xacro

```
<kobuki/>     melodic 版本写成这样

<xacro:kobuki/>   noetic 版本写成这样
```

kobuki_package/kobuki_desktop/kobuki_gazebo_plugins/src/gazebo_ros_kobuki_updates.cpp （大概在 71 行）

```C++
  std::string odom_frame = gazebo_ros_->resolveTF("odom");
  std::string base_frame = gazebo_ros_->resolveTF("base_footprint"); // melodic 版本写成这样
  
  std::string odom_frame = gazebo_ros_->resolveTF("robot_1/odom");
  std::string base_frame = gazebo_ros_->resolveTF("robot_1/base_footprint"); // noetic 版本写成这样，如果想做多机器人，需要自己在源码添加 namespace, 而melodic 版本完全不需要，写成上面那样就可以了
```

参考链接：

https://blog.csdn.net/weixin_44420419/article/details/112236009

https://blog.csdn.net/qq_39779233/article/details/128406859



# 在 noetic 版本下 先锋机器人报错， melodic 版本应该不会报错

## 1 No  frame hokuyo_utm30lx_link 

rrt_exploration_tutorials/launch/single_simulated_house_p3dx.launch 写上如下： ==melodic==不用写

```
<node pkg="tf" type="static_transform_publisher" name="Noetic_Debug" args="0 0 0 0 0 0 /robot_1/hokuyo_utm30lx_link hokuyo_utm30lx_link 20" />
```

## 2 TF_REPEATED_DATA ignoring data with redundant timestamp for frame

p3dx/p3dx_description/urdf/p3dx/**pioneer3dx_plugins.xacro**

```
  <!-- <gazebo>
    <plugin name="joint_state_publisher" filename="libgazebo_ros_joint_state_publisher.so">
      <jointName>chassis_swivel_joint, swivel_wheel_joint, left_hub_joint, right_hub_joint</jointName>
      <updateRate>10.0</updateRate>
      <alwaysOn>true</alwaysOn>
    </plugin>
  </gazebo> -->
修改成如下：
  <gazebo>
    <plugin name="joint_state_publisher" filename="libgazebo_ros_joint_state_publisher.so">
      <!-- <robotNamespace>$(arg robot_namespace)</robotNamespace> -->
      <jointName>chassis_swivel_joint, swivel_wheel_joint</jointName>
      <updateRate>10.0</updateRate>
      <alwaysOn>true</alwaysOn>
    </plugin>
  </gazebo>
```

https://mr-winter.blog.csdn.net/article/details/126291629?spm=1001.2101.3001.6661.1&utm_medium=distribute.pc_relevant_t0.none-task-blog-2%7Edefault%7ECTRLIST%7ERate-1-126291629-blog-126875442.235%5Ev38%5Epc_relevant_default_base3&depth_1-utm_source=distribute.pc_relevant_t0.none-task-blog-2%7Edefault%7ECTRLIST%7ERate-1-126291629-blog-126875442.235%5Ev38%5Epc_relevant_default_base3&utm_relevant_index=1



## 3 \#!/usr/bin/env python  NO python 文件

```bash
sudo apt-get install python-is-python3
```

https://blog.csdn.net/GerZhouGengCheng/article/details/118468389

## 4 A NumPy version ＞=1.19.5 and ＜1.27.0 is required for this version of SciPy版本不兼容，更新numpy

```bash
sudo apt install python3-pip
pip install --upgrade numpy
```

https://blog.csdn.net/weixin_58283807/article/details/129692220

## 5 ModuleNotFoundError: No module named ‘sklearn‘

```bash
pip install numpy
pip install scipy
pip install matplotlib
pip install scikit-learn
```

https://blog.csdn.net/weixin_50952710/article/details/127961433?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522168854952916800184176686%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=168854952916800184176686&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduend~default-1-127961433-null-null.142^v88^koosearch_v1,239^v2^insert_chatgpt&utm_term=ModuleNotFoundError%3A%20No%20module%20named%20sklearn&spm=1018.2226.3001.4187

# 在 noetic 版本下 kobuki 机器人报错， melodic 版本应该不会报错

## 1 No  frame base_laser_link 

rrt_exploration_tutorials/launch/single_simulated_house.launch 写上如下： ==melodic==不用写

```
<node pkg="tf" type="static_transform_publisher" name="Noetic_Debug" args="0 0 0 0 0 0 /robot_1/base_laser_link base_laser_link 20" />
```



# 运行算法

运行 rrt_exploration_tutorials/launch/single_simulated_house_p3dx.launch （先锋）

==或者==

运行 rrt_exploration_tutorials/launch/single_simulated_house.launch （kobuki）

==然后==

运行 rrt_exploration/launch/single.launch