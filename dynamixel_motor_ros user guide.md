### dynamixel 舵机 ros使用指南

> http://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_workbench/
> http://emanual.robotis.com/docs/en/dxl/x/xh430-w350/#control-table-of-eeprom-area
> http://emanual.robotis.com/docs/en/dxl/protocol2/#status-packet

首先从官网下载

将文件`dynamixel-workbench,dynamixel-workbench-msgs,DynamixelsSDK`文件夹放进工作空间`catkin_ws`内，其中前面两个是main packages，SDK是depend package。

#### 如何编写一个简单的舵机控制程序

首先建立自己的功能包，添加的依赖：

```cmake
find_package(catkin REQUIRED COMPONENTS
  dynamixel_workbench_toolbox
  roscpp
  std_msgs
)
```

在`cpp`文件中是引入 `dynamixel_workbench_toolbox`这个API，这个API定义了官方提到的一些封装号的函数

```c++
#include<dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include<ros/ros.h>
```

基本使用的方法如下（更多函数的用法参照官网，或者源文件）

```c++
DynamixelWorkbench dxl_wb; 	//声明对象
dxl_wb.begin("/dev/ttyUSB0",BAUDRATE);	//打开串口
dxl_wb.setPositionControlMode(ID);	//设置位置控制模式
dxl_wb.torqueOn(ID);	//上电，只有断电的情况下才能设置控制模式
dxl_wb.itemRead(ID,"Present_Position",&present_position);	//读取当前的位置
dxl_wb.itemWrite(ID,"Goal_Position",goal_position);			//写入目标的位置
// 这里的position current velocity 都是以int32_t的形式存储
```

#### 如何自己复现一个workbench

官方提供的workbench能够实现**YAML文件配置舵机，轨迹订阅、发布、插值**等功能，采用上小节的方法只能实现单舵机控制，官方的代码更具有稳定性和便捷性，但由于官方代码中还包含一些集成好的控制模式（二轮差速等），直接在其进行二次开发代码可读性差，灵活性差，因此本小节介绍如何借助官方源码实现`my_workbench`

本小节的思路按照各个功能展开：

##### 舵机配置

舵机配置需要借助YAML文件，其文件的内容如下所示：

```yaml
# yaml文件是一种可读性高的配置文件，官方提供的库能够方便的将其数据内容存储为树的形式
# 舵机中可以配置的参数参照官方提供的control table，注意命名方式

zero:		#舵机的别名
  ID: 0		#舵机的ID
  Operating_Mode: 0		#控制模式
  Current_Limit: 50		#电流限制
  Torque_Enable: 1
```

使用YAML文件需要在包中添加yaml的依赖，如果没有需提前安装

CMakeList文件：

```cmake
# add yaml_cpp
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR
  NAMES yaml_cpp.h
  PATHS ${YAML_CPP_INCLUDE_DIRS}
)
find_library(YAML_CPP_LIBRARY
  NAMES YAML_CPP
  PATHS ${YAML_CPP_LIBRARY_DIRS}
)
link_directories(${YAML_CPP_LIBRARY_DIRS})

if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")


# 包含路径
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

# 生成可执行文件时也需要增加依赖
target_link_libraries(name ${catkin_LIBRARIES}  ${YAML_CPP_LIBRARIES})
```

Cpp文件以及h文件

h文件

```c++
#include <yaml-cpp/yaml.h>
#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
```

cpp文件

流程：

```c++
std::string port_name = "/dev/ttyUSB0";
uint32_t baud_rate = 57600;

// 在启动时可以修改串口位置以及波特率
if (argc < 2)
{
    ROS_ERROR("Please set '-port_name' and  '-baud_rate' arguments for connected Dynamixels");
    return 0;
}
else
{
    port_name = argv[1];
    baud_rate = atoi(argv[2]);
}

//定义了自己的controller类，这个controller 包含上小节的DynamixelWorkbench对象
MyDynamixelController dynamixel_controller;	

//获取yaml配置文件文件名,在launch文件中定义
std::string yaml_file = node_handle.param<std::string>("dynamixel_info", "");

//下列具体参考官方提供的源代码，流程如下
//这些函数都是controller对象的功能函数，其实现都从官方源码copy过来

//打开串口
dynamixel_controller.initWorkbench(port_name, baud_rate);	
//读取配置文件内容，并存储
dynamixel_controller.getDynamixelsInfo(yaml_file);	
//加载配置文件，显示当前舵机在线的数量，名字，以及id号
dynamixel_controller.loadDynamixels();
//将yaml文件的参数写入舵机
dynamixel_controller.initDynamixels();
//初始化control table，获取舵机当前值和目标值
dynamixel_controller.initControlItems();
// addSyncWriteHandler addSyncReadHandler
dynamixel_controller.initSDKHandlers();

//舵机配置部分结束
ROS_INFO("Welcome my dynamixel workbench!");
```

##### 读写舵机参数，并发布

官方提供了舵机状态信息的message，位于`dynamixel_workbench_msgs`中

```C++
#include <dynamixel_workbench_msgs/DynamixelStateList.h>
```

状态函数保存在

```yaml
# DynamixelStateList.msg
DynamixelState[] dynamixel_state

# DynamixelState.msg
string name
uint8  id

int32  present_position
int32  present_velocity
int16  present_current
```

通过**定时器**定时读取舵机的信息（位置、速度、电流等）保存在DynamixelStateList类型的msg中，发布舵机的传感参数，定义舵机的控制周期

```c++
ros::Timer read_timer = node_handle.createTimer(ros::Duration(dynamixel_controller.getReadPeriod()),
                                  &MyDynamixelController::readCallback,&dynamixel_controller);

ros::Timer write_timer = node_handle.createTimer(ros::Duration(dynamixel_controller.getWritePeriod()),
                               &MyDynamixelController::writeCallback, &dynamixel_controller);

ros::Timer publish_timer = node_handle.createTimer(ros::Duration(dynamixel_controller.getPublishPeriod()), 
                              &MyDynamixelController::publishCallback, &dynamixel_controller);
```

下列一次说明各个callback函数的注意点

```c++
/*******	readCallback	*******/
//readCallBack 中可以定义需要哪些数据，如果你需要位置、速度、电流的话
#define SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT 0	//定义在头文件中
// 接着copy提供的源文件即可,如
result = dxl_wb_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                id_array,
                                id_cnt,
                                control_items_["Present_Current"]->address,
                                control_items_["Present_Current"]->data_length,
                                get_current,
                                &log);

/*******	writeCallback	*******/
//在writeCallBack中，需要根据控制模式在头文件中预定义
/*
#define SYNC_WRITE_HANDLER_FOR_GOAL_POSITION 0
#define SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY 1
#define SYNC_WRITE_HANDLER_FOR_GOAL_CURRENT  2
*/
result = dxl_wb_->syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_CURRENT, id_array, id_cnt, dynamixel_current, 1, &log);	//例如以电流控制模式


/*******	publishCallback	*******/
dynamixel_state_list_pub_.publish(dynamixel_state_list_);
```

通过`rosbag`指令可以方便记录

```shell
rosbag record /my_dxl_master/dynamixel_state	#记录dynamixel_state msg

#将记录的数据转变成txt文件便于处理
rostopic echo -b 2020-03-31-20-05-26.bag -p my_dxl_master/dynamixel_state > state.txt

# 也可以通过	play 命令，此时通过rqt_plot 可以观察到数据
rosbag play filename.bag -l #循环播放
```

##### 轨迹发布以及插值

官方提供的解决方案是controller发布轨迹msg被master接受，再根据控制频率进行最小急动度插值

轨迹的文件有yaml文件格式定义，通过launch文件指定。

定义operator类

```c++
// 头文件
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
```

trajectory_msgs

```yaml
#trajectory_msgs/JointTrajectory.msg
Header header
string[] joint_names
JointTrajectoryPoint[] points

#trajectory_msgs/JointTrajectoryPoint.msg
# Each trajectory point specifies either positions[, velocities[, accelerations]]
# or positions[, effort] for the trajectory to be executed.
# All specified values are in the same order as the joint names in JointTrajectory.msg
float64[] positions
float64[] velocities
float64[] accelerations
float64[] effort
duration time_from_start

#desired_trajectory.msg
uint8[] id
int32[] goal_position
```

operator完成的任务

（1）解析轨迹yaml文件并存储到JointTrajectory中（2）将JointTrajectory发布出去

在这里官方的提供的轨迹文件格式说实话我有些看不懂（可能和当时需求有关），于是自己定义了一个格式，可供参考

```yaml
joint:
  names: [zero,first,second]
trajectory:
  index: [wp1,wp2]
  wp1:
    pos: [0,0,0]
    vel: [0.0,0.0]
    
    time_from_start: 0.2
  wp2:
    pos: [3.14,3.14,3.14]
    vel: [0.0,0.0]	#也可以定义加速度，我没用到
    time_from_start: 0.8
```

由于自己定义了文件的格式因此需要自己写解析的函数，参考如下

```c++
bool MyOperator::getTrajectoryInfo(const std::string yaml_file,
                     trajectory_msgs::JointTrajectory *jnt_tra_msg)
{
    YAML::Node file;
    file = YAML::LoadFile(yaml_file.c_str());
    if (file == NULL)
        return false;
    //
    YAML::Node joint = file["joint"];
    uint16_t joint_size = joint["names"].size();

    for (uint16_t index = 0; index < joint_size; index++)
    {
        std::string joint_name = joint["names"][index].as<std::string>();
        jnt_tra_msg->joint_names.push_back(joint["names"][index].as<std::string>());
    }
    //
    YAML::Node trajectory = file["trajectory"];
    uint16_t trajectory_size = trajectory["index"].size();
    for (uint16_t index = 0;index<trajectory_size;index++)
    {
        trajectory_msgs::JointTrajectoryPoint jnt_tra_point;
        std::string index_ = trajectory["index"][index].as<std::string>();
        YAML::Node wp_num = trajectory[index_];
        if (joint_size != wp_num["pos"].size())
        {
            ROS_ERROR("Please check way point pos size. It must be equal to joint size");
            return 0;
        }
        for (uint16_t size=0;size<wp_num["pos"].size();size++)
        {
            jnt_tra_point.positions.push_back(wp_num["pos"][size].as<double>());
            ROS_INFO("NO.way point : %s, position : %f", index_.c_str(), wp_num["pos"][size].as<double>());
        }
        if (wp_num["vel"] != NULL)
        {
            // 速度的size也要和pos一样
            for (uint16_t size=0;size<wp_num["vel"].size();size++)
            {
                jnt_tra_point.velocities.push_back(wp_num["vel"][size].as<double>());
                ROS_INFO("NO.way point : %s, velocity : %f", index_.c_str(), wp_num["vel"][size].as<double>());
            }
        }
        if (wp_num["acc"] != NULL)
        {
            //加速度的size也要和pos一样
            for (uint16_t size=0;size<wp_num["acc"].size();size++)
            {
                jnt_tra_point.accelerations.push_back(wp_num["acc"][size].as<double>());
                ROS_INFO("NO.way point : %s, acceleration : %f", index_.c_str(), wp_num["acc"][size].as<double>());
            }
        }
        if (wp_num["time_from_start"] == NULL)
        {
            ROS_ERROR("Please check time_from_start. It must be set time_from_start each waypoint");
            return 0;
        }
        jnt_tra_point.time_from_start.fromSec(wp_num["time_from_start"].as<double>());

        ROS_INFO("time_from_start : %f", wp_num["time_from_start"].as<double>());

        jnt_tra_msg->points.push_back(jnt_tra_point);     
    }
    return true;
}
```

发布轨迹的msg

```c++
joint_trajectory_pub_.publish(*jnt_tra_msg_);
```

轨迹发布之后就由master先接收，再根据控制频率进行插值

定义了一个`trajectoryMsgCallback`函数，代码可直接copy官方源码



