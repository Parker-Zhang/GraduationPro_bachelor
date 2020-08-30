/**********************************************************************
//my_dxl_master.cpp
//Date:2020.2.28
//Descripotion:This is a control_master that connect real motor through
//USB,publish motor's state,subscribe motor's control command from motor
//node.
//我们在启动的时候，需要通过配置文件来对舵机进行初始化
//另外需要有一个发布者，发布的内容包括舵机的ID，舵机的电流，位置，速度，力矩等信息
//同时需要一个订阅者，订阅motor节点发布的控制命令，并通过串口发送到实体舵机
**********************************************************************/
#include<dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include<ros/ros.h>
#include<my_dynamixel_workbench_test/dxl_state.h>
#include<iostream>
#include <fstream>
#include <sstream>
#include<my_dxl_master.h>
#include<my_dynamixel_workbench_test/ChangeGoalPosition.h>

using namespace std;

#define MOTOR_NUM 3

MyDynamixelController::MyDynamixelController()
 :node_handle_(""),
  priv_node_handle_("~")
{
    dxl_wb_ = new DynamixelWorkbench;
    jnt_tra_msg_ = new trajectory_msgs::JointTrajectory;
    jnt_tra_test_ = new trajectory_msgs::JointTrajectory;
    read_period_ = priv_node_handle_.param<double>("dxl_read_period", 0.010f);
    write_period_ = priv_node_handle_.param<double>("dxl_write_period", 0.010f);
    pub_period_ = priv_node_handle_.param<double>("publish_period", 0.10f);
    jnt_tra_ = new JointTrajectory;
    
    goal_position = 0;
    p_gain = 0.05;//0.05
    i_gain = 0;//
    d_gain = 0.01;//0.1
    is_moving_ = false;
}


bool MyDynamixelController::initWorkbench(const std::string port_name, const uint32_t baud_rate)
{
  bool result = false;
  const char* log;

  result = dxl_wb_->init(port_name.c_str(),baud_rate,&log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
  }

  return result;
}

bool MyDynamixelController::getDynamixelsInfo(const std::string yaml_file)
{
  //node可以理解为 树的根节点，这里什么时候分叉，需要看缩进符的结构
  YAML::Node dynamixel;
  dynamixel = YAML::LoadFile(yaml_file.c_str());

  if (dynamixel == NULL)
    return false;
  //iterator 翻译为 迭代器，有时又称游标（cursor）是程序设计的软件设计模式，可在容器上遍访的接口
  for (YAML::const_iterator it_file = dynamixel.begin(); it_file != dynamixel.end(); it_file++)
  {
    //这里觉得是读到用户自定义舵机的别名
    std::string name = it_file->first.as<std::string>();
    if (name.size() == 0)
    {
      continue;
    }

    YAML::Node item = dynamixel[name];
    for (YAML::const_iterator it_item = item.begin(); it_item != item.end(); it_item++)
    {
      std::string item_name = it_item->first.as<std::string>();
      int32_t value = it_item->second.as<int32_t>();

      //把ID号赋值给舵机的别名
      if (item_name == "ID")
        dynamixel_[name] = value;

      ItemValue item_value = {item_name, value};
      std::pair<std::string, ItemValue> info(name, item_value);

      dynamixel_info_.push_back(info);
    }
  }
  return true;
}

//按照加载的配置文件，显示当前舵机在线的数量，名字，以及id号
bool MyDynamixelController::loadDynamixels(void)
{
  bool result = false;
  const char* log;

  for (auto const& dxl:dynamixel_)
  {
    uint16_t model_number = 0;
    result = dxl_wb_->ping((uint8_t)dxl.second, &model_number, &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
      ROS_ERROR("Can't find Dynamixel ID '%d'", dxl.second);
      return result;
    }
    else
    {      
      ROS_INFO("Name : %s, ID : %d, Model Number : %d", dxl.first.c_str(), dxl.second, model_number);
    }
  }

  return result;
}
//初始化舵机，注意写入的时候，舵机的力矩要torqueoff，写入完成，再torque on
bool MyDynamixelController::initDynamixels(void)
{
  const char* log;

  for (auto const& dxl:dynamixel_)
  {
    dxl_wb_->torqueOff((uint8_t)dxl.second);

    for (auto const& info:dynamixel_info_)
    {
      if (dxl.first == info.first)
      {
        if (info.second.item_name != "ID" && info.second.item_name != "Baud_Rate")
        {
          bool result = dxl_wb_->itemWrite((uint8_t)dxl.second, info.second.item_name.c_str(), info.second.value, &log);
          if (result == false)
          {
            ROS_ERROR("%s", log);
            ROS_ERROR("Failed to write value[%d] on items[%s] to Dynamixel[Name : %s, ID : %d]", info.second.value, info.second.item_name.c_str(), dxl.first.c_str(), dxl.second);
            return false;
          }
        }
      }
    }

    dxl_wb_->torqueOn((uint8_t)dxl.second);
  }

  return true;
}
//这是初始化控制表，即获得舵机当前值以及目标值
bool MyDynamixelController::initControlItems(void)
{
  bool result = false;
  const char* log = NULL;

  auto it = dynamixel_.begin();

  const ControlItem *goal_position = dxl_wb_->getItemInfo(it->second, "Goal_Position");
  if (goal_position == NULL) return false;

  const ControlItem *goal_velocity = dxl_wb_->getItemInfo(it->second, "Goal_Velocity");
  if (goal_velocity == NULL)  goal_velocity = dxl_wb_->getItemInfo(it->second, "Moving_Speed");
  if (goal_velocity == NULL)  return false;

  const ControlItem *goal_current = dxl_wb_->getItemInfo(it->second, "Goal_Current");
  if (goal_current == NULL) {
    ROS_INFO("goal_current NULL");
    return false;  }

  const ControlItem *present_position = dxl_wb_->getItemInfo(it->second, "Present_Position");
  if (present_position == NULL) return false;

  const ControlItem *present_velocity = dxl_wb_->getItemInfo(it->second, "Present_Velocity");
  if (present_velocity == NULL)  present_velocity = dxl_wb_->getItemInfo(it->second, "Present_Speed");
  if (present_velocity == NULL) return false;

  const ControlItem *present_current = dxl_wb_->getItemInfo(it->second, "Present_Current");
  if (present_current == NULL)  present_current = dxl_wb_->getItemInfo(it->second, "Present_Load");
  if (present_current == NULL) return false;

  control_items_["Goal_Position"] = goal_position;
  control_items_["Goal_Velocity"] = goal_velocity;
  control_items_["Goal_Current"] = goal_current;

  control_items_["Present_Position"] = present_position;
  control_items_["Present_Velocity"] = present_velocity;
  control_items_["Present_Current"] = present_current;

  return true;
}
//Handler 事件处理器？ 对象执行某个动作？？
bool MyDynamixelController::initSDKHandlers(void)
{
  bool result = false;
  const char* log = NULL;

  auto it = dynamixel_.begin();

  result = dxl_wb_->addSyncWriteHandler(control_items_["Goal_Position"]->address, control_items_["Goal_Position"]->data_length, &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
    return result;
  }
  else
  {
    ROS_INFO("%s", log);
  }

  result = dxl_wb_->addSyncWriteHandler(control_items_["Goal_Velocity"]->address, control_items_["Goal_Velocity"]->data_length, &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
    return result;
  }
  else
  {
    ROS_INFO("%s", log);
  }

  result = dxl_wb_->addSyncWriteHandler(control_items_["Goal_Current"]->address, control_items_["Goal_Current"]->data_length, &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
    return result;
  }
  else
  {
    ROS_INFO("%s", log);
  }

  //采用通讯协议2.0版本
  if (dxl_wb_->getProtocolVersion() == 2.0f)
  {  
    uint16_t start_address = std::min(control_items_["Present_Position"]->address, control_items_["Present_Current"]->address);

    /* 
      As some models have an empty space between Present_Velocity and Present Current, read_length is modified as below.
    */    
    // uint16_t read_length = control_items_["Present_Position"]->data_length + control_items_["Present_Velocity"]->data_length + control_items_["Present_Current"]->data_length;
    uint16_t read_length = control_items_["Present_Position"]->data_length + control_items_["Present_Velocity"]->data_length + control_items_["Present_Current"]->data_length+2;

    result = dxl_wb_->addSyncReadHandler(start_address,
                                          read_length,
                                          &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
      return result;
    }
  }

  return result;
}
//同时读取所有舵机的数值，Sync Read 

//同时控制所有舵机的数值，Sync Write

// subscriber
void MyDynamixelController::initSubscriber(){
  trajectory_sub_ = node_handle_.subscribe("joint_trajectory", 100, &MyDynamixelController::trajectoryMsgCallback, this);
}

bool MyDynamixelController::getPresentPosition(std::vector<std::string> dxl_name)
{
  bool result = false;
  const char* log = NULL;
  int32_t get_position[dxl_name.size()];

  uint8_t id_array[dxl_name.size()];
  uint8_t id_array2[dxl_name.size()];
  uint8_t id_cnt = 0;
// 参考系统自动检测的顺序：1 4 2 3 0
  for (auto const& dxl:dynamixel_)
  {
    id_array2[id_cnt++] = (uint8_t)dxl.second;
    // ROS_INFO("system id:%d id_index:%d",dxl.second,id_cnt-1);
  }

  uint8_t index_array[dynamixel_.size()];
  uint8_t cnt = 0;

  id_cnt = 0;
  for (auto const& name:dxl_name)
  {
    // 参考轨迹文件的顺序：2 3 4 0 1
    id_array[id_cnt++] = (uint8_t)dynamixel_[name];
  }
  for (auto const& name:dxl_name)
  {
    uint8_t index = 0;
    for (;index<id_cnt;index++)
    {
      if(id_array2[index]==(uint8_t)dynamixel_[name])
      {
        // ROS_INFO("file id:%d index:%d",dynamixel_[name],index);
        break;
      }
    }
    index_array[cnt++]=index;
  }

  result = dxl_wb_->syncRead(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                               id_array,
                               dxl_name.size(),
                               &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
  }
  WayPoint wp;

  result = dxl_wb_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                    id_array,
                                    id_cnt,
                                    control_items_["Present_Position"]->address,
                                    control_items_["Present_Position"]->data_length,
                                    get_position,
                                    &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
  }
  else
  {
    for(uint8_t index = 0; index < id_cnt; index++)
    {
      // ROS_INFO("present_position:%d,id:%d,id_index:%d",get_position[index],id_array[index],index_array[index]);
      wp.position = dxl_wb_->convertValue2Radian(id_array[index], get_position[index]-init_pos[index_array[index]]);
      wp.position = 0.0;
      
      // ROS_INFO("init position:%f id:%d",wp.position,id_array[index]);
      pre_goal_.push_back(wp);
    }
  }
  return result;
}

void MyDynamixelController::trajectoryMsgCallback(const trajectory_msgs::JointTrajectory::ConstPtr &msg)
{
  ROS_INFO("receive msg!");
  uint8_t id_cnt = 0;
  bool result = false;
  WayPoint wp;
  if(is_moving_ == false)
  {
    jnt_tra_msg_->joint_names.clear();
    jnt_tra_msg_->points.clear();
    pre_goal_.clear();

    result = getPresentPosition(msg->joint_names);
    if (result == false)
      ROS_ERROR("Failed to get Present Position");

    for (auto const& joint:msg->joint_names)
    {
      ROS_INFO("'%s' is ready to move", joint.c_str());
      jnt_tra_msg_->joint_names.push_back(joint);
      id_cnt++;
    }

    // tightenRope();

    if (id_cnt != 0)
    {
      uint8_t cnt = 0;
      // 遍历轨迹上的点
      while(cnt < msg->points.size())
      {
        std::vector<WayPoint> goal;
        //遍历一个点上的所有舵机位置
        for (std::vector<int>::size_type id_num = 0; id_num < msg->points[cnt].positions.size(); id_num++)
        {
          wp.position = msg->points[cnt].positions.at(id_num);

          if (msg->points[cnt].velocities.size() != 0)  wp.velocity = msg->points[cnt].velocities.at(id_num);
          else wp.velocity = 0.0f;

          if (msg->points[cnt].accelerations.size() != 0)  wp.acceleration = msg->points[cnt].accelerations.at(id_num);
          else wp.acceleration = 0.0f;

          goal.push_back(wp);
        }
        // size的值 对应要控制的舵机的数量
        jnt_tra_->setJointNum((uint8_t)msg->points[cnt].positions.size());
        // 获取运动的时长，时长在msg中获取
        double move_time = 0.0f;
        if (cnt == 0) move_time = msg->points[cnt].time_from_start.toSec();
        else move_time = msg->points[cnt].time_from_start.toSec() - msg->points[cnt-1].time_from_start.toSec();
        // pre_goal 表示上一个终点的位置，生成点需要初始点和重点，然后在中间处根据写的频率进行插值
        jnt_tra_->init(move_time,
                        write_period_,
                        pre_goal_,
                        goal);

        std::vector<WayPoint> way_point;
        trajectory_msgs::JointTrajectoryPoint jnt_tra_point_msg;
        // 从trajectory中得到目标按写频率分布的轨迹，存放到jnt_tra_point_msg中
        for (double index = 0.0; index < move_time; index = index + write_period_)
        {
          way_point = jnt_tra_->getJointWayPoint(index);

          for (uint8_t id_num = 0; id_num < id_cnt; id_num++)
          {
            jnt_tra_point_msg.positions.push_back(way_point[id_num].position);
            jnt_tra_point_msg.velocities.push_back(way_point[id_num].velocity);
            jnt_tra_point_msg.accelerations.push_back(way_point[id_num].acceleration);
          }

          jnt_tra_msg_->points.push_back(jnt_tra_point_msg);
          jnt_tra_point_msg.positions.clear();
          jnt_tra_point_msg.velocities.clear();
          jnt_tra_point_msg.accelerations.clear();
        }

        pre_goal_ = goal;
        cnt++;
      } 

      ROS_INFO("Succeeded to get joint trajectory!");
      is_moving_ = true;
    }
    else
    {
      ROS_WARN("Please check joint_name");
    }
  }
  else
  {
    ROS_WARN("Dynamixel is moving");
  }

}
//define  publisher
void MyDynamixelController::initPublisher()
{
  dynamixel_state_list_pub_ = priv_node_handle_.advertise<dynamixel_workbench_msgs::DynamixelStateList>("dynamixel_state", 100);
  desired_tra_pub_ = priv_node_handle_.advertise<my_dynamixel_workbench_test::desired_trajectory>("desired_tra",100);
}

//自定义service callback
void MyDynamixelController::initServer()
{
  changeGoalPositonSrv = node_handle_.advertiseService("changePosition",&MyDynamixelController::changePositionCallback,this);
  changePIDGainSrv = node_handle_.advertiseService("changePidGain",&MyDynamixelController::changePIDGain,this);
  changeGoalCurrentSrv = node_handle_.advertiseService("changeCurrent",&MyDynamixelController::changeGoalCurrentCallback,this);
}
bool MyDynamixelController::changePositionCallback
    (my_dynamixel_workbench_test::ChangeGoalPosition::Request &req
      ,my_dynamixel_workbench_test::ChangeGoalPosition::Response &res){
    uint8_t id_cnt = 0;
    uint8_t index = 0;
    for (auto const& dxl:dynamixel_)
    {
      if(req.id==(uint8_t)dxl.second)
      {
          index = id_cnt;
      }
      id_cnt++;
    }
    ROS_INFO("request:set id:%d  goalPosition:%d",req.id,req.goal_position);
    setGoalPosition(req.goal_position);
    goal_pos[index] = req.goal_position;
    res.result=true;
    for (auto const& dxl:dynamixel_)
    {
      dxl_wb_->torqueOff((uint8_t)dxl.second);
    }
    return true;
}
bool MyDynamixelController::changePIDGain
    (my_dynamixel_workbench_test::ChangePIDGain::Request &req
      ,my_dynamixel_workbench_test::ChangePIDGain::Response &res)
{
    ROS_INFO("set pidGain:   p_gain:%f   i_gain:%f   d_gain:%f",req.p_gain,req.i_gain,req.d_gain);
    position_err = 0;
    last_position_err = 0;
    err_integral = 0;
    for(int i=0;i<dynamixel_.size();i++)
    {
      pos_err[i]=0;
      last_pos_err[i]=0;
      pos_err_integral[i]=0;
    }
    for (auto const& dxl:dynamixel_)
    {
      dxl_wb_->torqueOn((uint8_t)dxl.second);
    }
    setPidGain(req.p_gain,req.i_gain,req.d_gain);
    res.result=true;
    return true;
}
bool MyDynamixelController::changeGoalCurrentCallback
  (my_dynamixel_workbench_test::ChangeGoalCurrent::Request &req
    ,my_dynamixel_workbench_test::ChangeGoalCurrent::Response &res){
    uint8_t id_cnt = 0;
    uint8_t index = 0;
    if(req.id!=10)
    {
      for (auto const& dxl:dynamixel_)
      {
        if(req.id==(uint8_t)dxl.second)
        {
            index = id_cnt;
        }
        id_cnt++;
      }
      ROS_INFO("request:set id:%d  goalCurrent:%d",req.id,req.goal_current);
      goal_cur[index] = req.goal_current;
    }
    else
    {
      for (auto const& dxl:dynamixel_)
      {
        goal_cur[id_cnt] = req.goal_current;

        id_cnt++;
      }
      ROS_INFO("change demo goalCurrent:%d",req.goal_current);
    }
    res.result=true;
    return true;
}


//ros timer callback
void MyDynamixelController::writeCallback(const ros::TimerEvent&){
  bool result = false;
  const char* log = NULL;

  uint8_t id_array[dynamixel_.size()];
  uint8_t id_cnt = 0;

  int32_t dynamixel_current[dynamixel_.size()];
  int32_t dynamixel_position[dynamixel_.size()];//应该是舵机的目标点的位置

  static uint32_t point_cnt = 0;
  static uint32_t position_cnt = 0;

  if (is_moving_==true)
  {
    // 存储序号 循环names
    // 循环id_array index 0:2
    // id_array[index](1 2 0)==(uint8_t)dynamixel_[joint](0 1 2)
    // true break;
    // cnt记录的是输入轨迹中定义的舵机的数量
    // index_array[cnt++]=index (2 0 1)
    // for cnt
    // index_cnt =0 gol_pos(index_arry[index_cnt]=2)=
    for (auto const& dxl:dynamixel_)
    {
      id_array[id_cnt++] = (uint8_t)dxl.second;
    }
    uint8_t index_array[dynamixel_.size()];
    uint8_t cnt = 0;
    for (auto const& joint:jnt_tra_msg_->joint_names)
    {
      uint8_t index = 0;
      for (;index<id_cnt;index++)
      {
        if(id_array[index]==(uint8_t)dynamixel_[joint])
        {
          break;
        }
      }
      index_array[cnt++]=index;
    }
    for (uint8_t index = 0; index < cnt; index++)
    {
      //如果输入的是弧度制先转换成整数值,默认的是弧度制
      dynamixel_position[index] = dxl_wb_->convertRadian2Value(id_array[index], jnt_tra_msg_->points[point_cnt].positions.at(index));
      // ROS_INFO("id:%d ,goal position:%d  index:%d",id_array[index],dynamixel_position[index],index);
      goal_pos[index_array[index]]=dynamixel_position[index]-2048;
      // ROS_INFO("id:%d ,goal position:%d  index:%d",id_array[index],goal_pos[index_array[index]],index);
      // ROS_INFO("id:%d,goal radian:%f",id_array[index],jnt_tra_msg_->points[point_cnt].positions.at(index));
      position_cnt++;
    }    
    //写入目标位置
    /*
    result = dxl_wb_->syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_POSITION, id_array, id_cnt, dynamixel_position, 1, &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
    }
    */
    //如果遍历了所有点的话，表明结束轨迹，不再运动
    //position_cnt++;
    //jnt_tra_msg_->points[point_cnt].positions.size() 表示第point_cnt个点，舵机的数量
    if (position_cnt >= jnt_tra_msg_->points[point_cnt].positions.size())
    {
      point_cnt++;
      position_cnt = 0;
      //ROS_INFO("the %d th point",point_cnt);
      // jnt_tra_msg_->points.size() 表示路径上点的总数
      if (point_cnt >= jnt_tra_msg_->points.size())
      {
        //ROS_INFO("totoal points size:%d",jnt_tra_msg_->points.size());
        is_moving_ = false;
        point_cnt = 0;
        position_cnt = 0;
        ROS_INFO("Complete Execution");
      }
    }
  }
  else{
      for (auto const& dxl:dynamixel_)
    {
      id_array[id_cnt++] = (uint8_t)dxl.second;//1 2 0
    }
  }


/*

*/
  d_tra.id.clear();
  d_tra.goal_position.clear();
  for (uint8_t index = 0; index < id_cnt; index++)
  {
      //d_tra->id[index]=id_array[index];
      // 如果是在运动的状态：id_arry=[0 1 2],goal_position按顺序对应的是 0 1 2
      // 如果是运动结束状态：id_arry=[1 2 0],goal_position按顺序对应的是 0 1 2 ...
      //，这时候给goal_position赋值的顺序应该变成 1 2 0 
      // 如果是在运动的状态：id_arry=[10 11 22],goal_position按顺序对应的是 0 1 2
      //如果是运动结束状态：id_arry=[11 22 10],goal_position按顺序对应的是 1 2 0 ...
      // 需要保证id_arry的顺序和goal_position的顺序一致
      // 但是最后写的话，goal_position顺序是没变的，但是id_arry的顺序改变了，所以导致最后位置改变了
      d_tra.id.push_back(id_array[index]);
      d_tra.goal_position.push_back(goal_pos[index]);
      //d_tra->goal_position[index]=goal_pos[index];
      dynamixel_current[index] = pidController(goal_pos[index],id_array[index]);
      // dynamixel_current[index] = goal_cur[index]; //进行示教
      // ROS_INFO("id:%d ,goal position:%d  index:%d",id_array[index],goal_pos[index],index);
      // ROS_INFO("id:%d ,goal_current:%d index:%d",id_array[index],dynamixel_current[index],index);
  }
  //写入目标电流
  result = dxl_wb_->syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_CURRENT, id_array, id_cnt, dynamixel_current, 1, &log);
  //result = true;
  if (result == false)
  {
    ROS_ERROR("%s", log);
  }
}

void MyDynamixelController::readCallback(const ros::TimerEvent&){
  bool result = false;
  const char* log = NULL;

  dynamixel_workbench_msgs::DynamixelState  dynamixel_state[dynamixel_.size()];
  dynamixel_state_list_.dynamixel_state.clear();

  int32_t get_current[dynamixel_.size()];
  int32_t get_velocity[dynamixel_.size()];
  int32_t get_position[dynamixel_.size()];

  uint8_t id_array[dynamixel_.size()];//这个什么作用还不清楚，对应的是id的排序 
  uint8_t id_cnt = 0;
  // 遍历 dynamixel_，id_cnt 对应数组的序号，？？如果放在定时器里的话应该是定时读取，有必要吗？
  for (auto const& dxl:dynamixel_)
  {
    dynamixel_state[id_cnt].name = dxl.first;
    dynamixel_state[id_cnt].id = (uint8_t)dxl.second;

    id_array[id_cnt++] = (uint8_t)dxl.second;
  }
  
  for (uint8_t i = 0;i < id_cnt;i++)
  {
    float radian = 0.0;
    dxl_wb_->getRadian(id_array[i],&radian);
    //ROS_INFO("dynamixel id: %d present_radian:%f",id_array[i],radian);
  }

  /*
  // 输出数据,感觉可以学习官方的方法
  for (uint8_t i = 0;i < id_cnt;i++)
  {
      dxl_wb_->itemRead(id_array[i],"Present_Position",&get_position[i]);
    //  ROS_INFO("dynamixel id: %d   present_radian:%f",id_array[i], dxl_wb_->getRadian(id_array[i],&radian));
    // ;
      dxl_wb_->itemRead(id_array[i],"Present_Velocity",&get_velocity[i]);//得到的是整数
    //  velocity= dxl_wb_->convertValue2Velocity(ID,velocity_data);  
      dxl_wb_->itemRead(id_array[i],"Present_Current",&get_current[i]);
    //  current = dxl_wb_->convertValue2Current(ID,current_data);
  }*/
    result = dxl_wb_->syncRead(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                              id_array,
                              dynamixel_.size(),
                              &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
  }

  result = dxl_wb_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                                id_array,
                                                id_cnt,
                                                control_items_["Present_Current"]->address,
                                                control_items_["Present_Current"]->data_length,
                                                get_current,
                                                &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
  }

  result = dxl_wb_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                                id_array,
                                                id_cnt,
                                                control_items_["Present_Velocity"]->address,
                                                control_items_["Present_Velocity"]->data_length,
                                                get_velocity,
                                                &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
  }

  result = dxl_wb_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                                id_array,
                                                id_cnt,
                                                control_items_["Present_Position"]->address,
                                                control_items_["Present_Position"]->data_length,
                                                get_position,
                                                &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
  }

  for(uint8_t index = 0; index < id_cnt; index++)
  {
    dynamixel_state[index].present_current = get_current[index];
    dynamixel_state[index].present_velocity = get_velocity[index];
    dynamixel_state[index].present_position = get_position[index];
    dynamixel_state_list_.dynamixel_state.push_back(dynamixel_state[index]);
    //ROS_INFO("id:%d present_position:%d  index:%d",id_array[index],dynamixel_state_list_.dynamixel_state[index].present_position,index);
  }
  
}

void MyDynamixelController::publishCallback(const ros::TimerEvent&)
{
  //uint8_t id_array[dynamixel_.size()];
  //uint8_t id_cnt = 0;
  //for (auto const& dxl:dynamixel_)
  //{
  //  id_array[id_cnt++] = (uint8_t)dxl.second;
  //}
  desired_tra_pub_.publish(d_tra);
  dynamixel_state_list_pub_.publish(dynamixel_state_list_);
}

// load trajectory
void MyDynamixelController::initGoalPos(){
  // // case 1:人为设定初始值
  // uint8_t id_array[dynamixel_.size()];
  // uint8_t id_cnt = 0;
  // for (auto const& dxl:dynamixel_)
  // {
  //   id_array[id_cnt++] = (uint8_t)dxl.second;//1 2 0
  // }

  // for (uint8_t index = 0; index < id_cnt; index++)
  // {
  //   // 初始化目标位置
  //   goal_pos[index] = 2048;
  // }

  // case 2:设定开机状态的位置值为初始值
  // 先使绳线张紧
  tightenRope();
  bool result = false;
  const char* log = NULL;

  dynamixel_workbench_msgs::DynamixelState  dynamixel_state[dynamixel_.size()];

  int32_t get_position[dynamixel_.size()];
  uint8_t id_array[dynamixel_.size()];
  uint8_t id_cnt = 0;

  for (auto const& dxl:dynamixel_)
  {
    dynamixel_state[id_cnt].name = dxl.first;
    dynamixel_state[id_cnt].id = (uint8_t)dxl.second;

    id_array[id_cnt++] = (uint8_t)dxl.second;
  }

  result = dxl_wb_->syncRead(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                              id_array,
                              dynamixel_.size(),
                              &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
  }

  result = dxl_wb_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                                id_array,
                                                id_cnt,
                                                control_items_["Present_Position"]->address,
                                                control_items_["Present_Position"]->data_length,
                                                get_position,
                                                &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
  }

  for(uint8_t index = 0; index < id_cnt; index++)
  {
    
    init_pos[index] = get_position[index];
    goal_pos[index] = 0;
    ROS_INFO("id:%d present_position:%d  index:%d",id_array[index],get_position[index],index);
  }



}

bool MyDynamixelController::getTrajectoryInfo(const std::string yaml_file, trajectory_msgs::JointTrajectory *jnt_tra_msg)
{
  YAML::Node file;
  file = YAML::LoadFile(yaml_file.c_str());

  if (file == NULL)
    return false;

  //
  YAML::Node joint = file["joint"];
  uint8_t joint_size = joint["names"].size();

  for (uint8_t index = 0; index < joint_size; index++)
  {
    std::string joint_name = joint["names"][index].as<std::string>();
    jnt_tra_msg->joint_names.push_back(joint["names"][index].as<std::string>());
  }

  //
  YAML::Node trajectory = file["trajectory"];
  uint8_t trajectory_size = trajectory["index"].size();
  for (uint8_t index = 0;index<trajectory_size;index++)
  {
      trajectory_msgs::JointTrajectoryPoint jnt_tra_point;
      std::string index_ = trajectory["index"][index].as<std::string>();
      YAML::Node wp_num = trajectory[index_];
      //ROS_INFO("joint_size:%d  pos size:%d",joint_size,wp_num["pos"].size());
      if (joint_size != wp_num["pos"].size())
      {
        ROS_ERROR("Please check way point pos size. It must be equal to joint size");
        return 0;
      }
      for (uint8_t size=0;size<wp_num["pos"].size();size++)
      {
        jnt_tra_point.positions.push_back(wp_num["pos"][size].as<double>());
        // 这个显示的格式感觉有点问题
        ROS_INFO("NO.way point : %s, position : %f", index_.c_str(), wp_num["pos"][size].as<double>());
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
void MyDynamixelController::TrajectoryInfoInit(){
  std::string yaml_file = node_handle_.param<std::string>("trajectory_info", "");
  bool result = getTrajectoryInfo(yaml_file, jnt_tra_test_);
  if (result == false)
  {
    ROS_ERROR("Please check YAML file");
    exit(0);
  }
  generatoTra(jnt_tra_test_);
  //is_moving_=true;
}
void MyDynamixelController::generatoTra(trajectory_msgs::JointTrajectory *msg)
{
  uint8_t id_cnt = 0;
  bool result = false;
  WayPoint wp;
  if(is_moving_ == false)
  { 
    jnt_tra_msg_->joint_names.clear();
    jnt_tra_msg_->points.clear();
    pre_goal_.clear();

    result = getPresentPosition(msg->joint_names);
    if (result == false)
      ROS_ERROR("Failed to get Present Position");

   // /*
    for (auto const& joint:msg->joint_names)
    {
      ROS_INFO("'%s' is ready to move", joint.c_str());
      jnt_tra_msg_->joint_names.push_back(joint);
      id_cnt++;
    }

    if (id_cnt != 0)
    {
      uint8_t cnt = 0;
      int wpNum = 0;
      // 遍历轨迹上的点
      while(cnt < msg->points.size())
      {
        std::vector<WayPoint> goal;
        //遍历一个点上的所有舵机位置
        for (std::vector<int>::size_type id_num = 0; id_num < msg->points[cnt].positions.size(); id_num++)
        {
          wp.position = msg->points[cnt].positions.at(id_num);

          if (msg->points[cnt].velocities.size() != 0)  wp.velocity = msg->points[cnt].velocities.at(id_num);
          else wp.velocity = 0.0f;

          if (msg->points[cnt].accelerations.size() != 0)  wp.acceleration = msg->points[cnt].accelerations.at(id_num);
          else wp.acceleration = 0.0f;

          goal.push_back(wp);
        }
        // size的值 对应要控制的舵机的数量
        jnt_tra_->setJointNum((uint8_t)msg->points[cnt].positions.size());
        // 获取运动的时长，时长在msg中获取
        double move_time = 0.0f;
        if (cnt == 0) move_time = msg->points[cnt].time_from_start.toSec();
        else move_time = msg->points[cnt].time_from_start.toSec() - msg->points[cnt-1].time_from_start.toSec();
        // pre_goal 表示起点，生成点需要初始点和终点，然后在中间处根据写的频率进行插值

        ROS_INFO("move_time:%f",move_time);
        jnt_tra_->init(move_time,
                        write_period_,
                        pre_goal_,
                        goal);

        std::vector<WayPoint> way_point;
        trajectory_msgs::JointTrajectoryPoint jnt_tra_point_msg;
        // 从trajectory中得到目标按写频率分布的轨迹，存放到jnt_tra_point_msg中 index表示点个序号
        for (double index = 0.0; index < move_time; index = index + write_period_)
        {
          way_point = jnt_tra_->getJointWayPoint(index);
          
          //给每隔舵机赋值
          for (uint8_t id_num = 0; id_num < id_cnt; id_num++)
          {
            jnt_tra_point_msg.positions.push_back(way_point[id_num].position);
            jnt_tra_point_msg.velocities.push_back(way_point[id_num].velocity);
            jnt_tra_point_msg.accelerations.push_back(way_point[id_num].acceleration);
          }

          jnt_tra_msg_->points.push_back(jnt_tra_point_msg);
          jnt_tra_point_msg.positions.clear();
          jnt_tra_point_msg.velocities.clear();
          jnt_tra_point_msg.accelerations.clear();
          wpNum++;
        }

        pre_goal_ = goal;
        cnt++;
      }
      ROS_INFO("way point sum up :%d ",wpNum);
      ROS_INFO("Succeeded to get joint trajectory!");
      is_moving_ = true;
    }
    else
    {
      ROS_WARN("Please check joint_name");
    }
  // */
  }
  else
  {
    ROS_WARN("Dynamixel is moving");
  }
}

// pid controller function
void MyDynamixelController::pidControllerInit(){
  pos_err = new int[dynamixel_.size()];
  last_pos_err = new int[dynamixel_.size()];
  pos_err_integral = new int[dynamixel_.size()];
  goal_pos = new int[dynamixel_.size()];
  init_pos = new int[dynamixel_.size()];
  goal_cur = new int16_t[dynamixel_.size()];
  for (int i=0;i<dynamixel_.size();i++)
  {
    pos_err[i] = 0 ;
    last_pos_err[i] = 0 ;
    pos_err_integral[i] = 0;

    // 这里初始化回零了，应该以读出来的位置作为初始值
    goal_pos[i] = 0;
    goal_cur[i] = demo_current;
  }
  ROS_INFO("P Gain: %f I Gain:%f  D Gain: %f",p_gain,i_gain,d_gain);
}
bool MyDynamixelController::getGoalPosition(int & position){
  position = goal_position;
  return true;
}
bool MyDynamixelController::setGoalPosition(int position){
  goal_position = position;
  return true;
}
bool MyDynamixelController::setPidGain(float p,float i,float d){
  p_gain = p;
  i_gain = i;
  d_gain = d;
  return true;
}
bool MyDynamixelController::setLimitCurrent(int lim_cur)
{
  limit_current = lim_cur;
  return true;
}
int MyDynamixelController::pidController(int present_position){
  int goal_current = 0;
  last_position_err = position_err;
  position_err = goal_position - present_position;
  err_integral += position_err;
  //ROS_INFO("p_gain:%f    goal_position=%d,",p_gain,goal_position);

  goal_current = (int) (p_gain * position_err+ i_gain * err_integral +d_gain * (position_err-last_position_err));
  if(goal_current>limit_current)
  {
    goal_current = limit_current;
  }
  if(goal_current<-limit_current)
  {
    goal_current = -limit_current;
  }
  return goal_current;
}
int MyDynamixelController::pidController(int goal_position_ , int id)
{
  //这个读舵机当前位置的方法有些问题 已知id，从id中找index读？
  uint8_t id_array[dynamixel_.size()];
  uint8_t id_cnt = 0;
  int index = 0;
  for (auto const& dxl:dynamixel_)
  {
    id_array[id_cnt++] = (uint8_t)dxl.second;
  }
  for(int i=0;i<id_cnt;i++)
  {
    if(id_array[i]==id)
    {
      index = i;
    }
  }
  int32_t present_position = dynamixel_state_list_.dynamixel_state[index].present_position;
  present_position = present_position - init_pos[index];
  int16_t goal_current = 0;
  last_pos_err[index] = pos_err[index];
  pos_err[index] = goal_position_ - present_position ;
  pos_err_integral[index] += pos_err[index];
  goal_current = (int16_t) (p_gain * pos_err[index]+ i_gain * pos_err_integral[index] +d_gain * (pos_err[index]-last_pos_err[index])); 
  //ROS_INFO("id=%d  present_pos=%d  pos_err=%d index:%d ",id, present_position, pos_err[index],index);
  if(goal_current>limit_current)
  {
    goal_current = limit_current;
  }
  if(goal_current<-limit_current)
  {
    goal_current = -limit_current;
  }

  return goal_current;
}

void MyDynamixelController::endTest()
{
  for (auto const& dxl:dynamixel_)
  {
    dxl_wb_->torqueOff((uint8_t)dxl.second);
  }
  ROS_INFO("end test! Good luck!");
}

void MyDynamixelController::tightenRope()
{
// 先给每个舵机小电流，延时一段时间，放松
bool result = false;
const char* log = NULL;
int32_t dynamixel_current[dynamixel_.size()];
uint8_t id_array[dynamixel_.size()];
uint8_t id_cnt = 0;

for (auto const& dxl:dynamixel_)
{
  dynamixel_current[id_cnt] = -3;
  id_array[id_cnt++] = (uint8_t)dxl.second;
}

result = dxl_wb_->syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_CURRENT, id_array, id_cnt, dynamixel_current, 1, &log);
//result = true;
if (result == false)
{
  ROS_ERROR("%s", log);
}
usleep(5000*1000);

for(int i=0;i<id_cnt;i++)
{
  dynamixel_current[id_cnt] = 0;
}

result = dxl_wb_->syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_CURRENT, id_array, id_cnt, dynamixel_current, 1, &log);

ROS_INFO("Tighten ropes finish!");

}

// main function
//void recordDxlState2Txt(my_dynamixel_workbench_test::dxl_state state_msg);
int main(int argc,char ** argv)
{
    ros::init(argc,argv,"my_dxl_master");
    ros::NodeHandle node_handle;

    std::string port_name = "/dev/ttyUSB0";
    uint32_t baud_rate = 57600;

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

    MyDynamixelController dynamixel_controller;

    bool result = false;

    std::string yaml_file = node_handle.param<std::string>("dynamixel_info", "");

    result = dynamixel_controller.initWorkbench(port_name, baud_rate);
    if (result == false)
    {
        ROS_ERROR("Please check USB port name");
        return 0;
    }
    
    result = dynamixel_controller.getDynamixelsInfo(yaml_file);
    if (result == false)
    {
        ROS_ERROR("Please check YAML file");
        return 0;
    }

    result = dynamixel_controller.loadDynamixels();
    if (result == false)
    {
        ROS_ERROR("Please check Dynamixel ID or BaudRate");
        return 0;
    }

    result = dynamixel_controller.initDynamixels();
    if (result == false)
    {
        ROS_ERROR("Please check control table (http://emanual.robotis.com/#control-table)");
        return 0;
    }

    result = dynamixel_controller.initControlItems();
    if (result == false)
    {
        ROS_ERROR("Please check control items");
        return 0;
    }

    result = dynamixel_controller.initSDKHandlers();
    if (result == false)
    {
        ROS_ERROR("Failed to set Dynamixel SDK Handler");
        return 0;
    }
    dynamixel_controller.pidControllerInit();
    ROS_INFO("Welcome my dynamixel workbench!");

    DynamixelWorkbench * dxl_wb = dynamixel_controller.dxl_wb_;
    //ros topic publish
    ros::Publisher dxl_state_pub = node_handle.advertise<my_dynamixel_workbench_test::dxl_state>("dxl_state_topic",1);
    my_dynamixel_workbench_test::dxl_state state_msg;
     
    //ros service that you can change goal_position
    dynamixel_controller.initServer();

    int limit_current = 20;
    //ros::ServiceServer changeGoalPositonSrv = node_handle.advertiseService("changePosition",&changePositionCallback);
    dynamixel_controller.setLimitCurrent(limit_current);

   // /*
    dynamixel_controller.initSubscriber();
    dynamixel_controller.initPublisher();
    dynamixel_controller.initGoalPos();
    ROS_INFO("write_period:%f",dynamixel_controller.getWritePeriod());
    ros::Timer read_timer = node_handle.createTimer(ros::Duration(dynamixel_controller.getReadPeriod()),
                                           &MyDynamixelController::readCallback, &dynamixel_controller);
    ros::Timer write_timer = node_handle.createTimer(ros::Duration(dynamixel_controller.getWritePeriod()),
                                           &MyDynamixelController::writeCallback, &dynamixel_controller);
    ros::Timer publish_timer = node_handle.createTimer(ros::Duration(dynamixel_controller.getPublishPeriod()), 
                                          &MyDynamixelController::publishCallback, &dynamixel_controller);
    // dynamixel_controller.tightenRope();
    //dynamixel_controller.TrajectoryInfoInit();
    //sleep(5);
    ros::spin();
    dynamixel_controller.endTest();
}
/*
void recordDxlState2Txt(my_dynamixel_workbench_test::dxl_state state_msg){
  //fout1 << "stamp:" << state_msg.stamp << std::endl;
  ROS_INFO("record data!");
  fout1 << "radian:" << state_msg.present_radian <<std::endl;
  fout1 << "velocity:"<< state_msg.present_velocity <<std::endl;
  fout1 << "current:" << state_msg.present_current <<std::endl;
  fout1 << std::endl;
}*/
