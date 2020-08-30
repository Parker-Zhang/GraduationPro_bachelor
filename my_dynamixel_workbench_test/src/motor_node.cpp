#include<ros/ros.h>
#include <dynamixel_workbench_msgs/DynamixelStateList.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

int main(int argc,char **argv)
{
    ros::init(argc,argv,"motor_node");
    ros::NodeHandle n;
    ros::Publisher goal_pos_pub;
    ros::Publisher state_pub;
    ros::Subscriber dxl_state_sub;

}
