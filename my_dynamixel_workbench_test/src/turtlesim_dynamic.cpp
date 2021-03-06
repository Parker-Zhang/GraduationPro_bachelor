#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<dynamic_reconfigure/server.h>
#include<my_dynamixel_workbench_test/dynamicConfig.h>

using namespace std;

// the name of topic
string cmd_topic = "/turtle1/cmd_vel";
int loop_rate = 1;
double linear_x = 1.0;
double angular_z = 1.0;
bool move_flag = true;
int log_level = 0;

void dynamic_callback(turtlesim_dynamic::dynamicConfig &config)
{
    ROS_INFO("Reconfigure Request: %s %d %f %f %s %d",
            config.cmd_topic.c_str(),
            config.cmd_pub_rate,
            config.linear_x,
            config.angular_z,
            config.move_flag?"True":"False",
            config.log_level);
 
    cmd_topic = config.cmd_topic;
    loop_rate = config.cmd_pub_rate;
    linear_x  = config.linear_x;
    angular_z = config.angular_z;
    move_flag = config.move_flag;
    log_level = config.log_level;
}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"turtlesim_dynamic_node");
    ros::NodeHandle handle;

    dynamic_reconfigure::Server<turtlesim_dynamic::dynamicConfig> server;
    dynamic_reconfigure::Server<turtlesim_dynamic::dynamicConfig>::CallbackType callback;
 
    callback = boost::bind(&dynamic_callback, _1);
    server.setCallback(callback);
 
    ros::Publisher cmdVelPub;
    geometry_msgs::Twist speed;
 
    while(ros::ok())
    {
        cmdVelPub = handle.advertise<geometry_msgs::Twist>(cmd_topic, 1);
        ros::Rate rate(loop_rate);
        if(move_flag)
        {
            speed.linear.x  = linear_x;
            speed.angular.z = angular_z;
            cmdVelPub.publish(speed);
 
            switch(log_level)
            {
                case 0:
                    ROS_INFO("log level: INFO, cmd_pub_rate: %d", loop_rate);
                    break;
                case 1:
                    ROS_WARN("log level: WARN, cmd_pub_rate: %d", loop_rate);
                    break;
                case 2:
                    ROS_ERROR("log level: ERROR, cmd_pub_rate: %d", loop_rate);
                    break;
                default:
                    break;
            }
        }
        ros::spinOnce();
        rate.sleep();
    }

/*
    ros::param::get("~cmd_topic",cmd_topic);
    ros::param::get("~loop_rate", loop_rate);
    ros::param::get("~linear_x", linear_x);
    ros::param::get("~angular_z", angular_z);
    ros::param::get("~move_flag", move_flag);

    ros::Publisher cmdVelPub = handle.advertise<geometry_msgs::Twist>(cmd_topic,1);
    geometry_msgs::Twist speed;

    ros::Rate rate(loop_rate);
    while(ros::ok())
    {
        if(move_flag)
        {
            speed.linear.x = linear_x;
            speed.angular.z = angular_z;
            cmdVelPub.publish(speed);
        }
        ros::spinOnce();
        rate.sleep();
    }
*/
}