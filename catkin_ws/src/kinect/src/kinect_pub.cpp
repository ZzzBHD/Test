#include "kinect_head.h"
#include "kinect_pub.h"

void pub_up( ros::Publisher kinect_whell_pub,std_msgs::Float64 wheel_output,ros::Publisher kinect_speed_pub,std_msgs::Int32MultiArray speed_output,ros::Publisher test_pub,std_msgs::Float64 test_output,int count)
{
    wheel_output.data=0;
    test_output.data=count;
    speed_output.data.push_back (2100);
    speed_output.data.push_back (1);
    kinect_whell_pub.publish(wheel_output);
    kinect_speed_pub.publish(speed_output);
    test_pub.publish(test_output);
}

void pub_up( ros::Publisher kinect_whell_pub,std_msgs::Float64 wheel_output,ros::Publisher kinect_speed_pub,std_msgs::Int32MultiArray speed_output)
{
    wheel_output.data=0;
    speed_output.data.push_back (2100);
    speed_output.data.push_back (1);
    kinect_whell_pub.publish(wheel_output);
    kinect_speed_pub.publish(speed_output);
}

void pub_stop( ros::Publisher kinect_whell_pub,std_msgs::Float64 wheel_output,ros::Publisher kinect_speed_pub,std_msgs::Int32MultiArray speed_output)
{
    wheel_output.data=0;
    speed_output.data.push_back (1700);
    speed_output.data.push_back (80);
    kinect_whell_pub.publish(wheel_output);
    kinect_speed_pub.publish(speed_output);
}

void pub_left( ros::Publisher kinect_whell_pub,std_msgs::Float64 wheel_output,ros::Publisher kinect_speed_pub,std_msgs::Int32MultiArray speed_output)
{
    wheel_output.data=150;
    speed_output.data.push_back (2100);
    speed_output.data.push_back (1);
    kinect_whell_pub.publish(wheel_output);
    kinect_speed_pub.publish(speed_output);
}

void pub_right( ros::Publisher kinect_whell_pub,std_msgs::Float64 wheel_output,ros::Publisher kinect_speed_pub,std_msgs::Int32MultiArray speed_output)
{
    wheel_output.data=-150;
    speed_output.data.push_back (2100);
    speed_output.data.push_back (1);
    kinect_whell_pub.publish(wheel_output);
    kinect_speed_pub.publish(speed_output);
}






