#ifndef KINECT_PUB
#define KINECT_PUB

#include "kinect_head.h"

void pub_up( ros::Publisher kinect_whell_pub,std_msgs::Float64 wheel_output,ros::Publisher kinect_speed_pub,std_msgs::Int32MultiArray speed_output);
void pub_stop( ros::Publisher kinect_whell_pub,std_msgs::Float64 wheel_output,ros::Publisher kinect_speed_pub,std_msgs::Int32MultiArray speed_output);
void pub_left( ros::Publisher kinect_whell_pub,std_msgs::Float64 wheel_output,ros::Publisher kinect_speed_pub,std_msgs::Int32MultiArray speed_output);
void pub_right( ros::Publisher kinect_whell_pub,std_msgs::Float64 wheel_output,ros::Publisher kinect_speed_pub,std_msgs::Int32MultiArray speed_output);
void pub_up( ros::Publisher kinect_whell_pub,std_msgs::Float64 wheel_output,ros::Publisher kinect_speed_pub,std_msgs::Int32MultiArray speed_output,ros::Publisher test_pub,std_msgs::Float64 test_output,int count);

#endif
