#!/usr/bin/env python
# coding: utf-8

#python端的主要功能是
#1 调用训练好的ssd-tensorflow模型
#2 通过socket接收来自windows端的rgb图像（可能还有深度图像）
#3 发布话题，话题名称自定，话题内容包括深度信息，是否是persion等，用自定义的msg

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32
from std_msgs.msg import Float64

def kinect_talker():
    pub_depth = rospy.Publisher('person_depth', Float64, queue_size=10)
    pub_flag = rospy.Publisher('person_flag', Int32, queue_size=10)
    rospy.init_node('kinect_py_pub', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        depth=3.33
        flag=1
        pub_depth.publish(depth)
        pub_flag.publish(flag)
        rate.sleep()

if __name__ == '__main__':
    try:
        kinect_talker()
    except rospy.ROSInterruptException:
        pass
