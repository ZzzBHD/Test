#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
import cv2

#def talker():
#    pub_steer = rospy.Publisher('steer_cmd', Float64, queue_size=10)
#    pub_speed = rospy.Publisher('speed_cmd', Float64, queue_size=10)
rospy.init_node('kinect_py_testpub', anonymous=True)
#    rate = rospy.Rate(10) # 10hz
#    while not rospy.is_shutdown():
#        hello_str = "hello world %s" % rospy.get_time()
#        steer_cmd=3.33
#        speed_cmd=5.64
#        pub_speed.publish(speed_cmd)
#        pub_steer.publish(steer_cmd)
#        rate.sleep()

#if __name__ == '__main__':
#    try:
#        talker()
#    except rospy.ROSInterruptException:
#        pass
while 1:
    img=cv2.imread('/home/cyberc3/socket/3.png')
    cv2.imshow('test',img)
    if cv2.waitKey(20)==27:
        break

cv2.destroyAllWindows()
