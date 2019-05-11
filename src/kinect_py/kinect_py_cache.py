#!/usr/bin/env python
# coding: utf-8
#这是存放所有缓存的（无用或者失败的代码）
import rospy
rospy.init_node('kinect_py_socket', anonymous=True)
import cv2
import socket
import copy
import numpy as np
import tensorflow as tf
import matplotlib.pyplot as plt

PORT=6000
BUFFER_SIZE=800*500*3
#imgFix = np.zeros((300, 500, 3), np.uint8)

server_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM,0)
hostname = '0.0.0.0' #设置主机名
port = 6000  #设置端口号
addr = (hostname,port)

server_socket.bind(addr)
server_socket.listen(5)
print("Listening......\n")
client_socket,clinet_address = server_socket.accept()
print("Accept!\n")

while True:
    try:
        client_socket.settimeout(5)
        buf = client_socket.recv(BUFFER_SIZE,socket.MSG_WAITALL)
        print("recv data!\n")
#        print(str.shape)
        if buf==0:
            client_socket.close()
            print("buff==0!")
            break
#        plt.imshow(buf)
        with tf.Session() as sess:
#            buf=tf.image.decode_jpeg(buf)
            image=sess.run(buf)
            plt.imshow(image)
#        data=np.fromstring(buf,dtype='uint8')
#        img=cv2.imdecode(data,1)
#        img=cv2.fromarray(data)
#        cv2.imshow("test",data)
#        if cv2.waitKey(10)==27:
#            break

    except socket.timeout :
        print 'time out'


server_socket.close()
#cv2.destroyAllWindows()







