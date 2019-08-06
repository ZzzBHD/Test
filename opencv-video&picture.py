#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import os

vc=cv2.VideoCapture('/home/cyberc3/Kinect-test/室外离线测试（开云车）.mp4')
fps=int(vc.get(cv2.CAP_PROP_FPS))
width=int(vc.get(cv2.CAP_PROP_FRAME_WIDTH))
height=int(vc.get(cv2.CAP_PROP_FRAME_HEIGHT))
frame_number=int(vc.get(cv2.CAP_PROP_FRAME_COUNT))
print "fps=",fps,',width=',width,'height=',height,',frame number=',frame_number,'\n'
c=1
if vc.isOpened():
	rval,frame=vc.read()
else:
	rval=false
while rval:
	rval,frame=vc.read()
	cv2.imwrite('/home/cyberc3/img_k/'+str(c)+'.jpg',frame)
	c=c+1
	
	cv2.waitKey(1)
vc.release


# fps=10
# width=1890
# height=1018
# frame_number=526
# out=cv2.VideoWriter('out.avi',cv2.VideoWriter_fourcc('M','J','P','G'),fps,(width,height))
# i=1
# for i in range(1,frame_number):
# 	img=cv2.imread(filename='/home/cyberc3/Kinect-test/img/'+str(i)+'.jpg')
# 	cv2.waitKey(1)
# 	out.write(img)
# out.release()
