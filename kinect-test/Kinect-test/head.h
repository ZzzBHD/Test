#pragma once
#ifndef HEAD
#define HEAD
#include <Kinect.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/*
修改：
code是否需要用到？//目前没有用到，但可以保留
指针的去extern（包括sensor，source，reader等）//已修改,已解决
变量的去extern（包括colorHeight, colorWidth，bodycount）//已修改，已解决
*/


using namespace cv;

//extern IKinectSensor   * mySensor;
//
//extern IColorFrameSource   * myColorSource;
//extern IColorFrame * myColorFrame;
//extern IColorFrameReader   * myColorReader;
//
//extern IBodyFrameSource    * myBodySource;
//extern IBodyFrame  * myBodyFrame;
//extern IBodyFrameReader    * myBodyReader;
//extern IFrameDescription   * myDescription;

extern ICoordinateMapper   * myMapper;
//extern int myBodyCount;
extern int colorHeight, colorWidth;//彩色帧的高度和宽度(1902*1080)
extern bool GetGesture;//识别到人体的flag

extern Joint myJointArr[JointType_Count];//读取到的人体骨骼节点数据
extern Joint myJointArrcopy[JointType_Count];//保存的上x帧的人体骨骼节点数据，当前x=1

extern const int*code;//读到的那个人体编号，且不能修改，目前还没有用到

void Initialization();//初始化函数
void Release();//释放函数
void DrawMethod(Mat & img, Joint & r_1, Joint & r_2, ICoordinateMapper * myMapper);
void DrawBodypoints(Mat & img, ICoordinateMapper * myMapper);
void ColorSign(Mat colorimg);

#endif