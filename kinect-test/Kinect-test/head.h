#pragma once
#ifndef HEAD
#define HEAD
#include <Kinect.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/*
�޸ģ�
code�Ƿ���Ҫ�õ���//Ŀǰû���õ��������Ա���
ָ���ȥextern������sensor��source��reader�ȣ�//���޸�,�ѽ��
������ȥextern������colorHeight, colorWidth��bodycount��//���޸ģ��ѽ��
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
extern int colorHeight, colorWidth;//��ɫ֡�ĸ߶ȺͿ��(1902*1080)
extern bool GetGesture;//ʶ�������flag

extern Joint myJointArr[JointType_Count];//��ȡ������������ڵ�����
extern Joint myJointArrcopy[JointType_Count];//�������x֡����������ڵ����ݣ���ǰx=1

extern const int*code;//�������Ǹ������ţ��Ҳ����޸ģ�Ŀǰ��û���õ�

void Initialization();//��ʼ������
void Release();//�ͷź���
void DrawMethod(Mat & img, Joint & r_1, Joint & r_2, ICoordinateMapper * myMapper);
void DrawBodypoints(Mat & img, ICoordinateMapper * myMapper);
void ColorSign(Mat colorimg);

#endif