#pragma once
#include "head.h"
#include <cstring>
#include <fstream>

/*
修改：
fout，fin,doc在main中声明，并作为参数传入函数，减少extern的数量//已删除fin,已修改doc,已修改fout
*w_num 与 &write_num是否可以删去，用myJointFramnum代替，需要确认//已删除
保存节点数据的p指针new开出的大小（目前为13）是否增加接口，又传入参数决定?//目前没什么特别好的办法
WriteData增加fout的接口//已修改，已解决
分帧函数训练用和实际识别用需要重新分开写//已修改，待测试，测试后记得同步到match工程中，已解决
读入的数据中myJointtype中的Tracksign是否需要用于回显中？//放在kinect_match中解决
myJointData_ALL和指针的去extern？//无法去除
把需要的Jointname的switch放宽到extern，并且在画骨骼节点图的时候调用//画图的时候选择无用，放弃
*/
#define OrderLength 200//定义手势指令容许的最大帧长度

extern vector<Joint*>myJointData_ALL;//保存的vector
extern Joint* p[OrderLength];
extern int myJointFramenum;

bool Cal_Frame(Joint first_joint[], Joint second_joint[]);
bool GetmyGesture_Train(bool & gesture_start, bool & CalFrameflag, Joint  myJointArr[], vector<Joint*> & myJointData_ALL);
void CopyJoint(Joint model[], Joint copy[]);
void SaveJointData(Joint myJointarr[]);
void WriteData(vector<Joint*> myJointData_ALL,std::ofstream & fout);
void ReleasePoint(Joint* p[], int num);
/*	
	case    0:return  JointType_SpineBase; break;
	case    1:return  JointType_SpineMid; break;
	case    2:return  JointType_Neck; break;
	case    3:return  JointType_Head; break;
	case    4:return  JointType_ShoulderLeft; break;
	case    5:return  JointType_ElbowLeft; break;
	case    6:return  JointType_WristLeft; break;
	case    7:return  JointType_HandLeft; break;
	case    8:return  JointType_ShoulderRight; break;
	case    9:return  JointType_ElbowRight; break;
	case    10:return  JointType_WristRight; break;
	case    11:return  JointType_HandRight; break;
		//case    12:return  JointType_HipLeft; break;
		//case    16:return  JointType_HipRight; break;
	case    20:return  JointType_SpineShoulder; break;
	default:return JointType_Count;

}*/










