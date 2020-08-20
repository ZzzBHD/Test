#pragma once
#include "head.h"
#include <cstring>
#include <fstream>

/*
�޸ģ�
fout��fin,doc��main������������Ϊ�������뺯��������extern������//��ɾ��fin,���޸�doc,���޸�fout
*w_num �� &write_num�Ƿ����ɾȥ����myJointFramnum���棬��Ҫȷ��//��ɾ��
����ڵ����ݵ�pָ��new�����Ĵ�С��ĿǰΪ13���Ƿ����ӽӿڣ��ִ����������?//Ŀǰûʲô�ر�õİ취
WriteData����fout�Ľӿ�//���޸ģ��ѽ��
��֡����ѵ���ú�ʵ��ʶ������Ҫ���·ֿ�д//���޸ģ������ԣ����Ժ�ǵ�ͬ����match�����У��ѽ��
�����������myJointtype�е�Tracksign�Ƿ���Ҫ���ڻ����У�//����kinect_match�н��
myJointData_ALL��ָ���ȥextern��//�޷�ȥ��
����Ҫ��Jointname��switch�ſ�extern�������ڻ������ڵ�ͼ��ʱ�����//��ͼ��ʱ��ѡ�����ã�����
*/
#define OrderLength 200//��������ָ����������֡����

extern vector<Joint*>myJointData_ALL;//�����vector
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










