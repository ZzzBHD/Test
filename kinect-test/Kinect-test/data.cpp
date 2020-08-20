
#include "data.h"
#include <cstdlib>
#include <cmath>

/*
  Ψһָ���ߵ�������
  ��ȡѡ��ڵ��x��y��z��jointtype����Ϣ��������txt��
*/

int myJointFramenum = 0;
int CutFrameCount = 0;//С����ֵʱ�ļ�����
int TrickFameCount = 0;//������ֵʱ�ļ�����

Joint* p[OrderLength];//������vector��ָ��ÿһ֡��������ڵ������ָ��
vector<Joint*>myJointData_ALL;//��������ָ��ڵ����ݵ�vector

/*
ѡ����Ҫ������ģ�������е�joint�ڵ��б�
ע�����޸�����Ҫͬ���޸�vector��ָ��new���ռ�Ĵ�С����SaveJointData����
*/
const JointType myJointname(int n)
{
	switch (n)
	{
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
	}
}

const JointType myJointCalname(int n)//ѡ����Ҫ�����joint�ڵ��б�
{
	switch (n)
	{
		//case    0:return  JointType_SpineBase; break;
		//case    1:return  JointType_SpineMid; break;
		//case    2:return  JointType_Neck; break;
		//case    3:return  JointType_Head; break;
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
		//case    20:return  JointType_SpineShoulder; break;
	default:return JointType_Count;
	}
}

/*
������������ڵ㺯��
���뵱ǰ֡�Ĺ����ڵ���������myJointarr
����һ֡�����ݱ�����һ��vector�У�vector��ÿһ��Ԫ����ָ��һ֡��Joint���������ָ��
myJointArr��ԭʼ��ȡ�����ݣ�������ȫ��25�������ڵ����������
myJointFrame�ǵ�ǰ֡�ı��
ʹ��ʱ������ͬ��ʹ��Realease����
*/
void SaveJointData(Joint myJointArr[])
{
	if (GetGesture)
	{
		p[myJointFramenum] = new Joint[13];//������ѡ�����Ҫ����Ĺ����ڵ������
		int j = 0;
		for (int i = 0; i < 25; i++)
		{
			JointType wantJoint = myJointname(myJointArr[i].JointType);
			if (wantJoint != JointType_Count)
			{
				if (myJointArr[i].TrackingState != TrackingState_NotTracked)
				{
					p[myJointFramenum][j].JointType = myJointArr[i].JointType;
					p[myJointFramenum][j].TrackingState = myJointArr[i].TrackingState;
					p[myJointFramenum][j].Position.X = myJointArr[i].Position.X;
					p[myJointFramenum][j].Position.Y = myJointArr[i].Position.Y;
					p[myJointFramenum][j].Position.Z = myJointArr[i].Position.Z;
				}
				else
				{
					p[myJointFramenum][j].JointType = myJointArr[i].JointType;
					p[myJointFramenum][j].TrackingState = TrackingState_Inferred;
					p[myJointFramenum][j].Position.X = 0;
					p[myJointFramenum][j].Position.Y = 0;
					p[myJointFramenum][j].Position.Z = 0;
				}
				j++;
			}
		}
	}
	myJointData_ALL.push_back(p[myJointFramenum]);
	myJointFramenum++;
}

/*
WriteData�����ȡ��������ڵ�λ�����ݣ�����һ֡������д���ļ���
*/
void WriteData(vector<Joint*> myJointData_ALL,std::ofstream & fout)
{
	if (GetGesture)
	{
		if (fout.is_open())//�ж��Ƿ���ļ�
		{
			for (int j = 0; j < myJointFramenum; j++)
			{
				for (int i = 0; i < 13; i++)//������ѡ�����Ҫ����Ĺ����ڵ������
				{
					fout << myJointData_ALL[j][i].JointType << ","
						<< myJointData_ALL[j][i].Position.X << ","
						<< myJointData_ALL[j][i].Position.Y << ","
						<<myJointData_ALL[j][i].Position.Z
						<<std::endl;
				}
			}	
			ReleasePoint(p, myJointFramenum);//�ͷ��ѱ���õ�����
		}
		else//δ�ɹ����ļ�
		{
			std::cerr << "Can't open the file!" << std::endl;
			fout.close();
			Release();
			exit(EXIT_FAILURE);
		}
	}
}

/*
��֡���㺯��������ǰ����֮֡������ڵ��࣬������һ��boolֵ
��ǰ����֮֡�����С����Ŀ��δ�д�Ķ������򷵻�false
��ǰ����֮֡����ܴ󣬼�Ŀ���д�Ķ������򷵻�true
����ǰ����֡joint���ݣ������㡰֡���ƶ����롱
*/
bool Cal_Frame(Joint first_joint[],Joint second_joint[])
{
	double Cal_value=0;
	for (int i = 0; i < 13; i++)//������25��Ŀǰֻȡ���м�������ϸɸѡ��myJointCalname
	{
		JointType wantJoint = myJointCalname(myJointArr[i].JointType);
		if (wantJoint != JointType_Count)
		{
			Cal_value += sqrt(pow((first_joint[i].Position.X*10 - second_joint[i].Position.X*10), 2)
				+ pow((first_joint[i].Position.Y*10 - second_joint[i].Position.Y*10), 2));
		}
	}
	if (Cal_value*10 > 10.0)//����Ҫ�޸�
	{
		std::cout << Cal_value*10 << std::endl;
		return TRUE;
	}
	else
	{
		std::cout << Cal_value*10 << std::endl;
		return FALSE;
	}
}

/*
��ѵ���ã�����ָ���֡�������������Ƶ������ȡ��������ָ��Ĳ���
����Cal_Frame���㡰֡���ƶ����롱����������ֵ���򱣴浽�����ļ��У�ֱ���ٴη�֡
�����жϹ��̼��ļ�
TrickFameCount����ֵΪ����ָ�����̳���
CutFrameCount����ֵΪ����Ϊ����ָ�������̾�ֹ����
*/
/*void GetmyGesture_Train(bool & gesture_start, bool & CalFrameflag, Joint  myJointArr[], vector<myJointtype*> & myJointData_ALL)
{
	if (!gesture_start)
	{
		if (CalFrameflag) gesture_start = TRUE;//����¼��״̬
	}
	else
	{
		if (CalFrameflag)//����¼��
		{
			TrickFameCount++;
			CutFrameCount = 0;
			SaveJointData(myJointArr);
		}
		else//���ݳ���С��Χ�ƶ�����ʼ������������¼��
		{
			CutFrameCount++;
			if (CutFrameCount <= 6)
			{
				SaveJointData(myJointArr);
			}
			else
			{
				if (TrickFameCount >= 6)//�������ָ�����ȡ
				{					
					TrickFameCount = 0;
					CutFrameCount = 0;
					WriteData(myJointData_ALL);//������������ָ��д���ļ���
					myJointData_ALL.clear();				
					std::cout << "Get Complete" << std::endl;
					fout << "Get Gesture:" << myJointFramenum << std::endl << std::endl;//�����ȡ��֡��
					myJointFramenum = 0;
					gesture_start = FALSE;
				}
				else//���ƶ������̣�����//��ǰ���Ѿ�¼���ļ��е�û�취��Щ�����ˣ�������
				{
					myJointData_ALL.clear();
					myJointFramenum = 0;
					TrickFameCount = 0;
					CutFrameCount = 0;
					gesture_start = FALSE;
					std::cout << "Empty Data" << std::endl;
					
				}
			}
		}

	}
}
*/
bool GetmyGesture_Train(bool & gesture_start, bool & CalFrameflag, Joint  myJointArr[], vector<Joint*> & myJointData_ALL)
{
	if (!gesture_start)
	{
		if (CalFrameflag)
		{
			gesture_start = TRUE;//����¼��״̬
			return FALSE;
		}
		return FALSE;
	}
	else
	{
		if (CalFrameflag)//����¼��
		{
			TrickFameCount++;
			CutFrameCount = 0;
			SaveJointData(myJointArr);
			return FALSE;
		}
		else//���ݳ���С��Χ�ƶ�����ʼ������������¼��
		{
			CutFrameCount++;
			if (CutFrameCount <= 6)
			{
				SaveJointData(myJointArr);
				return FALSE;
			}
			else
			{
				if (TrickFameCount >= 6)//�������ָ�����ȡ
				{
					TrickFameCount = 0;
					CutFrameCount = 0;
					std::cout << "Get Complete" << std::endl;
					gesture_start = FALSE;
					return TRUE;
				}
				else//���ƶ������̣�����//��ǰ���Ѿ�¼���ļ��е�û�취��Щ�����ˣ�������
				{
					myJointData_ALL.clear();
					myJointFramenum = 0;
					TrickFameCount = 0;
					CutFrameCount = 0;
					std::cout << "Empty Data" << std::endl;
					gesture_start = FALSE;
					return FALSE;

				}
			}
		}

	}
}

/*
Joint�������������𿽱�Joint�������ݣ���Ҫ����myJointname��ѡ����Ҫ���ƵĹ�������
*/
void CopyJoint(Joint model[],Joint copy[])
{
	for (int i = 0; i < 25; i++)
	{
		JointType wantJoint = myJointname(model[i].JointType);
		if (wantJoint != JointType_Count)
		{
			copy[i] = model[i];
		}
	}
}

/*
�ͷ�����new���Ĵ����������ڵ����ݵ�myJointtype���͵�����
�����ѵ��ʱ
*/
void ReleasePoint(Joint* p[],int num)
{
	if (num == 0);
	else
	{
		for (int i = 0; i < num; i++)
			delete[] p[i];
	}
}
