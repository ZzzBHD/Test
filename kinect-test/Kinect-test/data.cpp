
#include "data.h"
#include <cstdlib>
#include <cmath>

/*
  唯一指令者的人体编号
  读取选择节点的x，y，z，jointtype的信息，并存入txt中
*/

int myJointFramenum = 0;
int CutFrameCount = 0;//小于阈值时的计数器
int TrickFameCount = 0;//大于阈值时的计数器

Joint* p[OrderLength];//保存在vector的指向每一帧人体骨骼节点的数据指针
vector<Joint*>myJointData_ALL;//保存手势指令节点数据的vector

/*
选择需要保存在模板数据中的joint节点列表
注意在修改是需要同步修改vector下指针new出空间的大小，见SaveJointData函数
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

const JointType myJointCalname(int n)//选择需要计算的joint节点列表
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
保存人体骨骼节点函数
输入当前帧的骨骼节点数据数组myJointarr
将这一帧的数据保存在一个vector中，vector的每一个元素是指向一帧的Joint数据数组的指针
myJointArr是原始读取的数据，包括了全部25个骨骼节点的所有数据
myJointFrame是当前帧的标号
使用时，必须同步使用Realease函数
*/
void SaveJointData(Joint myJointArr[])
{
	if (GetGesture)
	{
		p[myJointFramenum] = new Joint[13];//容量是选择的需要保存的骨骼节点的数量
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
WriteData负责读取人体骨骼节点位置数据，并将一帧的数据写入文件中
*/
void WriteData(vector<Joint*> myJointData_ALL,std::ofstream & fout)
{
	if (GetGesture)
	{
		if (fout.is_open())//判断是否打开文件
		{
			for (int j = 0; j < myJointFramenum; j++)
			{
				for (int i = 0; i < 13; i++)//容量是选择的需要保存的骨骼节点的数量
				{
					fout << myJointData_ALL[j][i].JointType << ","
						<< myJointData_ALL[j][i].Position.X << ","
						<< myJointData_ALL[j][i].Position.Y << ","
						<<myJointData_ALL[j][i].Position.Z
						<<std::endl;
				}
			}	
			ReleasePoint(p, myJointFramenum);//释放已保存好的数据
		}
		else//未成功打开文件
		{
			std::cerr << "Can't open the file!" << std::endl;
			fout.close();
			Release();
			exit(EXIT_FAILURE);
		}
	}
}

/*
分帧计算函数，计算前后两帧之间骨骼节点差距，并返回一个bool值
若前后两帧之间差距很小，即目标未有大的动作，则返回false
若前后两帧之间差距很大，即目标有大的动作，则返回true
传入前后两帧joint数据，并计算“帧间移动距离”
*/
bool Cal_Frame(Joint first_joint[],Joint second_joint[])
{
	double Cal_value=0;
	for (int i = 0; i < 13; i++)//总数是25，目前只取其中几个，详细筛选见myJointCalname
	{
		JointType wantJoint = myJointCalname(myJointArr[i].JointType);
		if (wantJoint != JointType_Count)
		{
			Cal_value += sqrt(pow((first_joint[i].Position.X*10 - second_joint[i].Position.X*10), 2)
				+ pow((first_joint[i].Position.Y*10 - second_joint[i].Position.Y*10), 2));
		}
	}
	if (Cal_value*10 > 10.0)//尚需要修改
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
（训练用）手势指令分帧函数，负责从视频流中提取包含手势指令的部分
调用Cal_Frame计算“帧间移动距离”，若大于阈值，则保存到数据文件中，直到再次分帧
具体判断过程见文件
TrickFameCount的阈值为手势指令的最短长度
CutFrameCount的阈值为可作为手势指令的最长容忍静止长度
*/
/*void GetmyGesture_Train(bool & gesture_start, bool & CalFrameflag, Joint  myJointArr[], vector<myJointtype*> & myJointData_ALL)
{
	if (!gesture_start)
	{
		if (CalFrameflag) gesture_start = TRUE;//进入录入状态
	}
	else
	{
		if (CalFrameflag)//继续录入
		{
			TrickFameCount++;
			CutFrameCount = 0;
			SaveJointData(myJointArr);
		}
		else//短暂出现小范围移动，开始计数，并继续录入
		{
			CutFrameCount++;
			if (CutFrameCount <= 6)
			{
				SaveJointData(myJointArr);
			}
			else
			{
				if (TrickFameCount >= 6)//完成手势指令的提取
				{					
					TrickFameCount = 0;
					CutFrameCount = 0;
					WriteData(myJointData_ALL);//把正常完整的指令写到文件中
					myJointData_ALL.clear();				
					std::cout << "Get Complete" << std::endl;
					fout << "Get Gesture:" << myJointFramenum << std::endl << std::endl;//输出读取的帧数
					myJointFramenum = 0;
					gesture_start = FALSE;
				}
				else//手势动作过短，舍弃//但前面已经录到文件中的没办法做些事情了，待处理
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
			gesture_start = TRUE;//进入录入状态
			return FALSE;
		}
		return FALSE;
	}
	else
	{
		if (CalFrameflag)//继续录入
		{
			TrickFameCount++;
			CutFrameCount = 0;
			SaveJointData(myJointArr);
			return FALSE;
		}
		else//短暂出现小范围移动，开始计数，并继续录入
		{
			CutFrameCount++;
			if (CutFrameCount <= 6)
			{
				SaveJointData(myJointArr);
				return FALSE;
			}
			else
			{
				if (TrickFameCount >= 6)//完成手势指令的提取
				{
					TrickFameCount = 0;
					CutFrameCount = 0;
					std::cout << "Get Complete" << std::endl;
					gesture_start = FALSE;
					return TRUE;
				}
				else//手势动作过短，舍弃//但前面已经录到文件中的没办法做些事情了，待处理
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
Joint拷贝函数，负责拷贝Joint骨骼数据，需要调用myJointname以选择需要复制的骨骼数据
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
释放所有new出的存放人体骨骼节点数据的myJointtype类型的数组
针对于训练时
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
