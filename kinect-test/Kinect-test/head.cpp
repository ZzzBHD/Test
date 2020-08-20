
#include "head.h"
#include "data.h"

/*
取colour和body两个源
读取sensor彩色图像及人体骨骼图像，并标记
copy当前读取到的人体骨骼数据joint
*/

IKinectSensor   * mySensor = nullptr;//sensor指针

IColorFrameSource   * myColorSource = nullptr;//彩色源
IColorFrame * myColorFrame = nullptr;//彩色帧指针
IColorFrameReader   * myColorReader = nullptr;//彩色帧读取指针

IBodyFrameSource    * myBodySource = nullptr;//身体源
IBodyFrame  * myBodyFrame = nullptr;//身体帧指针
IBodyFrameReader    * myBodyReader = nullptr;//身体帧读取指针

ICoordinateMapper   * myMapper = nullptr;//转换指针
IFrameDescription   * myDescription=nullptr;//帧描述指针

int colorHeight = 0, colorWidth = 0;//彩色帧的高度和宽度
int myBodyCount = 0;//人数

Joint  myJointArr[JointType_Count];
Joint myJointArrcopy[JointType_Count];

int me = 0;
const int*code = &me;
bool GetGesture = FALSE;
using namespace cv;


/*
采集彩色图像及body图像,同时得到身体数据数组
输入彩色图的Mat，同Mat实参返回彩色图像
*/
void ColorSign(Mat colorimg)
{
	while (myColorReader->AcquireLatestFrame(&myColorFrame) != S_OK);//读到彩色图
	myColorFrame->CopyConvertedFrameDataToArray(colorHeight * colorWidth * 4, colorimg.data, ColorImageFormat_Bgra);//读取彩色帧
	myColorFrame->Release();
}

/*
人体骨骼线画图函数
用两个关节点来做线段的两端，并且进行状态过滤
*/
void DrawMethod(Mat & img, Joint & r_1, Joint & r_2, ICoordinateMapper * myMapper)
{
	if (r_1.TrackingState == TrackingState_Tracked && r_2.TrackingState == TrackingState_Tracked)
	{
		ColorSpacePoint t_point;    //要把关节点用的摄像机坐标下的点转换成彩色空间的点
		Point   p_1, p_2;
		myMapper->MapCameraPointToColorSpace(r_1.Position, &t_point);
		p_1.x = t_point.X;
		//if(r_1.JointType== JointType_Head)
		//	std::cout << "gX:" << r_1.Position.X << std::endl << "cX" << p_1.x << std::endl;
		p_1.y = t_point.Y;
		myMapper->MapCameraPointToColorSpace(r_2.Position, &t_point);
		p_2.x = t_point.X;
		p_2.y = t_point.Y;
		line(img, p_1, p_2, Scalar(0, 255, 0), 5);//（蓝，绿，红）
		circle(img, p_1, 10, Scalar(255, 0, 0), -1);
		circle(img, p_2, 10, Scalar(255, 0, 0), -1);
	}
}

/*
画出所有的人体骨骼点
*/
void DrawBodypoints(Mat & img, ICoordinateMapper * myMapper)
{
	while (myBodyReader->AcquireLatestFrame(&myBodyFrame) != S_OK); //读取身体图像
	IBody   **  myBodyArr = new IBody *[myBodyCount];       //为存身体数据的数组做准备
	for (int i = 0; i < myBodyCount; i++)
		myBodyArr[i] = nullptr;
	if (myBodyFrame->GetAndRefreshBodyData(myBodyCount, myBodyArr) == S_OK)
	{
		for (int i = 0; i < myBodyCount; i++)
		{
			BOOLEAN  result = false;
			if (myBodyArr[i]->get_IsTracked(&result) == S_OK && result) //先判断是否侦测到
			{
				me = i;//读取到当前那个人的编号
				if (myBodyArr[i]->GetJoints(JointType_Count, myJointArr) == S_OK)
				{
					GetGesture = TRUE;//判断是否读取到人体骨骼数据

					DrawMethod(img, myJointArr[JointType_Head], myJointArr[JointType_Neck], myMapper);

					DrawMethod(img, myJointArr[JointType_Neck], myJointArr[JointType_SpineShoulder], myMapper);

					DrawMethod(img, myJointArr[JointType_SpineShoulder], myJointArr[JointType_ShoulderLeft], myMapper);
					DrawMethod(img, myJointArr[JointType_SpineShoulder], myJointArr[JointType_SpineMid], myMapper);
					DrawMethod(img, myJointArr[JointType_SpineShoulder], myJointArr[JointType_ShoulderRight], myMapper);

					DrawMethod(img, myJointArr[JointType_ShoulderLeft], myJointArr[JointType_ElbowLeft], myMapper);
					DrawMethod(img, myJointArr[JointType_SpineMid], myJointArr[JointType_SpineBase], myMapper);
					DrawMethod(img, myJointArr[JointType_ShoulderRight], myJointArr[JointType_ElbowRight], myMapper);

					DrawMethod(img, myJointArr[JointType_ElbowLeft], myJointArr[JointType_WristLeft], myMapper);

					//DrawMethod(img, myJointArr[JointType_SpineBase], myJointArr[JointType_HipLeft], myMapper);
				    //DrawMethod(img, myJointArr[JointType_SpineBase], myJointArr[JointType_HipRight], myMapper);
					
					DrawMethod(img, myJointArr[JointType_ElbowRight], myJointArr[JointType_WristRight], myMapper);
					//DrawMethod(img, myJointArr[JointType_WristLeft], myJointArr[JointType_ThumbLeft], myMapper);

					DrawMethod(img, myJointArr[JointType_WristLeft], myJointArr[JointType_HandLeft], myMapper);
					//DrawMethod(img, myJointArr[JointType_HipLeft], myJointArr[JointType_KneeLeft], myMapper);
					//DrawMethod(img, myJointArr[JointType_HipRight], myJointArr[JointType_KneeRight], myMapper);
					//DrawMethod(img, myJointArr[JointType_WristRight], myJointArr[JointType_ThumbRight], myMapper);
					DrawMethod(img, myJointArr[JointType_WristRight], myJointArr[JointType_HandRight], myMapper);

					//DrawMethod(img, myJointArr[JointType_HandLeft], myJointArr[JointType_HandTipLeft], myMapper);
					//DrawMethod(img, myJointArr[JointType_KneeLeft], myJointArr[JointType_FootLeft], myMapper);
					//DrawMethod(img, myJointArr[JointType_KneeRight], myJointArr[JointType_FootRight], myMapper);
					//DrawMethod(img, myJointArr[JointType_HandRight], myJointArr[JointType_HandTipRight], myMapper);
				}
			}
		}
	}
	myBodyFrame->Release();
}

/*
初始化函数
（1）打开sensor（2）打开source（3）打开reader（4）读取分辨率
*/
void Initialization()
{
	GetDefaultKinectSensor(&mySensor);
	mySensor->Open();//打开kinect
	/*获取彩色图像及分辨率*/
	mySensor->get_ColorFrameSource(&myColorSource);//获取彩色数据
	myColorSource->OpenReader(&myColorReader);//读取彩色图像
	myColorSource->get_FrameDescription(&myDescription);
	myDescription->get_Height(&colorHeight);
	myDescription->get_Width(&colorWidth);//取得彩色图像的分辨率
	/*获取骨骼图像等信息*/
	mySensor->get_BodyFrameSource(&myBodySource);
	myBodySource->OpenReader(&myBodyReader);
	myBodySource->get_BodyCount(&myBodyCount);

	mySensor->get_CoordinateMapper(&myMapper);//坐标转换
/*
	if(GetDefaultKinectSensor(&mySensor)!=S_OK)
		std::cerr<< "Get Sensor failed" << std::endl;
	if (mySensor->Open() != S_OK)
		std::cerr << "Can't get frame source" << std::endl;
	if (mySensor->get_ColorFrameSource(&myColorSource) != S_OK)
		std::cerr << "Can't get color frame source" << std::endl;
	if(myColorSource->OpenReader(&myColorReader)!=S_OK)
		std::cerr << "Can't open color frame reader" << std::endl;
	if (myColorSource->get_FrameDescription(&myDescription) == S_OK)
	{
		myDescription->get_Height(&colorHeight);
		myDescription->get_Width(&colorWidth);
	}
	if(mySensor->get_BodyFrameSource(&myBodySource)!=S_OK)
		std::cerr << "Can't get body frame source" << std::endl;
	if(myBodySource->OpenReader(&myBodyReader)!=S_OK)
		std::cerr << "Can't open body frame reader" << std::endl;
	if(myBodySource->get_BodyCount(&myBodyCount)!=S_OK)
		std::cerr << "Can't get body count" << std::endl;
*/
}

/*
释放函数
*/
void Release()
{
	myMapper->Release();
	myDescription->Release();
	myColorReader->Release();
	myColorSource->Release();
	myBodyReader->Release();
	myBodySource->Release();
	mySensor->Close();
	mySensor->Release();
}