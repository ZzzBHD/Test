
#include "head.h"
#include "data.h"

/*
ȡcolour��body����Դ
��ȡsensor��ɫͼ���������ͼ�񣬲����
copy��ǰ��ȡ���������������joint
*/

IKinectSensor   * mySensor = nullptr;//sensorָ��

IColorFrameSource   * myColorSource = nullptr;//��ɫԴ
IColorFrame * myColorFrame = nullptr;//��ɫָ֡��
IColorFrameReader   * myColorReader = nullptr;//��ɫ֡��ȡָ��

IBodyFrameSource    * myBodySource = nullptr;//����Դ
IBodyFrame  * myBodyFrame = nullptr;//����ָ֡��
IBodyFrameReader    * myBodyReader = nullptr;//����֡��ȡָ��

ICoordinateMapper   * myMapper = nullptr;//ת��ָ��
IFrameDescription   * myDescription=nullptr;//֡����ָ��

int colorHeight = 0, colorWidth = 0;//��ɫ֡�ĸ߶ȺͿ��
int myBodyCount = 0;//����

Joint  myJointArr[JointType_Count];
Joint myJointArrcopy[JointType_Count];

int me = 0;
const int*code = &me;
bool GetGesture = FALSE;
using namespace cv;


/*
�ɼ���ɫͼ��bodyͼ��,ͬʱ�õ�������������
�����ɫͼ��Mat��ͬMatʵ�η��ز�ɫͼ��
*/
void ColorSign(Mat colorimg)
{
	while (myColorReader->AcquireLatestFrame(&myColorFrame) != S_OK);//������ɫͼ
	myColorFrame->CopyConvertedFrameDataToArray(colorHeight * colorWidth * 4, colorimg.data, ColorImageFormat_Bgra);//��ȡ��ɫ֡
	myColorFrame->Release();
}

/*
��������߻�ͼ����
�������ؽڵ������߶ε����ˣ����ҽ���״̬����
*/
void DrawMethod(Mat & img, Joint & r_1, Joint & r_2, ICoordinateMapper * myMapper)
{
	if (r_1.TrackingState == TrackingState_Tracked && r_2.TrackingState == TrackingState_Tracked)
	{
		ColorSpacePoint t_point;    //Ҫ�ѹؽڵ��õ�����������µĵ�ת���ɲ�ɫ�ռ�ĵ�
		Point   p_1, p_2;
		myMapper->MapCameraPointToColorSpace(r_1.Position, &t_point);
		p_1.x = t_point.X;
		//if(r_1.JointType== JointType_Head)
		//	std::cout << "gX:" << r_1.Position.X << std::endl << "cX" << p_1.x << std::endl;
		p_1.y = t_point.Y;
		myMapper->MapCameraPointToColorSpace(r_2.Position, &t_point);
		p_2.x = t_point.X;
		p_2.y = t_point.Y;
		line(img, p_1, p_2, Scalar(0, 255, 0), 5);//�������̣��죩
		circle(img, p_1, 10, Scalar(255, 0, 0), -1);
		circle(img, p_2, 10, Scalar(255, 0, 0), -1);
	}
}

/*
�������е����������
*/
void DrawBodypoints(Mat & img, ICoordinateMapper * myMapper)
{
	while (myBodyReader->AcquireLatestFrame(&myBodyFrame) != S_OK); //��ȡ����ͼ��
	IBody   **  myBodyArr = new IBody *[myBodyCount];       //Ϊ���������ݵ�������׼��
	for (int i = 0; i < myBodyCount; i++)
		myBodyArr[i] = nullptr;
	if (myBodyFrame->GetAndRefreshBodyData(myBodyCount, myBodyArr) == S_OK)
	{
		for (int i = 0; i < myBodyCount; i++)
		{
			BOOLEAN  result = false;
			if (myBodyArr[i]->get_IsTracked(&result) == S_OK && result) //���ж��Ƿ���⵽
			{
				me = i;//��ȡ����ǰ�Ǹ��˵ı��
				if (myBodyArr[i]->GetJoints(JointType_Count, myJointArr) == S_OK)
				{
					GetGesture = TRUE;//�ж��Ƿ��ȡ�������������

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
��ʼ������
��1����sensor��2����source��3����reader��4����ȡ�ֱ���
*/
void Initialization()
{
	GetDefaultKinectSensor(&mySensor);
	mySensor->Open();//��kinect
	/*��ȡ��ɫͼ�񼰷ֱ���*/
	mySensor->get_ColorFrameSource(&myColorSource);//��ȡ��ɫ����
	myColorSource->OpenReader(&myColorReader);//��ȡ��ɫͼ��
	myColorSource->get_FrameDescription(&myDescription);
	myDescription->get_Height(&colorHeight);
	myDescription->get_Width(&colorWidth);//ȡ�ò�ɫͼ��ķֱ���
	/*��ȡ����ͼ�����Ϣ*/
	mySensor->get_BodyFrameSource(&myBodySource);
	myBodySource->OpenReader(&myBodyReader);
	myBodySource->get_BodyCount(&myBodyCount);

	mySensor->get_CoordinateMapper(&myMapper);//����ת��
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
�ͷź���
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