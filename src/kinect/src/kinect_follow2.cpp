#include "kinect_head.h"
#include "kinect_pub.h"
#include <algorithm>

using namespace std;

static vector<int>bounding_x;
static vector<int>bounding_size;
static bool bounding_flag=false;

void BoxCallback(const std_msgs::Int32MultiArray::ConstPtr& msg)
{
    if(msg->data.size ()!=0)
    {
        for(std::vector<int>::const_iterator i = msg->data.begin(); i != msg->data.end(); i+=2)
//        for(std::vector<int>::const_iterator i = msg->data.begin(); i != msg->data.end(); i+=1)
        {
            bounding_size.push_back (*i);
            bounding_x.push_back (*(i+1));
        }
        bounding_flag=true;
    }
}

int main(int argc,char * argv[])
{
    int line_diff;//当前距离与安全距离之间的差值
    int x_diff;//当前位置与中间位置之间的差值
    int maxsize=100;
    int x_avr;
    int x_sum=0;
    int person_num=0;
    int stop_flag=0;

    ros::init (argc,argv,"kinect_follow_calculator");
    ros::NodeHandle nh;
    ros::Publisher kinect_steer_pub=nh.advertise <std_msgs::Float64>("steer_cmd",50);
    ros::Publisher kinect_speed_pub=nh.advertise <std_msgs::Float64>("speed_target",50);
    ros::Subscriber kinect_bounding_pub=nh.subscribe <std_msgs::Int32MultiArray>("kinect_bounding_box",50,BoxCallback);
    ros::Publisher kinect_follow_pub=nh.advertise<std_msgs::Int32>("kinect_winback_flag",10);
    std_msgs::Int32 winback_flag;
    std_msgs::Float64 wheel_output;
    std_msgs::Float64 speed_cmd;

    printf("FOLLOW init\n");
    while(ros::ok ())
    {
        ros::spinOnce ();
        if(bounding_flag)
        {
            if((bounding_x[0]!=0)&&(bounding_size[0]!=0))
//            if(bounding_size[0]!=0)
            {
                person_num=bounding_size.size();
                if(person_num!=0)
                {
                    for(int i=0;i<person_num;i++)
                    {
                        maxsize=max(maxsize,bounding_size[i]);
                        x_sum+=bounding_x[i];
                    }
                    x_avr=x_sum/person_num;
                    line_diff=1500-maxsize;//SAFELINE!!!!!!
                    x_diff=x_avr-160;//X_AVR!!!!!!!!
                                     //大于0的就是right，小于0的就是left
                    cout<<"x_avr:"<<x_avr<<"    maxsize:"<<maxsize<<endl
                       <<"line_diff:"<<line_diff<<"    x_diff:"<<x_diff<<endl;
                    if(line_diff<0)//进入安全距离以内
                    {
                        stop_flag++;
                        speed_cmd.data=0;
                        kinect_speed_pub.publish(speed_cmd);
                        cout<<"stop_flag:"<<stop_flag<<endl;
                        if(stop_flag>5)//待修改（目标处于安全区以内的时间间隔判定）
                        {
                            winback_flag.data=1;
                            kinect_follow_pub.publish(winback_flag);
                            stop_flag=0;
                        }
                    }
                    else
                    {
                        stop_flag=0;
                        speed_cmd.data=0.001*(float)line_diff+0.05;
                        speed_cmd.data=min(speed_cmd.data,1.3);//最大值限定在5km/h
                        kinect_speed_pub.publish(speed_cmd);
                        if(abs(x_diff)>20)
                        {
                            wheel_output.data=x_diff;
                            kinect_steer_pub.publish(wheel_output);
                        }
                    }
                }
            }
            else
                ros::shutdown ();
        }
        bounding_x.clear ();
        bounding_size.clear ();
        x_sum=0;
        maxsize=100;
        bounding_flag=false;
    }
    printf("FOLLOW node close...\n");
    return 0;
}
