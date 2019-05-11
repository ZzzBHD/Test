#include "kinect_socket.h"
#include "kinect_head.h"
#include "kinect_opencv.h"

#define DEFAULT_PORT 6000
#define DEFAULT_PORTs 5000
#define IMG_HEIGHT 400
#define IMG_LENGTH 200
#define BUFFER_SIZE sizeof(unsigned char)*IMG_HEIGHT*IMG_LENGTH*3
#define BUFFER_SIZES 4

using namespace std;
using namespace cv;
static int client_socket=0;
char Buffers[BUFFER_SIZES];
void WinbackCallback(const std_msgs::Int32::ConstPtr& msg)
{
    sprintf (Buffers,"%d",msg->data);
    send(client_socket,Buffers,4,0);
    ROS_INFO("send msg = %d", msg->data);
}


int main(int argc,char * argv[])
{
    ros::init (argc,argv,"kinect_socket_pub");
    ros::NodeHandle nh;
    ros::Publisher kinect_socket_pub=nh.advertise <std_msgs::Int32>("kinect_rgb_flag",100);
    ros::Subscriber kinect_winback_pub=nh.subscribe <std_msgs::Int32>("kinect_winback_flag",100,WinbackCallback);

    Mat frame(IMG_HEIGHT,IMG_LENGTH,CV_8UC3);
    Mat cc(IMG_HEIGHT,IMG_LENGTH,CV_8UC3);
    resize(frame,frame,Size(IMG_HEIGHT,IMG_LENGTH));
    bool img_flag=0;
    vector<int> para;
    para.push_back (CV_IMWRITE_PNG_COMPRESSION);
    para.push_back (5);


    struct sockaddr_in server_addr;
    char Buffer[BUFFER_SIZE];
    bzero(Buffer,BUFFER_SIZE);

    bzero(Buffers,BUFFER_SIZES);
    struct sockaddr_in client_addr;
    socklen_t length=sizeof(client_addr);
    bzero(Buffer,BUFFER_SIZE);

//    int client_socket=0;
    int server_socket = 0;

    std_msgs::Int32 rgb_flag;

//    int soc_flag;
//    soc_flag=fcntl(client_socket,F_GETFL,0);
//    fcntl(client_socket,F_SETFL,soc_flag|O_NONBLOCK);

    if((server_socket = socket(AF_INET,SOCK_STREAM,0))== -1)
    {
        perror("create socket is failure\n");
        exit(1);
    }
    std::cout<<"Creat Socket Succeed!"<<std::endl;
    bzero(&server_addr, sizeof(server_addr)); // 置字节字符串前n个字节为0，包括\0
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(DEFAULT_PORT);

    if(bind(server_socket, (struct sockaddr*)&server_addr,sizeof(server_addr))==-1)
    {
        printf("bind socket error: %s(errno: %d)\n",strerror(errno),errno);
        exit(1);
    }
    std::cout<<"Bind Succeed!"<<std::endl;

    if(listen(server_socket,10)== - 1)
    {
        perror("listen");
        exit(1);
    }
    std::cout<<"Listening......\n"<<std::endl;
    while(ros::ok())
    {

        if((client_socket=accept(server_socket,(struct sockaddr*)&client_addr,&length))==-1)
        {
            printf("Server Accept Failed!\n");
            continue;
        }
        else
        {
            printf("Server Accept Message!\n");
            break;
        }
    }
    while(1)
    {
        bzero(Buffer,BUFFER_SIZE);
        if (img_flag)
            imwrite ("/home/cyberc3/socket/4.png",frame,para);
//        int ret=recv(client_socket,Buffer,BUFFER_SIZE,MSG_NOSIGNAL|MSG_DONTWAIT);
        int ret=recv(client_socket,Buffer,BUFFER_SIZE,MSG_NOSIGNAL|MSG_WAITALL);
        ros::spinOnce ();

        if(ret<0)
        {
            printf("accept socket error: %s(errno: %d)",strerror(errno),errno);
            printf("Server Recieve Data Failed!\n");
            img_flag=0;
            break;
        }
        else if(ret>0)
        {
//            printf("accept message\n");
//            std::cout<<Buffer<<std::endl;
            memcpy (frame.data,Buffer,BUFFER_SIZE);
            imwrite ("/home/cyberc3/socket/3.png",frame,para);
            if(img_flag)
            {
                rgb_flag.data=img_flag;
                kinect_socket_pub.publish(rgb_flag);
            }
            img_flag=1;
            usleep(10000);
//            resize(frame,cc,Size(800,500));
//            imshow("t",cc);
//            waitKey (1);
        }
        else
        {
            img_flag=0;
            rgb_flag.data=img_flag;
            kinect_socket_pub.publish(rgb_flag);
            break;
        }

    }
    close(client_socket);
    close(server_socket);
    return 0;
}
