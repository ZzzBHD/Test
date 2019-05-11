#include "../include/kinect_socket.h"
#include "../include/kinect_head.h"
#include "../include/kinect_pub.h"

#define DEFAULT_PORT 6000
#define BUFFER_SIZE 200
using namespace std;


int main(int argc,char * argv[])
{
    ros::init (argc,argv,"kinect_publisher");
    ros::NodeHandle nh;
    ros::Publisher kinect_whell_pub=nh.advertise <std_msgs::Float64>("steer_cmd",100);
    ros::Publisher kinect_speed_pub=nh.advertise <std_msgs::Int32MultiArray>("speed_cmd",100);
    ros::Publisher test_pub=nh.advertise <std_msgs::Float64>("test_cmd",100);
    std_msgs::Float64 wheel_output;
    std_msgs::Int32MultiArray speed_output;
    std_msgs::Float64 test_output;

    struct sockaddr_in server_addr;
    char Buffer[BUFFER_SIZE];
    bzero(Buffer,BUFFER_SIZE);
    struct sockaddr_in client_addr;
    socklen_t length=sizeof(client_addr);

    int client_socket=0;

    int soc_flag;
    soc_flag=fcntl(client_socket,F_GETFL,0);
    fcntl(client_socket,F_SETFL,soc_flag|O_NONBLOCK);

    int server_socket = 0;
    if((server_socket = socket(AF_INET,SOCK_STREAM,0))== -1)
    {
        perror("create socket is failure\n");
        exit(1);
    }
    printf ("Creat Socket Succeed!\n");
    bzero(&server_addr, sizeof(server_addr)); // 置字节字符串前n个字节为0，包括\0
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(DEFAULT_PORT);

    if(bind(server_socket, (struct sockaddr*)&server_addr,sizeof(server_addr))==-1)
    {
        printf("bind socket error: %s(errno: %d)\n",strerror(errno),errno);
        exit(1);
    }
    printf("Bind Succeed!\n");

    if(listen(server_socket,10)== - 1)
    {
        perror("listen");
        exit(1);
    }
    printf ("Listening......\n");
    while(1)
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
    int flag=0;
    ros::Rate loop_rate(1);
    while(1)
    {
        for(int i=0;i<1000;i++)
        {
            for(int j=0;j<3000;j++);
        }
        bzero(Buffer,BUFFER_SIZE);
//        int ret=recv(client_socket,Buffer,BUFFER_SIZE,MSG_NOSIGNAL);
        int ret=recv(client_socket,Buffer,BUFFER_SIZE,MSG_DONTWAIT);
//        ROS_INFO("send msg = %d", ret);
        if(ret<0)//非阻塞下no data返回-1
        {
//            printf("small than 0---->ret=%d,flag=%d,count=%d\n",ret,flag,count);
//            ROS_INFO("send msg = %d", flag);

            if(flag==1)pub_up(kinect_whell_pub,wheel_output,kinect_speed_pub,speed_output);
            if(flag==2)pub_stop(kinect_whell_pub,wheel_output,kinect_speed_pub,speed_output);
            if(flag==3)pub_left(kinect_whell_pub,wheel_output,kinect_speed_pub,speed_output);
            if(flag==4)pub_right(kinect_whell_pub,wheel_output,kinect_speed_pub,speed_output);
//            printf("Accept socket error: %s(errno: %d)",strerror(errno),errno);
//            printf("Server receive data Failed!\n");
        }
        else if(ret>0)
        {
            string result=Buffer;
            printf("get message!\n");
            printf("This Gesture is:%s\n", Buffer);
            if(result=="Up")
            {
                ROS_INFO("send msg = %s", result.c_str ());
                pub_up(kinect_whell_pub,wheel_output,kinect_speed_pub,speed_output);
                flag=1;
                printf("ret=%d,flag=%d\n",ret,flag);
//                wheel_output.data=0;
//                speed_output.data.push_back (2100);
//                speed_output.data.push_back (1);
//                kinect_whell_pub.publish(wheel_output);
//                kinect_speed_pub.publish(speed_output);
            }
            else if(result=="Stop")
            {
                ROS_INFO("send msg = %s", result.c_str ());
                pub_stop(kinect_whell_pub,wheel_output,kinect_speed_pub,speed_output);
                flag=2;
                printf("ret=%d,flag=%d\n",ret,flag);
//                wheel_output.data=0;
//                speed_output.data.push_back (1700);
//                speed_output.data.push_back (80);
//                kinect_whell_pub.publish(wheel_output);
//                kinect_speed_pub.publish(speed_output);
            }
            else if(result=="Left")
            {
                ROS_INFO("send msg = %s", result.c_str ());
                pub_left(kinect_whell_pub,wheel_output,kinect_speed_pub,speed_output);
                flag=3;
                printf("ret=%d,flag=%d\n",ret,flag);
//                wheel_output.data=150;
//                speed_output.data.push_back (2100);
//                speed_output.data.push_back (1);
//                kinect_whell_pub.publish(wheel_output);
//                kinect_speed_pub.publish(speed_output);
            }
            else if(result=="Right")
            {
                ROS_INFO("send msg = %s", result.c_str ());
                pub_right(kinect_whell_pub,wheel_output,kinect_speed_pub,speed_output);
                flag=4;
                printf("ret=%d,flag=%d\n",ret,flag);
//                wheel_output.data=-150;
//                speed_output.data.push_back (2100);
//                speed_output.data.push_back (1);
//                kinect_whell_pub.publish(wheel_output);
//                kinect_speed_pub.publish(speed_output);
            }

            else//meaningless下将会保持上一个手势指令
            {
                if(flag==1)pub_stop(kinect_whell_pub,wheel_output,kinect_speed_pub,speed_output);
                if(flag==2)pub_left(kinect_whell_pub,wheel_output,kinect_speed_pub,speed_output);
                if(flag==3)pub_right(kinect_whell_pub,wheel_output,kinect_speed_pub,speed_output);
                if(flag==4)pub_up(kinect_whell_pub,wheel_output,kinect_speed_pub,speed_output);
            }
            
            printf("________NEXT_______\n");
            loop_rate.sleep ();
//            sleep (1*1000);
        }
        else
        {
            printf("Client cancel the connect!\n");
            break;
        }
    }

    close(client_socket);
    close(server_socket);
    return 0;
}
