#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <unistd.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define DEFAULT_PORT 6000
#define BUFFER_SIZE sizeof(unsigned char)*800*500*3
using namespace std;
using namespace cv;


int main()
{

    Mat frame(800,500,CV_8UC3);
    Mat cc(800,500,CV_8UC3);
    resize(frame,frame,Size(800,500));

    vector<int> para;
    para.push_back (CV_IMWRITE_PNG_COMPRESSION);
    para.push_back (1);
    struct sockaddr_in server_addr;
    char Buffer[BUFFER_SIZE];
    bzero(Buffer,BUFFER_SIZE);
    struct sockaddr_in client_addr;
    socklen_t length=sizeof(client_addr);
    bzero(Buffer,BUFFER_SIZE);
//    string str2(Buffer,BUFFER_SIZE);

    int client_socket=0;
    int server_socket = 0;
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
    while(1)
    {
        bzero(Buffer,BUFFER_SIZE);
//        for(int i=0;i<50;i++)
//        {
        int ret=recv(client_socket,Buffer,BUFFER_SIZE,MSG_NOSIGNAL|MSG_WAITALL);
//        std::cout<<ret<<endl;
        if(ret<0)
        {
            printf("accept socket error: %s(errno: %d)",strerror(errno),errno);
            printf("Server Recieve Data Failed!\n");
            break;
        }
        else if(ret>0)
        {
            memcpy (frame.data,Buffer,BUFFER_SIZE);
            cc=frame.clone ();
            resize(cc,cc,Size(800,500));
            imshow ("test",cc);
            //        imwrite ("/home/cyberc3/socket/2.png",frame,para);
            waitKey (1);
        }

//            printf("get message!\n");
//            if(ret<0)
//            {
//                printf("accept socket error: %s(errno: %d)",strerror(errno),errno);
//                printf("Server Recieve Data Failed!\n");
//                break;
//            }
//            else if(ret>0)
//            {
//                printf("get message!\n");
//                printf("%s\n", Buffer);
//            }
        else if(ret==0)
            break;
    }
    cvDestroyWindow ("test");
    close(client_socket);
    close(server_socket);
    return 0;
}
