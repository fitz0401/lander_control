#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <netinet/in.h>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <string.h>
//#include <boost/algorithm/string.hpp>
#include <vector>
#include <string>
using namespace std;
#define PORT 10001
 
int control_server()
{
    // 初始化**********************************************************************//
    struct sockaddr_in s_in;//server address structure
    struct sockaddr_in c_in;//client address structure
    int l_fd,c_fd;
    socklen_t len;
    char buf[100];//content buff area
    string tem;
    float x_value = 0.0;
    float y_value = 0.0;
    float z_value = 0.0;
    memset((void *)&s_in,0,sizeof(s_in));
 
    s_in.sin_family = AF_INET;//IPV4 communication domain
    s_in.sin_addr.s_addr = INADDR_ANY;//accept any address
    s_in.sin_port = htons(PORT);//change port to netchar
 
    l_fd = socket(AF_INET,SOCK_STREAM,0);//socket(int domain, int type, int protocol)
    bind(l_fd,(struct sockaddr *)&s_in,sizeof(s_in));
    listen(l_fd,5);//同时只能有一个连接
 
    cout<<"begin"<<endl;
    // 初始化**********************************************************************//

    while(1){
        c_fd = accept(l_fd,(struct sockaddr *)&c_in,&len);  // 外加循环防止断连

        // 数据处理*****************************************************************//
        while(1){
            // buf清零***********************************************************//
            for(int j=0;j<100;j++){
                buf[j] = 0;
            }
            // buf清零***********************************************************//

            // 读取客户端发来的消息*********************************************//
            int n = read(c_fd,buf,100);
            // 读取客户端发来的消息*********************************************//

            // 检测是否需要关闭(我没有用这个)***********************************//
            if(!strcmp(buf, "q\n") || !strcmp(buf, "Q\n")){
                cout << "q pressed\n";
                close(c_fd);
                break;
            }
            // 检测是否需要关闭**************************************************//

            // 将获得的三维数据拆解**********************************************//
            vector<string> vec;
//            boost::split(vec, buf,boost::is_any_of("."), boost::token_compress_on);
            x_value = atof(vec[0].c_str());   // x坐标
            y_value = atof(vec[1].c_str());   // y坐标
            z_value = atof(vec[2].c_str());  // z坐标
            cout<<"x="<<x_value<<endl;
            cout<<"y="<<y_value<<endl;
            cout<<"z="<<z_value<<endl;
            // 将获得的三维数据拆解**********************************************//

            // 向client返回数据***************************************************//
            write(c_fd,buf,n);//sent message back to client 也就是双向通讯
            // 向client返回数据***************************************************//
        }
        // 数据处理*****************************************************************//
    }
    return 0;
}
