#include <iostream>
#include <thread>
#include <stdlib.h> //Sleep
#include <ros/ros.h>
#include "std_msgs/String.h"

#include <unistd.h>
// #include <windows.h>

using namespace std;
ros::Publisher pub;
ros::Subscriber sub;

void doMsg(const std_msgs::String::ConstPtr& msg_p){
    cout << "t33331" << endl;
}

void StatePub_thread()
{
    while (1)
    {
        cout << "t22222\n";
        usleep(1000 * 1000);
    }
}
int main(int argc, char *argv[])
{

    ros::init(argc,argv,"Lander");
    ros::NodeHandle nh;
    pub = nh.advertise<std_msgs::String>("state",2);  // 定义状态发布方
    sub = nh.subscribe<std_msgs::String>("control",10,doMsg);  // 定义控制量订阅方

    // thread th1(t1); //实例化一个线程对象th1，使用函数t1构造，然后该线程就开始执行了（t1()）
    thread mythread(StatePub_thread);

    // 必须ros::spin()在mythread.join()之前：才能保证主线程中的sub和分线程中的程序同时运行  
    ros::spin();//循环读取接收的数据，并调用回调函数处理
    mythread.join();// 必须将线程join或者detach 等待子线程结束主进程才可以退出

    // or use detach
    // th1.detach();
    // th2.detach();

    cout << "here is main\n\n";

    return 0;
}
