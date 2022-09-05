#include <iostream>
#include <thread>
#include <aris.hpp>
#include <atomic>
#include <string>
#include <vector>
#include <filesystem>
#include <fstream>
#include <ControlRTPlan.h>
#include <ros/ros.h>
#include "std_msgs/String.h"

using namespace aris::dynamic;
using namespace std;
auto xmlpath = "/home/kaanh/Desktop/Lander_ws/src/lander/kaanh.xml";	//获取当前工程所在的路径
const std::string xmlfile = "kaanh.xml";		//控制配置文件名称
// aris全局控制单例
auto&cs = aris::server::ControlServer::instance();    
ros::Publisher pub;
ros::Subscriber sub;

void doControlMsg(const std_msgs::String::ConstPtr& msg_p){
    // 在这里用来接受规划端发出的控制指令，接收到即刻通过pipe发送到机器人执行进程中，更改preparert的相关数值
    // executeRT中每隔10ms读取该数值，来改变控制指令，通过这种方式实现实时控制
    setlocale(LC_ALL,"");
    // 回调函数拿到状态返回值
    cs.executeCmd("PlanMsg --motion_val=" + msg_p->data, [](aris::plan::Plan& plan){
        // std::string msg = to_string(std::any_cast<int>(plan.ret()));
        // pub.publish(msg);
    });
}

void StatePub_thread()
{
    // 在这里用来接收机器人执行进程pipe导出的状态数据，并每1ms通过pub发送出去?????????
    // 由于要和imu数据进行同频处理，所以实际上规划端收到的是每10ms的状态数据然后进行实时规划
    while (1)
    {
        cout << "t22222\n";
        usleep(1000 * 1000);
    }
}

int main(int argc, char *argv[])
{
    aris::core::fromXmlFile(cs, xmlpath);		//加载kaanh.xml配置
    cs.resetPlanRoot(ControlRTPlan::createPlanRoot().release());//加载cmd配置

    cs.init();									//初始化
    cs.start();                                 //不注释，则程序运行时开启控制器服务

    std::cout << "start" <<std::endl;
    cs.executeCmd("ds --limit_time=20000");    
    cs.executeCmd("md --limit_time=20000");
    cs.executeCmd("en --limit_time=20000");
	// 开启WebSocket/socket服务器//
    cs.open();
    
    // 初始化 ROS 节点:命名(唯一)
    ros::init(argc,argv,"Lander");
    ros::NodeHandle nh;
    pub = nh.advertise<std_msgs::String>("/state",2);// 定义状态发布方
    sub = nh.subscribe<std_msgs::String>("/control",2,doControlMsg); // 定义控制量订阅方
    ros::param::set("state_val", 100);
    sleep(8);

    // 添加WaitingPlan分进程
    cs.executeCmd("WaitingPlan");      

    //实例化状态发布分线程，使用函数StatePub_thread构造，然后该线程就开始执行了（t1()）
    thread Pubthread(StatePub_thread);  
    
    // 必须ros::spin()在mythread.join()之前：才能保证主线程中的sub和分线程中的程序同时运行  
    ros::spin();//循环读取接收的数据，并调用回调函数处理
    Pubthread.join();// 必须将线程join或者detach 等待子线程结束主进程才可以退出

    while (1)
    {}
    
    return 0;
}

