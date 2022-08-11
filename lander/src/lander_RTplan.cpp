#include <iostream>
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

void doMsg(const std_msgs::String::ConstPtr& msg_p){
    setlocale(LC_ALL,"");
    // 回调函数拿到状态返回值
    cs.executeCmd("PlanMsg --motion_val=" + msg_p->data, [](aris::plan::Plan& plan){
        std::string msg = to_string(std::any_cast<int>(plan.ret()));
        pub.publish(msg);
    });
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
    // ros::init(argc,argv,"Lander");
    // ros::NodeHandle nh;
    // pub = nh.advertise<std_msgs::String>("state",2);
    // ros::param::set("state_val", 100);
    // sleep(8);
    cs.executeCmd("WaitingPlan");        
    // ros::Subscriber sub = nh.subscribe<std_msgs::String>("control",10,doMsg);
    // ros::spin();//循环读取接收的数据，并调用回调函数处理

    while (1)
    {}
    
    return 0;
}

