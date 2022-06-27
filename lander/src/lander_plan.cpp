#include <iostream>
#include <aris.hpp>
#include <atomic>
#include <string>
#include <filesystem>
#include <ControlCMD.h>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "lander/mv_msgs.h"

using namespace aris::dynamic;
auto xmlpath = "/home/kaanh/Desktop/Lander_ws/src/lander/kaanh.xml";	//获取当前工程所在的路径
const std::string xmlfile = "kaanh.xml";		//控制配置文件名称
// aris全局控制单例
auto&cs = aris::server::ControlServer::instance();    

// 收到控制指令后所要进行的动作。bool 返回值由于标志是否处理成功
bool doReq(lander::mv_msgs::Request& req,
          lander::mv_msgs::Response& resp){
    bool ifFinishFlag = false;

    ROS_INFO("服务器接收到的请求数据为:leg = %d, x = %f, y = %f, z = %f",
    req.leg, (double)req.x_motion,(double)req.y_motion,(double)req.z_motion);

    ros::param::set("leg_index", req.leg);
    ros::param::set("x_motion", (double)req.x_motion / 1000.0);
    ros::param::set("y_motion", (double)req.y_motion / 1000.0);
    ros::param::set("z_motion", (double)req.z_motion / 1000.0);
    cs.executeCmd("planfoot");
    while(!ifFinishFlag){
        ros::param::get("ifFinishFlag",ifFinishFlag);
    }
    
    resp.isFinish = true;
    return true;
}

int main(int argc, char *argv[])
{
	aris::core::fromXmlFile(cs, xmlpath);		//加载kaanh.xml配置
    cs.resetPlanRoot(ControlCMD::createPlanRoot().release());//加载cmd配置

    cs.init();									//初始化
    cs.start();                                 //不注释，则程序运行时开启控制器服务

    std::cout << "start" <<std::endl;
    //aris::plan::Plan::ret();
    cs.executeCmd("ds");    
    cs.executeCmd("md");
    cs.executeCmd("en");
	//开启WebSocket/socket服务器//
    cs.open();
    
    // 开启ros端信号接受程序
    setlocale(LC_ALL,"");
    // 2.初始化 ROS 节点
    ros::init(argc,argv,"Control_Plan_Server");
    // 3.创建 ROS 句柄
    ros::NodeHandle nh;
    // 4.创建 服务 对象
    ros::ServiceServer server = nh.advertiseService("mv_msgs",doReq);
    ROS_INFO("服务已经启动....");
    //     5.回调函数处理请求并产生响应
    //     6.由于请求有多个，需要调用 ros::spin()
    ros::spin();

    return 0;
}

