#include <iostream>
#include <aris.hpp>
#include <atomic>
#include <string>
#include <filesystem>
#include <ControlCMD.h>
#include <ros/ros.h>

using namespace aris::dynamic;
auto xmlpath = "/home/kaanh/Desktop/Lander_ws/src/lander/kaanh.xml";	//获取当前工程所在的路径
const std::string xmlfile = "kaanh.xml";		//控制配置文件名称

int main(int argc, char *argv[])
{
	auto&cs = aris::server::ControlServer::instance();
    aris::core::fromXmlFile(cs, xmlpath);		//加载kaanh.xml配置
    cs.resetPlanRoot(ControlCMD::createPlanRoot().release());//加载cmd配置
//    aris::core::toXmlFile(cs, xmlpath);		   //save kaanh.xml配置
    cs.init();									//初始化
    cs.start();                                 //不注释，则程序运行时开启控制器服务

    std::cout << "start" <<std::endl;

    cs.executeCmd("ds --limit_time=20000");  
    cs.executeCmd("md --limit_time=20000");
    cs.executeCmd("en --limit_time=20000");
    
	//开启WebSocket/socket服务器//
    cs.open();

	//等待终端输入函数，本函数不能去掉，否则实时线程和主线程都会结束//
	cs.runCmdLine();

	return 0;
}

