#include <iostream>
#include <aris.hpp>
#include <atomic>
#include <string>
#include <vector>
#include <filesystem>
#include <fstream>
#include <ControlPlan.h>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "lander/gait_plan_msgs.h"
#include "lander/gait_feedback_msgs.h"

using namespace aris::dynamic;
using namespace std;
auto xmlpath = "/home/kaanh/Desktop/Lander_ws/src/lander/kaanh.xml";	//获取当前工程所在的路径
const std::string xmlfile = "kaanh.xml";		//控制配置文件名称
// aris全局控制单例
auto&cs = aris::server::ControlServer::instance();    

// 收到控制指令后所要进行的动作[无反馈信息，用于自由步态行走]
void doMsg(const lander::gait_plan_msgs::ConstPtr& req) {
    // ROS全局参数中isFinishFlag设置为false，指令执行完后会被设置为true
    ros::param::set("isFinishFlag",false);
    bool isFinishFlag = false;

    // 解析请求命令
    // 0:执行getpos：开机时检查编码器位置信息和足端位置信息是否正确
    if (req->command_index == 0) {
        ROS_INFO("————————打印着陆编码器位置信息和足端位置信息————————");
        cs.executeCmd("getposplan");
    }  
    // 1:执行init：所有指令运行结束后，在关机之前复位，保证下次开机时着陆器位于初始位置
    else if (req->command_index == 1) {
        ROS_INFO("————————正在执行位姿初始化————————");
        cs.executeCmd("initplan");
    }
    // 2:执行planfoot，单腿或多腿运动沿直线运动.一般用于起始时发出-z=-75的指令，进入运动规划初始位置
    else if (req->command_index == 2) {
        ROS_INFO("————————正在执行末端轨迹运动————————");
        ros::param::set("leg_index", req->leg_index);
        ROS_INFO("足端目标位移为:");
        aris::core::Matrix end_mat(4, 3, 0.0);
        for (int i = 0; i < 3; i++) {
            end_mat(0, i) = req->foot1_motion[i] / 1000.0;
            end_mat(1, i) = req->foot2_motion[i] / 1000.0;
            end_mat(2, i) = req->foot3_motion[i] / 1000.0;
            end_mat(3, i) = req->foot4_motion[i] / 1000.0;
        }
        for (int i = 0; i < 4; i++) {
            if (req->leg_index == 12 || req->leg_index == i) {
                cout << "leg" << i << ": ";
                for (int j = 0; j < 3; j++) {
                    cout << end_mat(i ,j) * 1000.0 << " ";
                }
                cout << endl;
            }
        }
        auto plan = cs.executeCmd("planfoot --end_mat=" + end_mat.toString());
        while(!isFinishFlag) {
            ros::param::get("isFinishFlag",isFinishFlag);
        }
        if (plan->retCode() != 0) {
            cout << "retCode: " << plan->retCode() << endl;
            cout << "retMsg: " << plan->retMsg()  << endl; 
        }       
    }
    // 3:执行运动规划执行，每次接收四个足端运动轨迹数组
    else if (req->command_index == 3) {
        ROS_INFO("————————正在沿规划轨迹运动————————");
        ros::param::set("data_num", req->data_num);
        aris::core::Matrix trace_mat(12, req->data_num, 0.0);
        for (int i = 0; i < req->data_num; i++) {
            trace_mat(0, i) = req->foot1_trace_x[i];
            trace_mat(1, i) = req->foot1_trace_y[i];
            trace_mat(2, i) = req->foot1_trace_z[i];
            trace_mat(3, i) = req->foot2_trace_x[i];
            trace_mat(4, i) = req->foot2_trace_y[i];
            trace_mat(5, i) = req->foot2_trace_z[i];
            trace_mat(6, i) = req->foot3_trace_x[i];
            trace_mat(7, i) = req->foot3_trace_y[i];
            trace_mat(8, i) = req->foot3_trace_z[i];
            trace_mat(9, i) = req->foot4_trace_x[i];
            trace_mat(10, i) = req->foot4_trace_y[i];
            trace_mat(11, i) = req->foot4_trace_z[i];
        }
        auto plan = cs.executeCmd("planmotion --trace_mat=" + trace_mat.toString());
        while(!isFinishFlag) {
            ros::param::get("isFinishFlag",isFinishFlag);
        } 
        // 打印错误信息
        if (plan->retCode() != 0) {
            cout << "retCode: " << plan->retCode() << endl;
            cout << "retMsg: " << plan->retMsg()  << endl; 
        }    
    }  
    else if (req->command_index == -1) {
        ROS_INFO("————————正在清除错误信息并重新使能电机————————");
        cs.executeCmd("cl");
        cs.executeCmd("en");
        ros::param::set("isFinishFlag",true);
    }  
    // 等待着陆器将指令运行完毕
    while(!isFinishFlag){
        ros::param::get("isFinishFlag",isFinishFlag);
    }
}

// 收到控制指令后所要进行和反馈的动作[有反馈信息，用于触地检测行走]
bool doReq(lander::gait_feedback_msgs::Request& req, lander::gait_feedback_msgs::Response& resp){
    // ROS全局参数中isFinishFlag设置为false，指令执行完后会被设置为true
    ros::param::set("isFinishFlag",false);
    bool isFinishFlag = false;

    // 解析请求命令
    // 0:执行getpos：开机时检查编码器位置信息和足端位置信息是否正确
    if (req.command_index == 0) {
        ROS_INFO("————————打印着陆编码器位置信息和足端位置信息————————");
        cs.executeCmd("getposplan");
    }  
    // 1:执行init：所有指令运行结束后，在关机之前复位，保证下次开机时着陆器位于初始位置
    else if (req.command_index == 1) {
        ROS_INFO("————————正在执行位姿初始化————————");
        cs.executeCmd("initplan");
    }
    // 2:执行planfoot，单腿或多腿运动沿直线运动.一般用于起始时发出-z=-75的指令，进入运动规划初始位置
    else if (req.command_index == 2) {
        ROS_INFO("————————正在执行末端轨迹运动————————");
        ros::param::set("leg_index", req.leg_index);
        ROS_INFO("足端目标位移为:");
        aris::core::Matrix end_mat(4, 3, 0.0);
        for (int i = 0; i < 3; i++) {
            end_mat(0, i) = req.foot1_motion[i] / 1000.0;
            end_mat(1, i) = req.foot2_motion[i] / 1000.0;
            end_mat(2, i) = req.foot3_motion[i] / 1000.0;
            end_mat(3, i) = req.foot4_motion[i] / 1000.0;
        }
        for (int i = 0; i < 4; i++) {
            if (req.leg_index == 12 || req.leg_index == i) {
                cout << "leg" << i << ": ";
                for (int j = 0; j < 3; j++) {
                    cout << end_mat(i ,j) * 1000.0 << " ";
                }
                cout << endl;
            }
        }
        auto plan = cs.executeCmd("planfootfeedback --end_mat=" + end_mat.toString());
        while(!isFinishFlag) {
            ros::param::get("isFinishFlag",isFinishFlag);
        }
        if (plan->retCode() != 0) {
            cout << "retCode: " << plan->retCode() << endl;
            cout << "retMsg: " << plan->retMsg()  << endl; 
        }
        // 获取函数返回值
        vector<vector<double>> feet_position = any_cast<vector<vector<double>>>(plan->ret());
        for (int i = 0; i < 3; ++i) {
            resp.foot1_position[i] = feet_position[0][i];
            resp.foot2_position[i] = feet_position[1][i];
            resp.foot3_position[i] = feet_position[2][i];
            resp.foot4_position[i] = feet_position[3][i];
        }   
    }
    // 3:执行运动规划执行，每次接收四个足端运动轨迹数组
    else if (req.command_index == 3) {
        ROS_INFO("————————正在沿规划轨迹运动————————");
        ros::param::set("data_num", req.data_num);
        aris::core::Matrix trace_mat(12, req.data_num, 0.0);
        for (int i = 0; i < req.data_num; i++) {
            trace_mat(0, i) = req.foot1_trace_x[i];
            trace_mat(1, i) = req.foot1_trace_y[i];
            trace_mat(2, i) = req.foot1_trace_z[i];
            trace_mat(3, i) = req.foot2_trace_x[i];
            trace_mat(4, i) = req.foot2_trace_y[i];
            trace_mat(5, i) = req.foot2_trace_z[i];
            trace_mat(6, i) = req.foot3_trace_x[i];
            trace_mat(7, i) = req.foot3_trace_y[i];
            trace_mat(8, i) = req.foot3_trace_z[i];
            trace_mat(9, i) = req.foot4_trace_x[i];
            trace_mat(10, i) = req.foot4_trace_y[i];
            trace_mat(11, i) = req.foot4_trace_z[i];
        }
        auto plan = cs.executeCmd("planmotion --trace_mat=" + trace_mat.toString());
        while(!isFinishFlag) {
            ros::param::get("isFinishFlag",isFinishFlag);
        } 
        // 打印错误信息
        if (plan->retCode() != 0) {
            cout << "retCode: " << plan->retCode() << endl;
            cout << "retMsg: " << plan->retMsg()  << endl; 
        }
    }
    else if (req.command_index == -1) {
        ROS_INFO("————————正在清除错误信息并重新使能电机————————");
        cs.executeCmd("cl");
        cs.executeCmd("en");
        ros::param::set("isFinishFlag",true);
    }  
    // 等待着陆器将指令运行完毕
    while(!isFinishFlag){
        ros::param::get("isFinishFlag",isFinishFlag);
    }
    resp.isFinish = true;
    return true;
}

int main(int argc, char *argv[])
{
	aris::core::fromXmlFile(cs, xmlpath);		//加载kaanh.xml配置
    cs.resetPlanRoot(ControlPlan::createPlanRoot().release());//加载cmd配置

    cs.init();									//初始化
    cs.start();                                 //不注释，则程序运行时开启控制器服务

    std::cout << "start" <<std::endl;
    cs.executeCmd("ds --limit_time=20000");    
    cs.executeCmd("md --limit_time=20000");
    cs.executeCmd("en --limit_time=20000");
	// 开启WebSocket/socket服务器//
    cs.open();
    
    // 开启ros端信号接受程序
    setlocale(LC_ALL,"");
    // 初始化 ROS 节点
    ros::init(argc,argv,"Control_Plan_Server");
    // 创建 ROS 句柄
    ros::NodeHandle nh;

    // 实例化"订阅者"对象：用于自由步态无反馈控制
    //ros::Subscriber sub = nh.subscribe<lander::gait_plan_msgs>("GaitPlan",2,doMsg);
    // 创建 "服务" 对象：用于触地检测后的反馈控制
    ros::ServiceServer server = nh.advertiseService("GaitPlanFeedback",doReq);

    ROS_INFO("服务已经启动....");
    // 5.回调函数处理请求并产生响应
    // 6.由于请求有多个，需要调用 ros::spin()
    ros::spin();

    return 0;
}
