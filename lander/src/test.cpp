/*
    需求: 
        编写两个节点实现服务通信，客户端节点需要提交两个整数到服务器
        服务器需要解析客户端提交的数据，相加后，将结果响应回客户端，
        客户端再解析

    服务器实现:
        1.包含头文件
        2.初始化 ROS 节点
        3.创建 ROS 句柄
        4.创建 服务 对象
        5.回调函数处理请求并产生响应
        6.由于请求有多个，需要调用 ros::spin()

*/
#include "ros/ros.h"
#include "lander/mv_msgs.h"

// bool 返回值由于标志是否处理成功
bool doReq(lander::mv_msgs::Request& req,
          lander::mv_msgs::Response& resp){
    int command_index = req.command_index;
    double z_motion = req.z_motion;

    ROS_INFO("服务器接收到的请求数据为:command_index = %d, num2 = %f",command_index, z_motion);

    //逻辑处理
    if (command_index < 0 || command_index > 4)
    {
        ROS_ERROR("提交的命令异常");
        return false;
    }

    //如果没有异常，那么相加并将结果赋值给 resp
    resp.isFinish = true;
    return true;
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    // 2.初始化 ROS 节点
    ros::init(argc,argv,"Lander_Server");
    // 3.创建 ROS 句柄
    ros::NodeHandle nh;
    // 4.创建 服务 对象
    ros::ServiceServer server = nh.advertiseService("PlanMsg",doReq);
    ROS_INFO("服务已经启动....");
    //     5.回调函数处理请求并产生响应
    //     6.由于请求有多个，需要调用 ros::spin()
    ros::spin();
    return 0;
}
