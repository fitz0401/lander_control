#include <iostream>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <robot_state_publisher/robot_state_publisher.h>
using namespace std;

int main(int argc, char** argv)
 {
    ros::init(argc, argv, "MotorState_publisher"); //节点的名称
    ros::NodeHandle nh;
    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);    //设置一个发布者，将消息发布出去,发布到相应的节点中去
    ros::Rate loop_rate(10);    //这个设置的太大，效果很不好，目前觉得为10最好了
    const double degree = M_PI/180;
    const double radius = 2;
    int i=-69;

    // robot state
    double angle= 0;
    // message declarations
    sensor_msgs::JointState joint_state;
  //  for(int j=0;j<6;++j)
   // {
        //joint_state.velocity.push_back(1);
        //joint_state.effort.push_back(200);
    //}

   

    while (ros::ok()) {
        //update joint_state
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(12);
        joint_state.position.resize(12);
        joint_state.velocity.resize(12);
        joint_state.effort.resize(12);
	    
        joint_state.name[0]="joint0";
        joint_state.position[0] = i*degree;
        joint_state.velocity[0] = i*degree;
        joint_state.effort[0] = i*degree;
	    
        joint_state.name[1] ="joint1";
        joint_state.position[1] = i*degree;
        joint_state.velocity[1] = i*degree;
        joint_state.effort[1] = i*degree;
	    
        joint_state.name[2] ="joint2";
        joint_state.position[2] = i*degree;
        joint_state.velocity[2] = i*degree;
        joint_state.effort[2] = i*degree;
        
        joint_state.name[3] ="joint3";
        joint_state.position[3] = i*degree;
        joint_state.velocity[3] = i*degree;
        joint_state.effort[3] = i*degree;
	    
        joint_state.name[4] ="joint4";
        joint_state.position[4] = i*degree;
        joint_state.velocity[4] = i*degree;
        joint_state.effort[4] = i*degree;
	    
        joint_state.name[5] ="joint5";
        joint_state.position[5] = i*degree;
        joint_state.velocity[5] = i*degree;
        joint_state.effort[5] = i*degree;

        joint_state.name[6]="joint6";
        joint_state.position[6] = i*degree;
        joint_state.velocity[6] = i*degree;
        joint_state.effort[6] = i*degree;
	    
        joint_state.name[7] ="joint7";
        joint_state.position[7] = i*degree;
        joint_state.velocity[7] = i*degree;
        joint_state.effort[7] = i*degree;
	    
        joint_state.name[8] ="joint8";
        joint_state.position[8] = i*degree;
        joint_state.velocity[8] = i*degree;
        joint_state.effort[8] = i*degree;
        
        joint_state.name[9] ="joint9";
        joint_state.position[9] = i*degree;
        joint_state.velocity[9] = i*degree;
        joint_state.effort[9] = i*degree;
	    
        joint_state.name[10] ="joint10";
        joint_state.position[10] = i*degree;
        joint_state.velocity[10] = i*degree;
        joint_state.effort[10] = i*degree;
	    
        joint_state.name[11] ="joint11";
        joint_state.position[11] = i*degree;
        joint_state.velocity[11] = i*degree;
        joint_state.effort[11] = i*degree;
        
        if(i<=70)
            i++;
        else
            i=-69; 

        //send the joint state and transform
        joint_pub.publish(joint_state);

        // This will adjust as needed per iteration
        loop_rate.sleep();
    }

    return 0;
}
