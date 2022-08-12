#!/usr/bin/env python
# coding=UTF-8
import rospy
from std_msgs.msg import String, Float32MultiArray

def doMsg(msg):
    StateData = msg.data
    print(StateData)

if __name__ == "__main__":
    #2.初始化 ROS 节点:命名(唯一)
    rospy.init_node("StateData_listener")
    #3.实例化 订阅者 对象
    rospy.loginfo("start")
    sub = rospy.Subscriber("/StateData",Float32MultiArray,doMsg,queue_size=10)
    #4.处理订阅的消息(回调函数)
    #5.设置循环调用回调函数
    rospy.spin()
