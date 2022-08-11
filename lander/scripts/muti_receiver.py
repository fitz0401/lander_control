#!/usr/bin/env python3.5
# coding=UTF-8
import rospy,math,random
import numpy as np
import message_filters
from std_msgs.msg import String
from sensor_msgs.msg import Imu

def multi_callback(Subcriber_laser,Subcriber_pose):
    print("同步完成！")

if __name__ == '__main__':
    rospy.init_node('DataSynchronization',anonymous=True)

    subcriber_LegImu0 = message_filters.Subscriber('/LegImu0/imu_data', Imu, queue_size=1)
    subcriber_LegImu1 = message_filters.Subscriber('/LegImu1/imu_data', Imu, queue_size=1)
    subcriber_LegImu2 = message_filters.Subscriber('/LegImu2/imu_data', Imu, queue_size=1)
    subcriber_LegImu3 = message_filters.Subscriber('/LegImu3/imu_data', Imu, queue_size=1)
    subcriber_TorsoImu = message_filters.Subscriber('/TorsoImu/imu_data', Imu, queue_size=1)
    
    # sync = message_filters.ApproximateTimeSynchronizer(
    #     [subcriber_LegImu0, subcriber_LegImu1,subcriber_LegImu2, subcriber_LegImu3,subcriber_TorsoImu],
    #     10,0.1,allow_headerless=True)

    # sync.registerCallback(multi_callback)

    # rospy.spin()
