#!/usr/bin/env python
# coding=UTF-8
import rospy
import message_filters
from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import Imu

def multi_callback(subcriber_LegImu0, subcriber_LegImu1,subcriber_LegImu2, subcriber_LegImu3,subcriber_TorsoImu, subcriber_Chatter):
    # print(str(rospy.get_time())+"   同步完成！")
    StateData = [subcriber_LegImu0.orientation.x,
                 subcriber_LegImu0.orientation.y,
                 subcriber_LegImu0.orientation.z,
                 subcriber_LegImu0.orientation.w,
                 subcriber_LegImu0.angular_velocity.x,
                 subcriber_LegImu0.angular_velocity.y,
                 subcriber_LegImu0.angular_velocity.z,
                 subcriber_LegImu0.linear_acceleration.x,
                 subcriber_LegImu0.linear_acceleration.y,
                 subcriber_LegImu0.linear_acceleration.z,

                 subcriber_LegImu1.orientation.x,
                 subcriber_LegImu1.orientation.y,
                 subcriber_LegImu1.orientation.z,
                 subcriber_LegImu1.orientation.w,
                 subcriber_LegImu1.angular_velocity.x,
                 subcriber_LegImu1.angular_velocity.y,
                 subcriber_LegImu1.angular_velocity.z,
                 subcriber_LegImu1.linear_acceleration.x,
                 subcriber_LegImu1.linear_acceleration.y,
                 subcriber_LegImu1.linear_acceleration.z,

                 subcriber_LegImu2.orientation.x,
                 subcriber_LegImu2.orientation.y,
                 subcriber_LegImu2.orientation.z,
                 subcriber_LegImu2.orientation.w,
                 subcriber_LegImu2.angular_velocity.x,
                 subcriber_LegImu2.angular_velocity.y,
                 subcriber_LegImu2.angular_velocity.z,
                 subcriber_LegImu2.linear_acceleration.x,
                 subcriber_LegImu2.linear_acceleration.y,
                 subcriber_LegImu2.linear_acceleration.z,

                 subcriber_LegImu3.orientation.x,
                 subcriber_LegImu3.orientation.y,
                 subcriber_LegImu3.orientation.z,
                 subcriber_LegImu3.orientation.w,
                 subcriber_LegImu3.angular_velocity.x,
                 subcriber_LegImu3.angular_velocity.y,
                 subcriber_LegImu3.angular_velocity.z,
                 subcriber_LegImu3.linear_acceleration.x,
                 subcriber_LegImu3.linear_acceleration.y,
                 subcriber_LegImu3.linear_acceleration.z,

                 subcriber_TorsoImu.orientation.x,
                 subcriber_TorsoImu.orientation.y,
                 subcriber_TorsoImu.orientation.z,
                 subcriber_TorsoImu.orientation.w,
                 subcriber_TorsoImu.angular_velocity.x,
                 subcriber_TorsoImu.angular_velocity.y,
                 subcriber_TorsoImu.angular_velocity.z,
                 subcriber_TorsoImu.linear_acceleration.x,
                 subcriber_TorsoImu.linear_acceleration.y,
                 subcriber_TorsoImu.linear_acceleration.z]

    StateDate_Pub = Float32MultiArray(data=StateData)
    publisher_StateDate.publish(StateDate_Pub)
    print(str(rospy.get_time()))

    
# 该程序为用来进行数据同步的测试：
# 1连接imu并打开ALLImus_driver.launch文件；
# 2打开自定义的test_server.py,这个最终实验的时候应该替换成机器人执行程序；
# 3就可以运行此程序，imu频率为100hz，自定义chatter频率为1000hz,该程序运行可以看到稳定的每0.01秒实现数据同步；
# 4该程序会将所有数据打包为一个Float32MultiArray统一发送出去，需要添加的状态数据的话直接在上面添加即可；
# 5执行Sub_StateData.py可以用来测试订阅同频处理后的数据；
if __name__ == '__main__':
    rospy.init_node('DataSynchronization',anonymous=True)

    subcriber_LegImu0 = message_filters.Subscriber('/LegImu0/imu_data', Imu, queue_size=1)
    subcriber_LegImu1 = message_filters.Subscriber('/LegImu1/imu_data', Imu, queue_size=1)
    subcriber_LegImu2 = message_filters.Subscriber('/LegImu2/imu_data', Imu, queue_size=1)
    subcriber_LegImu3 = message_filters.Subscriber('/LegImu3/imu_data', Imu, queue_size=1)
    subcriber_TorsoImu = message_filters.Subscriber('/TorsoImu/imu_data', Imu, queue_size=1)
    subcriber_Chatter = message_filters.Subscriber('/chatter', String, queue_size=1)  # 模拟Lander的数据

    publisher_StateDate = rospy.Publisher("/StateData",Float32MultiArray,queue_size=10)  # 发布打包好的状态数据
    
    sync = message_filters.ApproximateTimeSynchronizer(
        [subcriber_LegImu0, subcriber_LegImu1,subcriber_LegImu2, subcriber_LegImu3, subcriber_TorsoImu, subcriber_Chatter],
        10,0.01,allow_headerless=True)

    sync.registerCallback(multi_callback)

    rospy.spin()
