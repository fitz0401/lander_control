RTrun.sh使用方法：
	1在工控机打开roscore
	2在运动规划端开启运动规划执行程序（仅开启ros收发，不发具体命令，因为控制端还未执行）---首先确保network上最后一个有线打开，才能连接上
	3执行./RTrun.sh，这个命令会分别执行lander_plan,imu_driver,muti_receiver,以及数据显示端口
	4继续运动规划端命令，开始发布需要执行的命令，在命令执行期间会看到实时的电机运动显示
	5结束指令:新建终端执行： tmux kill-server
# 注意：依次执行各程序的原因在于：ros中的pub要先于sub建立，才能够确立连接。在建立完成后，只要roscore不关闭，pub和sub有断连直接重连就可不再需要考虑顺序。
