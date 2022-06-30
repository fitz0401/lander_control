# Lander

#### 介绍
着陆器项目

#### 控制系统架构与函数说明
初始化［init］：
经测试发现，12个elmo电机中，重新开关机后，4个主电机的编码器读数会改变，而8个辅电机的编码器读数一般不会改变．解决方法：每次重新开机后调用＂getpos＂函数得到电机位置读数，将这些读数与xml配置文件中的主电机＂pos_offset＂参数相加．
一定要先调用getpos，检查电机位置是否正确，再使用init．
每次调试结束关机前，都要调用init复位，以便于下次调试开始时检查位置是否正确．
调用格式：init

单电机运动［mvm］：
用于移动单个电机．
针对辅电机，单位为deg，移动多少度pos参数设置为多少即可，正值为向箱体外侧转动运动；
针对主电机，单位为mm，但要乘以比例系数57.3，负值为丝杠向上运动，足端下探．例如想让丝杠向上运动10cm，pos应设置为-5730．
调用格式：
mvm --pos=10 -m=1 -t=3 
mvm --pos=-5730 -m=3  -t=10

单腿仿导入仿真点运动［mvleg］：
本条指令依赖仿真点的设置，设置不当易造成速度和加速度不连续／电机掉使能的问题，待修改．
［注意插值函數的使用：初始化操作要在RT中執行，可以放在count=1中］
调用格式：mvleg

足端直线运动［mvline］：
本指令用于移动单条腿，给定足端位移（单位：mm），沿直线运动到目标点．采用梯形轨迹规划，设置运动速度约为1cm/s．每次執行後會記錄新的起始位置，因此可以實現幾條指令連續執行．全部指令執行完後，必須執行init，恢復初始位置．
也可以一起移動所有腿，參數爲-a，但需要保證四條腿初始位置都一模一樣．一般用於運行步態規劃軌跡前的初始化．
调用格式：
mvline -l=1 -x=30 -z=-50
mvline (-a) -z=-75

沿步態規劃的軌跡運動[mvplan]:
輸入規劃後的軌跡點，利用param.interval_time指定軌跡點間用時．程序會自動進行三次樣條插值，擬合足端在每1ms的軌跡．目前軌跡點暫存於build目錄下的＂PlanTrace＂文件內．
調用格式：
mvplan (-a) 所有腿都執行運動
mvplan -l=1 (讀入所有數據，但只執行1號腿)

电机限位：
利用xml文件实现，主电机［-150，0.01］，即丝杠长度；辅电机［-0.01，1.15］，此处单位为弧度，及0～60deg．


#### 安装教程

1.  xxxx
2.  xxxx
3.  xxxx

#### 注意事项

1.  xxxx
2.  xxxx
3.  xxxx

#### 参与贡献

1.  Fork 本仓库
2.  新建 Feat_xxx 分支
3.  提交代码
4.  新建 Pull Request


#### 特技

1.  使用 Readme\_XXX.md 来支持不同的语言，例如 Readme\_en.md, Readme\_zh.md
2.  Gitee 官方博客 [blog.gitee.com](https://blog.gitee.com)
3.  你可以 [https://gitee.com/explore](https://gitee.com/explore) 这个地址来了解 Gitee 上的优秀开源项目
4.  [GVP](https://gitee.com/gvp) 全称是 Gitee 最有价值开源项目，是综合评定出的优秀开源项目
5.  Gitee 官方提供的使用手册 [https://gitee.com/help](https://gitee.com/help)
6.  Gitee 封面人物是一档用来展示 Gitee 会员风采的栏目 [https://gitee.com/gitee-stars/](https://gitee.com/gitee-stars/)
