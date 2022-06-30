#include <array>
#include <stdlib.h>
#include <string>
#include <vector>
#include <bitset>
#include <fstream>
#include "CubicSplineNoDynamic.h"
#include "ControlPlan.h"
#include "GetPosIK.h"
#include "math.h"
#include "WalkLegExeLegIk.h"
#include "Param.h"
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "lander/mv_msgs.h"

using namespace aris::dynamic;
using namespace aris::plan;

namespace ControlPlan
{
    //指令的执行顺序
    //1、先执行prepareNrt，每条指令只执行一次
    //2、然后执行executeRT,executeRT每一个循环周期(默认1ms)会被实时核调用一次，执行的总时间由用户给定
    //3、执行结束后，本指令会被析构
    //指令功能：某一电机或者所有电机1-cos(theta)轨迹运行，幅值为pos，周期为time，周期数为timenum

    //get current pos
    struct GetPosPlanParam
    {
        std::vector<bool> active_motor;			//目标电机
        std::vector<double> begin_pjs;			//起始位置
        std::vector<double> step_pjs;			//目标位置
    };
    auto GetPosPlan::prepareNrt()->void
    {
        GetPosPlanParam param;
        param.active_motor.clear();
        param.active_motor.resize(controller()->motorPool().size(), false);
        param.begin_pjs.resize(controller()->motorPool().size(), 0.0);
        param.step_pjs.resize(controller()->motorPool().size(), 0.0);
        //解析指令参数
        for (auto &p : cmdParams())	{
                if (p.first == "all") {
                    std::fill(param.active_motor.begin(), param.active_motor.end(), true);
                }
                else if (p.first == "motion_id") {
                    param.active_motor.at(int32Param(p.first)) = true;
                }
            }
        this->param() = param;
        std::vector<std::pair<std::string, std::any>> ret_value;
        ret() = ret_value;
    }
    auto GetPosPlan::executeRT()->int
    {
        auto &param = std::any_cast<GetPosPlanParam&>(this->param());
        //第一个周期设置log文件名称，获取当前电机所在位置
        if (count() == 1){
            ecMaster()->logFileRawName("motion_replay");
            //for (Size i = 0; i < param.active_motor.size(); ++i) {
            for (Size i = 0; i < 12; ++i) {
                if (param.active_motor[i]) {
                    param.begin_pjs[i] = controller()->motorPool()[i].targetPos();
                    mout() << "motor " << i << " beginpos: " << std::setprecision(10) << param.begin_pjs[i] << "  "<<std::endl;
                }
            }
        }
        //返回0表示正常结束
        return 1 - count();
    }
    auto GetPosPlan::collectNrt()->void {
        double cur_pos[4][3];
        ifstream inFile("/home/kaanh/Desktop/Lander_ws/src/RobotParam", ios::in);
        if (!inFile.is_open()) {
            mout() << "Can not open the parameter file." << endl;
        }
        mout() << "Current position of four feet:" << endl;
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 3; j++) {
                inFile >> cur_pos[i][j];
                mout() << cur_pos[i][j] << " ";
            }
            mout() << endl;
        }
        inFile.close();
        ros::param::set("ifFinishFlag", true);
    }
    GetPosPlan::~GetPosPlan() = default;
    GetPosPlan::GetPosPlan(const std::string &name)
    {
        aris::core::fromXmlString(command(),
            "<Command name=\"getposplan\">"
            "	<GroupParam>"
            "		<UniqueParam default=\"all\">"\
            "			<Param name=\"all\" abbreviation=\"a\"/>"\
            "			<Param name=\"motion_id\" abbreviation=\"m\" default=\"0\"/>"
            "		</UniqueParam>"
            "	</GroupParam>"
            "</Command>");
    }

    //Initialize
    struct InitPlanParam
        {
            std::vector<bool> active_motor;			//目标电机
            std::vector<double> begin_pjs;			//起始位置
            std::vector<double> step_pjs;			//目标位置
            double initT[12];
            double maxT;
        };
    auto FindHomePlan::prepareNrt()->void
        {
            InitPlanParam param;
            param.active_motor.clear();
            param.active_motor.resize(controller()->motorPool().size(), false);
            param.begin_pjs.resize(controller()->motorPool().size(), 0.0);
            param.step_pjs.resize(controller()->motorPool().size(), 0.0);
            //解析指令参数
            for (auto &p : cmdParams())	{
                    if (p.first == "all") {
                        std::fill(param.active_motor.begin(), param.active_motor.end(), true);
                    }
                    else if (p.first == "motion_id") {
                        param.active_motor.at(int32Param(p.first)) = true;
                    }
                }
            this->param() = param;
            std::vector<std::pair<std::string, std::any>> ret_value;
            ret() = ret_value;
        }
    auto FindHomePlan::executeRT()->int
        {             
            auto time = static_cast<int32_t>(30000);
            auto totaltime = static_cast<int32_t>(15000);    //运行总时间
            auto &param = std::any_cast<InitPlanParam&>(this->param());
            //第一个周期设置log文件名称，获取当前电机所在位置
            if (count() == 1){
                ecMaster()->logFileRawName("motion_replay");
                //for (Size i = 0; i < param.active_motor.size(); ++i) {
                for (Size i = 0; i < 12; ++i) {
                    if (param.active_motor[i]) {
                        param.begin_pjs[i] = controller()->motorPool()[i].targetPos();
                        mout() << "motor " << i << " beginpos: " << std::setprecision(10) << param.begin_pjs[i] << "  "<<std::endl;
                    }
                }
            }
            for (Size i = 0; i < 12; ++i) {
                if (param.active_motor[i] && abs(param.begin_pjs[i]) > 1e-5) {
                    param.step_pjs[i] = param.begin_pjs[i] - (param.begin_pjs[i] / 2.0) * (1.0 - std::cos(2.0 * PI*count() / time));
                    controller()->motorPool().at(i).setTargetPos(param.step_pjs[i]);
                }
            }
            //返回0表示正常结束
            return totaltime - count();
        }
    auto FindHomePlan::collectNrt()->void {
        // 重置足端初始位置,文件寫入
        double initPos[4][3] = {{0.441840698, 0.0, -0.445142639},
                                {0.441840698, 0.0, -0.445142639},
                                {0.441840698, 0.0, -0.445142639},
                                {0.441840698, 0.0, -0.445142639}};
        ofstream outFile("/home/kaanh/Desktop/Lander_ws/src/RobotParam", ios::trunc);
        if(!outFile.is_open()){
            mout() << "Can not open the parameter file." << endl;
        }
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 2; j++) {
                outFile << initPos[i][j] << " ";
            }
            outFile << initPos[i][2] << endl;
        }
        outFile.close();
        mout() << "Finish Init" << endl;
        ros::param::set("ifFinishFlag", true); 
    }
    FindHomePlan::~FindHomePlan() = default;
    FindHomePlan::FindHomePlan(const std::string &name)
    {
        //构造函数参数说明，构造函数通过xml的格式定义本条指令的接口，name表示参数名，default表示输入参数，abbreviation表示参数名的缩写(缩写只能单个字符)
        //1 GroupParam下面的各个节点都是输入参数，如果没有给定会使用默认值
        //2 UniqueParam下面的各个节点互斥，有且只能使用其中的一个
        //3 例如，通过terminal或者socket发送“mvs --pos=0.1”，控制器实际会按照mvs --pos=0.1rad --time=1s --timenum=2 --all执行
        aris::core::fromXmlString(command(),
            "<Command name=\"initplan\">"
            "	<GroupParam>"
            "		<UniqueParam default=\"all\">"\
            "			<Param name=\"all\" abbreviation=\"a\"/>"\
            "			<Param name=\"motion_id\" abbreviation=\"m\" default=\"0\"/>"
            "		</UniqueParam>"
            "	</GroupParam>"
            "</Command>");
    }

    struct PlanFootParam
    {
        std::vector<bool> active_motor;			//目标电机
        std::vector<double> begin_pjs;			//起始位置
        std::vector<double> step_pjs;			//目标位置

        double init_pos[4][3];      
        //記錄四條腿末端在程序開始執行時的位置，並在執行結束後進行更新
        double begin_pos[3];        //從init_pos中取出目標腿末端的初始位置
        int legIndex;
        double x, y, z;
        double distance;

        myGetPosIK myPos;
        int selectIndex[3] = {1,1,2};
        double d1_ori, theta2_ori, theta3_ori;
        double d1, theta2, theta3;

        Size totaltime;
    };
    auto PlanFoot::prepareNrt()->void
    {      
        PlanFootParam param;
        param.active_motor.clear();
        param.active_motor.resize(controller()->motorPool().size(), false);
        param.begin_pjs.resize(controller()->motorPool().size(), 0.0);
        param.step_pjs.resize(controller()->motorPool().size(), 0.0);

        param.legIndex = 0;
        param.d1 = 0.0;     param.theta2 = 0.0;     param.theta3 = 0.0;
        param.d1_ori = 0.0; param.theta2_ori = 0.0; param.theta3_ori = 0.0;
        param.x = 0.0;      param.y = 0.0;          param.z = 0.0;
        
        ros::param::get("leg_index", param.legIndex);
        ros::param::get("x_motion", param.x);
        ros::param::get("y_motion", param.y);
        ros::param::get("z_motion", param.z);
        //解析输入参数
        if (param.legIndex == 12){
            std::fill(param.active_motor.begin(), param.active_motor.end(), true);
        }
        else {
            param.active_motor.at(3 * param.legIndex) = true;
            param.active_motor.at(3 * param.legIndex + 1) = true;
            param.active_motor.at(3 * param.legIndex + 2) = true;
        }
        // 從文件中讀取電機初始位置
        ifstream inFile("/home/kaanh/Desktop/Lander_ws/src/RobotParam", ios::in);
        if (!inFile.is_open()) {
            mout() << "Can not open the parameter file." << endl;
        }
        mout() << "Start position of four feet:" << endl;
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 3; j++) {
                inFile >> param.init_pos[i][j];
                mout() << param.init_pos[i][j] << " ";
            }
            mout() << endl;
        }
        inFile.close();

        param.begin_pos[0] = param.init_pos[param.legIndex % 4][0];
        param.begin_pos[1] = param.init_pos[param.legIndex % 4][1];
        param.begin_pos[2] = param.init_pos[param.legIndex % 4][2];
        param.distance = sqrt(pow(param.x, 2) + pow(param.y, 2) + pow(param.z, 2));
        mout() << "distance(mm): " << 1000 * param.distance << endl;
        mout() << "legIndex: " << param.legIndex << endl;
        this->param() = param;
        std::vector<std::pair<std::string, std::any>> ret_value;
        ret() = ret_value;
    }
    auto PlanFoot::executeRT()->int
    {
        auto &param = std::any_cast<PlanFootParam&>(this->param());
        // 所有腿都運動
        Size begin_num, end_num;
        if (param.legIndex == 12) {
            begin_num = 0;
            end_num = 12;
        }
        // 單腿運動
        else{
            begin_num = param.legIndex * 3;
            end_num = param.legIndex * 3 + 3;
        }
        double next_pos, next_vel, next_acc;
        //第一个周期设置log文件名称，获取当前电机所在位置
        if (count() == 1){
            ecMaster()->logFileRawName("20220224_test02");
            for (Size i = begin_num; i < end_num; ++i) {
                if (param.active_motor[i]) {
                    param.begin_pjs[i] = controller()->motorPool()[i].targetPos();
                    mout() << "begin_pjs" << i << ":" << param.begin_pjs[i] << endl;
                    param.myPos.fromS1GetMotorAngle(param.begin_pos, param.selectIndex, param.d1_ori, param.theta2_ori, param.theta3_ori);
                }
            }
            moveAbsolute(count(), 0, param.distance, 0.00001, 0.000001, 0.000001, next_pos, next_vel, next_acc, param.totaltime);
            mout() << "totaltime(ms): " << param.totaltime << endl;
        }

        moveAbsolute(count(), 0, param.distance, 0.00001, 0.000001, 0.000001, next_pos, next_vel, next_acc, param.totaltime);
        double end_point[3] = {param.begin_pos[0] + next_pos / param.distance * param.x,
                               param.begin_pos[1] + next_pos / param.distance * param.y,
                               param.begin_pos[2] + next_pos / param.distance * param.z};
        param.myPos.fromS1GetMotorAngle(end_point, param.selectIndex, param.d1, param.theta2, param.theta3);
        for(Size i = begin_num; i < end_num; ++i) {
            if(!param.active_motor[i]) return 0;
        }

        for(Size i = begin_num; i < end_num; i += 3) {
            // 主電機
            param.step_pjs[i] = param.begin_pjs[i] + 1000 * (param.d1 - param.d1_ori);
            controller()->motorPool().at(i).setTargetPos(param.step_pjs[i]);
            // 左輔電機
            param.step_pjs[i + 1] = param.begin_pjs[i + 1] + (param.theta2 - param.theta2_ori);
            controller()->motorPool().at(i + 1).setTargetPos(param.step_pjs[i + 1]);
            // 右輔電機
            param.step_pjs[i + 2] = param.begin_pjs[i + 2] + (param.theta3_ori - param.theta3);
            controller()->motorPool().at(i + 2).setTargetPos(param.step_pjs[i + 2]);
        }

        //打印
        if(count() % 1000 == 0){
            mout() << "next_pos(mm): " << std::setprecision(10) << 1000 * next_pos << endl;
            mout() << "end_x:" << std::setprecision(5) << end_point[0] << " "
                   << "end_y:" << std::setprecision(5) << end_point[1] << " "
                   << "end_z:" << std::setprecision(5) << end_point[2] << " " << endl;
        }
      
        return param.totaltime - count();
    }
    auto PlanFoot::collectNrt()->void {
        auto &param = std::any_cast<PlanFootParam&>(this->param());
        // 更新全局參數
        if (param.legIndex == 12) {
            for (int i = 0; i < 4; i++)     param.init_pos[i][0] += param.x;
            for (int i = 0; i < 4; i++)     param.init_pos[i][1] += param.y;
            for (int i = 0; i < 4; i++)     param.init_pos[i][2] += param.z;
        }
        else {
            param.init_pos[param.legIndex][0] += param.x;
            param.init_pos[param.legIndex][1] += param.y;
            param.init_pos[param.legIndex][2] += param.z;
        }
        ofstream outFile("/home/kaanh/Desktop/Lander_ws/src/RobotParam", ios::trunc);
        if(!outFile.is_open()){
            mout() << "Can not open the parameter file." << endl;
        }
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 2; j++) {
                outFile << param.init_pos[i][j] << " ";
            }
            outFile << param.init_pos[i][2] << endl;
        }
        outFile.close();
        mout() << "Finish motion." << endl;
        ros::param::set("ifFinishFlag", true); 
    }
    PlanFoot::~PlanFoot() = default;
    PlanFoot::PlanFoot(const std::string &name)
    {
        //构造函数参数说明，构造函数通过xml的格式定义本条指令的接口，name表示参数名，default表示输入参数，abbreviation表示参数名的缩写(缩写只能单个字符)
        //1 GroupParam下面的各个节点都是输入参数，如果没有给定会使用默认值
        //2 UniqueParam下面的各个节点互斥，有且只能使用其中的一个
        //3 例如，通过terminal或者socket发送“mvs --pos=0.1”，控制器实际会按照mvs --pos=0.1rad --time=1s --timenum=2 --all执行
        aris::core::fromXmlString(command(),
            "<Command name=\"planfoot\">"
            "	<GroupParam>"
            "	</GroupParam>"
            "</Command>");
    }

    struct PlanMotionParam
    {
        std::vector<bool> active_motor;			//目标电机
        std::vector<double> begin_pjs;			//起始位置
        std::vector<double> step_pjs;			//目标位置
        
        // 目标运动轨迹
        int data_num;
        CubicSpline cs0;
        CubicSpline cs1;
        CubicSpline cs2;
        CubicSpline cs3;
        CubicSpline cs4;
        CubicSpline cs5;
        CubicSpline cs6;
        CubicSpline cs7;
        CubicSpline cs8;
        CubicSpline cs9;
        CubicSpline cs10;
        CubicSpline cs11;
        // 输入轨迹点数上限为100
        double legTrace[12][100];
        double deltaT[100];

        double interval_time;
        double init_pos[4][3];      //記錄四條腿末端在程序開始執行時的位置，並在執行結束後進行更新
        double begin_pos[4][3];        //從init_pos中取出目標腿末端的初始位置

        // 運動學反解參數
        myGetPosIK myPos[4];
        int selectIndex[3] = {1,1,2};
        double d1_ori[4] = {0.0}, theta2_ori[4] = {0.0}, theta3_ori[4] = {0.0};
        double d1[4] = {0.0}, theta2[4] = {0.0}, theta3[4] = {0.0};
        // 總執行時間
        double totalT;
        Size totaltime;
    };
    auto PlanMotion::prepareNrt()->void
    {
        PlanMotionParam param;
        param.active_motor.clear();
        param.active_motor.resize(controller()->motorPool().size(), false);
        param.begin_pjs.resize(controller()->motorPool().size(), 0.0);
        param.step_pjs.resize(controller()->motorPool().size(), 0.0);

        // 从ROS全局参数中获取通讯系统传来的指令
        ros::param::get("data_num", param.data_num);
        std::fill(param.active_motor.begin(), param.active_motor.end(), true);

        // 每一步用時0.5s,可在此處修改
        param.interval_time = 0.5;
        param.totaltime = param.interval_time * 1000 * param.data_num;
        param.totalT = static_cast<double>(param.totaltime);
        for (int i = 0; i < param.data_num; i++)    param.deltaT[i] = param.interval_time * i;

        // 從文件中讀取足端初始位置
        ifstream inFile1("/home/kaanh/Desktop/Lander_ws/src/RobotParam", ios::in);
        if (!inFile1.is_open()) {
            mout() << "Can not open the parameter file." << endl;
            return;
        }
        mout() << "Start position of four feet:" << endl;
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 3; j++) {
                inFile1 >> param.init_pos[i][j];
                param.begin_pos[i][j] = param.init_pos[i][j];
            }
            mout() << endl;
        }
        inFile1.close();

        // 从文件中读取目标轨迹
        ifstream inFile2("/home/kaanh/Desktop/Lander_ws/src/ROSPlanTrace", ios::in);
        if (!inFile2.is_open()) {
            mout() << "Can not open the ROSPlanTrace file." << endl;
            return;
        }
        for (int i = 0; i < 12; i++) {
            for (int j = 0; j < param.data_num; j++) {
                inFile2 >> param.legTrace[i][j];
            }
        }
        inFile2.close();

        this->param() = param;
        std::vector<std::pair<std::string, std::any>> ret_value;
        ret() = ret_value;
    }
    auto PlanMotion::executeRT()->int
    {
        auto &param = std::any_cast<PlanMotionParam&>(this->param());
        // 所有腿都運動
        Size begin_num = 0;     Size end_num = 12;
        // 第一个周期设置log文件名称，获取当前电机所在位置; 初始化插值函數
        if (count() == 1){
            // 插值函数初始化    
            param.cs0.Initialize(param.deltaT, param.legTrace[0], param.data_num);
            param.cs1.Initialize(param.deltaT, param.legTrace[1], param.data_num);
            param.cs2.Initialize(param.deltaT, param.legTrace[2], param.data_num);
            param.cs3.Initialize(param.deltaT, param.legTrace[3], param.data_num);
            param.cs4.Initialize(param.deltaT, param.legTrace[4], param.data_num);
            param.cs5.Initialize(param.deltaT, param.legTrace[5], param.data_num);
            param.cs6.Initialize(param.deltaT, param.legTrace[6], param.data_num);
            param.cs7.Initialize(param.deltaT, param.legTrace[7], param.data_num);
            param.cs8.Initialize(param.deltaT, param.legTrace[8], param.data_num);
            param.cs9.Initialize(param.deltaT, param.legTrace[9], param.data_num);
            param.cs10.Initialize(param.deltaT, param.legTrace[10], param.data_num);
            param.cs11.Initialize(param.deltaT, param.legTrace[11], param.data_num);
            ecMaster()->logFileRawName("20220224_test02");
             // 初始化電機初始位置［電機坐標系中位置，控制信號用］
            for (Size i = begin_num; i < end_num; ++i) {
                if (param.active_motor[i]) {
                    param.begin_pjs[i] = controller()->motorPool()[i].targetPos();
                    mout() << "begin_pjs" << i << ":" << param.begin_pjs[i] << endl;
                }
            }
            // 初始化電機初始位置［運動學反解坐標系中位置］
            for (int i = 0; i < 4; i++) {
                param.myPos[i].fromS1GetMotorAngle(param.begin_pos[i], param.selectIndex, param.d1_ori[i], param.theta2_ori[i], param.theta3_ori[i]);
            }
        }
        // 把0-1s映射到0-(interval_time * data_num)s, 注意單位爲m
        double time_scale = param.interval_time * param.data_num / param.totalT;    // 當前時間在總時間中的佔比
        double end_point[4][3] = {{param.cs0.Interpolate(time_scale * count()) / 1000.0,
                                   param.cs1.Interpolate(time_scale * count()) / 1000.0,
                                   param.cs2.Interpolate(time_scale * count()) / 1000.0},
                                  {param.cs3.Interpolate(time_scale * count()) / 1000.0,
                                   param.cs4.Interpolate(time_scale * count()) / 1000.0,
                                   param.cs5.Interpolate(time_scale * count()) / 1000.0},
                                  {param.cs6.Interpolate(time_scale * count()) / 1000.0,
                                   param.cs7.Interpolate(time_scale * count()) / 1000.0,
                                   param.cs8.Interpolate(time_scale * count()) / 1000.0},
                                  {param.cs9.Interpolate(time_scale * count()) / 1000.0,
                                   param.cs10.Interpolate(time_scale * count()) / 1000.0,
                                   param.cs11.Interpolate(time_scale * count()) / 1000.0}};
        // 運動學反解
        for (int i = 0; i < 4; i++) {
            param.myPos[i].fromS1GetMotorAngle(end_point[i], param.selectIndex, param.d1[i], param.theta2[i], param.theta3[i]);
        }
        // 電機執行反解結果
        for(Size i = begin_num; i < end_num; ++i) {
            if(!param.active_motor[i]) return 0;
        }
        for(Size i = begin_num; i < end_num; i += 3) {
            // 主電機; i / 3爲腿的序號
            param.step_pjs[i] = param.begin_pjs[i] + 1000 * (param.d1[i / 3] - param.d1_ori[i / 3]);
            controller()->motorPool().at(i).setTargetPos(param.step_pjs[i]);
            // 左輔電機
            param.step_pjs[i + 1] = param.begin_pjs[i + 1] + (param.theta2[i / 3] - param.theta2_ori[i / 3]);
            controller()->motorPool().at(i + 1).setTargetPos(param.step_pjs[i + 1]);
            // 右輔電機
            param.step_pjs[i + 2] = param.begin_pjs[i + 2] + (param.theta3_ori[i / 3] - param.theta3[i / 3]);
            controller()->motorPool().at(i + 2).setTargetPos(param.step_pjs[i + 2]);
        }
        // 打印
        if(count() % 5000 == 0){
            mout() << "end_x_leg1:" << std::setprecision(5) << end_point[0][0] << " "
                   << "end_y_leg1:" << std::setprecision(5) << end_point[0][1] << " "
                   << "end_z_leg1:" << std::setprecision(5) << end_point[0][2] << " " << endl;
            mout() << "end_x_leg2:" << std::setprecision(5) << end_point[1][0] << " "
                   << "end_y_leg2:" << std::setprecision(5) << end_point[1][1] << " "
                   << "end_z_leg2:" << std::setprecision(5) << end_point[1][2] << " " << endl;
            mout() << "end_x_leg3:" << std::setprecision(5) << end_point[2][0] << " "
                   << "end_y_leg3:" << std::setprecision(5) << end_point[2][1] << " "
                   << "end_z_leg3:" << std::setprecision(5) << end_point[2][2] << " " << endl;
            mout() << "end_x_leg4:" << std::setprecision(5) << end_point[3][0] << " "
                   << "end_y_leg4:" << std::setprecision(5) << end_point[3][1] << " "
                   << "end_z_leg4:" << std::setprecision(5) << end_point[3][2] << " " << endl;
        }
        // 更新全局參數
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 3; j++) {
                param.init_pos[i][j] = end_point[i][j];
            }
        }   
        //返回0表示正常结束，返回负数表示报错，返回正数表示正在执行
        return param.totaltime - count();
    }
    auto PlanMotion::collectNrt()->void {
        auto &param = std::any_cast<PlanMotionParam&>(this->param());
        // 将全局参数输出到记录文件中
        ofstream outFile("/home/kaanh/Desktop/Lander_ws/src/RobotParam", ios::trunc);
        if(!outFile.is_open()){
            mout() << "Can not open the parameter file." << endl;
        }
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 2; j++) {
                outFile << param.init_pos[i][j] << " ";
            }
            outFile << param.init_pos[i][2] << endl;
        }
        outFile.close();
        mout() << "Finish motion." << endl;
        ros::param::set("ifFinishFlag", true); 
    }
    PlanMotion::~PlanMotion() = default;
    PlanMotion::PlanMotion(const std::string &name)
    {
        //构造函数参数说明，构造函数通过xml的格式定义本条指令的接口，name表示参数名，default表示输入参数，abbreviation表示参数名的缩写(缩写只能单个字符)
        //1 GroupParam下面的各个节点都是输入参数，如果没有给定会使用默认值
        //2 UniqueParam下面的各个节点互斥，有且只能使用其中的一个
        //3 例如，通过terminal或者socket发送“mvs --pos=0.1”，控制器实际会按照mvs --pos=0.1rad --time=1s --timenum=2 --all执行
        aris::core::fromXmlString(command(),
            "<Command name=\"planmotion\">"
            "</Command>");
    }

	ARIS_REGISTRATION
	{
        aris::core::class_<GetPosPlan>("GetPosPlan")
            .inherit<Plan>()
            ;

        aris::core::class_<FindHomePlan>("FindHomePlan")
            .inherit<Plan>()
            ;

        aris::core::class_<PlanFoot>("PlanFoot")
        .inherit<Plan>()
        ;

        aris::core::class_<PlanMotion>("PlanMotion")
        .inherit<Plan>()
        ;
	}

	auto createPlanRoot()->std::unique_ptr<aris::plan::PlanRoot>
	{
		std::unique_ptr<aris::plan::PlanRoot> plan_root(new aris::plan::PlanRoot);
		plan_root->planPool().add<aris::plan::Disable>();
		plan_root->planPool().add<aris::plan::Mode>();
        plan_root->planPool().add<aris::plan::Enable>();
		plan_root->planPool().add<aris::plan::Recover>();
		plan_root->planPool().add<aris::plan::Show>();
		plan_root->planPool().add<aris::plan::Clear>();	

        plan_root->planPool().add<ControlPlan::GetPosPlan>();
        plan_root->planPool().add<ControlPlan::FindHomePlan>();
        plan_root->planPool().add<ControlPlan::PlanFoot>();
        plan_root->planPool().add<ControlPlan::PlanMotion>();
		return plan_root;
	}
}
