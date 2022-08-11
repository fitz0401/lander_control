#include <array>
#include <stdlib.h>
#include <string>
#include <vector>
#include <bitset>
#include <fstream>
#include <numeric>
#include "CubicSplineNoDynamic.h"
#include "ControlPlan.h"
#include "GetPosIK.h"
#include "math.h"
#include "WalkLegExeLegIk.h"
#include "Param.h"
#include <ros/ros.h>
#include "std_msgs/String.h"

using namespace aris::dynamic;
using namespace aris::plan;

namespace ControlPlan
{  
    // 存储状态数据
    struct StateParam
    {
        double position[12];
        double velocity[12];
        double effort[12];
    };
    // 传输状态数据的管道，在.h文件中声明，在当前文件中被首先定义
    aris::core::Pipe pipe_global_stateMsg;

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
        ros::param::set("isFinishFlag", true);
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
        // 0.441840698, 0.0, -0.445142639  短腿版本
        // 0.5368450, 0.0, -0.5980247 最长腿
        double initPos[4][3] = {{0.4941750, 0.0, -0.5303521},
                                {0.4941750, 0.0, -0.5303521},
                                {0.4941750, 0.0, -0.5303521},
                                {0.4941750, 0.0, -0.5303521}};
        ofstream outFile("/home/kaanh/Desktop/Lander_ws/src/RobotParam", ios::trunc);
        if(!outFile.is_open()){
            mout() << "Can not open the parameter file." << endl;
        }
        outFile.setf(ios::fixed);
        outFile.precision(6);
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 2; j++) {
                outFile << initPos[i][j] << " ";
            }
            outFile << initPos[i][2] << endl;
        }
        outFile.close();
        mout() << "Finish Init" << endl;
        ros::param::set("isFinishFlag", true); 
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
        double begin_pos[4][3];        //從init_pos中取出目標腿末端的初始位置
        int legIndex;
        Size begin_motor, end_motor;
        aris::core::Matrix end_mat;
        double distance[4];

        myGetPosIK myPos[4];
        int selectIndex[3] = {1,1,2};
        double d1_ori[4] = {0.0}, theta2_ori[4] = {0.0}, theta3_ori[4] = {0.0};
        double d1[4] = {0.0}, theta2[4] = {0.0}, theta3[4] = {0.0};

        Size totaltime[4];
        Size max_time;

        // 反馈控制相关参数
        // 函数返回值为一个4*3的四足末端位置数组
        vector<vector<double>> ret_value;
        // 缓存500ms内的力矩信息，用于滤波
        double main_toq[500] = {0.0};
        double sub_toq_left[500] = {0.0};
        double sub_toq_right[500] = {0.0};
    };
    auto PlanFoot::prepareNrt()->void
    {      
        PlanFootParam param;
        param.active_motor.clear();
        param.active_motor.resize(controller()->motorPool().size(), false);
        param.begin_pjs.resize(controller()->motorPool().size(), 0.0);
        param.step_pjs.resize(controller()->motorPool().size(), 0.0);
        param.max_time = 0;

        ros::param::get("leg_index", param.legIndex);
        mout() << "legIndex: " << param.legIndex << endl;
        // 所有腿都運動
        if (param.legIndex == 12) {
            param.begin_motor = 0;
            param.end_motor = 12;
            std::fill(param.active_motor.begin(), param.active_motor.end(), true);
        }
        // 單腿運動
        else{
            param.begin_motor = param.legIndex * 3;
            param.end_motor = param.legIndex * 3 + 3;
            param.active_motor.at(3 * param.legIndex) = true;
            param.active_motor.at(3 * param.legIndex + 1) = true;
            param.active_motor.at(3 * param.legIndex + 2) = true;
        }

        //解析输入参数
        for (auto &p : cmdParams())	{
            if (p.first == "end_mat") {
                param.end_mat = matrixParam(p.first);
            }
        }
        // 四条腿的位移
        for (int i = 0; i < 4; i++){
            if (param.legIndex == 12 || param.legIndex == i) {
                param.distance[i] = sqrt(pow(param.end_mat(i, 0), 2) 
                + pow(param.end_mat(i, 1), 2) + pow(param.end_mat(i, 2), 2));
                mout() << "distance" << i << "(mm): " << 1000 * param.distance[i] << endl;
            }
        }

        // 從文件中讀取電機初始位置
        ifstream inFile("/home/kaanh/Desktop/Lander_ws/src/RobotParam", ios::in);
        if (!inFile.is_open()) {
            mout() << "Can not open the parameter file." << endl;
        }
        inFile.setf(ios::fixed);
        inFile.precision(6);
        mout() << "Start position of four feet:" << endl;
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 3; j++) {
                inFile >> param.init_pos[i][j];
                mout() << param.init_pos[i][j] << " ";
                param.begin_pos[i][j] = param.init_pos[i][j];
            }
            mout() << endl;
        }
        inFile.close();
    
        this->param() = param;
        std::vector<std::pair<std::string, std::any>> ret_value;
        ret() = ret_value;
    }
    auto PlanFoot::executeRT()->int
    {
        auto &param = std::any_cast<PlanFootParam&>(this->param());
        double next_pos[4], next_vel[4], next_acc[4];
        //第一个周期设置log文件名称，获取当前电机所在位置
        if (count() == 1){
            // 初始化電機初始位置［電機坐標系中位置，控制信號用］
            for (Size i = param.begin_motor; i < param.end_motor; ++i) {
                if (param.active_motor[i]) {
                    param.begin_pjs[i] = controller()->motorPool()[i].targetPos();
                    mout() << "begin_pjs" << i << ":" << param.begin_pjs[i] << endl;
                }
            }
            // 初始化電機初始位置［運動學反解坐標系中位置］
            for (int i = 0; i < 4; i++) {
                param.myPos[i].fromS1GetMotorAngle(param.begin_pos[i], param.selectIndex, param.d1_ori[i], param.theta2_ori[i], param.theta3_ori[i]);
                moveAbsolute(count(), 0, param.distance[i], 0.00001, 0.000001, 0.000001, next_pos[i], next_vel[i], next_acc[i], param.totaltime[i]);
                if(param.totaltime[i] > param.max_time)    param.max_time = param.totaltime[i];
            }         
            mout() << "max_time(ms): " << param.max_time << endl;
        }
        
        // 运动学反解结果
        double end_point[4][3] = {0.0};
        if(param.legIndex == 12) {
            for (int i = 0; i < 4; i++) {
                moveAbsolute(count(), 0, param.distance[i], 0.00001, 0.000001, 0.000001, next_pos[i], next_vel[i], next_acc[i], param.totaltime[i]);
                for(int j = 0; j < 3; j++) {
                    end_point[i][j] = param.begin_pos[i][j] + next_pos[i] / param.distance[i] * param.end_mat(i, j);
                }
                param.myPos[i].fromS1GetMotorAngle(end_point[i], param.selectIndex, param.d1[i], param.theta2[i], param.theta3[i]);
            }
        }
        else {
            moveAbsolute(count(), 0, param.distance[param.legIndex], 0.00001, 0.000001, 0.000001, next_pos[param.legIndex], next_vel[param.legIndex], next_acc[param.legIndex], param.totaltime[param.legIndex]);
            for(int j = 0; j < 3; j++) {
                end_point[param.legIndex][j] = param.begin_pos[param.legIndex][j] + next_pos[param.legIndex] / param.distance[param.legIndex] * param.end_mat(param.legIndex, j);
            }
            param.myPos[param.legIndex].fromS1GetMotorAngle(end_point[param.legIndex], param.selectIndex, param.d1[param.legIndex], param.theta2[param.legIndex], param.theta3[param.legIndex]);
        }
        
        // 电机执行程序
        for(Size i = param.begin_motor; i < param.end_motor; ++i) {
            if(!param.active_motor[i]) return 0;
        }
        for(Size i = param.begin_motor; i < param.end_motor; i += 3) {
            // 由于四条腿执行时长不一致，需要判断当前电机是否运动完毕
            if(count() <= param.totaltime[i / 3]) {
                // 主電機; i / 3爲腿的序號; i = 0,3,6,9
                param.step_pjs[i] = param.begin_pjs[i] + 1000 * (param.d1[i / 3] - param.d1_ori[i / 3]);
                controller()->motorPool().at(i).setTargetPos(param.step_pjs[i]);
                // 左輔電機
                param.step_pjs[i + 1] = param.begin_pjs[i + 1] + (param.theta2[i / 3] - param.theta2_ori[i / 3]);
                controller()->motorPool().at(i + 1).setTargetPos(param.step_pjs[i + 1]);
                // 右輔電機
                param.step_pjs[i + 2] = param.begin_pjs[i + 2] + (param.theta3_ori[i / 3] - param.theta3[i / 3]);
                controller()->motorPool().at(i + 2).setTargetPos(param.step_pjs[i + 2]);
            }   
        }
        
        // 向主进程发送状态信息
        StateParam state_param;
        for(Size i = 0; i < 12; ++i) {
            // 主电机位置信息为丝杠当前位置，单位：mm
            if(i % 3 == 0) {
                state_param.position[i] = controller()->motorPool()[i].actualPos();
            }
            // 辅电机位置信息为辅电机角度，单位：°
            else {
                state_param.position[i] = controller()->motorPool()[i].actualPos() / PI * 180.0;
            }
            state_param.velocity[i] = controller()->motorPool()[i].actualVel();
            state_param.effort[i] = controller()->motorPool()[i].actualToq();
        }
        aris::core::Msg state_msg;
        state_msg.copyStruct(state_param);
        pipe_global_stateMsg.sendMsg(state_msg);
        
        return param.max_time - count();
    }
    auto PlanFoot::collectNrt()->void {
        auto &param = std::any_cast<PlanFootParam&>(this->param());
        // 更新全局參數
        for (int i = 0; i < 4; i++) {
            if (param.legIndex == 12 || param.legIndex == i) {
                for (int j = 0; j < 3; j++) {
                    param.init_pos[i][j] += param.end_mat(i ,j);
                }
            }
        }
        // 输出全局参数到文件中
        ofstream outFile("/home/kaanh/Desktop/Lander_ws/src/RobotParam", ios::trunc);
        if(!outFile.is_open()){
            mout() << "Can not open the parameter file." << endl;
        }
        outFile.setf(ios::fixed);
        outFile.precision(6);
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 2; j++) {
                outFile << param.init_pos[i][j] << " ";
            }
            outFile << param.init_pos[i][2] << endl;
        }
        outFile.close();
        mout() << "Finish motion." << endl;
        ros::param::set("isFinishFlag", true); 
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
            "		<Param name=\"end_mat\" abbreviation=\"end\"/>"   
            "</Command>");
    }

    struct PlanMotionParam
    {
        std::vector<double> begin_pjs;			//起始位置
        std::vector<double> target_pjs;			//目标位置
        
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
        aris::core::Matrix trace_mat;
        double legTrace[12][100];
        double deltaT[100];
        double interval_time;
        double init_pos[4][3];      //記錄四條腿末端在程序開始執行時的位置，並在執行結束後進行更新
        double begin_pos[4][3];     //從init_pos中取出目標腿末端的初始位置

        // 運動學反解參數
        myGetPosIK myPos;
        int selectIndex[3] = {1,1,2};
        double d1_ori[4], theta2_ori[4], theta3_ori[4];
        double d1[4], theta2[4], theta3[4];
        // 總執行時間
        Size totaltime;
        // 运动腿下标，用于触地检测反馈
        int leg_index;
        // 函数返回值为一个4*3的四足末端位置数组
        vector<vector<double>> ret_value;
        // 缓存500ms内的力矩信息，用于滤波
        double main_toq[500] = {0.0};
        double sub_toq_left[500] = {0.0};
        double sub_toq_right[500] = {0.0};
    };
    auto PlanMotion::prepareNrt()->void
    {
        PlanMotionParam param;
        param.begin_pjs.resize(controller()->motorPool().size(), 0.0);
        param.target_pjs.resize(controller()->motorPool().size(), 0.0);

        // 从ros中获取通讯系统传来的指令，通过解析指令参数实现
        for (auto &p : cmdParams())	{
            if (p.first == "trace_mat") {
                param.trace_mat = matrixParam(p.first);
            }
        }
        ros::param::get("data_num", param.data_num);
        mout() << "data_num: " << param.data_num << endl;

        // 每一步用時0.5s,可在此處修改
        param.interval_time = 0.5;
        param.totaltime = param.interval_time * 1000 * (param.data_num - 1);
        mout() << "totaltime: " << param.totaltime << endl;
        
        // 对插值变量x进行赋值
        for (int i = 0; i < param.data_num; i++)    param.deltaT[i] = param.interval_time * 1000.0 * i;

        // 获得目标轨迹
        for (int i = 0; i < 12; i++) {
            for (int j = 0; j < param.data_num; j++) {
                param.legTrace[i][j] = param.trace_mat(i, j) / 1000.0;
            }
        }
        // 获得运动学反解的初始位置,取出param.legTrace的第一列付给4*3的矩阵
        mout() << "Start position of four feet:" << endl;
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 3; j++){
                param.begin_pos[i][j] = param.legTrace[i * 3 + j][0];
                mout() << param.begin_pos[i][j] << " ";
            }
            mout() << endl;
        }

        // 初始化電機初始位置［運動學反解坐標系中位置］
        for (Size i = 0; i < 4; i++) {
            param.myPos.fromS1GetMotorAngle(param.begin_pos[i], param.selectIndex, param.d1_ori[i], param.theta2_ori[i], param.theta3_ori[i]);
        }
        this->param() = param;
        std::vector<std::pair<std::string, std::any>> ret_value;
        ret() = ret_value;
    }
    auto PlanMotion::executeRT()->int
    {
        auto &param = std::any_cast<PlanMotionParam&>(this->param());
        // 第一个周期设置log文件名称，获取当前电机所在位置; 初始化插值函數
        if (count() == 1){
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
            ecMaster()->logFileRawName("motion_plan_test");
            // 初始化電機初始位置［電機坐標系中位置，控制信號用］
            for (Size i = 0; i < 12; ++i) {
                param.begin_pjs[i] = controller()->motorPool()[i].targetPos();
                //mout() << "begin_pjs" << i << ":" << param.begin_pjs[i] << endl;
            }
        }
        double end_point[4][3] = {{param.cs0.Interpolate(1.0 * count()),
                                   param.cs1.Interpolate(1.0 * count()),
                                   param.cs2.Interpolate(1.0 * count())},
                                  {param.cs3.Interpolate(1.0 * count()),
                                   param.cs4.Interpolate(1.0 * count()),
                                   param.cs5.Interpolate(1.0 * count())},
                                  {param.cs6.Interpolate(1.0 * count()),
                                   param.cs7.Interpolate(1.0 * count()),
                                   param.cs8.Interpolate(1.0 * count())},
                                  {param.cs9.Interpolate(1.0 * count()),
                                   param.cs10.Interpolate(1.0 * count()),
                                   param.cs11.Interpolate(1.0 * count())}};
        
        // 運動學反解
        for (int i = 0; i < 4; i++) {
            param.myPos.fromS1GetMotorAngle(end_point[i], param.selectIndex, param.d1[i], param.theta2[i], param.theta3[i]);
        }
        // 電機執行反解結果
        for(Size i = 0; i < 12; i += 3) {
            // 主電機; i / 3爲腿的序號
            param.target_pjs[i] = param.begin_pjs[i] + 1000 * (param.d1[i / 3] - param.d1_ori[i / 3]);
            controller()->motorPool().at(i).setTargetPos(param.target_pjs[i]);
            // 左輔電機
            param.target_pjs[i + 1] = param.begin_pjs[i + 1] + (param.theta2[i / 3] - param.theta2_ori[i / 3]);
            controller()->motorPool().at(i + 1).setTargetPos(param.target_pjs[i + 1]);
            // 右輔電機
            param.target_pjs[i + 2] = param.begin_pjs[i + 2] + (param.theta3_ori[i / 3] - param.theta3[i / 3]);
            controller()->motorPool().at(i + 2).setTargetPos(param.target_pjs[i + 2]);
        }

        //打印
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
        outFile.setf(ios::fixed);
        outFile.precision(6);
        for (int i = 0; i < 4; i++) {
            outFile << param.legTrace[3 * i][param.data_num - 1] << " ";
            outFile << param.legTrace[3 * i + 1][param.data_num - 1] << " ";
            outFile << param.legTrace[3 * i + 2][param.data_num - 1] << endl;
        }
        outFile.close();
        mout() << "Finish plan motion." << endl;
        ros::param::set("isFinishFlag", true); 
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
            "		<Param name=\"trace_mat\" abbreviation=\"mat\"/>"
            "</Command>");
    }

    struct PlanAdjustParam
    {
        std::vector<double> begin_pjs;			//起始位置
        std::vector<double> target_pjs;			//目标位置
        
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
        aris::core::Matrix trace_mat;
        double motorTrace[12][100];
        double deltaT[100];
        double interval_time;

        // 總執行時間
        Size totaltime;
    };
    auto PlanAdjust::prepareNrt()->void
    {
        PlanAdjustParam param;
        param.begin_pjs.resize(controller()->motorPool().size(), 0.0);
        param.target_pjs.resize(controller()->motorPool().size(), 0.0);

        // 从ros中获取通讯系统传来的指令，通过解析指令参数实现
        for (auto &p : cmdParams())	{
            if (p.first == "trace_mat") {
                param.trace_mat = matrixParam(p.first);
            }
        }
        ros::param::get("data_num", param.data_num);
        mout() << "data_num: " << param.data_num << endl;

        // 每一步用時0.5s,可在此處修改
        param.interval_time = 0.5;
        param.totaltime = param.interval_time * 1000 * (param.data_num - 1);
        mout() << "totaltime: " << param.totaltime << endl;
        
        // 对插值变量x进行赋值
        for (int i = 0; i < param.data_num; i++)    param.deltaT[i] = param.interval_time * 1000.0 * i;

        // 获得目标轨迹
        for (int i = 0; i < 12; i++) {
            for (int j = 0; j < param.data_num; j++) {
                param.motorTrace[i][j] = param.trace_mat(i, j) / 1000.0;
            }
        }
        this->param() = param;
        std::vector<std::pair<std::string, std::any>> ret_value;
        ret() = ret_value;
    }
    auto PlanAdjust::executeRT()->int
    {
        auto &param = std::any_cast<PlanAdjustParam&>(this->param());
        // 第一个周期设置log文件名称，获取当前电机所在位置; 初始化插值函數
        if (count() == 1){
            param.cs0.Initialize(param.deltaT, param.motorTrace[0], param.data_num);
            param.cs1.Initialize(param.deltaT, param.motorTrace[1], param.data_num);
            param.cs2.Initialize(param.deltaT, param.motorTrace[2], param.data_num);
            param.cs3.Initialize(param.deltaT, param.motorTrace[3], param.data_num);
            param.cs4.Initialize(param.deltaT, param.motorTrace[4], param.data_num);
            param.cs5.Initialize(param.deltaT, param.motorTrace[5], param.data_num);
            param.cs6.Initialize(param.deltaT, param.motorTrace[6], param.data_num);
            param.cs7.Initialize(param.deltaT, param.motorTrace[7], param.data_num);
            param.cs8.Initialize(param.deltaT, param.motorTrace[8], param.data_num);
            param.cs9.Initialize(param.deltaT, param.motorTrace[9], param.data_num);
            param.cs10.Initialize(param.deltaT, param.motorTrace[10], param.data_num);
            param.cs11.Initialize(param.deltaT, param.motorTrace[11], param.data_num);
            ecMaster()->logFileRawName("motion_plan_test");
            // 初始化電機初始位置［電機坐標系中位置，控制信號用］
            for (Size i = 0; i < 12; ++i) {
                param.begin_pjs[i] = controller()->motorPool()[i].targetPos();
                //mout() << "begin_pjs" << i << ":" << param.begin_pjs[i] << endl;
            }
        }
        double end_point[12] = {param.cs0.Interpolate(1.0 * count()),
                                param.cs1.Interpolate(1.0 * count()),
                                param.cs2.Interpolate(1.0 * count()),
                                param.cs3.Interpolate(1.0 * count()),
                                param.cs4.Interpolate(1.0 * count()),
                                param.cs5.Interpolate(1.0 * count()),
                                param.cs6.Interpolate(1.0 * count()),
                                param.cs7.Interpolate(1.0 * count()),
                                param.cs8.Interpolate(1.0 * count()),
                                param.cs9.Interpolate(1.0 * count()),
                                param.cs10.Interpolate(1.0 * count()),
                                param.cs11.Interpolate(1.0 * count())};
        // 電機執行反解結果
        for(int i = 0; i <= 9; i += 3) {
            // 左輔電機
            param.target_pjs[i + 1] = param.begin_pjs[i + 1] + (end_point[i + 1] - param.motorTrace[i + 1][0]);
            controller()->motorPool().at(i + 1).setTargetPos(param.target_pjs[i + 1]);
            // 右輔電機
            param.target_pjs[i + 2] = param.begin_pjs[i + 2] + (end_point[i + 2] - param.motorTrace[i + 2][0]);
            controller()->motorPool().at(i + 2).setTargetPos(param.target_pjs[i + 2]);
        }
        
        //打印
        if(count() % 5000 == 0){
            mout() << "end_motor1:" << std::setprecision(5) << end_point[1] << " "
                   << "end_motor2:" << std::setprecision(5) << end_point[2] << " " << endl;
            mout() << "end_motor4:" << std::setprecision(5) << end_point[4] << " "
                   << "end_motor5:" << std::setprecision(5) << end_point[5] << " " << endl;
            mout() << "end_motor7:" << std::setprecision(5) << end_point[7] << " "
                   << "end_motor8:" << std::setprecision(5) << end_point[8] << " " << endl;
            mout() << "end_motor10:" << std::setprecision(5) << end_point[10] << " "
                   << "end_motor11:" << std::setprecision(5) << end_point[11] << " " << endl;
        }

        //返回0表示正常结束，返回负数表示报错，返回正数表示正在执行
        return param.totaltime - count();
    }
    auto PlanAdjust::collectNrt()->void {
        mout() << "Finish plan adjust." << endl;
        ros::param::set("isFinishFlag", true); 
    }
    PlanAdjust::~PlanAdjust() = default;
    PlanAdjust::PlanAdjust(const std::string &name)
    {
        //构造函数参数说明，构造函数通过xml的格式定义本条指令的接口，name表示参数名，default表示输入参数，abbreviation表示参数名的缩写(缩写只能单个字符)
        //1 GroupParam下面的各个节点都是输入参数，如果没有给定会使用默认值
        //2 UniqueParam下面的各个节点互斥，有且只能使用其中的一个
        //3 例如，通过terminal或者socket发送“mvs --pos=0.1”，控制器实际会按照mvs --pos=0.1rad --time=1s --timenum=2 --all执行
        aris::core::fromXmlString(command(),
            "<Command name=\"planadjust\">"
            "		<Param name=\"trace_mat\" abbreviation=\"mat\"/>"
            "</Command>");
    }

    // 反馈控制命令实现
    auto PlanFootFeedback::prepareNrt()->void
    {      
        // 仍使用无反馈的数据结构
        PlanFootParam param;
        param.active_motor.clear();
        param.active_motor.resize(controller()->motorPool().size(), false);
        param.begin_pjs.resize(controller()->motorPool().size(), 0.0);
        param.step_pjs.resize(controller()->motorPool().size(), 0.0);
        param.max_time = 0;

        ros::param::get("leg_index", param.legIndex);
        mout() << "legIndex: " << param.legIndex << endl;
        // 所有腿都運動
        if (param.legIndex == 12) {
            param.begin_motor = 0;
            param.end_motor = 12;
            std::fill(param.active_motor.begin(), param.active_motor.end(), true);
        }
        // 單腿運動
        else{
            param.begin_motor = param.legIndex * 3;
            param.end_motor = param.legIndex * 3 + 3;
            param.active_motor.at(3 * param.legIndex) = true;
            param.active_motor.at(3 * param.legIndex + 1) = true;
            param.active_motor.at(3 * param.legIndex + 2) = true;
        }

        //解析输入参数
        for (auto &p : cmdParams())	{
            if (p.first == "end_mat") {
                param.end_mat = matrixParam(p.first);
            }
        }
        // 四条腿的位移
        for (int i = 0; i < 4; i++){
            if (param.legIndex == 12 || param.legIndex == i) {
                param.distance[i] = sqrt(pow(param.end_mat(i, 0), 2) 
                + pow(param.end_mat(i, 1), 2) + pow(param.end_mat(i, 2), 2));
                mout() << "distance" << i << "(mm): " << 1000 * param.distance[i] << endl;
            }
        }

        // 從文件中讀取電機初始位置
        ifstream inFile("/home/kaanh/Desktop/Lander_ws/src/RobotParam", ios::in);
        if (!inFile.is_open()) {
            mout() << "Can not open the parameter file." << endl;
        }
        inFile.setf(ios::fixed);
        inFile.precision(6);
        mout() << "Start position of four feet:" << endl;
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 3; j++) {
                inFile >> param.init_pos[i][j];
                mout() << param.init_pos[i][j] << " ";
                param.begin_pos[i][j] = param.init_pos[i][j];
            }
            mout() << endl;
        }
        inFile.close();
    
        // 函数返回值为一个4*3的四足末端位置数组
        param.ret_value.resize(4, vector<double>(3, 0.0));
        this->param() = param;
    }
    auto PlanFootFeedback::executeRT()->int
    {
        auto &param = std::any_cast<PlanFootParam&>(this->param());
        double next_pos[4], next_vel[4], next_acc[4];
        //第一个周期设置log文件名称，获取当前电机所在位置
        if (count() == 1){
            // 初始化電機初始位置［電機坐標系中位置，控制信號用］
            for (Size i = param.begin_motor; i < param.end_motor; ++i) {
                if (param.active_motor[i]) {
                    param.begin_pjs[i] = controller()->motorPool()[i].targetPos();
                    mout() << "begin_pjs" << i << ":" << param.begin_pjs[i] << endl;
                }
            }
            // 初始化電機初始位置［運動學反解坐標系中位置］
            for (int i = 0; i < 4; i++) {
                param.myPos[i].fromS1GetMotorAngle(param.begin_pos[i], param.selectIndex, param.d1_ori[i], param.theta2_ori[i], param.theta3_ori[i]);
                moveAbsolute(count(), 0, param.distance[i], 0.00001, 0.000001, 0.000001, next_pos[i], next_vel[i], next_acc[i], param.totaltime[i]);
                if(param.totaltime[i] > param.max_time)    param.max_time = param.totaltime[i];
            }         
            mout() << "max_time(ms): " << param.max_time << endl;
        }
        
        // 运动学反解结果
        double end_point[4][3] = {0.0};
        if(param.legIndex == 12) {
            for (int i = 0; i < 4; i++) {
                moveAbsolute(count(), 0, param.distance[i], 0.00001, 0.000001, 0.000001, next_pos[i], next_vel[i], next_acc[i], param.totaltime[i]);
                for(int j = 0; j < 3; j++) {
                    end_point[i][j] = param.begin_pos[i][j] + next_pos[i] / param.distance[i] * param.end_mat(i, j);
                }
                param.myPos[i].fromS1GetMotorAngle(end_point[i], param.selectIndex, param.d1[i], param.theta2[i], param.theta3[i]);
            }
        }
        else {
            moveAbsolute(count(), 0, param.distance[param.legIndex], 0.00001, 0.000001, 0.000001, next_pos[param.legIndex], next_vel[param.legIndex], next_acc[param.legIndex], param.totaltime[param.legIndex]);
            for(int j = 0; j < 3; j++) {
                end_point[param.legIndex][j] = param.begin_pos[param.legIndex][j] + next_pos[param.legIndex] / param.distance[param.legIndex] * param.end_mat(param.legIndex, j);
            }
            param.myPos[param.legIndex].fromS1GetMotorAngle(end_point[param.legIndex], param.selectIndex, param.d1[param.legIndex], param.theta2[param.legIndex], param.theta3[param.legIndex]);
        }
        
        // 电机执行程序
        for(Size i = param.begin_motor; i < param.end_motor; ++i) {
            if(!param.active_motor[i]) return 0;
        }
        for(Size i = param.begin_motor; i < param.end_motor; i += 3) {
            // 由于四条腿执行时长不一致，需要判断当前电机是否运动完毕
            if(count() <= param.totaltime[i / 3]) {
                // 主電機; i / 3爲腿的序號; i = 0,3,6,9
                param.step_pjs[i] = param.begin_pjs[i] + 1000 * (param.d1[i / 3] - param.d1_ori[i / 3]);
                controller()->motorPool().at(i).setTargetPos(param.step_pjs[i]);
                // 左輔電機
                param.step_pjs[i + 1] = param.begin_pjs[i + 1] + (param.theta2[i / 3] - param.theta2_ori[i / 3]);
                controller()->motorPool().at(i + 1).setTargetPos(param.step_pjs[i + 1]);
                // 右輔電機
                param.step_pjs[i + 2] = param.begin_pjs[i + 2] + (param.theta3_ori[i / 3] - param.theta3[i / 3]);
                controller()->motorPool().at(i + 2).setTargetPos(param.step_pjs[i + 2]);
            }   
        }

        // 触地检测
        // 以500ms作为时间区间，将该区间内的均值作为当前count的力矩值
        param.main_toq[(count() - 1) % 500] = controller()->motorPool()[3 * param.legIndex].actualToq();
        param.sub_toq_left[(count() - 1) % 500] = controller()->motorPool()[3 * param.legIndex + 1].actualToq();
        param.sub_toq_right[(count() - 1) % 500] = controller()->motorPool()[3 * param.legIndex + 2].actualToq();
        double main_toq_mean = accumulate(std::begin(param.main_toq),std::end(param.main_toq),0) / 500.0;
        double sub_toq_left_mean = accumulate(std::begin(param.sub_toq_left),std::end(param.sub_toq_left),0) / 500.0;
        double sub_toq_right_mean = accumulate(std::begin(param.sub_toq_right),std::end(param.sub_toq_right),0) / 500.0;
        if(count() >= 1000 && abs(main_toq_mean) >= 0.2) {
            for (Size i = 0; i < 4; ++i) {
                for (Size j = 0; j < 3; ++j) {
                    param.ret_value[i][j] = end_point[i][j] * 1000.0;
                }
            }
            ret() = param.ret_value;
            mout() << "————已经接触地面。停止运动，等待下次规划命令————" << endl;
            return 0;
        }
        ret() = param.ret_value;
        return param.max_time - count();
    }
    auto PlanFootFeedback::collectNrt()->void {
        auto &param = std::any_cast<PlanFootParam&>(this->param());
        // 更新全局參數
        for (int i = 0; i < 4; i++) {
            if (param.legIndex == 12 || param.legIndex == i) {
                for (int j = 0; j < 3; j++) {
                    param.init_pos[i][j] += param.end_mat(i ,j);
                }
            }
        }
        // 输出全局参数到文件中
        ofstream outFile("/home/kaanh/Desktop/Lander_ws/src/RobotParam", ios::trunc);
        if(!outFile.is_open()){
            mout() << "Can not open the parameter file." << endl;
        }
        outFile.setf(ios::fixed);
        outFile.precision(6);
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 2; j++) {
                outFile << param.init_pos[i][j] << " ";
            }
            outFile << param.init_pos[i][2] << endl;
        }
        outFile.close();
        mout() << "Finish motion." << endl;
        ros::param::set("isFinishFlag", true); 
    }
    PlanFootFeedback::~PlanFootFeedback() = default;
    PlanFootFeedback::PlanFootFeedback(const std::string &name)
    {
        //构造函数参数说明，构造函数通过xml的格式定义本条指令的接口，name表示参数名，default表示输入参数，abbreviation表示参数名的缩写(缩写只能单个字符)
        //1 GroupParam下面的各个节点都是输入参数，如果没有给定会使用默认值
        //2 UniqueParam下面的各个节点互斥，有且只能使用其中的一个
        //3 例如，通过terminal或者socket发送“mvs --pos=0.1”，控制器实际会按照mvs --pos=0.1rad --time=1s --timenum=2 --all执行
        aris::core::fromXmlString(command(),
            "<Command name=\"planfootfeedback\">"
            "		<Param name=\"end_mat\" abbreviation=\"end\"/>"   
            "</Command>");
    }

    auto PlanMotionFeedback::prepareNrt()->void
    {
        // 仍使用无反馈的数据结构
        PlanMotionParam param;
        param.begin_pjs.resize(controller()->motorPool().size(), 0.0);
        param.target_pjs.resize(controller()->motorPool().size(), 0.0);

        // 从ros中获取通讯系统传来的指令，通过解析指令参数实现
        for (auto &p : cmdParams())	{
            if (p.first == "trace_mat") {
                param.trace_mat = matrixParam(p.first);
            }
        }
        ros::param::get("leg_index", param.leg_index);
        ros::param::get("data_num", param.data_num);
        mout() << "data_num: " << param.data_num << endl;

        // 每一步用時0.5s,可在此處修改
        param.interval_time = 0.5;
        param.totaltime = param.interval_time * 1000 * (param.data_num - 1);
        mout() << "totaltime: " << param.totaltime << endl;
        
        // 对插值变量x进行赋值
        for (int i = 0; i < param.data_num; i++)    param.deltaT[i] = param.interval_time * 1000.0 * i;

        // 获得目标轨迹
        for (int i = 0; i < 12; i++) {
            for (int j = 0; j < param.data_num; j++) {
                param.legTrace[i][j] = param.trace_mat(i, j) / 1000.0;
            }
        }
        // 获得运动学反解的初始位置,取出param.legTrace的第一列付给4*3的矩阵
        mout() << "Start position of four feet:" << endl;
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 3; j++){
                param.begin_pos[i][j] = param.legTrace[i * 3 + j][0];
                mout() << param.begin_pos[i][j] << " ";
            }
            mout() << endl;
        }

        // 初始化電機初始位置［運動學反解坐標系中位置］
        for (Size i = 0; i < 4; i++) {
            param.myPos.fromS1GetMotorAngle(param.begin_pos[i], param.selectIndex, param.d1_ori[i], param.theta2_ori[i], param.theta3_ori[i]);
        }
        this->param() = param;
        std::vector<std::pair<std::string, std::any>> ret_value;
        ret() = ret_value;
    }
    auto PlanMotionFeedback::executeRT()->int
    {
        auto &param = std::any_cast<PlanMotionParam&>(this->param());
        // 第一个周期设置log文件名称，获取当前电机所在位置; 初始化插值函數
        if (count() == 1){
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
            // 设置记录文件名称
            ecMaster()->logFileRawName("220831_feedback_toqdata");
            // 初始化電機初始位置［電機坐標系中位置，控制信號用］
            for (Size i = 0; i < 12; ++i) {
                param.begin_pjs[i] = controller()->motorPool()[i].targetPos();
                //mout() << "begin_pjs" << i << ":" << param.begin_pjs[i] << endl;
            }
        }
        double end_point[4][3] = {{param.cs0.Interpolate(1.0 * count()),
                                   param.cs1.Interpolate(1.0 * count()),
                                   param.cs2.Interpolate(1.0 * count())},
                                  {param.cs3.Interpolate(1.0 * count()),
                                   param.cs4.Interpolate(1.0 * count()),
                                   param.cs5.Interpolate(1.0 * count())},
                                  {param.cs6.Interpolate(1.0 * count()),
                                   param.cs7.Interpolate(1.0 * count()),
                                   param.cs8.Interpolate(1.0 * count())},
                                  {param.cs9.Interpolate(1.0 * count()),
                                   param.cs10.Interpolate(1.0 * count()),
                                   param.cs11.Interpolate(1.0 * count())}};
        
        // 運動學反解
        for (int i = 0; i < 4; i++) {
            param.myPos.fromS1GetMotorAngle(end_point[i], param.selectIndex, param.d1[i], param.theta2[i], param.theta3[i]);
        }
        // 電機執行反解結果
        for(Size i = 0; i < 12; i += 3) {
            // 主電機; i / 3爲腿的序號
            param.target_pjs[i] = param.begin_pjs[i] + 1000 * (param.d1[i / 3] - param.d1_ori[i / 3]);
            controller()->motorPool().at(i).setTargetPos(param.target_pjs[i]);
            // 左輔電機
            param.target_pjs[i + 1] = param.begin_pjs[i + 1] + (param.theta2[i / 3] - param.theta2_ori[i / 3]);
            controller()->motorPool().at(i + 1).setTargetPos(param.target_pjs[i + 1]);
            // 右輔電機
            param.target_pjs[i + 2] = param.begin_pjs[i + 2] + (param.theta3_ori[i / 3] - param.theta3[i / 3]);
            controller()->motorPool().at(i + 2).setTargetPos(param.target_pjs[i + 2]);
        }

        //打印
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

        // 触地检测[待修改：触地检测判据]
        // 以500ms作为时间区间，将该区间内的均值作为当前count的力矩值
        param.main_toq[(count() - 1) % 500] = controller()->motorPool()[3 * param.leg_index].actualToq();
        param.sub_toq_left[(count() - 1) % 500] = controller()->motorPool()[3 * param.leg_index + 1].actualToq();
        param.sub_toq_right[(count() - 1) % 500] = controller()->motorPool()[3 * param.leg_index + 2].actualToq();
        double main_toq_mean = accumulate(std::begin(param.main_toq),std::end(param.main_toq),0) / 500.0;
        double sub_toq_left_mean = accumulate(std::begin(param.sub_toq_left),std::end(param.sub_toq_left),0) / 500.0;
        double sub_toq_right_mean = accumulate(std::begin(param.sub_toq_right),std::end(param.sub_toq_right),0) / 500.0;
        auto &lout = ecMaster()->lout();
        // 记录运动腿三个电机的力矩信息
        for(int i = 3 * param.leg_index; i < 3 * param.leg_index + 3; i++){
            lout << "motor: " << i << " ";
            lout << "actualToq: " << std::setprecision(10) << controller()->motorPool()[i].actualToq()<<"  "; 
        }
        lout << std::endl;
        // 测试阈值的时候，只记录数据，把if判断这部分注释掉
        if(count() >= 1000 && abs(main_toq_mean) >= 0.2) {
            for (Size i = 0; i < 4; ++i) {
                for (Size j = 0; j < 3; ++j) {
                    param.ret_value[i][j] = end_point[i][j] * 1000.0;
                }
            }
            ret() = param.ret_value;
            mout() << "————已经接触地面。停止运动，等待下次规划命令————" << endl;
            return 0;
        }
        ret() = param.ret_value;
        //返回0表示正常结束，返回负数表示报错，返回正数表示正在执行
        return param.totaltime - count();
    }
    auto PlanMotionFeedback::collectNrt()->void {
        auto &param = std::any_cast<PlanMotionParam&>(this->param());  
        // 将全局参数输出到记录文件中
        ofstream outFile("/home/kaanh/Desktop/Lander_ws/src/RobotParam", ios::trunc);
        if(!outFile.is_open()){
            mout() << "Can not open the parameter file." << endl;
        }
        outFile.setf(ios::fixed);
        outFile.precision(6);
        for (int i = 0; i < 4; i++) {
            outFile << param.legTrace[3 * i][param.data_num - 1] << " ";
            outFile << param.legTrace[3 * i + 1][param.data_num - 1] << " ";
            outFile << param.legTrace[3 * i + 2][param.data_num - 1] << endl;
        }
        outFile.close();
        mout() << "Finish plan motion." << endl;
        ros::param::set("isFinishFlag", true); 
    }
    PlanMotionFeedback::~PlanMotionFeedback() = default;
    PlanMotionFeedback::PlanMotionFeedback(const std::string &name)
    {
        //构造函数参数说明，构造函数通过xml的格式定义本条指令的接口，name表示参数名，default表示输入参数，abbreviation表示参数名的缩写(缩写只能单个字符)
        //1 GroupParam下面的各个节点都是输入参数，如果没有给定会使用默认值
        //2 UniqueParam下面的各个节点互斥，有且只能使用其中的一个
        //3 例如，通过terminal或者socket发送“mvs --pos=0.1”，控制器实际会按照mvs --pos=0.1rad --time=1s --timenum=2 --all执行
        aris::core::fromXmlString(command(),
            "<Command name=\"planmotionfeedback\">"
            "		<Param name=\"trace_mat\" abbreviation=\"mat\"/>"
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

        aris::core::class_<PlanAdjust>("PlanAdjust")
        .inherit<Plan>()
        ;

        aris::core::class_<PlanFootFeedback>("PlanFootFeedback")
        .inherit<Plan>()
        ;

        aris::core::class_<PlanMotionFeedback>("PlanMotionFeedback")
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
        plan_root->planPool().add<ControlPlan::PlanAdjust>();
        plan_root->planPool().add<ControlPlan::PlanFootFeedback>();
        plan_root->planPool().add<ControlPlan::PlanMotionFeedback>();
		return plan_root;
	}
}