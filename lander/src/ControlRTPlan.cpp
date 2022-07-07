#include <array>
#include <stdlib.h>
#include <string>
#include <vector>
#include <bitset>
#include <fstream>
#include "CubicSplineNoDynamic.h"
#include "ControlRTPlan.h"
#include "GetPosIK.h"
#include "math.h"
#include "WalkLegExeLegIk.h"
#include "Param.h"
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "lander/mv_msgs.h"

using namespace aris::dynamic;
using namespace aris::plan;

namespace ControlRTPlan
{   
    //指令的执行顺序
    //1、先执行prepareNrt，每条指令只执行一次
    //2、然后执行executeRT,executeRT每一个循环周期(默认1ms)会被实时核调用一次，执行的总时间由用户给定
    //3、执行结束后，本指令会被析构
    //指令功能：某一电机或者所有电机1-cos(theta)轨迹运行，幅值为pos，周期为time，周期数为timenum

    aris::core::Pipe pipe_global_planMsg;
    aris::core::Pipe pipe_global_stateMsg;
    struct WalkingPlanParam
    {
        Size isExecuted;
        Size motion_val;
    };
    struct WalkingStateParam
    {
        int state_val;
    };
    
    auto WaitingPlan::prepareNrt()->void
    {
        WalkingPlanParam plan_param;
        this->param() = plan_param;
    }
    auto WaitingPlan::executeRT()->int {       
        auto &plan_param = std::any_cast<WalkingPlanParam&>(this->param());
        // 数据收发
        if (count() % 100 == 0) {
            aris::core::MsgFix<1024> plan_msg_recv;
            pipe_global_planMsg.recvMsg(plan_msg_recv);    
            plan_msg_recv.pasteStruct(plan_param);
            // 已执行过则下次不再执行
            if (plan_param.isExecuted == 0) {
                plan_param.isExecuted = 1;
                // 修改执行标志位
                aris::core::Msg plan_msg_executed;
                plan_msg_executed.copyStruct(plan_param);
                pipe_global_planMsg.sendMsg(plan_msg_executed);
                // 发送状态数据
                WalkingStateParam state_param;
                state_param.state_val = count();
                aris::core::Msg state_msg;
                state_msg.copyStruct(state_param);
                pipe_global_stateMsg.sendMsg(state_msg);
            }
        }
        // 电机运动
        if(count() % 10 == 0) {
            mout() << "current count:" << count() << endl;
            mout() <<  plan_param.motion_val << endl;
        }   
        return 1;
    }
    auto WaitingPlan::collectNrt()->void {
        mout() << "Finish the state of waiting plan!" << endl;
    }
    WaitingPlan::~WaitingPlan() = default;
    WaitingPlan::WaitingPlan(const std::string &name)
    {
        //构造函数参数说明，构造函数通过xml的格式定义本条指令的接口，name表示参数名，default表示输入参数，abbreviation表示参数名的缩写(缩写只能单个字符)
        //1 GroupParam下面的各个节点都是输入参数，如果没有给定会使用默认值
        //2 UniqueParam下面的各个节点互斥，有且只能使用其中的一个
        //3 例如，通过terminal或者socket发送“mvs --pos=0.1”，控制器实际会按照mvs --pos=0.1rad --time=1s --timenum=2 --all执行
        aris::core::fromXmlString(command(),
            "<Command name=\"WaitingPlan\">"
            "</Command>");
    }
    
    auto PlanMsg::prepareNrt()->void
    {
        WalkingPlanParam plan_param;
        // 从ros中获取通讯系统传来的指令，通过解析指令参数实现
        for (auto &p : cmdParams())	{
            if (p.first == "motion_val") {
                plan_param.motion_val = int32Param(p.first);
            }
            // else if (p.first == "isExecuted") {
            //     plan_param.isExecuted = int32Param(p.first);
            // }
        }
        plan_param.isExecuted = 0;

        aris::core::Msg plan_msg;
        plan_msg.copyStruct(plan_param);
        pipe_global_planMsg.sendMsg(plan_msg);

        // 确保接收到WaitingPlan返回的状态信息后，将状态信息发给ros
        WalkingStateParam state_param;
        aris::core::MsgFix<1024> state_msg_recv;
        while(1) {
            if (pipe_global_stateMsg.recvMsg(state_msg_recv)) {
                state_msg_recv.pasteStruct(state_param);
                break;
            }
        }
        this->option() = Plan::NOT_RUN_EXECUTE_FUNCTION | Plan::NOT_RUN_COLLECT_FUNCTION;
        ret() = state_param.state_val;
    }
    auto PlanMsg::executeRT()->int {return 0;}
    auto PlanMsg::collectNrt()->void {}
    PlanMsg::~PlanMsg() = default;
    PlanMsg::PlanMsg(const std::string &name)
    {
        //构造函数参数说明，构造函数通过xml的格式定义本条指令的接口，name表示参数名，default表示输入参数，abbreviation表示参数名的缩写(缩写只能单个字符)
        //1 GroupParam下面的各个节点都是输入参数，如果没有给定会使用默认值
        //2 UniqueParam下面的各个节点互斥，有且只能使用其中的一个
        //3 例如，通过terminal或者socket发送“mvs --pos=0.1”，控制器实际会按照mvs --pos=0.1rad --time=1s --timenum=2 --all执行
        aris::core::fromXmlString(command(),
            "<Command name=\"PlanMsg\">"
            "	<GroupParam>"
            "	    <Param name=\"motion_val\" abbreviation=\"val\"/>"\
            "		<Param name=\"isExecuted\" abbreviation=\"is\" default=\"0\"/>"
            "	</GroupParam>"         
            "</Command>");
    }


    auto GetState::prepareNrt()->void
    {
        WalkingPlanParam plan_param;
        this->param() = plan_param;
    }
    auto GetState::executeRT()->int {       
        auto &plan_param = std::any_cast<WalkingPlanParam&>(this->param());
        // 数据收发
        if (count() % 50 == 0) {
            for (int i=0; i<3; i++){
                mout() << "motor" << i << " position: " << controller()->motorPool()[i].actualPos() << " ";
                mout() << " velocity: " << controller()->motorPool()[i].actualVel() << " ";
                mout() << " Toque: "    << controller()->motorPool()[i].actualToq() << " ";
                mout() << " Current: "  << controller()->motorPool()[i].actualCur() << " ";
                mout() << endl;
            }        
        }
        return 1;
    }
    auto GetState::collectNrt()->void {}
    GetState::~GetState() = default;
    GetState::GetState(const std::string &name)
    {
        //构造函数参数说明，构造函数通过xml的格式定义本条指令的接口，name表示参数名，default表示输入参数，abbreviation表示参数名的缩写(缩写只能单个字符)
        //1 GroupParam下面的各个节点都是输入参数，如果没有给定会使用默认值
        //2 UniqueParam下面的各个节点互斥，有且只能使用其中的一个
        //3 例如，通过terminal或者socket发送“mvs --pos=0.1”，控制器实际会按照mvs --pos=0.1rad --time=1s --timenum=2 --all执行
        aris::core::fromXmlString(command(),
            "<Command name=\"getstate\">"
            "</Command>");
    }
   

	ARIS_REGISTRATION
	{
        aris::core::class_<WaitingPlan>("WaitingPlan")
        .inherit<Plan>()
        ;

        aris::core::class_<PlanMsg>("PlanMsg")
        .inherit<Plan>()
        ;

        aris::core::class_<GetState>("GetState")
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

        plan_root->planPool().add<ControlRTPlan::WaitingPlan>();
        plan_root->planPool().add<ControlRTPlan::PlanMsg>();
        plan_root->planPool().add<ControlRTPlan::GetState>();
		return plan_root;
	}
}
