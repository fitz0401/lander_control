#ifndef CONTROLPLAN_H_
#define CONTROLPLAN_H_

#include <memory>
#include <aris.hpp>

namespace ControlPlan
{
    // 通讯规划指令，lander_plan项目使用
	using Size = std::size_t;
	constexpr double PI = 3.141592653589793;
    // 全局管道，用于和主进程之间传递状态信息
    extern aris::core::Pipe pipe_global_stateMsg;
    extern int contact_index;
    // 状态信息
    struct StateParam
    {
        int count;
        double position[12];
        double velocity[12];
        double current[12];
    };
    // 决策反馈信息
    struct PlanParam
    {
        int contact_index;
    };
    extern StateParam state_pub;

    class GetPosPlan :public aris::core::CloneObject<GetPosPlan, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~GetPosPlan();
        explicit GetPosPlan(const std::string &name = "getposplan");
    };

    class FindHomePlan :public aris::core::CloneObject<FindHomePlan, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~FindHomePlan();
        explicit FindHomePlan(const std::string &name = "initplan");
    };

    class PlanFoot :public aris::core::CloneObject<PlanFoot, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~PlanFoot();
        explicit PlanFoot(const std::string &name = "planfoot");
    };

    class PlanMotion :public aris::core::CloneObject<PlanMotion, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~PlanMotion();
        explicit PlanMotion(const std::string &name = "planmotion");
    };

    class PlanAdjust :public aris::core::CloneObject<PlanAdjust, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~PlanAdjust();
        explicit PlanAdjust(const std::string &name = "planadjust");
    };

    // 本条指令与PlanFoot相比，多了检测触地+反馈停止时的足端位置信息
    class PlanFootFeedback :public aris::core::CloneObject<PlanFootFeedback, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~PlanFootFeedback();
        explicit PlanFootFeedback(const std::string &name = "planfootfeedback");
    };

    // 本条指令与PlanMotion相比，多了检测触地+反馈停止时的足端位置信息
    class PlanMotionFeedback :public aris::core::CloneObject<PlanMotionFeedback, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~PlanMotionFeedback();
        explicit PlanMotionFeedback(const std::string &name = "planmotionfeedback");
    };

    // 实时发送状态信息
    class SendState :public aris::core::CloneObject<SendState, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~SendState();
        explicit SendState(const std::string &name = "sendstate");
    };

	auto createPlanRoot()->std::unique_ptr<aris::plan::PlanRoot>;
}

#endif
