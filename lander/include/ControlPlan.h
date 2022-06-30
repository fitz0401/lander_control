#ifndef CONTROLPLAN_H_
#define CONTROLPLAN_H_

#include <memory>
#include <aris.hpp>

namespace ControlPlan
{
    //其他参数和函数声明
	using Size = std::size_t;
	constexpr double PI = 3.141592653589793;

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

    // 通讯规划指令，lander_plan项目使用
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

	auto createPlanRoot()->std::unique_ptr<aris::plan::PlanRoot>;
}

#endif
