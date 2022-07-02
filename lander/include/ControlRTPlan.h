#ifndef CONTROLPLAN_H_
#define CONTROLPLAN_H_

#include <memory>
#include <aris.hpp>

namespace ControlRTPlan
{
    //其他参数和函数声明
	using Size = std::size_t;
	constexpr double PI = 3.141592653589793;

    class WaitingPlan :public aris::core::CloneObject<WaitingPlan, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~WaitingPlan();
        explicit WaitingPlan(const std::string &name = "WaitingPlan");
    };

    class PlanMsg :public aris::core::CloneObject<PlanMsg, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;
        
        virtual ~PlanMsg();
        explicit PlanMsg(const std::string &name = "PlanMsg");
    };

	auto createPlanRoot()->std::unique_ptr<aris::plan::PlanRoot>;
}

#endif
