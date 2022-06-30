#ifndef CONTROLCMD_H_
#define CONTROLCMD_H_

#include <memory>
#include <aris.hpp>

namespace ControlCMD
{
    //其他参数和函数声明
	using Size = std::size_t;
	constexpr double PI = 3.141592653589793;
    
    // 命令行指令，lander_cmd项目使用
    class MoveS :public aris::core::CloneObject<MoveS, aris::plan::Plan>
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;
		auto virtual collectNrt()->void;

		virtual ~MoveS();
		explicit MoveS(const std::string &name = "mvs");
	};

    class MoveMotor :public aris::core::CloneObject<MoveMotor, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~MoveMotor();
        explicit MoveMotor(const std::string &name = "mvm");
    };

    class GetPos :public aris::core::CloneObject<GetPos, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~GetPos();
        explicit GetPos(const std::string &name = "getpos");
    };

    class FindHome :public aris::core::CloneObject<FindHome, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~FindHome();
        explicit FindHome(const std::string &name = "init");
    };

    class MoveLeg :public aris::core::CloneObject<MoveLeg, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~MoveLeg();
        explicit MoveLeg(const std::string &name = "mvleg");
    };

    class MoveRobot :public aris::core::CloneObject<MoveRobot, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~MoveRobot();
        explicit MoveRobot(const std::string &name = "mvrobot");
    };

    class MoveLine :public aris::core::CloneObject<MoveLine, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~MoveLine();
        explicit MoveLine(const std::string &name = "mvline");
    };

    class MovePlan :public aris::core::CloneObject<MovePlan, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~MovePlan();
        explicit MovePlan(const std::string &name = "mvplan");
    };

	auto createPlanRoot()->std::unique_ptr<aris::plan::PlanRoot>;
}

#endif
