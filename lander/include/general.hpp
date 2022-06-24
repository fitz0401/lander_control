#ifndef GENERAL_HPP_
#define GENERAL_HPP_

#include <aris.hpp>

#define ATOMIC_LOAD(ato_var) ato_var.load(std::memory_order_acquire)
#define ATOMIC_STORE(ato_var, val) ato_var.store(val, std::memory_order_release)

using Size = aris::Size;

class SysConfig {
private:
    SysConfig() {}
    ~SysConfig() {}

public:
    static auto instance()->SysConfig& {
        static SysConfig sys_config;
        return sys_config;
    }

    auto execPath() const->std::filesystem::path {
        return std::filesystem::absolute(".");
    }

    auto configFile() const->std::filesystem::path {
        return execPath() / "kaanh.xml";
    }

    auto logPath() const->std::filesystem::path {
        return execPath() / "log";
    }

    auto createPlanRoot() const->aris::plan::PlanRoot*;
};

static void debugPrint(const std::string &str) {
    static bool debug_on = true;
    if (debug_on) {
        std::cout << "## " << str << std::endl;
    }
}

// 梯形轨迹规划
static auto moveAbsolute(double i, double begin_pos, double end_pos, double vel, double acc, double dec, double &current_pos, double &current_vel, double &current_acc, Size& total_count)->void
{
    double v = std::abs(vel);
    double a = std::abs(acc);
    double d = std::abs(dec);
    double s = std::abs(end_pos - begin_pos);

    Size n1 = static_cast<Size>(std::ceil(v / a));
    Size n3 = static_cast<Size>(std::ceil(v / d));

    a = v / n1;
    d = v / n3;

    double s1 = a * n1 * n1 / 2.0;
    double s3 = d * n3 * n3 / 2.0;
    double s2 = s - s1 - s3;

    // 判断是否有匀速段
    if (s2 > 0)
    {
        Size n2 = static_cast<Size>(std::ceil(s2 / v));
        double coe = s / (s1 + v * n2 + s3);

        a *= coe;
        d *= coe;
        v *= coe;
        s1 *= coe;
        s2 *= coe;
        s3 *= coe;

        total_count = n1 + n2 + n3;
        if (i < n1)
        {
            current_pos = a * i * i / 2.0;
            current_vel = a * i;
            current_acc = a;
        }
        else if (i < n1 + n2)
        {
            current_pos = s1 + v * (i - n1);
            current_vel = v;
            current_acc = 0.0;
        }
        else if (i < n1 + n2 + n3)
        {
            current_pos = s - d * (n1 + n2 + n3 - i) * (n1 + n2 + n3 - i) / 2.0;
            current_vel = d * (n1 + n2 + n3 - i);
            current_acc = -d;
        }
        else
        {
            current_pos = s;
            current_vel = 0;
            current_acc = 0;
        }
    }
    else
    {
        v = std::sqrt(2 * s * a * d / (a + d));
        n1 = static_cast<Size>(std::ceil(v / a));
        n3 = static_cast<Size>(std::ceil(v / d));

        double coe = s / (a*n1*n1 / 2.0 + d * n3*n3 / 2.0);
        a *= coe;
        d *= coe;

        total_count = n1 + n3;
        if (i < n1)
        {
            current_pos = a * i * i / 2.0;
            current_vel = a * i;
            current_acc = a;
        }
        else if (i < n1 + n3)
        {
            current_pos = s - d * (n1 + n3 - i) * (n1 + n3 - i) / 2.0;
            current_vel = d * (n1 + n3 - i);
            current_acc = -d;
        }
        else
        {
            current_pos = s;
            current_vel = 0;
            current_acc = 0;
        }

    }

    // 修正位置、速度、加速度方向
    if (end_pos < begin_pos)
    {
        current_pos = begin_pos - current_pos;
        current_vel = -current_vel;
        current_acc = -current_acc;
    }
    else
    {
        current_pos = begin_pos + current_pos;
        current_vel = current_vel;
        current_acc = current_acc;
    }
    //目标位置太小情况处理//
    if (std::abs(end_pos - begin_pos) <= 1e-9)
    {
        total_count = 0.0;
        current_pos = end_pos;
        current_vel = 0.0;
        current_acc = 0.0;
    }
    if (i >= total_count)
    {
        current_pos = end_pos;
        current_vel = 0.0;
        current_acc = 0.0;
    }
}

// 空间直线点坐标求解
struct Coordinate {
    double x{0};
    double y{0};
    double z{0};

    Coordinate() {}

    Coordinate(double _x, double _y, double _z) {
        x = _x;
        y = _y;
        z = _z;
    }

    void print () const {
        std::cout << "X: " << x
                  << " Y: " << y
                  << " Z: " << z
                  << std::endl;
    }
};

static double modulusCalculation(const Coordinate &start, const Coordinate &end) {
    Coordinate v(end.x- start.x, end.y-start.y, end.z-start.z);
    return sqrt(pow(v.x, 2) + pow(v.y, 2) + pow(v.z, 2));
}

static Coordinate coordinateCalculation(const Coordinate &start, const Coordinate &end, double distance) {
    Coordinate v(end.x- start.x, end.y-start.y, end.z-start.z);

    double modulus = sqrt(pow(v.x, 2) + pow(v.y, 2) + pow(v.z, 2));

    Coordinate tar;
    tar.x = start.x + distance * v.x / modulus;
    tar.y = start.y + distance * v.y / modulus;
    tar.z = start.z + distance * v.z / modulus;

    return tar;
}

static bool trapezoidPlanAT(double count, double start, double end, double a, double T, double max_v, double &next) {
    if (std::abs(end - start) <= 1e-9) {
        next = end;
        return true;
    }

    a = std::abs(a);
    T = std::abs(T);
    max_v = std::abs(max_v);

    // 归一化
    double normal_a = a / std::abs(end - start);
    double max_normal_v = max_v / std::abs(end - start);

    // 计算归一化后的规划速度
    double normal_v = 0.5 * (normal_a*T - sqrt(pow(normal_a,2)*pow(T, 2) - 4*normal_a));

    // 约束校验
    if (normal_a * pow(T, 2) < 4 || normal_v > max_normal_v) {
        return false;
    }

    // 计算加速/减速时长
    double t = normal_v / normal_a;

    double normal_next = 0;
    if (count <= t) {
        normal_next = 0.5 * normal_a * pow(count, 2);
    } else if (count > t && count <= T-t) {
        normal_next = normal_v*count - 0.5 * pow(normal_v,2) / normal_a;
    } else if (count > T-t && count <= T) {
        normal_next = (2*normal_a*normal_v*T-2*pow(normal_v,2)-pow(normal_a,2)*pow(T-count,2)) / (2*normal_a);
    } else {
        next = end;
        return true;
    }

    next = start + (end-start)*normal_next;
    return true;
}

#endif // GENERAL_HPP_