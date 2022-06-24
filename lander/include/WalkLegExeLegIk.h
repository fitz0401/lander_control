#ifndef _WALKLEGEXELEGIK_HPP
#define _WALKLEGEXELEGIK_HPP

#include "Param.h"
#include <math.h>

using namespace std;

class WalkLegExeLegIk                //反解求解
{
public:
    WalkLegExeLegIk();

    void MyS1toLink123Angle(double S1__C[], int* SelectSolutionIndex, double& alpha1, double& alpha2, double& alpha3);    //上层调用返回反解角度函数

    ~WalkLegExeLegIk();

private:
    void IK_Link1_Matrix(double X, double Y, double Z, double& m_alpha1, double& m_beta1, double& m_gamma1);
    void IK_Link2_Matrix(double alpha1, double beta1, double gamma1, int SelectSolutionIndex[], double& m_alpha2);
    void IK_Link3_Matrix(double alpha1, double beta1, double gamma1, int SelectSolutionIndex[], double& m_alpha3);

    //运动学反解中用到的量
    double beta1;
    double gamma1;
    double beta2;
    double gamma2;
    double beta3;
    double gamma3;
    double Link123_AngleMatrix[9] = {0};
    double Link1Angle[3] = {0};
    double Link2Angle[3] = {0};
    double Link3Angle[3] = {0};
//	friend class myGetPosIK;

    int SelectSolutionIndex[3];  //行走模式在两个二支链上有两个解，所以在数组后两个元素有1，2两种选择，共4种
    double S1__C[3];   // 位置反解的输入参数：S1在C基下的坐标;
};

#endif
