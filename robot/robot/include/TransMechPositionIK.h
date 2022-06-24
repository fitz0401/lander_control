#ifndef _TRANSMECHPOSITIONIK_HPP_
#define _TRANSMECHPOSITIONIK_HPP_

#include "Param.h"
#include "math.h"
#include <stdlib.h>

using namespace std;

class CTransMechPositionIK
{
public:
    CTransMechPositionIK();

    void MyLink123AngletoInput1(double alpha1, double& d1);
    void MyLink123AngletoInput2(double phi2, double alpha2, double& theta2);
    void MyLink123AngletoInput3(double phi3, double alpha3, double& theta3);

    ~CTransMechPositionIK();

private:
    double alpha1;
    double alpha2;
    double alpha3;
    double Link123_AngleMatrix[9] = { 0.0 };
    double Link1Angle[3] = { 0.0 };
    double Link2Angle[3] = { 0.0 };
    double Link3Angle[3] = { 0.0 };

    int SelectSolutionIndex[3] = { 0 };  //行走模式在两个二支链上有两个解，所以在数组后两个元素有1，2两种选择，共4种
    double S1__C[3] = { 0.0 };   // 位置反解的输入参数：S1在C基下的坐标;

};


#endif
