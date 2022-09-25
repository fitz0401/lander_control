#ifndef _WALKLEGEXELEGIK_HPP
#define _WALKLEGEXELEGIK_HPP

#include "Param.h"

using namespace std;

class WalkLegExeLegIk                //·ŽœâÇóœâ
{
public:
    WalkLegExeLegIk();

    //ÉÏ²ãµ÷ÓÃ·µ»ØÔË¶¯Ñ§·ŽœâËùÐèÒªµÄœÇ¶È
    void MyS1toLink123Angle(double S1__C[], int* SelectSolutionIndex, double& alpha1, double& alpha2, double& alpha3);  
    //ÉÏ²ãµ÷ÓÃ·µ»ØÑÅ¿É±ÈŸØÕóÇóœâËùÐèÒªµÄËùÓÐ¹ØœÚ×ªœÇ
    void MyS1toAllJointAngles(double S1__C[], int* SelectSolutionIndex, double* alpha, double* beta, double* gamma);

    ~WalkLegExeLegIk();

private:
    void IK_Link1_Matrix(double X, double Y, double Z, double& m_alpha1, double& m_beta1, double& m_gamma1);
    void IK_Link2_Matrix(double alpha1, double beta1, double gamma1, int SelectSolutionIndex[], double& m_alpha2);
    void IK_Link3_Matrix(double alpha1, double beta1, double gamma1, int SelectSolutionIndex[], double& m_alpha3);

    //ÔË¶¯Ñ§·ŽœâÖÐÓÃµœµÄÁ¿
    double beta1;
    double gamma1;
    double beta2;
    double gamma2;
    double beta3;
    double gamma3;
    double Link123_AngleMatrix[9] = { 0 };
    double Link1Angle[3] = { 0 };
    double Link2Angle[3] = { 0 };
    double Link3Angle[3] = { 0 };

    int SelectSolutionIndex[3];  //ÐÐ×ßÄ£ÊœÔÚÁœžö¶þÖ§ÁŽÉÏÓÐÁœžöœâ£¬ËùÒÔÔÚÊý×éºóÁœžöÔªËØÓÐ1£¬2ÁœÖÖÑ¡Ôñ£¬¹²4ÖÖ
    double S1__C[3];   // Î»ÖÃ·ŽœâµÄÊäÈë²ÎÊý£ºS1ÔÚC»ùÏÂµÄ×ø±ê;
};

#endif