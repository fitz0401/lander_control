#ifndef _TRANSMECHPOSITIONIK_HPP_
#define _TRANSMECHPOSITIONIK_HPP_

#include "Param.h"
#include "math.h"
#include <stdlib.h>

using namespace std;

class CTransMechPositionIK               
{
public:
    CTransMechPositionIK(){
        alpha1 = 0;                  //初始化
        alpha2 = 0;
        alpha3 = 0;

        d1 = 0.0;
        theta2 = 0.0;
        theta3 = 0.0;
    }

    double d1;
    double theta2;
    double theta3;

    void MyLink123AngletoInput1(double alpha1, double& d1){
        double theta_t1 = Param::theta_t0 - Param::theta_d - (M_PI / 2 - (Param::theta_origin1 + alpha1));
        d1 = sqrt(pow(Param::d2, 2) + pow(Param::d3, 2) - 2 * Param::d2 * Param::d3 * std::cos(theta_t1));
    }

    void MyLink123AngletoInput2(double phi2, double alpha2, double& theta2){
        double eps = 0.0001;

        // 求theta2, 存在多解可能，需讨论
        double A0 = 2 * Param::q1 * Param::q3x * std::sin(alpha2) - 2 * Param::q1 * Param::q3z * cos(alpha2) + 2 * Param::q1 * Param::q4z;
        double B0 = -2 * Param::q1 * Param::q3x * cos(Param::phi2_initial) * cos(alpha2) - 2 * Param::q1 * Param::q3z * cos(Param::phi2_initial) * sin(alpha2) - 2 * Param::q1 * Param::q4x;
        double C0 = 2 * Param::q3x * Param::q4x * cos(Param::phi2_initial) * cos(alpha2) + 2 * Param::q3x * Param::q4y * sin(Param::phi2_initial) * cos(alpha2) - \
            2 * Param::q3z * Param::q4z * cos(alpha2) + 2 * Param::q3z * Param::q4x * cos(Param::phi2_initial) * sin(alpha2) + 2 * Param::q3z * Param::q4y * sin(Param::phi2_initial) * sin(alpha2) + \
            2 * Param::q3x * Param::q4z * sin(alpha2) + pow(Param::q1, 2) - pow(Param::q2, 2) + pow(Param::q3x, 2) + pow(Param::q3z, 2) + pow(Param::q4x, 2) + pow(Param::q4y, 2) + pow(Param::q4z, 2);
        C0 = -C0;
        double beta3;
        double gamma3;

        int theta3_vector_i = 0;
        double theta3_vector[10];
        if (abs(B0 + C0) >= eps && pow(A0, 2) + pow(B0, 2) - pow(C0, 2) < 0) { // 二次方程无解情形
    //		cout << "无解" << endl;
        }
        else if (abs(B0 + C0) >= eps && pow(A0, 2) + pow(B0, 2) - pow(C0, 2) >= 0) { // 二次方程有解情形
            for (int k = -2; k <= 2; k++) {
                theta3_vector[theta3_vector_i] = 2 * (atan((A0 + sqrt(pow(A0, 2) + pow(B0, 2) - pow(C0, 2))) / (B0 + C0)) + k * M_PI);
                theta3_vector[theta3_vector_i + 5] = 2 * (atan((A0 - sqrt(pow(A0, 2) + pow(B0, 2) - pow(C0, 2))) / (B0 + C0)) + k * M_PI);
                theta3_vector_i++;
            }

            for (int i = 0; i < 10; i++) {
                if (theta3_vector[i] >= Param::theta2_min && theta3_vector[i] <= Param::theta2_max) {
                    this->theta2 = theta3_vector[i];
                }
            }
        }
        else { //退化为一次方程求解
            theta3_vector_i = 0;
            for (int k = -2; k <= 2; k++) {
                theta3_vector[theta3_vector_i] = 2 * (atan((C0 - B0) / (2 * A0)) + k * M_PI);
                theta3_vector_i++;
            }
            for (int i = 0; i < 10; i++) {
                if (theta3_vector[i] >= Param::theta2_min && theta3_vector[i] <= Param::theta2_max) {
                    this->theta2 = theta3_vector[i];
                }
            }
        }
        //cout << "支链2传动theta2 = " << theta2 << endl;
    }

    void MyLink123AngletoInput3(double phi3, double alpha3, double& theta3){
        double eps = 0.0001;

        // 求theta3, 存在多解可能，需讨论
        double A0 = 2 * Param::q1 * Param::q3z * cos(alpha3) - 2 * Param::q1 * Param::q3x * sin(alpha3) - 2 * Param::q1 * Param::q4z;
        double B0 = 2 * Param::q1 * Param::q3x * cos(Param::phi3_initial) * cos(alpha3) + 2 * Param::q1 * Param::q3z * cos(Param::phi3_initial) * sin(alpha3) - 2 * Param::q1 * Param::q4x;
        double C0 = 2 * Param::q3z * Param::q4y * sin(Param::phi3_initial) * sin(alpha3) - 2 * Param::q3z * Param::q4x * cos(Param::phi3_initial) * sin(alpha3) + \
            2 * Param::q3x * Param::q4z * sin(alpha3) + 2 * Param::q3x * Param::q4y * sin(Param::phi3_initial) * cos(alpha3) - 2 * Param::q3x * Param::q4x * cos(Param::phi3_initial) * cos(alpha3) - \
            2 * Param::q3z * Param::q4z * cos(alpha3) + pow(Param::q1, 2) - pow(Param::q2, 2) + pow(Param::q3x, 2) + pow(Param::q3z, 2) + pow(Param::q4x, 2) + pow(Param::q4y, 2) + pow(Param::q4z, 2);

        int theta3_vector_i = 0;
        double theta3_vector[10];
        if (abs(B0 + C0) >= eps && pow(A0, 2) + pow(B0, 2) - pow(C0, 2) < 0) { // 二次方程无解情形
    //		cout << "无解" << endl;
        }
        else if (abs(B0 + C0) >= eps && pow(A0, 2) + pow(B0, 2) - pow(C0, 2) >= 0) { // 二次方程有解情形
            for (int k = -2; k <= 2; k++) {
                theta3_vector[theta3_vector_i] = 2 * (atan((A0 + sqrt(pow(A0, 2) + pow(B0, 2) - pow(C0, 2))) / (B0 + C0)) + k * M_PI);
                theta3_vector[theta3_vector_i + 5] = 2 * (atan((A0 - sqrt(pow(A0, 2) + pow(B0, 2) - pow(C0, 2))) / (B0 + C0)) + k * M_PI);
                theta3_vector_i++;
            }

            for (int i = 0; i < 10; i++) {
                if (theta3_vector[i] >= Param::theta3_min && theta3_vector[i] <= Param::theta3_max) {
                    this->theta3 = theta3_vector[i];
                }
            }
        }
        else { //退化为一次方程求解
            theta3_vector_i = 0;
            for (int k = -2; k <= 2; k++) {
                theta3_vector[theta3_vector_i] = 2 * (atan((C0 - B0) / (2 * A0)) + k * M_PI);
                theta3_vector_i++;
            }
            for (int i = 0; i < 10; i++) {
                if (theta3_vector[i] >= Param::theta2_min && theta3_vector[i] <= Param::theta2_max) {
                    this->theta3 = theta3_vector[i];
                }
            }
        }
        //cout << "支链3传动theta3 = " << theta3 << endl;
    }

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

//	friend class myGetPosIK;
};


#endif
