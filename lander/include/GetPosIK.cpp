/********************************************************/
/*File:      WalkingModeCal_IK.cpp                     */
/*Func:      ִ�л��� -> �������� �ۺϼ������         */
/*Author:    ��� ����  2022/2/18                      */
/*Revised:   2021.11.30                                */
/********************************************************/

#include "Param.h"
#include "GetPosIK.h"
#include <math.h>
using namespace std;

// ������ʼ��
myGetPosIK::myGetPosIK() {
    myGetPosIK::alpha1 = 0;
    myGetPosIK::beta1 = 0;
    myGetPosIK::gamma1 = 0;
    myGetPosIK::alpha2 = 0;
    myGetPosIK::beta2 = 0;
    myGetPosIK::gamma2 = 0;
    myGetPosIK::alpha3 = 0;
    myGetPosIK::beta3 = 0;
    myGetPosIK::gamma3 = 0;

    myGetPosIK::phi2_initial = Param::phi2_initial;
    myGetPosIK::phi3_initial = Param::phi3_initial;

    myGetPosIK::d1 = 0;
    myGetPosIK::theta2 = 0;
    myGetPosIK::theta3 = 0;
}

void myGetPosIK::fromS1GetMotorAngle(double S1__C[], int* SelectSolutionIndex, double& d1, double& theta2, double& theta3) {
	// ����ִ�л������
    exeMech.MyS1toLink123Angle(S1__C, SelectSolutionIndex, alpha1, alpha2, alpha3);

//	cout << "alpha1: " << alpha1 << ", "
//		<< "alpha2: " << alpha2 << ", "
//		<< "alpha3: " << alpha3 << ", " << endl;

	// ��������������
    transMech.MyLink123AngletoInput1(alpha1, d1);
    transMech.MyLink123AngletoInput2(phi2_initial, alpha2, theta2);
    transMech.MyLink123AngletoInput3(phi3_initial, alpha3, theta3);

//	cout << "d1: " << transMech.d1 << ", "
//		<< "theta2: " << transMech.theta2 << ", "
//		<< "theta3: " << transMech.theta3 << endl;
}

myGetPosIK::~myGetPosIK() {

}
