/********************************************************/
/*Func:      ���ܳ���ͷ�ļ�                             */
/*Version:   1.0                                        */
/*Author:    ���    ����        2022/02/18             */
/********************************************************/

#ifndef _GETPOSIK_H_
#define _GETPOSIK_H_

#pragma once
#include "TransMechPositionIK.h"
#include "WalkLegExeLegIk.h"

class myGetPosIK {
public:
	myGetPosIK();

    void fromS1GetMotorAngle(double S1__C[], int* SelectSolutionIndex, double& d1, double& theta2, double& theta3);

	~myGetPosIK();

private:
    WalkLegExeLegIk exeMech;
	CTransMechPositionIK transMech;

    double alpha1;
    double beta1;
    double gamma1;
    double alpha2;
    double beta2;
    double gamma2;
    double alpha3;
    double beta3;
    double gamma3;

    double phi2_initial;
    double phi3_initial;

    double d1;
    double theta2;
    double theta3;
};

#endif
