/********************************************************/
/*File:      AdjustingModeCal_IK.cpp                    */
/*Func:      ������                                     */
/*Version:   1.0                                        */
/*Author:    ���    ����        2022/02/18             */
/********************************************************/

#include "WalkLegExeLegIk.h" //����
#include "param.h" //����
#include "GetPosIK.h"
#include <math.h>
#include <iostream>
using namespace std;

int main()
{
	int SelectSolutionIndex[3] = { 1,1,2 };  //����ģʽ��������֧�����������⣬���������������Ԫ����1��2����ѡ�񣬹�4��
	double S1__C[3] = { 0.48585, 0, -0.382590243417449 };   // λ�÷�������������S1��C���µ�����;

	myGetPosIK getpos;
	getpos.fromS1GetMotorAngle(S1__C, SelectSolutionIndex);

	return 0;
}