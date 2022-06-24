/********************************************************/
/*File:      AdjustingModeCal_IK.cpp                    */
/*Func:      主函数                                     */
/*Version:   1.0                                        */
/*Author:    徐浩    符泽        2022/02/18             */
/********************************************************/

#include "WalkLegExeLegIk.h" //行走
#include "param.h" //参数
#include "GetPosIK.h"
#include <math.h>
#include <iostream>
using namespace std;

int main()
{
	int SelectSolutionIndex[3] = { 1,1,2 };  //行走模式在两个二支链上有两个解，所以在数组后两个元素有1，2两种选择，共4种
	double S1__C[3] = { 0.48585, 0, -0.382590243417449 };   // 位置反解的输入参数：S1在C基下的坐标;

	myGetPosIK getpos;
	getpos.fromS1GetMotorAngle(S1__C, SelectSolutionIndex);

	return 0;
}