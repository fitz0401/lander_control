#ifndef _PARAM_H_
#define _PARAM_H_

#pragma once
#include <math.h>

namespace Param {
	// 执行机构
    const double a1 = 251.0 / 1000;
    const double b1 = 202.9166 / 1000;
    const double a2 = 35.3553 / 2 / 1000;
    const double b2 = 38.3391 / 1000;
    const double a3 = 127.0 / 1000;
    const double b3 = 60.8790 / 1000;
    const double l1 = 213.6473 / 1000;
    const double l2 = 406.3691 / 1000;
    const double l3 = 406.3691 / 1000;
    const double l4 = 448.7958 / 1000;
    const double d = 863.0 / 2000;
    //const double h = 451.0;
    const double theta_in = 17.6909 * M_PI / 180;
    const double theta_ex = M_PI / 6;
    const double theta_p = (40.0 * M_PI) / 180;
    const double p1 = 129.0 / 1000;
    const double p2x = 124.0 / 1000;
    const double p2y = 27.7771 / 1000;
    const double p2z = 26.0140 / 1000;
    const double p3x = 124.0 / 1000;
    const double p3y = 27.7771 / 1000;
    const double p3z = 26.0140 / 1000;

	// 辅传动
    const double q1 = 82.4761 / 1000;
    const double q2 = 141.0035 / 1000;
    const double q3x = 61.0 / 1000;
    const double q3z = 24.0 / 1000;
    const double q4x = 18.5 / 1000;
    const double q4y = 103.8522 / 1000;
    const double q4z = 9.0 / 1000;

	// 主传动
    const double theta_t0 = (167.2238 * M_PI) / 180;
    const double theta_d = (43.3231 * M_PI) / 180;
    const double d2 = 229.5545 / 1000;
    const double d3 = 172.1663 / 1000;
    const double theta_origin1 = M_PI / 2 - M_PI / 6;

	// 范围限定
    const double alpha1_min = -3 * M_PI / 3;     const double alpha1_max = 0;
    const double alpha2_min = -3 * M_PI / 3;     const double alpha2_max = 0;
    const double alpha3_min = -3 * M_PI / 3;     const double alpha3_max = 0;
    const double phi1_min = -M_PI / 2;           const double phi1_max = M_PI / 2;
    const double phi2_min = -M_PI / 2;           const double phi2_max = M_PI / 2;
    const double phi3_min = M_PI / 2;            const double phi3_max = -M_PI / 2; // 注意此项特殊，取逻辑或
    const double beta1_min = -M_PI / 3;          const double beta1_max = M_PI / 3;
    const double beta2_min = -M_PI / 3;          const double beta2_max = M_PI / 3;
    const double beta3_min = -M_PI / 3;          const double beta3_max = M_PI / 3;
    const double gamma1_min = -M_PI / 3;		 const double gamma1_max = M_PI / 3;
    const double gamma2_min = -M_PI / 3;         const double gamma2_max = M_PI / 3;
    const double gamma3_min = -M_PI / 3;         const double gamma3_max = M_PI / 3;
    const double ksim = M_PI / 3;


	// 其他参数
    //const double h = d;
    const double Rms = d + b1 * sin(theta_in);
    //const double Rsp = d + b3 * sin(theta_in) + (l1 + l4) * sin(theta_ex);
    const double Rsp = 2.1;
    const double phi1_initial = 0;         //-0.0403
    const double phi2_initial = 0;			//0.0133
    const double phi3_initial = -M_PI;		//-3.1224
    const double alpha1_initial = 0;
    const double alpha2_initial = 0;
    const double alpha3_initial = 0;
    const double theta2_min = 0.0;
    const double theta2_max = M_PI;
    const double theta3_min = 0.0;
    const double theta3_max = M_PI;

    //const double walkingphi_a1 = 0;
    //const double walkingphi_a2 = 0;
    //const double walkingphi_a3 = M_PI;
    //const double theta_in = M_PI / 6;
    //const double a2 = 35.3553 / 2 / 1000;
    //const double b2 = 38.3391 / 1000;
    //const double a3 = 127 / 1000;
    //const double b3 = 60.8790 / 1000;
    //const double l1 = 213.6473 / 1000;
    //const double l2 = 406.3691 / 1000;
    //const double l3 = 406.3691 / 1000;
    //const double l4 = 448.7958 / 1000;
    //const double p1 = 129.0 / 1000;
    //const double p2 = 124.0 / 1000; //p2x
    //const double p3 = 124.0 / 1000; //p3x
    //const double d = 863.0 / 2 / 1000;
    //const double offy = 27.7771 / 1000; //p2y, p3y
    //const double offz = 26.0140 / 1000; //p2z, p3z
    //const double offa = (40 * M_PI) / 180; //theta_p

    //const double theta_origin1 = M_PI / 2 - M_PI / 6;
    //const double theta_d = (43.3231 * M_PI) / 180;
    //const double theta_t0 = (167.2238 * M_PI) / 180;
    //const double d2 = 229.5545 / 1000;
    //const double d3 = 172.1663 / 1000;

    //const double q1 = 82.4761 / 1000;
    //const double q2 = 141.0035 / 1000;
    //const double q3x = 61.0 / 1000; //等式右边没有浮点数时会强制转换为int
    //const double q3z = 24.0 / 1000;
    //const double q4x = 18.5 / 1000;
    //const double q4y = 103.8522 / 1000;
    //const double q4z = 9.0 / 1000;
    //const double theta2_min = 0.0;
    //const double theta2_max = M_PI;
    //const double theta3_min = 0.0;
    //const double theta3_max = M_PI;
    //const double phi2_initial = 0.0;
    //const double phi3_initial = -M_PI;
}

#endif
