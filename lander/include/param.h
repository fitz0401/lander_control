#ifndef _PARAM_H_
#define _PARAM_H_

#include <math.h>

namespace Param {
	// 执行机构
    const float a1 = 251.0 / 1000;
    const float b1 = 202.9166 / 1000;
    const float a2 = 35.3553 / 2 / 1000;
    const float b2 = 38.3391 / 1000;
    const float a3 = 127.0 / 1000;
    const float b3 = 60.8790 / 1000;
    const float l1 = 213.6473 / 1000;
    const float l2 = 406.3691 / 1000;
    const float l3 = 406.3691 / 1000;
    const float l4 = 448.7958 / 1000;
    const float d = 863.0 / 2000;
    //const float h = 451.0;
    const float theta_in = 17.6909 * M_PI / 180;
    const float theta_ex = M_PI / 6;
    const float theta_p = (40.0 * M_PI) / 180;
    const float p1 = 129.0 / 1000;
    const float p2x = 124.0 / 1000;
    const float p2y = 27.7771 / 1000;
    const float p2z = 26.0140 / 1000;
    const float p3x = 124.0 / 1000;
    const float p3y = 27.7771 / 1000;
    const float p3z = 26.0140 / 1000;

	// 辅传动
    const float q1 = 82.4761 / 1000;
    const float q2 = 141.0035 / 1000;
    const float q3x = 61.0 / 1000;
    const float q3z = 24.0 / 1000;
    const float q4x = 18.5 / 1000;
    const float q4y = 103.8522 / 1000;
    const float q4z = 9.0 / 1000;

	// 主传动
    const float theta_t0 = (167.2238 * M_PI) / 180;
    const float theta_d = (43.3231 * M_PI) / 180;
    const float d2 = 229.5545 / 1000;
    const float d3 = 172.1663 / 1000;
    const float theta_origin1 = M_PI / 2 - M_PI / 6;

	// 范围限定
    const float alpha1_min = -3 * M_PI / 3;     const float alpha1_max = 0;
    const float alpha2_min = -3 * M_PI / 3;     const float alpha2_max = 0;
    const float alpha3_min = -3 * M_PI / 3;     const float alpha3_max = 0;
    const float phi1_min = -M_PI / 2;           const float phi1_max = M_PI / 2;
    const float phi2_min = -M_PI / 2;           const float phi2_max = M_PI / 2;
    const float phi3_min = M_PI / 2;            const float phi3_max = -M_PI / 2; // 注意此项特殊，取逻辑或
    const float beta1_min = -M_PI / 3;          const float beta1_max = M_PI / 3;
    const float beta2_min = -M_PI / 3;          const float beta2_max = M_PI / 3;
    const float beta3_min = -M_PI / 3;          const float beta3_max = M_PI / 3;
    const float gamma1_min = -M_PI / 3;		 const float gamma1_max = M_PI / 3;
    const float gamma2_min = -M_PI / 3;         const float gamma2_max = M_PI / 3;
    const float gamma3_min = -M_PI / 3;         const float gamma3_max = M_PI / 3;
    const float ksim = M_PI / 3;


	// 其他参数
    //const float h = d;
    const float Rms = d + b1 * sin(theta_in);
    //const float Rsp = d + b3 * sin(theta_in) + (l1 + l4) * sin(theta_ex);
    const float Rsp = 2.1;
    const float phi1_initial = 0;         //-0.0403
    const float phi2_initial = 0;			//0.0133
    const float phi3_initial = -M_PI;		//-3.1224
    const float alpha1_initial = 0;
    const float alpha2_initial = 0;
    const float alpha3_initial = 0;
    const float theta2_min = 0.0;
    const float theta2_max = M_PI;
    const float theta3_min = 0.0;
    const float theta3_max = M_PI;

    //const float walkingphi_a1 = 0;
    //const float walkingphi_a2 = 0;
    //const float walkingphi_a3 = M_PI;
    //const float theta_in = M_PI / 6;
    //const float a2 = 35.3553 / 2 / 1000;
    //const float b2 = 38.3391 / 1000;
    //const float a3 = 127 / 1000;
    //const float b3 = 60.8790 / 1000;
    //const float l1 = 213.6473 / 1000;
    //const float l2 = 406.3691 / 1000;
    //const float l3 = 406.3691 / 1000;
    //const float l4 = 448.7958 / 1000;
    //const float p1 = 129.0 / 1000;
    //const float p2 = 124.0 / 1000; //p2x
    //const float p3 = 124.0 / 1000; //p3x
    //const float d = 863.0 / 2 / 1000;
    //const float offy = 27.7771 / 1000; //p2y, p3y
    //const float offz = 26.0140 / 1000; //p2z, p3z
    //const float offa = (40 * M_PI) / 180; //theta_p

    //const float theta_origin1 = M_PI / 2 - M_PI / 6;
    //const float theta_d = (43.3231 * M_PI) / 180;
    //const float theta_t0 = (167.2238 * M_PI) / 180;
    //const float d2 = 229.5545 / 1000;
    //const float d3 = 172.1663 / 1000;

    //const float q1 = 82.4761 / 1000;
    //const float q2 = 141.0035 / 1000;
    //const float q3x = 61.0 / 1000; //等式右边没有浮点数时会强制转换为int
    //const float q3z = 24.0 / 1000;
    //const float q4x = 18.5 / 1000;
    //const float q4y = 103.8522 / 1000;
    //const float q4z = 9.0 / 1000;
    //const float theta2_min = 0.0;
    //const float theta2_max = M_PI;
    //const float theta3_min = 0.0;
    //const float theta3_max = M_PI;
    //const float phi2_initial = 0.0;
    //const float phi3_initial = -M_PI;
}

#endif
