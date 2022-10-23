#ifndef _PARAM_H_
#define _PARAM_H_

#pragma once
#include <math.h>

namespace Param {
	// ÖŽÐÐ»ú¹¹
    const double a1 = 251.0 / 1000;
    const double b1 = 202.9166 / 1000;
    const double a2 = 35.3553 / 2 / 1000;
    const double b2 = 38.3391 / 1000;
    const double a3 = 127.0 / 1000;
    const double b3 = 60.8790 / 1000;
    const double l1 = (213.6473 + 80) / 1000;
    const double l1x = 5.3619 / 1000;
    const double l2 = 406.3691 / 1000;
    const double l3 = 406.3691 / 1000;
    const double l4 = 448.7958 / 1000;
    const double d = 863.0 / 2000;
    const double h = 451.0;
    const double theta_in = M_PI / 6;
    const double theta_m = 17.6909 * M_PI / 180;
    const double theta_ex = M_PI / 6;
    const double theta_p = (40.0 * M_PI) / 180;
    const double p1 = 129.0 / 1000;
    const double p2x = 123.8 / 1000;
    const double p2y = 25.5 / 1000;
    const double p2z = 28.0985 / 1000;
    const double p3x = 123.8 / 1000;
    const double p3y = 25.5 / 1000;
    const double p3z = 28.0985 / 1000;

    // žšŽ«¶¯
    const double q1 = 82.0008 / 1000;
    const double q2 = 141.0035 / 1000;
    const double q3x = 61.0 / 1000;
    const double q3z = 24.0 / 1000;
    const double q4x = 19.8 / 1000;
    const double q4y = 95 / 1000;
    const double q4z = 9 / 1000;

    // Ö÷Ž«¶¯
    const double theta_t0 = (167.2238 * M_PI) / 180;
    const double theta_d = (43.3231 * M_PI) / 180;
    const double d2 = 229.5545 / 1000;
    const double d3 = 172.1663 / 1000;
    const double theta_origin1 = M_PI / 2 - M_PI / 6;

    // ·¶Î§ÏÞ¶š
    const double alpha1_min = -3 * M_PI / 3;     const double alpha1_max = 0;
    const double alpha2_min = -3 * M_PI / 3;     const double alpha2_max = 0;
    const double alpha3_min = -3 * M_PI / 3;     const double alpha3_max = 0;
    const double phi1_min = -M_PI / 2;           const double phi1_max = M_PI / 2;
    const double phi2_min = -M_PI / 2;           const double phi2_max = M_PI / 2;
    const double phi3_min = M_PI / 2;            const double phi3_max = -M_PI / 2; // ×¢ÒâŽËÏîÌØÊâ£¬È¡ÂßŒ­»ò
    const double beta1_min = -M_PI / 3;          const double beta1_max = M_PI / 3;
    const double beta2_min = -M_PI / 3;          const double beta2_max = M_PI / 3;
    const double beta3_min = -M_PI / 3;          const double beta3_max = M_PI / 3;
    const double gamma1_min = -M_PI / 3;		 const double gamma1_max = M_PI / 3;
    const double gamma2_min = -M_PI / 3;         const double gamma2_max = M_PI / 3;
    const double gamma3_min = -M_PI / 3;         const double gamma3_max = M_PI / 3;
    const double ksim = M_PI / 3;


    // ÆäËû²ÎÊý
    const double Rms = d + b1 * sin(theta_in);
    const double Rsp = 2.1;
    const double phi1_initial = 0;
    const double phi2_initial = 0;
    const double phi3_initial = -M_PI;
    const double alpha1_initial = 0;
    const double alpha2_initial = 0;
    const double alpha3_initial = 0;
    const double theta2_min = 0.0;
    const double theta2_max = M_PI;
    const double theta3_min = 0.0;
    const double theta3_max = M_PI;
}

#endif
