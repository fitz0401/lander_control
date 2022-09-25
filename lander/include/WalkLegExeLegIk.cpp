#include "WalkLegExeLegIk.h"

WalkLegExeLegIk::WalkLegExeLegIk() {
    //��ʼ��
    beta1 = 0;
    gamma1 = 0;
    beta2 = 0;
    gamma2 = 0;
    beta3 = 0;
    gamma3 = 0;
}

void WalkLegExeLegIk::MyS1toLink123Angle(double S1__C[], int* SelectSolutionIndex, double& alpha1, double& alpha2, double& alpha3) {
    IK_Link1_Matrix(S1__C[0], S1__C[1], S1__C[2], alpha1, beta1, gamma1);
    IK_Link2_Matrix(alpha1, beta1, gamma1, SelectSolutionIndex, alpha2);
    IK_Link3_Matrix(alpha1, beta1, gamma1, SelectSolutionIndex, alpha3);
}

void WalkLegExeLegIk::MyS1toAllJointAngles(double S1__C[], int* SelectSolutionIndex, double* alpha, double* beta, double* gamma) {
    IK_Link1_Matrix(S1__C[0], S1__C[1], S1__C[2], alpha[0], beta1, gamma1);
    IK_Link2_Matrix(alpha[0], beta1, gamma1, SelectSolutionIndex, alpha[1]);
    IK_Link3_Matrix(alpha[0], beta1, gamma1, SelectSolutionIndex, alpha[2]);
    beta[0] = this->beta1;
    beta[1] = this->beta2;
    beta[2] = this->beta3;
    gamma[0] = this->gamma1;
    gamma[1] = this->gamma2;
    gamma[2] = this->gamma3;
}

WalkLegExeLegIk::~WalkLegExeLegIk() {}

void WalkLegExeLegIk::IK_Link1_Matrix(double X, double Y, double Z, double& m_alpha1, double& m_beta1, double& m_gamma1) {
    double x = Z * cos(Param::theta_in) - Param::b3 * sin(Param::theta_in) * sin(Param::theta_m) + X * sin(Param::theta_in);
    double y = -Y;
    double z = X * cos(Param::theta_in) - Z * sin(Param::theta_in) - Param::b3 * cos(Param::theta_in) * sin(Param::theta_m);
    m_gamma1 = asin((x * sin(Param::phi1_initial) - y * cos(Param::phi1_initial)) / (Param::l1 + Param::l4));
    double A_beta1 = 2 * Param::l1x * Param::p1;
    double B_beta1 = 2 * Param::p1 * cos(m_gamma1) * (Param::l1 + Param::l4);
    double C_beta1 = pow(x, 2) + pow(y, 2) + pow(z, 2) - pow(Param::p1, 2) - pow((Param::l1 + Param::l4), 2);
    double beta1_vector[2] = { 0.0, 0.0 };
    if (A_beta1 + C_beta1 == 0) {
        double u = (C_beta1 - A_beta1) / (2 * B_beta1);
        beta1_vector[0] = atan2((2 * u), 1 - pow(u, 2));
    }
    else if (pow(A_beta1, 2) + pow(B_beta1, 2) - pow(C_beta1, 2) >= 0) {
        double u1 = (B_beta1 - sqrt(pow(A_beta1, 2) + pow(B_beta1, 2) - pow(C_beta1, 2))) / (A_beta1 + C_beta1);
        double u2 = (B_beta1 + sqrt(pow(A_beta1, 2) + pow(B_beta1, 2) - pow(C_beta1, 2))) / (A_beta1 + C_beta1);
        beta1_vector[0] = atan2((2 * u1), 1 - pow(u1, 2));
        beta1_vector[1] = atan2((2 * u2), 1 - pow(u2, 2));
    }
    else   return;
    m_beta1 = beta1_vector[0];
    double alpha_imag_1 = ((-(Param::l1 + Param::l4) * sin(Param::phi1_initial) * cos(m_beta1) * sin(m_gamma1) - z * cos(Param::phi1_initial) * sin(m_beta1) + x * cos(m_beta1)) * (Param::l1 + Param::l4) * cos(m_gamma1) - Param::p1 * z * cos(Param::phi1_initial)) + Param::l1x * ((Param::l1 + Param::l4) * sin(Param::phi1_initial) * sin(m_beta1) * sin(m_gamma1) - z * cos(Param::phi1_initial) * cos(m_beta1) - x * sin(m_beta1));
    double alpha_real_1 = ((-(Param::l1 + Param::l4) * sin(Param::phi1_initial) * sin(m_beta1) * sin(m_gamma1) + z * cos(Param::phi1_initial) * cos(m_beta1) + x * sin(m_beta1)) * (Param::l1 + Param::l4) * cos(m_gamma1) + Param::p1 * (x - (Param::l1 + Param::l4) * sin(Param::phi1_initial) * sin(m_gamma1)) + Param::l1x * (x * cos(m_beta1) - z * cos(Param::phi1_initial) * sin(m_beta1) - (Param::l1 + Param::l4) * sin(Param::phi1_initial) * cos(m_beta1) * sin(m_gamma1)));
    m_alpha1 = atan2(alpha_imag_1, alpha_real_1);
}

void WalkLegExeLegIk::IK_Link2_Matrix(double alpha1, double beta1, double gamma1, int SelectSolutionIndex[], double& m_alpha2)
{
    //֧��1λ������
    double x = Param::a2 * (sin(gamma1) * (cos(alpha1) * sin(beta1) * sin(Param::phi1_initial) + cos(beta1) * sin(alpha1) * sin(Param::phi1_initial)) + cos(gamma1) * cos(Param::phi1_initial)) - Param::a3 - Param::l4 * (cos(Param::phi1_initial) * sin(gamma1) - cos(gamma1) * (cos(alpha1) * sin(beta1) * sin(Param::phi1_initial) + cos(beta1) * sin(alpha1) * sin(Param::phi1_initial))) - Param::b2 * (cos(alpha1) * cos(beta1) * sin(Param::phi1_initial) - sin(alpha1) * sin(beta1) * sin(Param::phi1_initial)) + Param::p1 * cos(alpha1) * sin(Param::phi1_initial);
    double y = Param::a2 * (cos(Param::theta_in) * (cos(gamma1) * sin(Param::phi1_initial) - sin(gamma1) * (cos(alpha1) * cos(Param::phi1_initial) * sin(beta1) + cos(beta1) * cos(Param::phi1_initial) * sin(alpha1))) - sin(gamma1) * sin(Param::theta_in) * (sin(alpha1) * sin(beta1) - cos(alpha1) * cos(beta1))) - cos(Param::theta_in) * (Param::l4 * (sin(gamma1) * sin(Param::phi1_initial) + cos(gamma1) * (cos(alpha1) * cos(Param::phi1_initial) * sin(beta1) + cos(beta1) * cos(Param::phi1_initial) * sin(alpha1))) + Param::p1 * cos(alpha1) * cos(Param::phi1_initial))  \
        - sin(Param::theta_in) * (Param::p1 * sin(alpha1) + Param::l4 * cos(gamma1) * (sin(alpha1) * sin(beta1) - cos(alpha1) * cos(beta1))) - Param::b3 * cos(Param::theta_m) + Param::b2 * (cos(Param::theta_in) * (cos(alpha1) * cos(beta1) * cos(Param::phi1_initial) - cos(Param::phi1_initial) * sin(alpha1) * sin(beta1)) + sin(Param::theta_in) * (cos(alpha1) * sin(beta1) + cos(beta1) * sin(alpha1)));
    double z = sin(Param::theta_in) * (Param::l4 * (sin(gamma1) * sin(Param::phi1_initial) + cos(gamma1) * (cos(alpha1) * cos(Param::phi1_initial) * sin(beta1) + cos(beta1) * cos(Param::phi1_initial) * sin(alpha1))) + Param::p1 * cos(alpha1) * cos(Param::phi1_initial)) - Param::a2 * (sin(Param::theta_in) * (cos(gamma1) * sin(Param::phi1_initial) - sin(gamma1) * (cos(alpha1) * cos(Param::phi1_initial) * sin(beta1) + cos(beta1) * cos(Param::phi1_initial) * sin(alpha1))) + cos(Param::theta_in) * sin(gamma1) * (sin(alpha1) * sin(beta1) - cos(alpha1) * cos(beta1)))  \
        - cos(Param::theta_in) * (Param::p1 * sin(alpha1) + Param::l4 * cos(gamma1) * (sin(alpha1) * sin(beta1) - cos(alpha1) * cos(beta1))) + Param::b3 * sin(Param::theta_m) - Param::b2 * (sin(Param::theta_in) * (cos(alpha1) * cos(beta1) * cos(Param::phi1_initial) - cos(Param::phi1_initial) * sin(alpha1) * sin(beta1)) - cos(Param::theta_in) * (cos(alpha1) * sin(beta1) + cos(beta1) * sin(alpha1)));

    //����֧��2
    //����gamma2������˫��
    double A_2 = 4 * pow(Param::l2, 2) * (pow(Param::p2x, 2) + pow(Param::p2z, 2));
    double E_2 = x * sin(Param::phi2_initial) - y * cos(Param::phi2_initial) + Param::p2y;
    double D_2 = sin(Param::theta_p) * (pow(x, 2) + pow(y, 2) + pow(z, 2) - pow(Param::p2x, 2) - pow(Param::p2y, 2) - pow(Param::p2z, 2) - pow(Param::l2, 2)) - 2 * (Param::p2y * sin(Param::theta_p) + Param::p2z * cos(Param::theta_p)) * (y * cos(Param::phi2_initial) - x * sin(Param::phi2_initial) - Param::p2y);
    double B_2 = -4 * D_2 * Param::l2 * Param::p2z - 8 * pow(Param::p2x, 2) * Param::l2 * cos(Param::theta_p) * E_2;
    double C_2 = pow(D_2, 2) + 4 * pow(Param::p2x, 2) * pow(E_2, 2) - pow((2 * Param::l2 * Param::p2x * sin(Param::theta_p)), 2);

    // ֧��2ȡ��ֵ
    if (SelectSolutionIndex[1] == 1) {
        double gamma2_1 = asin((-B_2 + sqrt(pow(B_2, 2) - 4 * A_2 * C_2)) / (2 * A_2));
        double beta2_1_imag = (sin(Param::theta_p) * (2 * x * Param::p2y * sin(Param::phi2_initial) - 2 * y * Param::p2y * cos(Param::phi2_initial) + pow(x, 2) + pow(y, 2) + pow(z, 2) - pow(Param::p2x, 2) + pow(Param::p2y, 2) - pow(Param::p2z, 2) - pow(Param::l2, 2)) + 2 * Param::p2z * ((x * sin(Param::phi2_initial) - y * cos(Param::phi2_initial) + Param::p2y) * cos(Param::theta_p) - Param::l2 * sin(gamma2_1)));
        double beta2_1_real = (2 * Param::p2x * (Param::l2 * cos(Param::theta_p) * sin(gamma2_1) - x * sin(Param::phi2_initial) + y * cos(Param::phi2_initial) - Param::p2y));
        double beta2_1 = atan2(beta2_1_imag, beta2_1_real);

        double alpha2_1_real = ((z * cos(Param::theta_p) * cos(beta2_1) + sin(beta2_1) * (x * cos(Param::phi2_initial) + y * sin(Param::phi2_initial))) * Param::l2 * cos(gamma2_1) + Param::l2 * z * sin(Param::theta_p) * sin(gamma2_1) + Param::p2x * x * cos(Param::phi2_initial) + Param::p2x * y * sin(Param::phi2_initial) + Param::p2z * z);
        double alpha2_1_imag = ((cos(Param::theta_p) * cos(beta2_1) * (x * cos(Param::phi2_initial) + y * sin(Param::phi2_initial)) - z * sin(beta2_1)) * Param::l2 * cos(gamma2_1) + Param::l2 * sin(Param::theta_p) * sin(gamma2_1) * (x * cos(Param::phi2_initial) + y * sin(Param::phi2_initial)) + x * Param::p2z * cos(Param::phi2_initial) + y * Param::p2z * sin(Param::phi2_initial) - Param::p2x * z);
        double alpha2_1 = atan2(alpha2_1_imag, alpha2_1_real);

        this->gamma2 = gamma2_1;
        this->beta2 = beta2_1;
        m_alpha2 = alpha2_1;
    }
    else if (SelectSolutionIndex[1] == 2) {
        double gamma2_2 = asin((-B_2 - sqrt(pow(B_2, 2) - 4 * A_2 * C_2)) / (2 * A_2));
        double beta2_2_real = (2 * Param::p2x * (Param::l2 * cos(Param::theta_p) * sin(gamma2_2) - x * sin(Param::phi2_initial) + y * cos(Param::phi2_initial) - Param::p2y));
        double beta2_2_imag = (sin(Param::theta_p) * (2 * x * Param::p2y * sin(Param::phi2_initial) - 2 * y * Param::p2y * cos(Param::phi2_initial) + pow(x, 2) + pow(y, 2) + pow(z, 2) - pow(Param::p2x, 2) + pow(Param::p2y, 2) - pow(Param::p2z, 2) - pow(Param::l2, 2)) + 2 * Param::p2z * ((x * sin(Param::phi2_initial) - y * cos(Param::phi2_initial) + Param::p2y) * cos(Param::theta_p) - Param::l2 * sin(gamma2_2)));
        double beta2_2 = atan2(beta2_2_imag, beta2_2_real);

        double alpha2_2_real = ((z * cos(Param::theta_p) * cos(beta2_2) + sin(beta2_2) * (x * cos(Param::phi2_initial) + y * sin(Param::phi2_initial))) * Param::l2 * cos(gamma2_2) + Param::l2 * z * sin(Param::theta_p) * sin(gamma2_2) + Param::p2x * x * cos(Param::phi2_initial) + Param::p2x * y * sin(Param::phi2_initial) + Param::p2z * z);
        double alpha2_2_imag = ((cos(Param::theta_p) * cos(beta2_2) * (x * cos(Param::phi2_initial) + y * sin(Param::phi2_initial)) - z * sin(beta2_2)) * Param::l2 * cos(gamma2_2) + Param::l2 * sin(Param::theta_p) * sin(gamma2_2) * (x * cos(Param::phi2_initial) + y * sin(Param::phi2_initial)) + x * Param::p2z * cos(Param::phi2_initial) + y * Param::p2z * sin(Param::phi2_initial) - Param::p2x * z);
        double alpha2_2 = atan2(alpha2_2_imag, alpha2_2_real);

        this->gamma2 = gamma2_2;
        this->beta2 = beta2_2;
        m_alpha2 = alpha2_2;
    }
}

void WalkLegExeLegIk::IK_Link3_Matrix(double alpha1, double beta1, double gamma1, int SelectSolutionIndex[], double& m_alpha3)
{
    //֧��1λ������
    double x = Param::a3 - Param::a2 * (sin(gamma1) * (cos(alpha1) * sin(beta1) * sin(Param::phi1_initial) + cos(beta1) * sin(alpha1) * sin(Param::phi1_initial)) + cos(gamma1) * cos(Param::phi1_initial)) - Param::l4 * (cos(Param::phi1_initial) * sin(gamma1) - cos(gamma1) * (cos(alpha1) * sin(beta1) * sin(Param::phi1_initial) + cos(beta1) * sin(alpha1) * sin(Param::phi1_initial))) - Param::b2 * (cos(alpha1) * cos(beta1) * sin(Param::phi1_initial) - sin(alpha1) * sin(beta1) * sin(Param::phi1_initial)) + Param::p1 * cos(alpha1) * sin(Param::phi1_initial);
    double y = Param::b2 * (cos(Param::theta_in) * (cos(alpha1) * cos(beta1) * cos(Param::phi1_initial) - cos(Param::phi1_initial) * sin(alpha1) * sin(beta1)) + sin(Param::theta_in) * (cos(alpha1) * sin(beta1) + cos(beta1) * sin(alpha1))) - cos(Param::theta_in) * (Param::l4 * (sin(gamma1) * sin(Param::phi1_initial) + cos(gamma1) * (cos(alpha1) * cos(Param::phi1_initial) * sin(beta1) + cos(beta1) * cos(Param::phi1_initial) * sin(alpha1))) + Param::p1 * cos(alpha1) * cos(Param::phi1_initial)) - Param::a2 * (cos(Param::theta_in) * (cos(gamma1) * sin(Param::phi1_initial) \
        - sin(gamma1) * (cos(alpha1) * cos(Param::phi1_initial) * sin(beta1) + cos(beta1) * cos(Param::phi1_initial) * sin(alpha1))) - sin(gamma1) * sin(Param::theta_in) * (sin(alpha1) * sin(beta1) - cos(alpha1) * cos(beta1))) - Param::b3 * cos(Param::theta_m) - sin(Param::theta_in) * (Param::p1 * sin(alpha1) + Param::l4 * cos(gamma1) * (sin(alpha1) * sin(beta1) - cos(alpha1) * cos(beta1)));
    double z = Param::a2 * (sin(Param::theta_in) * (cos(gamma1) * sin(Param::phi1_initial) - sin(gamma1) * (cos(alpha1) * cos(Param::phi1_initial) * sin(beta1) + cos(beta1) * cos(Param::phi1_initial) * sin(alpha1))) + cos(Param::theta_in) * sin(gamma1) * (sin(alpha1) * sin(beta1) - cos(alpha1) * cos(beta1))) - cos(Param::theta_in) * (Param::p1 * sin(alpha1) + Param::l4 * cos(gamma1) * (sin(alpha1) * sin(beta1) - cos(alpha1) * cos(beta1))) + sin(Param::theta_in) * (Param::l4 * (sin(gamma1) * sin(Param::phi1_initial)  \
        + cos(gamma1) * (cos(alpha1) * cos(Param::phi1_initial) * sin(beta1) + cos(beta1) * cos(Param::phi1_initial) * sin(alpha1))) + Param::p1 * cos(alpha1) * cos(Param::phi1_initial)) + Param::b3 * sin(Param::theta_m) - Param::b2 * (sin(Param::theta_in) * (cos(alpha1) * cos(beta1) * cos(Param::phi1_initial) - cos(Param::phi1_initial) * sin(alpha1) * sin(beta1)) - cos(Param::theta_in) * (cos(alpha1) * sin(beta1) + cos(beta1) * sin(alpha1)));

    //����֧��3
    double E_3 = x * sin(Param::phi3_initial) - y * cos(Param::phi3_initial) - Param::p3y;
    double A_3 = 4 * pow(Param::l3, 2) * (pow(Param::p3x, 2) + pow(Param::p3z, 2));
    double D_3 = -sin(Param::theta_p) * (pow(x, 2) + pow(y, 2) + pow(z, 2) - pow(Param::p3x, 2) - pow(Param::p3y, 2) - pow(Param::p3z, 2) - pow(Param::l3, 2)) - 2 * (Param::p3y * sin(Param::theta_p) + Param::p3z * cos(Param::theta_p)) * (y * cos(Param::phi3_initial) - x * sin(Param::phi3_initial) + Param::p3y);
    double B_3 = -4 * D_3 * Param::l3 * Param::p3z - 8 * pow(Param::p3x, 2) * Param::l3 * cos(Param::theta_p) * E_3;
    double C_3 = pow(D_3, 2) + 4 * pow(Param::p3x, 2) * pow(E_3, 2) - pow((2 * Param::l3 * Param::p3x * sin(Param::theta_p)), 2);

    // ֧��3ȡ��ֵ
    if (SelectSolutionIndex[2] == 1) {
        double gamma3_1 = asin((-B_3 + sqrt(pow(B_3, 2) - 4 * A_3 * C_3)) / (2 * A_3));
        double beta3_1_real = (-2 * Param::p3x * (Param::l3 * cos(Param::theta_p) * sin(gamma3_1) - x * sin(Param::phi3_initial) + y * cos(Param::phi3_initial) + Param::p3y));
        double beta3_1_imag = (sin(Param::theta_p) * (-2 * x * Param::p3y * sin(Param::phi3_initial) + 2 * y * Param::p3y * cos(Param::phi3_initial) + pow(x, 2) + pow(y, 2) + pow(z, 2) - pow(Param::p3x, 2) + pow(Param::p3y, 2) - pow(Param::p3z, 2) - pow(Param::l3, 2)) - 2 * Param::p3z * ((x * sin(Param::phi3_initial) - y * cos(Param::phi3_initial) - Param::p3y) * cos(Param::theta_p) - Param::l3 * sin(gamma3_1)));
        double beta3_1 = atan2(beta3_1_imag, beta3_1_real);

        double alpha3_1_real = ((z * cos(Param::theta_p) * cos(beta3_1) + sin(beta3_1) * (x * cos(Param::phi3_initial) + y * sin(Param::phi3_initial))) * Param::l3 * cos(gamma3_1) - Param::l3 * z * sin(Param::theta_p) * sin(gamma3_1) + Param::p3x * x * cos(Param::phi3_initial) + Param::p3x * y * sin(Param::phi3_initial) + Param::p3z * z);
        double alpha3_1_imag = ((cos(Param::theta_p) * cos(beta3_1) * (x * cos(Param::phi3_initial) + y * sin(Param::phi3_initial)) - z * sin(beta3_1)) * Param::l3 * cos(gamma3_1) - Param::l3 * sin(Param::theta_p) * sin(gamma3_1) * (x * cos(Param::phi3_initial) + y * sin(Param::phi3_initial)) + x * Param::p3z * cos(Param::phi3_initial) + y * Param::p3z * sin(Param::phi3_initial) - Param::p3x * z);
        double alpha3_1 = atan2(alpha3_1_imag, alpha3_1_real);

        this->gamma3 = gamma3_1;
        this->beta3 = beta3_1;
        m_alpha3 = alpha3_1;
    }
    else if (SelectSolutionIndex[2] == 2) {
        double gamma3_2 = asin((-B_3 - sqrt(pow(B_3, 2) - 4 * A_3 * C_3)) / (2 * A_3));
        double beta3_2_real = (-2 * Param::p3x * (Param::l3 * cos(Param::theta_p) * sin(gamma3_2) - x * sin(Param::phi3_initial) + y * cos(Param::phi3_initial) + Param::p3y));
        double beta3_2_imag = (sin(Param::theta_p) * (-2 * x * Param::p3y * sin(Param::phi3_initial) + 2 * y * Param::p3y * cos(Param::phi3_initial) + pow(x, 2) + pow(y, 2) + pow(z, 2) - pow(Param::p3x, 2) + pow(Param::p3y, 2) - pow(Param::p3z, 2) - pow(Param::l3, 2)) - 2 * Param::p3z * ((x * sin(Param::phi3_initial) - y * cos(Param::phi3_initial) - Param::p3y) * cos(Param::theta_p) - Param::l3 * sin(gamma3_2)));
        double beta3_2 = atan2(beta3_2_imag, beta3_2_real);

        double alpha3_2_real = ((z * cos(Param::theta_p) * cos(beta3_2) + sin(beta3_2) * (x * cos(Param::phi3_initial) + y * sin(Param::phi3_initial))) * Param::l3 * cos(gamma3_2) - Param::l3 * z * sin(Param::theta_p) * sin(gamma3_2) + Param::p3x * x * cos(Param::phi3_initial) + Param::p3x * y * sin(Param::phi3_initial) + Param::p3z * z);
        double alpha3_2_imag = ((cos(Param::theta_p) * cos(beta3_2) * (x * cos(Param::phi3_initial) + y * sin(Param::phi3_initial)) - z * sin(beta3_2)) * Param::l3 * cos(gamma3_2) - Param::l3 * sin(Param::theta_p) * sin(gamma3_2) * (x * cos(Param::phi3_initial) + y * sin(Param::phi3_initial)) + x * Param::p3z * cos(Param::phi3_initial) + y * Param::p3z * sin(Param::phi3_initial) - Param::p3x * z);
        double alpha3_2 = atan2(alpha3_2_imag, alpha3_2_real);

        this->gamma3 = gamma3_2;
        this->beta3 = beta3_2;
        m_alpha3 = alpha3_2;
    }
}