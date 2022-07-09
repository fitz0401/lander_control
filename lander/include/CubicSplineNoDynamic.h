/**
 * @author Jose Davis Nidhin
 * ��ֵ����ȥ��̬�ڴ����author�����
 * date: 2022.01.26
 */

#include <math.h>
#include <iostream>
#include <vector>

using namespace std;

class CubicSpline
{
public:
	CubicSpline();
	~CubicSpline();

    void Initialize(double * srcX, double * srcY, int size);
    double Interpolate(double x);

private:
    double* m_X, * m_Y;
    vector<double> m_B, m_C, m_D;
    //double m_B[99], m_C[100], m_D[99];
    unsigned int m_size;
    vector<double> H, alpha, L, U, Z;
    //double H[99], alpha[98], L[98], U[99], Z[99];

    int GetIndex(double x);
};
