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
    unsigned int m_size;
    vector<double> H, alpha, L, U, Z;

    int GetIndex(double x);
};
