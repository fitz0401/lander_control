/**
 * @author Jose Davis Nidhin
 * @desc A simple cubic spline interpolation class
 */

#include "CubicSpline.h"

/**
 *
 */
CubicSpline::CubicSpline()
{}

CubicSpline::~CubicSpline()
{
	Reset();
}

void CubicSpline::Initialize(double * srcX, double * srcY, const int size)
{
    m_X = srcX;
    m_Y = srcY;
	m_size = size;

	Reset();

    m_B = new double[size-1];
    m_C = new double[size];
    m_D = new double[size-1];
//    double m_B[size-1];
//    double m_C[size];
//    double m_D[size-1];

//    double * H = new double[size-1];
//    double * alpha = new double[size-2];
    double H[size-1];
    double alpha[size-2];

    double L[size-2];
    double U[size-1];
    double Z[size-1];

	for (int i = 0; i < size - 1; i++) {
		H[i] = srcX[i + 1] - srcX[i];
	}

	for (int i = 1; i < size - 1; i++) {
        alpha[i-1] = (3.0 / H[i] * (srcY[i + 1] - srcY[i])) - (3.0 / H[i - 1] * (srcY[i] - srcY[i - 1]));
	}

    U[0] = Z[0] = 0.0;

	for (int i = 1; i < size - 1; i++) {
        L[i-1] = (2.0 * (srcX[i + 1] - srcX[i - 1])) - (H[i - 1] * U[i - 1]);
		U[i] = H[i] / L[i-1];
		Z[i] = (alpha[i-1] - (H[i - 1] * Z[i - 1])) / L[i-1];
	}

    m_C[size - 1] = 0.0;

	for (int i = size - 2; i >= 0; i--) {
		m_C[i] = Z[i] - (U[i] * m_C[i + 1]);
		m_B[i] = ((srcY[i + 1] - srcY[i]) / H[i]) - (H[i] * (m_C[i + 1] + 2 * m_C[i]) / 3);
		m_D[i] = (m_C[i + 1] - m_C[i]) / (3 * H[i]);
	}

    delete [] m_B;
    delete [] m_C;
    delete [] m_D;
}

double CubicSpline::Interpolate(double x)
{
	int index = GetIndex(x);
	return Interpolate(x, index);
}

int CubicSpline::GetIndex(double x)
{
	int last = m_size - 1;
	int first = 0;
	int mid;

	while (last - first > 1) {
		mid = (last + first) / 2;

		if (x > m_X[mid])
			first = mid;
		else
			last = mid;
	}

	return first;
}

double CubicSpline::Interpolate(double x, int index)
{
    double y;

	y = m_Y[index] +
		m_B[index] * (x - m_X[index]) +
		m_C[index] * pow((x - m_X[index]), 2) +
		m_D[index] * pow((x - m_X[index]), 3);

	return y;
}

void CubicSpline::Reset()
{
	delete[] m_B;
	delete[] m_C;
	delete[] m_D;
}
