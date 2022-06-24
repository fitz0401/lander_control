/**
 * @author Jose Davis Nidhin
 * 插值操作去动态内存分配author：徐浩
 * date: 2022.01.26
 * @desc A simple cubic spline interpolation class
 */

#include "CubicSplineNoDynamic.h"

CubicSpline::CubicSpline(){
}

CubicSpline::~CubicSpline(){
}

void CubicSpline::Initialize(double * srcX, double * srcY, const int size)
{
    m_X = srcX;
    m_Y = srcY;
	m_size = size;

    m_B.resize(size-1);
    m_C.resize(size);
    m_D.resize(size - 1);

	H.resize(size - 1);
	alpha.resize(size - 2);
	L.resize(size - 2);
	U.resize(size - 1);
	Z.resize(size - 1);

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
}

double CubicSpline::Interpolate(double x){
    int index = GetIndex(x);

	double y;
	y = m_Y[index] +
		m_B[index] * (x - m_X[index]) +
		m_C[index] * pow((x - m_X[index]), 2) +
		m_D[index] * pow((x - m_X[index]), 3);

	return y;
}

int CubicSpline::GetIndex(double x){
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

