/**
 * @author Jose Davis Nidhin
 */

#pragma once

#include <math.h>
#include <iostream>

using namespace std;

class CubicSpline
{
public:
	CubicSpline();
	~CubicSpline();

    void Initialize(double * srcX, double * srcY, int size);
    double Interpolate(double x);

private:
    double *m_B, *m_C, *m_D, *m_X, *m_Y;
	int m_size;

    int GetIndex(double x);
    double Interpolate(double x, int index);
	void Reset();
};
