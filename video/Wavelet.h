#pragma once
#include "Dib.h"
#include "DibColor.h"
#define IMAGE_SIZE_WIDTH 752
#define IMAGE_SIZE_HEIGHT 480


class CWavelet
{
public:
	CWavelet();
	~CWavelet();

	double**	m_TempImg;
	double**	m_LowImg;
	double	C0, C1, C2, C3;

	void change_int(CDib& dib);
	void Daub4b(CDib& dib, int num, int nheight, int nwidth);
	void transform(int row, int n);
	void transpose(int nn);
};

