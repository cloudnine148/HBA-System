#include "stdafx.h"
#include "Wavelet.h"


CWavelet::CWavelet()
{
	C0 = 0.4829629131445341;
	C1 = 0.8365163037378079;
	C2 = 0.2241438680420134;
	C3 = -0.1294095225512604;

}


CWavelet::~CWavelet()
{
}

// 320 240

void CWavelet::Daub4b(CDib& dib, int num, int nheight, int nwidth)
{
	int i, j, n, k;

	DibResizeBilinear(dib, 32,32);

	BYTE** inImage = dib.GetPtr();

	//int maxNum = max(nheight, nwidth);
	int size = 32;

	m_TempImg = new double*[size];
	m_LowImg = new double*[size];

	for (i = 0; i < size; i++) {
		m_TempImg[i] = new double[size];
		m_LowImg[i] = new double[size];

		memset(m_TempImg[i], 0, sizeof(double) * size);
		memset(m_LowImg[i], 0, sizeof(double) * size);
	}


	/*for (i = 0; i<nwidth; i++)
		for (j = 0; j<nwidth; j++)
		{
			m_TempImg[i][j] = 0.0;
			m_LowImg[i][j] = 0.0;
		}

	for (i = 0; i<nheight; i++){
		for (k = 0; k<nwidth; k++){
			m_TempImg[i][k] = (double)inImage[i][k];
		}
	}

	for (k = 0; k<nwidth; k++){
		for (n = nwidth; n >= nwidth; n >>= 1){
			transform(k, n);
		}
		for (i = 0; i<nwidth; i++) m_TempImg[k][i] = m_LowImg[k][i];
	}

	transpose(nwidth);

	for (k = 0; k<nwidth; k++){
		for (n = nwidth; n >= nwidth; n >>= 1){
			transform(k, n);
		}
		for (i = 0; i<nwidth; i++) m_TempImg[k][i] = m_LowImg[k][i];
	}

	transpose(nwidth);*/


	for (i = 0; i<size; i++)
		for (j = 0; j<size; j++)
		{
			m_TempImg[i][j] = 0.0;
			m_LowImg[i][j] = 0.0;
		}

	for (i = 0; i<size; i++){
		for (k = 0; k<size; k++){
			m_TempImg[i][k] = (double)inImage[i][k];
		}
	}

	for (k = 0; k<size; k++){
		for (n = size; n >= size; n >>= 1){
			transform(k, n);
		}
		for (i = 0; i<size; i++) m_TempImg[k][i] = m_LowImg[k][i];
	}

	transpose(size);

	for (k = 0; k<size; k++){
		for (n = size; n >= size; n >>= 1){
			transform(k, n);
		}
		for (i = 0; i<size; i++) m_TempImg[k][i] = m_LowImg[k][i];
	}

	transpose(size);

	change_int(dib);


	for (int i = 0; i < size; i++) {
		delete[] m_TempImg[i];
		delete[] m_LowImg[i];
	}

	delete[] m_TempImg;
	delete[] m_LowImg;
}

void CWavelet::transform(int k, int n)
{
	int i, j;

	if (n >= 5) {
		int half = n >> 1;

		for (i = 0, j = 0; j < n - 3; j += 2, i++) {
			m_LowImg[k][i] = m_TempImg[k][j] * C0 + m_TempImg[k][j + 1] * C1 + m_TempImg[k][j + 2] * C2 + m_TempImg[k][j + 3] * C3;
			m_LowImg[k][i + half] = m_TempImg[k][j] * C3 - m_TempImg[k][j + 1] * C2 + m_TempImg[k][j + 2] * C1 - m_TempImg[k][j + 3] * C0;
		}
		m_LowImg[k][i] = m_TempImg[k][n - 2] * C0 + m_TempImg[k][n - 1] * C1 + m_TempImg[k][0] * C2 + m_TempImg[k][1] * C3;
		m_LowImg[k][i + half] = m_TempImg[k][n - 2] * C3 - m_TempImg[k][n - 1] * C2 + m_TempImg[k][0] * C1 - m_TempImg[k][1] * C0;
	}
}

void CWavelet::transpose(int nn)
{
	int i, j;
	double temp;

	for (i = 0; i<nn; i++) {
		for (j = 0; j<i; j++) {
			temp = m_TempImg[i][j];
			m_TempImg[i][j] = m_TempImg[j][i];
			m_TempImg[j][i] = temp;
		}
	}
}

void CWavelet::change_int(CDib& dib)
{
	double max = 0;
	double min = 0;

	int x, y;
	
	BYTE tmpVal;

	BYTE** inImage = dib.GetPtr();

	int image_height = dib.GetHeight();
	int image_width = dib.GetWidth();

	int interval = 0;

	for (x = 0; x < image_height / 2; x++)
		for (y = 0; y < image_width / 2; y++){
			if (max<fabs(m_TempImg[x][y]))
				max = fabs(m_TempImg[x][y]);
		}
	for (x = 0; x < image_height / 2; x++)
		for (y = 0; y < image_width / 2; y++){
			tmpVal = (BYTE)((fabs(m_TempImg[x][y]) / max) * 255.0);
			inImage[x][y] = tmpVal;
		}

	max = 0;

	for (x = 0; x < image_height / 2; x++)
		for (y = image_width / 2; y < image_width; y++){
			if (max<fabs(m_TempImg[x][y]))
				max = fabs(m_TempImg[x][y]);
		}

	for (x = 0; x < image_height / 2; x++)
		for (y = image_width / 2; y < image_width; y++){
			tmpVal = (BYTE)((fabs(m_TempImg[x][y]) / max) * 255.0);
			inImage[x][y] = tmpVal;
		}

	max = 0;

	for (x = image_height / 2; x < image_height; x++)
		for (y = 0; y < image_width / 2; y++){
			if (max<fabs(m_TempImg[x + interval][y]))
				max = fabs(m_TempImg[x + interval][y]);
		}

	for (x = image_height / 2; x < image_height; x++)
		for (y = 0; y < image_width / 2; y++){
			tmpVal = (BYTE)((fabs(m_TempImg[x + interval][y]) / max) * 255.0);
			inImage[x][y] = tmpVal;
		}


	max = 0;

	for (x = image_height / 2; x < image_height; x++)
		for (y = image_width / 2; y < image_width; y++){
			if (max<fabs(m_TempImg[x + interval][y]))
				max = fabs(m_TempImg[x + interval][y]);
		}

	for (x = image_height / 2; x < image_height; x++) {
		for (y = image_width / 2; y < image_width; y++){
			tmpVal = (BYTE)((fabs(m_TempImg[x + interval][y]) / max) * 255.0);
			inImage[x][y] = tmpVal;
		}
	}


}