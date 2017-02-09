#include "stdafx.h"
#include "Dib.h"
#include "DibEnhancement.h"
#include "RGBBYTE.h"
#include "Binarization.h"

void Lo_DibHistEqual(CDib& dib, int blockSize, int th[])
{
	register int i, j;
	int x, y;
	BYTE** ptr1 = dib.GetPtr();
	int w = dib.GetWidth();
	int h = dib.GetHeight();
	int temp[256];
	memset(temp, 0, sizeof(int) * 256);

	float hist[256] = { 0.0, };
	int blockCount = 0;

	CBinarization bir;

	//Calculate Histogram
	for (y = 0; y < h; y += blockSize)
	{
		for (x = 0; x < w; x += blockSize)
		{
			memset(temp, 0, sizeof(int) * 256);
			memset(hist, 0.0,sizeof(float)*(float)256);
			for (j = y; j < y + blockSize; j++)
			{
				for (i = x; i < x + blockSize; i++)
				{
					temp[ptr1[j][i]]++;
				}
			}
			//Histogram Normalization
			float area = (float)blockSize*blockSize;
			int n;
			for (n = 0; n < 256; n++)
				hist[n] = temp[n] / area;
			//반복적방법에 의한 임계값 구하기
			th[blockCount] = bir.Lo_DibBinarizationIterative(hist,blockSize);
			blockCount++;
		}
	}
}

void DibHistEqual(CDib& dib)
{
	register int i, j;

	int w = dib.GetWidth();
	int h = dib.GetHeight();

	//Calcuatle Histogram
	float hist[256];
	DibHistogram(dib, hist);

	//히스토그램 누적함수 계산
	double cdf[256] = { 0.0, 0 };
	cdf[0] = hist[0];
	for (i = 1; i < 256; i++)
		cdf[i] = cdf[i - 1] + hist[i];
	//Histogram Equalization
	BYTE** ptr = dib.GetPtr();
	for (j = 0; j < h; j++)
		for (i = 0; i < w; i++)
			ptr[j][i] = (BYTE)limit(cdf[ptr[j][i]] * 255);
}
void DibHistogram(CDib& dib, float histo[256])
{
	register int i, j;
	int w = dib.GetWidth();
	int h = dib.GetHeight();

	BYTE** ptr  = dib.GetPtr();

	//Calucalte Histogram

	int temp[256];
	memset(temp, 0, sizeof(int) * 256);
	for (j = 0; j < h; j++)
		for (i = 0; i < w; i++)
			temp[ptr[j][i]]++;

	//Histogram Normalization
	float area = (float)w*h;
	for (i = 0; i < 256; i++)
		histo[i] = temp[i] / area;
}
