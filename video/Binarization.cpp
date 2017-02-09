#include "stdafx.h"
#include "Binarization.h"
#include "Dib.h"
#include "DibEnhancement.h"
#include <cstdlib>
#include "math.h"


CBinarization::CBinarization(void)
{
}


CBinarization::~CBinarization(void)
{
}

void CBinarization::PreProcessing(CDib& dib)
{
	register int i, j;

	int w = dib.GetWidth();
	int h = dib.GetHeight();

	BYTE** ptr = dib.GetPtr();

	for (j = 0; j < h; j++) {
		for (i = 0; i < w; i++)
		{
			if (ptr[j][i] < 100)
				ptr[j][i] = 0;
		}
	}
/*
	for (j = 0; j < h*0.4; j++) {
		for(i = 0; i < w; i++) {
			ptr[j][i] = 0;
		}
	}

	for (j = h*0.8; j < h; j++) {
		for (i = 0; i < w; i++) {
			ptr[j][i] = 0;
		}
	}*/
}

//int CBinarization::Gl_DibBinarizationIterative(CDib& dib)
//{
//	register int i;
//
//	int w = dib.GetWidth();
//	int h = dib.GetHeight();
//
//	float hist[256] = { 0, };
//
//	//Normalized histogram을 구함. 
//	DibHistogram(dib, hist);
//	//초기 임계값 설정
//	int T, Told;
//
//	float sum = 0.f;
//	for (i = 0; i < 256; i++)
//	{
//		sum += (i*hist[i]);
//	}
//	T = (int)sum;
//
//	//반복에 의한 임계값 설정
//	float a1, b1, u1, a2, b2, u2;
//
//	do
//	{
//		Told = T;
//
//		a1 = b1 = 0;
//		for (i = 0; i <= Told; i++)
//		{
//			a1 += (i*hist[i]);
//			b1 += hist[i];
//		}
//		u1 = a1 / b1;
//
//		a2 = b2 = 0;
//		for (i = Told + 1; i < 256; i++)
//		{
//			a2 += (i*hist[i]);
//			b2 += hist[i];
//		}
//		u2 = a2 / b2;
//
//		if (b1 == 0)
//			b1 = 1.f;
//		if (b2 == 0)
//			b2 = 1.f;
//		T = (int)((u1 + u2) / 2);
//	} while (T == Told);
//	return T;
//}

//void CBinarization::Gl_DibBinarization(CDib &dib, int th)
//{
//	register int i, j;
//
//	int w = dib.GetWidth();
//	int h = dib.GetHeight();
//
//	BYTE** ptr = dib.GetPtr();
//
//	for (j = 0; j < h; j++) {
//		for (i = 0; i < w; i++)
//		{
//			if (j >= h*0.4 && j <= h*0.5) {
//				
//				ptr[j][i] = (ptr[j][i] > (th - 10 > 0 ? th-10 : 0) ) ? 255 : 0;
//			} 
//			else if (j > h*0.5 && j<= h*0.8) {
//				ptr[j][i] = (ptr[j][i] > th) ? 255 : 0;
//			}
//			
//		}
//	}
//}

int CBinarization::Lo_DibBinarizationIterative(float hist[], int blockSize)
{
	register int i;

	//초기 임계값 설정
	int T, Told;

	float sum = 0.f;
	for (i = 0; i < 256; i++)
	{
		sum += (i*hist[i]);
	}
	T = (int)sum;

	//반복에 의한 임계값 설정
	float a1, b1, u1, a2, b2, u2;
	int loopCount = 0;
	do
	{
		Told = T;

		a1 = b1 = 0;
		for (i = 0; i <= Told; i++)
		{
			a1 += (i*hist[i]);
			b1 += hist[i];
		}
		if (b1 == 0.f)
			u1 = 0;
		else
			u1 = a1 / b1;

		a2 = b2 = 0;
		for (i = Told + 1; i < 256; i++)
		{
			a2 += (i*hist[i]);
			b2 += hist[i];
		}
		if (a2 == 0.f)
			u2 = 0;
		else
			u2 = a2 / b2;

		if (b1 == 0)
			b1 = 1.f;
		if (b2 == 0)
			b2 = 1.f;
		T = (int)(((u1 + u2) / (double)2) + 0.5);
		loopCount++;
	} while (T == Told && loopCount <= 3);
	return T;
}


//void CBinarization::Lo_DibBinarization(CDib& dib, int blockSize,int th[])
//{
//	register int i, j;
//	BYTE** ptr = dib.GetPtr();
//	int w = dib.GetWidth();
//	int h = dib.GetHeight();
//	int x, y;
//	int blockCount = 0;
//	double fixelAverage = 0.0;
//	//for (i = 0; i < 510; i++)
////		th[i] = rand() % 15 + 100;
//	
//	/*
//	for (i = 0; i < 510; i++)
//	{
//		if (th[i] < 45)
//			th[i] = 45;
//	}
//	*/
//	for (y = 0; y < h; y += blockSize)
//	{
//		for (x = 0; x < w; x += blockSize)
//
//		{
//			for (j = y; j < y + blockSize; j++)
//			{
//				for (i = x; i < x + blockSize; i++)
//				{
//					ptr[j][i] = (ptr[j][i] > th[blockCount]) ? 255 : 0;
//				}
//			}
//			blockCount++;
//		}
//	}
//}

void CBinarization::FirstThresholding(CDib& Ydib, CDib& CrDib, CDib& dib, int ThresholdValue)
{
	register int i, j;

	int ImageWidth = dib.GetWidth();
	int ImageHeight = dib.GetHeight();

	BYTE** Yptr = Ydib.GetPtr();
	BYTE** Crptr = CrDib.GetPtr();
	RGBBYTE** ptr = dib.GetRGBPtr();


	//ROI(FROI포함한)영역 이외의 영역은 모두 0으로 set
	//ROI영역 윗부분
	for (j = 0; j < ROI_MIN; j++)
		for (i = 0; i < ImageWidth; i++)
		{
			ptr[j][i].r = 0;
			ptr[j][i].g = 0;
			ptr[j][i].b = 0;

			Yptr[j][i] = 0;
			Crptr[j][i] = 0;
		}
			
	//ROI영역 아랫부분
		for (j = ROI_MAX; j < ImageHeight; j++)
		for (i = 0; i < ImageWidth; i++)
		{
			ptr[j][i].r = 0;
			ptr[j][i].g = 0;
			ptr[j][i].b = 0;

			Yptr[j][i] = 0;
			Crptr[j][i] = 0;
		}

	/*
	//ROI영역 왼쪽 부분
	for (j = ROIStartingPointY; j < ROIEndingPointY; j++)
	{
		for (i = 0; i < ROIStartingPointX; i++)
		{
			ptr[j][i] = 0;
		}
	}

	//ROI영역 오른쪽 부분
	for (j = ROIStartingPointY; j < ROIEndingPointY; j++)
	{
		for (i = ROIEndingPointX; i < ImageWidth; i++)
		{
			ptr[j][i] = 0;
		}
	}
	*/

	//FROI영역 1차임계값 적용
		for (j = ROI_MIN; j < FROI_MAX; j++)
			for (i = ROI_MIN_X; i < ROI_MAX_X; i++)
			{
				if (ptr[j][i].r < ThresholdValue-60 && ptr[j][i].g < ThresholdValue && ptr[j][i].b < ThresholdValue)
				{
					ptr[j][i].r = 0;
					ptr[j][i].g = 0;
					ptr[j][i].b = 0;

					Yptr[j][i] = 0;
					Crptr[j][i] = 0;
				}

				else
					int a=0;
			}
		//ROI영역 1차 임계값 적용
		for (j = FROI_MAX; j < ROI_MAX; j++)
			for (i = ROI_MIN_X; i < ROI_MAX_X; i++)
			{
				if (ptr[j][i].r < ThresholdValue-60 && ptr[j][i].g < ThresholdValue && ptr[j][i].b < ThresholdValue)
				{
					ptr[j][i].r = 0;
					ptr[j][i].g = 0;
					ptr[j][i].b = 0;

					Yptr[j][i] = 0;
					Crptr[j][i] = 0;
				}
				else
					int a=0;
			}

		
}
//2차 임계값(AdaptiveThresholding을 이용한) 적용
void CBinarization::SecondThresholding(CDib &dib, int ROIth, int FROIth)
{
	register int i, j;

	int ImageWidth = dib.GetWidth();
	int ImageHeight = dib.GetHeight();

	BYTE** ptr = dib.GetPtr();
	
	//FROI영역에 2차 임계값(Adaptive Thresholding을 통해 구한)적용
	for (j = ROI_MIN; j < FROI_MAX; j++)
		for (i = ROI_MIN_X; i < ROI_MAX_X; i++)
			ptr[j][i] = (ptr[j][i] > FROIth) ? 255 : 0;

	//ROI영역에 2차 임계값(Adative Thresholding을 통해 구한) 적용
	for (j = FROI_MAX; j < ROI_MAX; j++)
		for (i = ROI_MIN_X; i < ROI_MAX_X; i++)
			ptr[j][i] = (ptr[j][i] > ROIth) ? 255 : 0;

}


//ROI영역에서 Adaptive Thresholding을 통한 Threshold값을 구하는 함수
double CBinarization::ROIAdaptiveThresholding(CDib& dib, double K)
{
	register int i, j;

	int ImageWidth = dib.GetWidth();
	int ImageHeight = dib.GetHeight();
	int pxSum = 0;
	double pxAverage = 0.0;
	double pxStandardDeviation = 0.0;
	double pxVariance = 0.0;
	double deviationSum = 0.0;
	double th = 0.0;

	BYTE** ptr = dib.GetPtr();
	int upperPxCount = 0;

	for (j = FROI_MAX; j < ROI_MAX; j++)
		for (i = ROI_MIN_X; i < ROI_MAX_X; i++)
		{
			if (ptr[j][i] > 0)
			{
				pxSum += ptr[j][i];
				upperPxCount++;
			}
		}

		if(upperPxCount == 0)
			return 0.0;

	//1프레임의 픽셀 평균값 
	pxAverage = (double)pxSum / (double)upperPxCount;

	for (j = FROI_MAX; j < ROI_MAX; j++)
		for (i = ROI_MIN_X; i < ROI_MAX_X; i++)
		{
			if (ptr[j][i] > 0)
				deviationSum += ((double)ptr[j][i] - pxAverage) * ((double)ptr[j][i] - pxAverage);
		}
	pxVariance = deviationSum / ((double)upperPxCount);
	pxStandardDeviation = sqrt(pxVariance);

	//임계값 구하는 공식 
	th = pxAverage - (K*pxStandardDeviation);

	//계산된 임계값이 255를 초과할때 최대값 255로 fix
	if (th >= 255.0)
		th = 255.0;
	
	else if(th <= 0)
		th = 0;
	//TRACE(_T("%lf\n"), th);
	return th;
}
//FROI영역에서 Adaptive Thresholding 구하는 함수
double CBinarization::FROIAdaptiveThresholding(CDib& dib, double K)
{
	register int i, j;

	int ImageWidth = dib.GetWidth();
	int ImageHeight = dib.GetHeight();
	int pxSum = 0;
	double pxAverage = 0.0;
	double pxStandardDeviation = 0.0;
	double pxVariance = 0.0;
	double deviationSum = 0.0;
	double th = 0.0;

	BYTE** ptr = dib.GetPtr();
	int upperPxCount = 0;

	for (j = ROI_MIN; j < FROI_MAX; j++)
		for (i = ROI_MIN_X; i < ROI_MAX_X; i++)
		{
			if (ptr[j][i] > 0)
			{
				pxSum += ptr[j][i];
				upperPxCount++;
			}
		}

		if(upperPxCount == 0)
			return 0.0;

	//1프레임의 픽셀 평균값 
	pxAverage = (double)pxSum / (double)upperPxCount;

	for (j = ROI_MIN; j < FROI_MAX; j++)
		for (i = ROI_MIN_X; i < ROI_MAX_X; i++)
		{
			if (ptr[j][i] > 0)
				deviationSum += ((double)ptr[j][i] - pxAverage) * ((double)ptr[j][i] - pxAverage);
		}

	pxVariance = deviationSum / ((double)upperPxCount);
	pxStandardDeviation = sqrt(pxVariance);

	//임계값 구하는 공식 
	th = pxAverage - (K*pxStandardDeviation);

	//계산된 임계값이 255를 초과할때 최대값 255로 fix
	if (th >= 255.0)
		th = 255.0;

	else if(th <= 0)
		th = 0;
	//TRACE(_T("%lf\n"), th);
	return th;
}


// 영상 이진화를 위한 mian fuction
void CBinarization::Binarization(CDib& Ydib, CDib& CrDib, CDib& grayDib, int ThresholdValue, double K_y, double K_cr) 
{
	/////////////////////////////////////////
	// 1차 thresholding : 노이즈 제거
	/////////////////////////////////////////
	FirstThresholding(Ydib, CrDib, grayDib, ThresholdValue);

	/////////////////////////////////////////
	// 2차 thresholding : 자동차 light 추출
	/////////////////////////////////////////
	//Adaptive Threshold value 추출 
	int ROIThresholdValue = (int)ROIAdaptiveThresholding(Ydib, K_y);
	int FROIThresholdValue = (int)FROIAdaptiveThresholding(Ydib, K_y);		
	SecondThresholding(Ydib, ROIThresholdValue, FROIThresholdValue);

	ROIThresholdValue = (int)ROIAdaptiveThresholding(CrDib, K_cr);
	FROIThresholdValue = (int)FROIAdaptiveThresholding(CrDib, K_cr);		
	SecondThresholding(CrDib, ROIThresholdValue, FROIThresholdValue);

}

void CBinarization::Initvalues(int roi_min_x, int roi_min_y, int roi_max_x, int roi_max_y, int froi_end_y)
{
	ROI_MIN = roi_min_y;
	ROI_MIN_X = roi_min_x;
	FROI_MAX = froi_end_y;
	ROI_MAX = roi_max_y;	
	ROI_MAX_X = roi_max_x;
}