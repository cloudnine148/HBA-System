#pragma once
#include "Dib.h"


// ROI
#define ROI_MIN_RATIO 3.6
#define ROI_MAX_RATIO 6

#define ROI_MIN_X_RATIO 0
#define ROI_MAX_X_RATIO 10

#define FROI_MAX_RATIO 5

struct sROI{
	int x;	// roi Ω√¿€ ¡¬«•
	int y;	// roi Ω√¿€ ¡¬«• 		
};

class CBinarization
{
public:
	CBinarization(void);
	~CBinarization(void);

	//ROI
	int ROI_MIN;
	int ROI_MAX;
	int FROI_MAX;
	int ROI_MIN_X;
	int ROI_MAX_X;

	void Binarization(CDib& Ydib, CDib& CrDib, CDib& dib, int ThresholdValue, double K_y, double K_cr);
	int Lo_DibBinarizationIterative(float hist[], int blockSize);
	void Initvalues(int roi_min_x, int roi_min_y, int roi_max_x, int roi_max_y, int froi_end_y);

private:
	//void Gl_DibBinarization(CDib &dib, int th);
	//int Gl_DibBinarizationIterative(CDib& dib);
	//void Lo_DibBinarization(CDib& dib,int blockSize,int th[]);	
	
	double AdaptiveThresholding(CDib& dib);	
	void PreProcessing(CDib& dib);
	void FirstThresholding(CDib& Ydib, CDib& CrDib, CDib& dib, int ThresholdValue);
	void SecondThresholding(CDib &dib, int ROIth, int FROIth);
	double ROIAdaptiveThresholding(CDib& dib, double K);
	double FROIAdaptiveThresholding(CDib& dib, double K);	
};

