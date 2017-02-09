#pragma once
#define MAX_LABEL 1000

#include "Dib.h"
#include "DibColor.h"
#include "dforest.h"
#include "Wavelet.h"

#include <opencv2\opencv.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\core\core.hpp>
//#include <iostream>
//#include <string>
//#include <fstream>

//using namespace std;

class CDetection
{
public:
	CDetection();
	~CDetection();

	void Labeling(CDib dib);

	int ROI_MIN;
	int ROI_MAX;
	int ROI_MIN_X;
	int ROI_MAX_X;

	// ���̺�
	int DibLabeling(CDib& dib, CDib& result);

	// ���̺� ���� ������ ����
	void DrawBox(CDib& dib, CDib& result);

	// ROI �ܴ̿� ����
	void RemoveROIExtern(CDib& dib);

	// ���̺� ��� ����ü
	lb DetectionInfo;

	// ���̺� ����
	int LabelArrangement();
	void RemoveIdx(int idx);

	void MergeImage(CDib& dib1, CDib& dib2, CDib& result);
	
	// ���̺� ���� Ű���
	void LabelAreaExtand(int w, int h);

	// �з�
	void Classfication(CDib& dib);

	void TrainSVM();
	void SVMClassification();
	CvSVM svm;

	// Ư¡ ����
	RANDOMFORESTS RFInfo_Color;
	void ExtractColorFeature(CDib& dib);
	void ExtractColorHistoFeature(CDib& dib);

	// RF
	RANDOMFORESTS RFInfo;

	void TrainRF(CString featurePath, CString treePath, int treeNum, int goodtreeNum, double sampleRatio, int class_num);
	int LoadTreeData(CString treePath, RANDOMFORESTS& rf);
	int LoadAllTreeData();
	void RFColorClassification();
	void RFWaveletClassification(CDib& dib);
	
	void GetOCSLBPFeatureFromImage(BYTE** ptr, int height, int width, double* feature, int featureSize, int blkWidth, int blkHeight, int binSize, int threshold);
	double GetRedness(int num, CDib& dib);

	// Wavelet
	CWavelet Wavelet;
	void ExtractWaveletFeature(CDib& dib);


	// Haar
	RANDOMFORESTS RFInfo_first;

	void generateIntegralImage(CDib& dib, double** integralValueForIntensity);
	void GetHaarlikeFeatureOfWindow(CDib& dib, int labelNum);
	int ComputeHaarLikeFeatureValue(int labelNum, int* block_sum, int num);
	double getHaarLikePatternValue(int* block_sum, std::vector<int>* positiveList, std::vector<int>* negativeList);

	void ExtractHaarFeatrue(CDib& dib);


	// OCS _ First
	RANDOMFORESTS RFInfo_first_tail;

	void InitValue(int roi_min_x, int roi_min_y, int roi_max_x, int roi_max_y);
	int GetDetectionInfo(lb& detectionInfo);


	// Save Only
	void SaveRegion(CDib& dib, int frameNum);

};

