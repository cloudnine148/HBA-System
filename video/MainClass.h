#pragma once

#include "Dib.h"
#include "DibColor.h"
#include "DibCv.h"
#include "Binarization.h"
#include "Detection.h"
#include "Tracking.h"
#include "Pairing.h"
#include "OpticalFlow.h"
#include "common.h"

class CMainClass
{
public:
	CMainClass(void);
	~CMainClass(void);
	
	void mainProcessing(CDib& origDib, int frameNum);	
	void InitValues(int imgh, int imgw);
	void DestoryValues();

	int LoadRFTreeDataForDetection();

	void GetDetectionResult(CDib& dib, CDib& result);
	void GetTrackingResult(CDib& reslutDib);
	void GetSegmentationResult(CDib& resultDib);
	void GetPairingResult(CDib& dib, CDib& result);
	void Draw_BeamText(CDC *pDC);
	void Draw_trackingText(CDC *pDC);	
	void DrawROI(CDib& dib);
	void TrainRF_Detection(CString featurePath, CString treePath, int treeNum, int goodtreeNum, double sampleRatio, int class_num);

	boolean GetPairResult();

private:
	// Binarization
	CBinarization m_binarization;
	// Detection
	CDetection m_detection;
	// Pairing
	CPairing m_pair;
	// Tracking
	CTracking m_tracking;
	// ROISetting	
	COpticalFlow m_opticalFlow;		// Optical Flow 클래스
	
	int m_nPrevFrameNum;			// 이전 frame Num	

	CDib m_SegmentDib;				// 이진화 결과 영상

	int m_nROIStartX;				// ROI 시작 x좌표
	int m_nROIStartY;				// ROI 시작 y좌표 
	int m_nROIWidth;				// ROI width
	int m_nROIHeight;				// ROI height


	void GetY_Cr(CDib& dib, CDib& Ydib, CDib& Crdib);		

	// Binarization

	// ROISetting
	void ExtractROI(CDib& dib, sROI& roi, int nCurrentFrame);
	void SetROIs(int imgh, int imgw, int roi_start_x, int roi_start_y);
	
	// detection
		
	// pairing
	void Setpairing(CDib& dib, lb& detectionInfo, boolean flag);
	PairInfo getpairInfo();
	

	// tracking
	
};

