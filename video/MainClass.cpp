#include "stdafx.h"
#include "MainClass.h"

CMainClass::CMainClass(void)
{

}
CMainClass::~CMainClass(void)
{
	
}

void CMainClass::InitValues(int imgh, int imgw)
{
	// ROI 기본 좌표 	
	m_nROIStartX = imgw * ROI_START_RATIO_X;
	m_nROIStartY = imgh * ROI_START_RATIO_Y;
	m_nROIWidth = imgw * ROI_WIDTH_RATIO;
	m_nROIHeight = imgh * ROI_HEIGHT_RATIO;
	SetROIs(imgh, imgw, m_nROIStartX, m_nROIStartY);	

	// Pairing Value 초기화
	m_pair.initPairingValue();

	// Setting ROI : optical Flow와 관련된 변수 세팅	
	m_opticalFlow.Func_Init(imgh, imgw, 4, 0.5, 0.8);	

	m_nPrevFrameNum = 9999999;
}

void CMainClass::DestoryValues()
{
	m_opticalFlow.Func_Destory();
}

// //Global Iterative Binarization
void CMainClass::mainProcessing(CDib& origDib, int frameNum)
{
	CDib dib = origDib;			// color
	CDib dib_ori = origDib;
	CDib gdib = origDib;		// gray
	CDib lbResult;				// 레이블링 결과 영상
	//CDib boxResult;			// detection 결과 영상
	//CDib detectResult;
	//CDib WaveletResult;
	CDib YCbCrResult;
	CDib YResult;
	CDib CrResult;
	CDib BinaryImage;

	int w = dib.GetWidth();
	int h = dib.GetHeight();

	lb detectionResult;			// detection Result boxes 저장
	int labling_cnt = 0;		// labling box 개수
	int detection_cnt = 0;		// detection result 개수

	// Color 영상의 graysclae로 변환	
	DibGrayscale(gdib);	
	YCbCrResult = origDib;

	///////////////////////////
	// ROI Setting
	///////////////////////////	
	sROI start_roi;					// optical flow 결과 roi 영역 시작 좌표 저장 변수
	ExtractROI(gdib, start_roi, frameNum);		
	m_nROIStartX = start_roi.x;
	m_nROIStartY = start_roi.y;
	SetROIs(dib.GetHeight(), dib.GetWidth(), m_nROIStartX, m_nROIStartY);

	// binazation, detection, pairing ROI 설정해야함!!!

	///////////////////////////
	// Thresholding
	///////////////////////////
	DibYCbCrscale(YCbCrResult);
	GetY_Cr(YCbCrResult, YResult, CrResult);

	/*DibGrayscale(YResult);
	DibGrayscale(CrResult);*/

	m_binarization.Binarization(YResult, CrResult, dib, 100, +1.2, +1.5);
	//m_binarization.Binarization(YResult, gdib, 100, -7);		// 150
	//m_binarization.Binarization(CrResult, gdib, 100, -3);	// 150

	//dt.RemoveROIExtern(dib);
	m_detection.MergeImage(YResult, CrResult, BinaryImage);
	m_detection.DibLabeling(BinaryImage, lbResult);

	m_SegmentDib = BinaryImage;

	//gl_drawImage(BinaryImage);	

	///////////////////////////
	// Calssification
	///////////////////////////	
	labling_cnt = m_detection.LabelArrangement();
	m_pair.setCurrentFrame(frameNum);

	if( labling_cnt <= 0 )
	{
		/*if(m_nPrevFrameNum < HL_FRAME_CHECK)
		{
			m_pair.plusHB_Count();
			m_pair.set_Histo_Queue_HIGH(m_nPrevFrameNum);
		}
		else
		{
			m_pair.plusHB_Count();
			m_pair.set_Histo_Queue_HIGH(HL_FRAME_CHECK);
			m_pair.Adjust_HL_Queue();
		}*/
		return;
	}
	m_detection.LabelAreaExtand(w, h);

	m_detection.ExtractColorFeature(dib);
	m_detection.ExtractWaveletFeature(gdib);
	m_detection.ExtractHaarFeatrue(dib);
	m_detection.ExtractColorHistoFeature(dib);

	m_detection.RFWaveletClassification(dib);
	detection_cnt = m_detection.GetDetectionInfo(detectionResult);

	// Blob Save
	//m_detection.SaveRegion(dib, frameNum);

	if( detection_cnt <= 0 )
	{
		return;
	}

	//detectResult = origDib;
	//m_detection.DrawBox(dib, detectResult);

	///////////////////////////
	// Pairing
	///////////////////////////
	boolean globalPairingflag = FALSE;
	//3프레임째이거나 첫프레임일경우. 사용하지않음. 
	if (frameNum % 1 == 0 || frameNum == 1)		globalPairingflag = TRUE;
	else										globalPairingflag = FALSE;	
	m_pair.SetPairing(gdib, detectionResult, globalPairingflag);

	///////////////////////////
	// Tracking
	///////////////////////////
	// 트레킹 백업
	// tracking.detection_n_tracking_processing(m_Dib,dt.getpairInfo(),m_nCurrentFrame);
	
	//DrawROI(boxResult);
	//DrawROI(detectResult);

	/*for (int i = 1; i < dt.DetectionInfo.cnt; i++) {

		TRACE(_T("처리후 : %d =>  w:%d / h:%d\n"), i, dt.DetectionInfo.rg[i].w, dt.DetectionInfo.rg[i].h);


	}*/
}


void CMainClass::GetY_Cr(CDib& dib, CDib& Ydib, CDib& Crdib) 
{
	int h = dib.GetHeight();
	int w = dib.GetWidth();

	Ydib.CreateGrayImage(w, h);
	Crdib.CreateGrayImage(w, h);

	RGBBYTE** ptr = dib.GetRGBPtr();
	BYTE** Yptr = Ydib.GetPtr();
	BYTE** Crptr = Crdib.GetPtr();

	for (int i = 0; i < h; i++) {
		for (int j = 0; j < w; j++) {
			Yptr[i][j] = ptr[i][j].r;
			Crptr[i][j] = ptr[i][j].b;
		}
	}
}

/**
	@brief		현재 영상의 ROI 설정
	@param		dib : 입력영상
				nCurrentFrame : 현재 프레임 번호
*/
void CMainClass::ExtractROI(CDib& dib, sROI& roi, int nCurrentFrame)
{	
	int nVehicleDir = -1;	// -1:초기화, 0:직진, 1:좌회전, 2:우회전
	double dVehicleMag = 0.0;

	IplImage* img = NULL; 

	// 이전 프레임과 연속된 프레임이 아니면 Optical Flow의 이전 영상 재설정
	if( nCurrentFrame == 1 )
	{		
		GetIplImageFromDib(dib, &img);
		m_opticalFlow.SetPreviousImg(img);
		m_nPrevFrameNum = nCurrentFrame;
		roi.x = dib.GetWidth() * ROI_START_RATIO_X;
		roi.y = dib.GetHeight() * ROI_START_RATIO_Y;

	}else if( abs(nCurrentFrame - m_nPrevFrameNum) >= SKIP_FRAME_FOR_ROI )			// 3프레임 간격으로 Optical Flow 체크
	{
		GetIplImageFromDib(dib, &img);
		// Optical Flow 호출 //////////////////////////////////////////
		m_opticalFlow.mainProcessing(img, nVehicleDir, dVehicleMag, nCurrentFrame);	
		m_opticalFlow.GetROIvalues(roi.x, roi.y);	
		m_nPrevFrameNum = nCurrentFrame;
	}
	
	if( img != NULL )
		cvReleaseImage(&img);

	return;		
}

void CMainClass::TrainRF_Detection(CString featurePath, CString treePath, int treeNum, int goodtreeNum, double sampleRatio, int class_num)
{
	m_detection.TrainRF(FILE_PATH_DETECT_FEATURE_WAVELET, 
						FILE_PATH_DETECT_RF_TREE_WAVELET, 
						RF_NUMBER_OF_TREE_F_WIN, 
						RF_NUMBER_OF_GOODTREE_F_WIN, 
						RF_RATIO_OF_SAMPLEDATA_F_WIN, 
						2);


	m_detection.TrainRF(FILE_PATH_DETECT_FEATURE_COLOR_HISTO,
		FILE_PATH_DETECT_RF_TREE_COLOR_HISTO,
		RF_NUMBER_OF_TREE_F_WIN,
		RF_NUMBER_OF_GOODTREE_F_WIN,
		RF_RATIO_OF_SAMPLEDATA_F_WIN,
		2);
}

/**
	@brief		Random Forest 분류기 Tree Data 로드
*/
int CMainClass::LoadRFTreeDataForDetection()
{
	int result = m_detection.LoadAllTreeData();		
	return result;
}

void CMainClass::Setpairing(CDib& dib, lb& detectionInfo, boolean flag)
{
	m_pair.setCurrentFrame(m_nPrevFrameNum);
	m_pair.SetPairing(dib, detectionInfo, flag);
}

PairInfo CMainClass::getpairInfo()
{
	return m_pair.getpairinfo();
}

void CMainClass::GetDetectionResult(CDib& dib, CDib& result)
{
	m_detection.DrawBox(dib, result);
}

void CMainClass::DrawROI(CDib& dib) 
{
	int i, j;
	int w = dib.GetWidth();
	int h = dib.GetHeight();
	int roi_start_x, roi_start_y;
	int roi_end_x, roi_end_y;
	roi_start_x = m_nROIStartX;
	roi_start_y = m_nROIStartY;
	roi_end_x = roi_start_x + m_nROIWidth - 2;
	roi_end_y = roi_start_y + m_nROIHeight;

	RGBBYTE** ptr = dib.GetRGBPtr();

	for (j = roi_start_y; j <= roi_end_y; j++)
	{
		for (i = 1; i < w; i++)
		{

			if (j == roi_start_y && i >= roi_start_x && i <= roi_end_x)
			{
				ptr[j][i].r = 255;
				ptr[j][i].g = 255;
				ptr[j][i].b = 0;

				ptr[j-1][i].r = 255;
				ptr[j-1][i].g = 255;
				ptr[j-1][i].b = 0;

				ptr[j + 1][i].r = 255;
				ptr[j + 1][i].g = 255;
				ptr[j + 1][i].b = 0;

			}

			else if (j == roi_end_y && i >= roi_start_x && i <= roi_end_x)
			{
				ptr[j][i].r = 255;
				ptr[j][i].g = 255;
				ptr[j][i].b = 0;

				ptr[j - 1][i].r = 255;
				ptr[j - 1][i].g = 255;
				ptr[j - 1][i].b = 0;

				ptr[j + 1][i].r = 255;
				ptr[j + 1][i].g = 255;
				ptr[j + 1][i].b = 0;

			}
			else if (roi_start_y <= j && roi_end_y >= j)
			{
				ptr[j][roi_end_x].r = 255;
				ptr[j][roi_end_x].g = 255;
				ptr[j][roi_end_x].b = 0;
				ptr[j][roi_end_x - 1].r = 255;
				ptr[j][roi_end_x - 1].g = 255;
				ptr[j][roi_end_x - 1].b = 0;
				ptr[j][roi_end_x - 2].r = 255;
				ptr[j][roi_end_x - 2].g = 255;
				ptr[j][roi_end_x - 2].b = 0;
				
				ptr[j][roi_start_x].r = 255;
				ptr[j][roi_start_x].g = 255;
				ptr[j][roi_start_x].b = 0;
				ptr[j][roi_start_x + 1].r = 255;
				ptr[j][roi_start_x + 1].g = 255;
				ptr[j][roi_start_x + 1].b = 0;
				ptr[j][roi_start_x + 2].r = 255;
				ptr[j][roi_start_x + 2].g = 255;
				ptr[j][roi_start_x + 2].b = 0;

			}
		}
	}
}

void CMainClass::GetTrackingResult(CDib& reslutDib)
{
	//m_trackingDib.Draw(pDC->m_hDC, left, top, width, height);
	m_tracking.detection_n_tracking_draw_tracking(reslutDib);	
}

void CMainClass::GetPairingResult(CDib& dib, CDib& resultDib)
{
	m_pair.DrawPairBox(dib, resultDib);
}

void CMainClass::Draw_BeamText(CDC *pDC)
{
	m_pair.setbeamText(pDC);
}

void CMainClass::Draw_trackingText(CDC *pDC)
{
	m_tracking.detection_n_tracking_draw_trackingText(pDC);
}

void CMainClass::GetSegmentationResult(CDib& resultDib)
{
	resultDib = m_SegmentDib;
}

/**
	@brief		입력 이미지 크기와 ROI시작 좌표를 이용하여 현재 이미지의 ROI 좌표 설정
	@param		imgh, imgw 이미지 크기
	@param		start_x, start_y ROI 시작 좌표
*/
void CMainClass::SetROIs(int imgh, int imgw, int roi_start_x, int roi_start_y)
{	
	int roi_end_x = roi_start_x + imgw * ROI_WIDTH_RATIO;
	int roi_end_y = roi_start_y + imgh * ROI_HEIGHT_RATIO;
	int froi_end_y = roi_start_y + imgh * FROI_HEIGHT_RATIO;

	//Adaptive Thresholding
	m_binarization.Initvalues(roi_start_x, roi_start_y, roi_end_x, roi_end_y, froi_end_y);

	// detection 
	m_detection.InitValue(roi_start_x, roi_start_y, roi_end_x, roi_end_y);

	// pairing
	m_pair.set_ROI_In_pairing(roi_start_y, roi_end_y);
}

boolean CMainClass::GetPairResult()
{
	return m_pair.getBeamresult();
}