#pragma once
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/legacy/legacy.hpp>
#include "common.h"

#define PI2 3.1415
#define GO_STRAIGHT	0
#define LEFT_TURN		1 
#define RIGHT_TURN		2

struct OPTICALFLOW_FEATURES
{
	int cx;			// 현재 포인트 x좌표
	int cy;			// 현재 포인트 y좌표
	int px;			// 이전 포인트 x좌표
	int py;			// 이전 포인트 y좌표
	double angle;	// optical flow 방향 (radian)
	int magnitude;	// optical flow 크기

	OPTICALFLOW_FEATURES()
	{
		cx = px = -1;
		cy = py = -1;
		angle = -1;
		magnitude = -1;
	}
};

class COpticalFlow
{
public:
	COpticalFlow();
	~COpticalFlow(void);

public:
	
private:	
	int		m_nImgHeight;		// 이미지 Height
	int		m_nImgWidth;		// 이미지 Width
	double	m_dResizeRatio;		// (원본영상사이즈 * m_dResizeRatio = op추출 영상 사이즈)
	bool	m_bSetPrevImg;		// 이전 프레임 설정 유무
	//bool	m_bFirstFrame;		// 첫번째 프레임 유무
	int		m_nROIHeight;		// optical flow 추출을 위한 ROI영역 Height (0~y좌표까지 ROI설정)	
	//int		m_nGridCount;		// Grid의 Feature Point 개수
	int		m_nGridInterval;	// 영상에서 Grid 간격		
	int		m_nVehicleDirectResult;			// 최종 추출 자동차 방향 (0:직진, 1:좌회전, 2:우회전)
	double	m_dVehicleMagnitudeResult;		// 최종 추출 자동차 움직임 평균 크기	
	double	m_dVehicleAngleResult;			// 최종 추출 자동차 움직임 평균 각도	

	IplImage* m_prevImg;	// 이전 영상
	IplImage* m_currImg;	// 현재 영상
	IplImage* m_testImg;	// Optical Flow 결과 확인을 위한 테스트 영상

	OPTICALFLOW_FEATURES* m_op;			// Optical flow 결과
	int m_nValidp_count;				// Optical flow 유효 포인트 개수

	CvPoint2D32f* m_prev_gridPoint;		// 움직임을 추정할 Point 배열
	CvPoint2D32f* m_curr_gridPoint;		// prev_gridPoint에 대한 결과 Point 배열

	int m_nPrevRoiStart_x;				// 이전 프레임에서의 roi 시작 좌표 (roi box의 상단왼쪽) : 원본 영상 사이즈의 좌표
	int m_nPrevRoiStart_y;
	int m_nCurrRoiStart_x;				// 현재 프로엠에서의 roi 시작 좌표 (roi box의 상단왼쪽) : 원본 영상 사이즈의 좌표
	int m_nCurrRoiStart_y;
	int m_nNonMovingCount;				// small moving 연속 프레임 카운트

public:

	// Optical Flow를 위한 메인 함수
	int mainProcessing(IplImage* currImg, int& dir, double& mag, int nCurrentFrameNum);	
	
	// Optical Flow 함수 초기화 및 소멸
	void Func_Init(int imgH, int imgW, int gridInterval, double resizeRatio, float rateRoiH=-1.0);
	void Func_Destory();

	// 첫번째 입력 영상의 이전 영상 설정 함수
	void SetPreviousImg(IplImage* img);

	// 현재 ROI 좌표 가져오기 함수
	void GetROIvalues(int& roix, int& roiy);

private:

	// optical flow 추출 함수
	void GetOpticalFlow(IplImage* preImg, IplImage* currImg, char* status, float* track_error, int nFeatureSize, CvPoint2D32f* prev_features, CvPoint2D32f* curr_features);

	// optical flow 추출을 위한 grid 설정 함수
	int SetGridPoints(IplImage* preImg, IplImage* currImg, CvPoint2D32f* prev_features, CvPoint2D32f* curr_features, int gridInterval);	

	void SetCurrentImg(IplImage* img);	

	// Optical Flow 결과 영상 출력 함수
	int print_OpticalFlow_result(int nCurrentFrameNum);
	
	// 자동차 주행 방향 추출	
	int extract_vehicle_direct_new(int& dir, double& avg_magnitude, double& result_angle);

	// 움직임 평균 크기 및 각도 추출
	void GetAvgOPMagAng(double& avg_magnitude, double& result_angle);
	
	// optical flow 결과 추출
	int get_opticalflow_result(OPTICALFLOW_FEATURES* result);

	// 유효한 op추출 결과 저장
	int SetOPValue(char* status, float* track_error, int nFeatureSize, CvPoint2D32f* prev_features, CvPoint2D32f* curr_features);

	// op추출 결과를 이용한 ROI 좌표 추출
	void SetROIvalues(int imgH, int imgW);

	// ROI 영역 및 optical flow 확인 (for debugging)
	void print_ROI(IplImage* img, const char* show_name);	

	// Farneback 알고리즘 을 이용한 optical flow
	//int mainProcessingForDenseFlow(IplImage* currImg, bool& bUpdateMask, int nCurrentFrameNum);
	//void drawOptFlowMap(const CvMat* flow, CvMat* cflowmap, int step, double scale, CvScalar color);
};



