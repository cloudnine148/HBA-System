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
	int cx;			// ���� ����Ʈ x��ǥ
	int cy;			// ���� ����Ʈ y��ǥ
	int px;			// ���� ����Ʈ x��ǥ
	int py;			// ���� ����Ʈ y��ǥ
	double angle;	// optical flow ���� (radian)
	int magnitude;	// optical flow ũ��

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
	int		m_nImgHeight;		// �̹��� Height
	int		m_nImgWidth;		// �̹��� Width
	double	m_dResizeRatio;		// (������������� * m_dResizeRatio = op���� ���� ������)
	bool	m_bSetPrevImg;		// ���� ������ ���� ����
	//bool	m_bFirstFrame;		// ù��° ������ ����
	int		m_nROIHeight;		// optical flow ������ ���� ROI���� Height (0~y��ǥ���� ROI����)	
	//int		m_nGridCount;		// Grid�� Feature Point ����
	int		m_nGridInterval;	// ���󿡼� Grid ����		
	int		m_nVehicleDirectResult;			// ���� ���� �ڵ��� ���� (0:����, 1:��ȸ��, 2:��ȸ��)
	double	m_dVehicleMagnitudeResult;		// ���� ���� �ڵ��� ������ ��� ũ��	
	double	m_dVehicleAngleResult;			// ���� ���� �ڵ��� ������ ��� ����	

	IplImage* m_prevImg;	// ���� ����
	IplImage* m_currImg;	// ���� ����
	IplImage* m_testImg;	// Optical Flow ��� Ȯ���� ���� �׽�Ʈ ����

	OPTICALFLOW_FEATURES* m_op;			// Optical flow ���
	int m_nValidp_count;				// Optical flow ��ȿ ����Ʈ ����

	CvPoint2D32f* m_prev_gridPoint;		// �������� ������ Point �迭
	CvPoint2D32f* m_curr_gridPoint;		// prev_gridPoint�� ���� ��� Point �迭

	int m_nPrevRoiStart_x;				// ���� �����ӿ����� roi ���� ��ǥ (roi box�� ��ܿ���) : ���� ���� �������� ��ǥ
	int m_nPrevRoiStart_y;
	int m_nCurrRoiStart_x;				// ���� ���ο������� roi ���� ��ǥ (roi box�� ��ܿ���) : ���� ���� �������� ��ǥ
	int m_nCurrRoiStart_y;
	int m_nNonMovingCount;				// small moving ���� ������ ī��Ʈ

public:

	// Optical Flow�� ���� ���� �Լ�
	int mainProcessing(IplImage* currImg, int& dir, double& mag, int nCurrentFrameNum);	
	
	// Optical Flow �Լ� �ʱ�ȭ �� �Ҹ�
	void Func_Init(int imgH, int imgW, int gridInterval, double resizeRatio, float rateRoiH=-1.0);
	void Func_Destory();

	// ù��° �Է� ������ ���� ���� ���� �Լ�
	void SetPreviousImg(IplImage* img);

	// ���� ROI ��ǥ �������� �Լ�
	void GetROIvalues(int& roix, int& roiy);

private:

	// optical flow ���� �Լ�
	void GetOpticalFlow(IplImage* preImg, IplImage* currImg, char* status, float* track_error, int nFeatureSize, CvPoint2D32f* prev_features, CvPoint2D32f* curr_features);

	// optical flow ������ ���� grid ���� �Լ�
	int SetGridPoints(IplImage* preImg, IplImage* currImg, CvPoint2D32f* prev_features, CvPoint2D32f* curr_features, int gridInterval);	

	void SetCurrentImg(IplImage* img);	

	// Optical Flow ��� ���� ��� �Լ�
	int print_OpticalFlow_result(int nCurrentFrameNum);
	
	// �ڵ��� ���� ���� ����	
	int extract_vehicle_direct_new(int& dir, double& avg_magnitude, double& result_angle);

	// ������ ��� ũ�� �� ���� ����
	void GetAvgOPMagAng(double& avg_magnitude, double& result_angle);
	
	// optical flow ��� ����
	int get_opticalflow_result(OPTICALFLOW_FEATURES* result);

	// ��ȿ�� op���� ��� ����
	int SetOPValue(char* status, float* track_error, int nFeatureSize, CvPoint2D32f* prev_features, CvPoint2D32f* curr_features);

	// op���� ����� �̿��� ROI ��ǥ ����
	void SetROIvalues(int imgH, int imgW);

	// ROI ���� �� optical flow Ȯ�� (for debugging)
	void print_ROI(IplImage* img, const char* show_name);	

	// Farneback �˰��� �� �̿��� optical flow
	//int mainProcessingForDenseFlow(IplImage* currImg, bool& bUpdateMask, int nCurrentFrameNum);
	//void drawOptFlowMap(const CvMat* flow, CvMat* cflowmap, int step, double scale, CvScalar color);
};



