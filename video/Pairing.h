#pragma once
#include "Dib.h"
#include "DibColor.h"
#include "RGBBYTE.h"
#include "common.h"
#include "Matching.h"
#include <math.h>

// 전체적인 PairBox정보를 저장하는 구조체
struct Pair
{
	// Blob Info
	sRegion LeftBlobInfo;
	sRegion RightBlobInfo;
	CDib LeftBlobDib;
	CDib RightBlobDib;

	int left_index;
	int right_index;

	// Box coordinate point
	int box_min_x;
	int box_max_x;
	int box_min_y;
	int box_max_y;
	int box_center_x;
	int box_center_y;

	// Area Info
	int leftArea;
	int rightArea;

	// Y Vertical Info
	int leftVertical;
	int rightVertical;

	// Box Ratio Info
	int boxWidth;
	int boxHeight;
	double boxRatio;

	// Total Score
	double score;

	// Pair Info
	int pairNumber;
	int pairType;

	 // For tracking
	int particleNumber;
};

//Pair를 맺은 Box의 갯수만큼 생성.
struct PairInfo
{
	Pair* pairinfo;
	int pair_cnt;
};

class CPairing
{
public:
	CPairing();
	~CPairing();

public:
	// Pairing을 맺을 때 확인하는 ROI 영역
	int ROI_MIN;
	int ROI_MAX;
	int ROI_HEIGHT;
	int ROI_FIRST_END, ROI_SECOND_END, ROI_THIRD_END, ROI_FOURTH_END;

	// 각 등분된 ROI_END에서 + OVERLAP_RATIO. 현재(2017-02-07)는 사용하지 않음
	int ROI_FIRST_END_OVER, ROI_SECOND_END_OVER, ROI_THIRD_END_OVER, ROI_FOURTH_END_OVER;

	// 각 level마다의 inner-width의 값
	// 총 5개의 level로 나뉘어져 있음.
	int PAIR_BLOB_FIRST_DISTANCE;
	int PAIR_BLOB_SECOND_DISTANCE;
	int PAIR_BLOB_THIRD_DISTANCE;
	int PAIR_BLOB_FOURTH_DISTANCE;
	int PAIR_BLOB_FIFTH_DISTANCE;


protected:
	PairInfo pair;
	boolean duplicate;
	/*
	True = High-Beam
	False = Low-Beam
	*/
	boolean beamControl;
	
	// 헝가리안 메소드
	CMatching matching;
	int* x_y;

	// Beam Control을 위한 배열.
	int histo_HL[2];
	int histo_HL_Queue[HL_FRAME_CHECK + 1];
	int currentframe;


public:
	/**
	@brief			전체적인 Pairing 수행
	초기화
	Association Check
	Pairing
	*/
	void SetPairing(CDib& dib, lb& DetectionInfo, boolean GlobalPairing);

	/**
	@brief			두 Blob간의 영역 비교
	return			두 Blob간 크기의 닯은 확률 0 ~ 1
	*/
	double Compare_Area(int firstArea, int secondArea);
	
	/**
	@brief			두 Blob간의 Y축 겹침율 비교
	return			두 Blob간 Y축 겹침 확률 0 ~ 1
	*/
	double Compare_Y_Overlap(int x1, int x2, int y1, int y2);

	/**
	@brief			두 Blob간의 Width 와 Height간 닮은 비율 비교
	return			두 Blob간 닮은 정도의 확률 0 ~1
	*/
	double Compare_Width_and_Height(int r_min_x, int r_max_x, int r_min_y, int r_max_y, int l_min_x, int l_max_x, int l_min_y, int l_max_y);

	/**
	@brief			각 blob 사이 거리 비교 12-05 추가
	지금 코드에서는 Association Checking의 매개변수로 들어가지만 실제 사용은 하고 있지 않음.
	*/
	double Compare_Distance_Two_blobs(int inner_width, int min_y);


	/**
	@brief			Association Check
	앞에서 만들어진 Area, Y축 겸침, Aspect ration를 이용하여 Score table을 만든다음, Hungaria Algorithm을 이용하여
	Score matrix를 생성. 해당 Blob에 대해서 Pairing을 수행.
	*/
	void Matching_Pairing_With_Detection(int count1, double** area, double** y_over, double** ratio, double** distance, lb& DetectionInfo);

	/////////// 이전 특징들 현재는 사용하지 않음 //////////
	boolean Compare_width_and_box_ratio(int r_min_x, int r_max_x, int r_min_y, int r_max_y, int l_min_x, int l_max_x, int l_min_y, int l_max_y, boolean GlobalPairing);

	boolean Compare_correlation(CDib& Left_dib, CDib& Right_dib, double l_mean, double r_mean, double l_sdev, double r_sdev, boolean GlobalPairing);
	///////////////////////////////////////////////////////

	/*
	@brief			나올 수 있는 PairBox의 개수를 지정. 
	Detection Blob의 수에 따라 달라진다.
	*/
	void SetPairInfo(lb& DetectionInfo);


	/*
	@brief			 Pair들의 정보를 알아 낼 수 있다.
	*/
	PairInfo getpairinfo();

	/*
	@brief			 Pairing을 맺은 Blob이 있는 Box를 Draw
	*/
	void DrawPairBox(CDib& dib, CDib& result);

	/*
	@brief			 Pairing에 사용되는 변수와 구조체 초기화
	*/
	void initPairing(lb& DetectionInfo);

	// Y축 대칭 영상만들기
	void DibMirror(CDib& dib);

	// ROI 받아오기
	void set_ROI_In_pairing(int ROI_MIN, int ROI_MAX);

	/*
	@brief			 Beam - Control.
	Text 출력
	*/
	void setbeamText(CDC* pDC);

	/*
	@brief			 Beam - Control. 
	현재 Beam이 HIGH인지 LOW인지
	*/
	boolean getBeamresult();

	void setCurrentFrame(int m_currentframe);

	/*
	@brief			 Beam - Control.
	HIGH_BEAM + 1
	*/
	void plusHB_Count();

	/*
	@brief			 Beam - Control.
	HIGH_BEAM - 1
	*/
	void MinusHB_Count();

	/*
	@brief			 Beam - Control.
	LOW_BEAM + 1
	*/
	void PlusLB_Count();

	/*
	@brief			 Beam - Control.
	LOW_BEAM - 1
	*/
	void MinusLB_Count();

	/*
	@brief			 Beam - Control.
	마지막 QUEUE에 HIGH_BEAM으로 set
	현재는 사용하지 않음
	*/
	void set_Histo_Queue_HIGH(int currentFrame);

	/*
	@brief			 Beam - Control.
	마지막 QUEUE에 LOW_BEAM으로 set
	현재는 사용하지 않음
	*/
	void set_Histo_Queue_LOW(int currentFrame);

	/*
	@brief			 Beam - Control.
	QUEUE의 가장 앞에 았는 요소가 LOW인지 HIGH인지 판단해서 QUEUE를 조정.
	*/
	void Adjust_HL_Queue();

	/*
	@brief			 Beam - Control.
	HIGH_BEAM과 LOW_BEAM이 쌓일 스택 초기화.
	*/
	void initPairingValue();
	
};

