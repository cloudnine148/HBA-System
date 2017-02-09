#pragma once
#include "Dib.h"
#include "DibColor.h"
#include "RGBBYTE.h"
#include "common.h"
#include "Matching.h"
#include <math.h>

// ��ü���� PairBox������ �����ϴ� ����ü
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

//Pair�� ���� Box�� ������ŭ ����.
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
	// Pairing�� ���� �� Ȯ���ϴ� ROI ����
	int ROI_MIN;
	int ROI_MAX;
	int ROI_HEIGHT;
	int ROI_FIRST_END, ROI_SECOND_END, ROI_THIRD_END, ROI_FOURTH_END;

	// �� ��е� ROI_END���� + OVERLAP_RATIO. ����(2017-02-07)�� ������� ����
	int ROI_FIRST_END_OVER, ROI_SECOND_END_OVER, ROI_THIRD_END_OVER, ROI_FOURTH_END_OVER;

	// �� level������ inner-width�� ��
	// �� 5���� level�� �������� ����.
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
	
	// �밡���� �޼ҵ�
	CMatching matching;
	int* x_y;

	// Beam Control�� ���� �迭.
	int histo_HL[2];
	int histo_HL_Queue[HL_FRAME_CHECK + 1];
	int currentframe;


public:
	/**
	@brief			��ü���� Pairing ����
	�ʱ�ȭ
	Association Check
	Pairing
	*/
	void SetPairing(CDib& dib, lb& DetectionInfo, boolean GlobalPairing);

	/**
	@brief			�� Blob���� ���� ��
	return			�� Blob�� ũ���� ���� Ȯ�� 0 ~ 1
	*/
	double Compare_Area(int firstArea, int secondArea);
	
	/**
	@brief			�� Blob���� Y�� ��ħ�� ��
	return			�� Blob�� Y�� ��ħ Ȯ�� 0 ~ 1
	*/
	double Compare_Y_Overlap(int x1, int x2, int y1, int y2);

	/**
	@brief			�� Blob���� Width �� Height�� ���� ���� ��
	return			�� Blob�� ���� ������ Ȯ�� 0 ~1
	*/
	double Compare_Width_and_Height(int r_min_x, int r_max_x, int r_min_y, int r_max_y, int l_min_x, int l_max_x, int l_min_y, int l_max_y);

	/**
	@brief			�� blob ���� �Ÿ� �� 12-05 �߰�
	���� �ڵ忡���� Association Checking�� �Ű������� ������ ���� ����� �ϰ� ���� ����.
	*/
	double Compare_Distance_Two_blobs(int inner_width, int min_y);


	/**
	@brief			Association Check
	�տ��� ������� Area, Y�� ��ħ, Aspect ration�� �̿��Ͽ� Score table�� �������, Hungaria Algorithm�� �̿��Ͽ�
	Score matrix�� ����. �ش� Blob�� ���ؼ� Pairing�� ����.
	*/
	void Matching_Pairing_With_Detection(int count1, double** area, double** y_over, double** ratio, double** distance, lb& DetectionInfo);

	/////////// ���� Ư¡�� ����� ������� ���� //////////
	boolean Compare_width_and_box_ratio(int r_min_x, int r_max_x, int r_min_y, int r_max_y, int l_min_x, int l_max_x, int l_min_y, int l_max_y, boolean GlobalPairing);

	boolean Compare_correlation(CDib& Left_dib, CDib& Right_dib, double l_mean, double r_mean, double l_sdev, double r_sdev, boolean GlobalPairing);
	///////////////////////////////////////////////////////

	/*
	@brief			���� �� �ִ� PairBox�� ������ ����. 
	Detection Blob�� ���� ���� �޶�����.
	*/
	void SetPairInfo(lb& DetectionInfo);


	/*
	@brief			 Pair���� ������ �˾� �� �� �ִ�.
	*/
	PairInfo getpairinfo();

	/*
	@brief			 Pairing�� ���� Blob�� �ִ� Box�� Draw
	*/
	void DrawPairBox(CDib& dib, CDib& result);

	/*
	@brief			 Pairing�� ���Ǵ� ������ ����ü �ʱ�ȭ
	*/
	void initPairing(lb& DetectionInfo);

	// Y�� ��Ī ���󸸵��
	void DibMirror(CDib& dib);

	// ROI �޾ƿ���
	void set_ROI_In_pairing(int ROI_MIN, int ROI_MAX);

	/*
	@brief			 Beam - Control.
	Text ���
	*/
	void setbeamText(CDC* pDC);

	/*
	@brief			 Beam - Control. 
	���� Beam�� HIGH���� LOW����
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
	������ QUEUE�� HIGH_BEAM���� set
	����� ������� ����
	*/
	void set_Histo_Queue_HIGH(int currentFrame);

	/*
	@brief			 Beam - Control.
	������ QUEUE�� LOW_BEAM���� set
	����� ������� ����
	*/
	void set_Histo_Queue_LOW(int currentFrame);

	/*
	@brief			 Beam - Control.
	QUEUE�� ���� �տ� �Ҵ� ��Ұ� LOW���� HIGH���� �Ǵ��ؼ� QUEUE�� ����.
	*/
	void Adjust_HL_Queue();

	/*
	@brief			 Beam - Control.
	HIGH_BEAM�� LOW_BEAM�� ���� ���� �ʱ�ȭ.
	*/
	void initPairingValue();
	
};

