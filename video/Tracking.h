#pragma once
#include "Dib.h"
#include "DibColor.h"
#include "Detection.h"
#include "Tracker.h"
#include "Matching.h"
#include "common.h"
#include "stdafx.h"
#include "OpticalFlow.h"
#include "Pairing.h"


class CTracking
{
public:
	CTracking(void);
	~CTracking(void);

protected:
	// ������ �˻縦 ���� ��ü����
	//�밡���� �޼ҵ�
	CMatching matching;
	int* x_y;

	// detection�� tracking�� ������ �˻� �� ���Ǵ� ���� �����ϴ� �迭
	// �Ÿ�
	double distance[50];
	// Ȯ��
	double probability[50];
	// ratio
	double ratio[50];
	// �� 4���� ���� ����ġ ���� score���� ����Ǵ� �迭
	double score[50];


	CTracker* tracker;

	// swap�� ���� tracking Ŭ���� ��ü
	CTracker tempForSwap;

	PairInfo pair;

	// tracker�� ��ġ ���� ����
	OTHERTARGETLOCATION otherTarget;


	// ���� ������
	int m_currentFrame;

	//���� �����Ǿ� �ִ� tracker�� ��
	int theNumberOfTracking;

	//Detection�� ��
	int m_ntheNumberOfDetection;
	//���� ������ ��ȣ
	int m_nPrevFrameNum;

	// ���������� �Ҵ�� tracker�� id
	int assignedLastTrackerID;

	// tracker�� id �Ҵ� ���� ����
	int** assignedTrackerID;

	// Optical Flow Ŭ���� 
	COpticalFlow m_opticalFlow;

	// Optical Flow Result
	OPTICALFLOW_FEATURES* m_opticalflow_result;
	int m_nOPFfeatures_count;

	// ���� �������� ����ũ ������Ʈ ����
	bool m_bUpdateMask;					

public:
	/**
	@brief		tracking by detection�� �����ϴ� ���� �Լ�
	detection ���� �� tracking ����
	@param		dib : �����̹���
	currentFrame : ���� ������
	*/
	void detection_n_tracking_processing(CDib& dib, PairInfo pair, int& currentFrame);


	//void trackingProcessing(CDib& dib, const int currentFrame, CTracking& tracking, CRect dragInfo);

	/**
	@brief			ȭ�� ����� ���� tracker�� id �Ҵ� ���� ����
	@param			id : tracker�� id
	*/
	void tracking_setAssignedTrackerID(int id);
	
	/**
	@brief			������
	�޸� �Ҵ�
	���� �ʱ�ȭ
	*/
	void trackingValueInit();
	
	/**
	@brief			������
	�޸� �Ҵ�
	���� �ʱ�ȭ
	tracking by detection ����
	*/
	void detection_n_tracking_ValueInit();

	/**
	@brief			������
	�޸� �Ҵ�
	���� �ʱ�ȭ
	Regioninfo�ʱ�ȭ
	*/
	void detectionValueInit();

	/**
	@brief			�ӽ÷� ���� �Լ�
	���ؼ������� Ʈ��ŷ������ ��Ī
	������ Association Check�κ�
	*/
	void setRectinfo(int& rectcount, int& trackercount, CRect* rectinfo, Pair pairinfo);

	/**
	@brief			tracking�� �����ϴ� ���� �Լ�
	�����带 ����� ���� ȣ����� ����
	@param			dib : ���� ����
	currentFrame : ���� frame
	tracking : tracking Ŭ���� ��ü
	detectionInfo : ��Ī�� detection �������� ����
	otherTarget : �ٸ� tracker �������� ����
	processingStep : ����Ǿ�� �� tracking �ܰ�
	*/
	void trackingProcessing(CDib& dib, const int currentFrame, CTracker& tracking, Pair pairinfo, OTHERTARGETLOCATION& otherTarget, int processingStep);

	/**
	@brief			ȭ�� ����� ���� tracker�� id �Ҵ�
	@return			tracker�� id
	*/
	int detection_tracking_getDisplayTrackerID();

	/**
	@brief			ȭ�� ����� ���� tracker�� id �Ҵ� ���� �� id ��ȯ
	@param			row : 0 -> tracker�� id
	1 -> tracker�� id �Ҵ� ����
	id : tracker�� id
	@return			tracker�� id �Ҵ� ���� or tracker�� id
	*/
	int detection_n_tracking_getAssignedTrackerID(int row, int id);

	/**
	@brief		tracker ������ ȭ�鿡 ���
	@param		dib : �����̹���
	*/
	void detection_n_tracking_draw_tracking(CDib& dib);
	
	/**
	@brief		tracker ID ���
	@param		pDC : �ؽ�Ʈ ����� ���� �ڵ鷯
	*/
	void detection_n_tracking_draw_trackingText(CDC *pDC);


	/**
	@brief		tracker ID ���
	@param		pDC : �ؽ�Ʈ ����� ���� �ڵ鷯
	*/
	void trackingDraw2(CDC *pDC);

	/**
	@brief		tracker ������ ȭ�鿡 ���
	@param		dib : �����̹���
	trk : tracking Ŭ���� ��ü
	*/
	void trackingDrawRect(CDib& dib, CTracker& trk);

	/**
	@brief		tracker ������ ȭ�鿡 ���
	@param		dib : �����̹���
	*/
	void trackingDraw(CDib& dib);

	/**
	@brief		tracker ID ���
	@param		pDC : �ؽ�Ʈ ����� ���� �ڵ鷯
	trk : tracking Ŭ���� ��ü
	i : tracker ID
	*/
	void trackingDrawText(CDC *pDC, CTracker& trk, int i);

	/**
	@brief		trakcing Ŭ���� ��ü �� ���� swap
	@param		trk1 : tracking Ŭ���� ��ü
	trk2 : tracking Ŭ���� ��ü
	*/
	void swap(CTracker& trk1, CTracker& trk2);


	/**
	@brief			�Ҹ���
	�޸� ����
	*/
	void valueDestroy();

	/**
	@brief			�Ҹ���
	�޸� �ʱ�ȭ
	*/
	void valueInit();

	/**
	@brief			�Ҹ���
	�޸� ����
	tracking ����
	*/
	void trackingValueDestroy();

	/**
	@brief			�Ҹ���
	�޸� ����
	tracking by detection ����
	*/
	void detection_n_tracking_ValueDestroy();

	/**
	@brief		�Ҹ���
	�޸� ����
	detection ����
	*/
	void detectionValueDestroy();

protected:
	/**
	@brief		�ٸ� tracker�� ��ġ ������ ����
	RF update�� �����ϱ� ���Ͽ� �����
	@param		otherTarget : tracker���� ������ �����ϴ� ����ü
	regionInfo1 : detection�� �� ����
	trk : tracking�� ����
	theNumberOfDetection : detection�� ��
	theNumberOfTracking : tracker ��
	currentFrame : ���� frame
	*/
	void detection_n_tracking_countTargetLocation(OTHERTARGETLOCATION& otherTarget, Pair* pairinfo, CTracker* trk, int theNumberOfDetection, int theNumberOfTracking, int currentFrame);

	/**
	@brief			OpticalFlow ȣ�� �Լ�
	@param			dib : �Է� ����
	@param			nCurrentFrame : ���� ������ ��ȣ
	*/
	int OpticalFlowProcessing(CDib& dib, int nCurrentFrame);

	/**
	@brief		Pairbox�� tracker ���� ������ �˻�
	@param		dib : �����̹���
	count1 : detection�� pairbox ����
	count2 : tracking�� pairbox ����
	pair : detection�� pair ����
	trk : tracking�� ����
	*/
	void MatchingPairBoxWithTracking(CDib& dib, int& count1, int& count2, Pair* pair, CTracker* trk);
	
};

