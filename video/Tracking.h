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
	// 연관성 검사를 위한 객체변수
	//헝가리안 메소드
	CMatching matching;
	int* x_y;

	// detection과 tracking의 연관성 검사 시 사용되는 값을 저장하는 배열
	// 거리
	double distance[50];
	// 확률
	double probability[50];
	// ratio
	double ratio[50];
	// 위 4가지 값을 가중치 합한 score값이 저장되는 배열
	double score[50];


	CTracker* tracker;

	// swap을 위한 tracking 클래스 객체
	CTracker tempForSwap;

	PairInfo pair;

	// tracker의 위치 정보 저장
	OTHERTARGETLOCATION otherTarget;


	// 현재 프레임
	int m_currentFrame;

	//현재 생성되어 있는 tracker의 수
	int theNumberOfTracking;

	//Detection된 수
	int m_ntheNumberOfDetection;
	//이전 프레임 번호
	int m_nPrevFrameNum;

	// 마지막으로 할당된 tracker의 id
	int assignedLastTrackerID;

	// tracker의 id 할당 여부 저장
	int** assignedTrackerID;

	// Optical Flow 클래스 
	COpticalFlow m_opticalFlow;

	// Optical Flow Result
	OPTICALFLOW_FEATURES* m_opticalflow_result;
	int m_nOPFfeatures_count;

	// 현재 프레임의 마스크 업데이트 유무
	bool m_bUpdateMask;					

public:
	/**
	@brief		tracking by detection을 수행하는 메인 함수
	detection 수행 후 tracking 수행
	@param		dib : 원본이미지
	currentFrame : 현재 프레임
	*/
	void detection_n_tracking_processing(CDib& dib, PairInfo pair, int& currentFrame);


	//void trackingProcessing(CDib& dib, const int currentFrame, CTracking& tracking, CRect dragInfo);

	/**
	@brief			화면 출력을 위한 tracker의 id 할당 해제 설정
	@param			id : tracker의 id
	*/
	void tracking_setAssignedTrackerID(int id);
	
	/**
	@brief			생성자
	메모리 할당
	변수 초기화
	*/
	void trackingValueInit();
	
	/**
	@brief			생성자
	메모리 할당
	변수 초기화
	tracking by detection 관련
	*/
	void detection_n_tracking_ValueInit();

	/**
	@brief			생성자
	메모리 할당
	변수 초기화
	Regioninfo초기화
	*/
	void detectionValueInit();

	/**
	@brief			임시로 만든 함수
	디텍션정보와 트래킹정보를 매칭
	원래는 Association Check부분
	*/
	void setRectinfo(int& rectcount, int& trackercount, CRect* rectinfo, Pair pairinfo);

	/**
	@brief			tracking을 수행하는 메인 함수
	스레드를 사용할 때는 호출되지 않음
	@param			dib : 원본 영상
	currentFrame : 현재 frame
	tracking : tracking 클래스 객체
	detectionInfo : 매칭된 detection 윈도우의 정보
	otherTarget : 다른 tracker 윈도우의 정보
	processingStep : 수행되어야 할 tracking 단계
	*/
	void trackingProcessing(CDib& dib, const int currentFrame, CTracker& tracking, Pair pairinfo, OTHERTARGETLOCATION& otherTarget, int processingStep);

	/**
	@brief			화면 출력을 위한 tracker의 id 할당
	@return			tracker의 id
	*/
	int detection_tracking_getDisplayTrackerID();

	/**
	@brief			화면 출력을 위한 tracker의 id 할당 여부 및 id 반환
	@param			row : 0 -> tracker의 id
	1 -> tracker의 id 할당 여부
	id : tracker의 id
	@return			tracker의 id 할당 여부 or tracker의 id
	*/
	int detection_n_tracking_getAssignedTrackerID(int row, int id);

	/**
	@brief		tracker 윈도우 화면에 출력
	@param		dib : 원본이미지
	*/
	void detection_n_tracking_draw_tracking(CDib& dib);
	
	/**
	@brief		tracker ID 출력
	@param		pDC : 텍스트 출력을 위한 핸들러
	*/
	void detection_n_tracking_draw_trackingText(CDC *pDC);


	/**
	@brief		tracker ID 출력
	@param		pDC : 텍스트 출력을 위한 핸들러
	*/
	void trackingDraw2(CDC *pDC);

	/**
	@brief		tracker 윈도우 화면에 출력
	@param		dib : 원본이미지
	trk : tracking 클래스 객체
	*/
	void trackingDrawRect(CDib& dib, CTracker& trk);

	/**
	@brief		tracker 윈도우 화면에 출력
	@param		dib : 원본이미지
	*/
	void trackingDraw(CDib& dib);

	/**
	@brief		tracker ID 출력
	@param		pDC : 텍스트 출력을 위한 핸들러
	trk : tracking 클래스 객체
	i : tracker ID
	*/
	void trackingDrawText(CDC *pDC, CTracker& trk, int i);

	/**
	@brief		trakcing 클래스 객체 간 정보 swap
	@param		trk1 : tracking 클래스 객체
	trk2 : tracking 클래스 객체
	*/
	void swap(CTracker& trk1, CTracker& trk2);


	/**
	@brief			소멸자
	메모리 해제
	*/
	void valueDestroy();

	/**
	@brief			소멸자
	메모리 초기화
	*/
	void valueInit();

	/**
	@brief			소멸자
	메모리 해제
	tracking 관련
	*/
	void trackingValueDestroy();

	/**
	@brief			소멸자
	메모리 해제
	tracking by detection 관련
	*/
	void detection_n_tracking_ValueDestroy();

	/**
	@brief		소멸자
	메모리 해제
	detection 관련
	*/
	void detectionValueDestroy();

protected:
	/**
	@brief		다른 tracker의 위치 정보를 저장
	RF update를 결정하기 위하여 사용함
	@param		otherTarget : tracker들의 정보를 저장하는 구조체
	regionInfo1 : detection된 블럭 정보
	trk : tracking된 정보
	theNumberOfDetection : detection된 수
	theNumberOfTracking : tracker 수
	currentFrame : 현재 frame
	*/
	void detection_n_tracking_countTargetLocation(OTHERTARGETLOCATION& otherTarget, Pair* pairinfo, CTracker* trk, int theNumberOfDetection, int theNumberOfTracking, int currentFrame);

	/**
	@brief			OpticalFlow 호출 함수
	@param			dib : 입력 영상
	@param			nCurrentFrame : 현재 프레임 번호
	*/
	int OpticalFlowProcessing(CDib& dib, int nCurrentFrame);

	/**
	@brief		Pairbox와 tracker 간의 연관성 검사
	@param		dib : 원본이미지
	count1 : detection된 pairbox 갯수
	count2 : tracking된 pairbox 갯수
	pair : detection된 pair 정보
	trk : tracking된 정보
	*/
	void MatchingPairBoxWithTracking(CDib& dib, int& count1, int& count2, Pair* pair, CTracker* trk);
	
};

