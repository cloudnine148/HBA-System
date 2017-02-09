#pragma once
#include "RGBBYTE.h"
#include "Dib.h"
#include "DibColor.h"
#include "Dib.h"
#include "OpticalFlow.h"
#include "common.h"
#include "RandomFern.h"
#include <math.h>



struct PARTLOCATION
{
	double probability;

	int leftOffset;
	int topOffset;
	int rightOffset;
	int bottomOffset;
};

struct PARTICLEINFO
{
	int x;
	int y;

	int w;
	int h;

	// Random Forests 확률값
	double probability;

	// 이전 타겟과 파티클 간의 거리
	double distance;

	// 거리와 확률값의 가중치 합
	double score;

	// OCS-LBP 특징 히스토그램
	double OCSLBP_histogram[OCSLBP_FEATURE_SIZE];
	// OCS-LBP distance
	double OCSLBP_distance;

	// LID 특징 히스토그램
	double LID_histogram[LID_FEATURE_SIZE];

	// Color 특징 히스토그램
	double Color_histogram[COLOR_FEATURE_SIZE_FOR_TRACKING];
	// Color distance
	double Color_distance;

	double new_histogram[NEW_FEATURE_SIZE];

	// part 위치
	PARTLOCATION PART_location[SUB_BLOCK];
	// 0 : head
	// 1 : left arm
	// 2 : right arm
	// 3 : left leg
	// 4 : right leg

	// part 별 확률값
	double OCSLBP_part_probability;

	int occlusionType;
	// 1 : 정상 추적
	// 2 : 부분 겹침
	// 3 : 완전 겹침
	// 4 : 객체 놓침
};

struct TARGETPARTICLE
{
	int x;
	int y;
	int w;
	int h;
	double probability;

	// part 위치
	PARTLOCATION PART_location[SUB_BLOCK];
	// 0 : head
	// 1 : left arm
	// 2 : right arm
	// 3 : left leg
	// 4 : right leg
};

struct OTHERTARGETLOCATION
{
	int theNumberOfOtherTarget;
	int myTargetNumber;

	int targetIndexs[TRACKING_NUM];
	int location[TRACKING_NUM][4];

	double targetProbability[TRACKING_NUM];

	// 누적 Y좌표를 위한 변수
	// tracker 생성시 모두 0으로 초기화
	int targetYaxis[TRACKING_NUM][3];
	int trackerCount;
	int avgYaxis[TRACKING_NUM];
};



class CTracker
{
public:
	CTracker();
	~CTracker();

public:
	// 내부 detection과 매칭용 id
	int trackerIndex;
	// 화면 표시용 id
	int displayTrackerID;
	//Framecount
	int m_Frame_Count_posi;
	int m_Frame_Count_nega;

protected:
	PARTICLEINFO* currentParticles;
	PARTICLEINFO* preParticles;

	TARGETPARTICLE preTargetParticle;
	TARGETPARTICLE estTargetParticle;

	//분류기
	CRandom_Fern rf;
	RANDOMFERNS rf_ocslbp;
	RANDOMFERNS rf_color;

	// 추출된 학습 데이터를 클래스 별로 분류하여 BRFs를 학습 시킬 때 사용하는 배열
	// [클래스 수][데이터 수][특징 차원 수]
	double*** trainDataSet_ocslbp;
	double*** trainDataSet_color;

	//	전역 OCS-LBP 특징 히스토그램
	double Template_OCSLBP_histogram[OCSLBP_FEATURE_SIZE];
	//	전역	Color 특징 히스토그램
	double Template_Color_histogram[COLOR_FEATURE_SIZE_FOR_TRACKING];

	// 현재 tracker가 detection과 매칭되지 않은 수
	int occlusionCount;

	// tracker가 영상 경계에 있을 경우
	// 화면에만 안 나타나게 하는 것
	int notDisplayCount;

	bool matchingFlag;

	// 해당 tracker가 생성된 프레임
	int frame;
	// update가 수행된 프레임
	int updateFrame;

	// 윈도우 크기 별 겹침 거리 임계값이 저장되는 변수
	double distBetweenObjectsThreshold;

	// 가속도 정보 저장
	int velocity_index;
	int velocity[2][2];

	// tracker 속도 계산을 위한 이전 frame 위치 저장
	int previousX;
	int previousY;

	// 현재 tracker와 이전 tracker 간의 거리
	double distanceBetweenPreAndCurr;

	// 추정된 현재 tracker와 다른 tracker의 거리 중 가장
	// 짧은 거리가 저장되는 변수
	double distanceBetweenObjects;
	double ratioBetweenObjects;

	// 겹침 구분을 위한 변수
	int targetYaxis[3];
	int trackerCount;
	int avgYaxis;

	int size_level;


protected:
	/**
	@brief			Detection 정보를 이용하여 tracker 윈도우의 초기 위치 및 크기 설정
	@param			particle : 이전 tracker의 정보 저장
	particle2 : 현재 tracker의 파티클 정보 저장
	cx, cy : 매칭된 detection 윈도우의 중심 좌표
	min_x, min_y, max_x, max_y : 매칭된 detection 윈도우의 좌표
	*/
	void getTargetLocationFromDetection(TARGETPARTICLE& particle, PARTICLEINFO* particle2, int& cx, int& cy, int& min_x, int& min_y, int& max_x, int& max_y);

	/**
	@brief			파티클 샘플링
	객체 윈도우의 크기에 따라 샘플링 범위가 다름
	@param			flag : 새로 할당된 tracker -> true, 기존에 할당된 tracker -> false
	xPoint, yPoint : 타겟 또는 매칭된 detection 윈도우의 중심 좌표
	particle1 : 현재 tracker의 파티클 정보 저장
	distanceBetweenTargets : 현재 tracker와 다른 tracker 간의 거리
	threshold : tracker 간의 겹침 발생 거리 임계값
	*/
	void createParticle(bool flag, const int& xPoint, const int& yPoint, PARTICLEINFO* particle1, int distanceBetweenTargets, double& threshold, int velocity[2][2]);

	/**
	@brief			가우시안 분포를 이용하여 파티클 샘플링
	@param			flag : 새로 할당된 tracker -> true, 기존에 할당된 tracker -> false
	xPoint, yPoint : 타겟 또는 매칭된 detection 윈도우의 중심 좌표
	particle1 : 현재 tracker의 파티클 정보 저장
	std : 파티클이 샘플링 되는 범위 (분산)
	*/
	void createParticlesForObject(bool flag, const int& xPoint, const int& yPoint, PARTICLEINFO* particle1, int std, int particleDistributionFlag);

	/**
	@brief			가우시안 분포 생성
	파티클을 가우시안 분포로 샘플링
	@param			mean : 평균
	std : 분산
	@return			가우시안 분포로 추정된 수
	*/
	double GaussianRand(const double& mean, const double& std);

	/**
	@brief			샘플링된 파티클 위치에 가속도를 더해줌
	@param			particle1 : 현재 tracker의 파티클 정보 저장
	velocity : t, t-1 프레임의 가속도 정보 저장
	*/
	void createParticleWithVelocity(PARTICLEINFO* particle1, int velocity[2][2]);

	/**
	@brief			Random ferns 학습
	@param			dib1 : Gray Image
	dib2 : HSI Image
	particle1 : 현재 tracker의 파티클 정보 저장
	df_ocslbp : OCS-LBP 특징을 사용하는 분류기
	df_color : Hue 특징을 사용하는 분류기
	@return			입력된 특징 벡터에 의해 추정된 확률값
	*/
	void RFtraining(CDib& dib, CDib& dib2, PARTICLEINFO* particle1, RANDOMFERNS& rf1, RANDOMFERNS& rf2, bool flag1, bool flag2);

	void trainingDataExtractionForTraining(CDib& dib, CDib& dib2, PARTICLEINFO& particle1, double** feature1, double** feature2, int& index, int flag);
	
	void getTwoFeatures(CDib& dib, CDib& dib2, PARTICLEINFO& particle);

	void L2Normalization(double* histogram, int histBins);

	/**
	@brief			특징 추출 (OCS-LBP, Hue)
	모든 파티클 대상
	@param			dib1 : Gray Image
	dib2 : HSI Image
	particle1 : 현재 tracker의 파티클 정보 저장
	*/
	void extractFeature(CDib& dib, CDib& dib2, PARTICLEINFO* particle1);

	void extractFeatureAtPartForTwoFeatures(CDib& dib, CDib& dib2, PARTICLEINFO& particle/*, double& firstProbability, double& secondProbability*/);

	/**
	@brief			Random forest를 이용한 특징의 확률 추정
	@param			feature : 특징 벡터
	df : 분류기
	featureSize : 특징 크기
	thenumberofclass : 클래스 수
	@return			입력된 특징 벡터에 의해 추정된 확률값
	*/
	double RFtesting(double* input, RANDOMFERNS& rf_classifier);

	/**
	@brief			OCS-LBP와 Color의 Template 배열을 만들어 준다
	*/
	void CreateTemplateValue(CDib& dib, CDib& dib2, int& cx, int& cy, int& min_x, int& min_y, int& max_x, int& max_y);

	/**
	@brief			score 값을 기반으로 파티클을 내림차순으로 정렬
	@param			particle1 : 현재 tracker의 파티클 정보 저장
	particle2 : 이전 또는 추정된 tracker 정보 저장
	n : 파티클 수
	*/
	void sortParticles(PARTICLEINFO* particle1, TARGETPARTICLE& particle2, const int& n, int velocity[2][2]);

	/**
	@brief			버블 정렬
	@param			particle1 : 현재 tracker의 파티클 정보 저장
	n : 파티클 수
	*/
	void bubbleSort(PARTICLEINFO* particle1, const int& n);

	/**
	@brief			상위 N개의 파티클을 이용하여 새로운 타겟의 상태 추정
	연관성 검사 알고리즘 수행 전에 수행됨
	@param			particle1 : 추정된 tracker 정보 저장
	particle2 : 현재 tracker의 파티클 정보 저장
	*/
	void estimateNewTargetStateUsingUpperParticles(TARGETPARTICLE& particle1, PARTICLEINFO* particle2);

	/**
	@brief			매칭 된 detection이 없을 경우 이전 tracker 정보를 이용하여 타겟 state 업데이트
	@param			particle1 : 추정된 tracker 정보 저장
	particle2 : 이전 tracker 정보 저장
	@return			업데이트 된 타겟과 이전 tracker 타겟 간의 거리
	*/
	double estimateNewTargetStateUsingPreTrackerInformation(TARGETPARTICLE& particle1, TARGETPARTICLE& particle2);

	/**
	@brief			새로운 타겟의 확률값을 추정하고 파티클 구조체에 저장하는 함수
	기존 파티클 구조체의 내용을 뒤로 한 칸씩 이동함
	@param			dib1 : Gray Image
	dib2 : HSI Image
	particle1 : 추정된 tracker 정보 저장
	particle2 : 현재 tracker의 파티클 정보 저장
	*/
	void moveParticleToNextIndex(CDib& dib, CDib& dib2, TARGETPARTICLE& particle1, PARTICLEINFO* particle2);

	/**
	@brief			최종 타겟의 특징 추출 후 확률 추정
	@param			dib1 : Gray Image
	dib2 : HSI Image
	particle1 : 현재 tracker의 파티클 정보 저장 (1개)
	*/
	void extractTargetFeature(CDib& dib, CDib& dib2, PARTICLEINFO& particle1);

	/**
	@brief			현재 tracker와 다른 tracker 간의 거리 계산
	@param			otherTarget : 다른 tracker의 위치 정보 저장
	@return			계산된 거리 중 가장 짧은 거리 반환
	*/
	int measureDistanceBetweenOtherTargets(TARGETPARTICLE& particle, OTHERTARGETLOCATION& otherTarget, double& ratioBetweenObjects);

	void setPreviousTrackerLocation(int px, int py);
	void getPreviousTrackerLocation(int& px, int& py);

	/**
	@brief			매칭 된 detection 윈도우 정보를 이용하여 타겟의 상태 업데이트
	연관성 검사 알고리즘 수행 후에 수행됨
	@param			particle1 : 추정된 tracker 정보 저장
	particle2 : 이전 tracker 정보 저장
	detectionCenterX, detectionCenterY : 매칭된 detection 윈도우의 중심 좌표
	detectionW, detectionH : 매칭된 detection 윈도우의 크기
	@return			업데이트 된 타겟과 이전 tracker 타겟 간의 거리
	*/
	double estimateNewTargetStateUsingDetectionInformation(TARGETPARTICLE& particle1, TARGETPARTICLE& particle2, int& detectionCenterX, int& detectionCenterY, int& detectionW, int& detectionH);

public:
	void initNotDisplayCounnt();

	/**
	@brief			Detection된 정보를 이용하여 새로운 tracker 생성
	@param			dib1 : Gray Image
	dib2 : HSI Image
	flag : 새로 할당된 tracker -> true, 기존에 할당된 tracker -> false
	cx, cy : 매칭된 detection 윈도우의 중심 좌표
	min_x, min_y, max_x, max_y : 매칭된 detection 윈도우의 좌표
	*/
	void processGenerationTracker(CDib& dib, CDib& dib2, bool flag, int& cx, int& cy, int& min_x, int& min_y, int& max_x, int& max_y);

	/**
	@brief			기존 tracker 정보만 이용하여 추적 수행
	새 타겟의 위치는 상위 N개 파티클의 평균 크기와 위치로 추정
	@param			dib1 : Gray Image
	dib2 : Color Image
	*/
	void processTracking(CDib& dib, CDib dib2);

	/**
	@brief			Detection과 matching 된 결과에 따라 tracker의 state를 업데이트 함
	매칭된 경우 detection 정보를 이용하여 업데이트
	매칭되지 않은 경우 이전 tracker 정보를 이용하여 업데이트
	@param			cx, cy : 매칭된 detection 윈도우의 중심 좌표
	min_x, min_y, max_x, max_y : 매칭된 detection 윈도우의 좌표
	*/
	void processUpdatingTrackerStates(CDib& dib, CDib& dib2, int& cx, int& cy, int& min_x, int& min_y, int& max_x, int& max_y);

	/**
	@brief			Tracker의 state, 다른 tracker와 거리, tracker의 확률값에 따라 분류기의 업데이트 결정
	@param			dib1 : Gray Image
	dib2 : HSI Image
	flag : 새로 할당된 tracker -> true, 기존에 할당된 tracker -> false
	otherTarget : 다른 tracker의 위치 정보
	*/
	void processUpdatingClassifiers(CDib& dib, CDib& dib2, bool flag, OTHERTARGETLOCATION& otherTarget, int currentFrame, COpticalFlow& opticalFlow);

	//////// 상위 4개 //////////

	/*
	@brief			Particle의 OCS-LBP, Color 값이 Template과 얼마나 비슷한지 측정
	*/
	void Compare_Distance();

	/**
	@brief			detection 윈도우와 tracker 간의 거리 측정
	연관성 검사 알고리즘에 사용됨
	@param			cx, cy : 매칭할 detection 윈도우의 중심 좌표
	@return			detection 윈도우와 tracker 간의 거리 반환
	*/
	double measureDistanceBetweenDetectionAndTracking(int& cx, int& cy);

	/**
	@brief			tracker의 분류기를 이용하여 detection 윈도우의 확률값 추정
	연관성 검사 알고리즘에 사용됨
	@param			dib1 : Gray Image
	dib2 : HSI Image
	cx, cy : 매칭할 detection 윈도우의 중심 좌표
	min_x, min_y, max_x, max_y : 매칭할 detection 윈도우의 좌표
	@return			detection 윈도우의 추정된 확률값
	*/
	double getFeature_n_measureProbabilityDetectionRegion(CDib& dib, int& cx, int& cy, int& min_x, int& min_y, int& max_x, int& max_y);

	double getOverlapRatioBetweenDetectionsAndTrackers(int& cx, int& cy, int& min_x, int& min_y, int& max_x, int& max_y);

	/**
	@brief			tracker 윈도우의 width를 가져옴
	연관성 검사 알고리즘에 사용됨
	@return			tracker 윈도우의 width
	*/
	double getWidthOfTrackingWindow();

	/**
	@brief			tracker 윈도우의 height를 가져옴
	연관성 검사 알고리즘에 사용됨
	@return			tracker 윈도우의 height
	*/
	double getHeightOfTrackingWindow();

	/**
	@brief			화면 출력을 위한 tracker의 id 설정
	@param			index : tracker의 id
	*/
	void setDisplayTrackerID(int index);

	/**
	@brief			tracker의 index 설정
	@param			index : tracker의 index
	*/
	void setTrackerIndex(int index);

	/**
	@brief			프로그램 실행 시 분류기 생성
	tracker가 할당될 때마다 분류기를 생성할 경우 많은 시간이 소요됨
	*/
	void generateClassifier();

	/**
	@brief			생성자
	메모리 할당
	변수 초기화
	*/
	void valueInit();

	/**
	@brief			소멸자
	메모리 해제
	*/
	void valueDestroy();

	/**
	@brief			detection과 매칭 시 사용되는 tracker의 index 반환
	@return			tracker의 index
	*/
	int getTrackerIndex();

	/**
	@brief			tracker의 겹침 카운트 초기화
	tracker가 새로 생성 되었을 경우
	@date			2014-01-07
	*/
	void initOcclusionCount();

	/**
	@brief			tracker의 겹침 카운트 감소
	detection 윈도우와 매칭되었을 경우
	*/
	void minusOcclusionCount();


	/**
	@brief			tracker의 겹침 카운트 반환
	tracker의 소멸 유무를 판단할 때 호출
	@return			겹침 카운트
	*/
	int getOcclusionCount();


	/**
	@brief			tracker의 겹침 카운트 증가
	detection 윈도우와 매칭되지 않았을 경우
	*/
	void plusOcclusionCount();

	void plusNotDisplayCounnt();

	void minusNotDisplayCounnt();

	/**
	@brief			다른 tracker와 겹쳤을 경우, 어떤 tracker가 앞쪽에 있는지 판단하기 위한 함수
	3 frame 누적
	@date			2014-01-07
	*/
	void initYaxisInformation();
	void setYaxis();
	int getYaxis();
	void setTrackerCountForYaxis();
	void setAvgYaxis(int value);
	int getAvgYaxis();

	/**
	@brief			tracker의 윈도우 정보 반환
	다른 tracker의 정보를 알아내기 위하여 사용
	@param			cx, cy : tracker 윈도우의 중심 좌표 저장
	w, h : tracker 윈도우의 크기 저장
	*/
	void getInformationOfTarget(int& cx, int& cy, int& w, int& h, double& pro);

	/**
	@brief			tracker가 생성된 frame 반환
	화면 출력 유무 판단 시 사용
	@return			tracker가 생성된 frame 반환
	@date			2014-01-07
	*/
	int getGenerationFrame();

	/**
	@brief			tracker가 생성된 frame 설정
	화면 출력 유무 판단 시 사용
	@param			currentFrame : tracker가 생성된 frame
	*/
	void initGenerationFrame(int currentFrame);


	/**
	@brief			path prediction을 위한 tracker의 상태 초기화
	*/
	void initStateForPrediction();

	/**
	@brief			tracker의 윈도우를 화면에 출력
	DISPLAY_TYPE 설정에 따라 추가적으로 파티클 샘플링 분포 또는 파트 모양 출력
	@param			dib : 원본 영상
	@date			2014-01-07
	*/
	void trackingDrawRect(CDib& dib);

	/**
	@brief			tracker의 ID를 화면체 출력
	@param			pDC : 텍스트 출력을 위한 핸들러
	i : tracker ID
	*/
	void trackingDrawText(CDC *pDC, int& i);

	/**
	@brief			화면 출력을 위한 tracker의 index 반환
	@return			tracker의 index
	*/
	int getDisplayTrackerID();

	int getNotDisplayCounnt();

	void getVelocity(TARGETPARTICLE& particle1, COpticalFlow& opticalFlow, int velocity[2][2], int& velocity_index);

	/**
	@brief			detection 윈도우와 tracker 간에 매칭 유무 설정
	@param			flag : 매칭 되었으면 true, 매칭되지 않았으면 false
	*/
	void setMatchingFlag(bool flag);

	/**
	@brief			tracker가 생성된 frame 설정
	화면 출력 유무 판단 시 사용
	@param			currentFrame : tracker가 생성된 frame
	*/
	bool getMatchingflag();

	/*
	@brief			Tracker와 Detection 간 RGB cross - correlation
	return			RGB 각각의 correlation을 더하고 3으로 나눈 평균 값.
	*/
	void Compare_correlation(CDib& d_Dib, CDib t_Dib, int width, int height, double* d_mean, double* t_mean, double* d_std, double* t_std, double& result);
};