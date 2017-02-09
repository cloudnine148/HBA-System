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

	// Random Forests Ȯ����
	double probability;

	// ���� Ÿ�ٰ� ��ƼŬ ���� �Ÿ�
	double distance;

	// �Ÿ��� Ȯ������ ����ġ ��
	double score;

	// OCS-LBP Ư¡ ������׷�
	double OCSLBP_histogram[OCSLBP_FEATURE_SIZE];
	// OCS-LBP distance
	double OCSLBP_distance;

	// LID Ư¡ ������׷�
	double LID_histogram[LID_FEATURE_SIZE];

	// Color Ư¡ ������׷�
	double Color_histogram[COLOR_FEATURE_SIZE_FOR_TRACKING];
	// Color distance
	double Color_distance;

	double new_histogram[NEW_FEATURE_SIZE];

	// part ��ġ
	PARTLOCATION PART_location[SUB_BLOCK];
	// 0 : head
	// 1 : left arm
	// 2 : right arm
	// 3 : left leg
	// 4 : right leg

	// part �� Ȯ����
	double OCSLBP_part_probability;

	int occlusionType;
	// 1 : ���� ����
	// 2 : �κ� ��ħ
	// 3 : ���� ��ħ
	// 4 : ��ü ��ħ
};

struct TARGETPARTICLE
{
	int x;
	int y;
	int w;
	int h;
	double probability;

	// part ��ġ
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

	// ���� Y��ǥ�� ���� ����
	// tracker ������ ��� 0���� �ʱ�ȭ
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
	// ���� detection�� ��Ī�� id
	int trackerIndex;
	// ȭ�� ǥ�ÿ� id
	int displayTrackerID;
	//Framecount
	int m_Frame_Count_posi;
	int m_Frame_Count_nega;

protected:
	PARTICLEINFO* currentParticles;
	PARTICLEINFO* preParticles;

	TARGETPARTICLE preTargetParticle;
	TARGETPARTICLE estTargetParticle;

	//�з���
	CRandom_Fern rf;
	RANDOMFERNS rf_ocslbp;
	RANDOMFERNS rf_color;

	// ����� �н� �����͸� Ŭ���� ���� �з��Ͽ� BRFs�� �н� ��ų �� ����ϴ� �迭
	// [Ŭ���� ��][������ ��][Ư¡ ���� ��]
	double*** trainDataSet_ocslbp;
	double*** trainDataSet_color;

	//	���� OCS-LBP Ư¡ ������׷�
	double Template_OCSLBP_histogram[OCSLBP_FEATURE_SIZE];
	//	����	Color Ư¡ ������׷�
	double Template_Color_histogram[COLOR_FEATURE_SIZE_FOR_TRACKING];

	// ���� tracker�� detection�� ��Ī���� ���� ��
	int occlusionCount;

	// tracker�� ���� ��迡 ���� ���
	// ȭ�鿡�� �� ��Ÿ���� �ϴ� ��
	int notDisplayCount;

	bool matchingFlag;

	// �ش� tracker�� ������ ������
	int frame;
	// update�� ����� ������
	int updateFrame;

	// ������ ũ�� �� ��ħ �Ÿ� �Ӱ谪�� ����Ǵ� ����
	double distBetweenObjectsThreshold;

	// ���ӵ� ���� ����
	int velocity_index;
	int velocity[2][2];

	// tracker �ӵ� ����� ���� ���� frame ��ġ ����
	int previousX;
	int previousY;

	// ���� tracker�� ���� tracker ���� �Ÿ�
	double distanceBetweenPreAndCurr;

	// ������ ���� tracker�� �ٸ� tracker�� �Ÿ� �� ����
	// ª�� �Ÿ��� ����Ǵ� ����
	double distanceBetweenObjects;
	double ratioBetweenObjects;

	// ��ħ ������ ���� ����
	int targetYaxis[3];
	int trackerCount;
	int avgYaxis;

	int size_level;


protected:
	/**
	@brief			Detection ������ �̿��Ͽ� tracker �������� �ʱ� ��ġ �� ũ�� ����
	@param			particle : ���� tracker�� ���� ����
	particle2 : ���� tracker�� ��ƼŬ ���� ����
	cx, cy : ��Ī�� detection �������� �߽� ��ǥ
	min_x, min_y, max_x, max_y : ��Ī�� detection �������� ��ǥ
	*/
	void getTargetLocationFromDetection(TARGETPARTICLE& particle, PARTICLEINFO* particle2, int& cx, int& cy, int& min_x, int& min_y, int& max_x, int& max_y);

	/**
	@brief			��ƼŬ ���ø�
	��ü �������� ũ�⿡ ���� ���ø� ������ �ٸ�
	@param			flag : ���� �Ҵ�� tracker -> true, ������ �Ҵ�� tracker -> false
	xPoint, yPoint : Ÿ�� �Ǵ� ��Ī�� detection �������� �߽� ��ǥ
	particle1 : ���� tracker�� ��ƼŬ ���� ����
	distanceBetweenTargets : ���� tracker�� �ٸ� tracker ���� �Ÿ�
	threshold : tracker ���� ��ħ �߻� �Ÿ� �Ӱ谪
	*/
	void createParticle(bool flag, const int& xPoint, const int& yPoint, PARTICLEINFO* particle1, int distanceBetweenTargets, double& threshold, int velocity[2][2]);

	/**
	@brief			����þ� ������ �̿��Ͽ� ��ƼŬ ���ø�
	@param			flag : ���� �Ҵ�� tracker -> true, ������ �Ҵ�� tracker -> false
	xPoint, yPoint : Ÿ�� �Ǵ� ��Ī�� detection �������� �߽� ��ǥ
	particle1 : ���� tracker�� ��ƼŬ ���� ����
	std : ��ƼŬ�� ���ø� �Ǵ� ���� (�л�)
	*/
	void createParticlesForObject(bool flag, const int& xPoint, const int& yPoint, PARTICLEINFO* particle1, int std, int particleDistributionFlag);

	/**
	@brief			����þ� ���� ����
	��ƼŬ�� ����þ� ������ ���ø�
	@param			mean : ���
	std : �л�
	@return			����þ� ������ ������ ��
	*/
	double GaussianRand(const double& mean, const double& std);

	/**
	@brief			���ø��� ��ƼŬ ��ġ�� ���ӵ��� ������
	@param			particle1 : ���� tracker�� ��ƼŬ ���� ����
	velocity : t, t-1 �������� ���ӵ� ���� ����
	*/
	void createParticleWithVelocity(PARTICLEINFO* particle1, int velocity[2][2]);

	/**
	@brief			Random ferns �н�
	@param			dib1 : Gray Image
	dib2 : HSI Image
	particle1 : ���� tracker�� ��ƼŬ ���� ����
	df_ocslbp : OCS-LBP Ư¡�� ����ϴ� �з���
	df_color : Hue Ư¡�� ����ϴ� �з���
	@return			�Էµ� Ư¡ ���Ϳ� ���� ������ Ȯ����
	*/
	void RFtraining(CDib& dib, CDib& dib2, PARTICLEINFO* particle1, RANDOMFERNS& rf1, RANDOMFERNS& rf2, bool flag1, bool flag2);

	void trainingDataExtractionForTraining(CDib& dib, CDib& dib2, PARTICLEINFO& particle1, double** feature1, double** feature2, int& index, int flag);
	
	void getTwoFeatures(CDib& dib, CDib& dib2, PARTICLEINFO& particle);

	void L2Normalization(double* histogram, int histBins);

	/**
	@brief			Ư¡ ���� (OCS-LBP, Hue)
	��� ��ƼŬ ���
	@param			dib1 : Gray Image
	dib2 : HSI Image
	particle1 : ���� tracker�� ��ƼŬ ���� ����
	*/
	void extractFeature(CDib& dib, CDib& dib2, PARTICLEINFO* particle1);

	void extractFeatureAtPartForTwoFeatures(CDib& dib, CDib& dib2, PARTICLEINFO& particle/*, double& firstProbability, double& secondProbability*/);

	/**
	@brief			Random forest�� �̿��� Ư¡�� Ȯ�� ����
	@param			feature : Ư¡ ����
	df : �з���
	featureSize : Ư¡ ũ��
	thenumberofclass : Ŭ���� ��
	@return			�Էµ� Ư¡ ���Ϳ� ���� ������ Ȯ����
	*/
	double RFtesting(double* input, RANDOMFERNS& rf_classifier);

	/**
	@brief			OCS-LBP�� Color�� Template �迭�� ����� �ش�
	*/
	void CreateTemplateValue(CDib& dib, CDib& dib2, int& cx, int& cy, int& min_x, int& min_y, int& max_x, int& max_y);

	/**
	@brief			score ���� ������� ��ƼŬ�� ������������ ����
	@param			particle1 : ���� tracker�� ��ƼŬ ���� ����
	particle2 : ���� �Ǵ� ������ tracker ���� ����
	n : ��ƼŬ ��
	*/
	void sortParticles(PARTICLEINFO* particle1, TARGETPARTICLE& particle2, const int& n, int velocity[2][2]);

	/**
	@brief			���� ����
	@param			particle1 : ���� tracker�� ��ƼŬ ���� ����
	n : ��ƼŬ ��
	*/
	void bubbleSort(PARTICLEINFO* particle1, const int& n);

	/**
	@brief			���� N���� ��ƼŬ�� �̿��Ͽ� ���ο� Ÿ���� ���� ����
	������ �˻� �˰��� ���� ���� �����
	@param			particle1 : ������ tracker ���� ����
	particle2 : ���� tracker�� ��ƼŬ ���� ����
	*/
	void estimateNewTargetStateUsingUpperParticles(TARGETPARTICLE& particle1, PARTICLEINFO* particle2);

	/**
	@brief			��Ī �� detection�� ���� ��� ���� tracker ������ �̿��Ͽ� Ÿ�� state ������Ʈ
	@param			particle1 : ������ tracker ���� ����
	particle2 : ���� tracker ���� ����
	@return			������Ʈ �� Ÿ�ٰ� ���� tracker Ÿ�� ���� �Ÿ�
	*/
	double estimateNewTargetStateUsingPreTrackerInformation(TARGETPARTICLE& particle1, TARGETPARTICLE& particle2);

	/**
	@brief			���ο� Ÿ���� Ȯ������ �����ϰ� ��ƼŬ ����ü�� �����ϴ� �Լ�
	���� ��ƼŬ ����ü�� ������ �ڷ� �� ĭ�� �̵���
	@param			dib1 : Gray Image
	dib2 : HSI Image
	particle1 : ������ tracker ���� ����
	particle2 : ���� tracker�� ��ƼŬ ���� ����
	*/
	void moveParticleToNextIndex(CDib& dib, CDib& dib2, TARGETPARTICLE& particle1, PARTICLEINFO* particle2);

	/**
	@brief			���� Ÿ���� Ư¡ ���� �� Ȯ�� ����
	@param			dib1 : Gray Image
	dib2 : HSI Image
	particle1 : ���� tracker�� ��ƼŬ ���� ���� (1��)
	*/
	void extractTargetFeature(CDib& dib, CDib& dib2, PARTICLEINFO& particle1);

	/**
	@brief			���� tracker�� �ٸ� tracker ���� �Ÿ� ���
	@param			otherTarget : �ٸ� tracker�� ��ġ ���� ����
	@return			���� �Ÿ� �� ���� ª�� �Ÿ� ��ȯ
	*/
	int measureDistanceBetweenOtherTargets(TARGETPARTICLE& particle, OTHERTARGETLOCATION& otherTarget, double& ratioBetweenObjects);

	void setPreviousTrackerLocation(int px, int py);
	void getPreviousTrackerLocation(int& px, int& py);

	/**
	@brief			��Ī �� detection ������ ������ �̿��Ͽ� Ÿ���� ���� ������Ʈ
	������ �˻� �˰��� ���� �Ŀ� �����
	@param			particle1 : ������ tracker ���� ����
	particle2 : ���� tracker ���� ����
	detectionCenterX, detectionCenterY : ��Ī�� detection �������� �߽� ��ǥ
	detectionW, detectionH : ��Ī�� detection �������� ũ��
	@return			������Ʈ �� Ÿ�ٰ� ���� tracker Ÿ�� ���� �Ÿ�
	*/
	double estimateNewTargetStateUsingDetectionInformation(TARGETPARTICLE& particle1, TARGETPARTICLE& particle2, int& detectionCenterX, int& detectionCenterY, int& detectionW, int& detectionH);

public:
	void initNotDisplayCounnt();

	/**
	@brief			Detection�� ������ �̿��Ͽ� ���ο� tracker ����
	@param			dib1 : Gray Image
	dib2 : HSI Image
	flag : ���� �Ҵ�� tracker -> true, ������ �Ҵ�� tracker -> false
	cx, cy : ��Ī�� detection �������� �߽� ��ǥ
	min_x, min_y, max_x, max_y : ��Ī�� detection �������� ��ǥ
	*/
	void processGenerationTracker(CDib& dib, CDib& dib2, bool flag, int& cx, int& cy, int& min_x, int& min_y, int& max_x, int& max_y);

	/**
	@brief			���� tracker ������ �̿��Ͽ� ���� ����
	�� Ÿ���� ��ġ�� ���� N�� ��ƼŬ�� ��� ũ��� ��ġ�� ����
	@param			dib1 : Gray Image
	dib2 : Color Image
	*/
	void processTracking(CDib& dib, CDib dib2);

	/**
	@brief			Detection�� matching �� ����� ���� tracker�� state�� ������Ʈ ��
	��Ī�� ��� detection ������ �̿��Ͽ� ������Ʈ
	��Ī���� ���� ��� ���� tracker ������ �̿��Ͽ� ������Ʈ
	@param			cx, cy : ��Ī�� detection �������� �߽� ��ǥ
	min_x, min_y, max_x, max_y : ��Ī�� detection �������� ��ǥ
	*/
	void processUpdatingTrackerStates(CDib& dib, CDib& dib2, int& cx, int& cy, int& min_x, int& min_y, int& max_x, int& max_y);

	/**
	@brief			Tracker�� state, �ٸ� tracker�� �Ÿ�, tracker�� Ȯ������ ���� �з����� ������Ʈ ����
	@param			dib1 : Gray Image
	dib2 : HSI Image
	flag : ���� �Ҵ�� tracker -> true, ������ �Ҵ�� tracker -> false
	otherTarget : �ٸ� tracker�� ��ġ ����
	*/
	void processUpdatingClassifiers(CDib& dib, CDib& dib2, bool flag, OTHERTARGETLOCATION& otherTarget, int currentFrame, COpticalFlow& opticalFlow);

	//////// ���� 4�� //////////

	/*
	@brief			Particle�� OCS-LBP, Color ���� Template�� �󸶳� ������� ����
	*/
	void Compare_Distance();

	/**
	@brief			detection ������� tracker ���� �Ÿ� ����
	������ �˻� �˰��� ����
	@param			cx, cy : ��Ī�� detection �������� �߽� ��ǥ
	@return			detection ������� tracker ���� �Ÿ� ��ȯ
	*/
	double measureDistanceBetweenDetectionAndTracking(int& cx, int& cy);

	/**
	@brief			tracker�� �з��⸦ �̿��Ͽ� detection �������� Ȯ���� ����
	������ �˻� �˰��� ����
	@param			dib1 : Gray Image
	dib2 : HSI Image
	cx, cy : ��Ī�� detection �������� �߽� ��ǥ
	min_x, min_y, max_x, max_y : ��Ī�� detection �������� ��ǥ
	@return			detection �������� ������ Ȯ����
	*/
	double getFeature_n_measureProbabilityDetectionRegion(CDib& dib, int& cx, int& cy, int& min_x, int& min_y, int& max_x, int& max_y);

	double getOverlapRatioBetweenDetectionsAndTrackers(int& cx, int& cy, int& min_x, int& min_y, int& max_x, int& max_y);

	/**
	@brief			tracker �������� width�� ������
	������ �˻� �˰��� ����
	@return			tracker �������� width
	*/
	double getWidthOfTrackingWindow();

	/**
	@brief			tracker �������� height�� ������
	������ �˻� �˰��� ����
	@return			tracker �������� height
	*/
	double getHeightOfTrackingWindow();

	/**
	@brief			ȭ�� ����� ���� tracker�� id ����
	@param			index : tracker�� id
	*/
	void setDisplayTrackerID(int index);

	/**
	@brief			tracker�� index ����
	@param			index : tracker�� index
	*/
	void setTrackerIndex(int index);

	/**
	@brief			���α׷� ���� �� �з��� ����
	tracker�� �Ҵ�� ������ �з��⸦ ������ ��� ���� �ð��� �ҿ��
	*/
	void generateClassifier();

	/**
	@brief			������
	�޸� �Ҵ�
	���� �ʱ�ȭ
	*/
	void valueInit();

	/**
	@brief			�Ҹ���
	�޸� ����
	*/
	void valueDestroy();

	/**
	@brief			detection�� ��Ī �� ���Ǵ� tracker�� index ��ȯ
	@return			tracker�� index
	*/
	int getTrackerIndex();

	/**
	@brief			tracker�� ��ħ ī��Ʈ �ʱ�ȭ
	tracker�� ���� ���� �Ǿ��� ���
	@date			2014-01-07
	*/
	void initOcclusionCount();

	/**
	@brief			tracker�� ��ħ ī��Ʈ ����
	detection ������� ��Ī�Ǿ��� ���
	*/
	void minusOcclusionCount();


	/**
	@brief			tracker�� ��ħ ī��Ʈ ��ȯ
	tracker�� �Ҹ� ������ �Ǵ��� �� ȣ��
	@return			��ħ ī��Ʈ
	*/
	int getOcclusionCount();


	/**
	@brief			tracker�� ��ħ ī��Ʈ ����
	detection ������� ��Ī���� �ʾ��� ���
	*/
	void plusOcclusionCount();

	void plusNotDisplayCounnt();

	void minusNotDisplayCounnt();

	/**
	@brief			�ٸ� tracker�� ������ ���, � tracker�� ���ʿ� �ִ��� �Ǵ��ϱ� ���� �Լ�
	3 frame ����
	@date			2014-01-07
	*/
	void initYaxisInformation();
	void setYaxis();
	int getYaxis();
	void setTrackerCountForYaxis();
	void setAvgYaxis(int value);
	int getAvgYaxis();

	/**
	@brief			tracker�� ������ ���� ��ȯ
	�ٸ� tracker�� ������ �˾Ƴ��� ���Ͽ� ���
	@param			cx, cy : tracker �������� �߽� ��ǥ ����
	w, h : tracker �������� ũ�� ����
	*/
	void getInformationOfTarget(int& cx, int& cy, int& w, int& h, double& pro);

	/**
	@brief			tracker�� ������ frame ��ȯ
	ȭ�� ��� ���� �Ǵ� �� ���
	@return			tracker�� ������ frame ��ȯ
	@date			2014-01-07
	*/
	int getGenerationFrame();

	/**
	@brief			tracker�� ������ frame ����
	ȭ�� ��� ���� �Ǵ� �� ���
	@param			currentFrame : tracker�� ������ frame
	*/
	void initGenerationFrame(int currentFrame);


	/**
	@brief			path prediction�� ���� tracker�� ���� �ʱ�ȭ
	*/
	void initStateForPrediction();

	/**
	@brief			tracker�� �����츦 ȭ�鿡 ���
	DISPLAY_TYPE ������ ���� �߰������� ��ƼŬ ���ø� ���� �Ǵ� ��Ʈ ��� ���
	@param			dib : ���� ����
	@date			2014-01-07
	*/
	void trackingDrawRect(CDib& dib);

	/**
	@brief			tracker�� ID�� ȭ��ü ���
	@param			pDC : �ؽ�Ʈ ����� ���� �ڵ鷯
	i : tracker ID
	*/
	void trackingDrawText(CDC *pDC, int& i);

	/**
	@brief			ȭ�� ����� ���� tracker�� index ��ȯ
	@return			tracker�� index
	*/
	int getDisplayTrackerID();

	int getNotDisplayCounnt();

	void getVelocity(TARGETPARTICLE& particle1, COpticalFlow& opticalFlow, int velocity[2][2], int& velocity_index);

	/**
	@brief			detection ������� tracker ���� ��Ī ���� ����
	@param			flag : ��Ī �Ǿ����� true, ��Ī���� �ʾ����� false
	*/
	void setMatchingFlag(bool flag);

	/**
	@brief			tracker�� ������ frame ����
	ȭ�� ��� ���� �Ǵ� �� ���
	@param			currentFrame : tracker�� ������ frame
	*/
	bool getMatchingflag();

	/*
	@brief			Tracker�� Detection �� RGB cross - correlation
	return			RGB ������ correlation�� ���ϰ� 3���� ���� ��� ��.
	*/
	void Compare_correlation(CDib& d_Dib, CDib t_Dib, int width, int height, double* d_mean, double* t_mean, double* d_std, double* t_std, double& result);
};