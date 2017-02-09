#pragma once

#ifndef _dforest_h
#define _dforest_h

#include <math.h>
#include "common.h"

#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define MIN(a, b) (((a) > (b)) ? (b) : (a))

// 2016-09-05 JMR
#define RF_REGRESSION_TREE			0			
#define RF_CLASSIFICATION_TREE		1		// lead node에 class정보 가짐
#define RF_CLASSIFICATION_TREE_PROB	2		// lead node에 class별 확률값을 가짐

struct RANDOMFORESTS
{
    int featureDemension;
    int theNumberOfClass;
    int theNumberOfTree;
    int bufsize;
    double* trees;
	
	// tree weight 
	double* treesWeight;

	// tree 사용 여부 설정 배열
	// 0 : 사용 불능
	// 1 : 사용 가능
	//int* treesFlag;

	RANDOMFORESTS()
	{
		featureDemension = 0;
		theNumberOfClass = 0;
		theNumberOfTree = 0;
		bufsize = 0;
		trees = NULL;

		treesWeight = NULL;
		//treesFlag = NULL;
	}
};

struct RF_INTERNAL_BUFFERS
{
    double* treebuf;
    int* idxbuf;
    double* tmpbufr;
    double* tmpbufr2;
#if gRFtype == RF_REGRESSION_TREE
//#ifdef POINT_REGRESSION_TREE	//jwgim 150825
	double* tmpbufr3;
//#endif
#endif
    int* tmpbufi;			
	double* tmpbufw;
    int* classibuf;
	double* classwbuf;
    int* varpool;
    bool* evsbin;
    double* evssplits;
};

// For Ransac
struct sPoint {
	double x, y;
};

struct sLine {
	double mx, my;
	double sx, sy;
};


/**
	@brief			랜덤 포레스트 생성 메인 함수
	@param			trainingData : 학습 데이터
					theNumberOfTrainingData : 학습 데이터 수
					featureDemension : 특징 차원 수
					theNumberOfClass : 클래스 수
					theNumberOfTree : 트리 수
					ratioOfTraininData : 학습 데이터 중 트리 생성에 사용할 데이터 비율
					theNumberOfGoodTree : 최종 사용할 트리 수
					info : 반환 코드
						-2 : 지정한 클래스 수 범위를 벗어난 경우
						-1 : 파라매터 값이 잘못된 경우
						 1 : 정상적으로 트리 생성
					df : 트리 정보를 저장하는 구조체
					offsetTh : Regression RF를 위해 Accuracy 측정을 위함: Out of Bag데이터로 각 트리의  추출된 offset과 GT offset과의 차이에 대한 임계값
*/
void generateRandomForests(double** trainingData,
     int theNumberOfTrainingData,
     int featureDemension,
     int theNumberOfClass,
     int theNumberOfTree,
     double ratioOfTrainingData,
	 int theNumberOfGoodTree,
     int& info,
     RANDOMFORESTS& df,
	 int offsetTh=10,
	 int ransacDistanceTh=10);

/**
	@brief			실제 트리를 생성하는 함수
	@param			trainingData : 학습 데이터
					theNumberOfTrainingData : 학습 데이터 수
					featureDemension : 특징 차원 수
					theNumberOfClass : 클래스 수
					theNumberOfTree : 트리 수
					samplesize : 학습 데이터수 * 서브셋 비율
					nfeatures : 특징 차원 수 * ratio
					theNumberOfGoodTree : 최종 사용할 트리 수
					flags : dfusestrongsplits+dfuseevs
					info : 반환 코드
						-2 : 지정한 클래스 수 범위를 벗어난 경우
						-1 : 파라매터 값이 잘못된 경우
						 1 : 정상적으로 트리 생성
					df : 트리 정보를 저장하는 구조체
					offsetTh : Regression RF를 위해 Accuracy 측정을 위함: Out of Bag데이터로 각 트리의  추출된 offset과 GT offset과의 차이에 대한 임계값
*/
void generateDecisionTrees(double** trainingData,
     int theNumberOfTrainingData,
     int featureDemension,
     int theNumberOfClass,
     int theNumberOfTree,
     int samplesize,
     int nfeatures,
	 int theNumberOfGoodTree,
     int flags,
     int& info,
     RANDOMFORESTS& df,
	 int offsetTh);


/**
	@brief			랜덤 포레스트를 이용하여 확률값 추정
	@param			df : 트리 정보를 저장하는 구조체
					feature : 특징 벡터
					bUsingProbInLeafNodeFlag : 리프노드의 확률값 정보 사용 유무
					result : 클래스 별 추정된 확률값 저장
*/
void estimateProbabilityUsingRandomForests(const RANDOMFORESTS& df,
     const double* feature,	
	 bool bUsingProbInLeafNodeFlag,
     double* result);

/**
	@brief			특징 벡터를 하나의 결정 트리에 적용하여 확률값 추정
	@param			df : 트리 정보를 저장하는 구조체
					offs : 1차원 배열에 저장되어 있는 트리의 시작 위치
					feature : 특징 벡터
					bUsingProbInLeafNodeFlag : 리프노드의 확률값 정보 사용 유무
					result : 클래스 별 추정된 확률값 저장
					treeIdx : 현재 tree index
*/
static void estimateProbabilityUsingDecisionTree(const RANDOMFORESTS& df,
     int offs,
     const double* feature,	
	 bool bUsingProbInLeafNodeFlag,
     double* result,
	 int treeIdx);


#ifdef RF_REGRESSION_TREE	//jwgim 150826

void estimateOffsetUsingRandomForests(const RANDOMFORESTS& df, 
	const double* feature, int* offset, double* var);

static void estimateOffsetUsingDecisionTree(const RANDOMFORESTS& df, 
	int offs, const double* feature, double* result, int treeIdx);

#endif
/**
	@brief			학습 데이터를 이용하여 하나의 트리를 성장시키는 함수
	@param			sampleData : 전체 학습 데이터 중 ratioOfSampleData 만큼 추출된 데이터
					theNumberOfSampleData : 샘플 데이터 수
					featureDemension : 특징 차원
					theNumberOfClass : 클래스 수
					nfeatures : 특징 차원 * ratio
					nvarsinpool : 특징 차원
					flags : dfusestrongsplits+dfuseevs
					bufs : 트리 생성에 필요한 값을 저장하는 구조체
*/
static void growDecisionTree(double** sampleData,
     int theNumberOfSampleData,
     int featureDemension,
     int theNumberOfClass,
     int nfeatures,
     int nvarsinpool,
     int flags,
     RF_INTERNAL_BUFFERS& bufs
	 );

/**
	@brief			재귀를 이용하여 하나의 트리를 성장시키는 함수
	@param			sampleData : 전체 학습 데이터 중 ratioOfSampleData 만큼 추출된 데이터
					theNumberOfSampleData : 샘플 데이터 수
					featureDemension : 특징 차원
					theNumberOfClass : 클래스 수
					nfeatures : 특징 차원 * ratio
					nvarsinpool : 특징 차원
					flags : dfusestrongsplits+dfuseevs
					numprocessed : 재귀 단계
					idx1, idx2 : 샘플 데이터 크기로 지정
								 numprocessed가 1일 때 idx1 = 0, idx2 = theNumberOfSampleData-1 설정
								 재귀가 진행될 수록 idx1은 증가, idx2는 감소함
					bufs : 트리 생성에 필요한 값을 저장하는 구조체
*/
static void growDecisionTreeUsingRecursive(double** sampleData,
     int theNumberOfSampleData,
     int featureDemension,
     int theNumberOfClass,
     int nfeatures,
     int nvarsinpool,
     int flags,
     int& numprocessed,
     int idx1,
     int idx2,
     RF_INTERNAL_BUFFERS& bufs
	 );


static void splitDataOnIndex(double* x,
     int* c,
	 double* weights,
     int* cntbuf,
	 double* wcntbuf,
     int n,
     int nc,
     int flags,
     int& info,
     double& threshold,
     double& e);


static void splitDataOnValue(double* x,
     double* y,
     int n,
     int flags,
     int& info,
     double& threshold,
     double& e);

#ifdef RF_REGRESSION_TREE	//jwgim 150825
static void splitDataOnValue(double* x,
     double* y,
	 double* z,
	 int n,
     int flags,
     int& info,
     double& threshold,
     double& e);
#endif

/**
	@brief			데이터 이동
	@param			vdst : 데이터가 이동하는 목적지
					stride_dst : 이동하는 단위
					vsrc : 데이터의 출발지
					stride_src : 출발하는 단위
					n : 이동해야 하는 수
*/
void moveData(double *vdst, int stride_dst, const double *vsrc,  int stride_src, int n);

/**
	@brief			데이터 곱하기
	@param			vdst : 데이터가 이동하는 목적지
					stride_dst : 이동하는 단위
					n : 이동해야 하는 수
					alpha : 각 데이터에 곱해지는 값
*/

/**
	@brief			Offset Regressor를 위하여 Leaf Node의 대표 offset 추출 함수
*/
void getOffsetValuesForRegressionLeafNode(double** sampleData, 
										  int featureDemension, 
										  int& numprocessed, 
										  int idx1, 
										  int idx2, 
										  RF_INTERNAL_BUFFERS& bufs, 
										  double& offset_x,			// result offset_x
										  double& offset_y			// result offset_y
										  );

void multiplyData(double *vdst,  int stride_dst, int n, double alpha);

void sortFastInteger(double* a, int* b, int n);
void sortFastReal(double* a, double* b, int n);



// =============== For Ransac ===================
// 2차원벡터 RANSAC
int ransac_line_fitting(sPoint *data, int no_data, sPoint *result_data, sLine &model, double distance_threshold, int no_inliers_threshold);
bool find_in_samples (sPoint *samples, int no_samples, sPoint *data);
void get_samples (sPoint *samples, int no_samples, sPoint *data, int no_data);
int compute_model_parameter(sPoint samples[], int no_samples, sLine &model);
double compute_distance(sLine &line, sPoint &x);
double model_verification (sPoint *inliers, int *no_inliers, sLine &estimated_model, sPoint *data, int no_data, double distance_threshold);
double model_verification2 (sLine &model, sPoint *data, int no_data, double distance_threshold);
// 1차원벡터 RANSAC
int ransac_point_fitting(int *data, int no_data, int *best_inliers, int &best_model, double distance_threshold, int no_inliers_threshold);
bool find_in_samples (int *samples, int no_samples, int data);
void get_samples (int *samples, int no_samples, int *data, int no_data);
int compute_model_parameter(int samples[], int no_samples, int &model);
int compute_distance(int &base_point, int &x);
double model_verification (int *inliers, int *no_inliers, int &estimated_model, int *data, int no_data, double distance_threshold);
double model_verification2 (int &model, int *data, int no_data, double distance_threshold);
//double get_distanceThForRansac(int* inliers, int no_inliers);
bool get_allsamevalue(int* data, int no_data);

#ifdef RF_REGRESSION_TREE	//jwgim 150825
void sortFastReal(double* a, double* b, double* c, int n);
#endif

static void selectGoodTrees(const RANDOMFORESTS& df, double** feature, int theNumberOfTrainingData, int theNumberOfGoodTree);
static void bubbleSort(const RANDOMFORESTS& df, double** data, const int& n);

bool measureQuiltyOfDecisionTree(const RANDOMFORESTS& df, double* treebuf, double** trainingData, double* dataWeight, int theNumberOfTrainingData, int theNumberOfClass, int treeIdx);
double measureQuiltyOfDecisionTree(const RANDOMFORESTS& df, double* treebuf, double** trainingData, int* is_InOfBagData, int theNumberOfTrainingData, int offsetTh);
void getWeightUsingTreeAccuracy(const RANDOMFORESTS& df, double* treeAccuracy, int treeSize);

#endif

