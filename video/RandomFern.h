#pragma once

#include <vector>

#include "Dib.h"
#include "RGBBYTE.h"
#include "DibColor.h"
#include "common.h"


#include <crtdbg.h>		//150213 for memory leak detection




//#define FUCNTION_THRESHOLD		0.0273075	// (OCS-LBP 특징 중간값 + Intensity 특징 중간값) / 2
//#define FUCNTION_THRESHOLD		0.053383	// (OCS-LBP 특징 평균 + Intensity 특징 평균) / 2
//#define FUCNTION_THRESHOLD		0.054615	// OCS-LBP 특징 중간값
//#define FUCNTION_THRESHOLD		0.000000	// Intensity 특징 중간값

#define FUCNTION_THRESHOLD		0
#define NORMALIZE_CONSTANT		1

////////// typedef 구조체

// function이 저장되는 구조체
typedef struct FERN_FUNCTION_INFO
{
	// function의 첫 번째 좌표
	int *arrLoc_Bin_1;

	// function의 두 번째 좌표
	int *arrLoc_Bin_2;

	FERN_FUNCTION_INFO()
	{
		arrLoc_Bin_1 = NULL;
		arrLoc_Bin_2 = NULL;
	}

}t_FernFunctionInfo;

// fern이 저장되는 구조체
typedef struct FERNINFO
{
	// 학습된 사후 확률 분포가 저장되는 배열
	// row : class
	// coloumn : fern size (pow( 2.0, RANGE_FERN_S ))
	double **arrHistoFern;

	FERNINFO()
	{
		arrHistoFern = NULL;
	}

}t_FernInfo;

// fern과 function이 저장되는 구조체
struct RANDOMFERNS
{
	// class 수 
	int theNumberOfClass;
	// fern 수
	int theNumberOfFern;
	// function 수
	int theNumberOfFunction;
	// 정규화 상수
	double normalizeFactor;
	double normalizeConstant;

	// function
	t_FernFunctionInfo* function;

	// fern
	t_FernInfo* fern;

	// sum of fern distribution
	double** sumFernDistribution;

	// 최적의 fern flag
	int* selectedBoostedFerns;

	// history
//	int currentFrame;
//	int historyLength;
//	t_FernInfo** historyFern;

	RANDOMFERNS()
	{
		theNumberOfClass = 0;
		theNumberOfFern = 0;
		theNumberOfFunction = 0;
		normalizeFactor = 0.0;
		normalizeConstant = 0.0;		

		function = NULL;
		fern = NULL;
		sumFernDistribution = NULL;
		selectedBoostedFerns = NULL;
		

	//	currentFrame = 0;
	//	historyLength = 0;
	//	historyFern = NULL;
	}
};

class CRandom_Fern
{
public:
	CRandom_Fern(void);
	~CRandom_Fern(void);


public:
	// Random fern 학습 함수
	void generateRandomFern(RANDOMFERNS& rf_classifier, double*** feature, int theNumberOfDataPerClass[], 
		int totalFeatureSize, int featureDimension, int theNumberOfClass, int theNumberOfFern, int theNumberOfFunction, double normalizeFactor, double normalizeConstant, int historyLength, bool flag1, bool flag2);
	
	// Random fern 분류 함수
	void classification(RANDOMFERNS& rf_classifier, double* feature, double* result);

	// Random fern을 boost 시키는 함수
	void boostedClassifier(RANDOMFERNS& rf_classifier, double*** feature, int theNumberOfDataPerClass[], 
		int totalFeatureSize, int featureDimension, int theNumberOfClass, int theNumberOfFern, int theNumberOfBoostedFern, int theNumberOfFunction, double normalizeFactor, double normalizeContant, int historyLength, bool flag1, bool flag2);

	// Random fern 복사
	void RFCopyLeftToRight(RANDOMFERNS& rf1, RANDOMFERNS& rf2);
	void RFCopyLeftToRight2(RANDOMFERNS& rf1, RANDOMFERNS& rf2);

	// boosted 랜덤 펀 생성
	void generateClassifier(RANDOMFERNS& rf_classifier, int theNumberOfClass, int theNumberOfFern, int theNumberOfBoostedFern, int theNumberOfFunction, double normalizeFactor, double normalizeContant, int historyLength);
	void generateClassifier(int theNumberOfClass, int theNumberOfFern, int theNumberOfBoostedFern, int theNumberOfFunction, double normalizeFactor, double normalizeContant, int historyLength);

	void initClasifier(RANDOMFERNS& rf_classifier);
	void destroyClassifier(RANDOMFERNS& rf_classifier);

	void valueInit();
	void valueDestroy();


protected:
	// build fern for traning
	void buildFernForTraining(RANDOMFERNS& rf_classifier, double*** feature, int theNumberOfDataPerClass[], 
		int totalFeatureSize, int featureDimension, int theNumberOfClass, int theNumberOfFern, int theNumberOfFunction, double normalizeFactor, double normalizeConstant, int historyLength, bool flag1, bool flag2);
	// random 하게 픽셀 위치와 ocslbp bin 값 정하기
	void decideRandomVariable(int& loc_Bin_1, int& loc_Bin_2, int totalFeatureSize, int featureDimension);
	// Binary code To Decimal
	//int convertBinaryToDecimal(int* arrBinary, int theNumberOfFunction);
	// build fern for classfication
	void buildFernForClassification(RANDOMFERNS& rf_classifier, double* feature, double* result);
	// classfy feature using Random fern
	void classifyUsingRandomFern(t_FernInfo* rf_fern, double** sumFernDistribution, int fernValue[], double* result, int theNumberOfClass, int theNumberOfFern, int theNumberOfFunction);

	// boosted algorithm
	void selectGoodFerns(RANDOMFERNS& rf_classifier, double*** feature, int theNumberOfDataPerClass[], 
		int totalFeatureSize, int featureDimension, int theNumberOfClass, int theNumberOfFern, int theNumberOfBoosedFern, int theNumberOfFunction, double normalizeFactor, double normalizeContant, int historyLength, bool flag1, bool flag2);

	// 최적의 fern 중복 선택 방지를 위한 swap
	void swap(RANDOMFERNS* temp1, int optimalIndex, int changeIndex, int theNumberOfClass, int theNumberOfFunction);
	// sample weight를 계산하기 위한 함수
	void calculateSampleWeight(RANDOMFERNS& rf_classifier, double* feature, double* result, int theNumberOfFunction, int fernIndex, double normalizeFactor, double normalizeContant);
	// optimal function을 이용하여 샘플 데이터에 대한 fern index를 추정
	int getIndexUsingOptimalFunction(t_FernFunctionInfo* rf_function, double* feature/*, double* fernValue*/, int theNumberOfFunction, int fernIndex);
	// optimal fern을 이용한 확률값 계산
	void getProbabilityUsingOptimalFern(t_FernInfo* rf_fern/*, double* feature*/, double* result, int fernIndex, int value, int theNumberOfClass, double normalizeFactor, double normalizeContant);	

	

protected:
	// boosted random fern 생성 시 사용
	// fern과 function을 저장할 임시 fern과 function 생성
	RANDOMFERNS temp;
	// 최적의 fern의 중복 선택을 방지하기 위해 swap에 사용되는 변수
//	RANDOMFERNS temp2;
};