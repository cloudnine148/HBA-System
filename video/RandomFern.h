#pragma once

#include <vector>

#include "Dib.h"
#include "RGBBYTE.h"
#include "DibColor.h"
#include "common.h"


#include <crtdbg.h>		//150213 for memory leak detection




//#define FUCNTION_THRESHOLD		0.0273075	// (OCS-LBP Ư¡ �߰��� + Intensity Ư¡ �߰���) / 2
//#define FUCNTION_THRESHOLD		0.053383	// (OCS-LBP Ư¡ ��� + Intensity Ư¡ ���) / 2
//#define FUCNTION_THRESHOLD		0.054615	// OCS-LBP Ư¡ �߰���
//#define FUCNTION_THRESHOLD		0.000000	// Intensity Ư¡ �߰���

#define FUCNTION_THRESHOLD		0
#define NORMALIZE_CONSTANT		1

////////// typedef ����ü

// function�� ����Ǵ� ����ü
typedef struct FERN_FUNCTION_INFO
{
	// function�� ù ��° ��ǥ
	int *arrLoc_Bin_1;

	// function�� �� ��° ��ǥ
	int *arrLoc_Bin_2;

	FERN_FUNCTION_INFO()
	{
		arrLoc_Bin_1 = NULL;
		arrLoc_Bin_2 = NULL;
	}

}t_FernFunctionInfo;

// fern�� ����Ǵ� ����ü
typedef struct FERNINFO
{
	// �н��� ���� Ȯ�� ������ ����Ǵ� �迭
	// row : class
	// coloumn : fern size (pow( 2.0, RANGE_FERN_S ))
	double **arrHistoFern;

	FERNINFO()
	{
		arrHistoFern = NULL;
	}

}t_FernInfo;

// fern�� function�� ����Ǵ� ����ü
struct RANDOMFERNS
{
	// class �� 
	int theNumberOfClass;
	// fern ��
	int theNumberOfFern;
	// function ��
	int theNumberOfFunction;
	// ����ȭ ���
	double normalizeFactor;
	double normalizeConstant;

	// function
	t_FernFunctionInfo* function;

	// fern
	t_FernInfo* fern;

	// sum of fern distribution
	double** sumFernDistribution;

	// ������ fern flag
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
	// Random fern �н� �Լ�
	void generateRandomFern(RANDOMFERNS& rf_classifier, double*** feature, int theNumberOfDataPerClass[], 
		int totalFeatureSize, int featureDimension, int theNumberOfClass, int theNumberOfFern, int theNumberOfFunction, double normalizeFactor, double normalizeConstant, int historyLength, bool flag1, bool flag2);
	
	// Random fern �з� �Լ�
	void classification(RANDOMFERNS& rf_classifier, double* feature, double* result);

	// Random fern�� boost ��Ű�� �Լ�
	void boostedClassifier(RANDOMFERNS& rf_classifier, double*** feature, int theNumberOfDataPerClass[], 
		int totalFeatureSize, int featureDimension, int theNumberOfClass, int theNumberOfFern, int theNumberOfBoostedFern, int theNumberOfFunction, double normalizeFactor, double normalizeContant, int historyLength, bool flag1, bool flag2);

	// Random fern ����
	void RFCopyLeftToRight(RANDOMFERNS& rf1, RANDOMFERNS& rf2);
	void RFCopyLeftToRight2(RANDOMFERNS& rf1, RANDOMFERNS& rf2);

	// boosted ���� �� ����
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
	// random �ϰ� �ȼ� ��ġ�� ocslbp bin �� ���ϱ�
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

	// ������ fern �ߺ� ���� ������ ���� swap
	void swap(RANDOMFERNS* temp1, int optimalIndex, int changeIndex, int theNumberOfClass, int theNumberOfFunction);
	// sample weight�� ����ϱ� ���� �Լ�
	void calculateSampleWeight(RANDOMFERNS& rf_classifier, double* feature, double* result, int theNumberOfFunction, int fernIndex, double normalizeFactor, double normalizeContant);
	// optimal function�� �̿��Ͽ� ���� �����Ϳ� ���� fern index�� ����
	int getIndexUsingOptimalFunction(t_FernFunctionInfo* rf_function, double* feature/*, double* fernValue*/, int theNumberOfFunction, int fernIndex);
	// optimal fern�� �̿��� Ȯ���� ���
	void getProbabilityUsingOptimalFern(t_FernInfo* rf_fern/*, double* feature*/, double* result, int fernIndex, int value, int theNumberOfClass, double normalizeFactor, double normalizeContant);	

	

protected:
	// boosted random fern ���� �� ���
	// fern�� function�� ������ �ӽ� fern�� function ����
	RANDOMFERNS temp;
	// ������ fern�� �ߺ� ������ �����ϱ� ���� swap�� ���Ǵ� ����
//	RANDOMFERNS temp2;
};