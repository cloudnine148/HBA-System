#pragma once

#ifndef _dforest_h
#define _dforest_h

#include <math.h>
#include "common.h"

#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define MIN(a, b) (((a) > (b)) ? (b) : (a))

// 2016-09-05 JMR
#define RF_REGRESSION_TREE			0			
#define RF_CLASSIFICATION_TREE		1		// lead node�� class���� ����
#define RF_CLASSIFICATION_TREE_PROB	2		// lead node�� class�� Ȯ������ ����

struct RANDOMFORESTS
{
    int featureDemension;
    int theNumberOfClass;
    int theNumberOfTree;
    int bufsize;
    double* trees;
	
	// tree weight 
	double* treesWeight;

	// tree ��� ���� ���� �迭
	// 0 : ��� �Ҵ�
	// 1 : ��� ����
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
	@brief			���� ������Ʈ ���� ���� �Լ�
	@param			trainingData : �н� ������
					theNumberOfTrainingData : �н� ������ ��
					featureDemension : Ư¡ ���� ��
					theNumberOfClass : Ŭ���� ��
					theNumberOfTree : Ʈ�� ��
					ratioOfTraininData : �н� ������ �� Ʈ�� ������ ����� ������ ����
					theNumberOfGoodTree : ���� ����� Ʈ�� ��
					info : ��ȯ �ڵ�
						-2 : ������ Ŭ���� �� ������ ��� ���
						-1 : �Ķ���� ���� �߸��� ���
						 1 : ���������� Ʈ�� ����
					df : Ʈ�� ������ �����ϴ� ����ü
					offsetTh : Regression RF�� ���� Accuracy ������ ����: Out of Bag�����ͷ� �� Ʈ����  ����� offset�� GT offset���� ���̿� ���� �Ӱ谪
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
	@brief			���� Ʈ���� �����ϴ� �Լ�
	@param			trainingData : �н� ������
					theNumberOfTrainingData : �н� ������ ��
					featureDemension : Ư¡ ���� ��
					theNumberOfClass : Ŭ���� ��
					theNumberOfTree : Ʈ�� ��
					samplesize : �н� �����ͼ� * ����� ����
					nfeatures : Ư¡ ���� �� * ratio
					theNumberOfGoodTree : ���� ����� Ʈ�� ��
					flags : dfusestrongsplits+dfuseevs
					info : ��ȯ �ڵ�
						-2 : ������ Ŭ���� �� ������ ��� ���
						-1 : �Ķ���� ���� �߸��� ���
						 1 : ���������� Ʈ�� ����
					df : Ʈ�� ������ �����ϴ� ����ü
					offsetTh : Regression RF�� ���� Accuracy ������ ����: Out of Bag�����ͷ� �� Ʈ����  ����� offset�� GT offset���� ���̿� ���� �Ӱ谪
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
	@brief			���� ������Ʈ�� �̿��Ͽ� Ȯ���� ����
	@param			df : Ʈ�� ������ �����ϴ� ����ü
					feature : Ư¡ ����
					bUsingProbInLeafNodeFlag : ��������� Ȯ���� ���� ��� ����
					result : Ŭ���� �� ������ Ȯ���� ����
*/
void estimateProbabilityUsingRandomForests(const RANDOMFORESTS& df,
     const double* feature,	
	 bool bUsingProbInLeafNodeFlag,
     double* result);

/**
	@brief			Ư¡ ���͸� �ϳ��� ���� Ʈ���� �����Ͽ� Ȯ���� ����
	@param			df : Ʈ�� ������ �����ϴ� ����ü
					offs : 1���� �迭�� ����Ǿ� �ִ� Ʈ���� ���� ��ġ
					feature : Ư¡ ����
					bUsingProbInLeafNodeFlag : ��������� Ȯ���� ���� ��� ����
					result : Ŭ���� �� ������ Ȯ���� ����
					treeIdx : ���� tree index
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
	@brief			�н� �����͸� �̿��Ͽ� �ϳ��� Ʈ���� �����Ű�� �Լ�
	@param			sampleData : ��ü �н� ������ �� ratioOfSampleData ��ŭ ����� ������
					theNumberOfSampleData : ���� ������ ��
					featureDemension : Ư¡ ����
					theNumberOfClass : Ŭ���� ��
					nfeatures : Ư¡ ���� * ratio
					nvarsinpool : Ư¡ ����
					flags : dfusestrongsplits+dfuseevs
					bufs : Ʈ�� ������ �ʿ��� ���� �����ϴ� ����ü
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
	@brief			��͸� �̿��Ͽ� �ϳ��� Ʈ���� �����Ű�� �Լ�
	@param			sampleData : ��ü �н� ������ �� ratioOfSampleData ��ŭ ����� ������
					theNumberOfSampleData : ���� ������ ��
					featureDemension : Ư¡ ����
					theNumberOfClass : Ŭ���� ��
					nfeatures : Ư¡ ���� * ratio
					nvarsinpool : Ư¡ ����
					flags : dfusestrongsplits+dfuseevs
					numprocessed : ��� �ܰ�
					idx1, idx2 : ���� ������ ũ��� ����
								 numprocessed�� 1�� �� idx1 = 0, idx2 = theNumberOfSampleData-1 ����
								 ��Ͱ� ����� ���� idx1�� ����, idx2�� ������
					bufs : Ʈ�� ������ �ʿ��� ���� �����ϴ� ����ü
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
	@brief			������ �̵�
	@param			vdst : �����Ͱ� �̵��ϴ� ������
					stride_dst : �̵��ϴ� ����
					vsrc : �������� �����
					stride_src : ����ϴ� ����
					n : �̵��ؾ� �ϴ� ��
*/
void moveData(double *vdst, int stride_dst, const double *vsrc,  int stride_src, int n);

/**
	@brief			������ ���ϱ�
	@param			vdst : �����Ͱ� �̵��ϴ� ������
					stride_dst : �̵��ϴ� ����
					n : �̵��ؾ� �ϴ� ��
					alpha : �� �����Ϳ� �������� ��
*/

/**
	@brief			Offset Regressor�� ���Ͽ� Leaf Node�� ��ǥ offset ���� �Լ�
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
// 2�������� RANSAC
int ransac_line_fitting(sPoint *data, int no_data, sPoint *result_data, sLine &model, double distance_threshold, int no_inliers_threshold);
bool find_in_samples (sPoint *samples, int no_samples, sPoint *data);
void get_samples (sPoint *samples, int no_samples, sPoint *data, int no_data);
int compute_model_parameter(sPoint samples[], int no_samples, sLine &model);
double compute_distance(sLine &line, sPoint &x);
double model_verification (sPoint *inliers, int *no_inliers, sLine &estimated_model, sPoint *data, int no_data, double distance_threshold);
double model_verification2 (sLine &model, sPoint *data, int no_data, double distance_threshold);
// 1�������� RANSAC
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

