#include "stdafx.h"
#include "dforest.h"


#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif 	


//static const int dfvnum = 8;
//
//static const int innernodewidth = 3;	// idxbest, tbest, numprocessed => size 3
//#ifdef POINT_REGRESSION_TREE	//jwgim 150825
//static const int leafnodewidth = 3;		//-1, X coordinate, Y coordinate => size 3
//#else
//static const int leafnodewidth = 2;		// -1, calss => size 2
//#endif
//static const int dfusestrongsplits = 1;
//static const int dfuseevs = 2;

static int dfvnum = 8;
static int innernodewidth = 3;	// idxbest, tbest, numprocessed => size 3
static int leafnodewidth = 2;
static int dfusestrongsplits = 1;
static int dfuseevs = 2;

// ===== tree depth ================ 2016-09-05 JMR
int treeLevel = 0;
int maxLevel = 0;
int treeDepthTh = 15;
// =================================

// ===== the minimum number of sample data in leaf node ================ 2016-09-05 JMR
int minSampleDataInLeadNoad = 10;

// ====== RANSAC
int ransacDistanceTh = 10;

int gRFtype = RF_CLASSIFICATION_TREE;

/**
	@brief			랜덤 포레스트 생성 메인 함수
	@param			trainingData : 학습 데이터
					theNumberOfTrainingData : 학습 데이터 수
					featureDemension : 특징 차원 수
					theNumberOfClass : 클래스 수
					theNumberOfTree : 트리 수
					ratioOfSampleData : 학습 데이터 중 트리 생성에 사용할 데이터 비율
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
     double ratioOfSampleData,
	 int theNumberOfGoodTree,	 
     int& info,
     RANDOMFORESTS& df,
	 int offsetTh,
	 int this_ransacDistanceTh)
{
    if( ratioOfSampleData <= 0.0 || ratioOfSampleData > 1.0 )
    {
        info = -1;
        return;
    }
	
	int samplesize = MAX(RND(ratioOfSampleData * theNumberOfTrainingData), 1);
	
	ransacDistanceTh = this_ransacDistanceTh;

	generateDecisionTrees(trainingData, theNumberOfTrainingData, featureDemension, theNumberOfClass, theNumberOfTree, samplesize, MAX((int)(featureDemension/2), 1), theNumberOfGoodTree, dfusestrongsplits+dfuseevs, info, df, offsetTh);
}


/**
	@brief			실제 트리를 생성하는 함수
	@param			trainingData : 학습 데이터
					theNumberOfTrainingData : 학습 데이터 수
					featureDemension : 특징 차원 수
					theNumberOfClass : 클래스 수
					theNumberOfTree : 트리 수
					samplesize : 학습 데이터수 * 서브셋 비율
					nfeatures : 트리 생성을 위해서 랜덤 index column(features)을 추출할 수
					theNumberOfGoodTree : 최종 사용할 트리 수
					flags : dfusestrongsplits+dfuseevs
					info : 반환 코드
						-2 : 지정한 클래스 수 범위를 벗어난 경우
						-1 : 파라매터 값이 잘못된 경우
						 1 : 정상적으로 트리 생성
					df : 트리 정보를 저장하는 구조체
					allocFlag : 트리 배열 메모리 할당 여부 확인
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
	 int offsetTh)
{
	if( gRFtype == RF_REGRESSION_TREE )	//jwgim
		leafnodewidth = 3;
	else if( gRFtype == RF_CLASSIFICATION_TREE_PROB )
		leafnodewidth = theNumberOfClass;
	else
		leafnodewidth = 2;

	int i = 0;
    int j = 0;
    int k = 0;
    int tmpi = 0;
    int lasttreeoffs = 0;
    int offs = 0;
    int treesize = 0;
    int nvarsinpool = 0;
    bool useevs = false;
    double v = 0.0;
    double vmin = 0.0;
    double vmax = 0.0;
    bool bflag = false;
	bool bUseDecisionTree = false;

	// 실제 클래스 별 학습 데이터
	int* countPerClass;
	countPerClass = new int[theNumberOfClass];

	// 클래스 별 뽑아야할 학습 데이터
	int* samplesizePerClass;
	samplesizePerClass = new int[theNumberOfClass];
	
    
    //
    // Test for inputs
    //
    if( theNumberOfTrainingData<1||samplesize<1||samplesize>theNumberOfTrainingData||featureDemension<1||theNumberOfClass<1||theNumberOfTree<1||nfeatures<1 )
    {
        info = -1;
        return;
    }
    if( theNumberOfClass>1 )
    {
        for(i = 0; i < theNumberOfTrainingData; i++)
        {
			if( RND(trainingData[i][featureDemension]) < 0 || RND(trainingData[i][featureDemension]) >= theNumberOfClass )				
            {
                info = -2;
                return;
            }
        }
    }
    info = 1;
    
    //
    // Flags
    //
    useevs = flags/dfuseevs%2!=0;
    
    //
    // Allocate data, prepare header
	// tresize :: innernodewidth :idxbext, tbest,numprocessed & leafnodewidth:-1,class
    //	
  	treesize = 1+innernodewidth*(samplesize-1)+leafnodewidth*samplesize;
	
	RF_INTERNAL_BUFFERS bufs;

	bufs.treebuf = new double[treesize];
	bufs.idxbuf = new int[theNumberOfTrainingData];
	bufs.tmpbufr = new double[theNumberOfTrainingData];
	bufs.tmpbufr2 = new double[theNumberOfTrainingData];		
	if( gRFtype == RF_REGRESSION_TREE )	//jwgim
		bufs.tmpbufr3 = new double[theNumberOfTrainingData];
	bufs.tmpbufi = new int[theNumberOfTrainingData];	
	bufs.varpool = new int[featureDemension];					// 해당 feature idx에서 데이터값의 최대,최소가 동일한 열을 제외한 index를 저장
	bufs.evsbin = new bool[featureDemension];					// 해당 feature idx의 이진값 여부 체크(binary:true, non-binary:false)
	bufs.evssplits = new double[featureDemension];				// 데이터값이 binary value => feature idx의 (최대+최소)/0.5 대입 (대부분 0.5값이 입력)
	bufs.classibuf = new int[2 * theNumberOfClass];	
	bufs.tmpbufw = new double[theNumberOfTrainingData];
	bufs.classwbuf = new double[2 * theNumberOfClass];

	if(df.trees != NULL)
	{
		delete [] df.trees;
		//delete [] df.treesFlag;
		delete [] df.treesWeight;
	}

	// tree 생성
	df.trees = new double[theNumberOfTree * treesize];

	// tree weight 저장을 위한 배열 생성
	df.treesWeight = new double[theNumberOfTree];
	memset(df.treesWeight, 0, sizeof(double) * theNumberOfTree);

	// tree 사용 여부 설정 배열 생성
	//df.treesFlag = new int[theNumberOfTree];
	//memset(df.treesFlag, 0, sizeof(int) * theNumberOfTree);

	// 학습 데이터 * ratioOfSampleData 수의 학습 데이터를 저장하는 배열 생성
	double** sampleData = new double*[samplesize];

	for(i = 0; i < samplesize; i++)
	{
		if( gRFtype == RF_REGRESSION_TREE )	//jwgim
			sampleData[i] = new double[featureDemension + 3];			// feature dimenstion + X offset + Y offset + data weight
		else
			sampleData[i] = new double[featureDemension + 2];			// featuredemension + class labe(1) + data weight(1)
	}

	// 랜덤적으로 추출된 데이터(row)가 저장되는 배열
	double** tempData = new double*[samplesize];
	for(i = 0; i < samplesize; i++)
	{
		if( gRFtype == RF_REGRESSION_TREE )	//jwgim
			tempData[i] = new double[featureDemension + 3];				// feature dimension + X offset + Y offset + data weight
		else
			tempData[i] = new double[featureDemension + 2];				// featuredemension + class labe(1) + data weight(1)
	}
    
	// 학습 데이터에 대한 weight
	double* dataWeight = new double[theNumberOfTrainingData];	
	for(i = 0 ; i < theNumberOfTrainingData ; i++)					// 초기 dataWeight 설정
	{
		dataWeight[i] = (double)(1.0/theNumberOfTrainingData);
	}

	// In-Of-Bag에 사용된 데이터 체크 배열
	int* is_InOfBagData = new int[theNumberOfTrainingData];	

	// Tree Accuracy 측정 결과 저장 배열
	double* accuracyOfTree = new double[theNumberOfTree];
	memset(accuracyOfTree, 0.0, sizeof(double)*theNumberOfTree);

	// Tree Accuracy 임계치 (50%)
	double accuracyTh = 0.5;

    //
    // Prepare variable pool and EVS (extended variable selection/splitting) buffers
    // (whether EVS is turned on or not):
    // 1. detect binary variables and pre-calculate splits for them
    // 2. detect variables with non-distinct values and exclude them from pool
    //
    for(i = 0; i < featureDemension; i++)
    {
        bufs.varpool[i] = i;
    }
    nvarsinpool = featureDemension;
    if( useevs )
    {
        for(j = 0; j < featureDemension; j++)
        {
			// vmin, max값 초기값 설정
            vmin = trainingData[0][j];								
            vmax = vmin;								
			
			// 해당 feature idx에서 max, min value 추출
            for(i = 0; i < theNumberOfTrainingData; i++)	
            {
                v = trainingData[i][j];
                vmin = MIN(vmin, v);
                vmax = MAX(vmax, v);
            }

			// 해당 feature idx에서 max, min value가 동일하다면 bufs.varpool에서 해당 j열 제외
            if( vmin == vmax )
            {   
                bufs.varpool[j] = bufs.varpool[nvarsinpool-1];		// feature의 index값이 저장된 bufs.varpool에서 해당 j index에 마지막 index를 대입
                bufs.varpool[nvarsinpool-1] = -1;					// 마지막 index에는 -1 대입
                nvarsinpool = nvarsinpool-1;						// bufs.varpool에 저장된 index값 개수 감소
                continue;
            }
			
			// 해당 feature idx의 데이터값이 binary value인지 non-binary value인지 체크해서 buf.evsbin에 대입
			// 해당 feature idx의 데이터값 중에서 vmin&vmax와 같지 않은 값이 존재한다면 데이터들은 non-binary value.			
            bflag = false;
            for(i = 0; i < theNumberOfTrainingData; i++)
            {
                v = trainingData[i][j];
                if( v != vmin && v != vmax )
                {
                    bflag = true;
                    break;
                }
            }
            if( bflag )
            {
                
                //
                // non-binary variable
                //
                bufs.evsbin[j] = false;
            }
            else
            {                
                //
                // Prepare
                //
                bufs.evsbin[j] = true;
                bufs.evssplits[j] = 0.5*(vmin+vmax);
                if( bufs.evssplits[j] <= vmin )
                {
                    bufs.evssplits[j] = vmax;
                }
            }
        }
    }
    
    //
    // RANDOM FOREST FORMAT
    // W[0]         -   size of array
    // W[1]         -   version number
    // W[2]         -   feature dimension
    // W[3]         -   the number of class (1 for regression)
    // W[4]         -   the number of tree
    // W[5]         -   trees offset
    //
    //
    // TREE FORMAT
    // W[Offs]      -   size of sub-array
    //     node info:
    // W[K+0]       -   variable number        (-1 for leaf mode)
    // W[K+1]       -   threshold              (class/value for leaf node)
    // W[K+2]       -   ">=" branch index      (absent for leaf node)
    //
    //
    df.featureDemension = featureDemension;
    df.theNumberOfClass = theNumberOfClass;
    df.theNumberOfTree = theNumberOfTree;

	if( gRFtype == RF_REGRESSION_TREE )	//jwgim
		countPerClass[0] = theNumberOfTrainingData;
	else {
		// 클래스별 데이터 수 알아내기
		for(i = 0; i < theNumberOfClass; i++)
		{
			countPerClass[i] = 0;		
			for(j = 0; j < theNumberOfTrainingData; j++)
			{
				if(trainingData[j][featureDemension] ==  i)
				{
					countPerClass[i]++;
				}
			}
		}
	}
	// 클래스 별 뽑아야 할 데이터 수 알아내기
	// 총합이 samplesize와 같아야 함
	for(i = 0; i < theNumberOfClass; i++)
	{
		samplesizePerClass[i] = (int)(samplesize / theNumberOfClass);
	}

	// 클래스 별 할당된 데이터 수의 총 합이
	// samplesize와 같아야 함
	int restCount = 0;
	int totalCount = 0;

	// 클래스 별 최소 할당 데이터 수 설정
	// 할당 되어야 할 데이터 수가 실제 데이터 수보다 많을 경우에는
	// 실제 데이터 수로 대체 함
	for(i = 0; i < 	theNumberOfClass; i++)
	{
		if(samplesizePerClass[i] > countPerClass[i])
		{
			// 할당 되어야 할 나머지 데이터 수의 합
			restCount += samplesizePerClass[i] - countPerClass[i];
			samplesizePerClass[i] = countPerClass[i];
		}
	}

	// 추정된 할당 데이터 수
	for(i = 0; i < 	theNumberOfClass; i++)
	{
		totalCount += samplesizePerClass[i];
	}

	// 추정된 할당 데이터 수와 추가적으로 할당해야할 데이터 수가 samplesize 보다 많은 경우
	// 추가적으로 할당해야할 데이터 수에 차를 빼줌
	if(totalCount + restCount > samplesize)
	{
		restCount -= (totalCount + restCount) - samplesize;
	}
	// 추정된 할당 데이터 수와 추가적으로 할당해야할 데이터 수가 samplesize 보다 작은 경우
	// 추가적으로 할당해야할 데이터 수에 차를 더해줌
	else if(totalCount + restCount < samplesize)
	{
		restCount += samplesize - (totalCount + restCount);
	}

	i = 0;
	totalCount = 0;
	if(restCount != 0)
	{
		// 추가적으로 할당해야할 데이터 수를 분배
		while(totalCount != samplesize)
		{
			totalCount = 0;
			// 할당된 데이터의 수 + 1이 실제 데이터의 수 보다 작거나 같을 경우
			// 할당된 데이터 수를 +1 함
			if(samplesizePerClass[i % theNumberOfClass] + 1 <= countPerClass[i % theNumberOfClass])
			{
				samplesizePerClass[i % theNumberOfClass]++;
				restCount--;

				// 추정된 할당 데이터 수
				for(j = 0; j < 	theNumberOfClass; j++)
				{
					totalCount += samplesizePerClass[j];
				}
			}
			i++;
		}
	}
    
    //
    // Build forest
    //
    offs = 0;

	// 2 class 분류기일 경우 각 클래스 데이터에서 균등하게 트리 생성 데이터를 추출함
	/*if(theNumberOfClass == 2)
	{
		int* permbuf = new int[samplesize];		// sampledate index 저장 배열
		for(int i1 = 0; i1 < samplesize; i1++)
		{
			permbuf[i1] = i1;
		}

		for(i = 0; i < theNumberOfTree; i++)
		{
			bUseDecisionTree = true;

			// 클래스 별로
			int dataCount = 0;
			int sampleCount = 0;
			for(k = 0; k < theNumberOfClass; k++)
			{
				// 랜덤 추출을 위한 배열 초기화
				int* permbuf2 = new int[samplesizePerClass[k]];
				for(int i1 = 0; i1 < samplesizePerClass[k]; i1++)
				{
					permbuf2[i1] = i1;
				}

				// 할당된 데이터 추출
				for(int i1 = 0; i1 < samplesizePerClass[k]; i1++)
				{
					j = i1 + (rand() % (samplesizePerClass[k]-i1));
					tmpi = permbuf2[i1];
					permbuf2[i1] = permbuf2[j];
					permbuf2[j] = tmpi;
					j = permbuf2[i1];

					moveData(tempData[sampleCount], 1, trainingData[dataCount + j], 1, featureDemension + 1);		// data feature + class label 복사
					tempData[sampleCount++][featureDemension+1] = dataWeight[dataCount + j];						// data weight 복사
				}

				dataCount += countPerClass[k];

				delete [] permbuf2;
			}

			for(k = 0; k < samplesize; k++)
			{
				j = k+(rand() % (samplesize-k));
				tmpi = permbuf[k];
				permbuf[k] = permbuf[j];
				permbuf[j] = tmpi;
				j = permbuf[k];
				moveData(sampleData[k], 1, tempData[j], 1, featureDemension + 1);									// data feature + class label 복사
				sampleData[k][featureDemension+1] = tempData[j][featureDemension+1];								// data weight 복사
			}
        
			//
			// build tree, copy
			//			
			growDecisionTree(sampleData, samplesize, featureDemension, theNumberOfClass, nfeatures, nvarsinpool, flags, bufs);
			
			//
			// measure the quilty of a builded tree
			//					
			bUseDecisionTree = measureQuiltyOfDecisionTree(df, bufs.treebuf, trainingData, dataWeight, theNumberOfTrainingData, i);			
			

			if( bUseDecisionTree )
			{
				j = RND(bufs.treebuf[0]);
				moveData(&df.trees[offs], 1, &bufs.treebuf[0], 1, j);
				lasttreeoffs = offs;
				offs = offs+j;

				for( k=0 ; k<j ; k++ )
				{
					bufs.treebuf[k] = 0;
				}
			}else
			{
				i--;
			}

		}

		delete [] permbuf;
	}
	// 그 외 1 class, multi class 분류기일 경우 학습 데이터에서 랜덤하게 트리 생성 데이터를 추출함
	else */
	{
		double accuracy = 0.0;

		int* permbuf = new int[theNumberOfTrainingData];
		for(int i1 = 0; i1 < theNumberOfTrainingData; i1++)
		{
			permbuf[i1] = i1;
		}

		for(i = 0; i < theNumberOfTree; i++)
		{
			bUseDecisionTree = true;

			// InOfBag사용 데이터 체크배열 초기화
			memset(is_InOfBagData, 0, sizeof(int)*theNumberOfTrainingData);

			// samplesize 만큼 랜덤적으로 idx1~idx2 행들을 추출
			for(k = 0; k < samplesize; k++)
			{
				j = k+(rand() % (theNumberOfTrainingData-k));
				tmpi = permbuf[k];
				permbuf[k] = permbuf[j];
				permbuf[j] = tmpi;
				j = permbuf[k];
				if( gRFtype == RF_REGRESSION_TREE ) {	//jwgim
					moveData(sampleData[k], 1, trainingData[j], 1, featureDemension + 2);								// data feature + X offset + Y offset 복사
					sampleData[k][featureDemension+2] = dataWeight[j];													// data weight 복사
				}
				else {
					moveData(sampleData[k], 1, trainingData[j], 1, featureDemension + 1);								// data feature + class label 복사
					sampleData[k][featureDemension+1] = dataWeight[j];													// data weight 복사
				}
				
				is_InOfBagData[j] = 1;		// 현재 트리 생성에 포함된 데이터 체크
			}
        			
			//
			// build tree, copy
			//

			TRACE(_T("\n=====Tree(%d) Start=================\n"), i);



			for (int itr = 0; itr < 156; itr++) {
				Trace(_T("\nDataWeight : %f \n"), dataWeight[itr]);
			}



			growDecisionTree(sampleData, samplesize, featureDemension, theNumberOfClass, nfeatures, nvarsinpool, flags, bufs);
			//TRACE(_T("=====Tree(%d) MaxLevel(%d) End===================\n"), i, maxLevel);
			//
			// measure the quilty of a builded tree
			//
			if( gRFtype == RF_REGRESSION_TREE ) {	//jmr : weight RF
				
				// 2016-03-22 Out-of-Bag을 이용하여 현재 생성된 트리의 Accuracy를 측정
				accuracy = measureQuiltyOfDecisionTree(df, bufs.treebuf, trainingData, is_InOfBagData, theNumberOfTrainingData, offsetTh);	
				
				// 현재 트리의 Accuracy가 50%이하라면, 현재 트리 폐기
				if( accuracy < accuracyTh )	{
					bUseDecisionTree = false;
					TRACE(_T("the current tree was deleted.\n"));		// 2016-09-21
				}
				else
					accuracyOfTree[i] = accuracy;

			}else // boosted RF
			{
				if( theNumberOfClass != 0 )
					bUseDecisionTree = measureQuiltyOfDecisionTree(df, bufs.treebuf, trainingData, dataWeight, theNumberOfTrainingData, theNumberOfClass, i);
			}
			if( bUseDecisionTree )
			{
				j = RND(bufs.treebuf[0]);
				moveData(&df.trees[offs], 1, &bufs.treebuf[0], 1, j);
				lasttreeoffs = offs;
				offs = offs+j;

				for( k=0 ; k<j ; k++ )
				{
					bufs.treebuf[k] = 0;
				}
			}else
			{
				i--;
			}			
		}

		delete [] permbuf;
	}
	
    df.bufsize = offs;

	// 2016-09-05 JMR Boosted RF - Decision Tree의 weight값의 합이 1이 되도록 갱신
	double wsum = 0.0;
	for( i=0 ; i<theNumberOfTree ; i++ )
		wsum += df.treesWeight[i];
	for( i=0 ; i<theNumberOfTree ; i++ )
		df.treesWeight[i] /= wsum;

	// 2016-03-22 트리들의 Accuracy를 이용하여 weight값 계산
	if( gRFtype == RF_REGRESSION_TREE ) {	//jmr
		getWeightUsingTreeAccuracy(df, accuracyOfTree, theNumberOfTree);
	}	

	/*if(theNumberOfTree != theNumberOfGoodTree)
	{	
		// 생성된 tree 중 좋은 tree 선별
		selectGoodTrees(df, trainingData, theNumberOfTrainingData, theNumberOfGoodTree);
	}
	else
	{
		for(i = 0; i < theNumberOfTree; i++)
		{
			df.treesFlag[i] = 1;
		}
	}*/

	// 메모리 해제
	for(i = 0; i < samplesize; i++)
	{
		delete [] sampleData[i];
		delete [] tempData[i];
	}
	
	delete [] sampleData;
	delete [] tempData;
	delete [] dataWeight;
	
	delete [] countPerClass;	
	delete [] samplesizePerClass;

	delete [] bufs.treebuf;
	delete [] bufs.idxbuf;
	delete [] bufs.tmpbufr;
	delete [] bufs.tmpbufr2;
	if( gRFtype == RF_REGRESSION_TREE )	//jwgim
		delete [] bufs.tmpbufr3;
	delete [] bufs.tmpbufi;
	delete [] bufs.varpool;
	delete [] bufs.evsbin;
	delete [] bufs.evssplits;
	delete [] bufs.classibuf;	
	delete [] bufs.tmpbufw;
	delete [] bufs.classwbuf;
	delete [] is_InOfBagData;
	delete [] accuracyOfTree;
}

#ifdef RF_REGRESSION_TREE	//jwgim 150826
void estimateOffsetUsingRandomForests(const RANDOMFORESTS& df, const double* feature, int* offset, double* var)
{
	leafnodewidth = 3;
	int offs;
	double* sumArr = new double[2];
	double** result = nullptr;
	result = new double*[df.theNumberOfTree];
	for( int i = 0; i < df.theNumberOfTree; i++ ) {
		result[i] = new double[2];
		memset( result[i], 0, sizeof(double)*2 );
	}
	memset( offset, 0, sizeof(int)*2 );
	memset( sumArr, 0, sizeof(double)*2 );
	offs = 0;
	
	//////////////////////
	//Tree result get	
	/////////////////////
	/*double mean_x, mean_y;
	double stddev_x, stddev_y;
	mean_x = mean_y = stddev_x = stddev_y = 0.0;

	
	// calculate mean
	for( int i = 0; i < df.theNumberOfTree; i++ ) {
		estimateOffsetUsingDecisionTree( df, offs, feature, result[i], i );

		mean_x += result[i][0];
		mean_y += result[i][1];

		offs = offs + RND( df.trees[offs] );
	}

	mean_x /= (double) df.theNumberOfTree;
	mean_y /= (double) df.theNumberOfTree;

	//calculate standard deviation
	for( int i = 0; i < df.theNumberOfTree; i++ ) {
		stddev_x += pow( mean_x - result[i][0], 2 );
		stddev_y += pow( mean_y - result[i][1], 2 );
	}
	stddev_x = sqrt( stddev_x / df.theNumberOfTree );
	stddev_y = sqrt( stddev_y / df.theNumberOfTree );

	*var = stddev_x + stddev_y;

	
	//stddev_x = stddev_x * 1.5;
	//stddev_y = stddev_y * 1.5;
	//int count = 0;
	for( int i = 0; i < df.theNumberOfTree; i++ ) {		

		if( (mean_x + stddev_x ) < result[i][0] || ( mean_x - stddev_x) > result[i][0] )
			continue;

		if( (mean_y + stddev_y ) < result[i][1] || ( mean_y - stddev_y) > result[i][1] )
			continue;
			

		//sumArr[0] += result[i][0];
		//sumArr[1] += result[i][1];
		//count++;

		sumArr[0] += result[i][0] * df.treesWeight[i];
		sumArr[1] += result[i][1] * df.treesWeight[i]; 				
		
	}*/

	//2016-10-06 
	for( int i = 0; i < df.theNumberOfTree; i++ ) {	
		estimateOffsetUsingDecisionTree( df, offs, feature, result[i], i );

		sumArr[0] += result[i][0] * df.treesWeight[i];
		sumArr[1] += result[i][1] * df.treesWeight[i]; 			

		offs = offs + RND( df.trees[offs] );
	}

	//sumArr[0] /= (double) df.theNumberOfTree;
	//sumArr[1] /= (double) df.theNumberOfTree;

	//offset[0] = (int)(sumArr[0] / count);
	//offset[1] = (int)(sumArr[1] / count);
	offset[0] = (int)sumArr[0];
	offset[1] = (int)sumArr[1];

	//release memory
	delete [] sumArr;	
	for( int i = 0; i < df.theNumberOfTree; i++ )
		delete [] result[i];
	delete [] result;
}

static void estimateOffsetUsingDecisionTree(const RANDOMFORESTS& df, int offs, const double* feature, double* result, int treeIdx)
{
	leafnodewidth = 3;
    int k;
    
    // Set pointer to the root
    k = offs+1;
    
    // Navigate through the tree
    while(true) {
        if( df.trees[k] == -1 ) {
			result[0] = df.trees[k+1];
			result[1] = df.trees[k+2];			
			break;
        }
        if( feature[RND(df.trees[k])] < df.trees[k+1] )
            k = k+innernodewidth;
        else
            k = offs+RND(df.trees[k+2]);
    }	
}
#endif



/**
	@brief			랜덤 포레스트를 이용하여 확률값 추정
	@param			df : 트리 정보를 저장하는 구조체
					feature : 특징 벡터
					result : 클래스 별 추정된 확률값 저장
					theNumberOfClass : 클래스 개수
					bUsingProbInLeafNodeFlag : 현재 학습된 tree의 leaf node의 확률값 사용 여부
*/
void estimateProbabilityUsingRandomForests(const RANDOMFORESTS& df,
     const double* feature,
	 bool bUsingProbInLeafNodeFlag,
     double* result
	 )
{
	if( bUsingProbInLeafNodeFlag )
		leafnodewidth = df.theNumberOfClass + 1;
	else
		leafnodewidth = 2;

    int offs;
    int i;
    double v;

    //
    // Proceed
    //
    offs = 0;
    for(i = 0; i < df.theNumberOfClass; i++)
    {
        result[i] = 0;
    }
    for(i = 0; i < df.theNumberOfTree; i++)
    {		
		//
		// Process basic tree
		//
		estimateProbabilityUsingDecisionTree(df, offs, feature, bUsingProbInLeafNodeFlag, result, i);
        
		//
		// Next tree
		//
		offs = offs + RND(df.trees[offs]);       
    }

 //   v = 1.0 / (double)(df.theNumberOfTree);
 //   multiplyData(&result[0], 1, df.theNumberOfClass, v);
	/*2016-09-05 JMR Boosted RF의 Decision Tree의 weight값은 트레이닝 단계에서 weight의 합이 1이 되도록 변경함
	if( bUsingProbInLeafNodeFlag )
	{
		for(i = 0; i < df.theNumberOfClass; i++)
			result[i] /= (double)df.theNumberOfTree;		
	}
	else
	{	
		v=0.0;
		for(i = 0; i < df.theNumberOfClass; i++)		
		   v += result[i];
		for(i = 0; i < df.theNumberOfClass; i++)
			result[i] /= v;
	}*/
}

/**
	@brief			특징 벡터를 하나의 결정 트리에 적용하여 확률값 추정
	@param			df : 트리 정보를 저장하는 구조체
					offs : 1차원 배열에 저장되어 있는 트리의 시작 위치
					feature : 특징 벡터
					bUsingProbInLeafNodeFlag :  현재 학습된 tree의 leaf node의 확률값 사용 여부
					result : 클래스 별 추정된 확률값 저장
					treeIdx : 현재 tree index
*/
static void estimateProbabilityUsingDecisionTree(const RANDOMFORESTS& df,
     int offs,	 
     const double* feature,	
	 bool bUsingProbInLeafNodeFlag,
     double* result,
	 int treeIdx)
{
	if( bUsingProbInLeafNodeFlag )
		leafnodewidth = df.theNumberOfClass + 1;
	else
		leafnodewidth = 2;

    int k;
    int idx;
    
    //
    // Set pointer to the root
    //
    k = offs+1;
    
    //
    // Navigate through the tree
    //
    while(true)
    {
        if( df.trees[k] == -1 )
        {
            if( df.theNumberOfClass==1 )
            {
                result[0] = result[0]+df.trees[k+1];
            }
            else
            {               
				// 2016-09-05 JMR
				if( bUsingProbInLeafNodeFlag )			
				{
					for( int i=0 ; i<df.theNumberOfClass ; i++ )
					{
						result[i] += df.trees[k+1+i]*df.treesWeight[treeIdx];			// 클래스별 확률값(해당트의 각 클래스별 확률값*treeWeight) 누적 
					}
				}else
				{
					idx = RND(df.trees[k+1]);		//  class number 					
					//result[idx] = result[idx]+1;
					result[idx] = result[idx] + df.treesWeight[treeIdx];
				}

            }
            break;
        }
        if( feature[RND(df.trees[k])] < df.trees[k+1] )
        {
            k = k+innernodewidth;
        }
        else
        {
            k = offs+RND(df.trees[k+2]);
        }
    }
}


/**
	@brief			학습 데이터를 이용하여 하나의 트리를 성장시키는 함수
	@param			sampleData : 전체 학습 데이터 중 ratioOfSampleData 만큼 추출된 데이터 (각 클래스마다 할당된 개수만큼 샘플데이터가 들어있음)
					theNumberOfSampleData : 샘플 데이터 수
					featureDemension : 특징 차원
					theNumberOfClass : 클래스 수
					nfeatures : 트리 생섯을 위해 랜덤적으로 추출한 feature index의 수
					nvarsinpool : 해당 feature index의 전체 데이터의 min, max가 같은 값을 가지는 feature index(열)을 제외한 feature index가 저장되어 있음
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
     RF_INTERNAL_BUFFERS& bufs)
{
    int numprocessed;
    int i;


	if(theNumberOfSampleData <= 0)
	{
		return;
	}
    
    //
    // Prepare IdxBuf. It stores indices of the training set elements.
    // When training set is being split, contents of IdxBuf is
    // correspondingly reordered so we can know which elements belong
    // to which branch of decision tree.
    //
    for(i = 0; i < theNumberOfSampleData; i++)
    {
        bufs.idxbuf[i] = i;
    }

	// 2016-09-05 JMR
	treeLevel = 0;
	maxLevel = 0;		

    //
    // Recursive procedure
    //
    numprocessed = 1;
    growDecisionTreeUsingRecursive(sampleData, theNumberOfSampleData, featureDemension, theNumberOfClass, nfeatures, nvarsinpool, flags, numprocessed, 0, theNumberOfSampleData-1, bufs);
    bufs.treebuf[0] = numprocessed;	

	
}


/**
	@brief			재귀를 이용하여 하나의 트리를 성장시키는 함수
	@param			sampleData : 전체 학습 데이터 중 ratioOfSampleData 만큼 추출된 데이터 (각 클래스마다 할당된 개수만큼 샘플데이터가 들어있음)
					theNumberOfSampleData : 샘플 데이터 수
					featureDemension : 특징 차원
					theNumberOfClass : 클래스 수
					nfeatures : 특징 차원 * ratio
					nvarsinpool : 해당 feature index의 전체 데이터의 min, max가 같은 값을 가지는 feature index(열)을 제외한 feature index가 저장되어 있음
					flags : dfusestrongsplits+dfuseevs
					numprocessed : 재귀 단계
					idx1, idx2 : 샘플 데이터 크기로 지정(row index들을 지정)
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
     RF_INTERNAL_BUFFERS& bufs)
{
	int a = 0;
	int i = 0, i1 = 0, i2 = 0, j = 0, k = 0;
	int info = 0;
	int idxbest = 0;	
    int varcur = 0;
	int oldnp = 0;

	double sl = 0.0, sr = 0.0;
	double ebest = 0.0, tbest = 0.0;
	double s = 0.0, v = 0.0, v1 = 0.0, v2 = 0.0;
    double w = 0.0, swl = 0.0, swr = 0.0;
    double threshold = 0.0;
	double currms = 0.0;
    double vl = 0.0, vr = 0.0, vn = 0.0;	

	bool bflag = false;
    bool useevs = false;

#ifdef RF_REGRESSION_TREE	//jwgim 150825
	double vx, vy;
	double vx1, vy1, vx2, vy2;
	double currmsx, currmsy;
#endif
	if( gRFtype == RF_CLASSIFICATION_TREE_PROB )
		leafnodewidth = theNumberOfClass + 1;
	else if( gRFtype == RF_REGRESSION_TREE )	//jwgim
		leafnodewidth = 3;
	else
		leafnodewidth = 2;
		
	// 2016-09-05 JMR			
	treeLevel++;		
	if(maxLevel < treeLevel)
	{
		maxLevel = treeLevel;
	}	

	if(theNumberOfSampleData <= 0)
	{
		return;
	}
	if(idx2 < idx1)
	{
		return;
	}

    useevs = flags/dfuseevs%2!=0;

	// 2013-12-31 JMR 
	int treeLevelForSetting = 3;
	int nCountNonZeroClass = 0;
	int nIndexNonZeroClass = -1;	

    //
    // Leaf node
    //
	if( idx2==idx1 )
    {
        bufs.treebuf[numprocessed] = -1;
     
		if( gRFtype == RF_CLASSIFICATION_TREE_PROB )
		{
			for( i=0 ; i<theNumberOfClass ; i++ )
			{
				if( i == sampleData[bufs.idxbuf[idx1]][featureDemension] )
					bufs.treebuf[numprocessed+1+i] = 1.0;
				else
					bufs.treebuf[numprocessed+1+i] = 0.0;
			}
		}else if( gRFtype == RF_REGRESSION_TREE )	//jwgim
		{
			bufs.treebuf[numprocessed+1] = sampleData[bufs.idxbuf[idx1]][featureDemension];		// class label		
			bufs.treebuf[numprocessed+2] = sampleData[bufs.idxbuf[idx1]][featureDemension+1];	// y offset
		}else
		{
			bufs.treebuf[numprocessed+1] = sampleData[bufs.idxbuf[idx1]][featureDemension];		// class label
		}
        numprocessed = numprocessed+leafnodewidth;
        return;
    }

	// 2016-09-05 JMR
	// treeDepth or leaf node에 존재하는 데이터 개수의 수 제한에 걸린다면
	// 현재 데이터 중에서 가장 많은 빈도를 차지하는 클래스가 대표 클래스로 설정
	if( treeLevel >= treeDepthTh || theNumberOfSampleData <= minSampleDataInLeadNoad )
	{
		bufs.treebuf[numprocessed] = -1;

		if( gRFtype == RF_REGRESSION_TREE )
		{
			// 현재 노드에 속한 샘플 데이터를 이용하여 대표 offset  추출 
			// 대표 offset은 outlier을 제거하고 평균값 사용
			getOffsetValuesForRegressionLeafNode(sampleData, featureDemension, numprocessed, idx1, idx2, bufs, vx, vy);

			bufs.treebuf[numprocessed+1] = vx;
			bufs.treebuf[numprocessed+2] = vy;		

		}else
		{
			int classLabel = -1;		// 대표 클래스
			int maxCount = -1;

			for(i = 0; i < theNumberOfClass; i++)
			{
				bufs.classibuf[i] = 0;
			}	

			// count class Label
			for(i = idx1; i <= idx2; i++)
			{
				j = RND(sampleData[bufs.idxbuf[i]][featureDemension]);
				bufs.classibuf[j] += 1;													
			}

			// extract the most frequent class Label 
			for(i = 0; i < theNumberOfClass; i++)
			{
				if( bufs.classibuf[i] > maxCount )  
				{
					classLabel = i;
					maxCount = bufs.classibuf[i];
				}
			}

			if( gRFtype == RF_CLASSIFICATION_TREE_PROB )
			{
				for(i = 0; i < theNumberOfClass; i++){				
					bufs.treebuf[numprocessed+1+i] =  (double)bufs.classibuf[i] / (double)(idx2-idx1+1);
				}				
			}
			else
				bufs.treebuf[numprocessed+1] = classLabel;		// class label						
		}
		numprocessed = numprocessed+leafnodewidth;
		return;
	}

	/* 2016-09-05 JMR 주석처리 : 이미 기존 코드에 포함되어 있음 
	// 2013-12-31 JMR Leaf node 조건 추가 ///////////////////////////////////////////////////////////////////////////////////////////////
	// 현재 입력된 행들의 클래스가 1개의 클래스만이 존재하면 leaf node !
	if( theNumberOfClass>1 )
	{
		for(i = 0; i < theNumberOfClass; i++)
		{
			bufs.classibuf[i] = 0;
		}
   
		for(i = idx1; i <= idx2; i++)								// JMR 해당 행(row)수를 for~
		{
			j = RND(sampleData[bufs.idxbuf[i]][featureDemension]);	// JMR j : sampleData의 i번째 행의 마지막 column값 (이건 feature data 마지막 class 분류값)
			bufs.classibuf[j] = bufs.classibuf[j]+1;				// JMR bufs의 해당 classbuf의 클래스버퍼를 1증가 시킨다.
		}

		nCountNonZeroClass = 0;
		nIndexNonZeroClass = -1;
		for(i = 0; i < theNumberOfClass; i++)
		{
			if ( bufs.classibuf[i] != 0 )
			{
				nCountNonZeroClass++;
				nIndexNonZeroClass = i;
			}			
		}

		if( nCountNonZeroClass == 1 )	// JMR 현재 입력된 행들의 클래스가 1개 밖에 존재하지 않는다면 Leaf node로 설정
		{
			bufs.treebuf[numprocessed] = -1;
			bufs.treebuf[numprocessed+1] = sampleData[bufs.idxbuf[idx1]][featureDemension];
			numprocessed = numprocessed+leafnodewidth;

			return;
		}
	}
	// 2013-12-31 JMR Leaf node 조건 추가 ///////////////////////////////////////////////////////////////////////////////////////////////*/
	    
    //
    // Non-leaf node.
    // Select random variable, prepare split:
    // 1. prepare default solution - no splitting, class at random
    // 2. investigate possible splits, compare with default/best
	// idx1 ~ idx2 행의 information Gain 조사
	// ebest : 최적 Inpormation Gain
	// idxbest : 현재 column index
	// tbest : 최적 threshold
    //
	// 추가 설명 : 
	// while(i)문이 nvarsinpool이나 nfeatures 수만큼 돌면서
	// 랜덤한 column index를 추출하고 해당 column값들의 최적의 threshold값과 E(currms)를 추출한다.
	// 이때 threshold값과 E값은 기존의 ebest와 비교되어서 변경된다. (기존 ebest는 xy의 전체행들에 대해서 각 클래스를 카운트하여 얻은 E값임)
	// 즉, 현재 idx1~idx2의 행에 대한 데이터를 잘 분류해줄 최적의 idxbest(column index), ebest, tbest를 추출
    idxbest = -1;
    if( theNumberOfClass>1 )
    {
		// 2016-09-05 JMR 주석 해제 초기 ebest값 필요 

        //
        // default solution for classification
        //
        for(i = 0; i < theNumberOfClass; i++)
        {
            bufs.classibuf[i] = 0;
			bufs.classwbuf[i] = 0.0;
        }
        s = idx2-idx1+1;		
        for(i = idx1; i <= idx2; i++)
        {
            j = RND(sampleData[bufs.idxbuf[i]][featureDemension]);
            bufs.classibuf[j] += 1;													// count class Label
			bufs.classwbuf[j] += sampleData[bufs.idxbuf[i]][featureDemension+1];	// sum the data weight by class label
        }
        ebest = vn = 0;
        for(i = 0; i < theNumberOfClass; i++)
        {			
            //ebest = ebest + bufs.classibuf[i] * pow(1-bufs.classibuf[i]/s, 2) + (s-bufs.classibuf[i]) * pow(bufs.classibuf[i]/s, 2);  
							
			// 2015-01-28 JMR ebest = ebest + ( bufs.classibuf[i]/s * (log((double)(bufs.classibuf[i]/s))/log(2.0)) );	
			// 2016-09-05 JMR ebest 수식 변경	
			w = bufs.classwbuf[i] / s;
			if( w != 0 )	ebest += w * ( log(w)/log(2.0) );			
        }
        // ebest = sqrt(ebest/(theNumberOfClass*(idx2-idx1+1)));		

		ebest = ebest * -1;		
		
    }
    else
    {        
        //
		// default solution for regression
		//
		if( gRFtype == RF_REGRESSION_TREE ) {	//jwgim
			vx = vy = 0;
			for(i = idx1; i <= idx2; i++) {
				vx = vx + sampleData[bufs.idxbuf[i]][featureDemension];
				vy = vy + sampleData[bufs.idxbuf[i]][featureDemension+1];
			}
			vx = vx / (idx2-idx1+1);
			vy = vy / (idx2-idx1+1);

			double ebestx, ebesty;
			ebestx = ebesty = 0;
			for(i = idx1; i <= idx2; i++) {
				ebestx = ebestx + pow( sampleData[bufs.idxbuf[i]][featureDemension]-vx, 2 );
				ebesty = ebesty + pow( sampleData[bufs.idxbuf[i]][featureDemension+1]-vy, 2 );
			}
			ebestx = sqrt( ebestx / (idx2-idx1+1) );
			ebesty = sqrt( ebesty / (idx2-idx1+1) );
			ebest = ebestx + ebesty;
		}
		else {
			v = 0;
			for(i = idx1; i <= idx2; i++)
			{
				v = v+sampleData[bufs.idxbuf[i]][featureDemension];
			}

			v = v/(idx2-idx1+1);
			ebest = 0;
			for(i = idx1; i <= idx2; i++)
			{
				ebest = ebest + pow(sampleData[bufs.idxbuf[i]][featureDemension]-v, 2); 					
			}
			ebest = sqrt(ebest/(idx2-idx1+1));
		}
    }
    i = 0;

	

	// 2013-12-31 JMR 
    // Root Node 일 경우, features의 n(4/10)번째 ~ n(6/10)번째 column에서 split 한다.
	if( numprocessed == 1 )
	{		
		i = int( RND( MIN(nfeatures, nvarsinpool) * 0.4 ));
	}

    // 2013-12-16 JMR : nfeatures(=column * ratio)와 nvarsinpool(=최대&최소값이 동일한 열을 제외한 column개수) 둘중 작은 값을 사용
	//					ocs-lbp, haar-like, lid는 모두 특징벡터수가 작기 때문에 nvarsinpool 로 사용
	while(i <= MIN(nfeatures, nvarsinpool)-1) 	
    {
		
		// 2013-12-31 JMR 
       	// Root Node 일 경우, features의 n(4/10)번째 ~ n(6/10)번째 column에서 split 한다.
		if( numprocessed == 1 )
		{
			if( i > int( RND( MIN(nfeatures, nvarsinpool) * 0.6 )) )
			{
				if( info > 0 )				
					break;					
				else 
					i = int( RND( MIN(nfeatures, nvarsinpool) * 0.4 )); 
			}
		}

        //
        // select variables from pool
		// bufs.varpool()을 순차적으로 돌면서 column index를 랜덤으로 추출하여, 
		// 랜덤으로 추출된 column index의 varpool의 값과 바꿔준다.
        //
        j = i + (rand() % (nvarsinpool-i));
        k = bufs.varpool[i];
        bufs.varpool[i] = bufs.varpool[j];
        bufs.varpool[j] = k;
        varcur = bufs.varpool[i];

        //
        // load variable values to working array
        //
        // apply EVS preprocessing: if all variable values are same,
        // variable is excluded from pool.
        //
        // This is necessary for binary pre-splits (see later) to work.
        //
        for(j = idx1; j <= idx2; j++)
        {
			// bufs.tmpbufr : idx1~idx2행을 돌면서 랜덤하게 추출된 column index의 feature값을 저장
            bufs.tmpbufr[j-idx1] = sampleData[bufs.idxbuf[j]][varcur];
        }
        if( useevs )
        {
            bflag = false;
            v = bufs.tmpbufr[0];
            for(j = 0; j <= idx2-idx1; j++)
            {
                if( bufs.tmpbufr[j] != v )
                {
                    bflag = true;
                    break;
                }
            }
            if( !bflag )
            {
                
                //
                // exclude variable from pool,
                // go to the next iteration.
                // I is not increased.
                //
                k = bufs.varpool[i];
                bufs.varpool[i] = bufs.varpool[nvarsinpool-1];
                bufs.varpool[nvarsinpool-1] = k;
                nvarsinpool = nvarsinpool-1;
                continue;
            }
        }
        
        //
        // load labels to working array
        //
        if( theNumberOfClass>1 )
        {
            for(j = idx1; j <= idx2; j++)
            {
				// bufs.tmpbufi : idx1 ~ idx2 행을 돌면서 해당 행의 class 저장
                bufs.tmpbufi[j-idx1] = RND(sampleData[bufs.idxbuf[j]][featureDemension]);		// class label
				bufs.tmpbufw[j-idx1] = sampleData[bufs.idxbuf[j]][featureDemension+1];			// data weight
            }
        }
        else
		{
			for(j = idx1; j <= idx2; j++)
			{
				if( gRFtype == RF_REGRESSION_TREE ) {	//jwgim
					// bufs.tmpbufr2 : 클래스수가 1보다 같거나 작을 때, idx1 ~ idx2 행을 돌면서 해당 행의 offset 저장 
					bufs.tmpbufr2[j-idx1] = sampleData[bufs.idxbuf[j]][featureDemension];			// X offset
					bufs.tmpbufr3[j-idx1] = sampleData[bufs.idxbuf[j]][featureDemension+1];			// Y offset
					bufs.tmpbufw[j-idx1] = sampleData[bufs.idxbuf[j]][featureDemension+2];			// data weight
				}
				else {
					// bufs.tmpbufr2 : 클래스수가 1보다 같거나 작을 때, idx1 ~ idx2 행을 돌면서 해당 행의 class 저장 
					bufs.tmpbufr2[j-idx1] = sampleData[bufs.idxbuf[j]][featureDemension];			// class label
					bufs.tmpbufw[j-idx1] = sampleData[bufs.idxbuf[j]][featureDemension+1];			// data weight
				}
			}
        }
        //
        // calculate split
        //
		if( useevs&&bufs.evsbin[varcur] )			// EVS를 사용하고 & 현재 추출된 column 값이 이진값이면
        {
            //
            // Pre-calculated splits for binary variables.
            // Threshold is already known, just calculate RMS error
            //
            threshold = bufs.evssplits[varcur];
            if( theNumberOfClass>1 )
            {
                
                //
                // classification-specific code
                //
                for(j = 0; j < 2*theNumberOfClass; j++)
                {
                    bufs.classibuf[j] = 0;
					bufs.classwbuf[j] = 0.0;
                }
                sl = sr = 0.0;          
				swl = swr = 0.0;
                for(j = 0; j <= idx2-idx1; j++)
                {
                    k = bufs.tmpbufi[j];					
                    if( bufs.tmpbufr[j] < threshold )
                    {
                        bufs.classibuf[k] += 1;
						bufs.classwbuf[k] += bufs.tmpbufw[j]; 						
						swl += bufs.tmpbufw[j]; 						
                        sl += 1;						
                    }
                    else
                    {
                        bufs.classibuf[k+theNumberOfClass] += 1;
						bufs.classwbuf[k+theNumberOfClass] += bufs.tmpbufw[j]; 
						swr += bufs.tmpbufw[j]; 						
                        sr += 1;						
                    }
                }
				
                currms = 0;
				vn = vl = vr = 0;
                for(j = 0; j < theNumberOfClass; j++)
                {										
					w = (bufs.classwbuf[j] + bufs.classibuf[theNumberOfClass+j]) / (swl+swr);		// left part + right part Entropy					
					if( w != 0 )	vn += w * ( log(w)/log(2.0) );					
					w = bufs.classwbuf[j] / swl;													// left part Entropy
					if( w != 0 )	vl += w * ( log(w)/log(2.0) );					
					w = bufs.classwbuf[theNumberOfClass+j] / swr;									// right part Entropy
					if( w != 0 )	vr += w * ( log(w)/log(2.0) );					
                }                
				vn *= -1;
				vl *= -1;
				vr *= -1;
				if( sl == 0 )			currms = vn - abs(sr/(sl+sr))*vr;
				else if( sr == 0 )		currms = vn - abs(sl/(sl+sr))*vl;
				else					currms = vn - abs(sl/(sl+sr))*vl - abs(sr/(sl+sr))*vr;

            }
            else
            {
                //
                // regression-specific code
                //
				if( gRFtype == RF_REGRESSION_TREE ) {	//jwgim
					sl = 0;
					sr = 0;
					vx1 = vx2 = vy1 = vy2 = 0;

					for(j = 0; j <= idx2-idx1; j++) {
						if( bufs.tmpbufr[j] < threshold ) {
							vx1 = vx1 + bufs.tmpbufr2[j];
							vy1 = vy1 + bufs.tmpbufr3[j];
							sl = sl+1;
						}
						else {
							vx2 = vx2 + bufs.tmpbufr2[j];
							vy2 = vy2 + bufs.tmpbufr3[j];
							sr = sr+1;
						}
					}

					vx1 = vx1 / sl;
					vy1 = vy1 / sl;
					vx2 = vx2 / sr;
					vy2 = vy2 / sr;

					currms = 0;
					currmsx = currmsy = 0;

					for(j = 0; j <= idx2-idx1; j++) {
						if( bufs.tmpbufr[j] < threshold ) {
							currmsx = currmsx + pow( vx1 -bufs.tmpbufr2[j], 2);
							currmsy = currmsy + pow( vy1 -bufs.tmpbufr3[j], 2);
						}
						else {
							currmsx = currmsx + pow( vx2 -bufs.tmpbufr2[j], 2);
							currmsy = currmsy + pow( vy2 -bufs.tmpbufr3[j], 2);
						}
					}
					currmsx = sqrt( currmsx / (idx2-idx1+1) );
					currmsy = sqrt( currmsy / (idx2-idx1+1) );
					currms = currmsx + currmsy;
				}
				else {
					sl = 0;
					sr = 0;
					v1 = 0;
					v2 = 0;
					for(j = 0; j <= idx2-idx1; j++)
					{
						if( bufs.tmpbufr[j] < threshold )
						{
							v1 = v1+bufs.tmpbufr2[j];
							sl = sl+1;
						}
						else
						{
							v2 = v2+bufs.tmpbufr2[j];
							sr = sr+1;
						}
					}

					v1 = v1/sl;
					v2 = v2/sr;
					currms = 0;
					for(j = 0; j <= idx2-idx1; j++)
					{
						if( bufs.tmpbufr[j] < threshold )
						{
							currms = currms + pow(v1-bufs.tmpbufr2[j], 2);
						}
						else
						{
							currms = currms + pow(v2-bufs.tmpbufr2[j], 2);
						}
					}
					currms = sqrt(currms/(idx2-idx1+1));
				}
			}
            info = 1;
        }
        else			// JMR EVS를 사용하지 않거나(OR) 현재 추출된 column 값이 이진값이 아닐때
        {
			// bufs.tmpbufr : idx1~idx2행을 돌면서 랜덤하게 추출된 column index의 feature값을 저장
            // bufs.tmpbufi : idx1 ~ idx2 행을 돌면서 해당 행의 class 저장
			// bufs.tmpbufr2 : 클래스수가 1보다 같거나 작을 때, idx1 ~ idx2 행을 돌면서 해당 행의 class 저장 
			// bufs.classibuf : n개의 클래스의 각 클래스를 가지는 데이터 개수
            //
            // Generic splits
            //
            if( theNumberOfClass>1 )
            {
				// 입력된 샘플데이터에서 랜덤적으로 추출된 column index의 데이터를 최적으로 분류해줄 Inpormation Gain E를 계산하여 최적의 threshold를 추출
                splitDataOnIndex(bufs.tmpbufr, bufs.tmpbufi, bufs.tmpbufw, bufs.classibuf, bufs.classwbuf, idx2-idx1+1, theNumberOfClass, dfusestrongsplits, info, threshold, currms);
            }
            else
            {
				if( gRFtype == RF_REGRESSION_TREE ) 	//jwgim
					splitDataOnValue(bufs.tmpbufr, bufs.tmpbufr2, bufs.tmpbufr3, idx2-idx1+1, dfusestrongsplits, info, threshold, currms);
				else
					splitDataOnValue(bufs.tmpbufr, bufs.tmpbufr2, idx2-idx1+1, dfusestrongsplits, info, threshold, currms);
            }
        }
        if( info>0 )		// splitDataOnIndex 함수에서 threshold, currms를 변경하고 난 후에는 1이 됨.
        {
			// ebest : bufs.tmpbufr의 값 전체를 조사해서 각 클래스 별로 E를 계산한 값
			// currms : bufs.tmpbufr값들의 임의의 split threshold를 정하고, 이 임계값에 따라서 bufs.tmpbufr의 rigth와 left의 그룹을 나눈후 각 right, left 그룹의 클래스에 따른 E를 계산
            if( theNumberOfClass>1 ) 
			{
				// 2016-09-05 JMRif( ( ebest == 0.0 && tbest == 0.0 && idxbest == 0 ) || currms >= ebest )		// ebest가 새롭게 추출된 currms보다 작다면 교체 ( 가장 높은 information Gain을 찾는다)
				if( currms >= ebest )		// ebest가 새롭게 추출된 currms보다 작다면 교체 ( 가장 높은 information Gain을 찾는다)
				{
					ebest = currms;			// ebest  : 최적의 information Gain
					idxbest = varcur;		// idbest : 최적의 E값을 가지는 column index
					tbest = threshold;		// tbest  : idbest에서의 threshold					
				}
			}else
			{
				if( currms < ebest )		// ebest가 새롭게 추출된 currms보다 크다면 교체 ( 낮은 Gini 불순도값을 찾는다)
				{
					ebest = currms;			// ebest  : 최적의 Gini 불순도
					idxbest = varcur;		// idbest : 최적의 E값을 가지는 column index
					tbest = threshold;		// tbest  : idbest에서의 threshold
				}
			}
        }
        
        //
        // Next iteration
        //
        i = i+1;
    }

    //
    // to split or not to split
    //
    if( idxbest<0 )
    {
        //
        // All values are same, cannot split.
        //
        bufs.treebuf[numprocessed] = -1;
        if( theNumberOfClass>1 )
        {             
			if( gRFtype == RF_CLASSIFICATION_TREE_PROB )
			{		
				int classLabel = sampleData[bufs.idxbuf[i]][featureDemension];				
				for( i=0 ; i<theNumberOfClass ; i++ )
					if( i == classLabel )	bufs.treebuf[numprocessed+1+i] =  1.0;
					else                    bufs.treebuf[numprocessed+1+i] =  0.0;


			}else
			{
				//
				// Select random class label (randomness allows us to
				// approximate distribution of the classes)
				//
				bufs.treebuf[numprocessed+1] = RND(sampleData[bufs.idxbuf[idx1 + (rand() % (idx2-idx1+1))]][featureDemension]);
			}
        }
        else
        {
			//
			// Select average (for regression task).
			//
			if( gRFtype == RF_REGRESSION_TREE ) {	//jwgim
				
				// 현재 노드에 속한 샘플 데이터를 이용하여 대표 offset  추출 
				// 대표 offset은 outlier을 제거하고 평균값 사용
				getOffsetValuesForRegressionLeafNode(sampleData, featureDemension, numprocessed, idx1, idx2, bufs, vx, vy);

				bufs.treebuf[numprocessed+1] = vx;
				bufs.treebuf[numprocessed+2] = vy;			
				
				//avg
				//vx = vy = 0;
				//for(i = idx1; i <= idx2; i++) {
				//	vx = vx + sampleData[bufs.idxbuf[i]][featureDemension];
				//	vy = vy + sampleData[bufs.idxbuf[i]][featureDemension+1];
				//}
				//vx = vx / (idx2-idx1+1);
				//vy = vy / (idx2-idx1+1);
				//bufs.treebuf[numprocessed+1] = vx;
				//bufs.treebuf[numprocessed+2] = vy;
			}			
			else{
				v = 0;
				for(i = idx1; i <= idx2; i++)
				{
					v = v + sampleData[bufs.idxbuf[i]][featureDemension] / (idx2-idx1+1);
				}
				bufs.treebuf[numprocessed+1] = v;
			}
		}
        numprocessed = numprocessed+leafnodewidth;
    }
    else  
    {      
        //
        // we can split
        //
        bufs.treebuf[numprocessed] = idxbest;
        bufs.treebuf[numprocessed+1] = tbest;
        i1 = idx1;		// idx1 : row 시작
        i2 = idx2;		// idx2 : row 끝
        while(i1<=i2)	
        {   
            //
            // Reorder indices so that left partition is in [Idx1..I1-1],
            // and right partition is in [I2+1..Idx2]
            //
			// 추가설명
			// 최적의 E값을 가지는 column index의 행을 i1~i2까지 돌면서 ( xy(전체행, 최적열) )
			// threshold값을 중심으로 left partition에는 작은값이, right partition에는 큰값이 들어가게끔
			// bufs.idxbuf를 변경
            if( sampleData[bufs.idxbuf[i1]][idxbest] < tbest )
            {
                i1 = i1+1;
                continue;
            }
            if( sampleData[bufs.idxbuf[i2]][idxbest] >= tbest )
            {
                i2 = i2-1;
                continue;
            }
            j = bufs.idxbuf[i1];
            bufs.idxbuf[i1] = bufs.idxbuf[i2];
            bufs.idxbuf[i2] = j;
            i1 = i1+1;
            i2 = i2-1;
        } // i1>i2 종료

		//
		// Result 
		// 최적 column의 데이터값 : [      작은값         ] tbest [      큰값          ]
		//                           idx1               i2         i1              idx2
		//
        oldnp = numprocessed;
        numprocessed = numprocessed+innernodewidth;

		// idx1 ~ i1-1까지 재귀함수 호출!!
        growDecisionTreeUsingRecursive(sampleData, theNumberOfSampleData, featureDemension, theNumberOfClass, nfeatures, nvarsinpool, flags, numprocessed, idx1, i1-1, bufs);
		treeLevel--;

        bufs.treebuf[oldnp+2] = numprocessed;		

		// JMR i2+1 ~ idx2까지 재귀함수 호출!!
        growDecisionTreeUsingRecursive(sampleData, theNumberOfSampleData, featureDemension, theNumberOfClass, nfeatures, nvarsinpool, flags, numprocessed, i2+1, idx2, bufs);
		treeLevel--;
    }
}


/**
	@breif		임의적 추출된 column index의 데이터를 최적으로 분류해줄 Inpormation Gain E를 계산하여 최적의 threshold를 추출
	@param		x : 임의적 추출된 column index의 데이터들
				c : x값의 클래스 
				weights : w값의 weight
				cntbuf : 데이터들의 각각의 클래스 개수
				wcntbuf : 데이터에서 각각의 클래스에 대한 weight 합
				n : 입력된 데이터 개수(idx1~idx2개수)
				nc : 클래스 수
				flags : dfusestrongsplits
				info : 함수 내부에서 초기에는 -3이었다가 e, threshold가 변경되면 1로 변경
				threshold  : 현재 split하는 데이터값
				e : 현재 x데이터의 최적 ebest값
	@note		x를 정렬한 후 전체에서 1/4, 2/4, 3/4 위치에 값들을 조사하여 그 값들을 중심으로 left와 right의 최적의 E값을 찾음
				이때 left와 right를 최적으로 나눠주는 임계값(cursplit)값을 찾음.
				E(cure)값이 e(maxrealnumber:1E300)보다 작다면
				threshold값을 현재 cursplit로 변경해주고 e도 현재 함수에서 추출된 cure값으로 변경.
*/
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
     double& e)
{
    int i;
    int neq;
    int nless;
    int ngreater;
    int q;
    int qmin;
    int qmax;
    int qcnt;
    double cursplit;
    int nleft;    
	double vn, vr, vl;	
    double cure;    
	double w, swl, swr;
    double sl, sr;
    
	// x sort 
    sortFastInteger(x, c, n);

    //e = 1E300;
	e = -9999;
    threshold = 0.5*(x[0]+x[n-1]);
    info = -3;

    if( flags/dfusestrongsplits % 2==0 )
    {
        
        //
        // weak splits, split at half
        //
        qcnt = 2;
        qmin = 1;
        qmax = 1;
    }
    else
    {
        
        //
        // strong splits: choose best quartile
        //
        qcnt = 4;
        qmin = 1;
        qmax = 3;
    }
    for(q = qmin; q <= qmax; q++)
    {
        cursplit = x[n*q/qcnt];
        neq = 0;
        nless = 0;
        ngreater = 0;
        for(i = 0; i <= n-1; i++)
        {
            if( x[i] < cursplit )
            {
                nless = nless+1;
            }
            if( x[i] == cursplit )
            {
                neq = neq+1;
            }
            if( x[i] > cursplit )
            {
                ngreater = ngreater+1;
            }
        }
        
        if( nless!=0||ngreater!=0 )
        {
            
            //
            // set threshold between two partitions, with
            // some tweaking to avoid problems with floating point
            // arithmetics.
            //
            // The problem is that when you calculates C = 0.5*(A+B) there
            // can be no C which lies strictly between A and B (for example,
            // there is no floating point number which is
            // greater than 1 and less than 1+eps). In such situations
            // we choose right side as theshold (remember that
            // points which lie on threshold falls to the right side).
            //
            if( nless<ngreater )
            {
                cursplit = 0.5 * (x[nless+neq-1]+x[nless+neq]);
                nleft = nless+neq;
                if( cursplit <= x[nless+neq-1] )
                {
                    cursplit = x[nless+neq];
                }
            }
            else
            {
                cursplit = 0.5*(x[nless-1]+x[nless]);
                nleft = nless;
                if( cursplit <= x[nless-1] )
                {
                    cursplit = x[nless];
                }
            }
            info = 1;
            cure = 0;
			swl = swr = 0.0;
            for(i = 0; i <= 2*nc-1; i++)
            {
                cntbuf[i] = 0;
				wcntbuf[i] = 0.0;
            }
            for(i = 0; i <= nleft-1; i++)
            {
                cntbuf[c[i]] += 1;
				wcntbuf[c[i]] += weights[i];
				swl += weights[i];
            }
            for(i = nleft; i <= n-1; i++)
            {
                cntbuf[nc+c[i]] += 1;
				wcntbuf[nc+c[i]] += weights[i];
				swr += weights[i];
            }
            sl = nleft;
            sr = n-nleft;
            vn = vl = vr = 0.0;	
            for(i = 0; i <= nc-1; i++)
            {
				w = (wcntbuf[i] + wcntbuf[nc+i]) / (swl+swr);				// 전체 데이터에 대한 entropy
				if( w != 0 )				vn += w * (log(w)/log(2.0)) ;								
				w = wcntbuf[i] / swl;										// left 데이터에 대한 entropy
				if( w != 0 && swl != 0 )	vl += w * (log(w)/log(2.0));
				w = wcntbuf[nc+i] / swr;									// right 데이터에 대한 entropy
				if( w != 0 && swr != 0 )	vr += w * (log(w)/log(2.0));

            }
			// cure = sqrt(v/(nc*n));
			vn *= -1;
			vl *= -1;
			vr *= -1;
			
			if( sl == 0 )		cure = vn - abs(sr/n)*vr;
			else if( sr == 0 )	cure = vn - abs(sl/n)*vl;
			else				cure = vn - abs(sl/n)*vl - abs(sr/n)*vr;

            if( cure > e )
            {
                threshold = cursplit;
                e = cure;
            }
        }
    }
}



static void splitDataOnValue(double* x,
     double* y,
     int n,
     int flags,
     int& info,
     double& threshold,
     double& e)
{
    int i;
    int neq;
    int nless;
    int ngreater;
    int q;
    int qmin;
    int qmax;
    int qcnt;
    double cursplit;
    int nleft;
    double v;
    double cure;

    sortFastReal(x, y, n);
    e = 1E300;
    threshold = 0.5*(x[0]+x[n-1]);
    info = -3;
    if( flags/dfusestrongsplits % 2==0 )
    {
        
        //
        // weak splits, split at half
        //
        qcnt = 2;
        qmin = 1;
        qmax = 1;
    }
    else
    {
        
        //
        // strong splits: choose best quartile
        //
        qcnt = 4;
        qmin = 1;
        qmax = 3;
    }
    for(q = qmin; q <= qmax; q++)
    {
        cursplit = x[n*q/qcnt];
        neq = 0;
        nless = 0;
        ngreater = 0;
        for(i = 0; i <= n-1; i++)
        {
            if( x[i] < cursplit )
            {
                nless = nless+1;
            }
            if( x[i] == cursplit )
            {
                neq = neq+1;
            }
            if( x[i] > cursplit )
            {
                ngreater = ngreater+1;
            }
        }
        
        if( nless!=0||ngreater!=0 )
        {
            
            //
            // set threshold between two partitions, with
            // some tweaking to avoid problems with floating point
            // arithmetics.
            //
            if( nless<ngreater )
            {
                cursplit = 0.5*(x[nless+neq-1]+x[nless+neq]);
                nleft = nless+neq;
                if( cursplit <= x[nless+neq-1] )
                {
                    cursplit = x[nless+neq];
                }
            }
            else
            {
                cursplit = 0.5*(x[nless-1]+x[nless]);
                nleft = nless;
                if( cursplit <= x[nless-1] )
                {
                    cursplit = x[nless];
                }
            }
            info = 1;
            cure = 0;
            v = 0;
            for(i = 0; i <= nleft-1; i++)
            {
                v = v+y[i];
            }
            v = v/nleft;
            for(i = 0; i <= nleft-1; i++)
            {
                cure = cure + pow(y[i]-v, 2);
            }
            v = 0;
            for(i = nleft; i <= n-1; i++)
            {
                v = v+y[i];
            }
            v = v/(n-nleft);
            for(i = nleft; i <= n-1; i++)
            {
                cure = cure + pow(y[i]-v, 2);
            }
            cure = sqrt(cure/n);
            if( cure < e )
            {
                threshold = cursplit;
                e = cure;
            }
        }
    }
}

#ifdef RF_REGRESSION_TREE	//jwgim 150825
static void splitDataOnValue(double* x,
     double* y,
	 double* z, 
     int n,
     int flags,
     int& info,
     double& threshold,
     double& e)
{
    int i;
    int neq;
    int nless;
    int ngreater;
    int q;
    int qmin;
    int qmax;
    int qcnt;
    double cursplit;
    int nleft;
    double vx, vy;
    double cure, curex, curey;

    sortFastReal(x, y, z, n);
    e = 1E300;
    threshold = 0.5*(x[0]+x[n-1]);
    info = -3;
    if( flags/dfusestrongsplits % 2==0 )
    {
        
        //
        // weak splits, split at half
        //
        qcnt = 2;
        qmin = 1;
        qmax = 1;
    }
    else
    {
        
        //
        // strong splits: choose best quartile
        //
        qcnt = 4;
        qmin = 1;
        qmax = 3;
    }
    for(q = qmin; q <= qmax; q++)
    {
        cursplit = x[n*q/qcnt];
        neq = 0;
        nless = 0;
        ngreater = 0;
        for(i = 0; i <= n-1; i++)
        {
            if( x[i] < cursplit )
            {
                nless = nless+1;
            }
            if( x[i] == cursplit )
            {
                neq = neq+1;
            }
            if( x[i] > cursplit )
            {
                ngreater = ngreater+1;
            }
        }
        
        if( nless!=0||ngreater!=0 )
        {
            
            //
            // set threshold between two partitions, with
            // some tweaking to avoid problems with floating point
            // arithmetics.
            //
            if( nless<ngreater )
            {
                cursplit = 0.5*(x[nless+neq-1]+x[nless+neq]);
                nleft = nless+neq;
                if( cursplit <= x[nless+neq-1] )
                {
                    cursplit = x[nless+neq];
                }
            }
            else
            {
                cursplit = 0.5*(x[nless-1]+x[nless]);
                nleft = nless;
                if( cursplit <= x[nless-1] )
                {
                    cursplit = x[nless];
                }
            }
            info = 1;
            cure = curex = curey = 0;
            vx = vy = 0;
            for(i = 0; i <= nleft-1; i++)
            {
				vx = vx + y[i];
				vy = vy + z[i];
            }

            vx = vx/nleft;
			vy = vy/nleft;
            for(i = 0; i <= nleft-1; i++)
            {
				curex = curex + pow(y[i]-vx, 2);
				curey = curey + pow(z[i]-vy, 2);
            }

            vx = vy = 0;
            for(i = nleft; i <= n-1; i++)
            {
                vx = vx + y[i];
				vy = vy + z[i];
            }

            vx = vx/(n-nleft);
			vy = vy/(n-nleft);
            for(i = nleft; i <= n-1; i++)
            {
				curex = curex + pow(y[i]-vx, 2);
				curey = curey + pow(z[i]-vy, 2);
            }
			curex = sqrt( curex / n );
			curey = sqrt( curey / n );

            cure = curex + curey;

            if( cure < e )
            {
                threshold = cursplit;
                e = cure;
            }
        }
    }
}
#endif


/**
	@brief			데이터 이동
	@param			vdst : 데이터가 이동하는 목적지
					stride_dst : 이동하는 단위
					vsrc : 데이터의 출발지
					stride_src : 출발하는 단위
					n : 이동해야 하는 수
*/
void moveData(double *vdst, int stride_dst, const double* vsrc,  int stride_src, int n)
{
	int i;

    if( stride_dst != 1 || stride_src != 1 )
    {
        for(i = 0; i < n; i++, vdst += stride_dst, vsrc += stride_src)
		{
            *vdst = *vsrc;
		}
    }
    else
    {
        int n2 = n / 2;
        for(i = 0; i < n2; i++, vdst += 2, vsrc += 2)
        {
            vdst[0] = vsrc[0];
            vdst[1] = vsrc[1];
        }
        if( n % 2 != 0 )
		{
            vdst[0] = vsrc[0];
		}
    }
}

/**
	@brief			데이터 곱하기
	@param			vdst : 데이터가 이동하는 목적지
					stride_dst : 이동하는 단위
					n : 이동해야 하는 수
					alpha : 각 데이터에 곱해지는 값
*/
void multiplyData(double *vdst,  int stride_dst, int n, double alpha)
{
    int i;
    if( stride_dst !=1 )
    {
        for(i = 0; i < n; i++, vdst += stride_dst)
		{
            *vdst *= alpha;
		}
    }
    else
    {
        for(i = 0; i < n; i++, vdst++)
		{
            *vdst *= alpha;
		}
    }
}

void sortFastInteger(double* a, int* b, int n)
{
    int i;
    int k;
    int t;
    double tmp;
    int tmpi;

    
    //
    // Special cases
    //
    if( n<=1 )
    {
        return;
    }
    
    //
    // General case, N>1: sort, update B
    //
    i = 2;
    do
    {
        t = i;
        while(t!=1)
        {
            k = t/2;
            if( a[k-1] >= a[t-1] )
            {
                t = 1;
            }
            else
            {
                tmp = a[k-1];
                a[k-1] = a[t-1];
                a[t-1] = tmp;
                tmpi = b[k-1];
                b[k-1] = b[t-1];
                b[t-1] = tmpi;
                t = k;
            }
        }
        i = i+1;
    }
    while(i<=n);
    i = n-1;
    do
    {
        tmp = a[i];
        a[i] = a[0];
        a[0] = tmp;
        tmpi = b[i];
        b[i] = b[0];
        b[0] = tmpi;
        t = 1;
        while(t!=0)
        {
            k = 2*t;
            if( k>i )
            {
                t = 0;
            }
            else
            {
                if( k<i )
                {
                    if( a[k] > a[k-1] )
                    {
                        k = k+1;
                    }
                }
                if( a[t-1] >= a[k-1] )
                {
                    t = 0;
                }
                else
                {
                    tmp = a[k-1];
                    a[k-1] = a[t-1];
                    a[t-1] = tmp;
                    tmpi = b[k-1];
                    b[k-1] = b[t-1];
                    b[t-1] = tmpi;
                    t = k;
                }
            }
        }
        i = i-1;
    }
    while(i>=1);
}
void sortFastReal(double* a, double* b, int n)
{
    int i;
    int k;
    int t;
    double tmp;
    double tmpr;

    
    //
    // Special cases
    //
    if( n<=1 )
    {
        return;
    }
    
    //
    // General case, N>1: sort, update B
    //
    i = 2;
    do
    {
        t = i;
        while(t!=1)
        {
            k = t/2;
            if( a[k-1] >= a[t-1] )
            {
                t = 1;
            }
            else
            {
                tmp = a[k-1];
                a[k-1] = a[t-1];
                a[t-1] = tmp;
                tmpr = b[k-1];
                b[k-1] = b[t-1];
                b[t-1] = tmpr;
                t = k;
            }
        }
        i = i+1;
    }
    while(i<=n);
    i = n-1;
    do
    {
        tmp = a[i];
        a[i] = a[0];
        a[0] = tmp;
        tmpr = b[i];
        b[i] = b[0];
        b[0] = tmpr;
        t = 1;
        while(t!=0)
        {
            k = 2*t;
            if( k>i )
            {
                t = 0;
            }
            else
            {
                if( k<i )
                {
                    if( a[k] > a[k-1] )
                    {
                        k = k+1;
                    }
                }
                if( a[t-1] >= a[k-1] )
                {
                    t = 0;
                }
                else
                {
                    tmp = a[k-1];
                    a[k-1] = a[t-1];
                    a[t-1] = tmp;
                    tmpr = b[k-1];
                    b[k-1] = b[t-1];
                    b[t-1] = tmpr;
                    t = k;
                }
            }
        }
        i = i-1;
    }
    while(i>=1);
}

#ifdef RF_REGRESSION_TREE	//jwgim 150825
void sortFastReal(double* a, double* b, double* c, int n)
{
    int i;
    int k;
    int t;
    double tmp;
    double tmpr;
	double tmpr2;

    
    //
    // Special cases
    //
    if( n<=1 )
    {
        return;
    }
    
    //
    // General case, N>1: sort, update B
    //
    i = 2;
    do
    {
        t = i;
        while(t!=1)
        {
            k = t/2;
            if( a[k-1] >= a[t-1] )
            {
                t = 1;
            }
            else
            {
                tmp = a[k-1];
                a[k-1] = a[t-1];
                a[t-1] = tmp;
                tmpr = b[k-1];
                b[k-1] = b[t-1];
                b[t-1] = tmpr;
				tmpr2 = c[k-1];
				c[k-1] = c[t-1];
                c[t-1] = tmpr2;
                t = k;
            }
        }
        i = i+1;
    }
    while(i<=n);
    i = n-1;
    do
    {
        tmp = a[i];
        a[i] = a[0];
        a[0] = tmp;
        tmpr = b[i];
        b[i] = b[0];
        b[0] = tmpr;
		tmpr2 = c[i];
        c[i] = c[0];
        c[0] = tmpr2;
        t = 1;
        while(t!=0)
        {
            k = 2*t;
            if( k>i )
            {
                t = 0;
            }
            else
            {
                if( k<i )
                {
                    if( a[k] > a[k-1] )
                    {
                        k = k+1;
                    }
                }
                if( a[t-1] >= a[k-1] )
                {
                    t = 0;
                }
                else
                {
                    tmp = a[k-1];
                    a[k-1] = a[t-1];
                    a[t-1] = tmp;
                    tmpr = b[k-1];
                    b[k-1] = b[t-1];
                    b[t-1] = tmpr;
					tmpr2 = c[k-1];
                    c[k-1] = c[t-1];
                    c[t-1] = tmpr2;
                    t = k;
                }
            }
        }
        i = i-1;
    }
    while(i>=1);
}
#endif

static void selectGoodTrees(const RANDOMFORESTS& df, double** feature, int theNumberOfTrainingData, int theNumberOfGoodTree)
{
	/*
	register int i, j;

	int classLabel = -1;
	int estimatedClassLabel = -1;
	
	double score = 0.0;
	double threshold = 0.7;

	double* data = new double[df.featureDemension];
	memset(data, 0, sizeof(double) * df.featureDemension);

	double** treePerformance = new double*[2];
	for(i = 0; i < 2; i++)
	{
		treePerformance[i] = new double[df.theNumberOfTree];
		memset(treePerformance[i], 1, sizeof(double) * df.theNumberOfTree);
	}

	int treeOffs = 0;
	int offs = 0;

	for(i = 0; i < df.theNumberOfTree; i++)
	{
		for(j = 0; j < theNumberOfTrainingData; j++)
		{
			// 학습 데이터 1개 추출
			moveData(data, 1, feature[j], 1, df.featureDemension);

			// 변수 초기화
			estimatedClassLabel = -1;

			// 트리의 offset 가져오기
			treeOffs = offs + 1;

			// tree에서 추출한 학습 데이터의 확률 추정
			while(true)
			{
				if( df.trees[treeOffs] == -1.0 )
				{
					estimatedClassLabel = (int)df.trees[treeOffs + 1];

					break;
				}
				if( data[RND(df.trees[treeOffs])] < df.trees[treeOffs + 1] )
				{
					treeOffs += innernodewidth;
				}
				else
				{
					treeOffs = offs + RND(df.trees[treeOffs + 2]);
				}
			}

			// 학습 데이터의 class label 가져오기
			classLabel = (int)feature[j][df.featureDemension];

			// 학습 데이터의 class label과 트리에 의해 추정된 class label이 다르면
			if(classLabel != estimatedClassLabel)
			{
				score++;
			}
		}

		// 다음 tree 사용을 위한 offset 증가
		offs += RND(df.trees[offs]);

		// tree의 분류 성능의 확률 계산
		score /= (double)theNumberOfTrainingData;

		// row 0 : tree index
		// row 1 : tree 분류 성능
		treePerformance[0][i] = i;
		treePerformance[1][i] = score;
	}

	// score에 따라 정렬
	bubbleSort(df, treePerformance, df.theNumberOfTree);

	// 성능이 좋은 상위 N개 tree만 사용
	for(i = 0; i < theNumberOfGoodTree; i++)
	{
		df.treesFlag[(int)treePerformance[0][i]] = 1;
	}



	delete [] data;

	for(i = 0; i < 2; i++)
	{
		delete [] treePerformance[i];
	}
	delete [] treePerformance;*/
}

static void bubbleSort(const RANDOMFORESTS& df, double** data, const int& n)
{
	register int j, k;
	double temp1, temp2;

	for(j = n - 1; j > 0; j--)
	{
		for(k = 0; k < j; k++)
		{
			if(data[1][k] > data[1][k+1])
			{
				temp1 = data[1][k+1];
				data[1][k+1] = data[1][k];
				data[1][k] = temp1;

				temp2 = data[0][k+1];
				data[0][k+1] = data[0][k];
				data[0][k] = temp2;
			}
		}
	}
}

/**
		@brief			Boosted Random Forest를 위한 생성된 하나의 DecisionTree의 성능을 평가
						하여 해당 decisionTree의 treeWeight값을 결정 및 dataWeight값 갱신
						** 반드시 2클래스이상의 멀티 클래스에만 적용가능
		@param			treebuf : decision Tree 구조체
						trainingData : 샘플 feature values + class label 정보를 가짐
						dataWeight : 샘플 데이터에 대한 weight
						theNumberOfTrainingData : 샘플 데이터 개수
						theNumberOfClass : 클래스 개수
						featureDemension : 샘플데이터 차원수
						treeIdx : 현재 트리의 index
		@return			현재 decision tree의 사용여부(0:사용X, 1:사용)
*/
bool measureQuiltyOfDecisionTree(const RANDOMFORESTS& df, double* treebuf, double** trainingData, double* dataWeight, int theNumberOfTrainingData, int theNumberOfClass, int treeIdx)
{
	register int i, j; 

	bool useDecisionTree = false;

	int classLabel = -1;
	int estimatedClassLabel = -1;
	
	int featureDemension = df.featureDemension;
	
	double* data = new double[featureDemension];					// feature data + class label(1)
	memset(data, 0, sizeof(double) * featureDemension);

	int* dataResult = new int[theNumberOfTrainingData];				// 현재 decision tree 적용 후, 학습 데이터의 결과 (0:정상검출, -1:오검출)
	memset(dataResult, 0, sizeof(int) * theNumberOfTrainingData);
		
	double treeError = 0.0;		// decision tree's error rate

	double allw = 0.0;			// 전체 데이터의 data weight 합
	double misw = 0.0;			// 오검출된 데이터의 data weight 합

	double maxProb = -1.0;		// 최대 확률값을 가지는 클래스의 확률값 
	
	int treeOffs = 1;
	
	for(i = 0; i < theNumberOfTrainingData; i++)
	{
		// 학습 데이터 1개 추출
		moveData(data, 1, trainingData[i], 1, featureDemension);		// feature data 복사

		// 변수 초기화
		estimatedClassLabel = -1;
		
		treeOffs = 1;

		maxProb = -1.0;

		//
		// Navigate through the tree : tree에서 추출한 학습 데이터의 확률 추정
		//		
		while(true)
		{
			if( treebuf[treeOffs] == -1.0 )
			{					
				if( gRFtype == RF_CLASSIFICATION_TREE_PROB )		// 2016-09-05 JMR
				{
					for( j=0 ; j<theNumberOfClass ; j++ )
					{
						if( treebuf[treeOffs + 1 + j] > maxProb )
						{
							maxProb = treebuf[treeOffs + 1 + j];
							estimatedClassLabel = j;
						}
					}
				}
				else
					estimatedClassLabel = (int)treebuf[treeOffs + 1];
				
				break;
			}
			if( data[RND(treebuf[treeOffs])] < treebuf[treeOffs + 1] )
			{
				treeOffs += innernodewidth;
			}
			else
			{
				treeOffs = RND(treebuf[treeOffs + 2]);
			}
		}				
		
		// 학습 데이터의 class label 가져오기
		classLabel = (int)trainingData[i][featureDemension];

		// 전체 학습 데이터의 data weight 합
		allw += dataWeight[i];

		// 오검출된 학습 데이터의 data weight 합
		if(classLabel != estimatedClassLabel)
		{
			misw += dataWeight[i];
			dataResult[i] = -1;
		}
	}

	// decision tree's error rate
	treeError = misw / allw;

	// decision tree's weight
	if (treeError == 0) df.treesWeight[treeIdx] = 1.5;
	else df.treesWeight[treeIdx] = 0.5 * ( log( ((df.theNumberOfClass-1)*(1-treeError))/treeError ) );
	
	//TRACE(_T("Decision Tree's weight : %lf (E=%lf)\n"), df.treesWeight[treeIdx], treeError);
	// decision tree 사용 여부 결정
	if( df.treesWeight[treeIdx] > 0 )		
	{
		// decision tree 사용
		useDecisionTree = true;		

		// 전체 학습데이터의 data weight 갱신
		for(i = 0; i < theNumberOfTrainingData ; i++)
		{
			if( dataResult[i] == -1 )
				dataWeight[i] = dataWeight[i] * exp(df.treesWeight[treeIdx]);
			else
				dataWeight[i] = dataWeight[i] * exp(df.treesWeight[treeIdx]*-1.0);
		}

	}else
	{
		useDecisionTree = false;		
	}

	delete [] data;	
	delete [] dataResult;

	return useDecisionTree;
}

/**
	@brief		Out-Of-Bag 데이터를 이용하여 현재 생성된 트리의 정확도를 측정
				Out-of-Bag 데이터 : 트리 생성될 때, 사용되지 않은 데이터
	@param		treebuf : decision Tree 구조체
				trainingData : 샘플 feature values + class label 정보를 가짐
				is_InOfBagData : 전체 트레이닝 데이터에서 현재 입력된 트리가 구성될 때 사용된 데이터 체크 배열 (1:사용, 0:미사용)
				theNumberOfTrainingData : 샘플 데이터 개수			
	@return		현재 decisionTree의 Accuracy
*/
double measureQuiltyOfDecisionTree(const RANDOMFORESTS& df, double* treebuf, double** trainingData, int* is_InOfBagData, int theNumberOfTrainingData, int offsetTh)
{
	register int i;//, j; 

	int featureDemension = df.featureDemension;
	
	double* data = new double[featureDemension];					// feature data + offset(x) + offset(y)
	memset(data, 0, sizeof(double) * featureDemension);

	int treeOffs = 1;
	int resultOffset_x, resultOffset_y;
	int gtOffset_x, gtOffset_y;
	double accuracy = 0.0;
	int count = 0;

	for(i = 0; i < theNumberOfTrainingData; i++)
	{
		// 현재 Tree 구성에 사용된 데이터를 제외한 데이터로 Out-of-Bag 구성
		if( is_InOfBagData[i] == 1 )		
			continue;

		count++;

		// 학습 데이터 1개 추출
		moveData(data, 1, trainingData[i], 1, featureDemension);		// feature data 복사

		// 변수 초기화		
		treeOffs = 1;

		//
		// Navigate through the tree : tree에서 추출한 학습 데이터의 확률 추정
		//		
		while(true)
		{
			if( treebuf[treeOffs] == -1.0 )
			{					
				resultOffset_x = (int)treebuf[treeOffs + 1];
				resultOffset_y = (int)treebuf[treeOffs + 2];
				
				break;
			}
			if( data[RND(treebuf[treeOffs])] < treebuf[treeOffs + 1] )
			{
				treeOffs += innernodewidth;
			}
			else
			{
				treeOffs = RND(treebuf[treeOffs + 2]);
			}
		}				
		
		// 학습 데이터의 GT_offset 가져오기
		gtOffset_x = (int)trainingData[i][featureDemension];
		gtOffset_y = (int)trainingData[i][featureDemension+1];

		// |gt - result| 오차 추출
		if( abs(gtOffset_x - resultOffset_x) < offsetTh && abs(gtOffset_y - resultOffset_y) < offsetTh )
			accuracy++;
	}
	
	accuracy = accuracy / count;

	delete [] data;

	return accuracy;
}

/**
	@brief			Out-of-Bag 데이터로 추출한 offset의 오차값을 이용하여 각 트리의 weight값을 계산
					weight총합은 1. 오차가 작을 수록 큰 weight를 가짐
	@param			treeAccuracy : Out-of-Bag 데이터로 추출한 GT-테스트결과간의 offset의 오차값 저장 배열					
					treeSize : 트리 개수
*/
void getWeightUsingTreeAccuracy(const RANDOMFORESTS& df, double* treeAccuracy, int treeSize)
{
	//TRACE(_T("getWeightUsingTreeAccuracy start"));
	register int i;

	int treeidx = 0;

	double sum = 0.0;

	for( i=0 ; i<treeSize ; i++ )
	{
		sum += treeAccuracy[i];
	}

	for( i=0 ; i<treeSize ; i++ )
	{
		df.treesWeight[i] = treeAccuracy[i] / sum;
	}		
	
	//TRACE(_T("end\n"));
}

/**
	@brief			Offset Regressor를 위하여 Leaf Node에 속한 샘플 데이터를 이용하여 대표 offset 추출
					대표 offset은 Lead Node에 속한 샘플 데이터들의 outlier 제거한 후 평균값 사용
	@date			2016-09-05 JMR
	@note			offset_x, offset_y : 현재 leaf node의 결과 offset
*/
/*void getOffsetValuesForRegressionLeafNode(double** sampleData, int featureDemension, int& numprocessed, int idx1, int idx2, RF_INTERNAL_BUFFERS& bufs, double& offset_x, double& offset_y)
{
	int i;

	//jwgim 160201 (avg --> remove outlier and avg)
	int tempsize = (idx2-idx1+1);
	double* sortarray1 = new double[tempsize];
	double* sortarray2 = new double[tempsize];
	double* sortdestarray1 = new double[tempsize];
	double* sortdestarray2 = new double[tempsize];
	memset( sortdestarray1, 0, sizeof(double)*tempsize );
	memset( sortdestarray2, 0, sizeof(double)*tempsize );
	for( i = idx1; i <= idx2; i++ ) {
		sortarray1[i-idx1] = sampleData[bufs.idxbuf[i]][featureDemension];		//x
		sortarray2[i-idx1] = sampleData[bufs.idxbuf[i]][featureDemension+1];	//y			
	}
	//정렬
	sortFastReal( sortarray1, sortdestarray1, tempsize );
	sortFastReal( sortarray2, sortdestarray2, tempsize );
	//outlier 제거하고 중심 데이터만 사용
	if( tempsize < 3 ) {	//데이터가 3개 미만인경우 median값 사용
		offset_x = sortarray1[(int)(tempsize/2)];
		offset_y = sortarray2[(int)(tempsize/2)];
	}
	else {	//데이터 정렬후 상위-하위 데이터 제거
		offset_x = offset_y = 0;
		int tempcount = 0;
		for( i = (tempsize/3); i <= tempsize-(tempsize/3); i++ ) {
			offset_x += sortarray1[i];
			offset_y += sortarray2[i];
			//2016-10-06 offset_x += sampleData[bufs.idxbuf[i]][featureDemension];
			//2016-10-06 offset_y += sampleData[bufs.idxbuf[i]][featureDemension+1];
			tempcount++;
		}
		offset_x /= tempcount;
		offset_y /= tempcount;
		
	}

	// Test 2016-09-21
	int x, y;
	int count;
	TRACE(_T("the values of leaf node (%d): \n"), tempsize);	
	if( tempsize < 3 ) {
		for( i=idx1 ; i<= idx2 ; i++)
		{	
			x = sampleData[bufs.idxbuf[i]][featureDemension];
			y = sampleData[bufs.idxbuf[i]][featureDemension+1];
		
			TRACE(_T("%d,%d,o;\n"), x, y);
		}
	}else
	{
		for( i=0 ; i< tempsize ; i++)
		{		
			x = sortarray1[i];
			y = sortarray2[i];
				
			if( i >= (tempsize/3) && i < tempsize-(tempsize/3) ) 
				TRACE(_T("%d,%d,o;\n"), x, y);
			else
				TRACE(_T("%d,%d,x;\n"), x, y);
		}	
		//for( i=idx1 ; i<= idx2 ; i++)
		//{		
		//	x = sampleData[bufs.idxbuf[i]][featureDemension];
		//	y = sampleData[bufs.idxbuf[i]][featureDemension+1];
		//		
		//	if( i >= (idx1+(tempsize/3)) && i <= (idx2-(tempsize/3)) )
		//		TRACE(_T("%d,%d,o;\n"), x, y);
		//	else
		//		TRACE(_T("%d,%d,x;\n"), x, y);
		//}		
	}
	TRACE(_T("\n"));

	delete [] sortarray1;
	delete [] sortarray2;
	delete [] sortdestarray1;
	delete [] sortdestarray2;
}*/

/**
	@brief			Offset Regressor를 위하여 Leaf Node에 속한 샘플 데이터를 이용하여 대표 offset 추출
					대표 offset은 Lead Node에 속한 샘플 데이터들의 outlier 제거한 후 평균값 사용
					outlier제거를 위해서 Ransac 적용
	@date			2016-09-05 JMR
	@note			offset_x, offset_y : 현재 leaf node의 결과 offset
*/
void getOffsetValuesForRegressionLeafNode(double** sampleData, int featureDemension, int& numprocessed, int idx1, int idx2, RF_INTERNAL_BUFFERS& bufs, double& offset_x, double& offset_y)
{		
	//// 2차원 RANSAC /////////////////////////////////////////////////////////
	//int i;

	////jwgim 160201 (avg --> remove outlier and avg)
	//int no_samples = (idx2-idx1+1);
	//int no_inliers = 0;

	//sPoint *samples = new sPoint[no_samples];
	//sPoint *inliers = new sPoint[no_samples];
	//sLine line;

	//offset_x = offset_y = 0;

	//for( i=0 ; i<no_samples ; i++ )
	//{
	//	samples[i].x = sampleData[bufs.idxbuf[i]][featureDemension];
	//	samples[i].y = sampleData[bufs.idxbuf[i]][featureDemension+1];
	//	inliers[i].x = 0;
	//	inliers[i].y = 0;
	//}
	//
	//// Ransac 적용 및 inlier 추출 : distance_thresold를 어떻게 정할것인가를 고민!!!!!!!!!!
	//no_inliers = ransac_line_fitting(samples, no_samples, inliers, line, 10, (int)(no_samples*0.7));

	//// inlier를 이용한 offset 평균값 추출
	//for( i=0 ; i<no_inliers ; i++ )
	//{
	//	offset_x += inliers[i].x;
	//	offset_y += inliers[i].y;
	//}
	//offset_x /= no_inliers;
	//offset_y /= no_inliers;

	//// Test 2016-09-21
	//int j;
	//int x, y;
	//int count;
	//bool buse;
	//int sx, sy, ex, ey;
	//sx = (int)(line.sx) - (int)(line.mx*20);
	//ex = (int)(line.sx) + (int)(line.mx*20);
	//sy = (int)(line.sy) - (int)(line.my*20);	
	//ey = (int)(line.sy) + (int)(line.my*20);

	//TRACE(_T("the values of leaf node (sample:%d, inliers:%d ): (startp:%d,%d, endp:%d,%d)\n"), no_samples, no_inliers, sx, sy, ex, ey);	

	//for( i=0 ; i<no_samples ; i++ )
	//{
	//	buse = false;
	//	TRACE(_T("%d,%d,"), (int)samples[i].x, (int)samples[i].y);
	//	for(j=0 ; j<no_inliers ; j++ )
	//	{
	//		if( inliers[j].x == samples[i].x && inliers[j].y == samples[i].y )
	//		{
	//			buse = true;
	//			break;
	//		}
	//	}

	//	if( buse )
	//		TRACE(_T("o;\n"));
	//	else
	//		TRACE(_T("x;\n"));
	//}
	//TRACE(_T("\n"));

	//delete [] samples;
	//delete [] inliers;		


	// 1차원 RANSAC /////////////////////////////////////////////////////////
	int i;

	//jwgim 160201 (avg --> remove outlier and avg)
	int no_samples = (idx2-idx1+1);
	int no_inliers_x = 0;
	int no_inliers_y = 0;

	int *samples_x = new int[no_samples];
	int *samples_y = new int[no_samples];
	int *inliers_x = new int[no_samples];
	int *inliers_y = new int[no_samples];
	int result_x, result_y;
	result_x = result_y = 0;
	offset_x = offset_y = 0;
	
	//////////////////////////////
	// x좌표 RANSAC 적용
	//////////////////////////////
	for( i=0 ; i<no_samples ; i++ )
	{
		samples_x[i] = sampleData[bufs.idxbuf[i+idx1]][featureDemension];
		samples_y[i] = sampleData[bufs.idxbuf[i+idx1]][featureDemension+1];
		inliers_x[i] = 0;
		inliers_y[i] = 0;
	}
	
	// Ransac 적용 및 inlier 추출 : distance_thresold를 어떻게 정할것인가를 고민!!!!!!!!!!
	no_inliers_x = ransac_point_fitting(samples_x, no_samples, inliers_x, result_x, ransacDistanceTh, (int)(no_samples*0.8));	//0.6
	no_inliers_y = ransac_point_fitting(samples_y, no_samples, inliers_y, result_y, ransacDistanceTh, (int)(no_samples*0.8));	//0.6
	
	offset_x = (double)result_x;
	offset_y = (double)result_y;

	// Test 2016-09-21
	/*int j;
	int x, y;
	int count;
	bool buse;

	TRACE(_T("the values of leaf node (sample:%d, inliers:(x:%d,y:%d) ): (offset_x:%d, offset_y:%d)\n"), no_samples, no_inliers_x, no_inliers_y, result_x, result_y);	
	
	TRACE(_T("x===\n"));
	for( i=0 ; i<no_samples ; i++ )
	{
		buse = false;
		TRACE(_T("%d,"), samples_x[i]);
		for(j=0 ; j<no_inliers_x ; j++ )
		{
			if( inliers_x[j] == samples_x[i] )
			{
				buse = true;
				break;
			}
		}

		if( buse )
			TRACE(_T("o;\n"));
		else
			TRACE(_T("x;\n"));
	}
	TRACE(_T("\ny===\n"));
	for( i=0 ; i<no_samples ; i++ )
	{
		buse = false;
		TRACE(_T("%d,"), samples_y[i]);
		for(j=0 ; j<no_inliers_y ; j++ )
		{
			if( inliers_y[j] == samples_y[i] )
			{
				buse = true;
				break;
			}
		}

		if( buse )
			TRACE(_T("o;\n"));
		else
			TRACE(_T("x;\n"));
	}
	TRACE(_T("\n\n"));*/

	delete [] samples_x;
	delete [] samples_y;
	delete [] inliers_x;	
	delete [] inliers_y;	
}


// ======== RANSAC ========================================================

/////////////////////////////////
// 2차원 벡터 RANSAC ////////////
/////////////////////////////////
int ransac_line_fitting(sPoint *data, int no_data, sPoint *best_inliers, sLine &best_model, double distance_threshold, int no_inliers_threshold)
{
	const int no_samples = 2;

	if (no_data <= no_samples) {
		for( int i=0 ; i<no_data ; i++ )
		{
			best_inliers[i].x = data[i].x;
			best_inliers[i].y = data[i].y;
		}
		return no_data;
	}

	sPoint *samples = new sPoint[no_samples];

	int no_best_inliers = 0;
	int no_inliers = 0;
	sPoint *inliers = new sPoint[no_data];
		
	

	sLine estimated_model;
	sLine model;
	double max_cost = 0.;
	double min_error = 9999.;

	int max_iteration = (int)(1 + log(1. - 0.99)/log(1. - pow(0.5, no_samples)));

	for (int i = 0; i<max_iteration; i++) {	
		// 1. hypothesis

		// 원본 데이터에서 임의로 N개의 셈플 데이터를 고른다.
		get_samples (samples, no_samples, data, no_data);

		// 이 데이터를 정상적인 데이터로 보고 모델 파라메터를 예측한다.
		compute_model_parameter (samples, no_samples, estimated_model);

		// 2. Verification

		// 원본 데이터가 예측된 모델에 잘 맞는지 검사한다.
		double cost = model_verification (inliers, &no_inliers, estimated_model, data, no_data, distance_threshold);

		// 만일 예측된 모델이 잘 맞는다면, 이 모델에 대한 유효한 데이터로 새로운 모델을 구한다.
		//if (max_cost < cost) {
		if( cost >= no_inliers_threshold ) {
	
			compute_model_parameter (inliers, no_inliers, model);

			double error = model_verification2(model, inliers, no_inliers, distance_threshold);

			if( min_error > error )
			{
				min_error = error;
				best_model.mx = model.mx;
				best_model.my = model.my;
				best_model.sx = model.sx;
				best_model.sy = model.sy;

				no_best_inliers = no_inliers;
				for( int i=0 ; i<no_best_inliers ; i++ )
				{
					best_inliers[i].x = inliers[i].x;
					best_inliers[i].y = inliers[i].y;
				}
			}
		}
	}
															
	delete [] samples;
	delete [] inliers;

	if( no_best_inliers == 0 )
	{
		for( int i=0 ; i<no_data ; i++ )
		{
			best_inliers[i].x = data[i].x;
			best_inliers[i].y = data[i].y;
		}
		no_best_inliers = no_data;
	}

	return no_best_inliers;
}

void get_samples (sPoint *samples, int no_samples, sPoint *data, int no_data)
{
	// 데이터에서 중복되지 않게 N개의 무작위 샘플을 채취한다.
	for (int i=0; i<no_samples; ) {
		int j = rand()%no_data;
		
		if (!find_in_samples(samples, i, &data[j])) {
			samples[i] = data[j];
			++i;
		}
	};
}

int compute_model_parameter(sPoint samples[], int no_samples, sLine &model)
{
	// PCA 방식으로 직선 모델의 파라메터를 예측한다.

	double sx  = 0, sy  = 0;
	double sxx = 0, syy = 0;
	double sxy = 0, sw  = 0;

	for(int i = 0; i<no_samples;++i)
	{
		double &x = samples[i].x;
		double &y = samples[i].y;

		sx  += x;	
		sy  += y;
		sxx += x*x; 
		sxy += x*y;
		syy += y*y;
		sw  += 1;
	}

	//variance;
	double vxx = (sxx - sx*sx/sw)/sw;
	double vxy = (sxy - sx*sy/sw)/sw;
	double vyy = (syy - sy*sy/sw)/sw;
	
	//principal axis
	double theta = atan2(2*vxy, vxx - vyy)/2;
	
	model.mx = cos(theta);
	model.my = sin(theta);
	
	//center of mass(xc, yc)
	model.sx = sx/sw;
	model.sy = sy/sw;
	
	//직선의 방정식: sin(theta)*(x - sx) = cos(theta)*(y - sy);
	return 1;
}

double compute_distance(sLine &line, sPoint &x)
{
	// 한 점(x)로부터 직선(line)에 내린 수선의 길이(distance)를 계산한다.

	return fabs((x.x - line.sx)*line.my - (x.y - line.sy)*line.mx)/sqrt(line.mx*line.mx + line.my*line.my);
}

double model_verification (sPoint *inliers, int *no_inliers, sLine &estimated_model, sPoint *data, int no_data, double distance_threshold)
{
	*no_inliers = 0;

	double cost = 0.;	

	for(int i=0; i<no_data; i++){
		// 직선에 내린 수선의 길이를 계산한다.
		double distance = compute_distance(estimated_model, data[i]);
	
		// 예측된 모델에서 유효한 데이터인 경우, 유효한 데이터 집합에 더한다.
		if (distance < distance_threshold) {
			cost += 1.;			

			inliers[*no_inliers] = data[i];
			++(*no_inliers);
		}
	}

	return cost;
}

double model_verification2 (sLine &model, sPoint *data, int no_data, double distance_threshold)
{
	double error = 0.;

	for(int i=0; i<no_data; i++){
		// 직선에 내린 수선의 길이를 계산한다.
		double distance = compute_distance(model, data[i]);
	
		// 예측된 모델에서 유효한 데이터인 경우, 유효한 데이터 집합에 더한다.
		if (distance < distance_threshold) {
			error += distance;
		}
	}

	error /= no_data;

	return error;
}

bool find_in_samples (sPoint *samples, int no_samples, sPoint *data)
{
	for (int i=0; i<no_samples; ++i) {
		if (samples[i].x == data->x && samples[i].y == data->y) {
			return true;
		}
	}
	return false;
}

/////////////////////////////////
// 1차원 벡터 RANSAC ////////////
/////////////////////////////////
int ransac_point_fitting(int *data, int no_data, int *best_inliers, int &best_model, double distance_threshold, int no_inliers_threshold)
{
	const int no_samples = 2;

	if (no_data <= no_samples) {	
		best_model = 0;
		for( int i=0 ; i<no_data ; i++ )
		{
			best_inliers[i] = data[i];
			best_model += data[i];
		}
		best_model /= no_data;
		return no_data;
	}

	// 현재 데이터의 구성이 모두 동일한 값을 가질 경우
	if( get_allsamevalue(data, no_data) )
	{
		for( int i=0 ; i<no_data ; i++ )
			best_inliers[i] = data[i];
		
		best_model = data[0];
		return no_data;
	}

	int *samples = new int[no_samples];

	int no_best_inliers = 0;
	int no_inliers = 0;
	int *inliers = new int[no_data];	

	int estimated_model;
	int model;
	double max_cost = 0.;
	double min_error = 9999.;

	int max_iteration = (int)(1 + log(1. - 0.99)/log(1. - pow(0.7, no_samples)));  // 0.5->0.6

	for (int i = 0; i<max_iteration; i++) {	
		// 1. hypothesis

		// 원본 데이터에서 임의로 N개의 셈플 데이터를 고른다.
		get_samples (samples, no_samples, data, no_data);

		// 이 데이터를 정상적인 데이터로 보고 모델 파라메터를 예측한다.
		compute_model_parameter (samples, no_samples, estimated_model);

		// 2. Verification

		// 원본 데이터가 예측된 모델에 잘 맞는지 검사한다.
		double cost = model_verification (inliers, &no_inliers, estimated_model, data, no_data, distance_threshold);

		// 만일 예측된 모델이 잘 맞는다면, 이 모델에 대한 유효한 데이터로 새로운 모델을 구한다.
		//if (max_cost < cost) {
		if( cost >= no_inliers_threshold ) {
	
			compute_model_parameter (inliers, no_inliers, model);

			double error = model_verification2(model, inliers, no_inliers, distance_threshold);

			if( min_error > error )
			{
				min_error = error;
				best_model = model;
				
				no_best_inliers = no_inliers;
				for( int i=0 ; i<no_best_inliers ; i++ )
				{
					best_inliers[i] = inliers[i];				
				}
			}
		}
	}
															
	delete [] samples;
	delete [] inliers;
		
	if( distance_threshold > 50 && no_best_inliers == 0 )
	{
		best_model = 0;
		for( int i=0 ; i<no_data ; i++ )
		{
			best_inliers[i] = data[i];
			best_model += data[i];
		}
		no_best_inliers = no_data;
		best_model /= no_data;
	}else if( no_best_inliers == 0 )
	{
		no_best_inliers = ransac_point_fitting(data, no_data, best_inliers, best_model, distance_threshold*1.5, no_inliers_threshold-(no_data*0.1));			
		return no_best_inliers;
	}
	/*if( no_best_inliers == 0 )
	{
		for( int i=0 ; i<no_data ; i++ )
		{
			best_inliers[i] = data[i];
			best_model += data[i];
		}
		no_best_inliers = no_data;
		best_model /= no_data;
	}*/

	return no_best_inliers;
}

bool find_in_samples (int *samples, int no_samples, int data)
{
	for (int i=0; i<no_samples; ++i) {
		if (samples[i] == data ) {
			return true;
		}
	}
	return false;
}

void get_samples (int *samples, int no_samples, int *data, int no_data)
{
	// 데이터에서 중복되지 않게 N개의 무작위 샘플을 채취한다.
	for (int i=0; i<no_samples; ) {
		int j = rand()%no_data;
		
		if (!find_in_samples(samples, i, data[j])) {
			samples[i] = data[j];
			++i;
		}
	};
}

int compute_model_parameter(int samples[], int no_samples, int &model)
{
	// PCA 방식으로 직선 모델의 파라메터를 예측한다.

	double sx  = 0;

	for(int i = 0; i<no_samples;++i)
	{
		int &x = samples[i];

		sx  += x;	
	}

	model = (int)(sx / no_samples);

	return 1;
}

int compute_distance(int &base_point, int &x)
{
	// 두 값사이의 거리
	return  abs(base_point - x);
}

double model_verification (int *inliers, int *no_inliers, int &estimated_model, int *data, int no_data, double distance_threshold)
{
	*no_inliers = 0;

	double cost = 0.;	

	for(int i=0; i<no_data; i++){
		// 직선에 내린 수선의 길이를 계산한다.
		int distance = compute_distance(estimated_model, data[i]);
	
		// 예측된 모델에서 유효한 데이터인 경우, 유효한 데이터 집합에 더한다.
		if (distance < distance_threshold) {
			cost += 1.;			

			inliers[*no_inliers] = data[i];
			++(*no_inliers);
		}
	}

	return cost;
}

double model_verification2 (int &model, int *data, int no_data, double distance_threshold)
{
	double error = 0.;

	for(int i=0; i<no_data; i++){
		// 직선에 내린 수선의 길이를 계산한다.
		int distance = compute_distance(model, data[i]);
	
		// 예측된 모델에서 유효한 데이터인 경우, 유효한 데이터 집합에 더한다.
		if (distance < distance_threshold) {
			error += distance;
		}
	}

	error /= no_data;

	return error;
}

/*double get_distanceThForRansac(int* inliers, int no_inliers)
{
	double var = 0.0;	
	double min_error = 9999.0;
	double error = 0.0;
	int best_model = 0.0;
	int curr_model = 0.0;

	for( int i=0 ; i<no_inliers ; i++ )
	{
		curr_model = inliers[i];
		error = 0.0;

		for( int j=0 ; j<no_inliers ; j++ )
			error += (inliers[j] - curr_model)*(inliers[j] - curr_model);		// mean square error

		if( min_error > error )
		{
			min_error = error;
			best_model = curr_model;
		}
	}

	for( int i=0 ; i<no_inliers ; i++ )
	{
		var += (inliers[i] - best_model)*(inliers[i] - best_model);
	}

	var /= (no_inliers-1);

	// inlier들의 분포가 정규분포를 가진다고 했을 때, distanceTh  2σ=>97.7%, 3σ=>99.9%의 inliner을 포함
	return sqrt(var) * 2;
}*/

/**
	@brief	현재 데이터의 값들이 모두 같은 값을 가질 경우
	@return	true:모두 같은 값 , false:2개 이상의 다른값을 가짐
*/
bool get_allsamevalue(int* data, int no_data)
{
	int i = 0;
	int no_diff = 0;
	int diff_value1 = -9999;
	int diff_value2 = -9999;
	
	int no_diff_values = 0;
	for( i=0 ; i<no_data ; i++ ){
				
		if( data[i] == diff_value1 ) 
			continue;
			
		if( data[i] == diff_value2 ) 	
			continue;
		
		if( diff_value1 != -9999 )
			diff_value1 = data[i];
		else
			diff_value2 = data[i];

		no_diff_values++;

		if( no_diff_values == 2 )			
			return false;
			
	}

	return true;
}