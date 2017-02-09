#include "stdafx.h"
#include "RandomFern.h"

#define _CRTDBG_MAP_ALLOC

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif 


//150213 for memory leak detection
//_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
//_CrtMemDumpAllObjectsSince(0);
//_CrtDumpMemoryLeaks();
///////////////////////////


CRandom_Fern::CRandom_Fern()
{
	valueInit();
}

CRandom_Fern::~CRandom_Fern()
{
	//	valueDestroy();
}


//////////////// public //////////////////////
// Random fern 학습 함수
void CRandom_Fern::generateRandomFern(RANDOMFERNS& rf_classifier, double*** feature, int theNumberOfDataPerClass[],
	int totalFeatureSize, int featureDimension, int theNumberOfClass, int theNumberOfFern, int theNumberOfFunction, double normalizeFactor, double normalizeConstant, int historyLength, bool flag1, bool flag2)
{
	buildFernForTraining(rf_classifier, feature, theNumberOfDataPerClass, totalFeatureSize, featureDimension, theNumberOfClass, theNumberOfFern, theNumberOfFunction, normalizeFactor, normalizeConstant, historyLength, flag1, flag2);
}

// Random fern 분류 함수
void CRandom_Fern::classification(RANDOMFERNS& rf_classifier, double* feature, double* result)
{
	//	int tstart = clock();

	buildFernForClassification(rf_classifier, feature, result);

	//	Trace(_T("펀 생성 : %f sec\n"), (double)(clock() - tstart)/CLOCKS_PER_SEC);
}

// Random fern을 boost 시키는 함수
void CRandom_Fern::boostedClassifier(RANDOMFERNS& rf_classifier, double*** feature, int theNumberOfDataPerClass[],
	int totalFeatureSize, int featureDimension, int theNumberOfClass, int theNumberOfFern, int theNumberOfBoostedFern, int theNumberOfFunction, double normalizeFactor, double normalizeContant, int historyLength, bool flag1, bool flag2)
{
	selectGoodFerns(rf_classifier, feature, theNumberOfDataPerClass, totalFeatureSize, featureDimension, theNumberOfClass, theNumberOfFern, theNumberOfBoostedFern, theNumberOfFunction, normalizeFactor, normalizeContant, historyLength, flag1, flag2);
}

// Random fern 복사
void CRandom_Fern::RFCopyLeftToRight(RANDOMFERNS& rf1, RANDOMFERNS& rf2)
{
	register int i, j, k;

	int theNumberOfClass = rf1.theNumberOfClass;
	int theNumberOfFern = rf1.theNumberOfFern;
	int theNumberOfFunction = rf1.theNumberOfFunction;
	double normalizeFactor = rf1.normalizeFactor;
	double normalizeConstant = rf1.normalizeConstant;

	int szFernHistoBin = (int)pow(2.0, theNumberOfFunction);

	// 메모리 할당이 안되어 있으면 새로 할당
	if (rf2.fern == NULL || rf2.function == NULL)
	{
		rf2.theNumberOfClass = theNumberOfClass;
		rf2.theNumberOfFern = theNumberOfFern;
		rf2.theNumberOfFunction = theNumberOfFunction;
		rf2.normalizeFactor = normalizeFactor;
		rf2.normalizeConstant = normalizeConstant;

		initClasifier(rf2);
	}

	for (i = 0; i < theNumberOfClass; i++)
	{
		for (j = 0; j < theNumberOfFern; j++)
		{
			for (k = 0; k < szFernHistoBin; k++)
			{
				rf2.fern[i].arrHistoFern[j][k] = rf1.fern[i].arrHistoFern[j][k];
			}

		}
	}

	for (i = 0; i < theNumberOfFern; i++)
	{
		for (j = 0; j < theNumberOfFunction; j++)
		{
			rf2.function[i].arrLoc_Bin_1[j] = rf1.function[i].arrLoc_Bin_1[j];
			rf2.function[i].arrLoc_Bin_2[j] = rf1.function[i].arrLoc_Bin_2[j];
		}
	}

	for (i = 0; i < theNumberOfClass; i++)
	{
		for (j = 0; j < theNumberOfFern; j++)
		{
			rf2.sumFernDistribution[i][j] = rf1.sumFernDistribution[i][j];
		}
	}
}
void CRandom_Fern::RFCopyLeftToRight2(RANDOMFERNS& rf1, RANDOMFERNS& rf2)
{
	register int i, j/*, k*/;

	int theNumberOfClass = rf1.theNumberOfClass;
	int theNumberOfFern = rf1.theNumberOfFern;
	int theNumberOfFunction = rf1.theNumberOfFunction;
	double normalizeFactor = 0. /*= RF_TRACKING_NORMALIZE_FACTOR*/;
	double normalizeConstant = 0./*= RF_TRACKING_CONSTANT*/;

	//	int currentFrame = rf1.currentFrame;
	//	int historyLength = rf1.historyLength;

	int szFernHistoBin = (int)pow(2.0, theNumberOfFunction);

	// 메모리 할당이 안되어 있으면 새로 할당
	if (rf2.fern == NULL || rf2.function == NULL)
	{
		rf2.theNumberOfClass = theNumberOfClass;
		rf2.theNumberOfFern = theNumberOfFern;
		rf2.theNumberOfFunction = theNumberOfFunction;
		rf2.normalizeFactor = normalizeFactor;
		rf2.normalizeConstant = normalizeConstant;

		//	rf2.currentFrame = currentFrame;
		//	rf2.historyLength = historyLength;

		initClasifier(rf2);
	}

	for (i = 0; i < theNumberOfFern; i++)
	{
		for (j = 0; j < theNumberOfFunction; j++)
		{
			rf2.function[i].arrLoc_Bin_1[j] = rf1.function[i].arrLoc_Bin_1[j];
			rf2.function[i].arrLoc_Bin_2[j] = rf1.function[i].arrLoc_Bin_2[j];
		}
	}
}

// boosted 랜덤 펀 생성
void CRandom_Fern::generateClassifier(RANDOMFERNS& rf_classifier, int theNumberOfClass, int theNumberOfFern, int theNumberOfBoostedFern, int theNumberOfFunction, double normalizeFactor, double normalizeContant, int historyLength)
{
	// fern과 function을 저장할 임시 fern과 function 생성 ==============================================
	//	RANDOMFERNS temp;

	//		temp.theNumberOfClass = theNumberOfClass;
	//		temp.theNumberOfFern = theNumberOfFern;
	//		temp.theNumberOfFunction = theNumberOfFunction;
	//		temp.normalizeFactor = normalizeFactor;
	//		temp.normalizeConstant = normalizeContant;
	//
	////		temp.historyLength = historyLength;
	////		temp.currentFrame = 0;
	//
	//		initClasifier(temp);
	// =============================================================================================

	// 최적의 fern의 중복 선택을 방지하기 위해 swap에 사용되는 변수 ========================================
	//	//	RANDOMFERNS temp2;
	//
	//		temp2.theNumberOfClass = theNumberOfClass;
	//		temp2.theNumberOfFern = theNumberOfFern;
	//		temp2.theNumberOfFunction = theNumberOfFunction;
	//		temp2.normalizeFactor = normalizeFactor;
	//		temp2.normalizeConstant = normalizeContant;
	//
	////		temp2.historyLength = historyLength;
	////		temp2.currentFrame = 0;
	//
	//		initClasifier(temp2);
	// =============================================================================================

	// 프로그램에서 사용되는 분류기 생성 =================================================================
	rf_classifier.theNumberOfClass = theNumberOfClass;
	rf_classifier.theNumberOfFern = theNumberOfBoostedFern;
	rf_classifier.theNumberOfFunction = theNumberOfFunction;
	rf_classifier.normalizeFactor = normalizeFactor;
	rf_classifier.normalizeConstant = normalizeContant;

	//		rf_classifier.historyLength = historyLength;
	//		rf_classifier.currentFrame = 0;

	initClasifier(rf_classifier);
	// =============================================================================================
}

void CRandom_Fern::generateClassifier(int theNumberOfClass, int theNumberOfFern, int theNumberOfBoostedFern, int theNumberOfFunction, double normalizeFactor, double normalizeContant, int historyLength)
{
	// fern과 function을 저장할 임시 fern과 function 생성 ==============================================
	//	RANDOMFERNS temp;

	temp.theNumberOfClass = theNumberOfClass;
	temp.theNumberOfFern = theNumberOfFern;
	temp.theNumberOfFunction = theNumberOfFunction;
	temp.normalizeFactor = normalizeFactor;
	temp.normalizeConstant = normalizeContant;

	//		temp.historyLength = historyLength;
	//		temp.currentFrame = 0;

	initClasifier(temp);
	// =============================================================================================

	// 최적의 fern의 중복 선택을 방지하기 위해 swap에 사용되는 변수 ========================================
	//	//	RANDOMFERNS temp2;
	//
	//		temp2.theNumberOfClass = theNumberOfClass;
	//		temp2.theNumberOfFern = theNumberOfFern;
	//		temp2.theNumberOfFunction = theNumberOfFunction;
	//		temp2.normalizeFactor = normalizeFactor;
	//		temp2.normalizeConstant = normalizeContant;
	//
	////		temp2.historyLength = historyLength;
	////		temp2.currentFrame = 0;
	//
	//		initClasifier(temp2);
	// =============================================================================================

	// 프로그램에서 사용되는 분류기 생성 =================================================================
	//rf_classifier.theNumberOfClass = theNumberOfClass;
	//rf_classifier.theNumberOfFern = theNumberOfBoostedFern;
	//rf_classifier.theNumberOfFunction = theNumberOfFunction;
	//rf_classifier.normalizeFactor = normalizeFactor;
	//rf_classifier.normalizeConstant = normalizeContant;

	////		rf_classifier.historyLength = historyLength;
	////		rf_classifier.currentFrame = 0;

	//initClasifier(rf_classifier);
	// =============================================================================================
}

// 분류기 할당
void CRandom_Fern::initClasifier(RANDOMFERNS& rf_classifier)
{
	destroyClassifier(rf_classifier);

	int theNumberOfClass = rf_classifier.theNumberOfClass;
	int theNumberOfFern = rf_classifier.theNumberOfFern;
	int theNumberOfFunction = rf_classifier.theNumberOfFunction;

	int fernSize = (int)pow(2.0, theNumberOfFunction);

	rf_classifier.fern = new t_FernInfo[theNumberOfClass];

	for (int i = 0; i < theNumberOfClass; i++)
	{
		rf_classifier.fern[i].arrHistoFern = new double*[theNumberOfFern];

		for (int j = 0; j < theNumberOfFern; j++)
		{
			rf_classifier.fern[i].arrHistoFern[j] = new double[fernSize];
			memset(rf_classifier.fern[i].arrHistoFern[j], 0, sizeof(double)*fernSize);
		}
	}

	rf_classifier.function = new t_FernFunctionInfo[theNumberOfFern];

	for (int i = 0; i < theNumberOfFern; i++)
	{
		rf_classifier.function[i].arrLoc_Bin_1 = new int[theNumberOfFunction];
		rf_classifier.function[i].arrLoc_Bin_2 = new int[theNumberOfFunction];
	}

	rf_classifier.sumFernDistribution = new double*[theNumberOfClass];

	for (int i = 0; i < theNumberOfClass; i++)
	{
		rf_classifier.sumFernDistribution[i] = new double[theNumberOfFern];
		memset(rf_classifier.sumFernDistribution[i], 0, sizeof(double)*theNumberOfFern);
	}

	rf_classifier.selectedBoostedFerns = new int[theNumberOfFern];
	memset(rf_classifier.selectedBoostedFerns, 0, sizeof(int) * theNumberOfFern);


	//=====================================================================================================
	//int historyLength = rf_classifier.historyLength;

	//if(historyLength != 0)
	//{
	//	rf_classifier.historyFern = new t_FernInfo*[historyLength];

	//	for(int k = 0; k < historyLength; k++)
	//	{
	//		rf_classifier.historyFern[k] = new t_FernInfo[theNumberOfClass];	
	//	}

	//	for(int k = 0; k < historyLength; k++)
	//	{
	//		for(int i = 0; i < theNumberOfClass; i++)
	//		{
	//			rf_classifier.historyFern[k][i].arrHistoFern = new double*[theNumberOfFern];		

	//			for(int j = 0; j < theNumberOfFern; j++ )
	//			{
	//				rf_classifier.historyFern[k][i].arrHistoFern[j] = new double[fernSize];
	//				memset( rf_classifier.historyFern[k][i].arrHistoFern[j], 0, sizeof(double)*fernSize );
	//			}
	//		}
	//	}
	//}

	//	rf_classifier.currentFrame = 0;

}
// 분류기 소멸
void CRandom_Fern::destroyClassifier(RANDOMFERNS& rf_classifier)
{
	if (rf_classifier.fern != NULL || rf_classifier.function != NULL)
	{
		int theNumberOfClass = rf_classifier.theNumberOfClass;
		int theNumberOfFern = rf_classifier.theNumberOfFern;

		// boosted Random fern 구조체 소멸
		for (int i = 0; i < theNumberOfClass; i++)
		{
			for (int j = 0; j < theNumberOfFern; j++)
			{
				delete[] rf_classifier.fern[i].arrHistoFern[j];
			}

			delete[] rf_classifier.fern[i].arrHistoFern;
		}

		delete[] rf_classifier.fern;

		rf_classifier.fern = NULL;

		for (int i = 0; i < theNumberOfFern; i++)
		{
			delete[] rf_classifier.function[i].arrLoc_Bin_1;
			delete[] rf_classifier.function[i].arrLoc_Bin_2;
		}

		delete[] rf_classifier.function;

		rf_classifier.function = NULL;

		for (int i = 0; i < theNumberOfClass; i++)
		{
			delete[] rf_classifier.sumFernDistribution[i];
		}
		delete[] rf_classifier.sumFernDistribution;

		rf_classifier.sumFernDistribution = NULL;

		delete[] rf_classifier.selectedBoostedFerns;

		rf_classifier.selectedBoostedFerns = NULL;

		//=====================================================================================================
		//int historyLength = rf_classifier.historyLength;

		//if(historyLength != 0)
		//{
		//	for(int k = 0; k < historyLength; k++)
		//	{
		//		for(int i = 0; i < theNumberOfClass; i++)
		//		{		
		//			for(int j = 0; j < theNumberOfFern; j++ )
		//			{
		//				delete [] rf_classifier.historyFern[k][i].arrHistoFern[j];
		//			}

		//			delete [] rf_classifier.historyFern[k][i].arrHistoFern;
		//		}
		//	}

		//	for(int k = 0; k < historyLength; k++)
		//	{
		//		delete [] rf_classifier.historyFern[k];
		//	}

		//	delete [] rf_classifier.historyFern;

		//	rf_classifier.historyFern = NULL;
		//}
	}
}






//////////////// private //////////////////////

void CRandom_Fern::buildFernForTraining(RANDOMFERNS& rf_classifier, double*** feature, int theNumberOfDataPerClass[],
	int totalFeatureSize, int featureDimension, int theNumberOfClass, int theNumberOfFern, int theNumberOfFunction, double normalizeFactor, double normalizeConstant, int historyLength, bool flag1, bool flag2)
{
	register int i, j, k, l;

	int theNumberOfData = 0;
	double value_1 = 0.0;
	double value_2 = 0.0;
	int sum = 0;

	int binaryValue = 0;

	//double minValue = 65000;
	//double maxValue = -1;

	//double Nmax = 1.0;
	//double Nmin = 0.0;

	// fern 생성
	//	int* arrBinary = new int[theNumberOfFunction];
	//	memset(arrBinary, 0, sizeof(int)*theNumberOfFunction);

	int fernSize = (int)pow(2.0, theNumberOfFunction);

	int historyIndex = 0;

	//	int* vecFern = new int[fernSize];
	//	memset(vecFern, 0, sizeof(int) * fernSize);

	bool randomFlag = true, randBinFlag = true;
	bool directionFlag = true;			// 가우시안 분포 두개 쓸때 사용


	if (flag1)	// tracker가 처음 할당될 때 한 번만 수행됨
	{
		// 프로그램에서 사용되는 분류기 생성 =================================================================
		rf_classifier.theNumberOfClass = theNumberOfClass;
		rf_classifier.theNumberOfFern = theNumberOfFern;
		rf_classifier.theNumberOfFunction = theNumberOfFunction;
		rf_classifier.normalizeFactor = normalizeFactor;
		rf_classifier.normalizeConstant = normalizeConstant;

		//		rf_classifier.historyLength = historyLength;
		//		rf_classifier.currentFrame = 0;

		initClasifier(rf_classifier);
		// =============================================================================================		
	}

	if (flag2)
	{
		int bin_1 = 0;
		int bin_2 = 0;

		for (k = 0; k < theNumberOfClass; k++)			// class 수
		{
			for (i = 0; i < theNumberOfFern; i++)	// fern 수
			{
				memset(rf_classifier.fern[k].arrHistoFern[i], 0, sizeof(int) * fernSize);
			}
		}

		//if(historyLength != 0)
		//{
		//	for(k = 0; k < historyLength; k++)
		//	{
		//		for( i = 0; i < theNumberOfClass; i++ )			// class 수
		//		{
		//			for( j = 0; j < theNumberOfFern; j++ )	// fern 수
		//			{
		//				memset(rf_classifier.historyFern[k][i].arrHistoFern[j], 0, sizeof(int) * fernSize);	
		//			}
		//		}
		//	}
		//}

		// 난수 발생
		srand(GetTickCount());

		for (i = 0; i < theNumberOfFern; i++)
		{
			for (j = 0; j < theNumberOfFunction; j++)
			{
				// random 변수 생성 ( 일반적으로 전체 영상에서 랜덤하게 생성 )
				decideRandomVariable(bin_1, bin_2, totalFeatureSize, featureDimension);

				rf_classifier.function[i].arrLoc_Bin_1[j] = bin_1;
				rf_classifier.function[i].arrLoc_Bin_2[j] = bin_2;
			}
		}

		// =============================================================================================

		//		rf_classifier.currentFrame = 0;

		//for( k = 0; k < theNumberOfClass; k++ )			// class 수
		//{
		//	for( i = 0; i < theNumberOfFern; i++ )	// fern 수
		//	{
		//		memset(rf_classifier.fern[k].arrHistoFern[i], 0, sizeof(int) * fernSize);	
		//	}
		//}

		//for(k = 0; k < historyLength; k++)
		//{
		//	for( i = 0; i < theNumberOfClass; i++ )			// class 수
		//	{
		//		for( j = 0; j < theNumberOfFern; j++ )	// fern 수
		//		{
		//			memset(rf_classifier.historyFern[k][i].arrHistoFern[j], 0, sizeof(int) * fernSize);	
		//		}
		//	}
		//}
	}

	// fern 생성
	for (k = 0; k < theNumberOfClass; k++)			// class 수
	{
		memset(rf_classifier.sumFernDistribution[k], 0, sizeof(double) * theNumberOfFern);

		for (i = 0; i < theNumberOfFern; i++)	// fern 수
		{
			theNumberOfData = theNumberOfDataPerClass[k];

			//	memset(vecFern, 0, sizeof(int) * fernSize);
			memset(rf_classifier.fern[k].arrHistoFern[i], 0, sizeof(double) * fernSize);

			//if(historyLength != 0)
			//{
			//	historyIndex = rf_classifier.currentFrame % rf_classifier.historyLength;

			//	if(rf_classifier.currentFrame >= historyLength)
			//	{
			//		for(l = 0; l < fernSize; l++)
			//		{
			//			rf_classifier.fern[k].arrHistoFern[i][l] -= rf_classifier.historyFern[historyIndex][k].arrHistoFern[i][l];
			//		}
			//	}

			//	memset(rf_classifier.historyFern[historyIndex][k].arrHistoFern[i], 0, sizeof(double) * fernSize);
			//}


			for (l = 0; l < theNumberOfData; l++)	// image 수
			{
				//		memset(arrBinary, 0, sizeof(int)*theNumberOfFunction);
				binaryValue = 0;

				for (j = 0; j < theNumberOfFunction; j++)	// fern size
				{
					// binary code 생성
					//value_1 = feature[k][l][rf_classifier.function[i].arrLoc_Bin_1[j]];
					//value_2 = feature[k][l][rf_classifier.function[i].arrLoc_Bin_2[j]];

					//// 두 픽셀 간의 비교
					//if( value_1 > value_2 )
					//{
					//	arrBinary[j] = 1;
					//}
					//else
					//{
					//	arrBinary[j] = 0;
					//}

					//		if(feature[k][l][rf_classifier.function[i].arrLoc_Bin_1[j]] >= feature[k][l][rf_classifier.function[i].arrLoc_Bin_2[j]])
					if (feature[k][l][rf_classifier.function[i].arrLoc_Bin_1[j]] - feature[k][l][rf_classifier.function[i].arrLoc_Bin_2[j]] >= FUCNTION_THRESHOLD)
					{
						binaryValue |= (int)pow(2.0, j);
					}
				}	// fern size end

				// binary code To decimal number				
				//		sum = convertBinaryToDecimal(arrBinary, theNumberOfFunction);
				//		vecFern[sum]++;


				rf_classifier.fern[k].arrHistoFern[i][binaryValue]++;

				//if(historyLength != 0)
				//{
				//	rf_classifier.historyFern[historyIndex][k].arrHistoFern[i][binaryValue / NORMALIZE_CONSTANT]++;
				//}

			}	// image 수 end			

			//for(l = 0; l < fernSize; l++)
			//{
			//	rf_classifier.fern[k].arrHistoFern[i][l] += vecFern[l];
			//}
			for (l = 0; l < fernSize; l++)
			{
				// 최종 확률값 추정 시 정규화에 사용하기 위하여 fern distribution 계산 후 저장
				rf_classifier.sumFernDistribution[k][i] += (rf_classifier.fern[k].arrHistoFern[i][l] + (normalizeFactor / normalizeConstant));
			}

		}// fern 수 end		
	}// class 수 end

	//	rf_classifier.currentFrame++;


	// 배열 소멸
	//	delete [] arrBinary;
	//	delete [] vecFern;
}

void CRandom_Fern::decideRandomVariable(int& loc_Bin_1, int& loc_Bin_2, int totalFeatureSize, int featureDimension)
{
	register int i;

	int size = totalFeatureSize / featureDimension;
	int sel_index = 0;
	int tmp = 0;

	int loc_grid_1 = 0;
	int loc_grid_2 = 0;

	int* index = new int[size];

	for (i = 0; i < size; i++)
	{
		index[i] = i;
	}

	// 랜덤한 gird 선택
	// 첫 번째 그리드 선택
	sel_index = rand() % size;

	// 첫 번째로 선택된 그리드 저장
	loc_grid_1 = index[sel_index];

	// 선택된 그리드를 가장 마지막 인덱스의 그리드와 교환
	// 한 번 선택된 그리드는 사용하지 않음
	tmp = index[sel_index];
	index[sel_index] = index[size - 1];
	index[size - 1] = tmp;

	// 두 번째 그리드 선택
	sel_index = rand() % (size - 1);

	// 두 번째로 선택된 그리드 저장
	loc_grid_2 = index[sel_index];

	// 랜덤한 bin 위치
	loc_Bin_1 = (loc_grid_1 * featureDimension) + (rand() % featureDimension);
	loc_Bin_2 = (loc_grid_2 * featureDimension) + (rand() % featureDimension);


	delete[] index;




	//register int i;

	//int size = totalFeatureSize;

	//int sel_index = 0;
	//int tmp = 0;

	//int* index = new int[size];
	//
	//for(i = 0; i < size; i++)
	//{
	//	index[i] = i;
	//}

	//sel_index = rand() % size;

	//loc_Bin_1 = index[sel_index];

	//tmp = index[sel_index];
	//index[sel_index] = index[size - 1];
	//index[size - 1] = tmp;

	//sel_index = rand() % (size - 1);

	//loc_Bin_2 = index[sel_index];

	//delete [] index;
}

//int CRandom_Fern::convertBinaryToDecimal(int* arrBinary, int theNumberOfFunction)
//{
//	// binary code To decimal number
//	int decimalNumber = 0;
//	int sizeIndex = theNumberOfFunction-1;
//	int m = sizeIndex;
//
//	while( m >= 0 )
//	{
//		decimalNumber += (int)arrBinary[sizeIndex-m] * (int)pow(2.0, sizeIndex-m );
//		m--;
//	}
//
//	return decimalNumber;
//}

void CRandom_Fern::buildFernForClassification(RANDOMFERNS& rf_classifier, double* feature, double* result)
{
	register int i, j, k;

	int theNumberOfClass = rf_classifier.theNumberOfClass;
	int theNumberOfFern = rf_classifier.theNumberOfFern;
	int theNumberOfFunction = rf_classifier.theNumberOfFunction;
	double normalizeFactor = rf_classifier.normalizeFactor;
	double normalizeConstant = rf_classifier.normalizeConstant;

	double pro_priorValue = 1.0 / (double)theNumberOfClass;

	// fern 생성
	//int sum = 0;

	//int loc_Bin_1 = 0;
	//int loc_Bin_2 = 0;

	//double value_1 = 0.0;
	//double value_2 = 0.0;


	//	int* arrBinary = new int[theNumberOfFunction];
	//	memset(arrBinary, 0, sizeof(int)*theNumberOfFunction);

	memset(result, 0, sizeof(double) * theNumberOfClass);

	int binaryValue = 0;

	// fern 생성		
	for (i = 0; i < theNumberOfFern; i++)	// fern 수
	{
		//	if(rf_classifier.selectedBoostedFerns[i] == 1)
		{
			// binary code 생성
			//		memset(arrBinary, 0, sizeof(int)*theNumberOfFunction);
			binaryValue = 0;

			for (j = 0; j < theNumberOfFunction; j++)	// fern size
			{
				//loc_Bin_1 = rf_function[i].arrLoc_Bin_1[j];
				//loc_Bin_2 = rf_function[i].arrLoc_Bin_2[j];

				// binary code 생성
				//value_1 = feature[loc_Bin_1];
				//value_2 = feature[loc_Bin_2];

				// 두 픽셀 간의 비교
				//if( value_1 > value_2 )
				//{
				//	arrBinary[j] = 1;
				//}
				//else
				//{
				//	arrBinary[j] = 0;
				//}


				//	if(feature[rf_classifier.function[i].arrLoc_Bin_1[j]] >= feature[rf_classifier.function[i].arrLoc_Bin_2[j]])
				if (feature[rf_classifier.function[i].arrLoc_Bin_1[j]] - feature[rf_classifier.function[i].arrLoc_Bin_2[j]] >= FUCNTION_THRESHOLD)
				{
					binaryValue |= (int)pow(2.0, j);
				}

			}// fern size end

			// binary code To decimal number
			//		fernValue[i] = convertBinaryToDecimal(arrBinary, theNumberOfFunction);
			//		fernValue[i] = binaryValue;

			//	result[0] = ((rf_classifier.fern[0].arrHistoFern[i][binaryValue] + 1.0) / rf_classifier.sumFernDistribution[0][i] * 1.0);
			//	result[1] = ((rf_classifier.fern[1].arrHistoFern[i][binaryValue] + 1.0) / rf_classifier.sumFernDistribution[1][i] * 1.0);
			for (k = 0; k < theNumberOfClass; k++)
			{
				result[k] += log((rf_classifier.fern[k].arrHistoFern[i][binaryValue / NORMALIZE_CONSTANT] + (normalizeFactor / normalizeConstant)) / rf_classifier.sumFernDistribution[k][i]);
				//	result[k] += log(((rf_classifier.fern[k].arrHistoFern[i][binaryValue / NORMALIZE_CONSTANT] + (normalizeFactor / normalizeConstant)) / rf_classifier.sumFernDistribution[k][i]) + 1.0);

			}
		}

	}// fern 수 end

	for (i = 0; i < theNumberOfClass; i++)
	{
		result[i] *= pro_priorValue;
	}

	//	delete arrBinary;
}

void CRandom_Fern::classifyUsingRandomFern(t_FernInfo* rf_fern, double** sumFernDistribution, int fernValue[], double* result, int theNumberOfClass, int theNumberOfFern, int theNumberOfFunction)
{
	register int i, j/*, k*/;

	//	int testFernNumber = 0;
	//	int fernSize = (int)pow(2.0, theNumberOfFunction);

	//	double sumClassHisto = 0.0;

	//double** pro_classFernHisto = new double*[theNumberOfClass];
	//for(i = 0; i < theNumberOfClass; i++)
	//{
	//	pro_classFernHisto[i] = new double[theNumberOfFern];
	//	memset(pro_classFernHisto[i], 0, sizeof(double) * theNumberOfFern);
	//}

	//	double* int_TRNclassFernHisto = new double[theNumberOfClass];
	//	memset(int_TRNclassFernHisto, 0, sizeof(double) * theNumberOfClass);

	//double** total_classHist = new double*[theNumberOfClass];
	//for(i = 0; i < theNumberOfClass; i++)
	//{
	//	total_classHist[i] = new double[theNumberOfFern];
	//	memset(total_classHist[i], 0, sizeof(double) * theNumberOfFern);
	//}

	double log_eachFern = 0.0;
	//	double calcLog = 0.0;
	//	double pro_priorValue = 0.0;

	//for( i = 0; i < theNumberOfFern; i++ )
	//{
	//	// 각 fern마다 정규화
	//	for( j = 0; j < theNumberOfClass; j++ )
	//	{
	//		testFernNumber = (int)feature[i];

	//		int_TRNclassFernHisto[j] = rf_fern[j].arrHistoFern[i][testFernNumber];
	//		
	//		total_classHisto += int_TRNclassFernHisto[j];
	//	}

	//	for( j = 0; j < theNumberOfClass; j++ )
	//	{
	//		pro_classFernHisto[i][j] = ( ( (double)int_TRNclassFernHisto[j] + 1.0 ) / ( (double)total_classHisto + 1.0 ) );
	//	}		
	//}

	double pro_priorValue = (double)1 / (double)theNumberOfClass;

	for (i = 0; i < theNumberOfClass; i++)
	{
		log_eachFern = 0.0;

		for (j = 0; j < theNumberOfFern; j++)
		{
			//		testFernNumber = fernValue[j];
			//		sumClassHisto = 0.0;

			//		for(k = 0; k < fernSize; k++)
			//		{
			////			total_classHist[i][j] += (rf_fern[i].arrHistoFern[j][k] + 1.0);
			//			sumClassHisto += (rf_fern[i].arrHistoFern[j][k] + 1.0);
			//		}

			//		log_eachFern += log((rf_fern[i].arrHistoFern[j][testFernNumber] + 1.0) / total_classHist[i][j]);
			//		log_eachFern += log((rf_fern[i].arrHistoFern[j][fernValue[j]] + 1.0) / sumClassHisto);
			log_eachFern += log((rf_fern[i].arrHistoFern[j][fernValue[j]] + 1.0) / sumFernDistribution[i][j]);
		}

		result[i] = log_eachFern * pro_priorValue;
	}



	//for(i = 0; i < theNumberOfClass; i++)
	//{
	//	for(j = 0; j < theNumberOfFern; j++)
	//	{
	//		pro_classFernHisto[i][j] = (pro_classFernHisto[i][j] + 1.0) / total_classHist[i][j];
	//	}
	//}


	//// 각 class끼리 log 취한 후 sum
	//for( i = 0; i < theNumberOfClass; i++ )
	//{
	//	log_eachFern = 0.0;

	//	for( j = 0; j < theNumberOfFern; j++ )
	//	{
	//		// 각 fern의 요소에 log 취한다음 sum 하는 부분
	//		log_eachFern += log(pro_classFernHisto[i][j]);
	//	}

	//	result[i] = log_eachFern;
	//}

	// 나온 결과 확률값을 가지고 사전 확률값을 product	

	//pro_priorValue = (double)1 / (double)theNumberOfClass;

	//for( i = 0; i < theNumberOfClass; i++ )
	//{
	//	result[i] *= pro_priorValue;
	//}

	//for(i = 0; i < theNumberOfClass; i++)
	//{
	//	delete [] pro_classFernHisto[i];
	//	delete [] total_classHist[i];
	//}

	//delete [] pro_classFernHisto;
	//delete [] total_classHist;
}

// boosted algorithm
void CRandom_Fern::selectGoodFerns(RANDOMFERNS& rf_classifier, double*** feature, int theNumberOfDataPerClass[],
	int totalFeatureSize, int featureDimension, int theNumberOfClass, int theNumberOfFern, int theNumberOfBoostedFern, int theNumberOfFunction, double normalizeFactor, double normalizeContant, int historyLength, bool flag1, bool flag2)
{
	register int i, j, i1, j1, k;

	// class 당 data 수
	int theNumberOfData = 0;

	// 모든 class data 수
	int theNumberOfAllData = 0;

	// sample weight 배열의 증감자
	int theIndexOfSampleData = 0;

	double value_1 = 0.0;
	double value_2 = 0.0;
	int sum = 0;

	int binaryValue = 0;

	// 확률값을 저장하는 배열
	double* result = new double[theNumberOfClass];
	memset(result, 0, sizeof(double)*theNumberOfClass);

	// fern 크기
	int fernSize = (int)pow(2.0, theNumberOfFunction);

#pragma region Random ferns 메모리 할당 ==============================================================
	if (flag1)
	{
		//	// fern과 function을 저장할 임시 fern과 function 생성 ==============================================
		////	RANDOMFERNS temp;

		//	temp.theNumberOfClass = theNumberOfClass;
		//	temp.theNumberOfFern = theNumberOfFern;
		//	temp.theNumberOfFunction = theNumberOfFunction;
		//	temp.normalizeFactor = normalizeFactor;
		//	temp.normalizeConstant = normalizeContant;

		//	temp.historyLength = historyLength;
		//	temp.currentFrame = 0;

		//	initClasifier(temp);
		//	// =============================================================================================

		//	// 최적의 fern의 중복 선택을 방지하기 위해 swap에 사용되는 변수 ========================================
		////	RANDOMFERNS temp2;

		//	temp2.theNumberOfClass = theNumberOfClass;
		//	temp2.theNumberOfFern = theNumberOfFern;
		//	temp2.theNumberOfFunction = theNumberOfFunction;
		//	temp2.normalizeFactor = normalizeFactor;
		//	temp2.normalizeConstant = normalizeContant;

		//	temp2.historyLength = historyLength;
		//	temp2.currentFrame = 0;

		//	initClasifier(temp2);
		//	// =============================================================================================
		//	
		//	// 프로그램에서 사용되는 분류기 생성 =================================================================
		//	rf_classifier.theNumberOfClass = theNumberOfClass;
		//	rf_classifier.theNumberOfFern = theNumberOfBoostedFern;
		//	rf_classifier.theNumberOfFunction = theNumberOfFunction;
		//	rf_classifier.normalizeFactor = normalizeFactor;
		//	rf_classifier.normalizeConstant = normalizeContant;

		//	rf_classifier.historyLength = historyLength;
		//	rf_classifier.currentFrame = 0;

		//	initClasifier(rf_classifier);
		//	// =============================================================================================

		generateClassifier(rf_classifier, theNumberOfClass, theNumberOfFern, theNumberOfBoostedFern, theNumberOfFunction, normalizeFactor, normalizeContant, historyLength);
		generateClassifier(theNumberOfClass, theNumberOfFern, theNumberOfBoostedFern, theNumberOfFunction, normalizeFactor, normalizeContant, historyLength);
	}
#pragma endregion

	// bhattacharyya 거리가 저장된 배열
	double* distance = new double[theNumberOfFern];
	memset(distance, 0, sizeof(double)*theNumberOfFern);

	// bhattacharyya 계수
	double bhattacharyya_coefficient = 0.0;

	double bhattacharyya_sum = 0.0;

	// bhattacharyya 거리를 이용한 최적의 인덱스를 찾기 위한 변수
	int optimalIndex = 0;
	double optimalValue = 1000.0;

	// 최적의 fern 표시
	int* optimalFern = new int[theNumberOfFern];
	memset(optimalFern, 0, sizeof(int)*theNumberOfFern);

	// random fern 생성에 필요한 변수 선언
	//	int* arrBinary = new int[theNumberOfFunction];
	//	memset(arrBinary, 0, sizeof(int)*theNumberOfFunction);	

	//	double* vecFern = new double[fernSize];
	//	memset(vecFern, 0, sizeof(double) * fernSize);

	// 전체 data 수
	for (i = 0; i < theNumberOfClass; i++)
	{
		theNumberOfAllData += theNumberOfDataPerClass[i];
	}

	// sample weight
	// coloumn : data 수
	double* weight_D = new double[theNumberOfAllData];

	// 초기 sample weight 계산
	for (i = 0; i < theNumberOfAllData; i++)
	{
		weight_D[i] = 1.0 / (double)theNumberOfAllData;

		//	weight_D[i] =  1.0;
	}

	const double initWeight_D = 1.0 / (double)theNumberOfAllData;

	//	const double initWeight_D = 1.0;


#pragma region function 생성	
	if (flag2)
	{
		// function 생성
		// =========================================================================================
		int bin_1 = 0;
		int bin_2 = 0;

		// class 수
		//for( k = 0; k < theNumberOfClass; k++ )
		//{
		//	// fern 수
		//	for( i = 0; i < theNumberOfFern; i++ )
		//	{
		//		memset(temp.fern[k].arrHistoFern[i], 0, sizeof(double) * fernSize);	
		//	}
		//}

		// 난수 발생
		//	srand(GetTickCount());

		for (i = 0; i < theNumberOfFern; i++)
		{
			for (j = 0; j < theNumberOfFunction; j++)
			{
				// random 변수 생성 ( 일반적으로 전체 영상에서 랜덤하게 생성 )
				decideRandomVariable(bin_1, bin_2, totalFeatureSize, featureDimension);

				temp.function[i].arrLoc_Bin_1[j] = bin_1;
				temp.function[i].arrLoc_Bin_2[j] = bin_2;
			}
		}
		// =========================================================================================
	}
#pragma endregion

	memset(rf_classifier.selectedBoostedFerns, 0, sizeof(int) * theNumberOfBoostedFern);

	//for(int i = 0; i < theNumberOfClass; i++)
	//{	
	//	for(int j = 0; j < theNumberOfFern; j++ )
	//	{
	//		memset( temp.fern[i].arrHistoFern[j], 0, sizeof(double)*fernSize );
	//	}
	//}

	// fern 생성 및 최적의 fern 선택
	// 선택하려는 fern의 수만큼 반복
	for (k = 0; k < theNumberOfBoostedFern; k++)
	{
		// 처음 생성된 fern을 모두 순회하면서 bhattacharyya 거리 계산
		// 최소가 되는 fern을 선택

		for (i = 0; i < theNumberOfFern; i++)
		{
			// 최적의 fern은 계산에서 제외
			if (optimalFern[i] == 0)
			{
#pragma region 각 fern의 각 class에 대한 분포 생성
				for (j = 0; j < theNumberOfClass; j++)
				{
					// 해당 class의 학습 데이터 수
					theNumberOfData = theNumberOfDataPerClass[j];

					memset(temp.fern[j].arrHistoFern[i], 0, sizeof(double) * fernSize);

					// 학습 데이터를 모두 사용하여 fern의 분포 생성
					for (i1 = 0; i1 < theNumberOfData; i1++)
					{
						binaryValue = 0;

						// function 수 만큼 반복
						for (j1 = 0; j1 < theNumberOfFunction; j1++)
						{
							// binary code 생성
							if (feature[j][i1][temp.function[i].arrLoc_Bin_1[j1]] >= feature[j][i1][temp.function[i].arrLoc_Bin_2[j1]])
							{
								binaryValue |= (int)pow(2.0, j1);
							}

						} // end j1

						// 분포에 sample weight를 누적 시킴
						temp.fern[j].arrHistoFern[i][binaryValue] += weight_D[theIndexOfSampleData++];

					} // end i1
				} // end j


#pragma endregion

#pragma region bhattacharyya 거리 계산

				bhattacharyya_coefficient = 0.0;

				double denom_h0 = 0.0;
				double denom_h1 = 0.0;

				for (j = 0; j < fernSize; j++)
				{
					denom_h0 += temp.fern[0].arrHistoFern[i][j]; // posi -> particle?
					denom_h1 += temp.fern[1].arrHistoFern[i][j]; // nega -> template?
				}

				for (j = 0; j < fernSize; j++)
				{
					//	bhattacharyya_coefficient += sqrt(temp.fern[0].arrHistoFern[i][j] * temp.fern[1].arrHistoFern[i][j]);
					bhattacharyya_coefficient += (sqrt(temp.fern[0].arrHistoFern[i][j] * temp.fern[1].arrHistoFern[i][j]) / sqrt(denom_h0 * denom_h1));
				} // end j

				// 거리 비교를 위해 배열에 저장
				distance[i] = sqrt(1.0 - bhattacharyya_coefficient);

				//for(j = 0; j < fernSize; j++)
				//{
				//	bhattacharyya_coefficient += sqrt((temp.fern[0].arrHistoFern[i][j] - temp.fern[1].arrHistoFern[i][j]) * (temp.fern[0].arrHistoFern[i][j] - temp.fern[1].arrHistoFern[i][j])) ;
				//} // end j

				//// 거리 비교를 위해 배열에 저장
				//distance[i] = bhattacharyya_coefficient;

				//			Trace(_T("%f\n"), distance[i]);

				// 변수 초기화
				bhattacharyya_sum = 0.0;
				bhattacharyya_coefficient = 0.0;

				theIndexOfSampleData = 0;
#pragma endregion
			}
		} // end i

		//for (int z = 0; z < 60; z++)
		//{
		//	Trace(_T("%f\n"), distance[z]);
		//}

#pragma region 최적의 fern과 function 추정
		optimalValue = -1.0;
		// 최적의 fern 추정
		for (i = 0; i < theNumberOfFern; i++)
		{
			if (optimalFern[i] == 0)
			{
				if (optimalValue < distance[i])
				{
					optimalIndex = i;
					optimalValue = distance[i];
				}
			}
		} // end i

		memset(distance, 0, sizeof(double) * theNumberOfFern);

		//	if(optimalValue > 0.5)
		{

			rf_classifier.selectedBoostedFerns[k] = 1;

			// 최적의 fern 저장
			for (i = 0; i < theNumberOfClass; i++)
			{
				for (j = 0; j < fernSize; j++)
				{
					rf_classifier.fern[i].arrHistoFern[k][j] = temp.fern[i].arrHistoFern[optimalIndex][j];

					// 최종 확률값 추정 시 정규화에 사용하기 위하여 fern distribution 계산 후 저장
					rf_classifier.sumFernDistribution[i][k] += (rf_classifier.fern[i].arrHistoFern[k][j] + (normalizeFactor / normalizeContant));
				}
			}// end i

			// 최적의 function 저장
			for (i = 0; i < theNumberOfFunction; i++)
			{
				rf_classifier.function[k].arrLoc_Bin_1[i] = temp.function[optimalIndex].arrLoc_Bin_1[i];
				rf_classifier.function[k].arrLoc_Bin_2[i] = temp.function[optimalIndex].arrLoc_Bin_2[i];
			}

			// 최적의 fern의 인덱스 저장
			optimalFern[optimalIndex] = 1;

			// 최적의 fern의 중복 선택 방지를 위해 최적의 fern을 계산에서 제외
			//swap(&temp, optimalIndex, k, theNumberOfClass, theNumberOfFunction);
#pragma endregion

#pragma region sample weight update
			// 각 data에 대한 weight update
			double weakClassifier = 0.0;

			// sample weight 식의 분자
			double numerator = 0.0;

			// sample weight 식의 분모
			double denominator = 0.0;

			// class index
			int classIndex = 0;

			double calResult = 0.0;

			theIndexOfSampleData = 0;

			// sample weight의 분모 계산
			for (i = 0; i < theNumberOfClass; i++)
			{
				for (j = 0; j < theNumberOfDataPerClass[i]; j++)
				{
					memset(result, 0, sizeof(double) * theNumberOfClass);

					calculateSampleWeight(rf_classifier, feature[i][j], result, theNumberOfFunction, k, normalizeFactor, normalizeContant);

					if (result[0] == 0.0 || result[1] == 0.0)
					{
						weakClassifier = 0.0;
					}
					else
					{
						weakClassifier = 0.5 * log(result[0] / result[1]);
					}

					if (i == 0)
					{
						classIndex = 1;
					}
					else
					{
						classIndex = -1;
					}

					calResult = (weight_D[theIndexOfSampleData] * exp(-1.0 * classIndex * weakClassifier));

					//			denominator += log(weight_D[theIndexOfSampleData++] * exp(-1.0 * classIndex * weakClassifier));
					//			denominator += (weight_D[theIndexOfSampleData++] * exp(-1.0 * classIndex * weakClassifier));

					denominator += calResult;

					weight_D[theIndexOfSampleData] = calResult;

					theIndexOfSampleData++;
				}
			}

			theIndexOfSampleData = 0;

			// sample weight update
			for (i = 0; i < theNumberOfAllData; i++)
			{
				weight_D[i] /= denominator;
			}

			// sample weight update
			//	for(i = 0; i < theNumberOfClass; i++)
			//	{
			//		for(j = 0; j < theNumberOfDataPerClass[i]; j++)
			//		{
			//			memset(result, 0, sizeof(double) * theNumberOfClass);

			//			calculateSampleWeight(rf_classifier, feature[i][j], result, theNumberOfFunction, k, normalizeFactor, normalizeContant);

			//			if(result[0] == 0.0 || result[1] == 0.0)
			//			{
			//				weakClassifier = 0.0;
			//			}
			//			else
			//			{
			//				weakClassifier = 0.5 * log(result[0] / result[1]);
			//			}

			//			if(i == 0)
			//			{
			//				classIndex = 1;
			//			}
			//			else
			//			{
			//				classIndex = -1;
			//			}

			////			numerator = log(weight_D[theIndexOfSampleData] * exp(-1.0 * classIndex * weakClassifier));
			//			numerator = (weight_D[theIndexOfSampleData] * exp(-1.0 * classIndex * weakClassifier));

			//			weight_D[theIndexOfSampleData++] = numerator / denominator;
			//		}
			//	}

			theIndexOfSampleData = 0;
#pragma endregion
		}

	} // end k



#pragma region 배열 소멸
	//	destroyClassifier(temp);
	//	destroyClassifier(temp2);

	delete[] result;
	delete[] distance;
	delete[] optimalFern;
	//	delete [] arrBinary;
	//	delete [] vecFern;

	delete[] weight_D;
#pragma endregion
}

// 최적의 fern 중복 선택 방지를 위한 swap
void CRandom_Fern::swap(RANDOMFERNS* temp1, int optimalIndex, int changeIndex, int theNumberOfClass, int theNumberOfFunction)
{
	register int i, j;

	// fern 크기
	int fernSize = (int)pow(2.0, theNumberOfFunction);

	double** temp_histoFern = new double*[theNumberOfClass];
	for (i = 0; i < theNumberOfClass; i++)
	{
		temp_histoFern[i] = new double[fernSize];
		memset(temp_histoFern[i], 0, sizeof(double) * fernSize);
	}

	int* temp_loc_Bin_1 = new int[theNumberOfFunction];
	int* temp_loc_Bin_2 = new int[theNumberOfFunction];

	memset(temp_loc_Bin_1, 0, sizeof(int) * theNumberOfFunction);
	memset(temp_loc_Bin_2, 0, sizeof(int) * theNumberOfFunction);


	// step 1
	for (i = 0; i < theNumberOfClass; i++)
	{
		for (j = 0; j < fernSize; j++)
		{
			temp_histoFern[i][j] = temp1->fern[i].arrHistoFern[optimalIndex][j];
		}
	}

	for (i = 0; i < theNumberOfFunction; i++)
	{
		temp_loc_Bin_1[i] = temp1->function[optimalIndex].arrLoc_Bin_1[i];
		temp_loc_Bin_2[i] = temp1->function[optimalIndex].arrLoc_Bin_2[i];
	}

	// step 2
	for (i = 0; i < theNumberOfClass; i++)
	{
		for (j = 0; j < fernSize; j++)
		{
			temp1->fern[i].arrHistoFern[optimalIndex][j] = temp1->fern[i].arrHistoFern[changeIndex][j];
		}
	}

	for (i = 0; i < theNumberOfFunction; i++)
	{
		temp1->function[optimalIndex].arrLoc_Bin_1[i] = temp1->function[changeIndex].arrLoc_Bin_1[i];
		temp1->function[optimalIndex].arrLoc_Bin_2[i] = temp1->function[changeIndex].arrLoc_Bin_2[i];
	}

	//step 3
	for (i = 0; i < theNumberOfClass; i++)
	{
		for (j = 0; j < fernSize; j++)
		{
			temp1->fern[i].arrHistoFern[changeIndex][j] = temp_histoFern[i][j];
		}
	}

	for (i = 0; i < theNumberOfFunction; i++)
	{
		temp1->function[changeIndex].arrLoc_Bin_1[i] = temp_loc_Bin_1[i];
		temp1->function[changeIndex].arrLoc_Bin_2[i] = temp_loc_Bin_2[i];
	}


	// 메모리 해제
	for (i = 0; i < theNumberOfClass; i++)
	{
		delete[] temp_histoFern[i];
	}
	delete[] temp_histoFern;

	delete[] temp_loc_Bin_1;
	delete[] temp_loc_Bin_2;
}

// sample weight를 계산하기 위한 함수
void CRandom_Fern::calculateSampleWeight(RANDOMFERNS& rf_classifier, double* feature, double* result, int theNumberOfFunction, int fernIndex, double normalizeFactor, double normalizeContant)
{
	int value = 0;

	int theNumberOfFern = rf_classifier.theNumberOfFern;
	int theNumberOfClass = rf_classifier.theNumberOfClass;

	//	double* fernValue = new double[theNumberOfFern];
	//	memset(fernValue, 0, sizeof(double) * theNumberOfFern);

	// optimal function을 이용하여 샘플 데이터에 대한 fern index를 추정
	value = getIndexUsingOptimalFunction(rf_classifier.function, feature/*, fernValue*/, theNumberOfFunction, fernIndex);

	// optimal fern을 이용한 확률값 계산
	getProbabilityUsingOptimalFern(rf_classifier.fern/*, fernValue*/, result, fernIndex, value, theNumberOfClass, normalizeFactor, normalizeContant);

	//	delete [] fernValue;
}

// optimal function을 이용하여 샘플 데이터에 대한 fern index를 추정
int CRandom_Fern::getIndexUsingOptimalFunction(t_FernFunctionInfo* rf_function, double* feature/*, double* fernValue*/, int theNumberOfFunction, int fernIndex)
{
	register int j;

	// fern 생성
	//int sum = 0;

	//int loc_Bin_1 = 0;
	//int loc_Bin_2 = 0;

	//double value_1 = 0.0;
	//double value_2 = 0.0;
	//
	//int* arrBinary = new int[theNumberOfFunction];
	//memset(arrBinary, 0, sizeof(int)*theNumberOfFunction);

	int binaryValue = 0;

	// fern 생성	
	// binary code 생성
	for (j = 0; j < theNumberOfFunction; j++)	// fern size
	{
		//loc_Bin_1 = rf_function[fernIndex].arrLoc_Bin_1[j];
		//loc_Bin_2 = rf_function[fernIndex].arrLoc_Bin_2[j];

		//////// binary code 생성
		//value_1 = feature[loc_Bin_1];
		//value_2 = feature[loc_Bin_2];

		//// 두 픽셀 간의 비교
		//if( value_1 > value_2 )
		//{
		//	arrBinary[j] = 1;
		//}
		//else
		//{
		//	arrBinary[j] = 0;
		//}

		//	if(feature[rf_function[fernIndex].arrLoc_Bin_1[j]] >= feature[rf_function[fernIndex].arrLoc_Bin_2[j]])
		if (feature[rf_function[fernIndex].arrLoc_Bin_1[j]] - feature[rf_function[fernIndex].arrLoc_Bin_2[j]] >= FUCNTION_THRESHOLD)
		{
			binaryValue |= (int)pow(2.0, j);
		}

	}// fern size end

	// binary code To decimal number				
	//	sum = convertBinaryToDecimal(arrBinary, theNumberOfFunction);

	//	delete [] arrBinary;

	// optimal function에 의해 계산된 값
	//	return sum;

	return binaryValue;
}

// optimal fern을 이용한 확률값 계산
void CRandom_Fern::getProbabilityUsingOptimalFern(t_FernInfo* rf_fern/*, double* feature*/, double* result, int fernIndex, int value, int theNumberOfClass, double normalizeFactor, double normalizeContant)
{
	register int /*i,*/ j;

	int testFernNumber = 0;

	//double* pro_classFernHisto = new double[theNumberOfClass];
	//memset(pro_classFernHisto, 0, sizeof(double) * theNumberOfClass);

	//double* int_TRNclassFernHisto = new double[theNumberOfClass];
	//memset(int_TRNclassFernHisto, 0, sizeof(double) * theNumberOfClass);

	double total_classHisto = 0.0;

	double log_eachFern = 0.0;
	//	double calcLog = 0.0;
	//	double pro_priorValue = 0.0;

	//	int curClassTotalBinSum = 0;

	//	// 각 fern마다 정규화
	//	for( j = 0; j < theNumberOfClass; j++ )
	//	{
	//		testFernNumber = (int)value;
	//
	//		int_TRNclassFernHisto[j] = rf_fern[j].arrHistoFern[fernIndex][testFernNumber];
	//			
	//		total_classHisto += int_TRNclassFernHisto[j];
	//	}
	//
	//	for( j = 0; j < theNumberOfClass; j++ )
	//	{
	//		pro_classFernHisto[j] = ( ( (double)int_TRNclassFernHisto[j] ) / ( (double)total_classHisto ) );
	//	}	
	//	
	//
	//	// 각 class끼리 log 취한 후 sum
	//	for( i = 0; i < theNumberOfClass; i++ )
	//	{
	////		log_eachFern = 0.0;
	//		
	//		// 각 fern의 요소에 log 취한다음 sum 하는 부분
	////		log_eachFern += log(pro_classFernHisto[i]);
	//
	//		result[i] = pro_classFernHisto[i];
	//	}

	testFernNumber = value;

	for (j = 0; j < theNumberOfClass; j++)
	{
		total_classHisto += (rf_fern[j].arrHistoFern[fernIndex][testFernNumber] + (normalizeFactor / normalizeContant));
	}

	for (j = 0; j < theNumberOfClass; j++)
	{
		result[j] = (rf_fern[j].arrHistoFern[fernIndex][testFernNumber] + (normalizeFactor / normalizeContant)) / total_classHisto;
	}

	// 나온 결과 확률값을 가지고 사전 확률값을 product	
	//pro_priorValue = (double)1 / (double)theNumberOfClass;
	//
	//for( i = 0; i < theNumberOfClass; i++ )
	//{
	//	result[i] *= pro_priorValue;
	//}

	//delete [] pro_classFernHisto;
	//delete [] int_TRNclassFernHisto;
}



void CRandom_Fern::valueInit()
{
}
void CRandom_Fern::valueDestroy()
{
	destroyClassifier(temp);
	//	destroyClassifier(temp2);
}