#include "stdafx.h"
#include "Tracker.h"


CTracker::CTracker()
{
}


CTracker::~CTracker()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////// Protected ////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

/**
@brief			Detection된 정보를 이용하여 새로운 tracker 생성
@param			dib1 : Gray Image
dib2 : HSI Image
flag : 새로 할당된 tracker -> true, 기존에 할당된 tracker -> false
cx, cy : 매칭된 detection 윈도우의 중심 좌표
min_x, min_y, max_x, max_y : 매칭된 detection 윈도우의 좌표
*/
void CTracker::processGenerationTracker(CDib& dib, CDib& dib2, bool flag, int& cx, int& cy, int& min_x, int& min_y, int& max_x, int& max_y)
{
	// 가속도 배열 초기화
	velocity[0][0] = 0;
	velocity[0][1] = 0;
	velocity[1][0] = 0;
	velocity[1][1] = 0;
	velocity_index = 0;

	//	memset(theNumberOfDataPerClass, 0, sizeof(int) * PART_CLASSES);

	// 다른 객체와 겹침이 발생하는 거리의 임계값
	distBetweenObjectsThreshold = 0;

	// 매칭된 detection 윈도우 정보를 이용하여 파티클 크기 초기화
	getTargetLocationFromDetection(preTargetParticle, currentParticles, cx, cy, min_x, min_y, max_x, max_y);

	// 매칭된 detection 윈도우 정보를 이용하여 파티클 샘플링
	createParticle(flag, preTargetParticle.x, preTargetParticle.y, currentParticles, 100, distBetweenObjectsThreshold, velocity);

	// OCS-LBP와 Color의 Template을 만들어 준다.
	CreateTemplateValue(dib, dib2, cx, cy, min_x, min_y, max_x, max_y); 
	
	// Random Ferns 학습
	//RFtraining(dib, dib2, currentParticles, rf_ocslbp, rf_color, false, true); // flase, true

	// 겹침 count 초기화
	initOcclusionCount();

	// 화면에서 사라지는 count 초기화
	initNotDisplayCounnt();

	// tracker state 초기화
	//initStateForPrediction();

	// 파티클 값 저장
	for (int i = 0; i < PARTICLE_NUMBER; i++)
	{
		preParticles[i] = currentParticles[i];
	}

	updateFrame = getGenerationFrame();

	//	threshold_max = TRACKING_PRO_MAX_TH;
	//	threshold_min = TRACKING_PRO_MIN_TH;

	return;
}

/**
@brief			기존 tracker 정보만 이용하여 추적 수행
새 타겟의 위치는 상위 N개 파티클의 평균 크기와 위치로 추정
@param			dib1 : Gray Image
dib2 : Color Image
*/
void CTracker::processTracking(CDib& dib, CDib dib2)
{
	// 파티클의 part 위치 초기화
	//	initializePartLocationInAllParticles(currentParticles, preTargetParticle);

	// 특징 추출 - 각 파티클의 확률값 추출
	extractFeature(dib, dib2, currentParticles);

	// optical flow 추정
	//	getVeolocity(preTargetParticle, opticalFlow, velocity);

	// 예측된 타겟의 위치와 파티클들의 거리와 각 파티클들의 확률값을 가중치 합하여 score를 계산함
	// score를 이용하여 파티클을 내림차순으로 정렬함
	sortParticles(currentParticles, preTargetParticle, PARTICLE_NUMBER, velocity);

	/*for (int i = 0; i < PARTICLE_NUMBER * PARTICLE_RATE; i++)
	{
		Trace(_T("%d번째 probability - %lf \n"), i, currentParticles[i].probability);
	}*/

	// 정렬된 파티클의 상위 N 개 윈도우의 위치와 크기 평균을 계산하여 새로운 타겟의 위치를 계산함
	estimateNewTargetStateUsingUpperParticles(estTargetParticle, currentParticles);
}

/**
@brief			Detection과 matching 된 결과에 따라 tracker의 state를 업데이트 함
매칭된 경우 detection 정보를 이용하여 업데이트
매칭되지 않은 경우 이전 tracker 정보를 이용하여 업데이트
@param			cx, cy : 매칭된 detection 윈도우의 중심 좌표
min_x, min_y, max_x, max_y : 매칭된 detection 윈도우의 좌표
*/
void CTracker::processUpdatingTrackerStates(CDib& dib, CDib& dib2, int& cx, int& cy, int& min_x, int& min_y, int& max_x, int& max_y)
{
	int detectionW = RND((max_x - min_x) / 2);
	int detectionH = RND((max_y - min_y) / 2);

	// detection과 tracker가 매칭되었는지 알아보기 위하여
	bool flag = getMatchingflag();



	//	int tid = getTrackerIndex();

	//	Trace(_T("ID : %d: dx : %d, dy : %d / tx : %d, ty : %d\n"), displayTrackerID, cx, cy, estTargetParticle.x, estTargetParticle.y);

	// detection과 매칭된 경우
	if (flag)
	{
		distanceBetweenPreAndCurr = estimateNewTargetStateUsingDetectionInformation(estTargetParticle, preTargetParticle, cx, cy, detectionW, detectionH);
	}
	// detection과 매칭 안 된 경우
	else
	{
		distanceBetweenPreAndCurr = estimateNewTargetStateUsingPreTrackerInformation(estTargetParticle, preTargetParticle);
	}


//	distanceBetweenPreAndCurr = estimateNewTargetStateUsingPreTrackerInformation(estTargetParticle, preTargetParticle);

	double thresholdBetweenPreAndCurr = preTargetParticle.w /** DIST_RATE*/;

	// 이전 tracker와 현재 tracker 간의 거리가 임계값 이하일 경우
	if (distanceBetweenPreAndCurr <= thresholdBetweenPreAndCurr)
	{
		// 새로운 타겟을 currentParticles 구조체 배열의 0 인덱스에 저장
		// currentParticles 구조체 배열의 내용들은 전부 뒤로 1 인덱스 씩 밀림
		// 새로운 타겟에는 확률값이 없기 때문에 RF를 이용하여 확률값 추정
		moveParticleToNextIndex(dib, dib2, estTargetParticle, currentParticles);
	}

	//Trace(_T("C Pro - %lf"), currentParticles[0].probability);

	//Trace(_T("%d\t%f\n"), displayTrackerID, dist);
}

/**
@brief			Tracker의 state, 다른 tracker와 거리, tracker의 확률값에 따라 분류기의 업데이트 결정
@param			dib1 : Gray Image
dib2 : HSI Image
flag : 새로 할당된 tracker -> true, 기존에 할당된 tracker -> false
otherTarget : 다른 tracker의 위치 정보
*/
void CTracker::processUpdatingClassifiers(CDib& dib, CDib& dib2, bool flag, OTHERTARGETLOCATION& otherTarget, int currentFrame, COpticalFlow& opticalFlow)
{
	register int i;

	double thresholdBetweenPreAndCurr = preTargetParticle.w * DIST_RATE;

	int frontBack_flag = -1;

	int cx = currentParticles[0].x;
	int cy = currentParticles[0].y;
	int min_x = currentParticles[0].x - currentParticles[0].w;
	int max_x = currentParticles[0].x + currentParticles[0].w;
	int min_y = currentParticles[0].y - currentParticles[0].h;
	int max_y = currentParticles[0].y + currentParticles[0].h;

	// 이전 tracker와 현재 tracker 간의 거리가 임계값 이하일 경우
	if (distanceBetweenPreAndCurr <= thresholdBetweenPreAndCurr)
	{
		//threshold_max = TRACKING_PRO_MAX_TH;
		//threshold_min = TRACKING_PRO_MIN_TH;		

		// 새로운 타겟을 currentParticles 구조체 배열의 0 인덱스에 저장
		// currentParticles 구조체 배열의 내용들은 전부 뒤로 1 인덱스 씩 밀림
		// 새로운 타겟에는 확률값이 없기 때문에 RF를 이용하여 확률값 추정
		//	moveParticleToNextIndex(dib, estTargetParticle, currentParticles);

		// 겹침 구분을 위해 다른 target과 거리 측정 후 가장 짧은 거리를 가져옴
		// 가장 짧은 거리가 임계값 보다 이하인 경우 겹침으로 판단
		//  0 : back
		//  1 : front
		// -1 : 알 수 없음
		frontBack_flag = measureDistanceBetweenOtherTargets(estTargetParticle, otherTarget, ratioBetweenObjects);

		//int currentArea = (currentParticles[0].w * 2) * (currentParticles[0].h * 2);

		//if (currentArea <= (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_5) * (DEFAULT_WINDOW_HEIGHT / IMG_LEVEL_RESIZE_RATE_5))
		//{
		//	threshold_max = 0.65;
		//	threshold_min = 0.49;
		//}
		//else if (currentArea <= (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_4) * (DEFAULT_WINDOW_HEIGHT / IMG_LEVEL_RESIZE_RATE_4))
		//{
		//	threshold_max = 0.65;
		//	threshold_min = 0.49;
		//}
		//else if (currentArea <= (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_3) * (DEFAULT_WINDOW_HEIGHT / IMG_LEVEL_RESIZE_RATE_3))
		//{
		//	threshold_max = 0.63;
		//	threshold_min = 0.46;
		//}
		//else if (currentArea <= (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_2) * (DEFAULT_WINDOW_HEIGHT / IMG_LEVEL_RESIZE_RATE_2))
		//{
		//	threshold_max = 0.62;
		//	threshold_min = 0.43;
		//}
		//else if (currentArea <= (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_1) * (DEFAULT_WINDOW_HEIGHT / IMG_LEVEL_RESIZE_RATE_1))
		//{
		//	threshold_max = 0.60;
		//	threshold_min = 0.43;
		//}
		//else if (currentArea <= (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_0) * (DEFAULT_WINDOW_HEIGHT / IMG_LEVEL_RESIZE_RATE_0))
		//{
		//	threshold_max = 0.55;
		//	threshold_min = 0.43;
		//}

		Trace(_T("current particle probability - %lf\n"), currentParticles[0].probability);

		// 새로운 타겟의 확률값이 임계값(max) 보다 높은 경우 && 다른 tracker와 거리가 임계값 보다 먼 경우
		// RF 업데이트 수행
		// 겹침 또는 큰 상태 변화가 없는 경우를 의미함
		if (/*(currentParticles[0].probability >= 0.5*/ /*threshold_max*//*) &&*/ (ratioBetweenObjects <= OVERLAP_RATIO_TH))
		{
			// 분류기를 업데이트 할 경우에만 가속도 정보를 업데이트 함
			// optical flow 추정
			//getVelocity(estTargetParticle, opticalFlow, velocity, velocity_index);

			velocity[velocity_index % 2][0] = (int)(((currentParticles[0].x - preTargetParticle.x) * 0.5));
			velocity[velocity_index % 2][1] = (int)(((currentParticles[0].y - preTargetParticle.y) * 0.5));

			velocity_index++;

			if (currentParticles[0].probability >= 0.7)
			{
				// Template 업데이트
				CreateTemplateValue(dib, dib2, cx, cy, min_x, min_y, max_x, max_y);

				currentParticles[0].occlusionType = 0;
			}

			else
				currentParticles[0].occlusionType = 1;

			// 현재 파티클 정보를 이전 파티클 구조체에 저장
			for (i = 0; i < PARTICLE_NUMBER; i++)
			{
				preParticles[i] = currentParticles[i];
			}

			// prediction에서 속도 계산을 위해 이전 tracker 위치 저장
			setPreviousTrackerLocation(preTargetParticle.x, preTargetParticle.y);

			preTargetParticle.x = currentParticles[0].x;
			preTargetParticle.y = currentParticles[0].y;
			preTargetParticle.w = currentParticles[0].w;
			preTargetParticle.h = currentParticles[0].h;


			// 새로운 타겟을 이용하여 particle 샘플링
			createParticle(flag, preTargetParticle.x, preTargetParticle.y, currentParticles, (int)distanceBetweenObjects, distBetweenObjectsThreshold, velocity);

			updateFrame = currentFrame;
			//	threshold_max = currentParticles[0].probability;풤


			// 화면에 표시될 윈도우의 색상 지정
			// 녹색
			//currentParticles[0].occlusionType = 1;
		}
		// 새로운 타겟의 확률값이 임계값(min) 보다 낮은 경우
		// RF 업데이트를 수행하지 않음
		// 완전 겹침 또는 화면에서 사라진 경우를 의미함
		//else if(currentParticles[0].probability < threshold_min)
		//{
		//	// 이전 파티클 정보를 사용함
		//	for(i = 0; i < PARTICLE_NUMBER; i++)
		//	{
		//		currentParticles[i] = preParticles[i];
		//	}

		////	threshold_min = currentParticles[0].probability;

		//	// 화면에 표시될 윈도우의 색상 지정
		//	// 붉은색
		//	currentParticles[0].occlusionType = 2;
		//}
		// 새로운 타겟의 확률값이 임계값들(max, min) 사이에 존재 || 다른 tracker와 겹침이 발생한 경우
		// RF 업데이트를 수행하지 않음
		// 약간의 겹침 또는 상태 변화를 의미함
		else if ((ratioBetweenObjects > OVERLAP_RATIO_TH))	// 부분 겹침
		{
			// 확률값이 임계값 max과 임계값 min 사이인 경우
			//  0 : back
			//  1 : front
			// -1 : 알 수 없음
			if (frontBack_flag)
			{
				//getVelocity(estTargetParticle, opticalFlow, velocity, velocity_index);

				velocity[velocity_index % 2][0] = (int)(((currentParticles[0].x - preTargetParticle.x) * 0.5));
				velocity[velocity_index % 2][1] = (int)(((currentParticles[0].y - preTargetParticle.y) * 0.5));
				velocity_index++;

				if (currentParticles[0].probability >= 0.7)
				{
					// Template 업데이트
					CreateTemplateValue(dib, dib2, cx, cy, min_x, min_y, max_x, max_y);
				}
			}

			// 현재 파티클 정보를 저장
			for (i = 0; i < PARTICLE_NUMBER; i++)
			{
				preParticles[i] = currentParticles[i];
			}

			// prediction에서 속도 계산을 위해 이전 tracker 위치 저장
			setPreviousTrackerLocation(preTargetParticle.x, preTargetParticle.y);

			preTargetParticle.x = currentParticles[0].x;
			preTargetParticle.y = currentParticles[0].y;
			preTargetParticle.w = currentParticles[0].w;
			preTargetParticle.h = currentParticles[0].h;

			// 새로운 타겟을 이용하여 particle 샘플링
			createParticle(flag, preTargetParticle.x, preTargetParticle.y, currentParticles, (int)distanceBetweenObjects, distBetweenObjectsThreshold, velocity);


			// 겹침이 발생한 경우
			// 뒷 쪽에 있는 경우
			if (frontBack_flag == 0)
			{
				// 화면에 표시될 윈도우의 색상 지정
				// 노란색
				currentParticles[0].occlusionType = 3;
			}
			// 앞 쪽에 있는 경우
			else
			{
				// 화면에 표시될 윈도우의 색상 지정
				// 파란색
				currentParticles[0].occlusionType = 1;
			}
		}
		else
		{
			// 이전 파티클 정보를 사용함
			for (i = 0; i < PARTICLE_NUMBER; i++)
			{
				currentParticles[i] = preParticles[i];
			}

			// 화면에 표시될 윈도우의 색상 지정
			// 붉은색
			currentParticles[0].occlusionType = 2;
		}
	}
	// 이전 tracker와 현재 tracker 간의 거리가 임계값 이상일 경우
	// 잘못된 추적으로 판단함
	else
	{
		// 이전 파티클 정보 사용
		for (i = 0; i < PARTICLE_NUMBER; i++)
		{
			currentParticles[i] = preParticles[i];
		}

		// 이전 파티클 정보를 이용하여 particle 샘플링
		createParticle(flag, preTargetParticle.x, preTargetParticle.y, currentParticles, (int)distanceBetweenObjects, distBetweenObjectsThreshold, velocity);

		// 화면에 표시될 윈도우의 색상 지정
		// 보라색
		currentParticles[0].occlusionType = 4;
	}
}

// 상위 4개

void CTracker::Compare_Distance()
{

}

/**
@brief			OCS-LBP와 Color의 Template 배열을 만들어 준다
*/
void CTracker::CreateTemplateValue(CDib& dib, CDib& dib2, int& cx, int& cy, int& min_x, int& min_y, int& max_x, int& max_y)
{
	register int i, j, i1, j1;

	BYTE** ptr = dib.GetPtr();
	RGBBYTE** rgbptr = dib2.GetRGBPtr();

	int imgW = IMAGE_WIDTH;
	int imgH = IMAGE_HEIGHT;

	double halfWidth = (max_x - min_x)/2;
	double halfHeight = (max_y - min_y)/2;

	int left_b = 0;
	int top_b = 0;
	int right_b = 0;
	int bottom_b = 0;

	int region = 0;
	int region_for_color = 0;

	const int SUB_REGION = SUB_BLOCK;
	const int FEATURE_SIZE = FEATURE_DEMENSION;
	const int FEATURE_SIZE_FOR_COLOR = FEATURE_COLOR_DEMENSION;

	// OCS-LBP =============================================================
	const int OCSLBP_SIZE = OCSLBP_FEATURE_SIZE;
	int t = OCSLBP_THRESHOLD;

	int quarterHeight = (int)(halfHeight  * 0.5);
	int quarterWidth = (int)(halfWidth  * 0.5);

	memset(Template_OCSLBP_histogram, 0, sizeof(double) * OCSLBP_SIZE);

	// Color ===============================================================
	const int COLOR_SIZE = COLOR_FEATURE_SIZE_FOR_TRACKING;

	// the normalised coordinates
	double dX = 0.0;
	double dY = 0.0;
	double dValue = 0.0;
	double eK = 0.0;
	int brX = 0;
	int bgX = 0;
	int bbX = 0;

	memset(Template_Color_histogram, 0, sizeof(double) * COLOR_SIZE);

	// feature extraction ==================================================
	for (i = 0; i < 4; i++)
	{
		for (j = 0; j < 4; j++)
		{
			top_b = min_y + (quarterHeight * i) + 1;
			left_b = min_x + (quarterWidth * j) + 1;
			right_b = (left_b - 1) + quarterWidth - 1;
			bottom_b = (top_b - 1) + quarterHeight - 1;

			region = (i * 4 + j) * FEATURE_SIZE;
			region_for_color = (i * 4 + j) * FEATURE_SIZE_FOR_COLOR;

			// feature extraction
			if (bottom_b < dib.GetHeight())
				for (i1 = top_b; i1 < bottom_b; i1++)
				{
					if (i1 < 1 || i1 >= imgH - 1)
					{
						continue;
					}

					for (j1 = left_b; j1 < right_b; j1++)
					{
						if (j1 < 1 || j1 >= imgW - 1)
						{
							continue;
						}

						// OCS-LBP ===========================================================
						if (ptr[i1][j1 + 1] - ptr[i1][j1 - 1] >= t)										// 0
						{
							Template_OCSLBP_histogram[region + 0] += 1;
						}
						else if (ptr[i1][j1 - 1] - ptr[i1][j1 + 1] >= t)									// 4
						{
							Template_OCSLBP_histogram[region + 4] += 1;
						}

						if (ptr[i1 + 1][j1 + 1] - ptr[i1 - 1][j1 - 1] >= t)								// 1
						{
							Template_OCSLBP_histogram[region + 1] += 1;
						}
						else if (ptr[i1 - 1][j1 - 1] - ptr[i1 + 1][j1 + 1] >= t)							// 5
						{
							Template_OCSLBP_histogram[region + 5] += 1;
						}

						if (ptr[i1 + 1][j1] - ptr[i1 - 1][j1] >= t)										// 2
						{
							Template_OCSLBP_histogram[region + 2] += 1;
						}
						else if (ptr[i1 - 1][j1] - ptr[i1 + 1][j1] >= t)									// 6
						{
							Template_OCSLBP_histogram[region + 6] += 1;
						}

						if (ptr[i1 + 1][j1 - 1] - ptr[i1 - 1][j1 + 1] >= t)								// 3
						{
							Template_OCSLBP_histogram[region + 3] += 1;
						}
						else if (ptr[i1 - 1][j1 + 1] - ptr[i1 + 1][j1 - 1] >= t)							// 7
						{
							Template_OCSLBP_histogram[region + 7] += 1;
						}

						// Color =============================================================
						// the Epanechnikov kernel

						// 언젠가 본다면.. 4x4가 맞는지 확인해보고
						// 고쳐보자.

						dY = (i1 - cy) / halfHeight;
						dX = (j1 - cx) / halfWidth;

						dValue = (dX * dX) + (dY * dY);

						if (1.0 >= dValue)
						{
							eK = 1.0 - dValue;
						}
						else
						{
							eK = 0.0;
						}

						// intensity 특징 추출
						brX = rgbptr[i1][j1].r / INTENSITY_QUANTITY;
						bgX = rgbptr[i1][j1].g / INTENSITY_QUANTITY;
						bbX = rgbptr[i1][j1].b / INTENSITY_QUANTITY;

						Template_Color_histogram[region_for_color + brX] += eK;
						Template_Color_histogram[region_for_color + COLOR_DISTANCE + bgX] += eK;
						Template_Color_histogram[region_for_color + 2 * COLOR_DISTANCE + bbX] += eK;

						brX = 0, bgX = 0, bbX = 0, dX = 0.0, dY = 0.0, eK = 0.0, dValue = 0.0;
					}
				}
		}
	}


	L2Normalization(Template_OCSLBP_histogram, OCSLBP_SIZE);
	L2Normalization(Template_Color_histogram, COLOR_SIZE);
}

/**
@brief			detection 윈도우와 tracker 간의 거리 측정
연관성 검사 알고리즘에 사용됨
@param			cx, cy : 매칭할 detection 윈도우의 중심 좌표
@return			detection 윈도우와 tracker 간의 거리 반환
*/
double CTracker::measureDistanceBetweenDetectionAndTracking(int& cx, int& cy)
{
	return sqrt((double)(cx - currentParticles[0].x) * (cx - currentParticles[0].x) + (double)(cy - currentParticles[0].y) * (cy - currentParticles[0].y));
}

/**
@brief			tracker의 분류기를 이용하여 detection 윈도우의 확률값 추정
연관성 검사 알고리즘에 사용됨
@param			dib1 : Gray Image
dib2 : HSI Image
cx, cy : 매칭할 detection 윈도우의 중심 좌표
min_x, min_y, max_x, max_y : 매칭할 detection 윈도우의 좌표
@return			detection 윈도우의 추정된 확률값
*/
double CTracker::getFeature_n_measureProbabilityDetectionRegion(CDib& dib, int& cx, int& cy, int& min_x, int& min_y, int& max_x, int& max_y)
{
	register int i, j, i1, j1;

	//BYTE** ptr = dib.GetPtr();

	RGBBYTE** rgbptr = dib.GetRGBPtr();

	int w = dib.GetWidth();
	int h = dib.GetHeight();

	int d_w = max_x - min_x;
	int d_h = max_y - min_y;


	int t_w = currentParticles[0].w;
	int t_h = currentParticles[0].h;

	int t_cx = currentParticles[0].x;
	int t_cy = currentParticles[0].y;

	double* d_mean = new double[3];
	double* t_mean = new double[3];
	double* d_std = new double[3];
	double* t_std = new double[3];

	double mean_RGB_correlation = 0.0;

	// 현재 x 좌표에서 원본 저장
	/*CString str = _T(".//DATA//Ori -");
	CString strcx;
	strcx.Format(_T("%d"), cx);

	str += strcx;

	str += _T(".bmp");

	dib.Save(str);*/

	// Tracker Window //
	CDib t_Dib;
	t_Dib.CreateRGBImage(t_w * 2, t_h * 2, RGB(0, 0, 0));

	RGBBYTE** t_Ptr = t_Dib.GetRGBPtr();

	for (i = 0, i1 = t_cy - t_h; i < t_h * 2, i1 < t_cy + t_h; i++, i1++)
	{
		for (j = 0, j1 = t_cx - t_w; j < t_w * 2, j1 < t_cx + t_w; j++, j1++)
		{
			t_Ptr[i][j].r = rgbptr[i1][j1].r;
			t_Ptr[i][j].g = rgbptr[i1][j1].g;
			t_Ptr[i][j].b = rgbptr[i1][j1].b;
		}
	}

	// Detection Window //
	CDib d_Dib;
	d_Dib.CreateRGBImage(d_w, d_h, RGB(0, 0, 0));
	
	RGBBYTE** d_ptr = d_Dib.GetRGBPtr();

	for (i = 0, i1 = min_y; i < d_h, i1 < max_y; i++, i1++)
	{
		for (j = 0, j1 = min_x; j < d_w, j1 < max_x; j++, j1++)
		{
			d_ptr[i][j].r = rgbptr[i1][j1].r;
			d_ptr[i][j].g = rgbptr[i1][j1].g;
			d_ptr[i][j].b = rgbptr[i1][j1].b;
		}
	}

	// 크기는 Detection Window에 맞춰서//
	// 양선형 보간법 - Tracker Window //
	DibResizeBilinear_FOR_RGBBYTE(t_Dib, d_w, d_h);
	t_Dib.Save(_T(".//DATA//Tracker Window.bmp"));
	
	// 양선형 보간법 - Detection Window //
	DibResizeBilinear_FOR_RGBBYTE(d_Dib, d_w, d_h);
	d_Dib.Save(_T(".//DATA//Detection Window.bmp"));

 	CalculateMeanValue_FOR_RGBBYTE(t_Dib, d_w, d_h, t_mean);
	CalculateMeanValue_FOR_RGBBYTE(d_Dib, d_w, d_h, d_mean);
	CalculateStandardDeviation_FOR_RGBBYTE(t_Dib, t_mean, d_w, d_h, t_std);
	CalculateStandardDeviation_FOR_RGBBYTE(d_Dib, d_mean, d_w, d_h, d_std);
	Compare_correlation(d_Dib, t_Dib, d_w, d_h, d_mean,t_mean, d_std, t_std, mean_RGB_correlation);

	if (mean_RGB_correlation < 0)
		int a = 0;


	delete[] d_mean;
	delete[] t_mean;
	delete[] d_std;
	delete[] t_std;

	return mean_RGB_correlation;

}

/*
@brief			Tracker와 Detection 간 RGB cross - correlation
*/
void CTracker::Compare_correlation(CDib& d_Dib, CDib t_Dib, int width, int height, double* d_mean, double* t_mean, double* d_std, double* t_std, double& result)
{
	RGBBYTE** d_ptr = d_Dib.GetRGBPtr();
	RGBBYTE** t_ptr = t_Dib.GetRGBPtr();

	double* RGB_corr = new double[3];
	memset(RGB_corr, 0, sizeof(double) * 3);

	double mean_RGB_corr = 0.0;


	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			if (t_std[0] == 0 || d_std[0] == 0)
			{
				RGB_corr[0] = 0;
			}
			else
			{
				if (d_ptr[i][j].r != 0 || t_ptr[i][j].r != 0)
					RGB_corr[0] += ((d_ptr[i][j].r - d_mean[0]) * (t_ptr[i][j].r - t_mean[0])) / (d_std[0] * t_std[0]);
			}
			if (t_std[1] == 0 || d_std[1] == 0)
			{
				RGB_corr[1] = 0;
			}
			else
			{
				if (d_ptr[i][j].g != 0 || t_ptr[i][j].g != 0)
					RGB_corr[1] += ((d_ptr[i][j].g - d_mean[1]) * (t_ptr[i][j].g - t_mean[1])) / (d_std[1] * t_std[1]);
			}
			if (t_std[2] == 0 || d_std[2] == 0)
			{
				RGB_corr[2] = 0;
			}
			else
			{
				if (d_ptr[i][j].b != 0 || t_ptr[i][j].b != 0)
					RGB_corr[2] += ((d_ptr[i][j].b - d_mean[2]) * (t_ptr[i][j].b - t_mean[2])) / (d_std[2] * t_std[2]);
			}
		}
	}

	if (width != 0 || height != 0)
	{
		for (int i = 0; i < 3; i++)
		{
			RGB_corr[i] = RGB_corr[i] / (width * height);
			mean_RGB_corr += RGB_corr[i];
		}

		mean_RGB_corr /= 3;

	}
	else
		mean_RGB_corr = 1;


	if (mean_RGB_corr >= 0)
		result = 1 - mean_RGB_corr;

	else
		result = mean_RGB_corr;

	delete[] RGB_corr;	
}

/**
@brief			Detection 정보를 이용하여 tracker 윈도우의 초기 위치 및 크기 설정
@param			particle : 이전 tracker의 정보 저장
particle2 : 현재 tracker의 파티클 정보 저장
cx, cy : 매칭된 detection 윈도우의 중심 좌표
min_x, min_y, max_x, max_y : 매칭된 detection 윈도우의 좌표
@date			2014-01-07
*/
void CTracker::getTargetLocationFromDetection(TARGETPARTICLE& particle, PARTICLEINFO* particle2, int& cx, int& cy, int& min_x, int& min_y, int& max_x, int& max_y)
{
	register int k/*, i*/;

	particle.x = cx;
	particle.y = cy;
	particle.w = RND((max_x - min_x) / 2);
	particle.h = RND((max_y - min_y) / 2);

	for (k = 0; k < PARTICLE_NUMBER; k++)
	{
		particle2[k].w = particle.w;
		particle2[k].h = particle.h;

		particle2[k].probability = 0.0;

		//for(i = 0; i < SUB_BLOCK; i++)
		//{
		//	particle2[k].OCSLBP_part_probability[i] = 0.0;
		//	particle2[k].LID_part_probability[i] = 0.0;
		//}

		particle2[k].OCSLBP_part_probability = 0.0;
	}
}

double CTracker::getOverlapRatioBetweenDetectionsAndTrackers(int& cx, int& cy, int& min_x, int& min_y, int& max_x, int& max_y)
{
	register int i, j;

	double overlapRatio = 0.0;

	CDib dib;
	dib.CreateGrayImage(IMAGE_WIDTH, IMAGE_HEIGHT, 0);

	BYTE** ptr = dib.GetPtr();

	int s_min_x = 0;
	int s_min_y = 0;
	int s_max_x = 0;
	int s_max_y = 0;

	int d_cx = cx;
	int d_cy = cy;
	int d_min_x = min_x;
	int d_min_y = min_y;
	int d_max_x = max_x;
	int d_max_y = max_y;

	int t_cx = currentParticles[0].x;
	int t_cy = currentParticles[0].y;
	int t_min_x = currentParticles[0].x - currentParticles[0].w;
	int t_min_y = currentParticles[0].y - currentParticles[0].h;
	int t_max_x = currentParticles[0].x + currentParticles[0].w;
	int t_max_y = currentParticles[0].y + currentParticles[0].h;

	int d_area = (d_max_x - d_min_x) * (d_max_y - d_min_y);
	int t_area = (t_max_x - t_min_x) * (t_max_y - t_min_y);
	int i_area = 0;

	// detection 윈도우와 tracker 윈도우로 이루어지는 사각형의 크기
	if (d_min_x <= t_min_x)
	{
		s_min_x = d_min_x;
	}
	else
	{
		s_min_x = t_min_x;
	}

	if (d_min_y <= t_min_y)
	{
		s_min_y = d_min_y;
	}
	else
	{
		s_min_y = t_min_y;
	}

	if (d_max_x >= t_max_x)
	{
		s_max_x = d_max_x;
	}
	else
	{
		s_max_x = t_max_x;
	}

	if (d_max_y >= t_max_y)
	{
		s_max_y = d_max_y;
	}
	else
	{
		s_max_y = t_max_y;
	}

	// detection 윈도우 표시
	for (i = d_min_y; i < d_max_y; i++)
	{
		for (j = d_min_x; j < d_max_x; j++)
		{
			ptr[i][j] += 1;
		}
	}

	// tracker 윈도우 표시
	for (i = t_min_y; i < t_max_y; i++)
	{
		for (j = t_min_x; j < t_max_x; j++)
		{
			ptr[i][j] += 1;
		}
	}

	// intersection 영역 크기 계산
	for (i = s_min_y; i < s_max_y; i++)
	{
		for (j = s_min_x; j < s_max_x; j++)
		{
			if (ptr[i][j] == 2)
			{
				i_area++;
			}
		}
	}

	//// detection 윈도우가 tracker 윈도우 보다 많이 큰 경우
	//if(d_area * 0.7 > t_area)
	//{
	//	// detection 윈도우 내에 tracker 윈도우가 포함되거나 100% 겹치는 경우
	//	if(d_min_x <= t_min_x && d_min_y <= t_min_y && d_max_x >= t_max_x && d_max_y >= t_max_y)
	//	{
	//		overlapRatio = 1.0;
	//	}
	//	else
	//	{
	//		overlapRatio = i_area / (double)t_area;
	//	}
	//}
	//// tracker 윈도우가 detection 윈도우 보다 많이 큰 경우
	//else if(t_area * 0.7 > d_area)
	//{
	//	// tracker 윈도우 내에 detection 윈도우가 포함되거나 100% 겹치는 경우
	//	if(d_min_x >= t_min_x && d_min_y >= t_min_y && d_max_x <= t_max_x && d_max_y <= t_max_y)
	//	{
	//		overlapRatio = 1.0;
	//	}
	//	else
	//	{
	//		overlapRatio = i_area / (double)d_area;
	//	}
	//}
	//// detection 윈도우와 tracker 윈도우의 크기가 비슷한 경우
	//else
	//{
	//	// detection 윈도우 내에 tracker 윈도우가 포함되거나 100% 겹치는 경우
	//	if(d_min_x <= t_min_x && d_min_y <= t_min_y && d_max_x >= t_max_x && d_max_y >= t_max_y)
	//	{
	//		overlapRatio = 1.0;
	//	}
	//	// tracker 윈도우 내에 detection 윈도우가 포함되거나 100% 겹치는 경우
	//	else if(d_min_x >= t_min_x && d_min_y >= t_min_y && d_max_x <= t_max_x && d_max_y <= t_max_y)
	//	{
	//		overlapRatio = 1.0;
	//	}
	//	else
	//	{
	//		overlapRatio = i_area / (double)(d_area + t_area - i_area);
	//	}
	//}

	// detection 윈도우 내에 tracker 윈도우가 포함되거나 100% 겹치는 경우
	if (d_min_x <= t_min_x && d_min_y <= t_min_y && d_max_x >= t_max_x && d_max_y >= t_max_y)
	{
		overlapRatio = i_area / (double)d_area;
	}
	// tracker 윈도우 내에 detection 윈도우가 포함되거나 100% 겹치는 경우
	else if (d_min_x >= t_min_x && d_min_y >= t_min_y && d_max_x <= t_max_x && d_max_y <= t_max_y)
	{
		overlapRatio = i_area / (double)t_area;
	}
	else
	{
		overlapRatio = i_area / (double)(d_area + t_area - i_area);
	}

	return overlapRatio;
}

/**
@brief			tracker 윈도우의 width를 가져옴
연관성 검사 알고리즘에 사용됨
@return			tracker 윈도우의 width
*/
double CTracker::getWidthOfTrackingWindow()
{
	return currentParticles[0].w;
}

/**
@brief			tracker 윈도우의 height를 가져옴
연관성 검사 알고리즘에 사용됨
@return			tracker 윈도우의 height
@date			2014-01-07
*/
double CTracker::getHeightOfTrackingWindow()
{
	return currentParticles[0].h;
}

/**
@brief			파티클 샘플링
객체 윈도우의 크기에 따라 샘플링 범위가 다름
@param			flag : 새로 할당된 tracker -> true, 기존에 할당된 tracker -> false
xPoint, yPoint : 타겟 또는 매칭된 detection 윈도우의 중심 좌표
particle1 : 현재 tracker의 파티클 정보 저장
distanceBetweenTargets : 현재 tracker와 다른 tracker 간의 거리
threshold : tracker 간의 겹침 발생 거리 임계값
*/
void CTracker::createParticle(bool flag, const int& xPoint, const int& yPoint, PARTICLEINFO* particle1, int distanceBetweenTargets, double& threshold, int velocity[2][2])
{
	int intervalX = 0;
	int intervalY = 0;
	int particleDistributionFlag = 1;

	double movingThreshold = 0.0;
	int distanceBetweenTargetsThreshold = 0;
	int rangeOfParticleGenerationWhenOcclusion = 0;
	int rangeOfParticleGenerationWhenNormal = 0;
	int rangeOfParticleGenerationWhenMoving = 0;

	int w = particle1[0].w * 2;
	int h = particle1[0].h * 2;

	int currentArea = w * h;

	//// CAVIAR
	//if(TEST_SAMPLE == 0)
	//{
	//	if(currentArea <=  (WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_3) * (WINDOW_HEIGHT / IMG_LEVEL_RESIZE_RATE_3))
	//	{
	//		distanceBetweenTargetsThreshold = 10;  //2013-12-10 JMR 15->10
	//		rangeOfParticleGenerationWhenOcclusion = 5;
	//		rangeOfParticleGenerationWhenNormal = 5;
	//	}
	//	else if(currentArea <=  (WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_2) * (WINDOW_HEIGHT / IMG_LEVEL_RESIZE_RATE_2))
	//	{
	//		distanceBetweenTargetsThreshold = 15; //2013-12-10 JMR 15->15
	//		rangeOfParticleGenerationWhenOcclusion = 7;
	//		rangeOfParticleGenerationWhenNormal = 7;
	//	}
	//	else if(currentArea <=  (WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_1) * (WINDOW_HEIGHT / IMG_LEVEL_RESIZE_RATE_1))
	//	{
	//		distanceBetweenTargetsThreshold = 20;	//2013-12-10 JMR DISTANCE_BETWEEN_OTHERS(23)->20
	//		rangeOfParticleGenerationWhenOcclusion = 8;
	//		rangeOfParticleGenerationWhenNormal = 10;
	//	}
	//	else if(currentArea <=  (WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_0) * (WINDOW_HEIGHT / IMG_LEVEL_RESIZE_RATE_0))
	//	{
	//		distanceBetweenTargetsThreshold = 25;	//2013-12-10 JMR DISTANCE_BETWEEN_OTHERS(23)->25
	//		rangeOfParticleGenerationWhenOcclusion = 10;
	//		rangeOfParticleGenerationWhenNormal = 12;
	//	}
	//	else
	//	{
	//		distanceBetweenTargetsThreshold = 32;
	//		rangeOfParticleGenerationWhenOcclusion = 12;
	//		rangeOfParticleGenerationWhenNormal = 14;
	//	}
	//}
	//// PETS 2009
	//else if(TEST_SAMPLE == 1)
	//{
	//	if(currentArea <=  (WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_3) * (WINDOW_HEIGHT / IMG_LEVEL_RESIZE_RATE_3))
	//	{
	//		distanceBetweenTargetsThreshold = 10;  //2013-12-10 JMR 15->10
	//		rangeOfParticleGenerationWhenOcclusion = 8;
	//		rangeOfParticleGenerationWhenNormal = 8;
	//	}
	//	else if(currentArea <=  (WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_2) * (WINDOW_HEIGHT / IMG_LEVEL_RESIZE_RATE_2))
	//	{
	//		distanceBetweenTargetsThreshold = 15; //2013-12-10 JMR 15->15
	//		rangeOfParticleGenerationWhenOcclusion = 10;
	//		rangeOfParticleGenerationWhenNormal = 10;
	//	}
	//	else if(currentArea <=  (WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_1) * (WINDOW_HEIGHT / IMG_LEVEL_RESIZE_RATE_1))
	//	{
	//		distanceBetweenTargetsThreshold = 20;	//2013-12-10 JMR DISTANCE_BETWEEN_OTHERS(23)->20
	//		rangeOfParticleGenerationWhenOcclusion = 14;
	//		rangeOfParticleGenerationWhenNormal = 12;
	//	}
	//	else if(currentArea <=  (WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_0) * (WINDOW_HEIGHT / IMG_LEVEL_RESIZE_RATE_0))
	//	{
	//		distanceBetweenTargetsThreshold = 25;	//2013-12-10 JMR DISTANCE_BETWEEN_OTHERS(23)->25
	//		rangeOfParticleGenerationWhenOcclusion = 16;
	//		rangeOfParticleGenerationWhenNormal = 14;
	//	}
	//	else
	//	{
	//		distanceBetweenTargetsThreshold = 32;
	//		rangeOfParticleGenerationWhenOcclusion = 18;
	//		rangeOfParticleGenerationWhenNormal = 16;
	//	}
	//}
	//// SK
	//else if(TEST_SAMPLE == 2 || TEST_SAMPLE == 3)
	//{
	//	if(currentArea <=  (WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_3) * (WINDOW_HEIGHT / IMG_LEVEL_RESIZE_RATE_3))
	//	{
	//		distanceBetweenTargetsThreshold = 10;  //2013-12-10 JMR 15->10
	//		rangeOfParticleGenerationWhenOcclusion = 8;
	//		rangeOfParticleGenerationWhenNormal = 8;
	//	}
	//	else if(currentArea <=  (WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_2) * (WINDOW_HEIGHT / IMG_LEVEL_RESIZE_RATE_2))
	//	{
	//		distanceBetweenTargetsThreshold = 15; //2013-12-10 JMR 15->15
	//		rangeOfParticleGenerationWhenOcclusion = 10;
	//		rangeOfParticleGenerationWhenNormal = 10;
	//	}
	//	else if(currentArea <=  (WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_1) * (WINDOW_HEIGHT / IMG_LEVEL_RESIZE_RATE_1))
	//	{
	//		distanceBetweenTargetsThreshold = 20;	//2013-12-10 JMR DISTANCE_BETWEEN_OTHERS(23)->20
	//		rangeOfParticleGenerationWhenOcclusion = 14;
	//		rangeOfParticleGenerationWhenNormal = 12;
	//	}
	//	else if(currentArea <=  (WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_0) * (WINDOW_HEIGHT / IMG_LEVEL_RESIZE_RATE_0))
	//	{
	//		distanceBetweenTargetsThreshold = 25;	//2013-12-10 JMR DISTANCE_BETWEEN_OTHERS(23)->25
	//		rangeOfParticleGenerationWhenOcclusion = 16;
	//		rangeOfParticleGenerationWhenNormal = 14;
	//	}
	//	else
	//	{
	//		distanceBetweenTargetsThreshold = 32;
	//		rangeOfParticleGenerationWhenOcclusion = 18;
	//		rangeOfParticleGenerationWhenNormal = 16;
	//	}
	//}

	double occlRate = 0.65; // 0.65
	double norRate = 0.6; // 0.4


	//////////////
	//윈도우 크기에 따라 범위를 다르게 해줌.///////
	//detection 되면 필요한 코드들.
	//////////////

	///*	if(currentArea <= (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_6) * (DEFAULT_WINDOW_HEIGHT / IMG_LEVEL_RESIZE_RATE_6))
	//{
	//distanceBetweenTargetsThreshold = RND((DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_6) * 0.5);
	//rangeOfParticleGenerationWhenOcclusion = RND((DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_6) * occlRate);
	//rangeOfParticleGenerationWhenNormal = RND((DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_6) * norRate);
	//}
	//else*/ if (currentArea <= (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_5) * (DEFAULT_WINDOW_HEIGHT / IMG_LEVEL_RESIZE_RATE_5))
	//{
	//	distanceBetweenTargetsThreshold = RND((DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_5) * 0.5);
	//	rangeOfParticleGenerationWhenOcclusion = RND((DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_5) * occlRate);
	//	rangeOfParticleGenerationWhenNormal = RND((DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_5) * norRate);
	//}
	//else if (currentArea <= (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_4) * (DEFAULT_WINDOW_HEIGHT / IMG_LEVEL_RESIZE_RATE_4))
	//{
	//	distanceBetweenTargetsThreshold = RND((DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_4) * 0.5);
	//	rangeOfParticleGenerationWhenOcclusion = RND((DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_4) * occlRate);
	//	rangeOfParticleGenerationWhenNormal = RND((DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_4) * norRate);
	//}
	//else if (currentArea <= (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_3) * (DEFAULT_WINDOW_HEIGHT / IMG_LEVEL_RESIZE_RATE_3))
	//{
	//	distanceBetweenTargetsThreshold = RND((DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_3) * 0.5);
	//	rangeOfParticleGenerationWhenOcclusion = RND((DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_3) * occlRate);
	//	rangeOfParticleGenerationWhenNormal = RND((DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_3) * norRate);
	//}
	//else if (currentArea <= (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_2) * (DEFAULT_WINDOW_HEIGHT / IMG_LEVEL_RESIZE_RATE_2))
	//{
	//	distanceBetweenTargetsThreshold = RND((DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_2) * 0.5);
	//	rangeOfParticleGenerationWhenOcclusion = RND((DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_2) * occlRate);
	//	rangeOfParticleGenerationWhenNormal = RND((DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_2) * norRate);
	//}
	//else if (currentArea <= (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_1) * (DEFAULT_WINDOW_HEIGHT / IMG_LEVEL_RESIZE_RATE_1))
	//{
	//	distanceBetweenTargetsThreshold = RND((DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_1) * 0.5);
	//	rangeOfParticleGenerationWhenOcclusion = RND((DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_1) * occlRate);
	//	rangeOfParticleGenerationWhenNormal = RND((DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_1) * norRate);
	//}
	//else if (currentArea <= (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_0) * (DEFAULT_WINDOW_HEIGHT / IMG_LEVEL_RESIZE_RATE_0))
	//{
	//	distanceBetweenTargetsThreshold = RND((DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_0) * 0.5);
	//	rangeOfParticleGenerationWhenOcclusion = RND((DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_0) * occlRate);
	//	rangeOfParticleGenerationWhenNormal = RND((DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_0) * norRate);
	//}
	//else
	//{
	//	distanceBetweenTargetsThreshold = RND((DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_0) * 0.5);
	//	rangeOfParticleGenerationWhenOcclusion = RND((DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_0) * occlRate);
	//	rangeOfParticleGenerationWhenNormal = RND((DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_0) * norRate);
	//}

	// 움직이는 방향 추정
	//for(int i = 0; i < 2; i++)
	//{
	//	intervalX += velocity[i][0];
	//	intervalY += velocity[i][1];
	//}

	//intervalX = (int)(intervalX * 0.5);
	//intervalY = (int)(intervalY * 0.5);

	//// 타겟의 움직임 방향이 대각선
	//// 파티클을 구형으로 샘플링
	//if(0.5 <= intervalX / (double)intervalY && intervalX / (double)intervalY <= 1.5)
	//{
	//	particleDistributionFlag = 0;
	//}
	//// 타겟의 움직임 방향이 수평
	//// 파티클을 수평으로 샘플링
	//else if(intervalX >= intervalY)
	//{
	//	particleDistributionFlag = 1;
	//}
	//// 타겟의 움직임 방향이 수직
	//// 파티클을 수직으로 샘플링
	//else if(intervalX < intervalY)
	//{
	//	particleDistributionFlag = -1;
	//}

	/*if (currentArea <= (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_5) * (DEFAULT_WINDOW_HEIGHT / IMG_LEVEL_RESIZE_RATE_5))
	{
		distanceBetweenTargetsThreshold = RND((DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_5) * 0.5);
		rangeOfParticleGenerationWhenOcclusion = RND((DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_5) * occlRate);
		rangeOfParticleGenerationWhenNormal = RND((DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_5) * norRate);
	}
	else if (currentArea <= (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_4) * (DEFAULT_WINDOW_HEIGHT / IMG_LEVEL_RESIZE_RATE_4))
	{
		distanceBetweenTargetsThreshold = RND((DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_4) * 0.5);
		rangeOfParticleGenerationWhenOcclusion = RND((DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_4) * occlRate);
		rangeOfParticleGenerationWhenNormal = RND((DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_4) * norRate);
	}
	else if (currentArea <= (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_3) * (DEFAULT_WINDOW_HEIGHT / IMG_LEVEL_RESIZE_RATE_3))
	{
		distanceBetweenTargetsThreshold = RND((DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_3) * 0.5);
		rangeOfParticleGenerationWhenOcclusion = RND((DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_3) * occlRate);
		rangeOfParticleGenerationWhenNormal = RND((DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_3) * norRate);
	}
	else if (currentArea <= (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_2) * (DEFAULT_WINDOW_HEIGHT / IMG_LEVEL_RESIZE_RATE_2))
	{
		distanceBetweenTargetsThreshold = RND((DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_2) * 0.5);
		rangeOfParticleGenerationWhenOcclusion = RND((DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_2) * occlRate);
		rangeOfParticleGenerationWhenNormal = RND((DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_2) * norRate);
	}
	else if (currentArea <= (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_1) * (DEFAULT_WINDOW_HEIGHT / IMG_LEVEL_RESIZE_RATE_1))
	{
		distanceBetweenTargetsThreshold = RND((DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_1) * 0.5);
		rangeOfParticleGenerationWhenOcclusion = RND((DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_1) * occlRate);
		rangeOfParticleGenerationWhenNormal = RND((DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_1) * norRate);
	}
	else if (currentArea <= (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_0) * (DEFAULT_WINDOW_HEIGHT / IMG_LEVEL_RESIZE_RATE_0))
	{
		distanceBetweenTargetsThreshold = RND((DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_0) * 0.5);
		rangeOfParticleGenerationWhenOcclusion = RND((DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_0) * occlRate);
		rangeOfParticleGenerationWhenNormal = RND((DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_0) * norRate);
	}
	else
	{
		distanceBetweenTargetsThreshold = RND((DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_0) * 0.5);
		rangeOfParticleGenerationWhenOcclusion = RND((DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_0) * occlRate);
		rangeOfParticleGenerationWhenNormal = RND((DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_0) * norRate);
	}*/


	// 객체 간의 겹침 임계값을 윈도우 크기에 따라 설정
	threshold = distanceBetweenTargetsThreshold;
	//	threshold = 0.5;

	rangeOfParticleGenerationWhenNormal = w * norRate;
	rangeOfParticleGenerationWhenOcclusion = w * occlRate;

	// 다른 객체와 거리가 가까운 경우
	srand((unsigned)GetTickCount() + (unsigned)getTrackerIndex());
	if (distanceBetweenTargets < distanceBetweenTargetsThreshold)
	{
		createParticlesForObject(flag, xPoint, yPoint, particle1, rangeOfParticleGenerationWhenOcclusion, particleDistributionFlag);
	}
	// 다른 객체와 거리가 먼 경우
	else
	{
		createParticlesForObject(flag, xPoint, yPoint, particle1, rangeOfParticleGenerationWhenNormal, particleDistributionFlag);
	}

	// 파티클 샘플링 후 가속도 적용
	// 처음 tracker가 할당될 때는 적용하지 않음
	if (!flag)
	{
		createParticleWithVelocity(currentParticles, velocity);
	}
}

/**
@brief			가우시안 분포를 이용하여 파티클 샘플링
@param			flag : 새로 할당된 tracker -> true, 기존에 할당된 tracker -> false
xPoint, yPoint : 타겟 또는 매칭된 detection 윈도우의 중심 좌표
particle1 : 현재 tracker의 파티클 정보 저장
std : 파티클이 샘플링 되는 범위 (분산)
*/
void CTracker::createParticlesForObject(bool flag, const int& xPoint, const int& yPoint, PARTICLEINFO* particle1, int std, int particleDistributionFlag)
{
	register int i, j;
	int sumX = 0, sumY = 0;
	double mean = 0;
	double stDeviation = 0;
	double range = 0;

	// tracker가 할당됐을 경우
	if (flag)
	{
		j = 1;

		if (xPoint <= particle1[0].w)
		{
			particle1[0].x = particle1[0].w + 1;
		}
		else if (xPoint >= IMAGE_WIDTH - particle1[0].w)
		{
			particle1[0].x = IMAGE_WIDTH - particle1[0].w - 1;
		}
		else
		{
			particle1[0].x = xPoint;
		}

		if (yPoint <= particle1[0].h)
		{
			particle1[0].y = particle1[0].h + 1;
		}
		else if (yPoint >= IMAGE_HEIGHT - particle1[0].h)
		{
			particle1[0].y = IMAGE_HEIGHT - particle1[0].h - 1;
		}
		else
		{
			particle1[0].y = yPoint;
		}
	}
	// tracker가 추적을 수행 중인 경우 상위 N개는 그대로 둠
	else
	{
		j = (int)(PARTICLE_NUMBER * PARTICLE_RATE);
		j = 1;
	}

	for (i = j; i < PARTICLE_NUMBER; i++)
	{
		mean = 0;
		stDeviation = std;
		range = 1;

		particle1[i].w = particle1[0].w;
		particle1[i].h = particle1[0].h;

		////		if(i < j + (PARTICLE_NUMBER * PARTICLE_RATE))
		//		if(i < j + (PARTICLE_NUMBER * 0.1))
		//		{
		//			sumX = (int)(xPoint + (GaussianRand(mean, stDeviation * RANGE_RATE)));
		//			sumY = (int)(yPoint + (GaussianRand(mean, stDeviation * RANGE_RATE)));
		//		}
		//		else
		{
			//	sumX = (int)(xPoint + (GaussianRand(mean, stDeviation)));
			//	sumY = (int)(yPoint + (GaussianRand(mean, stDeviation)));
		}

		// 타겟의 움직임 방향이 대각선
		// 파티클을 구형으로 샘플링
		if (particleDistributionFlag == 0)
		{
			sumX = (int)(xPoint + (GaussianRand(mean, stDeviation)));
			sumY = (int)(yPoint + (GaussianRand(mean, stDeviation)));
		}
		// 타겟의 움직임 방향이 수평
		// 파티클을 수평으로 샘플링
		else if (particleDistributionFlag == 1)
		{
			sumX = (int)(xPoint + (GaussianRand(mean, stDeviation)));
			sumY = (int)(yPoint + (GaussianRand(mean, stDeviation) * 0.4));
		}
		// 타겟의 움직임 방향이 수직
		// 파티클을 수직으로 샘플링
		else if (particleDistributionFlag == -1)
		{
			sumX = (int)(xPoint + (GaussianRand(mean, stDeviation) * 0.4));
			sumY = (int)(yPoint + (GaussianRand(mean, stDeviation)));
		}
		// 0, 1, -1 값이 아니면 구형으로 샘플링
		else
		{
			sumX = (int)(xPoint + (GaussianRand(mean, stDeviation)));
			sumY = (int)(yPoint + (GaussianRand(mean, stDeviation)));
		}


		if (sumX <= particle1[i].w)
		{
			particle1[i].x = particle1[i].w + 1;
		}
		else if (sumX >= IMAGE_WIDTH - particle1[i].w)
		{
			particle1[i].x = IMAGE_WIDTH - particle1[i].w - 1;
		}
		else
		{
			particle1[i].x = sumX;
		}

		if (sumY <= particle1[i].h)
		{
			particle1[i].y = particle1[i].h + 1;
		}
		else if (sumY >= IMAGE_HEIGHT - particle1[i].h)
		{
			particle1[i].y = IMAGE_HEIGHT - particle1[i].h - 1;
		}
		else
		{
			particle1[i].y = sumY;
		}
	}
}

/**
@brief			가우시안 분포 생성
파티클을 가우시안 분포로 샘플링
@param			mean : 평균
std : 분산
@return			가우시안 분포로 추정된 수
*/
double CTracker::GaussianRand(const double& mean, const double& std)
{
	double x1, x2, radius, factor, y1;
	static double y2;
	static int use_last = 0;

	if (use_last)
	{
		y1 = y2;
		use_last = 0;
	}
	else
	{
		do {
			x1 = (2.0 * rand() / RAND_MAX - 1.0); // JMR x1 = (2.0 * rand() / RAND_MAX - 1.0); 
			x2 = (2.0 * rand() / RAND_MAX - 1.0); // JMR x2 = (2.0 * rand() / RAND_MAX - 1.0);
			radius = x1 * x1 + x2 * x2;
		} while (radius < 0.00000001 || radius >= 1.0);

		factor = sqrt((-2.0 * log(radius)) / radius);

		y1 = x1 * factor;
		y2 = x2 * factor;

		use_last = 1;
	}

	return (mean + y1*std);
}

/**
@brief			샘플링된 파티클 위치에 가속도를 더해줌
@param			particle1 : 현재 tracker의 파티클 정보 저장
velocity : t, t-1 프레임의 가속도 정보 저장
*/
void CTracker::createParticleWithVelocity(PARTICLEINFO* particle1, int velocity[2][2])
{
	register int i;

	int sumX = 0;
	int	sumY = 0;

	int intervalX = 0;
	int intervalY = 0;

	for (i = 0; i < 2; i++)
	{
		intervalX += velocity[i][0];
		intervalY += velocity[i][1];
	}

	intervalX = (int)(intervalX * 0.5);
	intervalY = (int)(intervalY * 0.5);

	for (i = 0; i < PARTICLE_NUMBER; i++)
	{
		sumX = particle1[i].x + intervalX;

		if (sumX <= particle1[i].w)
		{
			particle1[i].x = particle1[i].w + 1;
		}
		else if (sumX >= IMAGE_WIDTH - particle1[i].w)
		{
			particle1[i].x = IMAGE_WIDTH - particle1[i].w - 1;
		}
		else
		{
			particle1[i].x = sumX;
		}

		sumY = particle1[i].y + intervalY;

		if (sumY <= particle1[i].h)
		{
			particle1[i].y = particle1[i].h + 1;
		}
		else if (sumY >= IMAGE_HEIGHT - particle1[i].h)
		{
			particle1[i].y = IMAGE_HEIGHT - particle1[i].h - 1;
		}
		else
		{
			particle1[i].y = sumY;
		}
	}
}

/**
@brief			Random forest 학습
@param			dib1 : Gray Image
dib2 : HSI Image
particle1 : 현재 tracker의 파티클 정보 저장
df_ocslbp : OCS-LBP 특징을 사용하는 분류기
df_color : Hue 특징을 사용하는 분류기
@return			입력된 특징 벡터에 의해 추정된 확률값
*/
void CTracker::RFtraining(CDib& dib, CDib& dib2, PARTICLEINFO* particle1, RANDOMFERNS& rf1, RANDOMFERNS& rf2, bool flag1, bool flag2)
{
	//	register int i/*, j, k*/;
	//
	//	int theNumberOfDataPerClass_ocslbp[2] = {0, };
	//	int theNumberOfDataPerClass_lid[2] = {0, };
	//
	//	int theNumberOfDataPerClass[2] = {0, };
	//
	//	double dist = 0.0;
	//
	//	int pos_count = 0;
	//	int neg_count = 0;
	//
	//	int pos_th = 0;
	//	int neg_th = 4;
	//
	//	int neg_left = 0, neg_right = 0, neg_top = 0, neg_bottom = 0;
	//
	//	//for(i = 0; i < PARTICLE_NUMBER; i++)
	//	//{
	//	//	dist = sqrt((double)((particle1[0].x - particle1[i].x) * (particle1[0].x - particle1[i].x)) 
	//	//		+ ((particle1[0].y - particle1[i].y) * (particle1[0].y - particle1[i].y)));
	//
	//	//	// positive class
	//	//	if(dist < particle1[0].h * DIST_RATE && pos_count < pos_th)
	//	//	{
	//	//		trainingDataExtractionForOCSLBP(dib, particle1[i], trainDataSet_ocslbp[0], theNumberOfDataPerClass_ocslbp[0]);
	//	//		trainingDataExtractionForColor(dib, particle1[i], trainDataSet_lid[0], theNumberOfDataPerClass_lid[0]);
	//
	//	//		pos_count++;
	//	//	}
	//	//		
	//
	//	//	if(pos_count == pos_th)
	//	//	{
	//	//		break;
	//	//	}
	//	//}
	//
	//	//for(i = PARTICLE_NUMBER - 1; i >= 0; i--)
	//	//{
	//	//	dist = sqrt((double)((particle1[0].x - particle1[i].x) * (particle1[0].x - particle1[i].x)) 
	//	//		+ ((particle1[0].y - particle1[i].y) * (particle1[0].y - particle1[i].y)));
	//
	//	//	// negative class
	//	//	if(dist > particle1[0].h * DIST_RATE && neg_count < neg_th)
	//	//	{
	//	//		trainingDataExtractionForOCSLBP(dib, particle1[i], trainDataSet_ocslbp[1], theNumberOfDataPerClass_ocslbp[1]);
	//	//		trainingDataExtractionForColor(dib, particle1[i], trainDataSet_lid[1], theNumberOfDataPerClass_lid[1]);
	//
	//	//		neg_count++;
	//	//	}
	//
	//	//	if(neg_count == neg_th)
	//	//	{
	//	//		break;
	//	//	}
	//	//}
	//
	//	trainingDataExtractionForTraining(dib, particle1[0], trainDataSet_ocslbp[0], trainDataSet_lid[0], theNumberOfDataPerClass[0]);
	//
	////	trainingDataExtractionForOCSLBP(dib, particle1[0], trainDataSet_ocslbp[0], theNumberOfDataPerClass_ocslbp[0]);
	////	trainingDataExtractionForColor(dib, particle1[0], trainDataSet_lid[0], theNumberOfDataPerClass_lid[0]);	
	//
	////	trainingDataExtractionForOCSLBP(dib, particle1[PARTICLE_NUMBER - 1], trainDataSet_ocslbp[1], theNumberOfDataPerClass_ocslbp[1]);
	////	trainingDataExtractionForColor(dib, particle1[PARTICLE_NUMBER - 1], trainDataSet_lid[1], theNumberOfDataPerClass_lid[1]);
	//
	//	for(i = PARTICLE_NUMBER - 1; i >= 0; i--)
	//	{
	//		dist = sqrt((double)((particle1[0].x - particle1[i].x) * (particle1[0].x - particle1[i].x)) 
	//			+ ((particle1[0].y - particle1[i].y) * (particle1[0].y - particle1[i].y)));
	//
	//		// negative class
	//	//	if(dist > particle1[0].h * DIST_RATE && neg_count < neg_th)
	//	//	{
	//	////		trainingDataExtractionForOCSLBP(dib, particle1[i], trainDataSet_ocslbp[1], theNumberOfDataPerClass_ocslbp[1]);
	//	////		trainingDataExtractionForColor(dib, particle1[i], trainDataSet_lid[1], theNumberOfDataPerClass_lid[1]);
	//
	//	//		trainingDataExtractionForTraining(dib, particle1[i], trainDataSet_ocslbp[1], trainDataSet_lid[1], theNumberOfDataPerClass[1]);
	//
	//	//		neg_count++;
	//	//	}
	//
	//	//	if(neg_count == neg_th)
	//	//	{
	//	//		break;
	//	//	}
	//
	//
	//
	//		if(dist > particle1[0].h * DIST_RATE && neg_count < neg_th)
	//		{
	//			if(neg_left == 0 && (particle1[i].x < particle1[0].x && abs(particle1[i].y - particle1[0].y) <= particle1[0].h * 0.1))
	//			{
	//				neg_left = 1;
	//
	//				trainingDataExtractionForTraining(dib, particle1[i], trainDataSet_ocslbp[1], trainDataSet_lid[1], theNumberOfDataPerClass[1]);
	//
	//				neg_count++;
	//			}
	//			else if(neg_right == 0 && (particle1[i].x > particle1[0].x && abs(particle1[i].y - particle1[0].y) <= particle1[0].h * 0.1))
	//			{
	//				neg_right = 1;
	//
	//				trainingDataExtractionForTraining(dib, particle1[i], trainDataSet_ocslbp[1], trainDataSet_lid[1], theNumberOfDataPerClass[1]);
	//
	//				neg_count++;
	//			}
	//			else if(neg_top == 0 && (particle1[i].y < particle1[0].y && abs(particle1[i].x - particle1[0].x) <= particle1[0].w * 0.1))
	//			{
	//				neg_top = 1;
	//
	//				trainingDataExtractionForTraining(dib, particle1[i], trainDataSet_ocslbp[1], trainDataSet_lid[1], theNumberOfDataPerClass[1]);
	//
	//				neg_count++;
	//			}
	//			else if(neg_bottom == 0 && (particle1[i].y > particle1[0].y && abs(particle1[i].x - particle1[0].x) <= particle1[0].w * 0.1))
	//			{
	//				neg_bottom = 1;
	//
	//				trainingDataExtractionForTraining(dib, particle1[i], trainDataSet_ocslbp[1], trainDataSet_lid[1], theNumberOfDataPerClass[1]);
	//
	//				neg_count++;
	//			}
	//		}
	//
	//		if(neg_count == neg_th)
	//		{
	//			break;
	//		}
	//
	//	}
	//
	//
	//
	//	// Random Ferns training using OCS-LBP features
	//	rf.generateRandomFern(rf_ocslbp, trainDataSet_ocslbp, theNumberOfDataPerClass, OCSLBP_FEATURE_SIZE, FEATURE_DEMENSION, 
	//		PART_CLASSES, THE_NUMBER_OF_GOOD_FERN_1, THE_NUMBER_OF_FUNCTION_1, RF_TRACKING_NORMALIZE_FACTOR, RF_TRACKING_CONSTANT, HISTORY_LENGTH, flag1, flag2);
	//			
	//	// Random Ferns training using LID features
	//	rf.generateRandomFern(rf_lid, trainDataSet_lid, theNumberOfDataPerClass, LID_FEATURE_SIZE, FEATURE_DEMENSION + 1, 
	//		PART_CLASSES, THE_NUMBER_OF_GOOD_FERN_2, THE_NUMBER_OF_FUNCTION_2, RF_TRACKING_NORMALIZE_FACTOR, RF_TRACKING_CONSTANT, HISTORY_LENGTH, flag1, flag2);


	register int i;

	int theNumberOfDataPerClass[2] = { 0, };

	int theNumberOfData = 0;

	double dist = 0.0;

	int pos_count = 0;
	int neg_count = 0;

	int pos_th = 0;
	int neg_th = 4;

	int neg_left = 0, neg_right = 0, neg_top = 0, neg_bottom = 0;

	//Positive 영상
	trainingDataExtractionForTraining(dib, dib2, particle1[0], trainDataSet_ocslbp[0], trainDataSet_color[0], theNumberOfDataPerClass[0], 0);

	double thresholdBetweenPreAndCurr = particle1[0].h * DIST_RATE;

	/*if ((vehicleDirection == 1 || vehicleDirection == 2) && USING_THRESHOLDS_ACCORDING_TO_ROTATION)
	{
		thresholdBetweenPreAndCurr = particle1[0].h * 0.8;
	}*/
	m_Frame_Count_nega = 0;
	//	if(flag1)
	{

		for (i = PARTICLE_NUMBER - 1; i >= 0; i--)
		{
			dist = sqrt((double)((particle1[0].x - particle1[i].x) * (particle1[0].x - particle1[i].x))
				+ ((particle1[0].y - particle1[i].y) * (particle1[0].y - particle1[i].y)));


			//if((/*dist > particle1[0].w * DIST_RATE || */dist > particle1[0].h * DIST_RATE) && neg_count < neg_th)
			//{
			//	if(neg_left == 0 && (particle1[i].x < particle1[0].x && abs(particle1[i].y - particle1[0].y) <= particle1[0].h * 0.1))
			//	{
			//		neg_left = 1;
			//
			//		trainingDataExtractionForTraining(dib, particle1[i], trainDataSet_ocslbp[1], trainDataSet_color[1], theNumberOfDataPerClass[1], 1);
			//
			//		neg_count++;
			//	}
			//	else if(neg_right == 0 && (particle1[i].x > particle1[0].x && abs(particle1[i].y - particle1[0].y) <= particle1[0].h * 0.1))
			//	{
			//		neg_right = 1;
			//
			//		trainingDataExtractionForTraining(dib, particle1[i], trainDataSet_ocslbp[1], trainDataSet_color[1], theNumberOfDataPerClass[1], 1);
			//
			//		neg_count++;
			//	}
			//	else if(neg_top == 0 && (particle1[i].y < particle1[0].y && abs(particle1[i].x - particle1[0].x) <= particle1[0].w * 0.1))
			//	{
			//		neg_top = 1;
			//
			//		trainingDataExtractionForTraining(dib, particle1[i], trainDataSet_ocslbp[1], trainDataSet_color[1], theNumberOfDataPerClass[1], 1);
			//
			//		neg_count++;
			//	}
			//	else if(neg_bottom == 0 && (particle1[i].y > particle1[0].y && abs(particle1[i].x - particle1[0].x) <= particle1[0].w * 0.1))
			//	{
			//		neg_bottom = 1;
			//
			//		trainingDataExtractionForTraining(dib, particle1[i], trainDataSet_ocslbp[1], trainDataSet_color[1], theNumberOfDataPerClass[1], 1);
			//
			//		neg_count++;
			//	}
			//}
			//
			//if(neg_count == neg_th)
			//{
			//	break;
			//}


			if (dist > thresholdBetweenPreAndCurr)
			{
				//for(int j = 0; j < 128; j++)
				//{
				//	trainDataSet_ocslbp[1][theNumberOfDataPerClass[1]][j] = particle1[i].OCSLBP_histogram[j];
				//	trainDataSet_color[1][theNumberOfDataPerClass[1]][j] = particle1[i].LID_histogram[j];
				//}

				//	for (int j = 0; j < NEW_FEATURE_SIZE; j++)
				//	{
				//		trainDataSet_ocslbp[1][theNumberOfDataPerClass[1]][j] = particle1[i].new_histogram[j];
				////		trainDataSet_color[1][theNumberOfDataPerClass[1]][j] = particle1[i].LID_histogram[j];
				//	}
				
				//Negative영상
				if (FEATURE_TYPE == 0)
				{
					for (int j = 0; j < OCSLBP_FEATURE_SIZE; j++)
					{
						trainDataSet_ocslbp[1][theNumberOfDataPerClass[1]][j] = particle1[i].OCSLBP_histogram[j];
						trainDataSet_color[1][theNumberOfDataPerClass[1]][j] = particle1[i].Color_histogram[j];
					}

					//////////////////
					/////확인용////////
					CDib dib_temp;
					dib_temp.CreateGrayImage(2*particle1[i].w, 2*particle1[i].h, 0);

					BYTE** ptr = dib_temp.GetPtr();

					BYTE** ptr_ori = dib.GetPtr();

					for (int ii = 0, k = particle1[i].y - particle1[i].h; k < particle1[i].y + particle1[i].h; ii++, k++)
					{
						for (int jj = 0, s = particle1[i].x - particle1[i].w; s < particle1[i].x + particle1[i].w; jj++, s++)
						{
							ptr[ii][jj] = ptr_ori[k][s];
						}
					}

					CString str = _T(".\\DATA\\negative\\");

					if (m_Frame_Count_nega < 10)
					{
						str.AppendFormat(_T("000%d.bmp"), m_Frame_Count_nega);
					}
					else if (m_Frame_Count_nega < 100)
					{
						str.AppendFormat(_T("00%d.bmp"), m_Frame_Count_nega);
					}
					else if (m_Frame_Count_nega < 1000)
					{
						str.AppendFormat(_T("0%d.bmp"), m_Frame_Count_nega);
					}
					else if (m_Frame_Count_nega < 10000)
					{
						str.AppendFormat(_T("%d.bmp"), m_Frame_Count_nega);
					}

					dib_temp.Save(str);
					m_Frame_Count_nega++; 

				}
				/*else if (FEATURE_TYPE == 1)
				{
					for (int j = 0; j < NEW_FEATURE_SIZE; j++)
					{
						trainDataSet_ocslbp[1][theNumberOfDataPerClass[1]][j] = particle1[i].new_histogram[j];
					}
				}
				else if (FEATURE_TYPE == 2)
				{
					for (int j = 0; j < NEW_FEATURE_SIZE; j++)
					{
						trainDataSet_ocslbp[1][theNumberOfDataPerClass[1]][j] = particle1[i].new_histogram[j];
					}
				}
				else if (FEATURE_TYPE == 3)
				{
					for (int j = 0; j < NEW_FEATURE_SIZE; j++)
					{
						trainDataSet_ocslbp[1][theNumberOfDataPerClass[1]][j] = particle1[i].new_histogram[j];
					}
				}*/

				theNumberOfDataPerClass[1]++;
			}

		}
	}

	//	flag1 = false;

	// Random Ferns training using OCS-LBP features
	//	rf.generateRandomFern(rf_ocslbp, trainDataSet_ocslbp, theNumberOfDataPerClass, OCSLBP_FEATURE_SIZE, FEATURE_DEMENSION, 
	//		PART_CLASSES, THE_NUMBER_OF_GOOD_FERN_1, THE_NUMBER_OF_FUNCTION_1, RF_TRACKING_NORMALIZE_FACTOR, RF_TRACKING_CONSTANT, HISTORY_LENGTH, flag1, flag2);

	// Random Ferns training using LID features
	//	rf.generateRandomFern(rf_color, trainDataSet_color, theNumberOfDataPerClass, LID_FEATURE_SIZE, FEATURE_DEMENSION, 
	//		PART_CLASSES, THE_NUMBER_OF_GOOD_FERN_2, THE_NUMBER_OF_FUNCTION_2, RF_TRACKING_NORMALIZE_FACTOR, RF_TRACKING_CONSTANT, HISTORY_LENGTH, flag1, flag2);


	//	rf.boostedClassifier(rf_ocslbp, trainDataSet_ocslbp, theNumberOfDataPerClass, NEW_FEATURE_SIZE, FEATURE_DEMENSION,
	//		PART_CLASSES, THE_NUMBER_OF_FERN_1, THE_NUMBER_OF_GOOD_FERN_1, THE_NUMBER_OF_FUNCTION_1, RF_TRACKING_NORMALIZE_FACTOR, RF_TRACKING_CONSTANT, HISTORY_LENGTH, flag1, flag2);

	if (FEATURE_TYPE == 0)
	{
		//	rf.generateRandomFern(rf_color, trainDataSet_color, theNumberOfDataPerClass, LID_FEATURE_SIZE, FEATURE_DEMENSION,
		//		PART_CLASSES, THE_NUMBER_OF_GOOD_FERN_2, THE_NUMBER_OF_FUNCTION_2, RF_TRACKING_NORMALIZE_FACTOR, RF_TRACKING_CONSTANT, HISTORY_LENGTH, flag1, flag2);

		rf.boostedClassifier(rf_color, trainDataSet_color, theNumberOfDataPerClass, COLOR_FEATURE_SIZE_FOR_TRACKING, FEATURE_DEMENSION,
			PART_CLASSES, THE_NUMBER_OF_FERN_2, THE_NUMBER_OF_GOOD_FERN_2, THE_NUMBER_OF_FUNCTION_2, RF_TRACKING_NORMALIZE_FACTOR, RF_TRACKING_CONSTANT, HISTORY_LENGTH, flag1, flag2);

		rf.boostedClassifier(rf_ocslbp, trainDataSet_ocslbp, theNumberOfDataPerClass, NEW_FEATURE_SIZE, FEATURE_DEMENSION,
			PART_CLASSES, THE_NUMBER_OF_FERN_1, THE_NUMBER_OF_GOOD_FERN_1, THE_NUMBER_OF_FUNCTION_1, RF_TRACKING_NORMALIZE_FACTOR, RF_TRACKING_CONSTANT, HISTORY_LENGTH, flag1, flag2);
	}
	/*else if (FEATURE_TYPE == 1)
	{
		rf.boostedClassifier(rf_ocslbp, trainDataSet_ocslbp, theNumberOfDataPerClass, NEW_FEATURE_SIZE, FEATURE_DEMENSION,
			PART_CLASSES, THE_NUMBER_OF_FERN_1, THE_NUMBER_OF_GOOD_FERN_1, THE_NUMBER_OF_FUNCTION_1, RF_TRACKING_NORMALIZE_FACTOR, RF_TRACKING_CONSTANT, HISTORY_LENGTH, flag1, flag2);
	}
	else if (FEATURE_TYPE == 2)
	{
		rf.boostedClassifier(rf_ocslbp, trainDataSet_ocslbp, theNumberOfDataPerClass, NEW_FEATURE_SIZE, FEATURE_DEMENSION,
			PART_CLASSES, THE_NUMBER_OF_FERN_1, THE_NUMBER_OF_GOOD_FERN_1, THE_NUMBER_OF_FUNCTION_1, RF_TRACKING_NORMALIZE_FACTOR, RF_TRACKING_CONSTANT, HISTORY_LENGTH, flag1, flag2);
	}
	else if (FEATURE_TYPE == 3)
	{
		rf.boostedClassifier(rf_ocslbp, trainDataSet_ocslbp, theNumberOfDataPerClass, NEW_FEATURE_SIZE, FEATURE_DEMENSION,
			PART_CLASSES, THE_NUMBER_OF_FERN_1, THE_NUMBER_OF_GOOD_FERN_1, THE_NUMBER_OF_FUNCTION_1, RF_TRACKING_NORMALIZE_FACTOR, RF_TRACKING_CONSTANT, HISTORY_LENGTH, flag1, flag2);
	}*/
	//	rf.boostedClassifier(rf_color, trainDataSet_color, theNumberOfDataPerClass, LID_FEATURE_SIZE, FEATURE_DEMENSION,
	//		PART_CLASSES, THE_NUMBER_OF_FERN_2, THE_NUMBER_OF_GOOD_FERN_2, THE_NUMBER_OF_FUNCTION_2, RF_TRACKING_NORMALIZE_FACTOR, RF_TRACKING_CONSTANT, HISTORY_LENGTH, flag1, flag2);
}

void CTracker::trainingDataExtractionForTraining(CDib& dib, CDib& dib2, PARTICLEINFO& particle1, double** feature1, double** feature2, int& index, int flag)
{
	register int i, j/*, i1, j1*/, t;

	CDib originDib, cpyDib;

	PARTICLEINFO temp;
	temp.w = 0;
	temp.h = 0;
	temp.x = 0;
	temp.y = 0;

	int imgW = dib.GetWidth();
	int imgH = dib.GetHeight();

	int scanningAbsRange_x = 0;
	int scanningAbsRange_y = 0;

	if (flag == 0)
	{
		scanningAbsRange_x = SCANNING_RANGE_X_AT_TRAINING / 2;
		scanningAbsRange_y = SCANNING_RANGE_Y_AT_TRAINING / 2;
	}
	else
	{
		scanningAbsRange_x = 4;
		scanningAbsRange_y = 4;
	}

	int currentArea = (particle1.w * 2) * (particle1.h * 2);
	double trackerSizeRatio = 0.0;
	double calRatio = 0.0;
	//	int movingInterval = (int)(((particle1.w * 2) * 0.1) * 0.5);
	int movingInterval = (int)((particle1.w * 2) * 0.05);

	m_Frame_Count_posi = 0;
#pragma region Translation
	for (i = -scanningAbsRange_y; i <= scanningAbsRange_y; i++)
	{
		for (j = -scanningAbsRange_x; j <= scanningAbsRange_x; j++)
		{
			temp.y = particle1.y + (i * movingInterval);
			temp.x = particle1.x + (j * movingInterval);
			temp.w = particle1.w;
			temp.h = particle1.h;

			// window가 영상 내에 있으면
			if ((temp.y - temp.h >= 0 && temp.y + temp.h < imgH) && (temp.x - temp.w >= 0 && temp.x + temp.w < imgW))
			{
				// OCS-LBP feature extraction
				//	getOCSLBPfeature(dib, temp);

				// Color feature extraction
				//	getColorfeature(dib, temp);

				// two features extraction

				if (FEATURE_TYPE == 0)
				{
					getTwoFeatures(dib, dib2, temp);

					for (t = 0; t < OCSLBP_FEATURE_SIZE; t++)
					{
						feature1[index][t] = temp.OCSLBP_histogram[t];

						feature2[index][t] = temp.Color_histogram[t];
					}


					////////////////
					/////확인용//////
					////////////////
					CDib dib_temp;
					dib_temp.CreateGrayImage(2*temp.w, 2*temp.h, 0);

					BYTE** ptr = dib_temp.GetPtr();

					BYTE** ptr_ori = dib.GetPtr();

					for (int ii = 0, i = temp.y - temp.h; i < temp.y + temp.h; ii++, i++)
					{
						for (int jj = 0, j = temp.x - temp.w; j < temp.x + temp.w; jj++, j++)
						{
							ptr[ii][jj] = ptr_ori[i][j];
						}
					}
		
					CString str = _T(".\\DATA\\positive\\");

					if (m_Frame_Count_posi < 10)
					{
						str.AppendFormat(_T("000%d.bmp"), m_Frame_Count_posi);
					}
					else if (m_Frame_Count_posi < 100)
					{
						str.AppendFormat(_T("00%d.bmp"), m_Frame_Count_posi);
					}
					else if (m_Frame_Count_posi < 1000)
					{
						str.AppendFormat(_T("0%d.bmp"), m_Frame_Count_posi);
					}
					else if (m_Frame_Count_posi < 10000)
					{
						str.AppendFormat(_T("%d.bmp"), m_Frame_Count_posi);
					}

					dib_temp.Save(str);

					m_Frame_Count_posi++;
				}

				/*else if (FEATURE_TYPE == 1)
				{
					getTwoFeatures2(dib, temp);

					for (t = 0; t < NEW_FEATURE_SIZE; t++)
					{
						feature1[index][t] = temp.new_histogram[t];
					}
				}
				else if (FEATURE_TYPE == 2)
				{
					getTwoFeatures3(dib, temp);

					for (t = 0; t < NEW_FEATURE_SIZE; t++)
					{
						feature1[index][t] = temp.new_histogram[t];
					}
				}
				else if (FEATURE_TYPE == 3)
				{
					getTwoFeatures4(dib, temp);

					for (t = 0; t < NEW_FEATURE_SIZE; t++)
					{
						feature1[index][t] = temp.new_histogram[t];
					}
				}
*/

				//for(t = 0; t < OCSLBP_FEATURE_SIZE; t++)
				//{
				//	feature1[index][t] = temp.OCSLBP_histogram[t];

				//	feature2[index][t] = temp.LID_histogram[t];
				//}

				//for (t = 0; t < NEW_FEATURE_SIZE; t++)
				//{
				//	feature1[index][t] = temp.new_histogram[t];
				//}

				index++;
			}
		}
	}
#pragma endregion

	//	if( (particle1.y - particle1.h >= 0 && particle1.y + particle1.h < imgH) && (particle1.x - particle1.w >= 0 && particle1.x + particle1.w < imgW) )
	{

		//originDib.CreateGrayImage(particle1.w * 2, particle1.h * 2, 0);

		//BYTE** ptr = dib.GetPtr();
		//BYTE** originPtr = originDib.GetPtr();

		//for(i = particle1.y - particle1.h, i1 = 0; i < particle1.y + particle1.h, i1 < particle1.h * 2; i++, i1++)
		//{
		//	for(j = particle1.x - particle1.w, j1 = 0; j < particle1.x + particle1.w, j1 < particle1.w * 2; j++, j1++)
		//	{
		//		originPtr[i1][j1] = ptr[i][j];
		//	}
		//}

#pragma region Rotation
		// rotation
		//for(i = 1; i < 11; i++)
		//{
		//	cpyDib.Copy(&originDib);

		//	DidRotate(cpyDib, i);

		//	temp.w = int(cpyDib.GetWidth() * 0.5);
		//	temp.h = int(cpyDib.GetHeight() * 0.5);
		//	temp.x = temp.w;
		//	temp.y = temp.h;

		//	// OCS-LBP feature extraction
		//	getOCSLBPfeature(cpyDib, temp);

		//	for(t = 0; t < OCSLBP_FEATURE_SIZE; t++)
		//	{
		//		feature[index][t] = temp.OCSLBP_histogram[t];
		//	}

		//	// class labeling
		//	feature[index][t] = classIndex;
		//	
		//	index++;

		//	//=================================================================

		//	cpyDib.Copy(&originDib);

		//	DidRotate(cpyDib, i * -1.0);

		//	temp.w = int(cpyDib.GetWidth() * 0.5);
		//	temp.h = int(cpyDib.GetHeight() * 0.5);
		//	temp.x = temp.w;
		//	temp.y = temp.h;

		//	// OCS-LBP feature extraction
		//	getOCSLBPfeature(cpyDib, temp);

		//	for(t = 0; t < OCSLBP_FEATURE_SIZE; t++)
		//	{
		//		feature[index][t] = temp.OCSLBP_histogram[t];
		//	}

		//	// class labeling
		//	feature[index][t] = classIndex;

		//	index++;
		//}
#pragma endregion

#pragma region Scale change
		// scale change
		//for(i = 1; i < 11; i++)
		//{
		//	temp.w = int(particle1.w * (1.0 + (i * 0.01)));
		//	temp.h = int(particle1.h * (1.0 + (i * 0.01)));
		//	temp.x = particle1.x;
		//	temp.y = particle1.y;

		//	// window가 영상 내에 있으면
		//	if( (temp.y - temp.h >= 0 && temp.y + temp.h < imgH) && (temp.x - temp.w >= 0 && temp.x + temp.w < imgW) )
		//	{
		//		// OCS-LBP feature extraction
		//		getOCSLBPfeature(dib, temp);
		//		
		//		// Color feature extraction
		//		getColorfeature(dib, temp);

		//		for(t = 0; t < OCSLBP_FEATURE_SIZE; t++)
		//		{
		//			feature1[index][t] = temp.OCSLBP_histogram[t];

		//			feature2[index][t] = temp.LID_histogram[t];
		//		}
		//		
		//		index++;
		//	}

		//	//=================================================================

		//	temp.w = int(particle1.w * (1.0 + (i * -0.01)));
		//	temp.h = int(particle1.h * (1.0 + (i * -0.01)));
		//	temp.x = particle1.x;
		//	temp.y = particle1.y;

		//	// window가 영상 내에 있으면
		//	if( (temp.y - temp.h >= 0 && temp.y + temp.h < imgH) && (temp.x - temp.w >= 0 && temp.x + temp.w < imgW) )
		//	{
		//		// OCS-LBP feature extraction
		//		getOCSLBPfeature(dib, temp);
		//		
		//		// Color feature extraction
		//		getColorfeature(dib, temp);

		//		for(t = 0; t < OCSLBP_FEATURE_SIZE; t++)
		//		{
		//			feature1[index][t] = temp.OCSLBP_histogram[t];

		//			feature2[index][t] = temp.LID_histogram[t];
		//		}
		//		
		//		index++;
		//	}
		//}
#pragma endregion
	}
}

void CTracker::getTwoFeatures(CDib& dib, CDib& dib2, PARTICLEINFO& particle)
{
	register int i, j, i1, j1;

	BYTE** ptr = dib.GetPtr();
	RGBBYTE** rgbptr = dib2.GetRGBPtr();

	int imgW = IMAGE_WIDTH;
	int imgH = IMAGE_HEIGHT;

	int left_w = particle.x - particle.w;
	int top_w = particle.y - particle.h;
	int right_w = particle.x + particle.w;
	int bottom_w = particle.y + particle.h;

	int left_b = 0;
	int top_b = 0;
	int right_b = 0;
	int bottom_b = 0;

	int region = 0;
	int region_for_color = 0;

	const int SUB_REGION = SUB_BLOCK;
	const int FEATURE_SIZE = FEATURE_DEMENSION;
	const int FEATURE_SIZE_FOR_COLOR = FEATURE_COLOR_DEMENSION;

	// OCS-LBP =============================================================
	const int OCSLBP_SIZE = OCSLBP_FEATURE_SIZE;
	int t = OCSLBP_THRESHOLD;

	int quarterHeight = (int)(particle.h  * 0.5);
	int quarterWidth = (int)(particle.w  * 0.5);

	memset(particle.OCSLBP_histogram, 0, sizeof(double) * OCSLBP_SIZE);

	// Color ===============================================================
	const int COLOR_SIZE = COLOR_FEATURE_SIZE_FOR_TRACKING;

	double halfHeight = particle.h;
	double halfWidth = particle.w;

	int cY = (int)(top_w + halfHeight);
	int cX = (int)(left_w + halfWidth);

	// the normalised coordinates
	double dX = 0.0;
	double dY = 0.0;
	double dValue = 0.0;
	double eK = 0.0;
	int brX = 0;
	int bgX = 0;
	int bbX = 0;

	memset(particle.Color_histogram, 0, sizeof(double) * COLOR_SIZE);

	// feature extraction ==================================================
	for (i = 0; i < 4; i++)
	{
		for (j = 0; j < 4; j++)
		{
			top_b = top_w + (quarterHeight * i) + 1;
			left_b = left_w + (quarterWidth * j) + 1;
			right_b = (left_b - 1) + quarterWidth - 1;
			bottom_b = (top_b - 1) + quarterHeight - 1;

			region = (i * 4 + j) * FEATURE_SIZE;
			region_for_color = (i * 4 + j) * FEATURE_SIZE_FOR_COLOR;

			//		cY = (int)((top_b + bottom_b) * 0.5);
			//		cX = (int)((left_b + right_b) * 0.5);

			// feature extraction
			if (bottom_b < dib.GetHeight())
			for (i1 = top_b; i1 < bottom_b; i1++)
			{
				if (i1 < 1 || i1 >= imgH - 1)
				{
					continue;
				}

				for (j1 = left_b; j1 < right_b; j1++)
				{
					if (j1 < 1 || j1 >= imgW - 1)
					{
						continue;
					}

					// OCS-LBP ===========================================================
					if (ptr[i1][j1 + 1] - ptr[i1][j1 - 1] >= t)										// 0
					{
						particle.OCSLBP_histogram[region + 0] += 1;
					}
					else if (ptr[i1][j1 - 1] - ptr[i1][j1 + 1] >= t)									// 4
					{
						particle.OCSLBP_histogram[region + 4] += 1;
					}

					if (ptr[i1 + 1][j1 + 1] - ptr[i1 - 1][j1 - 1] >= t)								// 1
					{
						particle.OCSLBP_histogram[region + 1] += 1;
					}
					else if (ptr[i1 - 1][j1 - 1] - ptr[i1 + 1][j1 + 1] >= t)							// 5
					{
						particle.OCSLBP_histogram[region + 5] += 1;
					}

					if (ptr[i1 + 1][j1] - ptr[i1 - 1][j1] >= t)										// 2
					{
						particle.OCSLBP_histogram[region + 2] += 1;
					}
					else if (ptr[i1 - 1][j1] - ptr[i1 + 1][j1] >= t)									// 6
					{
						particle.OCSLBP_histogram[region + 6] += 1;
					}

					if (ptr[i1 + 1][j1 - 1] - ptr[i1 - 1][j1 + 1] >= t)								// 3
					{
						particle.OCSLBP_histogram[region + 3] += 1;
					}
					else if (ptr[i1 - 1][j1 + 1] - ptr[i1 + 1][j1 - 1] >= t)							// 7
					{
						particle.OCSLBP_histogram[region + 7] += 1;
					}

					// Color =============================================================
					// the Epanechnikov kernel
					dY = (i1 - cY) / halfHeight;
					dX = (j1 - cX) / halfWidth;
					//		dY		= (i1 - cY) / (quarterHeight * 0.5);
					//		dX		= (j1 - cX) / (quarterWidth * 0.5);	
					dValue = (dX * dX) + (dY * dY);

					if (1.0 >= dValue)
					{
						eK = 1.0 - dValue;
					}
					else
					{
						eK = 0.0;
					}

					// intensity 특징 추출
					brX = rgbptr[i1][j1].r / INTENSITY_QUANTITY;
					bgX = rgbptr[i1][j1].g / INTENSITY_QUANTITY;
					bbX = rgbptr[i1][j1].b / INTENSITY_QUANTITY;

					particle.Color_histogram[region_for_color + brX] += eK;
					particle.Color_histogram[region_for_color + COLOR_DISTANCE + bgX] += eK;
					particle.Color_histogram[region_for_color + 2 * COLOR_DISTANCE + bbX] += eK;

					//	bX = ptr[i1][j1] / 2;

					//	particle.LID_histogram[bX] += eK;

					brX = 0, bgX = 0, bbX = 0, dX = 0.0, dY = 0.0, eK = 0.0, dValue = 0.0;
				}
			}
		}
	}

	// min-max normalization
	//	minMaxNormalize(particle.OCSLBP_histogram, OCSLBP_SIZE);
	//	minMaxNormalize(particle.LID_histogram, LID_SIZE);


	L2Normalization(particle.OCSLBP_histogram, OCSLBP_SIZE);
	L2Normalization(particle.Color_histogram, COLOR_SIZE);

	//	totalNormalization(particle.OCSLBP_histogram, OCSLBP_SIZE);
	//	totalNormalization(particle.LID_histogram, LID_SIZE);
}

void CTracker::L2Normalization(double* histogram, int histBins)
{
	register int i;

	double temp = 0.0;
	double dL2norm = 0.0;

	for (i = 0; i < histBins; i++)
	{
		temp += histogram[i] * histogram[i];
	}

	dL2norm = sqrt(temp) + 0.2;

	for (i = 0; i < histBins; i++)
	{
		histogram[i] = histogram[i] / dL2norm;
	}

}

/**
@brief			특징 추출 (OCS-LBP, Hue)
모든 파티클 대상
@param			dib1 : Gray Image
dib2 : HSI Image
particle1 : 현재 tracker의 파티클 정보 저장
*/
void CTracker::extractFeature(CDib& dib, CDib& dib2, PARTICLEINFO* particle1)
{
	register int i;

	double ocslbp_probability = 0.0;
	double color_probability = 0.0;

	for (i = 0; i<PARTICLE_NUMBER; i++)
	{
		ocslbp_probability = 0.0;
		color_probability = 0.0;

		//	extractFeatureAtPartForOCSLBP(dib, particle1[i], ocslbp_probability);

		//	extractFeatureAtPartForColor(dib, particle1[i], color_probability);

		extractFeatureAtPartForTwoFeatures(dib, dib2, particle1[i]/*, ocslbp_probability, color_probability*/);

		//	particle1[i].probability = OCSLBP_WEIGHT * ocslbp_probability + LID_WEIGHT * color_probability;

		//	particle1[i].probability = ocslbp_probability;

		if (FEATURE_TYPE == 0)
		{
			particle1[i].probability = OCSLBP_WEIGHT * particle1[i].OCSLBP_distance + LID_WEIGHT * particle1[i].Color_distance;
			
		}
		//Trace(_T("OCSLBP - distance - %lf, Color_ Distance - %lf \n"), particle1[i].OCSLBP_distance, particle1[i].Color_distance);
	}
}

void CTracker::extractFeatureAtPartForTwoFeatures(CDib& dib, CDib& dib2, PARTICLEINFO& particle/*, double& firstProbability, double& secondProbability*/)
{
	//	getTwoFeatures(dib, particle);
	//	firstProbability = RFtesting(particle.OCSLBP_histogram, rf_ocslbp);
	//	secondProbability = RFtesting(particle.LID_histogram, rf_color);


	if (FEATURE_TYPE == 0)
	{
		getTwoFeatures(dib, dib2, particle);

#pragma region bhattacharyya 거리 계산
		// bhattacharyya 계수
		double bhattacharyya_coefficient = 0.0;

		int j;

		bhattacharyya_coefficient = 0.0;

		double denom_h0 = 0.0;
		double denom_h1 = 0.0;

		//==================================== OCS - LBP ========================================
		for ( j = 0; j < OCSLBP_FEATURE_SIZE; j++)
		{
			denom_h0 += particle.OCSLBP_histogram[j]; // posi -> particle?
			denom_h1 += Template_OCSLBP_histogram[j]; // nega -> template?
			if (denom_h0 < 0 || denom_h1 < 0)
			{
				Trace(_T("Nega!!"));
			}
		}

		if (denom_h0 != 0 && denom_h1 != 0)
		{

			for (j = 0; j < OCSLBP_FEATURE_SIZE; j++)
			{
				//	bhattacharyya_coefficient += sqrt(temp.fern[0].arrHistoFern[i][j] * temp.fern[1].arrHistoFern[i][j]);
				bhattacharyya_coefficient += (sqrt(particle.OCSLBP_histogram[j] * Template_OCSLBP_histogram[j]) / sqrt(denom_h0 * denom_h1));
			} // end j

			// 거리 비교를 위해 배열에 저장
			particle.OCSLBP_distance = 1.0 - sqrt(1.0 - bhattacharyya_coefficient);
		}
		else
		{
			particle.OCSLBP_distance = 0.0;
		}

		// 변수 초기화
		denom_h0 = 0.0;
		denom_h1 = 0.0;
		bhattacharyya_coefficient = 0.0;

		//==================================== Color ========================================
		for (j = 0; j < COLOR_FEATURE_SIZE_FOR_TRACKING; j++)
		{
			denom_h0 += particle.Color_histogram[j]; // posi -> particle?
			denom_h1 += Template_Color_histogram[j]; // nega -> template?
		}

		if (denom_h0 != 0 && denom_h1 != 0)
		{
			for (j = 0; j < COLOR_FEATURE_SIZE_FOR_TRACKING; j++)
			{
				//	bhattacharyya_coefficient += sqrt(temp.fern[0].arrHistoFern[i][j] * temp.fern[1].arrHistoFern[i][j]);
				bhattacharyya_coefficient += (sqrt(particle.Color_histogram[j] * Template_Color_histogram[j]) / sqrt(denom_h0 * denom_h1));
			} // end j

			particle.Color_distance = 1.0 - sqrt(1.0 - bhattacharyya_coefficient);
		}
		else

		{
			particle.Color_distance = 0.0;
		}
#pragma endregion
	}
}

/**
@brief			Random forest를 이용한 특징의 확률 추정
@param			feature : 특징 벡터
df : 분류기
featureSize : 특징 크기
thenumberofclass : 클래스 수
@return			입력된 특징 벡터에 의해 추정된 확률값
*/
double CTracker::RFtesting(double* input, RANDOMFERNS& rf_classifier)
{
	register int m;

	double Probability = 0.0;

	double* result = new double[PART_CLASSES];

	// 윈도우별로 추출된 특징들을 RF트레이닝된 데이터를 통해서 평가 	
	for (m = 0; m < PART_CLASSES; m++)
	{
		result[m] = 0.0;
	}

	rf.classification(rf_classifier, input, result);

	Probability = 1.0 - (result[0] / (result[0] + result[1]));

	// 메모리 해제
	delete[] result;

	return Probability;
}

/**
@brief			score 값을 기반으로 파티클을 내림차순으로 정렬
@param			particle1 : 현재 tracker의 파티클 정보 저장
particle2 : 이전 또는 추정된 tracker 정보 저장
n : 파티클 수
*/
void CTracker::sortParticles(PARTICLEINFO* particle1, TARGETPARTICLE& particle2, const int& n, int velocity[2][2])
{
	register int i;
	double distanceSum = 0.0;

	int intervalX = 0;
	int intervalY = 0;

	double distThreshold = particle2.w;

	// 차량이 회전하면 거리 임계값을 크게 설정
	// 직진일 경우 임계값은 그대로
	// -1:초기화, 0:직진, 1:좌회전, 2:우회전	
	/*if ((vehicleDirection == 1 || vehicleDirection == 2) && USING_THRESHOLDS_ACCORDING_TO_ROTATION)
	{
	distThreshold *= 1.5;
	}*/

	for (i = 0; i < 2; i++)
	{
		intervalX += velocity[i][0];
		intervalY += velocity[i][1];
	}

	intervalX /= 2;
	intervalY /= 2;

	for (i = 0; i < PARTICLE_NUMBER; i++)
	{
		particle1[i].distance = sqrt((double)((particle1[i].x - (particle2.x + intervalX)) * (particle1[i].x - (particle2.x + intervalX))
			+ (particle1[i].y - (particle2.y + intervalY)) * (particle1[i].y - (particle2.y + intervalY))));

		if (particle1[i].distance <= distThreshold)
		{
			distanceSum += particle1[i].distance;
		}
	}

	for (i = 0; i < PARTICLE_NUMBER; i++)
	{
		if (particle1[i].distance <= distThreshold)
		{
			particle1[i].score = 0.5 * (1.0 - (particle1[i].distance / distanceSum)) + 0.5 * particle1[i].probability;
		}
		else
		{
			particle1[i].score = 0.5 * particle1[i].probability;
		}

			//Trace(_T("%d\t%f\n"), i, particle1[i].score);
	}

	bubbleSort(particle1, PARTICLE_NUMBER);
}

/**
@brief			버블 정렬
@param			particle1 : 현재 tracker의 파티클 정보 저장
n : 파티클 수
*/
void CTracker::bubbleSort(PARTICLEINFO* particle1, const int& n)
{
	register int j, k;
	PARTICLEINFO temp;

	for (j = n - 1; j > 0; j--)
	{
		for (k = 0; k < j; k++)
		{
			if (particle1[k].score < particle1[k + 1].score)
			{
				temp = particle1[k + 1];
				particle1[k + 1] = particle1[k];
				particle1[k] = temp;  //기준이 변경됨
			}
		}
	}
}

/**
@brief			상위 N개의 파티클을 이용하여 새로운 타겟의 상태 추정
연관성 검사 알고리즘 수행 전에 수행됨
@param			particle1 : 추정된 tracker 정보 저장
particle2 : 현재 tracker의 파티클 정보 저장
*/
void CTracker::estimateNewTargetStateUsingUpperParticles(TARGETPARTICLE& particle1, PARTICLEINFO* particle2)
{
	register int i;

	int targetX = 0;
	int targetY = 0;
	int targetW = 0;
	int targetH = 0;

	int aroundCount = 0;

	// 타겟은 상위 N개 파티클의 평균 위치와 크기로 계산된다.
	for (i = 0; i < PARTICLE_NUMBER * PARTICLE_RATE; i++)
	{
		targetX += particle2[i].x;
		targetY += particle2[i].y;

		targetW += particle2[i].w;
		targetH += particle2[i].h;

		aroundCount++;
	}


	particle1.x = (int)(targetX / aroundCount);
	particle1.y = (int)(targetY / aroundCount);

	particle1.w = (int)(targetW / aroundCount);
	particle1.h = (int)(targetH / aroundCount);

	//	particle1.w = particle2[0].w;
	//	particle1.h = particle2[0].h;

	// 위치 제한
	if (particle1.x <= particle1.w)
	{
		particle1.x = particle1.w + 1;
	}
	else if (particle1.x >= IMAGE_WIDTH - particle1.w)
	{
		particle1.x = IMAGE_WIDTH - particle1.w - 1;
	}

	if (particle1.y <= particle1.h)
	{
		particle1.y = particle1.h + 1;
	}
	else if (particle1.y >= IMAGE_HEIGHT - particle1.h)
	{
		particle1.y = IMAGE_HEIGHT - particle1.h - 1;
	}
}


/**
@brief			매칭 된 detection이 없을 경우 이전 tracker 정보를 이용하여 타겟 state 업데이트
@param			particle1 : 추정된 tracker 정보 저장
particle2 : 이전 tracker 정보 저장
@return			업데이트 된 타겟과 이전 tracker 타겟 간의 거리
*/
double CTracker::estimateNewTargetStateUsingPreTrackerInformation(TARGETPARTICLE& particle1, TARGETPARTICLE& particle2)
{
	// 이전 tracker 정보
	int pre_t_cx = particle2.x;
	int pre_t_cy = particle2.y;
	int pre_t_w = particle2.w;
	int pre_t_h = particle2.h;

	// 현재 tracker 정보
	int curr_t_cx = particle1.x;
	int curr_t_cy = particle1.y;
	int curr_t_w = particle1.w;
	int curr_t_h = particle1.h;

	int intervalX = 0;
	int intervalY = 0;

	// 움직이는 방향 추정
	for (int i = 0; i < 2; i++)
	{
		intervalX += velocity[i][0];
		intervalY += velocity[i][1];
	}

	intervalX = (int)(intervalX * 0.5);
	intervalY = (int)(intervalY * 0.5);

	// 이전 tracker 정보를 이용하여 현재 파티클 상태 업데이트
	//	particle1.x = curr_t_cx + intervalX;
	//	particle1.y = curr_t_cy + intervalY;

	particle1.x = curr_t_cx;
	particle1.y = curr_t_cy;

	//	particle1.x = (int)(PRE_TRACKER_WEIGHT * (curr_t_cx + intervalX) + CURR_TRACKER_WEIGHT2 * curr_t_cx);
	//	particle1.y = (int)(PRE_TRACKER_WEIGHT * (curr_t_cy + intervalY) + CURR_TRACKER_WEIGHT2 * curr_t_cy);

	//	particle1.x = (int)(PRE_TRACKER_WEIGHT * pre_t_cx + CURR_TRACKER_WEIGHT2 * curr_t_cx);
	//	particle1.y = (int)(PRE_TRACKER_WEIGHT * pre_t_cy + CURR_TRACKER_WEIGHT2 * curr_t_cy);

	//	particle1.w = curr_t_w;
	//	particle1.h = curr_t_h;

	particle1.w = (int)(PRE_TRACKER_WEIGHT * pre_t_w + CURR_TRACKER_WEIGHT2 * curr_t_w);
	particle1.h = (int)(PRE_TRACKER_WEIGHT * pre_t_h + CURR_TRACKER_WEIGHT2 * curr_t_h);

	// 위치 제한
	if (particle1.x <= particle1.w)
	{
		particle1.x = particle1.w + 1;
	}
	else if (particle1.x >= IMAGE_WIDTH - particle1.w)
	{
		particle1.x = IMAGE_WIDTH - particle1.w - 1;
	}

	if (particle1.y <= particle1.h)
	{
		particle1.y = particle1.h + 1;
	}
	else if (particle1.y >= IMAGE_HEIGHT - particle1.h)
	{
		particle1.y = IMAGE_HEIGHT - particle1.h - 1;
	}

	// 이전 tracker와 거리 계산
	return sqrt(((double)(particle1.x - particle2.x)*(particle1.x - particle2.x))
		+ ((double)(particle1.y - particle2.y) * (particle1.y - particle2.y)));
}

/**
@brief			새로운 타겟의 확률값을 추정하고 파티클 구조체에 저장하는 함수
기존 파티클 구조체의 내용을 뒤로 한 칸씩 이동함
@param			dib1 : Gray Image
dib2 : HSI Image
particle1 : 추정된 tracker 정보 저장
particle2 : 현재 tracker의 파티클 정보 저장
@date			2014-01-07
*/
void CTracker::moveParticleToNextIndex(CDib& dib, CDib& dib2, TARGETPARTICLE& particle1, PARTICLEINFO* particle2)
{
	register int i;

	for (i = PARTICLE_NUMBER - 2; i >= 0; i--)
	{
		particle2[i + 1] = particle2[i];
	}

	particle2[0].x = particle1.x;
	particle2[0].y = particle1.y;
	particle2[0].w = particle1.w;
	particle2[0].h = particle1.h;

	// 파트 위치 초기화
	//	initializePartLocationInOneParticle(particle2[0]);

	// 새 타겟의 특징 추출 후 확률 추정
	extractTargetFeature(dib, dib2, particle2[0]);


	particle1.probability = particle2[0].probability;


	particle1.x = particle2[0].x;
	particle1.y = particle2[0].y;
	particle1.w = particle2[0].w;
	particle1.h = particle2[0].h;
}

/**
@brief			최종 타겟의 특징 추출 후 확률 추정
@param			dib1 : Gray Image
dib2 : HSI Image
particle1 : 현재 tracker의 파티클 정보 저장 (1개)
*/
void CTracker::extractTargetFeature(CDib& dib, CDib& dib2, PARTICLEINFO& particle1)
{
	double ocslbp_probability = 0.0;
	double color_probability = 0.0;

	//	extractTargetFeature_ocslbp(dib, particle1, ocslbp_probability);

	//	extractTargetFeature_color(dib, particle1, color_probability);

	extractFeatureAtPartForTwoFeatures(dib, dib2, particle1/*, ocslbp_probability, color_probability*/);

	//	particle1.probability = OCSLBP_WEIGHT * ocslbp_probability + LID_WEIGHT * color_probability;

	//	particle1.probability = ocslbp_probability;

	if (FEATURE_TYPE == 0)
	{
		particle1.probability = OCSLBP_WEIGHT * particle1.OCSLBP_distance + LID_WEIGHT * particle1.Color_distance;
	}

	//Trace(_T("\n ID.%d. ocs_pro : %f, lid_pro : %f, pro : %f\n"), displayTrackerID, ocslbp_probability, lid_probability, particle1.probability);	
}

/**
@brief			현재 tracker와 다른 tracker 간의 거리 계산
@param			otherTarget : 다른 tracker의 위치 정보 저장
@return			계산된 거리 중 가장 짧은 거리 반환
@date			2014-01-07
*/
int CTracker::measureDistanceBetweenOtherTargets(TARGETPARTICLE& particle, OTHERTARGETLOCATION& otherTarget, double& ratioBetweenObjects)
{
	register int k, i, j;

	//	double distance = 0.0;
	//	double minDistance = 1000.0;

	CDib dib;

	dib.CreateGrayImage(IMAGE_WIDTH, IMAGE_HEIGHT, 0);

	BYTE** ptr = dib.GetPtr();

	int curr_start_y = particle.y - particle.h;
	int curr_end_y = particle.y + particle.h;
	int curr_start_x = particle.x - particle.w;
	int curr_end_x = particle.x + particle.w;

	int other_start_y = 0;
	int other_end_y = 0;
	int other_start_x = 0;
	int other_end_x = 0;

	double overlapCount = 0.0;

	// 최대 겹침율을 구하기 위한 변수
	double minRatioBetweenObjects = 0.0;
	double calRatioBetweenObjects = 0.0;

	int trackerArea = 0;
	int otherArea = 0;

	int trackerLevel = -1;
	int otherLevel = -1;

	//  0 : back
	//  1 : front
	// -1 : 알 수 없음
	int front_back_flag = -1;

	int trackerYlocation = 0;
	int otherYlocation = 0;

	for (k = 0; k < otherTarget.theNumberOfOtherTarget; k++)
	{
		if (otherTarget.targetIndexs[k] == trackerIndex)
		{
			trackerYlocation = otherTarget.avgYaxis[k];
			break;
		}
	}

	for (k = 0; k < otherTarget.theNumberOfOtherTarget; k++)
	{
		if (otherTarget.targetIndexs[k] != trackerIndex)
		{
			overlapCount = 0.0;
			otherArea = 0;
			//distance = measureDistanceBetweenDetectionAndTracking(otherTarget.location[k][0], otherTarget.location[k][1]);

			//if(distance < minDistance)
			//{
			//	minDistance = distance;
			//}

			// 현재 트래커의 이전 3프레임 Y 좌표의 평균
			// 현재 프레임의 Y 좌표는 계산에서 제외함
			//		trackerYlocation = getAvgYaxis();

			trackerArea = (particle.h * 2) * (particle.w * 2);

			//if (trackerArea <= (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_5) * (DEFAULT_WINDOW_HEIGHT / IMG_LEVEL_RESIZE_RATE_5))
			//{
			//	trackerLevel = 5;
			//}
			//else if (trackerArea <= (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_4) * (DEFAULT_WINDOW_HEIGHT / IMG_LEVEL_RESIZE_RATE_4))
			//{
			//	trackerLevel = 4;
			//}
			//else if (trackerArea <= (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_3) * (DEFAULT_WINDOW_HEIGHT / IMG_LEVEL_RESIZE_RATE_3))
			//{
			//	trackerLevel = 3;
			//}
			//else if (trackerArea <= (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_2) * (DEFAULT_WINDOW_HEIGHT / IMG_LEVEL_RESIZE_RATE_2))
			//{
			//	trackerLevel = 2;
			//}
			//else if (trackerArea <= (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_1) * (DEFAULT_WINDOW_HEIGHT / IMG_LEVEL_RESIZE_RATE_1))
			//{
			//	trackerLevel = 1;
			//}
			//else if (trackerArea <= (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_0) * (DEFAULT_WINDOW_HEIGHT / IMG_LEVEL_RESIZE_RATE_0))
			//{
			//	trackerLevel = 0;
			//}
			//else
			//{
			//	trackerLevel = 0;
			//}

			otherYlocation = otherTarget.avgYaxis[k];

			other_start_y = otherTarget.location[k][1] - otherTarget.location[k][3];
			other_end_y = otherTarget.location[k][1] + otherTarget.location[k][3];
			other_start_x = otherTarget.location[k][0] - otherTarget.location[k][2];
			other_end_x = otherTarget.location[k][0] + otherTarget.location[k][2];


			for (i = other_start_y; i < other_end_y; i++)
			{
				for (j = other_start_x; j < other_end_x; j++)
				{
					ptr[i][j] = 1;

					otherArea++;
				}
			}

			//if (otherArea <= (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_5) * (DEFAULT_WINDOW_HEIGHT / IMG_LEVEL_RESIZE_RATE_5))
			//{
			//	otherLevel = 5;
			//}
			//else if (otherArea <= (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_4) * (DEFAULT_WINDOW_HEIGHT / IMG_LEVEL_RESIZE_RATE_4))
			//{
			//	otherLevel = 4;
			//}
			//else if (otherArea <= (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_3) * (DEFAULT_WINDOW_HEIGHT / IMG_LEVEL_RESIZE_RATE_3))
			//{
			//	otherLevel = 3;
			//}
			//else if (otherArea <= (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_2) * (DEFAULT_WINDOW_HEIGHT / IMG_LEVEL_RESIZE_RATE_2))
			//{
			//	otherLevel = 2;
			//}
			//else if (otherArea <= (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_1) * (DEFAULT_WINDOW_HEIGHT / IMG_LEVEL_RESIZE_RATE_1))
			//{
			//	otherLevel = 1;
			//}
			//else if (otherArea <= (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_0) * (DEFAULT_WINDOW_HEIGHT / IMG_LEVEL_RESIZE_RATE_0))
			//{
			//	otherLevel = 0;
			//}
			//else
			//{
			//	otherLevel = 0;
			//}

			for (i = curr_start_y; i < curr_end_y; i++)
			{
				for (j = curr_start_x; j < curr_end_x; j++)
				{
					ptr[i][j] += 1;

					if (ptr[i][j] == 2)
					{
						overlapCount++;
					}
				}
			}

			calRatioBetweenObjects = overlapCount / (double)((curr_end_y - curr_start_y) * (curr_end_x - curr_start_x));


			if (calRatioBetweenObjects > minRatioBetweenObjects)
			{
				ratioBetweenObjects = calRatioBetweenObjects;
				minRatioBetweenObjects = calRatioBetweenObjects;

				// 값이 클 수록 트래커의 크기가 작다.
				// 현재 트래커가 더 크다 -> 앞 쪽에 있다.
				if (trackerYlocation > otherYlocation)
				{
					front_back_flag = 1;
				}
				// 현재 트래커와 다른 트래커의 크기가 같다.
				// 확률값으로 비교
				else if (trackerYlocation == otherYlocation)
				{
					// 겹침이 발생되어 있고, 현재 트래커의 확률값이 더 크다면
					// 현재 트래커는 겹침 상태에서 앞 쪽에 있는 것으로 판단
					if (particle.probability >= otherTarget.targetProbability[k])
						//		if (trackerYlocation >= otherYlocation)
					{
						front_back_flag = 1;
					}
					else
					{
						front_back_flag = 0;
					}
				}
				// 현재 트래커가 더 작다 -> 뒷 쪽에 있다.
				else
				{
					front_back_flag = 0;
				}

			}

			// ptr 초기화
			for (i = other_start_y; i < other_end_y; i++)
			{
				for (j = other_start_x; j < other_end_x; j++)
				{
					ptr[i][j] = 0;
				}
			}

			for (i = curr_start_y; i < curr_end_y; i++)
			{
				for (j = curr_start_x; j < curr_end_x; j++)
				{
					ptr[i][j] = 0;
				}
			}
		}
	}

	// 현재 트래커와 겹치는 트래커가 없다면
	if (minRatioBetweenObjects == 0.0)
	{
		ratioBetweenObjects = 0.0;
		front_back_flag = 1;
	}



	return front_back_flag;
}

void CTracker::setPreviousTrackerLocation(int px, int py)
{
	previousX = px;
	previousY = py;
}

void CTracker::getPreviousTrackerLocation(int &px, int &py)
{
	px = previousX;
	py = previousY;
}

void CTracker::getVelocity(TARGETPARTICLE& particle1, COpticalFlow& opticalFlow, int velocity[2][2], int& velocity_index)
{
	CvPoint cp;

	//int min_x = particle1.x - particle1.w;
	//int min_y = particle1.y - particle1.h;
	//int max_x = particle1.x + particle1.w;
	//int max_y = particle1.y + particle1.h;

	int min_x = particle1.x - particle1.w;
	int max_x = particle1.x + particle1.w;

	//int min_y = particle1.y - (int)(particle1.h * 0.3);	
	//int max_y = particle1.y - (int)(particle1.h * 0.05);

	//int min_y = particle1.y - (int)(particle1.h * 0.25);	
	//int max_y = particle1.y + (int)(particle1.h * 0.25);

	int min_y = particle1.y - particle1.h;
	int max_y = particle1.y + particle1.h;

	//	cp = opticalFlow.getOpticalFlow(min_x, min_y, max_x, max_y, (int)((max_x - min_x) * 0.1), (int)((max_y - min_y) * 0.25));
	//	cp = opticalFlow.getOpticalFlow(min_x, min_y, max_x, max_y, (int)((max_x - min_x) * 0.1), (int)((max_y - min_y) * 0.33), 2.0);
	//	cp = opticalFlow.getOpticalFlow(min_x, min_y, max_x, max_y, (int)((max_x - min_x) * 0.1), (int)((max_y - min_y) * 0.33), 2.0);


	int currentArea = (max_x - min_x) * (max_y - min_y);

	int w = (int)(DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_6);
	int h = (int)(DEFAULT_WINDOW_HEIGHT / IMG_LEVEL_RESIZE_RATE_6);

	double increaseRatio = 2.0;

	/*if (currentArea <= (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_5) * (DEFAULT_WINDOW_HEIGHT / IMG_LEVEL_RESIZE_RATE_5))
	{
		w = (int)((max_x - min_x) * 0.1);
		h = (int)((max_y - min_y) * 0.33); //0.2

		increaseRatio = 2.0;

		size_level = 5;
	}
	else if (currentArea <= (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_4) * (DEFAULT_WINDOW_HEIGHT / IMG_LEVEL_RESIZE_RATE_4))
	{
		w = (int)((max_x - min_x) * 0.1);
		h = (int)((max_y - min_y) * 0.33); //0.2

		increaseRatio = 2.0;

		size_level = 4;
	}
	else if (currentArea <= (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_3) * (DEFAULT_WINDOW_HEIGHT / IMG_LEVEL_RESIZE_RATE_3))
	{
		w = (int)((max_x - min_x) * 0.1);
		h = (int)((max_y - min_y) * 0.33); //0.3

		increaseRatio = 2.0;

		size_level = 3;
	}
	else if (currentArea <= (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_2) * (DEFAULT_WINDOW_HEIGHT / IMG_LEVEL_RESIZE_RATE_2))
	{
		w = (int)((max_x - min_x) * 0.1);
		h = (int)((max_y - min_y) * 0.33); //0.3

		increaseRatio = 2.0;

		size_level = 2;
	}
	else if (currentArea <= (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_1) * (DEFAULT_WINDOW_HEIGHT / IMG_LEVEL_RESIZE_RATE_1))
	{
		w = (int)((max_x - min_x) * 0.1);
		h = (int)((max_y - min_y) * 0.33);

		increaseRatio = 2.0;

		size_level = 1;
	}
	else if (currentArea <= (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_0) * (DEFAULT_WINDOW_HEIGHT / IMG_LEVEL_RESIZE_RATE_0))
	{
		w = (int)((max_x - min_x) * 0.1);
		h = (int)((max_y - min_y) * 0.33);

		increaseRatio = 2.0;

		size_level = 0;
	}
	*/

	w = (int)((max_x - min_x) * 0.2);
	h = (int)((max_y - min_y) * 0.35); //0.3

	
	// 2016-12-22 JMR	cp = opticalFlow.getOpticalFlow(min_x, min_y, max_x, max_y, (int)w, (int)h, increaseRatio);
	cp.x = 0;
	cp.y = 0;

	velocity[velocity_index % 2][0] = (int)(cp.x * 0.5);
	velocity[velocity_index % 2][1] = (int)(cp.y * 0.5);

	velocity_index++;
}

/**
@brief			매칭 된 detection 윈도우 정보를 이용하여 타겟의 상태 업데이트
연관성 검사 알고리즘 수행 후에 수행됨
@param			particle1 : 추정된 tracker 정보 저장
particle2 : 이전 tracker 정보 저장
detectionCenterX, detectionCenterY : 매칭된 detection 윈도우의 중심 좌표
detectionW, detectionH : 매칭된 detection 윈도우의 크기
@return			업데이트 된 타겟과 이전 tracker 타겟 간의 거리
*/
double CTracker::estimateNewTargetStateUsingDetectionInformation(TARGETPARTICLE& particle1, TARGETPARTICLE& particle2, int& detectionCenterX, int& detectionCenterY, int& detectionW, int& detectionH)
{
	// detection 정보
	int d_cx = detectionCenterX;
	int d_cy = detectionCenterY;
	int d_w = detectionW;
	int d_h = detectionH;

	// 현재 tracker 정보
	int t_cx = particle1.x;
	int t_cy = particle1.y;
	int t_w = particle1.w;
	int t_h = particle1.h;


	//	Trace(_T("dx : %d, dy : %d / tx : %d, ty : %d\n"), d_cx, d_cy, t_cx, t_cy);

	// detection 정보를 이용하여 현재 파티클 상태 업데이트
	particle1.x = (int)(DETECTION_WEIGHT * d_cx + CURR_TRACKER_WEIGHT1 * t_cx);
	particle1.y = (int)(DETECTION_WEIGHT * d_cy + CURR_TRACKER_WEIGHT1 * t_cy);
	particle1.w = (int)(DETECTION_WEIGHT * d_w + CURR_TRACKER_WEIGHT1 * t_w);
	particle1.h = (int)(DETECTION_WEIGHT * d_h + CURR_TRACKER_WEIGHT1 * t_h);

	// 위치 제한
	if (particle1.x <= particle1.w)
	{
		particle1.x = particle1.w + 1;
	}
	else if (particle1.x >= IMAGE_WIDTH - particle1.w)
	{
		particle1.x = IMAGE_WIDTH - particle1.w - 1;
	}

	if (particle1.y <= particle1.h)
	{
		particle1.y = particle1.h + 1;
	}
	else if (particle1.y >= IMAGE_HEIGHT - particle1.h)
	{
		particle1.y = IMAGE_HEIGHT - particle1.h - 1;
	}

	// 이전 tracker와 거리 계산
	return sqrt(((double)(particle1.x - particle2.x)*(particle1.x - particle2.x))
		+ ((double)(particle1.y - particle2.y) * (particle1.y - particle2.y)));
}


///**
//@brief			path prediction을 위한 tracker의 상태 초기화
//*/
//void CTracker::initStateForPrediction()
//{
//	state[0] = 0.25; //0.15 //0.9
//	state[1] = 0.25; //0.45 //0.9
//	state[2] = 0.25; //0.2 //0.9
//	state[3] = 0.25; //0.2 //0.9
//
//	for (int i = 0; i < 5; i++)
//	{
//		accumState[i][0] = 0.0;
//		accumState[i][1] = 0.0;
//		accumState[i][2] = 0.0;
//		accumState[i][3] = 0.0;
//	}
//
//	offsetX = 0;
//	offsetY = 0;
//	head_width = 0;
//	head_height = 0;
//
//	head_left = 0;
//	head_top = 0;
//	head_right = 0;
//	head_bottom = 0;
//
//	headDirection = -1;
//
//	trackerState = 0;
//}

////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////// public /////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

/**
@brief			화면 출력을 위한 tracker의 index 설정
@param			index : tracker의 index
*/
void CTracker::setDisplayTrackerID(int index)
{
	displayTrackerID = index;
}

/**
@brief			detection과 매칭 시 사용되는 tracker의 index 설정
@param			index : tracker의 index
*/
void CTracker::setTrackerIndex(int index)
{
	trackerIndex = index;
}

/**
@brief			detection과 매칭 시 사용되는 tracker의 index 반환
@return			tracker의 index
@date			2014-01-07
*/
int CTracker::getTrackerIndex()
{
	return trackerIndex;
}

/**
@brief			tracker의 겹침 카운트 초기화
tracker가 새로 생성 되었을 경우
@date			2014-01-07
*/
void CTracker::initOcclusionCount()
{
	occlusionCount = 0;
}

/**
@brief			tracker의 겹침 카운트 증가
detection 윈도우와 매칭되지 않았을 경우
*/
void CTracker::plusOcclusionCount()
{
	occlusionCount++;
}


/**
@brief			tracker의 겹침 카운트 반환
tracker의 소멸 유무를 판단할 때 호출
@return			겹침 카운트
*/
int CTracker::getOcclusionCount()
{
	return occlusionCount;
}

/**
@brief			tracker의 겹침 카운트 감소
detection 윈도우와 매칭되었을 경우
*/
void CTracker::minusOcclusionCount()
{
	//	occlusionCount -= 2;
	occlusionCount = 0;
}


void CTracker::initNotDisplayCounnt()
{
	notDisplayCount = 0;
}

void CTracker::plusNotDisplayCounnt()
{
	notDisplayCount++;
}

void CTracker::minusNotDisplayCounnt()
{
	notDisplayCount = 0;
}

void CTracker::valueInit()
{
	currentParticles = new PARTICLEINFO[PARTICLE_NUMBER];
	preParticles = new PARTICLEINFO[PARTICLE_NUMBER];

	trainDataSet_ocslbp = new double**[PART_CLASSES];
	trainDataSet_color = new double**[PART_CLASSES];

	for (int i = 0; i < PART_CLASSES; i++)
	{
		trainDataSet_ocslbp[i] = new double*[MAX_SIZE];
		trainDataSet_color[i] = new double*[MAX_SIZE];

		for (int j = 0; j < MAX_SIZE; j++)
		{
			//		trainDataSet_ocslbp[i][j] = new double[NEW_FEATURE_SIZE];
			//		memset(trainDataSet_ocslbp[i][j], 0, sizeof(double)* NEW_FEATURE_SIZE);

			if (FEATURE_TYPE == 0)
			{
				trainDataSet_ocslbp[i][j] = new double[OCSLBP_FEATURE_SIZE];
				memset(trainDataSet_ocslbp[i][j], 0, sizeof(double)* OCSLBP_FEATURE_SIZE);
			}
			/*else if (FEATURE_TYPE == 1)
			{
				trainDataSet_ocslbp[i][j] = new double[NEW_FEATURE_SIZE];
				memset(trainDataSet_ocslbp[i][j], 0, sizeof(double)* NEW_FEATURE_SIZE);
			}
			else if (FEATURE_TYPE == 2)
			{
				trainDataSet_ocslbp[i][j] = new double[NEW_FEATURE_SIZE];
				memset(trainDataSet_ocslbp[i][j], 0, sizeof(double)* NEW_FEATURE_SIZE);
			}
			else if (FEATURE_TYPE == 3)
			{
				trainDataSet_ocslbp[i][j] = new double[NEW_FEATURE_SIZE];
				memset(trainDataSet_ocslbp[i][j], 0, sizeof(double)* NEW_FEATURE_SIZE);
			}*/

			trainDataSet_color[i][j] = new double[COLOR_FEATURE_SIZE_FOR_TRACKING];
			memset(trainDataSet_color[i][j], 0, sizeof(double) * COLOR_FEATURE_SIZE_FOR_TRACKING);
		}
	}

	distanceBetweenObjects = 100.0;
	displayTrackerID = -1;
	size_level = -1;
	m_Frame_Count_posi = 0;
	m_Frame_Count_nega = 0;
}

/**
@brief			프로그램 실행 시 분류기 생성
tracker가 할당될 때마다 분류기를 생성할 경우 많은 시간이 소요됨
*/
void CTracker::generateClassifier()
{
	rf.generateClassifier(rf_ocslbp, PART_CLASSES, THE_NUMBER_OF_FERN_1, THE_NUMBER_OF_GOOD_FERN_1,
		THE_NUMBER_OF_FUNCTION_1, RF_TRACKING_NORMALIZE_FACTOR, RF_TRACKING_CONSTANT, HISTORY_LENGTH);

	rf.generateClassifier(rf_color, PART_CLASSES, THE_NUMBER_OF_FERN_2, THE_NUMBER_OF_GOOD_FERN_2,
		THE_NUMBER_OF_FUNCTION_2, RF_TRACKING_NORMALIZE_FACTOR, RF_TRACKING_CONSTANT, HISTORY_LENGTH);

	rf.generateClassifier(PART_CLASSES, THE_NUMBER_OF_FERN_2, THE_NUMBER_OF_GOOD_FERN_2,
		THE_NUMBER_OF_FUNCTION_2, RF_TRACKING_NORMALIZE_FACTOR, RF_TRACKING_CONSTANT, HISTORY_LENGTH);
}

/**
@brief			다른 tracker와 겹쳤을 경우, 어떤 tracker가 앞쪽에 있는지 판단하기 위한 함수
3 frame 누적
@date			2014-01-07
*/
void CTracker::initYaxisInformation()
{
	memset(targetYaxis, 0, sizeof(int) * 3);
	trackerCount = 0;
	avgYaxis = 0;
}

int CTracker::getYaxis()
{
	int avgY = (targetYaxis[0] + targetYaxis[1] + targetYaxis[2]) / 3;

	setAvgYaxis(avgY);

	return avgY;
}

void CTracker::setTrackerCountForYaxis()
{
	trackerCount++;
}

void CTracker::setYaxis()
{
	targetYaxis[trackerCount % 3] = currentParticles[0].y + currentParticles[0].h;
}

void CTracker::setAvgYaxis(int value)
{
	avgYaxis = value;
}
int CTracker::getAvgYaxis()
{
	return avgYaxis;
}

/**
@brief			tracker의 윈도우 정보 반환
다른 tracker의 정보를 알아내기 위하여 사용
@param			cx, cy : tracker 윈도우의 중심 좌표 저장
w, h : tracker 윈도우의 크기 저장
*/
void CTracker::getInformationOfTarget(int& cx, int& cy, int& w, int& h, double& pro)
{
	cx = currentParticles[0].x;
	cy = currentParticles[0].y;
	w = currentParticles[0].w;
	h = currentParticles[0].h;

	pro = currentParticles[0].probability;
}

/**
@brief			tracker가 생성된 frame 반환
화면 출력 유무 판단 시 사용
@return			tracker가 생성된 frame 반환
*/
int CTracker::getGenerationFrame()
{
	return frame;
}

/**
@brief			tracker가 생성된 frame 설정
화면 출력 유무 판단 시 사용
@param			currentFrame : tracker가 생성된 frame
*/
void CTracker::initGenerationFrame(int currentFrame)
{
	frame = currentFrame;
}

/**
@brief			tracker의 윈도우를 화면에 출력
DISPLAY_TYPE 설정에 따라 추가적으로 파티클 샘플링 분포 또는 파트 모양 출력
@param			dib : 원본 영상
@date			2014-01-07
*/
void CTracker::trackingDrawRect(CDib& dib)
{
	register int i, k;
	int R, G, B;
	R = G = B = 0;

	RGBBYTE** ptr = dib.GetRGBPtr();

	int sx = currentParticles[0].x - currentParticles[0].w;
	int ex = currentParticles[0].x + currentParticles[0].w;
	int sy = currentParticles[0].y - currentParticles[0].h;
	int ey = currentParticles[0].y + currentParticles[0].h;

	// 정상 추적
	if (currentParticles[0].occlusionType == 0)
	{
		R = 0;
		G = 255;
		B = 0;
	}
	// 부분 상태 변환 & 다른 타겟과 먼 거리
	else if (currentParticles[0].occlusionType == 1)
	{
		R = 255;
		G = 100;
		B = 0;
	}
	// 완전 겹침
	else if (currentParticles[0].occlusionType == 2)
	{
		R = 255;
		G = 0;
		B = 0;
	}
	// 부분 상태 변환 & 다른 타겟과 가까운 거리 /노란색
	else if (currentParticles[0].occlusionType == 3)
	{
		R = 255;
		G = 255;
		B = 0;
	}
	// 객체 놓침 /보라색
	else if (currentParticles[0].occlusionType == 4)
	{
		R = 255;
		G = 0;
		B = 255;
	}

	else
	{
		R = 255;
		G = 255;
		B = 255;
	}

	for (i = sx; i <= ex; i++)
	{
		ptr[sy][i].r = R;
		ptr[sy][i].g = G;
		ptr[sy][i].b = B;

		ptr[ey][i].r = R;
		ptr[ey][i].g = G;
		ptr[ey][i].b = B;

		ptr[sy + 1][i].r = R;
		ptr[sy + 1][i].g = G;
		ptr[sy + 1][i].b = B;

		ptr[ey - 1][i].r = R;
		ptr[ey - 1][i].g = G;
		ptr[ey - 1][i].b = B;
	}

	for (i = sy; i <= ey; i++)
	{
		ptr[i][sx].r = R;
		ptr[i][sx].g = G;
		ptr[i][sx].b = B;

		ptr[i][ex].r = R;
		ptr[i][ex].g = G;
		ptr[i][ex].b = B;

		ptr[i][sx + 1].r = R;
		ptr[i][sx + 1].g = G;
		ptr[i][sx + 1].b = B;

		ptr[i][ex - 1].r = R;
		ptr[i][ex - 1].g = G;
		ptr[i][ex - 1].b = B;
	}

	if (DISPLAY_TYPE == 1)
	{
		for (k = 1; k < PARTICLE_NUMBER; k++)
		{
			sx = currentParticles[k].x - 1;
			ex = currentParticles[k].x + 1;
			sy = currentParticles[k].y - 1;
			ey = currentParticles[k].y + 1;

			R = 255;
			G = 255;
			B = 0;

			for (i = sx; i <= ex; i++)
			{
				ptr[sy][i].r = R;
				ptr[sy][i].g = G;
				ptr[sy][i].b = B;

				ptr[ey][i].r = R;
				ptr[ey][i].g = G;
				ptr[ey][i].b = B;
			}

			for (i = sy; i <= ey; i++)
			{
				ptr[i][sx].r = R;
				ptr[i][sx].g = G;
				ptr[i][sx].b = B;

				ptr[i][ex].r = R;
				ptr[i][ex].g = G;
				ptr[i][ex].b = B;
			}
		}
	}
}

/**
@brief			tracker의 ID를 화면체 출력
@param			pDC : 텍스트 출력을 위한 핸들러
i : tracker ID
*/
void CTracker::trackingDrawText(CDC *pDC, int& i)
{
	CString tracker = _T("0");
	CString probability = _T("");
	CString level = _T("");

	size_level = 2;


	int sx = currentParticles[0].x - currentParticles[0].w;
	int sy = currentParticles[0].y - currentParticles[0].h;

	if (i < 10)
	{
		tracker.Format(_T("00%d"), i);
	}
	else if (i < 100)
	{
		tracker.Format(_T("0%d"), i);
	}

	probability.Format(_T("%.3f"), currentParticles[0].probability);
	level.Format(_T("%d"), size_level);

	pDC->SetTextColor(RGB(0, 255, 0));
	pDC->SetBkMode(TRANSPARENT);
	//pDC->TextOutW(sx, sy, tracker);
}

/**
@brief			화면 출력을 위한 tracker의 index 반환
@return			tracker의 index
*/
int CTracker::getDisplayTrackerID()
{
	return displayTrackerID;
}

int CTracker::getNotDisplayCounnt()
{
	return notDisplayCount;
}

/**
@brief			소멸자
메모리 해제
@date			2014-01-07
*/
void CTracker::valueDestroy()
{
	delete[] currentParticles;
	delete[] preParticles;


	for (int i = 0; i < PART_CLASSES; i++)
	{
		for (int j = 0; j < MAX_SIZE; j++)
		{
			delete[] trainDataSet_ocslbp[i][j];
			delete[] trainDataSet_color[i][j];
		}
	}

	for (int i = 0; i < PART_CLASSES; i++)
	{
		delete[] trainDataSet_ocslbp[i];
		delete[] trainDataSet_color[i];
	}
	delete[] trainDataSet_ocslbp;
	delete[] trainDataSet_color;

	//delete[] state;


	rf.destroyClassifier(rf_ocslbp);
	rf.destroyClassifier(rf_color);
	// temp 소멸
	rf.valueDestroy();



	/*for (int i = 0; i < 5; i++)
	{
		delete[] accumState[i];
	}

	delete[] accumState;*/
}

void CTracker::setMatchingFlag(bool flag)
{
	matchingFlag = flag;
}


/**
@brief			detection 윈도우와 tracker 간에 매칭 결과 반환
@return			detection 윈도우와 tracker 간의 매칭 결과
*/
bool CTracker::getMatchingflag()
{
	return matchingFlag;
}