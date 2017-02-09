#include "stdafx.h"
#include "Tracking.h"


CTracking::CTracking(void)
{
	detectionValueInit();
	trackingValueInit();
	detection_n_tracking_ValueInit();
}


CTracking::~CTracking(void)
{
	detectionValueDestroy();
	trackingValueDestroy();
	detection_n_tracking_ValueDestroy();
}

void CTracking::detection_n_tracking_processing(CDib& dib, PairInfo pair, int& currentFrame)
{
	register int i, j;

	bool restTracking = true;

	m_currentFrame = currentFrame;

	// Pairing 갯수를 Detection 갯수로
	m_ntheNumberOfDetection = pair.pair_cnt;

	if (currentFrame == 16)
	{
		int a=0;
	}

#pragma region 변수 초기화
	if (currentFrame <= 1)
	{
		theNumberOfTracking = 0;

		// detection과 매칭시 사용되는 tracker의 id 초기화
		for (i = 0; i < TRACKING_NUM; i++)
		{
			tracker[i].setTrackerIndex(i);
		}

		// 화면 표시용 tracker의 id 초기화
		for (int i = 0; i < TRACKING_NUM; i++)
		{
			tracker[i].setDisplayTrackerID(-1);
			tracking_setAssignedTrackerID(i);
		}

		assignedLastTrackerID = -1;
		//		m_opticalFlow.set_ReferenceLine_DefaultMask();

		//if (m_ntheNumberOfDetection != 0 || theNumberOfTracking != 0)
		//{
		//	for (i = 0; i < theNumberOfTracking; i++)
		//	{
		//		/*for (j = 0; j < rectcount; j++)
		//		{
		//		if (regioninfo[j].particleNumber == -1)
		//		{
		//		regioninfo[j].particleNumber = tracker[i].getTrackerIndex();
		//		break;
		//		}
		//		}*/
		//		trackingProcessing(dib, currentFrame, tracker[i], regioninfo[i], otherTarget, 0);
		//		tracker[i].initGenerationFrame(currentFrame);
		//	}
		//}
		//return;
	}
#pragma endregion

#pragma region Get OpticalFlow
	/*OpticalFlowProcessing(dib, currentFrame);
	m_nOPFfeatures_count = m_opticalFlow.get_opticalflow_result(m_opticalflow_result);*/
#pragma endregion

#pragma region Tracking Part Step 1
	// 기존 tracker 정보만 이용하여 추적 수행
	// 타겟의 위치는 상위 N개 파티클의 평균 크기와 위치로 추정
	for (i = 0; i < theNumberOfTracking; i++)
	{
		trackingProcessing(dib, currentFrame, tracker[i], pair.pairinfo[0], otherTarget, 1);
	}

	// Thread 이용시 아래에 추가

#pragma endregion

#pragma region Association Check
	// detection과 tracker 간의 연관성 검사
	// tracker는 상위 N개의 파티클을 이용하여 추정된 것
	MatchingPairBoxWithTracking(dib, m_ntheNumberOfDetection, theNumberOfTracking, pair.pairinfo, tracker);
#pragma endregion

#pragma region Tracking Part Step 2
	if (m_ntheNumberOfDetection != 0 || theNumberOfTracking != 0)
	{
		for (i = 0; i < theNumberOfTracking; i++)
		{
			// 현재 프레임 보다 이전 프레임에 생성되었으며 detection과 매칭되지 않은 것
			// 기존의 정보를 이용하여 계속해서 tracking 수행
			if (tracker[i].getGenerationFrame() != currentFrame && !tracker[i].getMatchingflag())
			{
				// 그냥 파라매터만 넘겨주고 쓰지 말아야 하기 때문에 -1을 넣어줌
				//pair.pairinfo[m_ntheNumberOfDetection].box_center_x = -1;
				otherTarget.myTargetNumber = tracker[i].getTrackerIndex();
				trackingProcessing(dib, currentFrame, tracker[i], pair.pairinfo[m_ntheNumberOfDetection], otherTarget, 2);
			}
			// 현재 프레임에 생성되었으며 detection과 매칭되지 않은 것
			// 새로운 detection에 의해 새롭게 할당 받은 tracking
			else if (tracker[i].getGenerationFrame() == currentFrame && !tracker[i].getMatchingflag())
			{
				for (j = 0; j < m_ntheNumberOfDetection; j++)
				{
					if (pair.pairinfo[j].particleNumber == -1)
					{
						pair.pairinfo[j].particleNumber = tracker[i].getTrackerIndex();
						break;
					}
				}

				otherTarget.myTargetNumber = tracker[i].getTrackerIndex();
				trackingProcessing(dib, currentFrame, tracker[i], pair.pairinfo[j], otherTarget, 0);
			}
			// detection과 매칭된 tracking
			else if (tracker[i].getMatchingflag())
			{
				for (j = 0; j < m_ntheNumberOfDetection; j++)
				{
					if (pair.pairinfo[j].particleNumber == tracker[i].getTrackerIndex())
					{
						break;
					}
				}

				otherTarget.myTargetNumber = tracker[i].getTrackerIndex();
				trackingProcessing(dib, currentFrame, tracker[i], pair.pairinfo[j], otherTarget, 2);
			}
		}
	}

#pragma endregion

#pragma region Get Other Tracker Information

	// RF update를 결정하기 위한 다른 target 들의 위치 정보를 저장하는 것
	detection_n_tracking_countTargetLocation(otherTarget, pair.pairinfo, tracker, m_ntheNumberOfDetection, theNumberOfTracking, currentFrame);

#pragma endregion

#pragma region Tracking Part Step 3
	// 최종적으로 타겟 정보가 업데이트 된 후
	// 추정된 타겟의 확률값에 따라 분류기의 업데이트를 결정
	if (m_ntheNumberOfDetection != 0 || theNumberOfTracking != 0)
	{
		for (i = 0; i < theNumberOfTracking; i++)
		{
			if (tracker[i].getGenerationFrame() != currentFrame)
			{
				trackingProcessing(dib, currentFrame, tracker[i], pair.pairinfo[0], otherTarget, 3);
			}
		}
	}

#pragma endregion

}

void CTracking::trackingProcessing(CDib& dib, const int currentFrame, CTracker& tracker, Pair pairinfo, OTHERTARGETLOCATION& otherTarget, int processingStep)
{
	CDib grayCpy/*, hsiCpy*/;
	grayCpy.Copy(&dib);

	CDib ColorCpy;
	ColorCpy.Copy(&dib);

	//	hsiCpy.Copy(&dib);

	if (dib.GetBitCount() != 8)
	{
		DibGrayscale(grayCpy);
		//		DibHSIscale(hsiCpy);
	}

	// Detection된 정보를 이용하여 새로운 tracker 생성
	if (processingStep == 0)
	{
		tracker.processGenerationTracker(grayCpy, ColorCpy, true, pairinfo.box_center_x, pairinfo.box_center_y, pairinfo.box_min_x, pairinfo.box_min_y, pairinfo.box_max_x, pairinfo.box_max_y);
	}

	// 기존 tracker 정보만 이용하여 추적 수행
	// 타겟의 위치는 상위 N개 파티클의 평균 크기와 위치로 추정
	else if (processingStep == 1)
	{
		tracker.processTracking(grayCpy, ColorCpy);
	}
	// Detection과 matching 된 결과에 따라 
	// tracker의 상태를 업데이트 함
	else if (processingStep == 2)
	{
		tracker.processUpdatingTrackerStates(grayCpy, ColorCpy, pairinfo.box_center_x, pairinfo.box_center_y, pairinfo.box_min_x, pairinfo.box_min_y, pairinfo.box_max_x, pairinfo.box_max_y);
	}
	// tracker의 상태, 다른 tracker와 거리, tracker의 확률값에 따라
	// 분류기의 업데이트를 결정
	else if (processingStep == 3)
	{
		tracker.processUpdatingClassifiers(grayCpy, ColorCpy, false, otherTarget, currentFrame, m_opticalFlow);
	}
}

/**
@brief			OpticalFlow 호출 함수
@param			dib : 입력 영상
@param			nCurrentFrame : 현재 프레임 번호
*/
int CTracking::OpticalFlowProcessing(CDib& dib, int nCurrentFrame)
{
/*
	// 영상을 각 레벨 별로 생성
	CDib cpyDib;
	BYTE** ptr;
	int nNewVehicleDirectResult = -1;		// -1:초기화, 0:직진, 1:좌회전, 2:우회전	

	cpyDib.Copy(&dib);

	if (cpyDib.GetBitCount() != 8)
	{
		DibGrayscale(cpyDib);
	}

	ptr = cpyDib.GetPtr();

	// 이전 프레임과 연속된 프레임이 아니면 Optical Flow의 이전 영상 재설정
	if (nCurrentFrame == 1)
	{
		m_opticalFlow.SetPreviousImg(ptr, cpyDib.GetWidth(), cpyDib.GetHeight());
		m_nPrevFrameNum = nCurrentFrame;
		m_bUpdateMask = false;

	}
	else if (abs(nCurrentFrame - m_nPrevFrameNum) >= SKIP_FRAME_FOR_OPTICAL)			// 3 프레임 간격으로 Optical Flow 체크
	{
		// Optical Flow 호출 //////////////////////////////////////////
		nNewVehicleDirectResult = m_opticalFlow.mainProcessing(ptr, m_bUpdateMask);
		//		nNewVehicleDirectResult = m_opticalFlow.mainProcessing_egoMotionCompensation_BM(ptr, m_bUpdateMask);
		m_nPrevFrameNum = nCurrentFrame;
	}

	return nNewVehicleDirectResult;			// 새로 추출된 방향 반환*/
	return 0;
}

/**
@brief			화면 출력을 위한 tracker의 id 할당 해제 설정
@param			id : tracker의 id
*/
void CTracking::tracking_setAssignedTrackerID(int id)
{
	assignedTrackerID[1][id] = 0;
}

/**
@brief			생성자
메모리 할당
변수 초기화
tracking 관련
*/
void CTracking::trackingValueInit()
{
	register int i;
	theNumberOfTracking = 0;

	tracker = new CTracker[TRACKING_NUM];

	for (i = 0; i < TRACKING_NUM; i++)
	{
		//tracker 값 초기화
		tracker[i].valueInit();

		//tracker 인덱스 초기화
		tracker[i].setTrackerIndex(i);

		// 분류기 생성
		tracker[i].generateClassifier();
	}
}

/**
@brief			생성자
메모리 할당
변수 초기화
tracking by detection 관련
*/
void CTracking::detection_n_tracking_ValueInit()
{
	//스레드를 이용할때 사용하기 위한 트래킹 정보를 담는 구조체 초기화. 아직 사용 x
	/*trackingInfo = new TRACKINGASSIGNING[TRACKING_NUM];
	memset(trackingInfo, NULL, sizeof(TRACKINGASSIGNING) * TRACKING_NUM);*/

	assignedTrackerID = new int*[2];
	for (int i = 0; i < 2; i++)
	{
		assignedTrackerID[i] = new int[TRACKING_NUM];
		memset(assignedTrackerID[i], 0, sizeof(int) * TRACKING_NUM);
	}
	for (int i = 0; i < TRACKING_NUM; i++)
	{
		assignedTrackerID[0][i] = i;
	}

	assignedLastTrackerID = -1;
}

/**
@brief			생성자
메모리 할당
변수 초기화
Regioninfo초기화
*/
void CTracking::detectionValueInit()
{
	pair.pairinfo = new Pair[9999];

	// Optical Flow 클래스
	//m_opticalFlow.Func_Init(IMG_SIZE_HEIGHT, IMG_SIZE_WIDTH, OPTICALfLOW_GRID_INTERVAL);
	m_nPrevFrameNum = 99999;

	//m_opticalflow_result = new OPTICALFLOW_FEATURES[m_opticalFlow.getGridCount()];
}

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
void CTracking::detection_n_tracking_countTargetLocation(OTHERTARGETLOCATION& otherTarget, Pair* pair, CTracker* trk, int theNumberOfDetection, int theNumberOfTracking, int currentFrame)
{
	register int i;
	int totalTarget = 0;
	int availableCount = 0;

	otherTarget.myTargetNumber = -1;
	otherTarget.theNumberOfOtherTarget = 0;

	for (i = 0; i < theNumberOfTracking; i++)
	{
		// 할당된지 얼마 안 된 tracker는 거리 계산을 하지 않음
		if (currentFrame - trk[i].getGenerationFrame() > CHECKING_FRAME)
		{
			// cx, cy, w, h
			trk[i].getInformationOfTarget(otherTarget.location[i][0], otherTarget.location[i][1], otherTarget.location[i][2], otherTarget.location[i][3], otherTarget.targetProbability[i]);

			otherTarget.targetIndexs[i] = trk[i].getTrackerIndex();

			// 겹침 추정을 위한 평균 Y 좌표 반환 밑 갱신
			// 3프레임 누적
			otherTarget.avgYaxis[i] = trk[i].getYaxis();
			trk[i].setYaxis();
			trk[i].setTrackerCountForYaxis();

			availableCount++;
		}
		else if (0 <= currentFrame - trk[i].getGenerationFrame() && currentFrame - trk[i].getGenerationFrame() <= CHECKING_FRAME)
		{
			// 겹침 추정을 위한 평균 Y 좌표 밑 갱신
			// 1~3프레임 누적
			trk[i].setYaxis();
			trk[i].setTrackerCountForYaxis();
		}
	}

	otherTarget.theNumberOfOtherTarget = availableCount;
}

/**
@brief		tracker 윈도우 화면에 출력
@param		dib : 원본이미지
*/
void CTracking::trackingDraw(CDib& dib)
{
	register int i;
	int generate_frame = 0;
	for (i = 0; i < theNumberOfTracking; i++)
	{
		generate_frame = tracker[i].getGenerationFrame();
		if (m_currentFrame - generate_frame > CHECKING_FRAME)
		{
			if (tracker[i].getNotDisplayCounnt() <= OCCLUSION_THRESHOLD)
			{
				// 화면 표시용 tracker의 id 할당
				if (tracker[i].getDisplayTrackerID() == -1)
				{
					tracker[i].setDisplayTrackerID(detection_tracking_getDisplayTrackerID());
				}

				trackingDrawRect(dib, tracker[i]);
			}
		}

		//trackingDrawRect(dib, tracker[i]);

	}
}

/**
@brief			화면 출력을 위한 tracker의 id 할당
@return			tracker의 id
*/
int CTracking::detection_tracking_getDisplayTrackerID()
{
	do
	{
		assignedLastTrackerID++;

		if (assignedLastTrackerID == TRACKING_NUM)
		{
			assignedLastTrackerID = 0;
		}

	} while (detection_n_tracking_getAssignedTrackerID(1, assignedLastTrackerID) == 1);

	return detection_n_tracking_getAssignedTrackerID(0, assignedLastTrackerID);
}

/**
@brief			화면 출력을 위한 tracker의 id 할당 여부 및 id 반환
@param			row : 0 -> tracker의 id
1 -> tracker의 id 할당 여부
id : tracker의 id
@return			tracker의 id 할당 여부 or tracker의 id
*/
int CTracking::detection_n_tracking_getAssignedTrackerID(int row, int id)
{
	return assignedTrackerID[row][id];
}

/**
@brief		tracker 윈도우 화면에 출력
@param		dib : 원본이미지
trk : tracking 클래스 객체
*/
void CTracking::trackingDrawRect(CDib& dib, CTracker& trk)
{
	trk.trackingDrawRect(dib);
}

/**
@brief		tracker 윈도우 화면에 출력
@param		dib : 원본이미지
*/
void CTracking::detection_n_tracking_draw_tracking(CDib& dib)
{
	// 임시 테스트를 위한 코드 추가 JMR //////////////////////////////////////////////////////////////
	//	dib.Copy(&m_dibTestDisplay);

	trackingDraw(dib);
}

/**
@brief		tracker ID 출력
@param		pDC : 텍스트 출력을 위한 핸들러
*/
void CTracking::detection_n_tracking_draw_trackingText(CDC *pDC)
{
	trackingDraw2(pDC);
}

void CTracking::trackingDraw2(CDC *pDC)
{
	register int i;
	for (i = 0; i < theNumberOfTracking; i++)
	{
		if (m_currentFrame - tracker[i].getGenerationFrame() > CHECKING_FRAME)
		{
			if (tracker[i].getNotDisplayCounnt() <= OCCLUSION_THRESHOLD)
			{
				trackingDrawText(pDC, tracker[i], tracker[i].getDisplayTrackerID());
			}
		}
	}
}

/**
@brief		tracker ID 출력
@param		pDC : 텍스트 출력을 위한 핸들러
trk : tracking 클래스 객체
i : tracker ID
*/
void CTracking::trackingDrawText(CDC *pDC, CTracker& trk, int i)
{
	trk.trackingDrawText(pDC, i);
}

/**
@brief		trakcing 클래스 객체 간 정보 swap
@param		trk1 : tracking 클래스 객체
trk2 : tracking 클래스 객체
*/
void CTracking::swap(CTracker& trk1, CTracker& trk2)
{
	// tracking class의 swap을 위한 변수

	//	CTracking tempForSwap;

	tempForSwap = trk1;
	trk1 = trk2;
	trk2 = tempForSwap;
}

/**
@brief			소멸자
메모리 해제
tracking by detection 관련
*/
void CTracking::detection_n_tracking_ValueDestroy()
{
	//delete[] trackingInfo;

	for (int i = 0; i < 2; i++)
	{
		delete[] assignedTrackerID[i];
	}

	delete[] assignedTrackerID;
}

/**
@brief			소멸자
메모리 해제
tracking 관련
*/
void CTracking::trackingValueDestroy()
{
	register int i;

	for (i = 0; i < TRACKING_NUM; i++)
	{
		tracker[i].valueDestroy();
	}
	delete[] tracker;

	//tempForSwap.valueDestroy();

	// Optical Flow 소멸
	//m_opticalFlow.Func_Destory();
}

/**
@brief		소멸자
메모리 해제
detection 관련
*/
void CTracking::detectionValueDestroy()
{
	delete[] pair.pairinfo;

	//delete[] m_opticalflow_result;

	//m_opticalFlow.Func_Destory();
}

//Association Check
void CTracking::MatchingPairBoxWithTracking(CDib& dib, int& count1, int& count2, Pair* pair, CTracker* trk)
{
	register int i, j;

	int sCount = 0;
	int size = 0;

	double distanceSum_dist = 0.0;
	//	double distanceSum_width = 0.0;
	//	double distanceSum_height = 0.0;
	double distPro = 0.0;
	//double widthPro = 0.0;
	//double heightPro = 0.0;
	double ratioPro = 0.0;

	double threshold = SCORE_THRESHOLD;

	double ocslbp_probability = 0.0;
	double lid_probability = 0.0;

	int tCount = count2;
	int trackingCount = 0;

	double dist = 0.0;
	double distThreshold = 0.0;

	int currentArea = 0;

	int distCount = 0;

	// 연관성 검사 시 거리 범위
	double range = 0.75; // 0.75

	for (i = 0; i < TRACKING_NUM; i++)
	{
		trk[i].setMatchingFlag(false);
	}

	if (count1 != 0)
	{
#pragma region hungarian method에 사용할 score matrix 만들기
		//CDib grayCpy/*, hsiCpy*/;
		//grayCpy.Copy(&dib);
		////	hsiCpy.Copy(&dib);

		//if (dib.GetBitCount() != 8)
		//{
		//	DibGrayscale(grayCpy);
		//	//		DibHSIscale(hsiCpy);
		//}

		CDib cpydib;
		cpydib.Copy(&dib);

		// 정방 행렬로 만들기 위한 것
		if (count1 > count2)
		{
			sCount = count1;
		}
		else if (count1 < count2)
		{
			sCount = count2;
		}
		else
		{
			sCount = count1;
		}

		// 모든 detection과 tracking 간의 score가 저장되는 배열
		// 정방 행렬을 만들기 위해서 숫자가 모자란 쪽은 0이 저장됨
		// 이 행렬을 가지고 매칭을 수행함
		// 행렬 크기가 1 x 1 인 경우 2 x 2로 늘려줌
		if (sCount == 1)
		{
			size = 2;
		}
		else
		{
			size = sCount;
		}

		double** squareScoreMatrix = new double*[size];
		for (i = 0; i < size; i++)
		{
			squareScoreMatrix[i] = new double[size];
		}

		// score 매트릭스 초기화
		for (i = 0; i < size; i++)
		{
			for (j = 0; j < size; j++)
			{
				//	squareScoreMatrix[i][j] = ((rand() % (int)((SCORE_THRESHOLD - 0.1) * 10)) * 0.1) + 0.1;
				squareScoreMatrix[i][j] = 0.0;
			}
		}

		// 모든 detection과 tracker 간의 score 값을 계산 한 후
		// 행렬에 저장				
		for (i = 0; i < count1; i++)
		{
			// detection 윈도우 크기 계산
			currentArea = (pair[i].boxWidth) * (pair[i].boxHeight);

			// detection 윈도우 크기에 따른 매칭 시 사용되는 거리 임계값을 다르게 설정
			/*			if(currentArea <=  (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_6) * (DEFAULT_WINDOW_HEIGHT / IMG_LEVEL_RESIZE_RATE_6))
			{
			distThreshold = (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_6) * range; // 0.75
			}
			else*/ 

			// 아래 주석 나중에 풀지도?
			//
			//
			/*if (currentArea <= (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_5) * (DEFAULT_WINDOW_HEIGHT / IMG_LEVEL_RESIZE_RATE_5))
			{
				distThreshold = (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_5) * range;
			}
			else if (currentArea <= (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_4) * (DEFAULT_WINDOW_HEIGHT / IMG_LEVEL_RESIZE_RATE_4))
			{
				distThreshold = (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_4) * range;
			}
			else if (currentArea <= (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_3) * (DEFAULT_WINDOW_HEIGHT / IMG_LEVEL_RESIZE_RATE_3))
			{
				distThreshold = (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_3) * range;
			}
			else if (currentArea <= (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_2) * (DEFAULT_WINDOW_HEIGHT / IMG_LEVEL_RESIZE_RATE_2))
			{
				distThreshold = (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_2) * range;
			}
			else if (currentArea <= (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_1) * (DEFAULT_WINDOW_HEIGHT / IMG_LEVEL_RESIZE_RATE_1))
			{
				distThreshold = (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_1) * range;
			}
			else if (currentArea <= (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_0) * (DEFAULT_WINDOW_HEIGHT / IMG_LEVEL_RESIZE_RATE_0))
			{
				distThreshold = (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_0) * range;
			}
			else
			{
				distThreshold = (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_0) * range;
			}*/

			distThreshold = (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_0) * range;

			// 차량이 회전하면 거리 임계값을 크게 설정
			// 직진일 경우 임계값은 그대로
			// -1:초기화, 0:직진, 1:좌회전, 2:우회전	
			//if ((vehicleDirection == 1 || vehicleDirection == 2) && USING_THRESHOLDS_ACCORDING_TO_ROTATION)
			//{
			//	distThreshold *= 1.5; //1.2
			//}


			// score 계산을 위한 거리와 확률값 계산
			for (j = 0; j < count2; j++)
			{
				// 거리 측정
				//distance[j] = trk[j].measureDistanceBetweenDetectionAndTracking(pair[i].box_center_x, pair[i].box_center_y);

				// Detection Region 특징 추출 및 RF 확률 측정
				probability[j] = trk[j].getFeature_n_measureProbabilityDetectionRegion(cpydib,
																					pair[i].box_center_x,
																					pair[i].box_center_y,
																					pair[i].box_min_x,
																					pair[i].box_min_y,
																					pair[i].box_max_x,
																					pair[i].box_max_y);

				ratio[j] = trk[j].getOverlapRatioBetweenDetectionsAndTrackers(pair[i].box_center_x,
																					pair[i].box_center_y,
																					pair[i].box_min_x,
																					pair[i].box_min_y,
																					pair[i].box_max_x,
																					pair[i].box_max_y);


				// detection window와 tracking window 간의 width 길이 비교
				//		widthDist[j] = fabs((trk[j].getWidthOfTrackingWindow() * 2.0) - (double)(regionInfo1[i].max_x - regionInfo1[i].min_x));

				// detection window와 tracking window 간의 height 길이 비교
				//		heightDist[j] = fabs((trk[j].getHeightOfTrackingWindow() * 2.0) - (double)(regionInfo1[i].max_y - regionInfo1[i].min_y));

				//		ratio[j] = fabs(1.0 - (((trk[j].getWidthOfTrackingWindow() * 2.0) * (trk[j].getHeightOfTrackingWindow() * 2.0)) / (double)((regionInfo1[i].max_x - regionInfo1[i].min_x) * (regionInfo1[i].max_y - regionInfo1[i].min_y))));

				//		if(ratio[j] > 1.0)
				//		{
				//			ratio[j] = 1.0;
				//		}

				if (distThreshold >= distance[j])
				{
					// 거리의 가중치를 계산하기 위한 것 
					distanceSum_dist += distance[j];

					// 거리가 임계값 보다 작고, 0인 경우를 위해 카운트함
					distCount++;
				}

				// width 차의 가중치를 계산하기 위한 것
				//		distanceSum_width += widthDist[j];

				// height 차의 가중치를 계산하기 위한 것
				//		distanceSum_height += heightDist[j];
			}

			// score 저장
			for (j = 0; j < count2; j++)
			{
				if (distCount == 0)
				{
					distPro = 0.0;
				}
				else if (distThreshold < distance[j])
				{
					distPro = 0.0;
				}
				else
				{
					// 거리가 0인 경우
					if (distanceSum_dist == 0)
					{
						distPro = 1.0;
					}
					else if (distCount == 1)
					{
						if (distThreshold * 0.2 >= distance[j])
						{
							distPro = 1.0;
						}
						else if (distThreshold * 0.4 >= distance[j])
						{
							distPro = 0.8;
						}
						else if (distThreshold * 0.6 >= distance[j])
						{
							distPro = 0.6;
						}
						else if (distThreshold * 0.8 >= distance[j])
						{
							distPro = 0.4;
						}
						else if (distThreshold * 1.0 >= distance[j])
						{
							distPro = 0.2;
						}
						else
						{
							distPro = 0.0;
						}

						//			distPro = 1.0;
					}
					else
					{
						distPro = 1.0 - (distance[j] / distanceSum_dist);
					}

					//	distPro = 1.0;
				}

				//if(distThreshold * 0.2 >= distance[j])
				//{
				//	distPro = 1.0;
				//}
				//else if(distThreshold * 0.4 >= distance[j])
				//{
				//	distPro = 0.8;
				//}
				//else if(distThreshold * 0.6 >= distance[j])
				//{
				//	distPro = 0.6;
				//}
				//else if(distThreshold * 0.8 >= distance[j])
				//{
				//	distPro = 0.4;
				//}
				//else if(distThreshold * 1.0 >= distance[j])
				//{
				//	distPro = 0.2;
				//}
				//else
				//{
				//	distPro = 0.0;
				//}
				//
				//if(distanceSum_width == 0)
				//{
				//	widthPro = 0.0;
				//}
				//else
				//{
				//	widthPro = 1.0 - (widthDist[j] / distanceSum_width);
				//}
				//
				//if(distanceSum_height == 0)
				//{
				//	heightPro = 0.0;
				//}
				//else
				//{
				//	heightPro = 1.0 - (heightDist[j] / distanceSum_height);
				//}
				//
				//ratioPro = 1.0 - ratio[j];
				//
				//if(count2 == 1)
				//{
				////	squareScoreMatrix[i][j] = 0.5 * probability[j] + 0.5 * ratioPro;
				////	squareScoreMatrix[i][j] = DIST_WEIGHT * distPro + PROB_WEIGHT * probability[j] + SIZE_WEIGHT * ratioPro;
				////	squareScoreMatrix[i][j] = DIST_WEIGHT * distPro + PROB_WEIGHT * probability[j] + SIZE_WEIGHT * ratio[j];
				////	squareScoreMatrix[i][j] = PROB_WEIGHT * probability[j] + SIZE_WEIGHT * ratio[j];
				//	squareScoreMatrix[i][j] = ratio[j] * probability[j] + (1.0 - ratio[j]) * distPro;
				//}
				//else
				//{
				////	squareScoreMatrix[i][j] = DIST_WEIGHT * distPro + PROB_WEIGHT * probability[j] + SIZE_WEIGHT * ratioPro;
				////	squareScoreMatrix[i][j] = DIST_WEIGHT * distPro + PROB_WEIGHT * probability[j] + SIZE_WEIGHT * ratio[j];
				////	squareScoreMatrix[i][j] = PROB_WEIGHT * probability[j] + SIZE_WEIGHT * ratio[j];
				//	squareScoreMatrix[i][j] = ratio[j] * probability[j] + (1.0 - ratio[j]) * distPro;
				//}
				//
				//	squareScoreMatrix[i][j] = ratio[j] * probability[j] + (1.0 - ratio[j]) * distPro;
				//	squareScoreMatrix[i][j] = ratio[j] * distPro + (1.0 - ratio[j]) * probability[j];
				//	squareScoreMatrix[i][j] = (1.0 - ratio[j]) * probability[j];
				//
				//	squareScoreMatrix[i][j] = PROB_WEIGHT * probability[j] + SIZE_WEIGHT * ratio[j];
				//	squareScoreMatrix[i][j] = distPro * probability[j] + (1.0 - distPro) * ratio[j];
				//	squareScoreMatrix[i][j] = (probability[j] * distPro)  + ((1.0 - probability[j]) * ratio[j]);
				//	squareScoreMatrix[i][j] = (0.5 * probability[j]) + (0.0 * distPro) + (0.5 * ratio[j]);
				squareScoreMatrix[i][j] = (0.5 * probability[j]) + (0.5 * ratio[j]); //+ (0.2 * distPro) 
				//	squareScoreMatrix[i][j] = (0.4 * probability[j]) + (0.2 * distPro) + (0.4 * ratio[j]);
				//	squareScoreMatrix[i][j] = distPro * (probability[j] * 0.5 + ratio[j] * 0.5);
			}

			distCount = 0;
			distanceSum_dist = 0.0;
			//		distanceSum_width = 0.0;
			//		distanceSum_height = 0.0;
		}
#pragma endregion

		// 초기화
		for (i = 0; i < sCount; i++)
		{
			pair[i].particleNumber = -1;
		}

		// hungarian method
		x_y = matching.matchingProcessing(sCount, squareScoreMatrix);

		for (i = 0; i < count1; i++)
		{
			//dist = trk[x_y[i]].measureDistanceBetweenDetectionAndTracking(regionInfo1[i].center_x, regionInfo1[i].center_y);

			//// detection 윈도우 크기 계산
			//currentArea = (regionInfo1[i].max_x - regionInfo1[i].min_x) * (regionInfo1[i].max_y - regionInfo1[i].min_y);

			//// detection 윈도우 크기에 따른 매칭 시 사용되는 거리 임계값을 다르게 설정
			//if(currentArea <=  (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_6) * (DEFAULT_WINDOW_HEIGHT / IMG_LEVEL_RESIZE_RATE_6))
			//{
			//	distThreshold = (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_6) * 0.75;
			//}
			//else if(currentArea <=  (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_5) * (DEFAULT_WINDOW_HEIGHT / IMG_LEVEL_RESIZE_RATE_5))
			//{
			//	distThreshold = (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_5) * 0.75;
			//}
			//else if(currentArea <=  (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_4) * (DEFAULT_WINDOW_HEIGHT / IMG_LEVEL_RESIZE_RATE_4))
			//{
			//	distThreshold = (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_4) * 0.75;
			//}
			//else if(currentArea <=  (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_3) * (DEFAULT_WINDOW_HEIGHT / IMG_LEVEL_RESIZE_RATE_3))
			//{
			//	distThreshold = (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_3) * 0.75;
			//}
			//else if(currentArea <=  (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_2) * (DEFAULT_WINDOW_HEIGHT / IMG_LEVEL_RESIZE_RATE_2))
			//{
			//	distThreshold = (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_2) * 0.75;
			//}
			//else if(currentArea <=  (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_1) * (DEFAULT_WINDOW_HEIGHT / IMG_LEVEL_RESIZE_RATE_1))
			//{
			//	distThreshold = (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_1) * 0.75;
			//}
			//else if(currentArea <=  (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_0) * (DEFAULT_WINDOW_HEIGHT / IMG_LEVEL_RESIZE_RATE_0))
			//{
			//	distThreshold = (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_0) * 0.75;
			//}
			//else
			//{
			//	distThreshold = (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_0) * 0.75;
			//}

			// matching이 된 경우
			if (squareScoreMatrix[i][x_y[i]] > threshold /*&& dist < distThreshold*/)		// 2013-12-13 JMR 80->100->20->100
			{
				// label 할당
				pair[i].particleNumber = trk[x_y[i]].getTrackerIndex();

				// matching 됐으므로 occlusion count를 감소
				trk[x_y[i]].setMatchingFlag(true);
			}
			else
			{
				int a = 0;
			}
		}

		// matching 되지 않은 detection에게 새로운 tracker를 할당하기 위해 tracker 초기화
		for (i = 0; i < count1; i++)
		{
			if (pair[i].particleNumber == -1)
			{
				trk[tCount].initOcclusionCount();
				trk[tCount].initNotDisplayCounnt();
				//trk[tCount].initStateForPrediction();
				trk[tCount].setMatchingFlag(false);
				trk[tCount].initGenerationFrame(m_currentFrame);
				trk[tCount].initYaxisInformation();
				tCount++;
			}
		}

		// 메모리 해제
		if (sCount != 1)
		{
			for (i = 0; i < sCount; i++)
			{
				delete[] squareScoreMatrix[i];
			}

			delete[] squareScoreMatrix;
		}
		else
		{
			for (i = 0; i < 2; i++)
			{
				delete[] squareScoreMatrix[i];
			}

			delete[] squareScoreMatrix;
		}
		x_y = NULL;
	}
	// tracking 수가 detection 수 보다 많을 경우 detection과 매칭되지 않은 tracking의 occlusion count를 +1 시켜준다
	for (i = 0; i < count2; i++)
	{
		if (!trk[i].getMatchingflag())
		{
			trk[i].plusOcclusionCount();
		}
		else
		{
			if (trk[i].getOcclusionCount() > 0)
			{
				trk[i].minusOcclusionCount();
			}
		}

		int cx = 0;
		int cy = 0;
		int cw = 0;
		int ch = 0;
		double pro = 0.;

		trk[i].getInformationOfTarget(cx, cy, cw, ch, pro);

		if (abs(cx - cw) < 10 || abs(752 - (cx + cw)) < 10 || abs(cy - ch) < 10 || abs(480 - (cy + ch)) < 10)
		{
			trk[i].plusNotDisplayCounnt();
			trk[i].plusNotDisplayCounnt();
			trk[i].plusNotDisplayCounnt();
			trk[i].plusNotDisplayCounnt();
		}
		else
		{
			trk[i].minusNotDisplayCounnt();
		}


	} // tracking

	// 새로운 detection에 의해 할당 받은 tracking 수를 추가해준다
	// 이 위치에 한 이유는 새로 생성된 후 occlusion count를 증가 시켜주지 않기 위해서
	count2 = tCount;
	// occlusion Count가 100 이상일 경우 완전히 사라진 것으로 보고 추적을 종료한다
	// tracker가 생성된 지 3프레임 내에 detection과 매칭이 되지 않았다면, 잘못된 detection으로 판단하고 소멸 시킨다	
	for (i = 0; i < count2; i++)
	{
		if ((trk[i].getOcclusionCount() >= OCCLUSION_THRESHOLD || trk[i].getNotDisplayCounnt() >= OCCLUSION_THRESHOLD)
			|| ((m_currentFrame - trk[i].getGenerationFrame() <= CHECKING_FRAME) && (trk[i].getOcclusionCount() >= CHECKING_THRESHOLD))
			|| (trk[i].getWidthOfTrackingWindow() <= 2 || trk[i].getHeightOfTrackingWindow() <= 4) && (m_currentFrame - trk[i].getGenerationFrame() > CHECKING_FRAME))
		{
			// 화면 표시용 tracker의 id 할당 해제
			if (trk[i].getDisplayTrackerID() != -1)
			{
				tracking_setAssignedTrackerID(trk[i].getDisplayTrackerID());
				trk[i].setDisplayTrackerID(-1);
			}

			for (j = i; j < TRACKING_NUM - 1; j++)
			{
				swap(trk[j], trk[j + 1]);
			}

			count2--;

			if (i < count2)
			{
				i--;
			}
		}
	}
}

void CTracking::valueDestroy()
{
	detectionValueDestroy();
	trackingValueDestroy();
	detection_n_tracking_ValueDestroy();
}

void CTracking::valueInit()
{
	detectionValueInit();
	trackingValueInit();
	detection_n_tracking_ValueInit();
}