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

	// Pairing ������ Detection ������
	m_ntheNumberOfDetection = pair.pair_cnt;

	if (currentFrame == 16)
	{
		int a=0;
	}

#pragma region ���� �ʱ�ȭ
	if (currentFrame <= 1)
	{
		theNumberOfTracking = 0;

		// detection�� ��Ī�� ���Ǵ� tracker�� id �ʱ�ȭ
		for (i = 0; i < TRACKING_NUM; i++)
		{
			tracker[i].setTrackerIndex(i);
		}

		// ȭ�� ǥ�ÿ� tracker�� id �ʱ�ȭ
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
	// ���� tracker ������ �̿��Ͽ� ���� ����
	// Ÿ���� ��ġ�� ���� N�� ��ƼŬ�� ��� ũ��� ��ġ�� ����
	for (i = 0; i < theNumberOfTracking; i++)
	{
		trackingProcessing(dib, currentFrame, tracker[i], pair.pairinfo[0], otherTarget, 1);
	}

	// Thread �̿�� �Ʒ��� �߰�

#pragma endregion

#pragma region Association Check
	// detection�� tracker ���� ������ �˻�
	// tracker�� ���� N���� ��ƼŬ�� �̿��Ͽ� ������ ��
	MatchingPairBoxWithTracking(dib, m_ntheNumberOfDetection, theNumberOfTracking, pair.pairinfo, tracker);
#pragma endregion

#pragma region Tracking Part Step 2
	if (m_ntheNumberOfDetection != 0 || theNumberOfTracking != 0)
	{
		for (i = 0; i < theNumberOfTracking; i++)
		{
			// ���� ������ ���� ���� �����ӿ� �����Ǿ����� detection�� ��Ī���� ���� ��
			// ������ ������ �̿��Ͽ� ����ؼ� tracking ����
			if (tracker[i].getGenerationFrame() != currentFrame && !tracker[i].getMatchingflag())
			{
				// �׳� �Ķ���͸� �Ѱ��ְ� ���� ���ƾ� �ϱ� ������ -1�� �־���
				//pair.pairinfo[m_ntheNumberOfDetection].box_center_x = -1;
				otherTarget.myTargetNumber = tracker[i].getTrackerIndex();
				trackingProcessing(dib, currentFrame, tracker[i], pair.pairinfo[m_ntheNumberOfDetection], otherTarget, 2);
			}
			// ���� �����ӿ� �����Ǿ����� detection�� ��Ī���� ���� ��
			// ���ο� detection�� ���� ���Ӱ� �Ҵ� ���� tracking
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
			// detection�� ��Ī�� tracking
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

	// RF update�� �����ϱ� ���� �ٸ� target ���� ��ġ ������ �����ϴ� ��
	detection_n_tracking_countTargetLocation(otherTarget, pair.pairinfo, tracker, m_ntheNumberOfDetection, theNumberOfTracking, currentFrame);

#pragma endregion

#pragma region Tracking Part Step 3
	// ���������� Ÿ�� ������ ������Ʈ �� ��
	// ������ Ÿ���� Ȯ������ ���� �з����� ������Ʈ�� ����
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

	// Detection�� ������ �̿��Ͽ� ���ο� tracker ����
	if (processingStep == 0)
	{
		tracker.processGenerationTracker(grayCpy, ColorCpy, true, pairinfo.box_center_x, pairinfo.box_center_y, pairinfo.box_min_x, pairinfo.box_min_y, pairinfo.box_max_x, pairinfo.box_max_y);
	}

	// ���� tracker ������ �̿��Ͽ� ���� ����
	// Ÿ���� ��ġ�� ���� N�� ��ƼŬ�� ��� ũ��� ��ġ�� ����
	else if (processingStep == 1)
	{
		tracker.processTracking(grayCpy, ColorCpy);
	}
	// Detection�� matching �� ����� ���� 
	// tracker�� ���¸� ������Ʈ ��
	else if (processingStep == 2)
	{
		tracker.processUpdatingTrackerStates(grayCpy, ColorCpy, pairinfo.box_center_x, pairinfo.box_center_y, pairinfo.box_min_x, pairinfo.box_min_y, pairinfo.box_max_x, pairinfo.box_max_y);
	}
	// tracker�� ����, �ٸ� tracker�� �Ÿ�, tracker�� Ȯ������ ����
	// �з����� ������Ʈ�� ����
	else if (processingStep == 3)
	{
		tracker.processUpdatingClassifiers(grayCpy, ColorCpy, false, otherTarget, currentFrame, m_opticalFlow);
	}
}

/**
@brief			OpticalFlow ȣ�� �Լ�
@param			dib : �Է� ����
@param			nCurrentFrame : ���� ������ ��ȣ
*/
int CTracking::OpticalFlowProcessing(CDib& dib, int nCurrentFrame)
{
/*
	// ������ �� ���� ���� ����
	CDib cpyDib;
	BYTE** ptr;
	int nNewVehicleDirectResult = -1;		// -1:�ʱ�ȭ, 0:����, 1:��ȸ��, 2:��ȸ��	

	cpyDib.Copy(&dib);

	if (cpyDib.GetBitCount() != 8)
	{
		DibGrayscale(cpyDib);
	}

	ptr = cpyDib.GetPtr();

	// ���� �����Ӱ� ���ӵ� �������� �ƴϸ� Optical Flow�� ���� ���� �缳��
	if (nCurrentFrame == 1)
	{
		m_opticalFlow.SetPreviousImg(ptr, cpyDib.GetWidth(), cpyDib.GetHeight());
		m_nPrevFrameNum = nCurrentFrame;
		m_bUpdateMask = false;

	}
	else if (abs(nCurrentFrame - m_nPrevFrameNum) >= SKIP_FRAME_FOR_OPTICAL)			// 3 ������ �������� Optical Flow üũ
	{
		// Optical Flow ȣ�� //////////////////////////////////////////
		nNewVehicleDirectResult = m_opticalFlow.mainProcessing(ptr, m_bUpdateMask);
		//		nNewVehicleDirectResult = m_opticalFlow.mainProcessing_egoMotionCompensation_BM(ptr, m_bUpdateMask);
		m_nPrevFrameNum = nCurrentFrame;
	}

	return nNewVehicleDirectResult;			// ���� ����� ���� ��ȯ*/
	return 0;
}

/**
@brief			ȭ�� ����� ���� tracker�� id �Ҵ� ���� ����
@param			id : tracker�� id
*/
void CTracking::tracking_setAssignedTrackerID(int id)
{
	assignedTrackerID[1][id] = 0;
}

/**
@brief			������
�޸� �Ҵ�
���� �ʱ�ȭ
tracking ����
*/
void CTracking::trackingValueInit()
{
	register int i;
	theNumberOfTracking = 0;

	tracker = new CTracker[TRACKING_NUM];

	for (i = 0; i < TRACKING_NUM; i++)
	{
		//tracker �� �ʱ�ȭ
		tracker[i].valueInit();

		//tracker �ε��� �ʱ�ȭ
		tracker[i].setTrackerIndex(i);

		// �з��� ����
		tracker[i].generateClassifier();
	}
}

/**
@brief			������
�޸� �Ҵ�
���� �ʱ�ȭ
tracking by detection ����
*/
void CTracking::detection_n_tracking_ValueInit()
{
	//�����带 �̿��Ҷ� ����ϱ� ���� Ʈ��ŷ ������ ��� ����ü �ʱ�ȭ. ���� ��� x
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
@brief			������
�޸� �Ҵ�
���� �ʱ�ȭ
Regioninfo�ʱ�ȭ
*/
void CTracking::detectionValueInit()
{
	pair.pairinfo = new Pair[9999];

	// Optical Flow Ŭ����
	//m_opticalFlow.Func_Init(IMG_SIZE_HEIGHT, IMG_SIZE_WIDTH, OPTICALfLOW_GRID_INTERVAL);
	m_nPrevFrameNum = 99999;

	//m_opticalflow_result = new OPTICALFLOW_FEATURES[m_opticalFlow.getGridCount()];
}

/**
@brief		�ٸ� tracker�� ��ġ ������ ����
RF update�� �����ϱ� ���Ͽ� �����
@param		otherTarget : tracker���� ������ �����ϴ� ����ü
regionInfo1 : detection�� �� ����
trk : tracking�� ����
theNumberOfDetection : detection�� ��
theNumberOfTracking : tracker ��
currentFrame : ���� frame
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
		// �Ҵ���� �� �� �� tracker�� �Ÿ� ����� ���� ����
		if (currentFrame - trk[i].getGenerationFrame() > CHECKING_FRAME)
		{
			// cx, cy, w, h
			trk[i].getInformationOfTarget(otherTarget.location[i][0], otherTarget.location[i][1], otherTarget.location[i][2], otherTarget.location[i][3], otherTarget.targetProbability[i]);

			otherTarget.targetIndexs[i] = trk[i].getTrackerIndex();

			// ��ħ ������ ���� ��� Y ��ǥ ��ȯ �� ����
			// 3������ ����
			otherTarget.avgYaxis[i] = trk[i].getYaxis();
			trk[i].setYaxis();
			trk[i].setTrackerCountForYaxis();

			availableCount++;
		}
		else if (0 <= currentFrame - trk[i].getGenerationFrame() && currentFrame - trk[i].getGenerationFrame() <= CHECKING_FRAME)
		{
			// ��ħ ������ ���� ��� Y ��ǥ �� ����
			// 1~3������ ����
			trk[i].setYaxis();
			trk[i].setTrackerCountForYaxis();
		}
	}

	otherTarget.theNumberOfOtherTarget = availableCount;
}

/**
@brief		tracker ������ ȭ�鿡 ���
@param		dib : �����̹���
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
				// ȭ�� ǥ�ÿ� tracker�� id �Ҵ�
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
@brief			ȭ�� ����� ���� tracker�� id �Ҵ�
@return			tracker�� id
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
@brief			ȭ�� ����� ���� tracker�� id �Ҵ� ���� �� id ��ȯ
@param			row : 0 -> tracker�� id
1 -> tracker�� id �Ҵ� ����
id : tracker�� id
@return			tracker�� id �Ҵ� ���� or tracker�� id
*/
int CTracking::detection_n_tracking_getAssignedTrackerID(int row, int id)
{
	return assignedTrackerID[row][id];
}

/**
@brief		tracker ������ ȭ�鿡 ���
@param		dib : �����̹���
trk : tracking Ŭ���� ��ü
*/
void CTracking::trackingDrawRect(CDib& dib, CTracker& trk)
{
	trk.trackingDrawRect(dib);
}

/**
@brief		tracker ������ ȭ�鿡 ���
@param		dib : �����̹���
*/
void CTracking::detection_n_tracking_draw_tracking(CDib& dib)
{
	// �ӽ� �׽�Ʈ�� ���� �ڵ� �߰� JMR //////////////////////////////////////////////////////////////
	//	dib.Copy(&m_dibTestDisplay);

	trackingDraw(dib);
}

/**
@brief		tracker ID ���
@param		pDC : �ؽ�Ʈ ����� ���� �ڵ鷯
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
@brief		tracker ID ���
@param		pDC : �ؽ�Ʈ ����� ���� �ڵ鷯
trk : tracking Ŭ���� ��ü
i : tracker ID
*/
void CTracking::trackingDrawText(CDC *pDC, CTracker& trk, int i)
{
	trk.trackingDrawText(pDC, i);
}

/**
@brief		trakcing Ŭ���� ��ü �� ���� swap
@param		trk1 : tracking Ŭ���� ��ü
trk2 : tracking Ŭ���� ��ü
*/
void CTracking::swap(CTracker& trk1, CTracker& trk2)
{
	// tracking class�� swap�� ���� ����

	//	CTracking tempForSwap;

	tempForSwap = trk1;
	trk1 = trk2;
	trk2 = tempForSwap;
}

/**
@brief			�Ҹ���
�޸� ����
tracking by detection ����
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
@brief			�Ҹ���
�޸� ����
tracking ����
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

	// Optical Flow �Ҹ�
	//m_opticalFlow.Func_Destory();
}

/**
@brief		�Ҹ���
�޸� ����
detection ����
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

	// ������ �˻� �� �Ÿ� ����
	double range = 0.75; // 0.75

	for (i = 0; i < TRACKING_NUM; i++)
	{
		trk[i].setMatchingFlag(false);
	}

	if (count1 != 0)
	{
#pragma region hungarian method�� ����� score matrix �����
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

		// ���� ��ķ� ����� ���� ��
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

		// ��� detection�� tracking ���� score�� ����Ǵ� �迭
		// ���� ����� ����� ���ؼ� ���ڰ� ���ڶ� ���� 0�� �����
		// �� ����� ������ ��Ī�� ������
		// ��� ũ�Ⱑ 1 x 1 �� ��� 2 x 2�� �÷���
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

		// score ��Ʈ���� �ʱ�ȭ
		for (i = 0; i < size; i++)
		{
			for (j = 0; j < size; j++)
			{
				//	squareScoreMatrix[i][j] = ((rand() % (int)((SCORE_THRESHOLD - 0.1) * 10)) * 0.1) + 0.1;
				squareScoreMatrix[i][j] = 0.0;
			}
		}

		// ��� detection�� tracker ���� score ���� ��� �� ��
		// ��Ŀ� ����				
		for (i = 0; i < count1; i++)
		{
			// detection ������ ũ�� ���
			currentArea = (pair[i].boxWidth) * (pair[i].boxHeight);

			// detection ������ ũ�⿡ ���� ��Ī �� ���Ǵ� �Ÿ� �Ӱ谪�� �ٸ��� ����
			/*			if(currentArea <=  (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_6) * (DEFAULT_WINDOW_HEIGHT / IMG_LEVEL_RESIZE_RATE_6))
			{
			distThreshold = (DEFAULT_WINDOW_WIDTH / IMG_LEVEL_RESIZE_RATE_6) * range; // 0.75
			}
			else*/ 

			// �Ʒ� �ּ� ���߿� Ǯ����?
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

			// ������ ȸ���ϸ� �Ÿ� �Ӱ谪�� ũ�� ����
			// ������ ��� �Ӱ谪�� �״��
			// -1:�ʱ�ȭ, 0:����, 1:��ȸ��, 2:��ȸ��	
			//if ((vehicleDirection == 1 || vehicleDirection == 2) && USING_THRESHOLDS_ACCORDING_TO_ROTATION)
			//{
			//	distThreshold *= 1.5; //1.2
			//}


			// score ����� ���� �Ÿ��� Ȯ���� ���
			for (j = 0; j < count2; j++)
			{
				// �Ÿ� ����
				//distance[j] = trk[j].measureDistanceBetweenDetectionAndTracking(pair[i].box_center_x, pair[i].box_center_y);

				// Detection Region Ư¡ ���� �� RF Ȯ�� ����
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


				// detection window�� tracking window ���� width ���� ��
				//		widthDist[j] = fabs((trk[j].getWidthOfTrackingWindow() * 2.0) - (double)(regionInfo1[i].max_x - regionInfo1[i].min_x));

				// detection window�� tracking window ���� height ���� ��
				//		heightDist[j] = fabs((trk[j].getHeightOfTrackingWindow() * 2.0) - (double)(regionInfo1[i].max_y - regionInfo1[i].min_y));

				//		ratio[j] = fabs(1.0 - (((trk[j].getWidthOfTrackingWindow() * 2.0) * (trk[j].getHeightOfTrackingWindow() * 2.0)) / (double)((regionInfo1[i].max_x - regionInfo1[i].min_x) * (regionInfo1[i].max_y - regionInfo1[i].min_y))));

				//		if(ratio[j] > 1.0)
				//		{
				//			ratio[j] = 1.0;
				//		}

				if (distThreshold >= distance[j])
				{
					// �Ÿ��� ����ġ�� ����ϱ� ���� �� 
					distanceSum_dist += distance[j];

					// �Ÿ��� �Ӱ谪 ���� �۰�, 0�� ��츦 ���� ī��Ʈ��
					distCount++;
				}

				// width ���� ����ġ�� ����ϱ� ���� ��
				//		distanceSum_width += widthDist[j];

				// height ���� ����ġ�� ����ϱ� ���� ��
				//		distanceSum_height += heightDist[j];
			}

			// score ����
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
					// �Ÿ��� 0�� ���
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

		// �ʱ�ȭ
		for (i = 0; i < sCount; i++)
		{
			pair[i].particleNumber = -1;
		}

		// hungarian method
		x_y = matching.matchingProcessing(sCount, squareScoreMatrix);

		for (i = 0; i < count1; i++)
		{
			//dist = trk[x_y[i]].measureDistanceBetweenDetectionAndTracking(regionInfo1[i].center_x, regionInfo1[i].center_y);

			//// detection ������ ũ�� ���
			//currentArea = (regionInfo1[i].max_x - regionInfo1[i].min_x) * (regionInfo1[i].max_y - regionInfo1[i].min_y);

			//// detection ������ ũ�⿡ ���� ��Ī �� ���Ǵ� �Ÿ� �Ӱ谪�� �ٸ��� ����
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

			// matching�� �� ���
			if (squareScoreMatrix[i][x_y[i]] > threshold /*&& dist < distThreshold*/)		// 2013-12-13 JMR 80->100->20->100
			{
				// label �Ҵ�
				pair[i].particleNumber = trk[x_y[i]].getTrackerIndex();

				// matching �����Ƿ� occlusion count�� ����
				trk[x_y[i]].setMatchingFlag(true);
			}
			else
			{
				int a = 0;
			}
		}

		// matching ���� ���� detection���� ���ο� tracker�� �Ҵ��ϱ� ���� tracker �ʱ�ȭ
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

		// �޸� ����
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
	// tracking ���� detection �� ���� ���� ��� detection�� ��Ī���� ���� tracking�� occlusion count�� +1 �����ش�
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

	// ���ο� detection�� ���� �Ҵ� ���� tracking ���� �߰����ش�
	// �� ��ġ�� �� ������ ���� ������ �� occlusion count�� ���� �������� �ʱ� ���ؼ�
	count2 = tCount;
	// occlusion Count�� 100 �̻��� ��� ������ ����� ������ ���� ������ �����Ѵ�
	// tracker�� ������ �� 3������ ���� detection�� ��Ī�� ���� �ʾҴٸ�, �߸��� detection���� �Ǵ��ϰ� �Ҹ� ��Ų��	
	for (i = 0; i < count2; i++)
	{
		if ((trk[i].getOcclusionCount() >= OCCLUSION_THRESHOLD || trk[i].getNotDisplayCounnt() >= OCCLUSION_THRESHOLD)
			|| ((m_currentFrame - trk[i].getGenerationFrame() <= CHECKING_FRAME) && (trk[i].getOcclusionCount() >= CHECKING_THRESHOLD))
			|| (trk[i].getWidthOfTrackingWindow() <= 2 || trk[i].getHeightOfTrackingWindow() <= 4) && (m_currentFrame - trk[i].getGenerationFrame() > CHECKING_FRAME))
		{
			// ȭ�� ǥ�ÿ� tracker�� id �Ҵ� ����
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