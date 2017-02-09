#pragma once
#include <math.h>
#include "Dib.h"
#include "typedef.h"

#define _function_		_T(__FUNCTION__)
#define _line_			__LINE__


struct sColor{
	float mean_R;
	float mean_G;
	float mean_B;

	float variance_R;
	float variance_G;
	float variance_B;
};

struct sColorHisto {
	double R[256];
	double G[256];
	double B[256];
};

struct sWavelet {
	double* OCSLBP_wavelet;
	int size;
};

struct sHaar {
	double* HaarValue;
	int size;
};

struct sFeature {
	sColor color;
	sColorHisto colorHisto;
	sWavelet wavelet;
	sHaar haar;
};

struct sRegion {
	int w;
	int h;

	int min_x;
	int min_y;

	int max_x;
	int max_y;

	float type; // HEAD, TAIL, OTHERS

	int pairNumber;

	sFeature feature;
	
};

struct lb {
	sRegion* rg;
	int cnt;
};

/// ���� ���� �Ǵ�
// TEST MODE ( 0 : Tracking, 1 : Detection, 2 : Prediction )
#define FOR_DETECTION_TEST				2

/// detection =========================================
// detection ��Ʈ���� ������ ��� ���� ����
// 0 : ���
// 1 : ������� ����
#define USING_THREAD_IN_DETECTION		0

/// tracking ==========================================
// ������ ��� ����
// 0 : �����
// 1 : ������� ����
#define USING_THREAD					0

/// prediction ========================================
// display line detection
// 0 : ��� �� ��
// 1 : ��� ��
#define DISPLAY_IMAGE					0
// display simulation
// 0 : ��� �� ��
// 1 : ��� ��
#define DISPLAY_SIMULATION				0

// display head detection
// 0 : ��� �� ��
// 1 : ��� ��
#define DISPLAY_HEAD_WINDOW				0

// display head search region
// 0 : ��� �� ��
// 1 : ��� ��
#define DISPLAY_HEAD_SEARCH_REGION				0

// display optical flow 
// 0 : �Ϲ� optical flow
// 1 : prediction�� ���� optical flow
#define DISPALY_OPTICAL_FLOW_FOR_PREDICTION		1

// tracker�� �Ÿ� ǥ��
// 0 : ��� �� ��
// 1 : ��� ��
#define DISPLAY_DISTANCE_OF_TRACKER				0

// tracker�� �̵� ���� ǥ��
// 0 : ��� �� ��
// 1 : ��� ��
#define DISPLAY_DIRECTION_OF_TRACKER			1

// DFA mode
// 0 : ���� DFA
// 1 : ���ο� DFA
// 2 : ���� ����ȸ ��ǥ DFA
#define USING_DFA_MODE							2
///=========================================================================================

// for "Feature_Extraction.cpp"
#define CLASS_NUM 2
#define CLASS_POS 0
#define CLASS_NEG 1

#define RND(r) ((int)(r+0.5))
#define NEGATIVE_TO_ZERO(a) ((a < 0) ? 0 : a)

// Default Image Size
#define IMG_SIZE_WIDTH							640
#define IMG_SIZE_HEIGHT							480

// Default Window Size
#define DEFAULT_WINDOW_WIDTH					97 //60
#define DEFAULT_WINDOW_HEIGHT					20 //138

// Head Window Size
#define HEAD_WINDOW_WIDTH						18
#define HEAD_WINDOW_HEIGHT						32

// OCS-LBP
#define OCSHISTOGRAM_BINS						8
#define OCSLBP_THRESHOLD						6	
#define OCSLBP_WINDOW_FEATURE_SIZE				128
#define OCSLBP_WINDOW_FEATURE_SIZE_FOR_HEAD		32

// HOG
#define HOGHISTOGRAM_BINS						9
#define HOG_WINDOW_FEATURE_SIZE					36	// 9bin*4cell=36

// Pedestrians Detection Random Forest Tree 
#define DEC_RF_OCSLBP_THE_NUMBER_OF_TREE			150
#define DEC_RF_RATIO_OF_SAMPLEDATA					0.80
#define DEC_RF_OCSLBP_THE_NUMBER_OF_GOODTREE		100

// ROI 
#define ROI_START_RATIO_X		0//0.2
#define ROI_START_RATIO_Y		0.3
#define ROI_WIDTH_RATIO			1//0.6
#define ROI_HEIGHT_RATIO		0.4
#define FROI_START_RATIO_X		ROI_START_RATIO_X
#define FROI_START_RATIO_Y		ROI_START_RATIO_Y
#define FROI_WIDTH_RATIO		ROI_WIDTH_RATIO
#define FROI_HEIGHT_RATIO		0.18
#define BROI_START_RATIO_X		ROI_START_RATIO_X
#define BROI_START_RATIO_Y		0.5
#define BROI_WIDTH_RATIO		ROI_WIDTH_RATIO
#define BROI_HEIGHT_RATIO		0.1


//// Haar-Like
//#define HAARHISTOGRAM_BINS						1
//#define HAARLIKE_WINDOW_FEATURE_SIZE			10
//
//// HOG
//#define HOGHISTOGRAM_BINS						9
//#define HOG_WINDOW_FEATURE_SIZE					144	// 3*11*(9bin*4cell) = 1188
//
//// Detection Random fern
//#define RF_OCSLBP_THE_NUMBER_OF_FERN			150
//#define RF_OCSLBP_THE_NUMBER_OF_GOODFERN		100
//#define RF_OCSLBP_THE_NUMBER_OF_FUNCTION		10
//
//#define RF_HL_THE_NUMBER_OF_FERN				150
//#define RF_HL_THE_NUMBER_OF_GOODFERN			100
//#define RF_HL_THE_NUMBER_OF_FUNCTION			10
//
//// Ŭ���� �� Ȯ���� ���� ��, ����ȭ�� ���Ǵ� ���
//#define RF_DETECTION_NORMALIZE_FACTOR			1.0
//#define RF_DETECTION_CONSTANT					10000000.0


// detection ���� �� ��ŵ�� ������ ��
#define SKIP_FRAME								1
#define SKIP_FRAME_FOR_OPTICAL					3

// ROI������ ���� ��ŵ ������ ��
#define SKIP_FRAME_FOR_ROI						1

#define PI										3.141592653

// WindowMerging Ŭ������ m_fBaseY �迭�� �� ���� ũ�� ��ŭ �޸𸮰� �Ҵ� ��
// ���� ���� �� �ϵ� �ڵ����� �־��� ������ MAX_LEVEL ���� �����ϸ�
// �ش� �ϵ� �ڵ��� �ּ� ó�� ����� ��
#define MAX_LEVEL					6

#define IMG_LEVEL				6

// SOI(Scale of Interest) by Level
#define IMG_LEVEL_RESIZE_RATE_0	0.460
#define IMG_LEVEL_RESIZE_RATE_1	0.730
#define IMG_LEVEL_RESIZE_RATE_2	1.000
#define IMG_LEVEL_RESIZE_RATE_3	1.310
#define IMG_LEVEL_RESIZE_RATE_4	1.461
#define IMG_LEVEL_RESIZE_RATE_5	1.900
#define IMG_LEVEL_RESIZE_RATE_6	1.0

// window interval by Level
#define IMG_LEVEL_WD_INTERVAL_0 4
#define IMG_LEVEL_WD_INTERVAL_1 4
#define IMG_LEVEL_WD_INTERVAL_2	6
#define IMG_LEVEL_WD_INTERVAL_3	6
#define IMG_LEVEL_WD_INTERVAL_4	8
#define IMG_LEVEL_WD_INTERVAL_5	8
#define IMG_LEVEL_WD_INTERVAL_6	8

// Ȯ���� ����ȭ �� �� ���
// Haar-Like Random Forest Prob Threshold
#define IMG_LEVEL_HAARLIKE_RF_TH_0			0.54
#define IMG_LEVEL_HAARLIKE_RF_TH_1			0.53
#define IMG_LEVEL_HAARLIKE_RF_TH_2			0.53
#define IMG_LEVEL_HAARLIKE_RF_TH_3			0.53
#define IMG_LEVEL_HAARLIKE_RF_TH_4			0.53
#define IMG_LEVEL_HAARLIKE_RF_TH_5			0.54
#define IMG_LEVEL_HAARLIKE_RF_TH_6			0.5


// OCS-LBP Randeom Forest Prob Threshold by Level
// summer
#define S_IMG_LEVEL_OCSLBPRF_TH_0	0.9 //0.95 //0.79
#define S_IMG_LEVEL_OCSLBPRF_TH_1	0.9 //0.95 //0.79
#define S_IMG_LEVEL_OCSLBPRF_TH_2	0.74 //0.79 //0.79
#define S_IMG_LEVEL_OCSLBPRF_TH_3	0.75 //0.80 //0.80
#define S_IMG_LEVEL_OCSLBPRF_TH_4	0.73 //0.78 //0.78
#define S_IMG_LEVEL_OCSLBPRF_TH_5	0.71 //0.76 //0.76
#define S_IMG_LEVEL_OCSLBPRF_TH_6	1.50
// winter
#define W_IMG_LEVEL_OCSLBPRF_TH_0	0.79
#define W_IMG_LEVEL_OCSLBPRF_TH_1	0.79
#define W_IMG_LEVEL_OCSLBPRF_TH_2	0.79
#define W_IMG_LEVEL_OCSLBPRF_TH_3	0.80
#define W_IMG_LEVEL_OCSLBPRF_TH_4	0.78
#define W_IMG_LEVEL_OCSLBPRF_TH_5	0.76
#define W_IMG_LEVEL_OCSLBPRF_TH_6	1.0

// Y coordinate of ROI(Region of Interest) by Level
// summer
#define S_IMG_LEVEL_ROI_START_RATE_0	0.740
#define S_IMG_LEVEL_ROI_END_RATE_0		0.950
#define S_IMG_LEVEL_ROI_START_RATE_1	0.670
#define S_IMG_LEVEL_ROI_END_RATE_1		0.820
#define S_IMG_LEVEL_ROI_START_RATE_2	0.600
#define S_IMG_LEVEL_ROI_END_RATE_2		0.750
#define S_IMG_LEVEL_ROI_START_RATE_3	0.580
#define S_IMG_LEVEL_ROI_END_RATE_3		0.650
#define S_IMG_LEVEL_ROI_START_RATE_4	0.540
#define S_IMG_LEVEL_ROI_END_RATE_4		0.640
#define S_IMG_LEVEL_ROI_START_RATE_5	0.520
#define S_IMG_LEVEL_ROI_END_RATE_5		0.610
#define S_IMG_LEVEL_ROI_START_RATE_6	0.528
#define S_IMG_LEVEL_ROI_END_RATE_6		0.576
// winter
#define W_IMG_LEVEL_ROI_START_RATE_0	0.740
#define W_IMG_LEVEL_ROI_END_RATE_0		0.950
#define W_IMG_LEVEL_ROI_START_RATE_1	0.670
#define W_IMG_LEVEL_ROI_END_RATE_1		0.820
#define W_IMG_LEVEL_ROI_START_RATE_2	0.600
#define W_IMG_LEVEL_ROI_END_RATE_2		0.750
#define W_IMG_LEVEL_ROI_START_RATE_3	0.580
#define W_IMG_LEVEL_ROI_END_RATE_3		0.650
#define W_IMG_LEVEL_ROI_START_RATE_4	0.540
#define W_IMG_LEVEL_ROI_END_RATE_4		0.640
#define W_IMG_LEVEL_ROI_START_RATE_5	0.520
#define W_IMG_LEVEL_ROI_END_RATE_5		0.610
#define W_IMG_LEVEL_ROI_START_RATE_6	0.528
#define W_IMG_LEVEL_ROI_END_RATE_6		0.576
//#define W_IMG_LEVEL_ROI_START_RATE_0	1.000
//#define W_IMG_LEVEL_ROI_END_RATE_0		0.960
//#define W_IMG_LEVEL_ROI_START_RATE_1	0.840
//#define W_IMG_LEVEL_ROI_END_RATE_1		0.980
//#define W_IMG_LEVEL_ROI_START_RATE_2	0.750
//#define W_IMG_LEVEL_ROI_END_RATE_2		0.860
//#define W_IMG_LEVEL_ROI_START_RATE_3	0.730
//#define W_IMG_LEVEL_ROI_END_RATE_3		0.770
//#define W_IMG_LEVEL_ROI_START_RATE_4	0.720
//#define W_IMG_LEVEL_ROI_END_RATE_4		0.750
//#define W_IMG_LEVEL_ROI_START_RATE_5	0.710
//#define W_IMG_LEVEL_ROI_END_RATE_5		0.730
//#define W_IMG_LEVEL_ROI_START_RATE_6	0.0
//#define W_IMG_LEVEL_ROI_END_RATE_6		0.0

#define IMG_ROI_START	0.52
#define IMG_ROI_END		0.95

// for Detection_n_Tracking.h
#define CHECKING_FRAME			4
#define OCCLUSION_THRESHOLD		20
#define CHECKING_THRESHOLD		1



///===========================================================================================

// for "Tracking.h"
// ���� ũ��
#define IMAGE_WIDTH								640
#define IMAGE_HEIGHT							480

// �н� ������ ���� �迭�� ũ��
#define MAX_SIZE								1000

// ���� ������ �޸� ��ü ��
#define TRACKING_NUM							15

// �з��� Ŭ����
#define PART_CLASSES							2

// ������ �� ��Ʈ ��
#define SUB_BLOCK								16

// ��ƼŬ ��
#define PARTICLE_NUMBER							90


// ��� ä�� ��
//#define USING_CHANNEL							1



// 0 : OCS-LBP, LID 4 x 4
// 1 : OCS-LBP + LID 1 x 1
// 2 : OCS-LBP + LID 4 x 4
// 3 : HOG + LID 4 x 4
#define FEATURE_TYPE							0 // 2

#if FEATURE_TYPE == 0
	// Ư¡ �⺻ ���� ��
	#define FEATURE_DEMENSION						8 // 8
	#define FEATURE_COLOR_DEMENSION					24
	#define COLOR_DISTANCE							8

	// OCS-LBP Ư¡ ������
	#define OCSLBP_FEATURE_SIZE						128
	#define INTENSITY_QUANTITY						32

	// Color Ư¡ ������
	// 2 ä�� ���
	#define LID_FEATURE_SIZE						128
	#define NEW_FEATURE_SIZE						2048

	#define COLOR_FEATURE_SIZE_FOR_TRACKING						384
#elif FEATURE_TYPE == 1
	// Ư¡ �⺻ ���� ��
	#define FEATURE_DEMENSION						8 // 8
	#define NEW_FEATURE_SIZE						2048
	#define INTENSITY_QUANTITY						1

	// OCS-LBP Ư¡ ������
	#define OCSLBP_FEATURE_SIZE						128
	// Color Ư¡ ������
	// 2 ä�� ���
	#define LID_FEATURE_SIZE						128
#elif FEATURE_TYPE == 2
	// Ư¡ �⺻ ���� ��
	#define FEATURE_DEMENSION						128 // 8
	#define NEW_FEATURE_SIZE						2048
	#define INTENSITY_QUANTITY						16

	// OCS-LBP Ư¡ ������
	#define OCSLBP_FEATURE_SIZE						128
	// Color Ư¡ ������
	// 2 ä�� ���
	#define LID_FEATURE_SIZE						128
#elif FEATURE_TYPE == 3
	// Ư¡ �⺻ ���� ��
	#define FEATURE_DEMENSION						144 // 8
	#define NEW_FEATURE_SIZE						2304
	#define INTENSITY_QUANTITY						16

	// OCS-LBP Ư¡ ������
	#define OCSLBP_FEATURE_SIZE						128
	// Color Ư¡ ������
	// 2 ä�� ���
	#define LID_FEATURE_SIZE						128
#endif

// �н� ������ ���� ��
// �����Ͱ� ����Ǵ� ��Ʈ �����̵� ����
#define SCANNING_RANGE_X_AT_TRAINING			6
#define SCANNING_RANGE_Y_AT_TRAINING			6

#define DISTANCE_BETWEEN_OTHERS					20

//// Random Forest
//// OCS-LBP
//#define TRA_RF_OCSLBP_THE_NUMBER_OF_TREE			150
//#define TRA_RF_OCSLBP_RATIO_OF_SAMPLEDATA			0.80
//#define TRA_RF_OCSLBP_THE_NUMBER_OF_GOODTREE		100
//// Color
//#define TRA_RF_COLOR_THE_NUMBER_OF_TREE			150
//#define TRA_RF_COLOR_RATIO_OF_SAMPLEDATA		0.80
//#define TRA_RF_COLOR_THE_NUMBER_OF_GOODTREE		100

// OCS-LBP Ư¡�� ���
// RF�� ���Ǵ� Ʈ����
#define THE_NUMBER_OF_FERN_1					60
// RF�� ���Ǵ� ����� ����
#define THE_NUMBER_OF_FUNCTION_1				8
// ���� ����� Ʈ�� ��
#define THE_NUMBER_OF_GOOD_FERN_1				40

// Color Ư¡�� ���
// RF�� ���Ǵ� Ʈ����
#define THE_NUMBER_OF_FERN_2					60
// RF�� ���Ǵ� ����� ����
#define THE_NUMBER_OF_FUNCTION_2				8
// ���� ����� Ʈ�� ��
#define THE_NUMBER_OF_GOOD_FERN_2				40

// Ŭ���� �� Ȯ���� ���� ��, ����ȭ�� ���Ǵ� ���
#define RF_TRACKING_NORMALIZE_FACTOR			1.0
#define RF_TRACKING_CONSTANT					10000000.0

// histoty length
#define HISTORY_LENGTH							0

// Tracking ��������/��ħ ���� RF MAX Ȯ�� (0.7->0.6)
#define TRACKING_PRO_MAX_TH						0.625 //0.625
// Tracking ��������/��ħ ���� RF MIN Ȯ�� (0.25)
#define TRACKING_PRO_MIN_TH						0.45  // 0.45

//// ��ܽ� ����� �Ӱ谪 - Ư¡ ���� �ܰ�
//#define CASCADE_TH_1							0.5
//// ��ܽ� ����� �Ӱ谪 - ��Ī �ܰ�
//#define CASCADE_TH_2							0.5

// ȭ�鿡 �߰������� ����� ����
// 0 : ��Ʈ ���� ����
// 1 : ��ƼŬ ���ø� ����
// 2 : ǥ������ ����
#define DISPLAY_TYPE							1 //2

// ������ �� ��Ʈ ����
// 0 : �j�� ���� type-01
// 1 : �޸� ���� type-02
// 2 : �׸��� ����
#define PART_TYPE								2

// �� �Լ����� ���Ǵ� ���� N���� ��ƼŬ�� �����ϱ� ���� ����
#define PARTICLE_RATE							0.15

// ���� tracker�� ���� tracker ���� �Ÿ� �Ӱ谪 ������ ���� ����
#define DIST_RATE								0.5 //0.5

// ��ƼŬ ���ø� ���� ����
#define RANGE_RATE								0.3

// �� Ư¡�� Ȯ���� ����ġ
// OCSLBP_WEIGHT + LID_WEIGHT = 1.0
#define OCSLBP_WEIGHT							0.5
#define LID_WEIGHT								0.5

// detection ������ ���� ����ġ
#define DETECTION_WEIGHT						0.5
// ���� tracker ������ ���� ����ġ
#define CURR_TRACKER_WEIGHT1					0.5
// ���� tracker ������ ���� ����ġ
#define CURR_TRACKER_WEIGHT2					0.5
// ���� tracker ������ ���� ����ġ
#define PRE_TRACKER_WEIGHT						0.5


// �ٸ� tracker�� ��Ī �Ÿ� �Ӱ谪
//#define DISTANCE_THRESHOLD						25.0

// ������ �˻��� score �Ӱ谪
#define SCORE_THRESHOLD							0.6  // 0.48
#define PAIR_SCORE_THRESHOLD					0.65


// ��Ī �� ���Ǵ� �Ÿ����� ����ġ
//#define	DIST_WEIGHT								0.33
// ��Ī �� ���Ǵ� Ȯ������ ����ġ
#define	PROB_WEIGHT								0.5
// ��Ī �� ���Ǵ� ũ�� ������ ����ġ
#define	SIZE_WEIGHT								0.5


// tracker �� ��ħ�� ������ üũ�ϴ� ����
#define OVERLAP_RATIO_TH						0.15


// Ȯ������ ������� �ʴ� ��Ʈ ��
//#define NOT_USING_PARTS							5

// Optical Flow�� Features ����Ʈ�� Grid ����
#define OPTICALfLOW_GRID_INTERVAL				10

// ȸ���� ���� �Ÿ� �Ӱ谪�� �޸� �ִ� ���
// 0 : ������
// 1 : ����� 
#define USING_THRESHOLDS_ACCORDING_TO_ROTATION		0

// Tracker Ȯ������ ������ �̵� ���� ǥ��
// 0 : ǥ�þ���
// 1 : ǥ����
#define DISPLAY_TRACKERS_PROBABILIRY_AND_VEHICLE_DIRECTION		0



///=============================================================================
// for "Prediction.h"
#define STATE					4
#define MATRIX_SIZE				STATE * STATE

// Head Detection Random Forest Tree 
#define HD_DEC_RF_OCSLBP_THE_NUMBER_OF_TREE			75
#define HD_DEC_RF_OCSLBP_THE_NUMBER_OF_GOODTREE		35


///=============================================================================
///=================================Detection Part==============================
///=============================================================================

#define PI	acos(-1.0)
#define RND(r) ((int)(r+0.5))
#define NEGATIVE_TO_MINUS(a) max(0, a)
#define MIN_VAL(a, b) (a<b ? a:b)
#define MAX_VAL(a, b) (a>b ? a:b)

#define _function_		_T(__FUNCTION__)
#define _line_			__LINE__

#define NORM_IMAGE_SIZE		200			//����ȭ �̹��� ũ�� (������)

#define RF_POINT_REGRESSION_TREE	0	//150825 regression tree ���
#define RF_CLASSIFICATION_TREE		1	//151029 classification tree ���

//#define TEST_SAVE_DETECTION	1//test��� ���� (detection)
#define TEST_SAVE_ALIGNMENT	1//test��� ���� (alignment)
//#define TEST_SAVE_GAZEESTIMATION	1//test��� ���� (gaze estimation)

//#define STILL_IMAGE			// ���� �׽�Ʈ ������ ��ƿ���� ���� (�������� �������� �ƴҰ��, ��ü shape model�� ���� regression ����)

#pragma region about_window
//detection window max num
#define MAX_DETECTION_WINDOW			500

#define IMAGE_LEVEL			4		//���� ũ�� ���� : default 7
#if IMAGE_LEVEL == 7
#define IMAGE_SCALE_FACTOR_0	0.63 //���� scale ���� for 7-level
#define IMAGE_SCALE_FACTOR_1	0.7 
#define IMAGE_SCALE_FACTOR_2	0.8
#define IMAGE_SCALE_FACTOR_3	0.9
#define IMAGE_SCALE_FACTOR_4	1.0
#define IMAGE_SCALE_FACTOR_5	1.1
#define IMAGE_SCALE_FACTOR_6	1.2
#elif IMAGE_LEVEL == 4
#define IMAGE_SCALE_FACTOR_0	0.63
#define IMAGE_SCALE_FACTOR_1	0.8
#define IMAGE_SCALE_FACTOR_2	1.0
#define IMAGE_SCALE_FACTOR_3	1.2
#define IMAGE_SCALE_FACTOR_4	1.0
#define IMAGE_SCALE_FACTOR_5	1.0
#define IMAGE_SCALE_FACTOR_6	1.0
#else
#define IMAGE_SCALE_FACTOR_0	0.9 //���� scale ���� for 3-level
#define IMAGE_SCALE_FACTOR_1	1.0 
#define IMAGE_SCALE_FACTOR_2	1.3
#endif

#define NOSE_LEVEL					2
#define NOSE_IMAGE_SCALE_FACTOR_0	0.9
#define NOSE_IMAGE_SCALE_FACTOR_1	1.0
#define NOSE_IMAGE_SCALE_FACTOR_2	1.1


//search window �̵� ����
#define DETECT_WIN_STRIDE_0			8 / IMAGE_SCALE_FACTOR_0 //4	
#define DETECT_WIN_STRIDE_1			8 / IMAGE_SCALE_FACTOR_1 //6
#define DETECT_WIN_STRIDE_2			8 / IMAGE_SCALE_FACTOR_2
#if IMAGE_LEVEL == 7
#define DETECT_WIN_STRIDE_3			8 / IMAGE_SCALE_FACTOR_3
#define DETECT_WIN_STRIDE_4			8 / IMAGE_SCALE_FACTOR_4
#define DETECT_WIN_STRIDE_5			8 / IMAGE_SCALE_FACTOR_5
#define DETECT_WIN_STRIDE_6			8 / IMAGE_SCALE_FACTOR_6
#define DETECT_WIN_STRIDE_NOSE		6	//for 7-level
#elif IMAGE_LEVEL == 4
#define DETECT_WIN_STRIDE_3			8 / IMAGE_SCALE_FACTOR_3
#define DETECT_WIN_STRIDE_4			8 / IMAGE_SCALE_FACTOR_4
#define DETECT_WIN_STRIDE_5			8 / IMAGE_SCALE_FACTOR_5
#define DETECT_WIN_STRIDE_6			8 / IMAGE_SCALE_FACTOR_6
#define DETECT_WIN_STRIDE_NOSE		6	//for 7-level
#else
#define DETECT_WIN_STRIDE_NOSE		8	//for 3-level
#endif

#if IMAGE_LEVEL == 7
#define WINDOW_SIZE_FOR_IMAGE_PYRAMID_X	146 //156 //170
#define WINDOW_SIZE_FOR_IMAGE_PYRAMID_Y	146 //156 //170 
#define WINDOW_SIZE_FOR_IMAGE_PYRAMID_X_NOSE		60 //70	//nose
#define WINDOW_SIZE_FOR_IMAGE_PYRAMID_Y_NOSE		46 //60	//nose
#elif IMAGE_LEVEL == 4
#define WINDOW_SIZE_FOR_IMAGE_PYRAMID_X	146 //156 //170
#define WINDOW_SIZE_FOR_IMAGE_PYRAMID_Y	146 //156 //170 
#define WINDOW_SIZE_FOR_IMAGE_PYRAMID_X_NOSE		60 //70	//nose
#define WINDOW_SIZE_FOR_IMAGE_PYRAMID_Y_NOSE		46 //60	//nose
#else
#define WINDOW_SIZE_FOR_IMAGE_PYRAMID_X	170
#define WINDOW_SIZE_FOR_IMAGE_PYRAMID_Y	200
#define WINDOW_SIZE_FOR_IMAGE_PYRAMID_X_NOSE	69	//nose
#define WINDOW_SIZE_FOR_IMAGE_PYRAMID_Y_NOSE	62	//nose
#endif

#define IMAGE_PYRAMID_WIN_SIZE_IDX		2

//window size
#define WINDOW_SIZE_X_0			WINDOW_SIZE_FOR_IMAGE_PYRAMID_X			
#define WINDOW_SIZE_Y_0			WINDOW_SIZE_FOR_IMAGE_PYRAMID_Y
#define WINDOW_SIZE_X_NOSE		WINDOW_SIZE_FOR_IMAGE_PYRAMID_X_NOSE	//nose
#define WINDOW_SIZE_Y_NOSE		WINDOW_SIZE_FOR_IMAGE_PYRAMID_Y_NOSE	//nose

#pragma endregion about_window

#pragma region about_feature
#define NUM_OF_FEATURE_FOR_DETECTION	2


// OCS-LBP ���� 
#define OCSLBP_ACCUM_TYPE		OCSLBP_ACCUM_DIST
#define OCSHISTOGRAM_BINS		8	//ocs-lbp bin ���� 
#define OCSLBP_THRESHOLD		6	//���� ���� �ȼ��� �Ӱ�ġ

#define LBP_BLK_WIDTH_F_WIN			8	//LBP ������ ��� ���� ���� (������ ��)
#define LBP_BLK_HEIGHT_F_WIN		LBP_BLK_WIDTH_F_WIN	//LBP ������ ��� ���� ���� (������ ��)
#define LBP_NUM_OF_PIX_WID_F_WIN	WINDOW_SIZE_X_0/LBP_BLK_WIDTH_F_WIN			//LBP ��ϴ� ���� �ȼ��� (������ ��)
#define LBP_NUM_OF_PIX_HEI_F_WIN	WINDOW_SIZE_Y_0/LBP_BLK_WIDTH_F_WIN			//LBP ��ϴ� ���� �ȼ��� (������ ��)
#define OCSLBP_FEATURE_SIZE_WIN		OCSHISTOGRAM_BINS*LBP_BLK_WIDTH_F_WIN*LBP_BLK_HEIGHT_F_WIN

#define LBP_BLK_WIDTH_F_WIN_NOSE			4								//LBP ������ ��� ���� ���� (������ ��)
#define LBP_BLK_HEIGHT_F_WIN_NOSE			LBP_BLK_WIDTH_F_WIN_NOSE		//LBP ������ ��� ���� ���� (������ ��)
#define LBP_NUM_OF_PIX_WID_F_WIN_NOSE		WINDOW_SIZE_X_NOSE/LBP_BLK_WIDTH_F_WIN_NOSE			//LBP ��ϴ� ���� �ȼ��� (������ ��)
#define LBP_NUM_OF_PIX_HEI_F_WIN_NOSE		WINDOW_SIZE_Y_NOSE/LBP_BLK_WIDTH_F_WIN_NOSE			//LBP ��ϴ� ���� �ȼ��� (������ ��)
#define OCSLBP_FEATURE_SIZE_WIN_NOSE		OCSHISTOGRAM_BINS*LBP_BLK_WIDTH_F_WIN_NOSE*LBP_BLK_HEIGHT_F_WIN_NOSE

// Haar-Like ����
#define HAARLIKE_BLK_WIDTH_F_WIN			8	//Haar-like ������ ��� ���� ���� (������ ��)
#define HAARLIKE_BLK_HEIGHT_F_WIN			HAARLIKE_BLK_WIDTH_F_WIN	//Haar-like ������ ��� ���� ���� (������ ��)
#define HAARLIKE_WINDOW_FEATURE_SIZE		61	//Haarlike Ư¡ ũ��(���ϼ�)

#define HAARLIKE_BLK_WIDTH_F_WIN_NOSE		8				//Haar-like ������ ��� ���� ���� (������ ��)
#define HAARLIKE_BLK_HEIGHT_F_WIN_NOSE		HAARLIKE_BLK_WIDTH_F_WIN_NOSE	//Haar-like ������ ��� ���� ���� (������ ��)
#define HAARLIKE_WINDOW_FEATURE_SIZE_NOSE	61				//Haarlike Ư¡ ũ��(���ϼ�)

#define FILE_PATH_DETECT_FEATURE_OCSLBP_FOR_DAY			_T("..\\OCSLBP-Feature_for_DayFace.txt")
#define FILE_PATH_DETECT_FEATURE_HAAR_FOR_DAY			_T("..\\HL-Feature_for_DayFace.txt")
#define FILE_PATH_DETECT_FEATURE_OCSLBP_NOSE_FOR_DAY	_T("..\\OCSLBP-Feature_for_DayNose.txt")
#define FILE_PATH_DETECT_FEATURE_HAAR_NOSE_FOR_DAY		_T("..\\HL-Feature_for_DayNose.txt")
#define FILE_PATH_DETECT_FEATURE_OCSLBP_FOR_NIGHT		_T("..\\OCSLBP-Feature_for_NightFace.txt")
#define FILE_PATH_DETECT_FEATURE_HAAR_FOR_NIGHT			_T("..\\HL-Feature_for_NightFace.txt")
#define FILE_PATH_DETECT_FEATURE_OCSLBP_NOSE_FOR_NIGHT	_T("..\\OCSLBP-Feature_for_NightNose.txt")
#define FILE_PATH_DETECT_FEATURE_HAAR_NOSE_FOR_NIGHT	_T("..\\HL-Feature_for_NightNose.txt")

// Head light
#define FILE_PATH_DETECT_FEATURE_COLOR	_T("..\\input_color.txt")
#define FILE_PATH_DETECT_FEATURE_WAVELET	_T("..\\input_wavelet.txt")
#define FILE_PATH_DETECT_FEATURE_HAAR	_T("..\\input_haar.txt")
#define FILE_PATH_DETECT_FEATURE_WAVELET_TAIL	_T("..\\input_wavelet_tail.txt")
#define FILE_PATH_DETECT_FEATURE_COLOR_HISTO	_T("..\\input_color_histo.txt")

#define FILE_PATH_DETECT_RF_TREE_COLOR	_T("..\\color_tree.txt")
#define FILE_PATH_DETECT_RF_TREE_WAVELET	_T("..\\wavelet_tree.txt")
#define FILE_PATH_DETECT_RF_TREE_HAAR	_T("..\\haar_tree.txt")
#define FILE_PATH_DETECT_RF_TREE_WAVELET_TAIL	_T("..\\wavelet_tree_tail.txt")
#define FILE_PATH_DETECT_RF_TREE_COLOR_HISTO	_T("..\\color_tree_histo.txt")

#pragma endregion about_feature

#pragma region about_rf
//class
#define CLASS_NUM	2
#define CLASS_POS	0
#define CLASS_NEG	1
#define CLASSES_TRAINING	CLASS_NUM
#define CLASSES_TESTING		CLASSES_TRAINING

// Detection Random Forest Tree 
#define RF_RATIO_OF_SAMPLEDATA_F_WIN		0.8
#define RF_OCSLBP_NUMBER_OF_TREE_F_WIN		100
#define RF_OCSLBP_NUMBER_OF_GOODTREE_F_WIN	RF_OCSLBP_NUMBER_OF_TREE_F_WIN 
#define RF_HAAR_NUMBER_OF_TREE_F_WIN		30
#define RF_HAAR_NUMBER_OF_GOODTREE_F_WIN	RF_HAAR_NUMBER_OF_TREE_F_WIN

// Head light
#define RF_NUMBER_OF_TREE_F_WIN		50
#define RF_NUMBER_OF_GOODTREE_F_WIN	RF_NUMBER_OF_TREE_F_WIN 


#define RF_RATIO_OF_SAMPLEDATA_F_WIN_NOSE				0.8
#define RF_OCSLBP_NUMBER_OF_TREE_F_WIN_NOSE				100
#define RF_OCSLBP_NUMBER_OF_GOODTREE_F_WIN_NOSE			40 
#define RF_HAAR_NUMBER_OF_TREE_F_WIN_NOSE				30
#define RF_HAAR_NUMBER_OF_GOODTREE_F_WIN_NOSE			20


#define FILE_PATH_DETECT_TREE_OCSLBP_FOR_DAY			_T("..\\OCSLBP-Feature_Tree_for_DayFace.txt")
#define FILE_PATH_DETECT_TREE_HAAR_FOR_DAY				_T("..\\HL-Feature_Tree_for_DayFace.txt")
#define FILE_PATH_DETECT_TREE_OCSLBP_NOSE_FOR_DAY		_T("..\\OCSLBP-Feature_Tree_for_DayNose.txt")
#define FILE_PATH_DETECT_TREE_HAAR_NOSE_FOR_DAY			_T("..\\HL-Feature_Tree_for_DayNose.txt")
#define FILE_PATH_DETECT_TREE_OCSLBP_FOR_NIGHT			_T("..\\OCSLBP-Feature_Tree_for_NightFace.txt")
#define FILE_PATH_DETECT_TREE_HAAR_FOR_NIGHT			_T("..\\HL-Feature_Tree_for_NightFace.txt")
#define FILE_PATH_DETECT_TREE_OCSLBP_NOSE_FOR_NIGHT		_T("..\\OCSLBP-Feature_Tree_for_NightNose.txt")
#define FILE_PATH_DETECT_TREE_HAAR_NOSE_FOR_NIGHT		_T("..\\HL-Feature_Tree_for_NightNose.txt")
#define FILE_PATH_GAZEESTIMATION_TREE_OCSLBP_FOR_DAY	_T("..\\OCSLBP-Feature_Tree_for_DayGaze.txt")
#define FILE_PATH_GAZEESTIMATION_TREE_OCSLBP_FOR_NIGHT	_T("..\\OCSLBP-Feature_Tree_for_NightGaze.txt")
#define FILE_PATH_GAZEESTIMATION_TREE_HAAR_FOR_DAY		_T("..\\HL-Feature_Tree_for_DayGaze.txt")
#define FILE_PATH_GAZEESTIMATION_TREE_HAAR_FOR_NIGHT	_T("..\\HL-Feature_Tree_for_NightGaze.txt")
#pragma endregion about_rf

#pragma region about_th
// �Ӱ谪
#define DETECT_PROB_TH_HL_0		0.5
#define DETECT_PROB_TH_HL_1		0.5
#define DETECT_PROB_TH_HL_2		0.5
#define DETECT_PROB_TH_HL_3		0.5
#define DETECT_PROB_TH_HL_4		0.5
#define DETECT_PROB_TH_HL_5		0.5
#define DETECT_PROB_TH_HL_6		0.5

#define DETECT_PROB_TH_OCS_0	0.5
#define DETECT_PROB_TH_OCS_1	0.5
#define DETECT_PROB_TH_OCS_2	0.5
#define DETECT_PROB_TH_OCS_3	0.5
#define DETECT_PROB_TH_OCS_4	0.5
#define DETECT_PROB_TH_OCS_5	0.5
#define DETECT_PROB_TH_OCS_6	0.5

#define DETECT_PROB_TH_HL_NOSE	0.3
#define DETECT_PROB_TH_OCS_NOSE	0.5
#pragma endregion about_th

#define FEATURE_NUM_FOR_FACE_DETECTION	2
#define FEATURE_NUM_FOR_NOSE_DETECTION	2

#define DETECTION_FRAME_PERIOD		12		//detection ���� ���� default :12->10 2016-10-26 ����

#define DRAW	true
#define NO_DRAW	false

#define TIME_DAY_DATA	0		// �ְ� ����
#define TIME_NIGHT_DATA	1		// �߰� ����



//#define DETECTION_WINDOW_EXPAND_MARGIN_FACE		20	//detection ��� window Ȯ�� pixel ��
//#define DETECTION_WINDOW_EXPAND_MARGIN_NOSE		10	


//#define USE_PARALLEL_PROCESSING		//151001 ���� ó�� �÷��� �߰�


///=============================================================================
///=================================Pairing Part================================
///=============================================================================

// Association Check�Ҷ� score�� Threshold
#define PAIR_SCORE_THRESHOLD					0.6

/*
ROI�� 5��� �� �� y�� ����(?)
FIRST���� Y���� ����. �� ȭ�� ��
*/
#define PAIR_ROI_CROP_FIRST						2
#define PAIR_ROI_CROP_SECOND					4
#define PAIR_ROI_CROP_THIRD						6
#define PAIR_ROI_CROP_FOURTH					8
#define PAIR_ROI_CROP_OVERLAP_RATIO				0.3

/*
ROI ������ �� Blob���� ���ΰŸ�(Inner_Width) Threshold
FIRST���� Y���� ����. �� ȭ�� ��
*/
#define PAIR_BLOB_FIRST_DISTANCE_MIN				2
#define PAIR_BLOB_SECOND_DISTANCE_MIN				2//10
#define PAIR_BLOB_THIRD_DISTANCE_MIN				2//20
#define PAIR_BLOB_FOURTH_DISTANCE_MIN				2//22
#define PAIR_BLOB_FIFTH_DISTANCE_MIN				2//30

#define PAIR_BLOB_FIRST_DISTANCE_MAX				15
#define PAIR_BLOB_SECOND_DISTANCE_MAX				15
#define PAIR_BLOB_THIRD_DISTANCE_MAX				60
#define PAIR_BLOB_FOURTH_DISTANCE_MAX				120
#define PAIR_BLOB_FIFTH_DISTANCE_MAX				150


/*
ROI ������ �� Blob���� ���ΰŸ�(Inner_Width) Threshold
Taillight�� ��쿡
*/
#define PAIR_BLOB_TAIL_FIRST_DISTANCE_MIN				15
#define PAIR_BLOB_TAIL_SECOND_DISTANCE_MIN				20
#define PAIR_BLOB_TAIL_THIRD_DISTANCE_MIN				20
#define PAIR_BLOB_TAIL_FOURTH_DISTANCE_MIN				15
#define PAIR_BLOB_TAIL_FIFTH_DISTANCE_MIN				15

#define PAIR_BLOB_TAIL_FIRST_DISTANCE_MAX				150
#define PAIR_BLOB_TAIL_SECOND_DISTANCE_MAX				150
#define PAIR_BLOB_TAIL_THIRD_DISTANCE_MAX				150
#define PAIR_BLOB_TAIL_FOURTH_DISTANCE_MAX				150
#define PAIR_BLOB_TAIL_FIFTH_DISTANCE_MAX				150

/* 
HIGHBEAM = 0
LOWBEAM = 1
*/
#define HIGH_BEAM					0
#define LOW_BEAM					1

/* 
HIGH-BEAM�� LOW-BEAM ��ǥ�� �� FRAME üũ�Ұ���
HIGH���� LOW �������� 20FRAME �������� ���� 20FRAME�� ��� HIGH���� ��� 10FRAME ���� LOW�� ���� �� �ڿ� LOW�� �ٲ��.
*/
#define HL_FRAME_CHECK				40


extern int gRFtype;


void Trace(TCHAR* pszFormat, ...);
bool IsValidFor8bitPixelVal(int val);
void L2Normalization(double* data, int size);
double GetDistanceBetween2Points(Point point1, Point point2);
void free_memory(void* arr, bool setnullptr);
Point GetCenterPointofRect(Point startp, Point endp);
Point SetPoint(int x, int y);
Point GetPointAfterExceptionHandling(Point cor, int xmin, int xmax, int ymin, int ymax);
Point GetRotatedPoint(Point centroid, Point point, double degree);

// ǥ������ ���ϴ� �Լ�
double CalculateStandardDeviation_FOR_BYTE(CDib& dib, double meanValue, int width, int height);
void CalculateStandardDeviation_FOR_RGBBYTE(CDib& dib, double* meanValue, int width, int height, double* std);

// ��� ���ϴ� �Լ�
double CalculateMeanValue_FOR_BYTE(CDib& dib, int width, int height);
void CalculateMeanValue_FOR_RGBBYTE(CDib& dib, int width, int height, double* mean);
