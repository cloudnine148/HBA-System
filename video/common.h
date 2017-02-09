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

/// 실행 유무 판단
// TEST MODE ( 0 : Tracking, 1 : Detection, 2 : Prediction )
#define FOR_DETECTION_TEST				2

/// detection =========================================
// detection 파트에서 스레드 사용 여부 결정
// 0 : 사용
// 1 : 사용하지 않음
#define USING_THREAD_IN_DETECTION		0

/// tracking ==========================================
// 스레드 사용 설정
// 0 : 사용함
// 1 : 사용하지 않음
#define USING_THREAD					0

/// prediction ========================================
// display line detection
// 0 : 사용 안 함
// 1 : 사용 함
#define DISPLAY_IMAGE					0
// display simulation
// 0 : 사용 안 함
// 1 : 사용 함
#define DISPLAY_SIMULATION				0

// display head detection
// 0 : 사용 안 함
// 1 : 사용 함
#define DISPLAY_HEAD_WINDOW				0

// display head search region
// 0 : 사용 안 함
// 1 : 사용 함
#define DISPLAY_HEAD_SEARCH_REGION				0

// display optical flow 
// 0 : 일반 optical flow
// 1 : prediction을 위한 optical flow
#define DISPALY_OPTICAL_FLOW_FOR_PREDICTION		1

// tracker의 거리 표시
// 0 : 사용 안 함
// 1 : 사용 함
#define DISPLAY_DISTANCE_OF_TRACKER				0

// tracker의 이동 방향 표시
// 0 : 사용 안 함
// 1 : 사용 함
#define DISPLAY_DIRECTION_OF_TRACKER			1

// DFA mode
// 0 : 예전 DFA
// 1 : 새로운 DFA
// 2 : 전자 공학회 발표 DFA
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
//// 클래스 별 확률값 추정 시, 정규화에 사용되는 상수
//#define RF_DETECTION_NORMALIZE_FACTOR			1.0
//#define RF_DETECTION_CONSTANT					10000000.0


// detection 수행 시 스킵할 프레임 수
#define SKIP_FRAME								1
#define SKIP_FRAME_FOR_OPTICAL					3

// ROI설정을 위한 스킵 프레임 수
#define SKIP_FRAME_FOR_ROI						1

#define PI										3.141592653

// WindowMerging 클래스의 m_fBaseY 배열이 이 변수 크기 만큼 메모리가 할당 됨
// 값을 넣을 때 하드 코딩으로 넣었기 때문에 MAX_LEVEL 값을 변경하면
// 해당 하드 코딩도 주석 처리 해줘야 함
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

// 확률값 정규화 안 한 경우
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
// 영상 크기
#define IMAGE_WIDTH								640
#define IMAGE_HEIGHT							480

// 학습 데이터 저장 배열의 크기
#define MAX_SIZE								1000

// 추적 가능한 휴먼 객체 수
#define TRACKING_NUM							15

// 분류기 클래스
#define PART_CLASSES							2

// 윈도우 내 파트 수
#define SUB_BLOCK								16

// 파티클 수
#define PARTICLE_NUMBER							90


// 사용 채널 수
//#define USING_CHANNEL							1



// 0 : OCS-LBP, LID 4 x 4
// 1 : OCS-LBP + LID 1 x 1
// 2 : OCS-LBP + LID 4 x 4
// 3 : HOG + LID 4 x 4
#define FEATURE_TYPE							0 // 2

#if FEATURE_TYPE == 0
	// 특징 기본 차원 수
	#define FEATURE_DEMENSION						8 // 8
	#define FEATURE_COLOR_DEMENSION					24
	#define COLOR_DISTANCE							8

	// OCS-LBP 특징 차원수
	#define OCSLBP_FEATURE_SIZE						128
	#define INTENSITY_QUANTITY						32

	// Color 특징 차원수
	// 2 채널 사용
	#define LID_FEATURE_SIZE						128
	#define NEW_FEATURE_SIZE						2048

	#define COLOR_FEATURE_SIZE_FOR_TRACKING						384
#elif FEATURE_TYPE == 1
	// 특징 기본 차원 수
	#define FEATURE_DEMENSION						8 // 8
	#define NEW_FEATURE_SIZE						2048
	#define INTENSITY_QUANTITY						1

	// OCS-LBP 특징 차원수
	#define OCSLBP_FEATURE_SIZE						128
	// Color 특징 차원수
	// 2 채널 사용
	#define LID_FEATURE_SIZE						128
#elif FEATURE_TYPE == 2
	// 특징 기본 차원 수
	#define FEATURE_DEMENSION						128 // 8
	#define NEW_FEATURE_SIZE						2048
	#define INTENSITY_QUANTITY						16

	// OCS-LBP 특징 차원수
	#define OCSLBP_FEATURE_SIZE						128
	// Color 특징 차원수
	// 2 채널 사용
	#define LID_FEATURE_SIZE						128
#elif FEATURE_TYPE == 3
	// 특징 기본 차원 수
	#define FEATURE_DEMENSION						144 // 8
	#define NEW_FEATURE_SIZE						2304
	#define INTENSITY_QUANTITY						16

	// OCS-LBP 특징 차원수
	#define OCSLBP_FEATURE_SIZE						128
	// Color 특징 차원수
	// 2 채널 사용
	#define LID_FEATURE_SIZE						128
#endif

// 학습 데이터 추출 시
// 데이터가 추출되는 파트 슬라이딩 영역
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

// OCS-LBP 특징에 사용
// RF에 사용되는 트리수
#define THE_NUMBER_OF_FERN_1					60
// RF에 사용되는 서브셋 비율
#define THE_NUMBER_OF_FUNCTION_1				8
// 최종 사용할 트리 수
#define THE_NUMBER_OF_GOOD_FERN_1				40

// Color 특징에 사용
// RF에 사용되는 트리수
#define THE_NUMBER_OF_FERN_2					60
// RF에 사용되는 서브셋 비율
#define THE_NUMBER_OF_FUNCTION_2				8
// 최종 사용할 트리 수
#define THE_NUMBER_OF_GOOD_FERN_2				40

// 클래스 별 확률값 추정 시, 정규화에 사용되는 상수
#define RF_TRACKING_NORMALIZE_FACTOR			1.0
#define RF_TRACKING_CONSTANT					10000000.0

// histoty length
#define HISTORY_LENGTH							0

// Tracking 정상추적/겹침 결정 RF MAX 확률 (0.7->0.6)
#define TRACKING_PRO_MAX_TH						0.625 //0.625
// Tracking 정상추적/겹침 결정 RF MIN 확률 (0.25)
#define TRACKING_PRO_MIN_TH						0.45  // 0.45

//// 계단식 방법의 임계값 - 특징 추출 단계
//#define CASCADE_TH_1							0.5
//// 계단식 방법의 임계값 - 매칭 단계
//#define CASCADE_TH_2							0.5

// 화면에 추가적으로 출력할 정보
// 0 : 파트 영역 형태
// 1 : 파티클 샘플링 형태
// 2 : 표시하지 않음
#define DISPLAY_TYPE							1 //2

// 윈도우 내 파트 형태
// 0 : 흅먼 형태 type-01
// 1 : 휴먼 형태 type-02
// 2 : 그리드 형태
#define PART_TYPE								2

// 각 함수에서 사용되는 상위 N개의 파티클을 선택하기 위한 비율
#define PARTICLE_RATE							0.15

// 이전 tracker와 현재 tracker 간의 거리 임계값 추정을 위한 비율
#define DIST_RATE								0.5 //0.5

// 파티클 샘플링 범위 비율
#define RANGE_RATE								0.3

// 각 특징의 확률별 가중치
// OCSLBP_WEIGHT + LID_WEIGHT = 1.0
#define OCSLBP_WEIGHT							0.5
#define LID_WEIGHT								0.5

// detection 정보에 대한 가중치
#define DETECTION_WEIGHT						0.5
// 현재 tracker 정보에 대한 가중치
#define CURR_TRACKER_WEIGHT1					0.5
// 현재 tracker 정보에 대한 가중치
#define CURR_TRACKER_WEIGHT2					0.5
// 이전 tracker 정보에 대한 가중치
#define PRE_TRACKER_WEIGHT						0.5


// 다른 tracker와 매칭 거리 임계값
//#define DISTANCE_THRESHOLD						25.0

// 연관성 검사의 score 임계값
#define SCORE_THRESHOLD							0.6  // 0.48
#define PAIR_SCORE_THRESHOLD					0.65


// 매칭 시 사용되는 거리값의 가중치
//#define	DIST_WEIGHT								0.33
// 매칭 시 사용되는 확률값의 가중치
#define	PROB_WEIGHT								0.5
// 매칭 시 사용되는 크기 비율의 가중치
#define	SIZE_WEIGHT								0.5


// tracker 간 겹침을 유무를 체크하는 비율
#define OVERLAP_RATIO_TH						0.15


// 확률값을 사용하지 않는 파트 수
//#define NOT_USING_PARTS							5

// Optical Flow의 Features 포인트의 Grid 간격
#define OPTICALfLOW_GRID_INTERVAL				10

// 회전에 따라 거리 임계값을 달리 주는 방법
// 0 : 사용안함
// 1 : 사용함 
#define USING_THRESHOLDS_ACCORDING_TO_ROTATION		0

// Tracker 확률값과 보행자 이동 방향 표시
// 0 : 표시안함
// 1 : 표시함
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

#define NORM_IMAGE_SIZE		200			//정규화 이미지 크기 (정방형)

#define RF_POINT_REGRESSION_TREE	0	//150825 regression tree 사용
#define RF_CLASSIFICATION_TREE		1	//151029 classification tree 사용

//#define TEST_SAVE_DETECTION	1//test결과 저장 (detection)
#define TEST_SAVE_ALIGNMENT	1//test결과 저장 (alignment)
//#define TEST_SAVE_GAZEESTIMATION	1//test결과 저장 (gaze estimation)

//#define STILL_IMAGE			// 현재 테스트 영상의 스틸영상 유무 (연속적인 프레임이 아닐경우, 전체 shape model에 대한 regression 수행)

#pragma region about_window
//detection window max num
#define MAX_DETECTION_WINDOW			500

#define IMAGE_LEVEL			4		//영상 크기 갯수 : default 7
#if IMAGE_LEVEL == 7
#define IMAGE_SCALE_FACTOR_0	0.63 //영상별 scale 비율 for 7-level
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
#define IMAGE_SCALE_FACTOR_0	0.9 //영상별 scale 비율 for 3-level
#define IMAGE_SCALE_FACTOR_1	1.0 
#define IMAGE_SCALE_FACTOR_2	1.3
#endif

#define NOSE_LEVEL					2
#define NOSE_IMAGE_SCALE_FACTOR_0	0.9
#define NOSE_IMAGE_SCALE_FACTOR_1	1.0
#define NOSE_IMAGE_SCALE_FACTOR_2	1.1


//search window 이동 간격
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


// OCS-LBP 관련 
#define OCSLBP_ACCUM_TYPE		OCSLBP_ACCUM_DIST
#define OCSHISTOGRAM_BINS		8	//ocs-lbp bin 갯수 
#define OCSLBP_THRESHOLD		6	//방향 무시 픽셀값 임계치

#define LBP_BLK_WIDTH_F_WIN			8	//LBP 추출할 블록 가로 갯수 (윈도우 별)
#define LBP_BLK_HEIGHT_F_WIN		LBP_BLK_WIDTH_F_WIN	//LBP 추출할 블록 세로 갯수 (윈도우 별)
#define LBP_NUM_OF_PIX_WID_F_WIN	WINDOW_SIZE_X_0/LBP_BLK_WIDTH_F_WIN			//LBP 블록당 가로 픽셀수 (윈도우 별)
#define LBP_NUM_OF_PIX_HEI_F_WIN	WINDOW_SIZE_Y_0/LBP_BLK_WIDTH_F_WIN			//LBP 블록당 세로 픽셀수 (윈도우 별)
#define OCSLBP_FEATURE_SIZE_WIN		OCSHISTOGRAM_BINS*LBP_BLK_WIDTH_F_WIN*LBP_BLK_HEIGHT_F_WIN

#define LBP_BLK_WIDTH_F_WIN_NOSE			4								//LBP 추출할 블록 가로 갯수 (윈도우 별)
#define LBP_BLK_HEIGHT_F_WIN_NOSE			LBP_BLK_WIDTH_F_WIN_NOSE		//LBP 추출할 블록 세로 갯수 (윈도우 별)
#define LBP_NUM_OF_PIX_WID_F_WIN_NOSE		WINDOW_SIZE_X_NOSE/LBP_BLK_WIDTH_F_WIN_NOSE			//LBP 블록당 가로 픽셀수 (윈도우 별)
#define LBP_NUM_OF_PIX_HEI_F_WIN_NOSE		WINDOW_SIZE_Y_NOSE/LBP_BLK_WIDTH_F_WIN_NOSE			//LBP 블록당 세로 픽셀수 (윈도우 별)
#define OCSLBP_FEATURE_SIZE_WIN_NOSE		OCSHISTOGRAM_BINS*LBP_BLK_WIDTH_F_WIN_NOSE*LBP_BLK_HEIGHT_F_WIN_NOSE

// Haar-Like 관련
#define HAARLIKE_BLK_WIDTH_F_WIN			8	//Haar-like 추출할 블록 가로 갯수 (윈도우 별)
#define HAARLIKE_BLK_HEIGHT_F_WIN			HAARLIKE_BLK_WIDTH_F_WIN	//Haar-like 추출할 블록 세로 갯수 (윈도우 별)
#define HAARLIKE_WINDOW_FEATURE_SIZE		61	//Haarlike 특징 크기(패턴수)

#define HAARLIKE_BLK_WIDTH_F_WIN_NOSE		8				//Haar-like 추출할 블록 가로 갯수 (윈도우 별)
#define HAARLIKE_BLK_HEIGHT_F_WIN_NOSE		HAARLIKE_BLK_WIDTH_F_WIN_NOSE	//Haar-like 추출할 블록 세로 갯수 (윈도우 별)
#define HAARLIKE_WINDOW_FEATURE_SIZE_NOSE	61				//Haarlike 특징 크기(패턴수)

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
// 임계값
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

#define DETECTION_FRAME_PERIOD		12		//detection 수행 간격 default :12->10 2016-10-26 수정

#define DRAW	true
#define NO_DRAW	false

#define TIME_DAY_DATA	0		// 주간 영상
#define TIME_NIGHT_DATA	1		// 야간 영상



//#define DETECTION_WINDOW_EXPAND_MARGIN_FACE		20	//detection 결과 window 확장 pixel 수
//#define DETECTION_WINDOW_EXPAND_MARGIN_NOSE		10	


//#define USE_PARALLEL_PROCESSING		//151001 병렬 처리 플래그 추가


///=============================================================================
///=================================Pairing Part================================
///=============================================================================

// Association Check할때 score의 Threshold
#define PAIR_SCORE_THRESHOLD					0.6

/*
ROI를 5등분 할 때 y축 비율(?)
FIRST부터 Y축의 위쪽. 즉 화면 위
*/
#define PAIR_ROI_CROP_FIRST						2
#define PAIR_ROI_CROP_SECOND					4
#define PAIR_ROI_CROP_THIRD						6
#define PAIR_ROI_CROP_FOURTH					8
#define PAIR_ROI_CROP_OVERLAP_RATIO				0.3

/*
ROI 내에서 두 Blob간의 내부거리(Inner_Width) Threshold
FIRST부터 Y축의 위쪽. 즉 화면 위
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
ROI 내에서 두 Blob간의 내부거리(Inner_Width) Threshold
Taillight일 경우에
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
HIGH-BEAM과 LOW-BEAM 투표를 몇 FRAME 체크할건지
HIGH절반 LOW 절반으로 20FRAME 기준으로 이전 20FRAME이 모두 HIGH였을 경우 10FRAME 동안 LOW로 측정 한 뒤에 LOW로 바뀐다.
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

// 표준편차 구하는 함수
double CalculateStandardDeviation_FOR_BYTE(CDib& dib, double meanValue, int width, int height);
void CalculateStandardDeviation_FOR_RGBBYTE(CDib& dib, double* meanValue, int width, int height, double* std);

// 평균 구하는 함수
double CalculateMeanValue_FOR_BYTE(CDib& dib, int width, int height);
void CalculateMeanValue_FOR_RGBBYTE(CDib& dib, int width, int height, double* mean);
