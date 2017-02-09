#include "StdAfx.h"
#include "OpticalFlow.h"

#define _CRTDBG_MAP_ALLOC

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif 



COpticalFlow::COpticalFlow()
{	
	m_prevImg = NULL;
	m_currImg = NULL;
	m_testImg = NULL;

	m_nValidp_count = 0;
	m_dResizeRatio = 1.0;
}

COpticalFlow::~COpticalFlow(void)
{	
}

/**
	@brief		opticalflow클래스 관련 초기값 설정
	@param		h : orignal img height (optical flow에서는 원본 영상의 1/2 resizing영상 사용)
				w : orignal img width
				gridInterval : optical 추출을 위한 특징점 설정을 위한 grid 간격
				rateROiH : optical flow추출 ROI영역 설정을 위한 변수(영상 상단을 기준으로 영상 전체의 해당 rate비율 만큼  ROI y좌표 설정함)
*/
void COpticalFlow::Func_Init(int h, int w, int gridInterval, double imgReseRatio, float rateRoiH)
{	
	int imgH, imgW;	
	int gridCnt;
	
	// 현재 영상 resizing 비율
	m_dResizeRatio = imgReseRatio;

	// 영상 관련 설정
	imgH = h * m_dResizeRatio;
	imgW = w * m_dResizeRatio;
	m_nImgHeight = imgH;
	m_nImgWidth = imgW;
	
	if( m_prevImg != NULL )			cvReleaseImage( &m_prevImg );	
	if( m_currImg != NULL )			cvReleaseImage( &m_currImg );
	if( m_testImg != NULL )			cvReleaseImage( &m_testImg );		

	m_prevImg = cvCreateImage(cvSize(imgW, imgH), IPL_DEPTH_8U, 1);
	m_currImg = cvCreateImage(cvSize(imgW, imgH), IPL_DEPTH_8U, 1);
	m_testImg = cvCreateImage(cvSize(imgW, imgH), IPL_DEPTH_8U, 3);
	
	cvSetZero(m_prevImg);
	cvSetZero(m_currImg);
	cvSetZero(m_testImg);	

	if( rateRoiH == -1.0 )	m_nROIHeight = m_nImgHeight;
	else					m_nROIHeight = m_nImgHeight * rateRoiH;	

	// grid 관련 설정
	m_nGridInterval = gridInterval;	
	int nGridInterval_H = m_nROIHeight / gridInterval;
	int nGridInterval_W = m_nImgWidth / gridInterval;
	gridCnt = (nGridInterval_H + 1)*(nGridInterval_W + 1);

	// 움직임 추정 Point 배열 
	m_prev_gridPoint = new CvPoint2D32f[gridCnt];			// 움직임을 추정할 Point 배열
	m_curr_gridPoint = new CvPoint2D32f[gridCnt];			// prev_gridPoint에 대한 결과 Point 배열

	m_op = new OPTICALFLOW_FEATURES[9999];
	for (int i = 0; i < 9999; i++)
	{
		m_op[i].cx = 0;
		m_op[i].cy = 0;
		m_op[i].px = 0;
		m_op[i].py = 0;
		m_op[i].angle = 0;
		m_op[i].magnitude = 0;
	}

	m_bSetPrevImg = false;	

	m_nCurrRoiStart_x = m_nPrevRoiStart_x = w * ROI_START_RATIO_X;
	m_nCurrRoiStart_y = m_nPrevRoiStart_y = h * ROI_START_RATIO_Y;
	m_nNonMovingCount = 0;
	
}

void COpticalFlow::Func_Destory()
{
	if( m_prevImg != NULL )		cvReleaseImage( &m_prevImg );	
	if( m_currImg != NULL )		cvReleaseImage( &m_currImg );
	if( m_testImg != NULL )		cvReleaseImage( &m_testImg );
		
	m_bSetPrevImg = false;

	m_prevImg = NULL;
	m_currImg = NULL;
	m_testImg = NULL;

	delete [] m_prev_gridPoint;
	delete [] m_curr_gridPoint;

	delete [] m_op;
}


/**
	@brief			이전 영상 전역 변수로 설정 함수
*/
void COpticalFlow::SetPreviousImg(IplImage* img)
{	
	if( img->nChannels == 1 )
		cvResize(img, m_prevImg);	//cvCopy(img, cpyimg);
	else
	{
		IplImage* cpyimg = cvCreateImage(cvSize(img->width, img->height), 8, 1);
		cvCvtColor(img, cpyimg, CV_BGR2GRAY);
		cvResize(cpyimg, m_prevImg);		// 1/2사이즈로 축소
		cvReleaseImage(&cpyimg);
	}	

	m_bSetPrevImg = true;	
}

/**
	@brief			현재 영상 전역 변수로 설정 함수
*/
void COpticalFlow::SetCurrentImg(IplImage* img)
{	
	if( img->nChannels == 1 )
		cvResize(img, m_currImg);	//cvCopy(img, cpyimg);
	else
	{
		IplImage* cpyimg = cvCreateImage(cvSize(img->width, img->height), 8, 1);
		cvCvtColor(img, cpyimg, CV_BGR2GRAY);
		cvResize(cpyimg, m_currImg);		// 1/2사이즈로 축소
		cvReleaseImage(&cpyimg);
	}		
}

/**
	@brief		Optical Flow를 위한 메인 함수(For ROI Setting)
	@param		currImg : 입력 IplImage 영상	
	@param		dir : 현재 자동차 대표 움직임 방향
	@param		mag : 현재 자동차 대표 움직임 크기
	@return		정상종료(0), 비정상종료(-1)
*/
int COpticalFlow::mainProcessing(IplImage* currImg, int& dir, double& mag, int nCurrentFrameNum)
{
	if( !m_bSetPrevImg )		
	{
		// m_prevImg 이전 영상이 설정 되지 않았을 경우 현재 영상을 이전 영상으로 대입
		SetPreviousImg(currImg);
		return -1;
	}	

	// 사용자 설정 grid 개수
	int gridCount = 0;					
	
	// 현재 입력 영상 IplImage 형 변환 및 사이즈 변환
	SetCurrentImg(currImg);

	// dense optical flow 추출을 위한 gird 설정		
	gridCount = SetGridPoints(m_prevImg, m_currImg, m_prev_gridPoint, m_curr_gridPoint, m_nGridInterval);

	///////////////////////////////
	// optical flow 추출
	///////////////////////////////
	// 움직임 추정 성공 여부 저장 배열
	char* status = new char[gridCount];
	float* track_error = new float[gridCount];	

	GetOpticalFlow(m_prevImg, m_currImg, status, track_error, gridCount, m_prev_gridPoint, m_curr_gridPoint);
	
	// optical flow 결과에서 유효포인트 전역 변수 저장	
	m_nValidp_count = SetOPValue(status, track_error, gridCount, m_prev_gridPoint, m_curr_gridPoint);	
	
	// 2016-12-23 
	if( m_nValidp_count > 0 )
	{
		// 자동차 평균 움직임 추출		
		GetAvgOPMagAng(m_dVehicleMagnitudeResult, m_dVehicleAngleResult);	//extract_vehicle_direct_new(m_nVehicleDirectResult, m_dVehicleMagnitudeResult, m_dVehicleAngleResult);		
	}else
	{
		m_dVehicleMagnitudeResult = 0; 
		m_dVehicleAngleResult = 0.0;
	}

	// optical flow를 이용한 ROI 시작 좌표 전역 변수로 설정
	SetROIvalues(currImg->height, currImg->width);

	// 결과 확인 
	//print_OpticalFlow_result(nCurrentFrameNum);		// optical flow
	//print_ROI(m_testImg, "roi1");					// 축소크기 영상에 ROI 출력

	//print_ROI(m_currImg, "roi2");					// 원본크기 영상에 ROI 출력
	// 현재 영상 이번 영상으로 복사
	cvCopy(m_currImg, m_prevImg);
			
	delete [] status;
	delete [] track_error;		
	
	return 0;
}

/*int COpticalFlow::mainProcessingForDenseFlow(IplImage* currImg, bool& bUpdateMask, int nCurrentFrameNum)
{
	int nVehicleDirectResult = 0;		// 0:직진, 1:좌회전, 2:우회전	

	if( !m_bSetPrevImg )		// m_prevImg 이전 영상이 설정 되지 않았을 경우 현재 영상을 이전 영상으로 대입
	{
		//BYTE2IplImage_Gray(currImg_ptr, m_prevImg);
		cvCvtColor(currImg, m_prevImg, CV_BGR2GRAY);
		m_bSetPrevImg = true;
		return -1;
	}	

	// 현재 입력 영상 IplImage 형 변환
	if( currImg->nChannels != 1 )	
		cvCvtColor(currImg, m_currImg, CV_BGR2GRAY); //BYTE2IplImage_Gray(currImg_ptr, m_currImg);	
		

	CvMat* flow = cvCreateMat(m_currImg->height, m_currImg->width, CV_32FC2);
	CvMat* cflow = cvCreateMat(m_currImg->height, m_currImg->width, CV_8UC3);
	//IplImage *flow = cvCreateImage(cvSize(m_currImg->width, m_currImg->height), IPL_DEPTH_32F, 2);	
	//cvCalcOpticalFlowFarneback(m_prevImg, m_currImg, flow, 0.5, 2, 5, 2, 7, 1.5, cv::OPTFLOW_FARNEBACK_GAUSSIAN);
		
	cvCalcOpticalFlowFarneback(m_prevImg, m_currImg, flow, 0.5, 3, 15, 3, 5, 1.2, 0);
	cvCvtColor(m_prevImg, cflow, CV_GRAY2BGR);
	
    drawOptFlowMap(flow, cflow, 16, 1.5, CV_RGB(0, 255, 0));

	cvShowImage("prev", m_prevImg);
	cvShowImage("curr", m_currImg);
	cvShowImage("flow", cflow);

	// 현재 영상 이번 영상으로 복사
	cvCopy(m_currImg, m_prevImg);

}

void COpticalFlow::drawOptFlowMap(const CvMat* flow, CvMat* cflowmap, int step, double scale, CvScalar color)
{
    int x, y;
    (void)scale;
    for( y = 0; y < cflowmap->rows; y += step)
        for( x = 0; x < cflowmap->cols; x += step)
        {
            CvPoint2D32f fxy = CV_MAT_ELEM(*flow, CvPoint2D32f, y, x);

			//TRACE(_T("(%f, %f)\n"), fxy.x, fxy.y);
			//if( fxy.x > 0.2 || fxy.y > 0.2 )			
			{
            cvLine(cflowmap, cvPoint(x,y), cvPoint(cvRound(x+fxy.x), cvRound(y+fxy.y)),
                 color, 1, 8, 0);
            cvCircle(cflowmap, cvPoint(x,y), 2, color, -1, 8, 0);
			}
        }
}*/

/**
	@brief		Optical Flow 결과 영상 출력 함수
*/
int COpticalFlow::print_OpticalFlow_result(int nCurrentFrameNum)
{	
	//OpenCV 화면에 글자쓰기
    char s_output_result[50];
    CvFont font;
	int valid_pt = 0;  //valid한 점의 개수 count	
	double avg_magnitude = 0.0;			// 움직임 크기 평균 
	int nMax_avgMagnitude = 0;			// 3프레임에서 최대 평균 움직임 크기 저장 변수

	if( m_currImg->nChannels == 1 )
		cvCvtColor(m_currImg, m_testImg, CV_GRAY2RGB);
	else
		cvCopy(m_currImg, m_testImg);
	
	int nResizeLine = 2;		// 화살표의 크기를 실제 몇배로..

	//draw arrow of optical flow point (화살표 모양으로 그리고 실제 움직인 거리의 2배 길이로 화살표 그려줌)
	for(int i=0 ; i<m_nValidp_count ; i++ )
    { 
        int line_thickness = 1;
        CvScalar line_color = CV_RGB(0,255,0);	
		CvPoint prep;
		CvPoint curp;
		double arrow_length = 0.0; 
		double radian;

		prep.x = m_op[i].px;
		prep.y = m_op[i].py;
		arrow_length = m_op[i].magnitude;
		radian = m_op[i].angle; //radian = m_op[i].angle * (PI/180.);

		//2배 길이로 화살표 그려줌
		curp.x = (int)(prep.x - nResizeLine * arrow_length * cos(radian));
		curp.y = (int)(prep.y - nResizeLine * arrow_length * sin(radian));
				
		if( arrow_length > 50 )		line_color = CV_RGB(100,100,100);		// gray
		else						line_color = CV_RGB(0,255,0);			// green

        //draw arrow
        cvLine( m_testImg, prep, curp, line_color, line_thickness, CV_AA, 0 );
        prep.x = (int) (curp.x + 5 * cos(radian + PI / 4));
		if(prep.x>=m_nImgWidth)           prep.x=m_nImgWidth-1;
        else if(prep.x<0)		          prep.x=0;
        prep.y = (int) (curp.y + 5 * sin(radian + PI / 4));
        if(prep.y>=m_nImgHeight)           prep.y=m_nImgHeight-1;
        else if(prep.y<0)		           prep.y=0;
        cvLine( m_testImg, prep, curp, line_color, line_thickness, CV_AA, 0 );
        prep.x = (int) (curp.x + 5 * cos(radian - PI / 4));
        if(prep.x>=m_nImgWidth)            prep.x=m_nImgWidth-1;
        else if(prep.x<0)		           prep.x=0;
        prep.y = (int) (curp.y + 5 * sin(radian - PI / 4));
        if(prep.y>m_nImgHeight)            prep.y=m_nImgHeight-1;
        else if(prep.y<0)		           prep.y=0;
        cvLine( m_testImg, prep, curp, line_color, line_thickness, CV_AA, 0 );
    }   
	
	// draw ROI Line
	cvLine(m_testImg, cvPoint(0, m_nROIHeight), cvPoint(m_testImg->width, m_nROIHeight), cvScalar(255,187,0), 2);
	/*
	int angle180 = 0.0;
	if( m_dVehicleAngleResult == 0.0 ) 
		angle180 = 0.0;
	else
		angle180 = (int)(m_dVehicleAngleResult * 180.0 / PI2);// ~180~180

	// draw vehicle direction & magnitude
	if( m_nVehicleDirectResult == 0 )
		sprintf_s(s_output_result, sizeof(s_output_result), "GO STRAIGHT-%.1f/%d", m_dVehicleMagnitudeResult, angle180);    //우선 sprintf로 문자열 생성
	else if( m_nVehicleDirectResult == 1 )
		sprintf_s(s_output_result, sizeof(s_output_result), "LEFT TURN-%.1f/%d", m_dVehicleMagnitudeResult, angle180);    //우선 sprintf로 문자열 생성
	else 
		sprintf_s(s_output_result, sizeof(s_output_result), "RIGHT TURN-%.1f/%d", m_dVehicleMagnitudeResult, angle180);    //우선 sprintf로 문자열 생성
	
	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, 0.7, 0.7, 0, 1); 
	cvPutText(m_testImg, s_output_result ,cvPoint(15,m_nImgHeight-15),&font,cvScalar(255,187,0));   //cvPoint로 글자 시작 위치 설정(uv)
	
	cvNamedWindow("Lkpyr_OpticalFlow", CV_WINDOW_AUTOSIZE); 
    cvShowImage("Lkpyr_OpticalFlow", m_testImg); 	
	*/

	return 0;
}

/**
	@brief		베이시안 분류기를 이용한 Optical Flow를 통한 자동차 진행 방향 추출 및 결과 저장	
	@date		2016-12-19
	@param		result_direct : 최종 자동차 진행 방향
	@param		avg_magnitude : 최종 자동차 움직임 평균 크기
	@return		정상종료 (1:정상종료)
*/
int COpticalFlow::extract_vehicle_direct_new(int& result_direct, double& avg_magnitude, double& result_angle)
{	
	double angle = 0.0;
	double arrow_length = 0.0; 
	int valid_pt = 0;				// valid한 점의 개수 count
	int count =0;
	
	// for reference Line
	double avg_mag = 0.0;			// 움직임 크기 평균 
	double avg_magnitude_forDataExpection = 0.0;		// 현재 영상의 움직임 평균 크기 추출 (이에 따라 max 제외 움직임 설정)
	double avg_ang = 0.0;

	// 베이시안 분류기를 이용한 주행 방향 결정 함수 관련 변수
	/*const double dMeanX[3] = {-0.34, -9.14, 5.12};		// {go, left, right} 벡터 x에 대한 평균	
	const double dMeanY[3] = {-0.31, -0.33, -0.25};			// {go, left, right} 벡터 y에 대한 평균	
	const double dVarianceX[3] = {10.26, 19.92, 5.12};		// {go, left, right} 벡터 x에 대한 분산
	const double dVarianceY[3] = {2.30, 3.84, 2.52};	    // {go, left, right} 벡터 y에 대한 분산	*/
	/*const double dMeanX[3] = {-0.59, -6.74, 4.46};			// {go, left, right} 벡터 x에 대한 평균	
	const double dMeanY[3] = {-0.40, -0.27, 0.49};			// {go, left, right} 벡터 y에 대한 평균	
	const double dVarianceX[3] = {16.18, 30.30, 23.98};	// {go, left, right} 벡터 x에 대한 분산
	const double dVarianceY[3] = {2.29, 2.51, 0.56};	    // {go, left, right} 벡터 y에 대한 분산	*/
	
	const double dMeanX[3] = {-0.57, -6.88, 7.01};			// {go, left, right} 벡터 x에 대한 평균	
	const double dMeanY[3] = {-0.39, -0.28, -0.18};			// {go, left, right} 벡터 y에 대한 평균	
	const double dVarianceX[3] = {16.05, 28.35, 28.00};		// {go, left, right} 벡터 x에 대한 분산
	const double dVarianceY[3] = {2.28, 2.36, 2.30};	    // {go, left, right} 벡터 y에 대한 분산
	
	double dClassification_FuncG[3] = {0.0, 0.0, 0.0};		// 최종 분류 함수 결과
	
	double dir_sumProb[3] = {0.0};		//  현재 프레임에서의 각 특징벡터에서의 방향별 확률값 누적 
	double dMax_value = 0.0;
	int nMax_dir = 0.0;
	int current_direct = 0;			// 현재 프레임의 결정 방향 
	//int result_direct = 0;			// 최종 결정 방향
	int count_direct[3] = {0,0,0};		// 이번 2프레임을 고려한 주행 방향 카운트
	double dValue_x, dValue_y;		// 현재 입력 x, y벡터 

	// 상수 사전에 계산
	double temp1[3] = {0};			// 1/variance_x
	double temp2[3] = {0};			// mean_x^2
	double temp3[3] = {0};			// 1/variance_y
	double temp4[3] = {0};			// mean_y^2

	for(int t=0 ; t<3 ; t++)
	{
		temp1[t] = 1.0/dVarianceX[t];
		temp2[t] = dMeanX[t]*dMeanX[t];
		temp3[t] = 1.0/dVarianceY[t];
		temp4[t] = dMeanY[t]*dMeanY[t];

	}	

	// 현재 영상의 움직임 평균 크기 추출 -> 이를 통하여 움직임 벡터 전체의 Max설정
	for(int i=0; i<m_nValidp_count; ++i) 
    {
		dValue_x = (int)m_op[i].px - (int)m_op[i].cx;
		dValue_y = (int)m_op[i].py - (int)m_op[i].cy;
		arrow_length = sqrt( pow(dValue_y,2) + pow(dValue_x,2) );			
		avg_magnitude_forDataExpection += arrow_length;		
	}
	avg_magnitude_forDataExpection = avg_magnitude_forDataExpection / m_nValidp_count;

	// optical flow 움직임 벡터 접근
	count = 0;
	for(int i=0; i<m_nValidp_count; ++i) 		
    {
		// 움직임 벡터 추출
		dValue_x = m_op[i].px - m_op[i].cx;
		dValue_y = m_op[i].py - m_op[i].cy;

		if( m_op[i].magnitude > avg_magnitude_forDataExpection+10 )
			continue;

		// 원본 영상사이즈와 op 추출 영상 사이즈를 고려하여 optical flow magnitude 계산 		
		dValue_x /= m_dResizeRatio;		
		dValue_y /= m_dResizeRatio;

		if( dValue_x > 15.0 ) dValue_x = 15.0;
		if( dValue_x < -15.0 ) dValue_x = -15.0;
		if( dValue_y > 15.0 ) dValue_y = 15.0;
		if( dValue_y < -15.0 ) dValue_y = -15.0;
	
		// conditional Probability (P_go, P_left, P_right)		
		dClassification_FuncG[0]	= exp( -0.5 * ( temp1[0]*( dValue_x*dValue_x-2.0*dMeanX[0]*dValue_x+temp2[0] ) + temp3[0]*( dValue_y*dValue_y-2.0*dMeanY[0]*dValue_y+temp4[0] ) ) );
		dClassification_FuncG[1]	= exp( -0.5 * ( temp1[1]*( dValue_x*dValue_x-2.0*dMeanX[1]*dValue_x+temp2[1] ) + temp3[1]*( dValue_y*dValue_y-2.0*dMeanY[1]*dValue_y+temp4[1] ) ) );
		dClassification_FuncG[2]	= exp( -0.5 * ( temp1[2]*( dValue_x*dValue_x-2.0*dMeanX[2]*dValue_x+temp2[2] ) + temp3[2]*( dValue_y*dValue_y-2.0*dMeanY[2]*dValue_y+temp4[2] ) ) );
		
		// 영상 전체의 방향별 움직임 확률값 누적
		dir_sumProb[0] += dClassification_FuncG[0];
		dir_sumProb[1] += dClassification_FuncG[1];
		dir_sumProb[2] += dClassification_FuncG[2];

		avg_mag += m_op[i].magnitude;	
		avg_ang += m_op[i].angle;
		count++	;
    }

	// 현재 영상의 평균 움직임 크기 추출
	avg_magnitude = (avg_mag / count);
	result_angle = (avg_ang / count);

	// 누적된 방향 개수를 이용한 최종 방향 결정
	dMax_value = dir_sumProb[0];
	current_direct = 0;
	for(int t=1 ; t<3 ; t++)
	{
		if( dMax_value < dir_sumProb[t] )
		{
			dMax_value = dir_sumProb[t];
			current_direct = t;
		} 
	}
	result_direct = current_direct;

	/*
	if( valid_pt < 100 && dMax_value/(double)valid_pt < 0.3 )			// max 확률값이 0.3 이하면 이전 프레임에서 추출된 최종 주행 방향으로 대체
	{
		current_direct = m_nPrevDirect[1];		// 이전 프레임에서 여러 프레임을 고려한 추출 방향
	}

	if( current_direct == 0 ) Trace(_T("==== current dir : Go\n"));
	else if( current_direct == 1 ) Trace(_T("==== current dir : Left\n"));
	else if( current_direct == 2 ) Trace(_T("==== current dir : Right\n"));
	
	//result_direct = current_direct;
	count_direct[current_direct]++;
	count_direct[m_nPrevDirect[0]]++;
	count_direct[m_nPrevDirect[1]]++;		
		
	
	if( count_direct[0] >= 2 )		// 두프레임전의 방향과 현재 방향, 세개를 고려해서 두개이상 카운트된 방향을 현재 결과 방향으로 결정
		result_direct = GO_STRAIGHT;
	else if( count_direct[1] >= 2 )	
		result_direct = LEFT_TURN;
	else
		result_direct = RIGHT_TURN;
	
	//Trace(_T("Dir=(r:%2d, c:%2d)-count:%d\tgo=%lf, left=%lf, right=%lf, avg=%lf\n"), result_direct, current_direct, valid_pt, dir_sumProb[0]/(double)valid_pt, dir_sumProb[1]/(double)valid_pt, dir_sumProb[2]/(double)valid_pt, avg_magnitude);

	m_nPrevDirect[0] = m_nPrevDirect[1];
	m_nPrevDirect[1] = current_direct;

	//bUpdateMask = set_ReferenceLine_processing(avg_magnitude, result_direct);	
	
	if( m_bFirstFrame ) 
	{
		//bUpdateMask = true;
		m_bFirstFrame = false;
	}*/
	
	return 1;
}

/**
	@brief		op의 manitude가 평균에서 많이 벗어나는 point를 제외한 op의 평균 angle, manitude 추출	
	@param		avg_magnitude : 자동차 움직임 평균 크기
	@param		result_angle : 자동차 움직임 각도(radius)	
*/
void COpticalFlow::GetAvgOPMagAng(double& avg_magnitude, double& result_angle)
{
	double arrow_length = 0.0; 
	double avg_mag = 0.0;			// 움직임 크기 평균 
	double avg_magnitude_forDataExpection = 0.0;		// 현재 영상의 움직임 평균 크기 추출 (이에 따라 max 제외 움직임 설정)
	double avg_ang = 0.0;
	int count = 0;
	double dValue_x, dValue_y;		// 현재 입력 x, y벡터 

	// 현재 영상의 움직임 평균 크기 추출 -> 이를 통하여 움직임 벡터 전체의 Max설정
	for(int i=0; i<m_nValidp_count; ++i) 
    {
		dValue_x = (int)m_op[i].px - (int)m_op[i].cx;
		dValue_y = (int)m_op[i].py - (int)m_op[i].cy;
		arrow_length = sqrt( pow(dValue_y,2) + pow(dValue_x,2) );			
		avg_magnitude_forDataExpection += arrow_length;		
	}
	avg_magnitude_forDataExpection = avg_magnitude_forDataExpection / m_nValidp_count;

	// optical flow 움직임 벡터 접근
	count = 0;
	for(int i=0; i<m_nValidp_count; ++i) 		
    {
		if( m_op[i].magnitude > avg_magnitude_forDataExpection+10 )
			continue;

		avg_mag += m_op[i].magnitude;	
		avg_ang += m_op[i].angle;
		count++	;
    }

	// 현재 영상의 평균 움직임 크기 추출
	avg_magnitude = (avg_mag / count);
	result_angle = (avg_ang / count);
}

/**
	@brief		전역변수 m_op변수에 현재 optical flow 결과 저장
	@return		유효한 optical flow point 개수
*/
int COpticalFlow::SetOPValue(char* status, float* track_error, int nFeatureSize, CvPoint2D32f* prev_features, CvPoint2D32f* curr_features)
{
	int valid_pt = 0;  //valid한 점의 개수 count
	double angle = 0.0;
	double arrow_length = 0.0; 
	double avg_magnitude = 0.0;			// 움직임 크기 평균 
	double dValue_x, dValue_y;

	for( int i=0 ; i<nFeatureSize ; i++ )
	{
		if(status[i] != 0)
		{
			dValue_x = (int)prev_features[i].x - (int)curr_features[i].x;
			dValue_y = (int)prev_features[i].y - (int)curr_features[i].y;
		    			
			//angle = atan2( dValue_y, dValue_x ) * 180.0 / PI;		// -180도 ~ 180도
			//if( angle < 0 )		angle += 360;					// 0도 ~ 360도 변환
			angle = atan2( dValue_y, dValue_x );
			arrow_length = sqrt( pow(dValue_y,2) + pow(dValue_x,2) );

			m_op[valid_pt].angle = angle;
			m_op[valid_pt].magnitude = arrow_length;
			m_op[valid_pt].cx = (int)(curr_features[i].x);
			m_op[valid_pt].cy = (int)(curr_features[i].y);
			m_op[valid_pt].px = (int)(prev_features[i].x);
			m_op[valid_pt].py = (int)(prev_features[i].y);

			valid_pt++;
		}
	}

	//m_nValidp_count = valid_pt;
	return valid_pt;
}

/**
	@brief		현재 추출된 optical flow정보를 기반으로 해서 roi영역 추출
	@date		2016-12-19
	@param		imgH, imgW : 이미지 사이즈
				roi_start_x, roi_start_y : roi시작 좌표 결과 저장 변수
*/
void COpticalFlow::SetROIvalues(int imgH, int imgW)
{	
	int roi_default_start_x = imgW * ROI_START_RATIO_X;
	int roi_default_start_y = imgH * ROI_START_RATIO_Y;
	int roi_w = imgW * ROI_WIDTH_RATIO;
	int roi_h = imgH * ROI_HEIGHT_RATIO;	
	int op_mag = m_dVehicleMagnitudeResult / m_dResizeRatio;	// 원본 영상사이즈와 op 추출 영상 사이즈를 고려하여 optical flow magnitude 계산 
	double angle180  = 0.0;	// ~180~180
	int roi_start_x, roi_start_y;

	m_nPrevRoiStart_x = m_nCurrRoiStart_x;
	m_nPrevRoiStart_y = m_nCurrRoiStart_y;

	// 직진 : small moving size || 좌우 움직임 : 기본 roi 좌표 사용
    //        large moving size && 상하 움직임 : 기본 roi 좌표에서 op 평균 크기만큼 이동	                                  
	angle180 = m_dVehicleAngleResult * 180.0 / PI2;		// radian -> ~180~180 변환
	if( m_dVehicleMagnitudeResult > 3. && ( abs(angle180) >= 55.0 && abs(angle180) <= 115.0 ) )	// large moving
	{
		// 기본좌표에서 현재 각도, 크기 만큼 이동								
		//roi_start_x = m_nPrevRoiStart_x - op_mag*cos(m_dVehicleAngleResult);		// 2016-12-29 수평 ROI만 사용
		roi_start_x = m_nPrevRoiStart_x;
		roi_start_y = m_nPrevRoiStart_y - op_mag*sin(m_dVehicleAngleResult); 
		m_nNonMovingCount = 0;
			
	}else                                   // small moving
	{
		// if the small moving occurs continuously, the current roi uses the defaualt roi. 
		if( m_nNonMovingCount >= 3 ) 
		{
			// use the default roi
			roi_start_x = roi_default_start_x;
			roi_start_y = roi_default_start_y;	
			m_nNonMovingCount = 0;
		}else
		{
			// use previous roi 
			roi_start_x = m_nPrevRoiStart_x; 
			roi_start_y = m_nPrevRoiStart_y; 
		}
		m_nNonMovingCount++;
	}	
	
	m_nCurrRoiStart_x = roi_start_x;
	m_nCurrRoiStart_y = roi_start_y;	

	// 단순하게 현재 optical flow 평균 추출 방향과 크기에 따라서 ROI 설정
	// 우회전일경우 : optical flow 평균 움직임 크기가 4이상이고 각도가 |135~180|
    // 좌회전일경우 : optical flow 평균 움직임 크기가 4이상이고 각도가 |0~45|
	// 각도를 고려한 좌&우회전 추출
	/*double angle180 = m_dVehicleAngleResult * 180.0 / PI2;// ~180~180
	if( abs(angle180) >= 135.0 && abs(angle180) <= 180.0 && m_dVehicleMagnitudeResult > 2. )		// 우회전 -> ROI 오른쪽으로 이동
	{
		roi_start_x = roi_default_start_x + roi_w * 0.07;
		roi_start_y = roi_default_start_y;
	}
	else if( abs(angle180) >= 0 && abs(angle180) <= 45.0 && m_dVehicleMagnitudeResult > 2.  )		// 좌회전 -> ROI 왼쪽으로 이동
	{
		roi_start_x = roi_default_start_x - roi_w * 0.07;	
		roi_start_y = roi_default_start_y;
	}*/
	// 베이지안분류기를 이용한 좌&우회전 추출
	/*if( m_nVehicleDirectResult == 2 )				// 우회전 -> ROI 오른쪽으로 이동
	{
		roi_start_x = roi_default_start_x + roi_w * 0.07;
		roi_start_y = roi_default_start_y;
	}
	else if( m_nVehicleDirectResult == 1 )			// 좌회전 -> ROI 왼쪽으로 이동
	{
		roi_start_x = roi_default_start_x - roi_w * 0.07;	
		roi_start_y = roi_default_start_y;
	}else                                           // 직진
	{
        if( m_dVehicleMagnitudeResult > 3. )	// large moving
		{
			// 기본좌표에서 현재 각도, 크기 만큼 이동								
			roi_start_x = roi_default_start_x - op_mag*cos(m_dVehicleAngleResult);
			roi_start_y = roi_default_start_y - op_mag*sin(m_dVehicleAngleResult); 
			
		}else                                   // small moving
		{
			// setting default roi 
			roi_start_x = roi_default_start_x;
			roi_start_y = roi_default_start_y;
		}					
	}	*/
}

/**
	@brief		ROI 영역 이미지 출력
*/
void COpticalFlow::print_ROI(IplImage* img, const char* show_name)
{
	int roi_width = img->width * ROI_WIDTH_RATIO;
	int roi_height = img->height * ROI_HEIGHT_RATIO;

	IplImage* testimg = cvCreateImage(cvSize(img->width, img->height), 8 ,3);

	if( img->nChannels == 1 )
		cvCvtColor(img, testimg, CV_GRAY2RGB);
	else
		cvCopy(img, testimg);

	// 기본 ROI
	int droi_start_x = img->width * ROI_START_RATIO_X;
	int droi_start_y = img->height * ROI_START_RATIO_Y;	
	int roi_start_x = m_nCurrRoiStart_x *  m_dResizeRatio;
	int roi_start_y = m_nCurrRoiStart_y *  m_dResizeRatio;
	cvLine(testimg, cvPoint(droi_start_x, droi_start_y), cvPoint(droi_start_x + roi_width, droi_start_y), CV_RGB(50,50,50));
	cvLine(testimg, cvPoint(droi_start_x + roi_width, droi_start_y), cvPoint(droi_start_x + roi_width, droi_start_y + roi_height), CV_RGB(50,50,50));
	cvLine(testimg, cvPoint(droi_start_x + roi_width, droi_start_y + roi_height), cvPoint(droi_start_x, droi_start_y + roi_height), CV_RGB(50,50,50));
	cvLine(testimg, cvPoint(droi_start_x, droi_start_y + roi_height), cvPoint(droi_start_x, droi_start_y), CV_RGB(50,50,50));

	// 현재 ROI
	cvLine(testimg, cvPoint(roi_start_x, roi_start_y), cvPoint(roi_start_x + roi_width, roi_start_y), CV_RGB(0,255,0));
	cvLine(testimg, cvPoint(roi_start_x + roi_width, roi_start_y), cvPoint(roi_start_x + roi_width, roi_start_y + roi_height), CV_RGB(0,255,0));
	cvLine(testimg, cvPoint(roi_start_x + roi_width, roi_start_y + roi_height), cvPoint(roi_start_x, roi_start_y + roi_height), CV_RGB(0,255,0));
	cvLine(testimg, cvPoint(roi_start_x, roi_start_y + roi_height), cvPoint(roi_start_x, roi_start_y), CV_RGB(0,255,0));

	// op 각도 및 방향 출력
	char s_output_result[50];
    CvFont font;
	int angle180 = 0.0;
	if( m_dVehicleAngleResult == 0.0 ) 		angle180 = 0.0;
	else                                    angle180 = (int)(m_dVehicleAngleResult * 180.0 / PI2);// ~180~180

	sprintf_s(s_output_result, sizeof(s_output_result), "m%.1f/a%d", m_dVehicleMagnitudeResult, angle180);    //우선 sprintf로 문자열 생성	
	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, 0.7, 0.7, 0, 1); 
	cvPutText(testimg, s_output_result ,cvPoint(15,m_nImgHeight-15),&font,cvScalar(255,187,0));   //cvPoint로 글자 시작 위치 설정(uv)

	cvShowImage(show_name, testimg);

	cvReleaseImage(&testimg);
}

/**
	@brief		현재영상과 이전 영상을 이용하여 optical flow 추출	 
	@param		status, track_error 움직임 추정 성공 여부 저장 배열
	@param		nFeatureSize : 이전 영상에서 사용자가 설정한 grid point 개수 (prev_features의 포인트 개수)
	@param		prev_features : 이전 영상에서의 grid points (사용자 설정)
	@param		curr_features : 현재 영상에서의 grid points (optical flow함수 결과)
*/
void COpticalFlow::GetOpticalFlow(IplImage* preImg, IplImage* currImg, char* status, float* track_error, int nGridCount, CvPoint2D32f* prev_features, CvPoint2D32f* curr_features)
{
	// Lucas-Kanede 알고리즘
	int nWin_size = 30;	

	// 피라미드 영상 저장 버퍼
	CvSize pyr_sz=cvSize(m_nImgWidth+8, m_nImgHeight/3);
    IplImage* pyrA=cvCreateImage(pyr_sz, IPL_DEPTH_32F, 1);
    IplImage* pyrB=cvCreateImage(pyr_sz, IPL_DEPTH_32F, 1);	

	/*// Good Feature 추출 =>> 너무 느려짐
	//cvGoodFeaturesToTrack함수에쓰일이미지버퍼생성
	IplImage* eig_image = cvCreateImage( cvGetSize(m_currImg), IPL_DEPTH_32F, 1 );
	IplImage* tmp_image = cvCreateImage( cvGetSize(m_currImg), IPL_DEPTH_32F, 1 );
 	
	//특징있는부분을검출하는함수실행
	cvGoodFeaturesToTrack(m_prevImg, eig_image, tmp_image, m_prev_gridPoint, &m_nGridCount, 0.01, 5.0, 0, 3, 0, 0.04);
	//cvGoodFeaturesToTrack(m_prevImg, eig_image, tmp_image, m_prev_gridPoint, &m_nGridCount, 0.01, 10.0, NULL, 10, 0, 0.4);
	cvFindCornerSubPix(m_prevImg, m_prev_gridPoint, m_nGridCount, cvSize(nWin_size, nWin_size), cvSize(-1, -1), cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 0.03));
	//cvFindCornerSubPix(m_prevImg, m_prev_gridPoint, m_nGridCount, cvSize(nWin_size, nWin_size), cvSize(-1, -1), cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 0.03));
	*/
	//추출한 코너(cornerA)를 추적함 -> 이동한 점들의 위치는 cornerB에 저장된다.	

    cvCalcOpticalFlowPyrLK(preImg, currImg, pyrA, pyrB, prev_features, curr_features, nGridCount,
            cvSize(nWin_size, nWin_size), 5, status, track_error,
            cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, .3), 0);	
	
	//cvReleaseImage( &eig_image ); 
	//cvReleaseImage( &tmp_image );
	cvReleaseImage(&pyrA);
    cvReleaseImage(&pyrB);
	
}

/**
	@brief		현재 영상 grid point와 이전 영상의 grid point 설정
	@param		prev_features : 이전 영상에서의 grid points (사용자 설정)
	@param		curr_features : 현재 영상에서의 grid points (optical flow함수 결과저장을 위한 초기화함)
	@return		사용자가 설정한 이전 영상에서의 grid points 개수
*/
int COpticalFlow::SetGridPoints(IplImage* preImg, IplImage* currImg, CvPoint2D32f* prev_features, CvPoint2D32f* curr_features, int gridInterval)
{
	////////////////////////
	// Grid Point 설정
	///////////////////////
	int count = 0;
	bool tmp_set = false;
	int tmp_set_count = 0;
	//int nBlock_size = gridInterval/2;

	for( int i=0 ; i<m_nROIHeight ; i+=gridInterval )
	{
		for( int j=0 ; j<m_nImgWidth ; j+=gridInterval )
		{
			if( (BYTE)preImg->imageData[i*preImg->widthStep + j] < 20 )
			{
				preImg->imageData[i*preImg->widthStep + j] = 0;
				tmp_set = false;
			}
			else 
				tmp_set = true;	

			if( tmp_set )
			{
				prev_features[count].x = (float)j;
				prev_features[count].y = (float)i;

				count++;
			}
		}
	}	

	// 이동 결과 포인트 변수 초기화
	for(int i=0 ; i<count ; i++ )
	{
		curr_features[i].x = 0;
		curr_features[i].y = 0;
	}

	return count;
}

void COpticalFlow::GetROIvalues(int& roix, int& roiy)
{
	roix = m_nCurrRoiStart_x;
	roiy = m_nCurrRoiStart_y;
}