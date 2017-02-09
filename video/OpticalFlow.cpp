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
	@brief		opticalflowŬ���� ���� �ʱⰪ ����
	@param		h : orignal img height (optical flow������ ���� ������ 1/2 resizing���� ���)
				w : orignal img width
				gridInterval : optical ������ ���� Ư¡�� ������ ���� grid ����
				rateROiH : optical flow���� ROI���� ������ ���� ����(���� ����� �������� ���� ��ü�� �ش� rate���� ��ŭ  ROI y��ǥ ������)
*/
void COpticalFlow::Func_Init(int h, int w, int gridInterval, double imgReseRatio, float rateRoiH)
{	
	int imgH, imgW;	
	int gridCnt;
	
	// ���� ���� resizing ����
	m_dResizeRatio = imgReseRatio;

	// ���� ���� ����
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

	// grid ���� ����
	m_nGridInterval = gridInterval;	
	int nGridInterval_H = m_nROIHeight / gridInterval;
	int nGridInterval_W = m_nImgWidth / gridInterval;
	gridCnt = (nGridInterval_H + 1)*(nGridInterval_W + 1);

	// ������ ���� Point �迭 
	m_prev_gridPoint = new CvPoint2D32f[gridCnt];			// �������� ������ Point �迭
	m_curr_gridPoint = new CvPoint2D32f[gridCnt];			// prev_gridPoint�� ���� ��� Point �迭

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
	@brief			���� ���� ���� ������ ���� �Լ�
*/
void COpticalFlow::SetPreviousImg(IplImage* img)
{	
	if( img->nChannels == 1 )
		cvResize(img, m_prevImg);	//cvCopy(img, cpyimg);
	else
	{
		IplImage* cpyimg = cvCreateImage(cvSize(img->width, img->height), 8, 1);
		cvCvtColor(img, cpyimg, CV_BGR2GRAY);
		cvResize(cpyimg, m_prevImg);		// 1/2������� ���
		cvReleaseImage(&cpyimg);
	}	

	m_bSetPrevImg = true;	
}

/**
	@brief			���� ���� ���� ������ ���� �Լ�
*/
void COpticalFlow::SetCurrentImg(IplImage* img)
{	
	if( img->nChannels == 1 )
		cvResize(img, m_currImg);	//cvCopy(img, cpyimg);
	else
	{
		IplImage* cpyimg = cvCreateImage(cvSize(img->width, img->height), 8, 1);
		cvCvtColor(img, cpyimg, CV_BGR2GRAY);
		cvResize(cpyimg, m_currImg);		// 1/2������� ���
		cvReleaseImage(&cpyimg);
	}		
}

/**
	@brief		Optical Flow�� ���� ���� �Լ�(For ROI Setting)
	@param		currImg : �Է� IplImage ����	
	@param		dir : ���� �ڵ��� ��ǥ ������ ����
	@param		mag : ���� �ڵ��� ��ǥ ������ ũ��
	@return		��������(0), ����������(-1)
*/
int COpticalFlow::mainProcessing(IplImage* currImg, int& dir, double& mag, int nCurrentFrameNum)
{
	if( !m_bSetPrevImg )		
	{
		// m_prevImg ���� ������ ���� ���� �ʾ��� ��� ���� ������ ���� �������� ����
		SetPreviousImg(currImg);
		return -1;
	}	

	// ����� ���� grid ����
	int gridCount = 0;					
	
	// ���� �Է� ���� IplImage �� ��ȯ �� ������ ��ȯ
	SetCurrentImg(currImg);

	// dense optical flow ������ ���� gird ����		
	gridCount = SetGridPoints(m_prevImg, m_currImg, m_prev_gridPoint, m_curr_gridPoint, m_nGridInterval);

	///////////////////////////////
	// optical flow ����
	///////////////////////////////
	// ������ ���� ���� ���� ���� �迭
	char* status = new char[gridCount];
	float* track_error = new float[gridCount];	

	GetOpticalFlow(m_prevImg, m_currImg, status, track_error, gridCount, m_prev_gridPoint, m_curr_gridPoint);
	
	// optical flow ������� ��ȿ����Ʈ ���� ���� ����	
	m_nValidp_count = SetOPValue(status, track_error, gridCount, m_prev_gridPoint, m_curr_gridPoint);	
	
	// 2016-12-23 
	if( m_nValidp_count > 0 )
	{
		// �ڵ��� ��� ������ ����		
		GetAvgOPMagAng(m_dVehicleMagnitudeResult, m_dVehicleAngleResult);	//extract_vehicle_direct_new(m_nVehicleDirectResult, m_dVehicleMagnitudeResult, m_dVehicleAngleResult);		
	}else
	{
		m_dVehicleMagnitudeResult = 0; 
		m_dVehicleAngleResult = 0.0;
	}

	// optical flow�� �̿��� ROI ���� ��ǥ ���� ������ ����
	SetROIvalues(currImg->height, currImg->width);

	// ��� Ȯ�� 
	//print_OpticalFlow_result(nCurrentFrameNum);		// optical flow
	//print_ROI(m_testImg, "roi1");					// ���ũ�� ���� ROI ���

	//print_ROI(m_currImg, "roi2");					// ����ũ�� ���� ROI ���
	// ���� ���� �̹� �������� ����
	cvCopy(m_currImg, m_prevImg);
			
	delete [] status;
	delete [] track_error;		
	
	return 0;
}

/*int COpticalFlow::mainProcessingForDenseFlow(IplImage* currImg, bool& bUpdateMask, int nCurrentFrameNum)
{
	int nVehicleDirectResult = 0;		// 0:����, 1:��ȸ��, 2:��ȸ��	

	if( !m_bSetPrevImg )		// m_prevImg ���� ������ ���� ���� �ʾ��� ��� ���� ������ ���� �������� ����
	{
		//BYTE2IplImage_Gray(currImg_ptr, m_prevImg);
		cvCvtColor(currImg, m_prevImg, CV_BGR2GRAY);
		m_bSetPrevImg = true;
		return -1;
	}	

	// ���� �Է� ���� IplImage �� ��ȯ
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

	// ���� ���� �̹� �������� ����
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
	@brief		Optical Flow ��� ���� ��� �Լ�
*/
int COpticalFlow::print_OpticalFlow_result(int nCurrentFrameNum)
{	
	//OpenCV ȭ�鿡 ���ھ���
    char s_output_result[50];
    CvFont font;
	int valid_pt = 0;  //valid�� ���� ���� count	
	double avg_magnitude = 0.0;			// ������ ũ�� ��� 
	int nMax_avgMagnitude = 0;			// 3�����ӿ��� �ִ� ��� ������ ũ�� ���� ����

	if( m_currImg->nChannels == 1 )
		cvCvtColor(m_currImg, m_testImg, CV_GRAY2RGB);
	else
		cvCopy(m_currImg, m_testImg);
	
	int nResizeLine = 2;		// ȭ��ǥ�� ũ�⸦ ���� ����..

	//draw arrow of optical flow point (ȭ��ǥ ������� �׸��� ���� ������ �Ÿ��� 2�� ���̷� ȭ��ǥ �׷���)
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

		//2�� ���̷� ȭ��ǥ �׷���
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
		sprintf_s(s_output_result, sizeof(s_output_result), "GO STRAIGHT-%.1f/%d", m_dVehicleMagnitudeResult, angle180);    //�켱 sprintf�� ���ڿ� ����
	else if( m_nVehicleDirectResult == 1 )
		sprintf_s(s_output_result, sizeof(s_output_result), "LEFT TURN-%.1f/%d", m_dVehicleMagnitudeResult, angle180);    //�켱 sprintf�� ���ڿ� ����
	else 
		sprintf_s(s_output_result, sizeof(s_output_result), "RIGHT TURN-%.1f/%d", m_dVehicleMagnitudeResult, angle180);    //�켱 sprintf�� ���ڿ� ����
	
	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, 0.7, 0.7, 0, 1); 
	cvPutText(m_testImg, s_output_result ,cvPoint(15,m_nImgHeight-15),&font,cvScalar(255,187,0));   //cvPoint�� ���� ���� ��ġ ����(uv)
	
	cvNamedWindow("Lkpyr_OpticalFlow", CV_WINDOW_AUTOSIZE); 
    cvShowImage("Lkpyr_OpticalFlow", m_testImg); 	
	*/

	return 0;
}

/**
	@brief		���̽þ� �з��⸦ �̿��� Optical Flow�� ���� �ڵ��� ���� ���� ���� �� ��� ����	
	@date		2016-12-19
	@param		result_direct : ���� �ڵ��� ���� ����
	@param		avg_magnitude : ���� �ڵ��� ������ ��� ũ��
	@return		�������� (1:��������)
*/
int COpticalFlow::extract_vehicle_direct_new(int& result_direct, double& avg_magnitude, double& result_angle)
{	
	double angle = 0.0;
	double arrow_length = 0.0; 
	int valid_pt = 0;				// valid�� ���� ���� count
	int count =0;
	
	// for reference Line
	double avg_mag = 0.0;			// ������ ũ�� ��� 
	double avg_magnitude_forDataExpection = 0.0;		// ���� ������ ������ ��� ũ�� ���� (�̿� ���� max ���� ������ ����)
	double avg_ang = 0.0;

	// ���̽þ� �з��⸦ �̿��� ���� ���� ���� �Լ� ���� ����
	/*const double dMeanX[3] = {-0.34, -9.14, 5.12};		// {go, left, right} ���� x�� ���� ���	
	const double dMeanY[3] = {-0.31, -0.33, -0.25};			// {go, left, right} ���� y�� ���� ���	
	const double dVarianceX[3] = {10.26, 19.92, 5.12};		// {go, left, right} ���� x�� ���� �л�
	const double dVarianceY[3] = {2.30, 3.84, 2.52};	    // {go, left, right} ���� y�� ���� �л�	*/
	/*const double dMeanX[3] = {-0.59, -6.74, 4.46};			// {go, left, right} ���� x�� ���� ���	
	const double dMeanY[3] = {-0.40, -0.27, 0.49};			// {go, left, right} ���� y�� ���� ���	
	const double dVarianceX[3] = {16.18, 30.30, 23.98};	// {go, left, right} ���� x�� ���� �л�
	const double dVarianceY[3] = {2.29, 2.51, 0.56};	    // {go, left, right} ���� y�� ���� �л�	*/
	
	const double dMeanX[3] = {-0.57, -6.88, 7.01};			// {go, left, right} ���� x�� ���� ���	
	const double dMeanY[3] = {-0.39, -0.28, -0.18};			// {go, left, right} ���� y�� ���� ���	
	const double dVarianceX[3] = {16.05, 28.35, 28.00};		// {go, left, right} ���� x�� ���� �л�
	const double dVarianceY[3] = {2.28, 2.36, 2.30};	    // {go, left, right} ���� y�� ���� �л�
	
	double dClassification_FuncG[3] = {0.0, 0.0, 0.0};		// ���� �з� �Լ� ���
	
	double dir_sumProb[3] = {0.0};		//  ���� �����ӿ����� �� Ư¡���Ϳ����� ���⺰ Ȯ���� ���� 
	double dMax_value = 0.0;
	int nMax_dir = 0.0;
	int current_direct = 0;			// ���� �������� ���� ���� 
	//int result_direct = 0;			// ���� ���� ����
	int count_direct[3] = {0,0,0};		// �̹� 2�������� ����� ���� ���� ī��Ʈ
	double dValue_x, dValue_y;		// ���� �Է� x, y���� 

	// ��� ������ ���
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

	// ���� ������ ������ ��� ũ�� ���� -> �̸� ���Ͽ� ������ ���� ��ü�� Max����
	for(int i=0; i<m_nValidp_count; ++i) 
    {
		dValue_x = (int)m_op[i].px - (int)m_op[i].cx;
		dValue_y = (int)m_op[i].py - (int)m_op[i].cy;
		arrow_length = sqrt( pow(dValue_y,2) + pow(dValue_x,2) );			
		avg_magnitude_forDataExpection += arrow_length;		
	}
	avg_magnitude_forDataExpection = avg_magnitude_forDataExpection / m_nValidp_count;

	// optical flow ������ ���� ����
	count = 0;
	for(int i=0; i<m_nValidp_count; ++i) 		
    {
		// ������ ���� ����
		dValue_x = m_op[i].px - m_op[i].cx;
		dValue_y = m_op[i].py - m_op[i].cy;

		if( m_op[i].magnitude > avg_magnitude_forDataExpection+10 )
			continue;

		// ���� ���������� op ���� ���� ����� ����Ͽ� optical flow magnitude ��� 		
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
		
		// ���� ��ü�� ���⺰ ������ Ȯ���� ����
		dir_sumProb[0] += dClassification_FuncG[0];
		dir_sumProb[1] += dClassification_FuncG[1];
		dir_sumProb[2] += dClassification_FuncG[2];

		avg_mag += m_op[i].magnitude;	
		avg_ang += m_op[i].angle;
		count++	;
    }

	// ���� ������ ��� ������ ũ�� ����
	avg_magnitude = (avg_mag / count);
	result_angle = (avg_ang / count);

	// ������ ���� ������ �̿��� ���� ���� ����
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
	if( valid_pt < 100 && dMax_value/(double)valid_pt < 0.3 )			// max Ȯ������ 0.3 ���ϸ� ���� �����ӿ��� ����� ���� ���� �������� ��ü
	{
		current_direct = m_nPrevDirect[1];		// ���� �����ӿ��� ���� �������� ����� ���� ����
	}

	if( current_direct == 0 ) Trace(_T("==== current dir : Go\n"));
	else if( current_direct == 1 ) Trace(_T("==== current dir : Left\n"));
	else if( current_direct == 2 ) Trace(_T("==== current dir : Right\n"));
	
	//result_direct = current_direct;
	count_direct[current_direct]++;
	count_direct[m_nPrevDirect[0]]++;
	count_direct[m_nPrevDirect[1]]++;		
		
	
	if( count_direct[0] >= 2 )		// ������������ ����� ���� ����, ������ ����ؼ� �ΰ��̻� ī��Ʈ�� ������ ���� ��� �������� ����
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
	@brief		op�� manitude�� ��տ��� ���� ����� point�� ������ op�� ��� angle, manitude ����	
	@param		avg_magnitude : �ڵ��� ������ ��� ũ��
	@param		result_angle : �ڵ��� ������ ����(radius)	
*/
void COpticalFlow::GetAvgOPMagAng(double& avg_magnitude, double& result_angle)
{
	double arrow_length = 0.0; 
	double avg_mag = 0.0;			// ������ ũ�� ��� 
	double avg_magnitude_forDataExpection = 0.0;		// ���� ������ ������ ��� ũ�� ���� (�̿� ���� max ���� ������ ����)
	double avg_ang = 0.0;
	int count = 0;
	double dValue_x, dValue_y;		// ���� �Է� x, y���� 

	// ���� ������ ������ ��� ũ�� ���� -> �̸� ���Ͽ� ������ ���� ��ü�� Max����
	for(int i=0; i<m_nValidp_count; ++i) 
    {
		dValue_x = (int)m_op[i].px - (int)m_op[i].cx;
		dValue_y = (int)m_op[i].py - (int)m_op[i].cy;
		arrow_length = sqrt( pow(dValue_y,2) + pow(dValue_x,2) );			
		avg_magnitude_forDataExpection += arrow_length;		
	}
	avg_magnitude_forDataExpection = avg_magnitude_forDataExpection / m_nValidp_count;

	// optical flow ������ ���� ����
	count = 0;
	for(int i=0; i<m_nValidp_count; ++i) 		
    {
		if( m_op[i].magnitude > avg_magnitude_forDataExpection+10 )
			continue;

		avg_mag += m_op[i].magnitude;	
		avg_ang += m_op[i].angle;
		count++	;
    }

	// ���� ������ ��� ������ ũ�� ����
	avg_magnitude = (avg_mag / count);
	result_angle = (avg_ang / count);
}

/**
	@brief		�������� m_op������ ���� optical flow ��� ����
	@return		��ȿ�� optical flow point ����
*/
int COpticalFlow::SetOPValue(char* status, float* track_error, int nFeatureSize, CvPoint2D32f* prev_features, CvPoint2D32f* curr_features)
{
	int valid_pt = 0;  //valid�� ���� ���� count
	double angle = 0.0;
	double arrow_length = 0.0; 
	double avg_magnitude = 0.0;			// ������ ũ�� ��� 
	double dValue_x, dValue_y;

	for( int i=0 ; i<nFeatureSize ; i++ )
	{
		if(status[i] != 0)
		{
			dValue_x = (int)prev_features[i].x - (int)curr_features[i].x;
			dValue_y = (int)prev_features[i].y - (int)curr_features[i].y;
		    			
			//angle = atan2( dValue_y, dValue_x ) * 180.0 / PI;		// -180�� ~ 180��
			//if( angle < 0 )		angle += 360;					// 0�� ~ 360�� ��ȯ
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
	@brief		���� ����� optical flow������ ������� �ؼ� roi���� ����
	@date		2016-12-19
	@param		imgH, imgW : �̹��� ������
				roi_start_x, roi_start_y : roi���� ��ǥ ��� ���� ����
*/
void COpticalFlow::SetROIvalues(int imgH, int imgW)
{	
	int roi_default_start_x = imgW * ROI_START_RATIO_X;
	int roi_default_start_y = imgH * ROI_START_RATIO_Y;
	int roi_w = imgW * ROI_WIDTH_RATIO;
	int roi_h = imgH * ROI_HEIGHT_RATIO;	
	int op_mag = m_dVehicleMagnitudeResult / m_dResizeRatio;	// ���� ���������� op ���� ���� ����� ����Ͽ� optical flow magnitude ��� 
	double angle180  = 0.0;	// ~180~180
	int roi_start_x, roi_start_y;

	m_nPrevRoiStart_x = m_nCurrRoiStart_x;
	m_nPrevRoiStart_y = m_nCurrRoiStart_y;

	// ���� : small moving size || �¿� ������ : �⺻ roi ��ǥ ���
    //        large moving size && ���� ������ : �⺻ roi ��ǥ���� op ��� ũ�⸸ŭ �̵�	                                  
	angle180 = m_dVehicleAngleResult * 180.0 / PI2;		// radian -> ~180~180 ��ȯ
	if( m_dVehicleMagnitudeResult > 3. && ( abs(angle180) >= 55.0 && abs(angle180) <= 115.0 ) )	// large moving
	{
		// �⺻��ǥ���� ���� ����, ũ�� ��ŭ �̵�								
		//roi_start_x = m_nPrevRoiStart_x - op_mag*cos(m_dVehicleAngleResult);		// 2016-12-29 ���� ROI�� ���
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

	// �ܼ��ϰ� ���� optical flow ��� ���� ����� ũ�⿡ ���� ROI ����
	// ��ȸ���ϰ�� : optical flow ��� ������ ũ�Ⱑ 4�̻��̰� ������ |135~180|
    // ��ȸ���ϰ�� : optical flow ��� ������ ũ�Ⱑ 4�̻��̰� ������ |0~45|
	// ������ ����� ��&��ȸ�� ����
	/*double angle180 = m_dVehicleAngleResult * 180.0 / PI2;// ~180~180
	if( abs(angle180) >= 135.0 && abs(angle180) <= 180.0 && m_dVehicleMagnitudeResult > 2. )		// ��ȸ�� -> ROI ���������� �̵�
	{
		roi_start_x = roi_default_start_x + roi_w * 0.07;
		roi_start_y = roi_default_start_y;
	}
	else if( abs(angle180) >= 0 && abs(angle180) <= 45.0 && m_dVehicleMagnitudeResult > 2.  )		// ��ȸ�� -> ROI �������� �̵�
	{
		roi_start_x = roi_default_start_x - roi_w * 0.07;	
		roi_start_y = roi_default_start_y;
	}*/
	// �������Ⱥз��⸦ �̿��� ��&��ȸ�� ����
	/*if( m_nVehicleDirectResult == 2 )				// ��ȸ�� -> ROI ���������� �̵�
	{
		roi_start_x = roi_default_start_x + roi_w * 0.07;
		roi_start_y = roi_default_start_y;
	}
	else if( m_nVehicleDirectResult == 1 )			// ��ȸ�� -> ROI �������� �̵�
	{
		roi_start_x = roi_default_start_x - roi_w * 0.07;	
		roi_start_y = roi_default_start_y;
	}else                                           // ����
	{
        if( m_dVehicleMagnitudeResult > 3. )	// large moving
		{
			// �⺻��ǥ���� ���� ����, ũ�� ��ŭ �̵�								
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
	@brief		ROI ���� �̹��� ���
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

	// �⺻ ROI
	int droi_start_x = img->width * ROI_START_RATIO_X;
	int droi_start_y = img->height * ROI_START_RATIO_Y;	
	int roi_start_x = m_nCurrRoiStart_x *  m_dResizeRatio;
	int roi_start_y = m_nCurrRoiStart_y *  m_dResizeRatio;
	cvLine(testimg, cvPoint(droi_start_x, droi_start_y), cvPoint(droi_start_x + roi_width, droi_start_y), CV_RGB(50,50,50));
	cvLine(testimg, cvPoint(droi_start_x + roi_width, droi_start_y), cvPoint(droi_start_x + roi_width, droi_start_y + roi_height), CV_RGB(50,50,50));
	cvLine(testimg, cvPoint(droi_start_x + roi_width, droi_start_y + roi_height), cvPoint(droi_start_x, droi_start_y + roi_height), CV_RGB(50,50,50));
	cvLine(testimg, cvPoint(droi_start_x, droi_start_y + roi_height), cvPoint(droi_start_x, droi_start_y), CV_RGB(50,50,50));

	// ���� ROI
	cvLine(testimg, cvPoint(roi_start_x, roi_start_y), cvPoint(roi_start_x + roi_width, roi_start_y), CV_RGB(0,255,0));
	cvLine(testimg, cvPoint(roi_start_x + roi_width, roi_start_y), cvPoint(roi_start_x + roi_width, roi_start_y + roi_height), CV_RGB(0,255,0));
	cvLine(testimg, cvPoint(roi_start_x + roi_width, roi_start_y + roi_height), cvPoint(roi_start_x, roi_start_y + roi_height), CV_RGB(0,255,0));
	cvLine(testimg, cvPoint(roi_start_x, roi_start_y + roi_height), cvPoint(roi_start_x, roi_start_y), CV_RGB(0,255,0));

	// op ���� �� ���� ���
	char s_output_result[50];
    CvFont font;
	int angle180 = 0.0;
	if( m_dVehicleAngleResult == 0.0 ) 		angle180 = 0.0;
	else                                    angle180 = (int)(m_dVehicleAngleResult * 180.0 / PI2);// ~180~180

	sprintf_s(s_output_result, sizeof(s_output_result), "m%.1f/a%d", m_dVehicleMagnitudeResult, angle180);    //�켱 sprintf�� ���ڿ� ����	
	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, 0.7, 0.7, 0, 1); 
	cvPutText(testimg, s_output_result ,cvPoint(15,m_nImgHeight-15),&font,cvScalar(255,187,0));   //cvPoint�� ���� ���� ��ġ ����(uv)

	cvShowImage(show_name, testimg);

	cvReleaseImage(&testimg);
}

/**
	@brief		���翵��� ���� ������ �̿��Ͽ� optical flow ����	 
	@param		status, track_error ������ ���� ���� ���� ���� �迭
	@param		nFeatureSize : ���� ���󿡼� ����ڰ� ������ grid point ���� (prev_features�� ����Ʈ ����)
	@param		prev_features : ���� ���󿡼��� grid points (����� ����)
	@param		curr_features : ���� ���󿡼��� grid points (optical flow�Լ� ���)
*/
void COpticalFlow::GetOpticalFlow(IplImage* preImg, IplImage* currImg, char* status, float* track_error, int nGridCount, CvPoint2D32f* prev_features, CvPoint2D32f* curr_features)
{
	// Lucas-Kanede �˰���
	int nWin_size = 30;	

	// �Ƕ�̵� ���� ���� ����
	CvSize pyr_sz=cvSize(m_nImgWidth+8, m_nImgHeight/3);
    IplImage* pyrA=cvCreateImage(pyr_sz, IPL_DEPTH_32F, 1);
    IplImage* pyrB=cvCreateImage(pyr_sz, IPL_DEPTH_32F, 1);	

	/*// Good Feature ���� =>> �ʹ� ������
	//cvGoodFeaturesToTrack�Լ��������̹������ۻ���
	IplImage* eig_image = cvCreateImage( cvGetSize(m_currImg), IPL_DEPTH_32F, 1 );
	IplImage* tmp_image = cvCreateImage( cvGetSize(m_currImg), IPL_DEPTH_32F, 1 );
 	
	//Ư¡�ִºκ��������ϴ��Լ�����
	cvGoodFeaturesToTrack(m_prevImg, eig_image, tmp_image, m_prev_gridPoint, &m_nGridCount, 0.01, 5.0, 0, 3, 0, 0.04);
	//cvGoodFeaturesToTrack(m_prevImg, eig_image, tmp_image, m_prev_gridPoint, &m_nGridCount, 0.01, 10.0, NULL, 10, 0, 0.4);
	cvFindCornerSubPix(m_prevImg, m_prev_gridPoint, m_nGridCount, cvSize(nWin_size, nWin_size), cvSize(-1, -1), cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 0.03));
	//cvFindCornerSubPix(m_prevImg, m_prev_gridPoint, m_nGridCount, cvSize(nWin_size, nWin_size), cvSize(-1, -1), cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 0.03));
	*/
	//������ �ڳ�(cornerA)�� ������ -> �̵��� ������ ��ġ�� cornerB�� ����ȴ�.	

    cvCalcOpticalFlowPyrLK(preImg, currImg, pyrA, pyrB, prev_features, curr_features, nGridCount,
            cvSize(nWin_size, nWin_size), 5, status, track_error,
            cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, .3), 0);	
	
	//cvReleaseImage( &eig_image ); 
	//cvReleaseImage( &tmp_image );
	cvReleaseImage(&pyrA);
    cvReleaseImage(&pyrB);
	
}

/**
	@brief		���� ���� grid point�� ���� ������ grid point ����
	@param		prev_features : ���� ���󿡼��� grid points (����� ����)
	@param		curr_features : ���� ���󿡼��� grid points (optical flow�Լ� ��������� ���� �ʱ�ȭ��)
	@return		����ڰ� ������ ���� ���󿡼��� grid points ����
*/
int COpticalFlow::SetGridPoints(IplImage* preImg, IplImage* currImg, CvPoint2D32f* prev_features, CvPoint2D32f* curr_features, int gridInterval)
{
	////////////////////////
	// Grid Point ����
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

	// �̵� ��� ����Ʈ ���� �ʱ�ȭ
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