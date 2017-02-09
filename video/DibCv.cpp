#include "stdafx.h"
#include "DibCv.h"
#include "common.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif


//150212 IplImage로부터 Dib 생성하는 함수 (24bit전용)
//@param :	1. 원본 iplimage
//			2. 복사해 갈 empty Dib
//@ return :	-1 : error
//				0 : normal
int Get24bitDibFromIplImage(IplImage* img, CDib& dib)
{
	int ret = 0;
	//상위 함수에서 예외처리 하기때문에 추가 검사 하지 않음
	
	dib.CreateRGBImage( img->width, img->height, 0 );
	RGBBYTE** ptr = dib.GetRGBPtr();

	int hei = dib.GetHeight();
	int wid = dib.GetWidth();
	int h,w;
	h = w = 0;

	//R,G,B 3색 영상(24bit)을 기준으로 하기 때문에 hei*wid*3
	for( double iter = 0; iter < hei*wid*3; iter=iter+1 ) {
		ptr[h][w].b = (BYTE)img->imageData[(h*wid*3)+(w*3)];
		ptr[h][w].g = (BYTE)img->imageData[(h*wid*3)+(w*3)+1];
		ptr[h][w].r = (BYTE)img->imageData[(h*wid*3)+(w*3)+2];
		w++;
		if( w == wid-1 ) {
			h++;
			w = 0;
		}
		if( h == hei-1 ) {
			//Trace(_T("[%s,%d] copy done (iter=%lf)\n"), _function_, _line_, iter);
			break;
		}
	} 
	return ret;
}

//150213 Dib로부터 IplImage 생성하는 함수 (24bit전용)
//@param :	1. 원본 Dib
//			2. 복사해 갈 empty iplimage
//@ return :	-1 : error
//				0 : normal
int GetIplImageFrom24bitDib(CDib& dib, IplImage** img)
{
	int ret = 0;
	//상위 함수에서 예외처리 하기때문에 추가 검사 하지 않음
	
	CvSize size;
	size.width = dib.GetWidth();
	size.height = dib.GetHeight();

	if( img != NULL )		cvReleaseImage(img);
	*img = cvCreateImage( size, IPL_DEPTH_8U, 3 );
	
	RGBBYTE** ptr = dib.GetRGBPtr();
	int h, w;
	h = w = 0;
	for( double iter = 0; iter < size.height*size.width*3; iter=iter+1 ) {
		(*img)->imageData[(h*size.width*3)+(w*3)]	= ptr[h][w].b;
		(*img)->imageData[(h*size.width*3)+(w*3)+1]	= ptr[h][w].g;
		(*img)->imageData[(h*size.width*3)+(w*3)+2]	= ptr[h][w].r;
		w++;
		if( w == size.width-1 ) {
			h++;
			w = 0;
		}
		if( h == size.height-1 ) {
			//Trace(_T("[%s,%d] copy done (iter=%lf)\n"), _function_, _line_, iter);
			break;
		}
	}
	return ret;
}

//150216 IplImage로부터 Dib 생성하는 함수 (8bit전용)
//@param :	1. 원본 iplimage
//			2. 복사해 갈 empty Dib
//@ return :	-1 : error
//				0 : normal
int Get8bitDibFromIplImage(IplImage* img, CDib&dib)
{
	int ret = 0;
	//상위 함수에서 예외처리 하기때문에 추가 검사 하지 않음
	
	dib.CreateGrayImage( img->width, img->height, 0 );
	BYTE** ptr = dib.GetPtr();

	int hei = dib.GetHeight();
	int wid = dib.GetWidth();
	int h,w;
	h = w = 0;

	for( double iter = 0; iter < hei*wid; iter=iter+1 ) {
		ptr[h][w] = (BYTE)img->imageData[ (h*wid)+w ];
		w++;
		if( w == wid-1 ) {
			h++;
			w = 0;
		}
		if( h == hei-1 ) {
			//Trace(_T("[%s,%d] copy done (iter=%lf)\n"), _function_, _line_, iter);
			break;
		}
	}
	return ret;
}

//150216 Dib로부터 IplImage 생성하는 함수 (8bit전용)
//@param :	1. 원본 Dib
//			2. 복사해 갈 empty iplimage
//@ return :	-1 : error
//				0 : normal
int GetIplImageFrom8bitDib(CDib& dib, IplImage** img)
{
	int ret = 0;
	//상위 함수에서 예외처리 하기때문에 추가 검사 하지 않음

	CvSize size;
	size.width = dib.GetWidth();
	size.height = dib.GetHeight();
	
	if( img != NULL )		cvReleaseImage(img);
	*img = cvCreateImage( size, IPL_DEPTH_8U, 1 );	//1 color

	BYTE** ptr = dib.GetPtr();
	int h, w;
	h = w = 0;
	for( double iter = 0; iter < size.height*size.width; iter=iter+1 ) {
		(*img)->imageData[ (h*size.width) + w ] = ptr[h][w];
		w++;
		if( w == size.width-1 ) {
			h++;
			w = 0;
		}
		if( h == size.height-1 ) {
			//trace(_t("[%s,%d] copy done (iter=%lf)\n"), _function_, _line_, iter);
			break;
		}
	}
	return ret;
}

int GetDibFromIplImage(IplImage* img, CDib& dib)
{
	int ret = 0;

	//예외처리
	if( img == nullptr ) {
		Trace( _T("[%s,%d] nullptr\n"), _function_, _line_);
		ret = -1;
		return ret;
	}
	if( img->nChannels != 1 && img->nChannels != 3 ) {		
		Trace( _T("[%s,%d] color channel wrong(%d) \n"), _function_, _line_, img->nChannels);
		ret = -1;
		return ret;
	}
	if( img->depth != IPL_DEPTH_8U ) {
		Trace( _T("[%s,%d] not 8 bit(depth=%d) \n"), _function_, _line_, img->depth);
		ret = -1;
		return ret;
	}

	if( img->nChannels == 1 ) 
		Get8bitDibFromIplImage( img, dib );
	else 
		Get24bitDibFromIplImage( img, dib );
	
	return ret;
}

int GetIplImageFromDib(CDib& dib, IplImage** img)
{
	int ret = 0;

	//예외처리
	if( dib.GetBitCount() != 8 && dib.GetBitCount() != 24 ) {
		Trace( _T("[%s,%d] bit count not match(%d)\n"), _function_, _line_, dib.GetBitCount() );
		return -1;
		return ret;
	}

	if( dib.GetBitCount() == 8 )
		GetIplImageFrom8bitDib( dib, img );
	else
		GetIplImageFrom24bitDib( dib, img );

	return ret;
}


void ShowImage(CDib& dib, char *name, int nWaitTime)
{
	int nChannelCount = dib.GetBitCount() / 8;
	
	IplImage* testImg = cvCreateImage(cvSize(dib.GetWidth(), dib.GetHeight()), 8, nChannelCount);

	if( nChannelCount == 3 )
		GetIplImageFrom24bitDib(dib, &testImg);
	else 
		GetIplImageFrom8bitDib(dib, &testImg);

	cvNamedWindow(name);
	cvShowImage(name, testImg);
	cvWaitKey();
	cvDestroyWindow(name);

	cvReleaseImage(&testImg);
}

	
//Based on the code from Dan and the suggestion of mkuse, here is a function with the same syntax as cv::line():

void arrowedLine(IplImage* img, CvPoint pt1, CvPoint pt2, const CvScalar& color, int thickness, int line_type, int shift, double tipLength)
{	
	double dist = sqrt( (pt1.x - pt2.x) * (pt1.x - pt2.x) + (pt1.y - pt2.y) * (pt1.y - pt2.y) );
	
	//const double tipSize = norm(pt1-pt2)*tipLength; // Factor to normalize the size of the tip depending on the length of the arrow
	const double tipSize = dist*tipLength; // Factor to normalize the size of the tip depending on the length of the arrow

    cvLine(img, pt1, pt2, color, thickness, line_type, shift);

    const double angle = atan2( (double) pt1.y - pt2.y, (double) pt1.x - pt2.x );

    CvPoint p = cvPoint(cvRound(pt2.x + tipSize * cos(angle + CV_PI / 4)), cvRound(pt2.y + tipSize * sin(angle + CV_PI / 4)));
	
    cvLine(img, p, pt2, color, thickness, line_type, shift);
    
	p.x = cvRound(pt2.x + tipSize * cos(angle - CV_PI / 4));
    p.y = cvRound(pt2.y + tipSize * sin(angle - CV_PI / 4));
    
	cvLine(img, p, pt2, color, thickness, line_type, shift);

	
}