#include "StdAfx.h"
#include "common.h"

//int gRFtype = RF_CLASSIFICATION_TREE;	//random forest 종류 선택을 위해 


void Trace(TCHAR* pszFormat, ...)
{    //가변인자의 길이를 모르기때문에 동적으로 생성하기 위한 포인터 변수    
	TCHAR* pBuffer = NULL;
	va_list args;
	va_start(args, pszFormat);
	// 가변인자 스트링의 길이를 구한다.    
	int nLen = _vsctprintf(pszFormat, args) + sizeof(TCHAR);
	// 가변인자 길이만큼 문자열을 생성한다.    
	pBuffer = new TCHAR[nLen];
	// 복사한다.    
	_vstprintf_s(pBuffer, nLen, pszFormat, args);
	va_end(args);
	// Windows에서는 필요없음     
	// 출력해준다.   
	::OutputDebugString(pBuffer);
	if (pBuffer)
	{
		delete[] pBuffer;
		pBuffer = NULL;
	}
}

//150318 값이 pixel 값으로 가능한 정수 값인지 확인하기 위한 함수
//@param :	1. 검사할 값
//@return :	true : 픽셀값으로 가능한 유효 값
//			false : 픽셀값으로 사용 불가능한 값
bool IsValidFor8bitPixelVal(int val)
{
	if (val < 0 || val > 255)
		return false;

	return true;
}

//150223 시작점과 종료점으로 사각영역의 중심점 좌표를 계산하는 함수
//@param :	1~2. 시작점과 종료점
//@return :	중심점 좌표
Point GetCenterPointofRect(Point startp, Point endp)
{
	Point center;

	center.x = (int)(((float)(endp.x - startp.x) / 2.0) + startp.x);
	center.y = (int)(((float)(endp.y - startp.y) / 2.0) + startp.y);

	return center;
}


void L2Normalization(double* data, int size)
{
	register int i;

	double temp = 0.0;
	double dL2norm = 0.0;

	for (i = 0; i < size; i++)
		temp += data[i] * data[i];

	dL2norm = sqrt(temp);

	for (i = 0; i < size; i++)
		data[i] = data[i] / (dL2norm + 0.2);
}

//150223 두점 사이의 거리 계산 함수
//@param :	1~2. 거리 계산할 지점
//@return : 두점 사이의 거리 (double)
double GetDistanceBetween2Points(Point point1, Point point2)
{
	double dist = 0.0f;

	dist = sqrt(pow((float)point1.x - point2.x, 2) + pow((float)point1.y - point2.y, 2));

	return dist;
}

//151103 x,y 좌표로 Point 자료형 리턴하는 함수
//@param :	1~2. x,y 좌표
//@return : 해당 좌표를 저장한 구조체
Point SetPoint(int x, int y)
{
	Point p;
	p.x = x;
	p.y = y;
	return p;
}

//151105 메모리 해제하고 nullptr로 셋팅하는 함수
//@param :	1. 해제할 배열 포인터
//			2. 해제후 nullptr로 set 여부
void free_memory(void* arr, bool setnullptr)
{
	delete[] arr;
	if (setnullptr)
		arr = nullptr;
}

//151110 좌표 정보 예외처리해서 반환하는 함수
//@ param :	1. 예외확인할 좌표 정보
//			2~5. x,y의 최소, 최대 범위
Point GetPointAfterExceptionHandling(Point cor, int xmin, int xmax, int ymin, int ymax)
{
	Point res;

	res.x = MAX_VAL(xmin, cor.x);
	res.x = MIN_VAL(xmax, res.x);

	res.y = MAX_VAL(ymin, cor.y);
	res.y = MIN_VAL(ymax, res.y);

	return res;
}

//151203 중심 점과 이동할 좌표를 기준으로 원하는 각도만큼 이동했을때 좌표를 계산하는 함수
//@param :	1. 이동 중심점 
//			2. 이동할 대상 지점
//			3. 이동각도 (degree)
Point GetRotatedPoint(Point centroid, Point point, double degree)
{
	Point ret;
	ret.x = point.x;
	ret.y = point.y;

	if (centroid.x == point.x && centroid.y == point.y)	//기준점(중심점)과 회전하려는 점이 같은 점인 경우 원래 위치 반환 
		return ret;

	double radian = degree * PI / 180.;
	double sinResult = sin(radian);
	double cosResult = cos(radian);
	ret.x = (int)((point.x - centroid.x)*cosResult - (point.y - centroid.y)*sinResult);
	ret.y = (int)((point.x - centroid.x)*sinResult + (point.y - centroid.y)*cosResult);
	ret.x += centroid.x;
	ret.y += centroid.y;

	//Trace( _T("@@ point=%d.%d centroid=%d.%d degree=%3.2f ret=%d.%d\n"), point.x, point.y, centroid.x, centroid.y,
	//	degree, ret.x, ret.y );
	return ret;
}

// 표준편차 구하는 함수
double CalculateStandardDeviation_FOR_BYTE(CDib& dib, double meanValue, int width, int height)
{
	BYTE** ptr = dib.GetPtr();

	int sum = 0, pixel_count = 0;
	double mean, variance, s_deviation;


	mean = meanValue;

	// 분산 구하기
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			sum += pow((ptr[i][j] - mean), 2);
			pixel_count++;
		}
	}

	if (pixel_count == 0)
		variance = 0;
	else
		variance = sum / pixel_count;

	s_deviation = sqrt(variance);

	return s_deviation;
}

// 표준편차 구하는 함수
void CalculateStandardDeviation_FOR_RGBBYTE(CDib& dib, double* meanValue, int width, int height, double* std)
{
	RGBBYTE** ptr = dib.GetRGBPtr();

	int* sum = new int[3];
	int* pixel_count = new int[3];
	double* variance = new double[3];

	memset(sum, 0, sizeof(int) * 3);
	memset(pixel_count, 0, sizeof(int) * 3);
	memset(variance, 0, sizeof(double) * 3);
	memset(std, 0, sizeof(double) * 3);

	// 분산 구하기
	/* 
	RGB Channel
	R = 0
	G = 1
	B = 2
	*/
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			if (ptr[i][j].r != 0)
			{
				sum[0] += pow((ptr[i][j].r - meanValue[0]), 2);
				pixel_count[0]++;
			}

			if (ptr[i][j].g != 0)
			{
				sum[1] += pow((ptr[i][j].g - meanValue[1]), 2);
				pixel_count[1]++;
			}

			if (ptr[i][j].b != 0)
			{
				sum[2] += pow((ptr[i][j].b - meanValue[2]), 2);
				pixel_count[2]++;
			}
		}
	}

	for (int i = 0; i < 3; i++)
	{
		if (pixel_count[i] == 0)
			variance[i] = 0;
		else
			variance[i] = sum[i] / pixel_count[i];
	}
	
	for (int i = 0; i < 3; i++)
	{
		std[i] = sqrt(variance[i]);
	}

	delete[] sum;
	delete[] pixel_count;
	delete[] variance;

}


// 평균계산
double CalculateMeanValue_FOR_BYTE(CDib& dib, int width, int height)
{
	BYTE** ptr = dib.GetPtr();
	int sum = 0, pixel_count = 0;
	double mean;
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			sum += ptr[i][j];
			pixel_count++;
		}
	}

	if (pixel_count == 0)
		mean = 0;
	else
		mean = sum / pixel_count;

	return mean;
}

void CalculateMeanValue_FOR_RGBBYTE(CDib& dib, int width, int height, double* mean)
{
	RGBBYTE** ptr = dib.GetRGBPtr();
	int* sum = new int[3];
	int* pixel_count = new int[3];
	

	//초기화
	memset(sum, 0, sizeof(int) * 3);
	memset(pixel_count, 0, sizeof(int) * 3);
	memset(mean, 0, sizeof(double) * 3);

	/*
	RGB Channel
	R = 0
	G = 1
	B = 2
	*/
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			if (ptr[i][j].r != 0)
			{
				sum[0] += ptr[i][j].r;
				pixel_count[0]++;
			}
			if (ptr[i][j].g != 0)
			{
				sum[1] += ptr[i][j].g;
				pixel_count[1]++;
			}
			if (ptr[i][j].b != 0)
			{
				sum[2] += ptr[i][j].b;
				pixel_count[2]++;
			}
		}
	}

	for (int i = 0; i < 3; i++)
	{
		if (pixel_count[i] == 0)
			mean[i] = 0;
		else
			mean[i] = sum[i] / pixel_count[i];
	}

	delete[] sum;
	delete[] pixel_count;
}