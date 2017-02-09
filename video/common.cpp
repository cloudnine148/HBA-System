#include "StdAfx.h"
#include "common.h"

//int gRFtype = RF_CLASSIFICATION_TREE;	//random forest ���� ������ ���� 


void Trace(TCHAR* pszFormat, ...)
{    //���������� ���̸� �𸣱⶧���� �������� �����ϱ� ���� ������ ����    
	TCHAR* pBuffer = NULL;
	va_list args;
	va_start(args, pszFormat);
	// �������� ��Ʈ���� ���̸� ���Ѵ�.    
	int nLen = _vsctprintf(pszFormat, args) + sizeof(TCHAR);
	// �������� ���̸�ŭ ���ڿ��� �����Ѵ�.    
	pBuffer = new TCHAR[nLen];
	// �����Ѵ�.    
	_vstprintf_s(pBuffer, nLen, pszFormat, args);
	va_end(args);
	// Windows������ �ʿ����     
	// ������ش�.   
	::OutputDebugString(pBuffer);
	if (pBuffer)
	{
		delete[] pBuffer;
		pBuffer = NULL;
	}
}

//150318 ���� pixel ������ ������ ���� ������ Ȯ���ϱ� ���� �Լ�
//@param :	1. �˻��� ��
//@return :	true : �ȼ������� ������ ��ȿ ��
//			false : �ȼ������� ��� �Ұ����� ��
bool IsValidFor8bitPixelVal(int val)
{
	if (val < 0 || val > 255)
		return false;

	return true;
}

//150223 �������� ���������� �簢������ �߽��� ��ǥ�� ����ϴ� �Լ�
//@param :	1~2. �������� ������
//@return :	�߽��� ��ǥ
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

//150223 ���� ������ �Ÿ� ��� �Լ�
//@param :	1~2. �Ÿ� ����� ����
//@return : ���� ������ �Ÿ� (double)
double GetDistanceBetween2Points(Point point1, Point point2)
{
	double dist = 0.0f;

	dist = sqrt(pow((float)point1.x - point2.x, 2) + pow((float)point1.y - point2.y, 2));

	return dist;
}

//151103 x,y ��ǥ�� Point �ڷ��� �����ϴ� �Լ�
//@param :	1~2. x,y ��ǥ
//@return : �ش� ��ǥ�� ������ ����ü
Point SetPoint(int x, int y)
{
	Point p;
	p.x = x;
	p.y = y;
	return p;
}

//151105 �޸� �����ϰ� nullptr�� �����ϴ� �Լ�
//@param :	1. ������ �迭 ������
//			2. ������ nullptr�� set ����
void free_memory(void* arr, bool setnullptr)
{
	delete[] arr;
	if (setnullptr)
		arr = nullptr;
}

//151110 ��ǥ ���� ����ó���ؼ� ��ȯ�ϴ� �Լ�
//@ param :	1. ����Ȯ���� ��ǥ ����
//			2~5. x,y�� �ּ�, �ִ� ����
Point GetPointAfterExceptionHandling(Point cor, int xmin, int xmax, int ymin, int ymax)
{
	Point res;

	res.x = MAX_VAL(xmin, cor.x);
	res.x = MIN_VAL(xmax, res.x);

	res.y = MAX_VAL(ymin, cor.y);
	res.y = MIN_VAL(ymax, res.y);

	return res;
}

//151203 �߽� ���� �̵��� ��ǥ�� �������� ���ϴ� ������ŭ �̵������� ��ǥ�� ����ϴ� �Լ�
//@param :	1. �̵� �߽��� 
//			2. �̵��� ��� ����
//			3. �̵����� (degree)
Point GetRotatedPoint(Point centroid, Point point, double degree)
{
	Point ret;
	ret.x = point.x;
	ret.y = point.y;

	if (centroid.x == point.x && centroid.y == point.y)	//������(�߽���)�� ȸ���Ϸ��� ���� ���� ���� ��� ���� ��ġ ��ȯ 
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

// ǥ������ ���ϴ� �Լ�
double CalculateStandardDeviation_FOR_BYTE(CDib& dib, double meanValue, int width, int height)
{
	BYTE** ptr = dib.GetPtr();

	int sum = 0, pixel_count = 0;
	double mean, variance, s_deviation;


	mean = meanValue;

	// �л� ���ϱ�
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

// ǥ������ ���ϴ� �Լ�
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

	// �л� ���ϱ�
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


// ��հ��
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
	

	//�ʱ�ȭ
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