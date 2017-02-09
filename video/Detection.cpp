#include "stdafx.h"
#include <vector>
#include "Detection.h"

#define PIXEL_TH 1500
#define RATIO_TH 5

#define LENGTH_TH 70

// Label area extanding
#define LABEL_AREA_EXTAND_RATIO 0.3

// Classification number
#define HEAD_LIGHT 0
#define TAIL_LIGHT 1
#define OTHERS 2
#define LOAD_SIGN 3
#define TRAFFIC_LIGHT 4


// Tail light threshold
#define TAIL_LIGHT_TH 1.5
#define TAIL_LIGHT_MIN_R 150

#define max(a,b) a > b ? a : b
#define min(a,b) a < b ? a : b

// OCSLBP
#define OCSLBP_SUBBLOCK_SIZE 2
#define OCSLBP_BIN_SIZE 8

#define COLOR_FEATURE_SIZE 6
#define WAVELET_FEATURE_SIZE 32


// Redness
#define REDNESS_TH 0.2

static const int MAX_CLASS = 3;	// 클래스 수

static struct sSampleParam {
	CvScalar color_pt;			// 셈플 색
	CvScalar color_bg;			// 배경 색
} sample_param[MAX_CLASS] = {
	{ CV_RGB(180, 0, 0), CV_RGB(255, 0, 0), },
	{ CV_RGB(0, 180, 0), CV_RGB(0, 255, 0), },
	{ CV_RGB(0, 0, 180), CV_RGB(0, 0, 255), }
	//{ CV_RGB(180, 0, 180), CV_RGB(255, 0, 255), },
};

CDetection::CDetection()
{
}


CDetection::~CDetection()
{
	if (RFInfo.trees != NULL) {
		delete[] RFInfo.trees;
		delete[] RFInfo.treesWeight;
	}

	if (RFInfo_first.trees != NULL) {
		delete[] RFInfo_first.trees;
		delete[] RFInfo_first.treesWeight;
	}
}

void CDetection::InitValue(int roi_min_x, int roi_min_y, int roi_max_x, int roi_max_y)
{
	ROI_MIN = roi_min_y;
	ROI_MIN_X = roi_min_x;
	ROI_MAX = roi_max_y;	
	ROI_MAX_X = roi_max_x;
}

void CDetection::Labeling(CDib dib)
{
	//	TRACE(_T("\n 레이블링 합니다!! 시작! \n"));
	register int i, j, ii, jj;

	int startH = 0;
	int startW = 0;
	int w = 0;
	int h = 0;


	RGBBYTE** ptr;

	ptr = dib.GetRGBPtr();
	startH = 1;
	startW = 1;

	w = dib.GetWidth();
	h = dib.GetHeight();


	// 레이블링 MAP
	CDib result_dib;
	result_dib.CreateGrayImage(w, h, 0);
	BYTE** res_ptr = result_dib.GetPtr();

	//-------------------------------------------------------------------------
	// 임시로 레이블을 저장할 메모리 공간과 등가 테이블 생성
	//-------------------------------------------------------------------------

	int** map = new int*[h];
	for (i = 0; i < h; i++)
	{
		map[i] = new int[w];
		memset(map[i], 0, sizeof(int)*w);
	}

	int eq_tbl[MAX_LABEL][2] = { { 0, }, };

	//-------------------------------------------------------------------------
	// 첫 번째 스캔 - 초기 레이블 지정 및 등가 테이블 생성
	//-------------------------------------------------------------------------

	int label = 0, maxl, minl, min_eq, max_eq;

	for (j = 1, jj = startH; j < h; j++, jj++) {
		for (i = 1, ii = startW; i < w; i++, ii++)
		{
			// 바로 위 픽셀과 왼쪽 픽셀 모두에 레이블이 존재하는 경우
			if ((map[j - 1][i] != 0) && (map[j][i - 1] != 0))
			{
				if (map[j - 1][i] == map[j][i - 1])
				{
					// 두 레이블이 서로 같은 경우
					map[j][i] = map[j - 1][i];
				}
				else
				{
					// 두 레이블이 서로 다른 경우, 작은 레이블을 부여
					maxl = max(map[j - 1][i], map[j][i - 1]);
					minl = min(map[j - 1][i], map[j][i - 1]);

					map[j][i] = minl;

					// 등가 테이블 조정
					min_eq = min(eq_tbl[maxl][1], eq_tbl[minl][1]);
					max_eq = max(eq_tbl[maxl][1], eq_tbl[minl][1]);

					eq_tbl[eq_tbl[max_eq][1]][1] = min_eq;
				}
			}
			else if (map[j - 1][i] != 0)
			{
				// 바로 위 픽셀에만 레이블이 존재할 경우
				map[j][i] = map[j - 1][i];
			}
			else if (map[j][i - 1] != 0)
			{
				// 바로 왼쪽 픽셀에만 레이블이 존재할 경우
				map[j][i] = map[j][i - 1];
			}
			else
			{
				// 이웃에 레이블이 존재하지 않으면 새로운 레이블을 부여
				label++;
				map[j][i] = label;
				eq_tbl[label][0] = label;
				eq_tbl[label][1] = label;
			}
		}
	}
	//-------------------------------------------------------------------------
	// 등가 테이블 정리
	//-------------------------------------------------------------------------

	int temp;
	for (i = 1; i <= label; i++)
	{
		temp = eq_tbl[i][1];
		if (temp != eq_tbl[i][0])
			eq_tbl[i][1] = eq_tbl[temp][1];
	}

	// 등가 테이블의 레이블을 1부터 차례대로 증가시키기

	int* hash = new int[label + 1];
	memset(hash, 0, sizeof(int)*(label + 1));

	for (i = 1; i <= label; i++)
		hash[eq_tbl[i][1]] = eq_tbl[i][1];

	int cnt = 1;
	for (i = 1; i <= label; i++)
		if (hash[i] != 0)
			hash[i] = cnt++;

	for (i = 1; i <= label; i++) {
		eq_tbl[i][1] = hash[eq_tbl[i][1]];
	}

	delete[] hash;

	//-------------------------------------------------------------------------
	// 두 번째 스캔 - 등가 테이블을 이용하여 모든 픽셀에 고유의 레이블 부여
	//-------------------------------------------------------------------------

	int* temp_check = new int[label + 1];
	memset(temp_check, 0, sizeof(int)*(label + 1));
	int sum_temp = 0, itr;
	double e_num = 0.05;
	double e = 0;
	for (j = 1; j < h; j++)
		for (i = 1; i < w; i++)
		{
			if (map[j][i] != 0)
			{
				temp = map[j][i];
				res_ptr[j][i] = (BYTE)(eq_tbl[temp][1]);
				temp_check[res_ptr[j][i]]++;
			}
		}

	for (i = 1; i < label + 1; i++) {
		sum_temp += temp_check[i];
	}


	e = e_num * (sum_temp / (cnt - 1));

	for (itr = 1; itr < label + 1; itr++) {
		if (temp_check[itr] < e) {
			for (j = 1; j < h; j++)
				for (i = 1; i < w; i++)
				{
					if (res_ptr[j][i] == itr)
					{
						res_ptr[j][i] = 0;
					}
				}
		}
	}


	delete[] temp_check;

	//-------------------------------------------------------------------------
	// 임시 메모리 공간 해제
	//-------------------------------------------------------------------------

	int rs = cnt - 1;

	for (i = 0; i < h; i++) {
		delete[] map[i];
	}

	delete[] map;

}



int CDetection::DibLabeling(CDib& dib, CDib& result)
{
	register int i, j;

	int w = dib.GetWidth();
	int h = dib.GetHeight();

	BYTE** ptr = dib.GetPtr();

	//-------------------------------------------------------------------------  
	// 임시로 레이블을 저장할 메모리 공간과 등가 테이블 생성  
	//-------------------------------------------------------------------------  

	int** map = new int*[h];
	for (i = 0; i < h; i++)
	{
		map[i] = new int[w];
		memset(map[i], 0, sizeof(int)*w);
	}

	int eq_tbl[MAX_LABEL][2] = { { 0, }, };

	//-------------------------------------------------------------------------  
	// 첫 번째 스캔 - 초기 레이블 지정 및 등가 테이블 생성  
	//-------------------------------------------------------------------------  

	int label = 0, maxl, minl, min_eq, max_eq;

	for (j = 1; j < h; j++)
		for (i = 1; i < w; i++)
		{
			if (ptr[j][i] != 0)
			{
				// 바로 위 픽셀과 왼쪽 픽셀 모두에 레이블이 존재하는 경우  
				if ((map[j - 1][i] != 0) && (map[j][i - 1] != 0))
				{
					if (map[j - 1][i] == map[j][i - 1])
					{
						// 두 레이블이 서로 같은 경우  
						map[j][i] = map[j - 1][i];
					}
					else
					{
						// 두 레이블이 서로 다른 경우, 작은 레이블을 부여  
						maxl = max(map[j - 1][i], map[j][i - 1]);
						minl = min(map[j - 1][i], map[j][i - 1]);

						map[j][i] = minl;

						// 등가 테이블 조정  
						min_eq = min(eq_tbl[maxl][1], eq_tbl[minl][1]);
						max_eq = max(eq_tbl[maxl][1], eq_tbl[minl][1]);

						eq_tbl[eq_tbl[max_eq][1]][1] = min_eq;
					}
				}
				else if (map[j - 1][i] != 0)
				{
					// 바로 위 픽셀에만 레이블이 존재할 경우  
					map[j][i] = map[j - 1][i];
				}
				else if (map[j][i - 1] != 0)
				{
					// 바로 왼쪽 픽셀에만 레이블이 존재할 경우  
					map[j][i] = map[j][i - 1];
				}
				else
				{
					// 이웃에 레이블이 존재하지 않으면 새로운 레이블을 부여  
					label++;
					map[j][i] = label;
					eq_tbl[label][0] = label;
					eq_tbl[label][1] = label;
				}
			}
		}

	//-------------------------------------------------------------------------  
	// 등가 테이블 정리  
	//-------------------------------------------------------------------------  

	int temp;
	for (i = 1; i <= label; i++)
	{
		temp = eq_tbl[i][1];
		if (temp != eq_tbl[i][0])
			eq_tbl[i][1] = eq_tbl[temp][1];
	}

	// 등가 테이블의 레이블을 1부터 차례대로 증가시키기  

	int* hash = new int[label + 1];
	memset(hash, 0, sizeof(int)*(label + 1));

	for (i = 1; i <= label; i++)
		hash[eq_tbl[i][1]] = eq_tbl[i][1];

	int cnt = 1;
	for (i = 1; i <= label; i++)
		if (hash[i] != 0)
			hash[i] = cnt++;

	for (i = 1; i <= label; i++)
		eq_tbl[i][1] = hash[eq_tbl[i][1]];

	delete[] hash;

	//-------------------------------------------------------------------------  
	// 두 번째 스캔 - 등가 테이블을 이용하여 모든 픽셀에 고유의 레이블 부여  
	//-------------------------------------------------------------------------  

	result.CreateGrayImage(w, h, 0);
	BYTE** result_ptr = result.GetPtr();

	for (j = 1; j < h; j++)
		for (i = 1; i < w; i++)
		{
			if (map[j][i] != 0)
			{
				temp = map[j][i];
				result_ptr[j][i] = (BYTE)(eq_tbl[temp][1]);
			}
		}


	//-------------------------------------------------------------------------  
	// Create information for detection structure 
	//-------------------------------------------------------------------------  

	DetectionInfo.rg = new sRegion[cnt];
	DetectionInfo.cnt = cnt;

	int min_x, min_y, max_x, max_y;

	for (int k = 1; k < DetectionInfo.cnt; k++) {
		
		min_x = w;
		max_x = 0;
		min_y = h;
		max_y = 0;

		for (j = 1; j < h; j++) {
			for (i = 1; i < w; i++) {
				if (k == result_ptr[j][i]) {
					if (min_x > i) min_x = i;
					if (max_x < i) max_x = i;
					if (min_y > j) min_y = j;
					if (max_y < j) max_y = j;
				}
			}
		}

		DetectionInfo.rg[k].min_x = min_x;
		DetectionInfo.rg[k].max_x = max_x;
		DetectionInfo.rg[k].min_y = min_y;
		DetectionInfo.rg[k].max_y = max_y;

		DetectionInfo.rg[k].h = max_y - min_y;
		DetectionInfo.rg[k].w = max_x - min_x;
	}


	//-------------------------------------------------------------------------  
	// 임시 메모리 공간 해제  
	//-------------------------------------------------------------------------  

	for (i = 0; i < h; i++)
		delete[] map[i];
	delete[] map;

	return (cnt - 1);
}


void CDetection::DrawBox(CDib& dib, CDib& result) {
		
	if( DetectionInfo.cnt <= 0 )			// 2016-12-126 JMR
		return;

	int i, j, k;
	int w = dib.GetWidth();
	int h = dib.GetHeight();

	RGBBYTE** res_ptr = result.GetRGBPtr();

	int type_r[] = { 0,			244,		255 };
	int type_g[] = { 255,		120,		0 };
	int type_b[] = { 0,			50,			0 };

	int type = 0;

	for (k = 1; k < DetectionInfo.cnt; k++) {

		type = DetectionInfo.rg[k].type;
	
			for (j = 1; j < h; j++)
				for (i = 1; i < w; i++)
				{
					if (j == DetectionInfo.rg[k].min_y && (DetectionInfo.rg[k].min_x <= i && DetectionInfo.rg[k].max_x >= i))
					{



						//res_ptr[j][i].r = type_r[type];
						//res_ptr[j][i].g = type_g[DetectionInfo.rg[k].type];
						//res_ptr[j][i].b = type_b[DetectionInfo.rg[k].type];

						//res_ptr[j-1][i-1].r = type_r[DetectionInfo.rg[k].type];
						//res_ptr[j-1][i-1].g = type_g[DetectionInfo.rg[k].type];
						//res_ptr[j-1][i-1].b = type_b[DetectionInfo.rg[k].type];

						//res_ptr[j + 1][i + 1].r = type_r[DetectionInfo.rg[k].type];
						//res_ptr[j + 1][i + 1].g = type_g[DetectionInfo.rg[k].type];
						//res_ptr[j + 1][i + 1].b = type_b[DetectionInfo.rg[k].type];

						res_ptr[j][i] = RGB(type_r[type], type_g[type], type_b[type]);
						res_ptr[j - 1][i - 1] = RGB(type_r[type], type_g[type], type_b[type]);
						res_ptr[j + 1][i + 1] = RGB(type_r[type], type_g[type], type_b[type]);


					}

					else if (j == DetectionInfo.rg[k].max_y && (DetectionInfo.rg[k].min_x <= i && DetectionInfo.rg[k].max_x >= i))
					{
						//res_ptr[j][i].r = type_r[DetectionInfo.rg[k].type];
						//res_ptr[j][i].g = type_g[DetectionInfo.rg[k].type];
						//res_ptr[j][i].b = type_b[DetectionInfo.rg[k].type];

						//res_ptr[j - 1][i - 1].r = type_r[DetectionInfo.rg[k].type];
						//res_ptr[j - 1][i - 1].g = type_g[DetectionInfo.rg[k].type];
						//res_ptr[j - 1][i - 1].b = type_b[DetectionInfo.rg[k].type];

						//res_ptr[j + 1][i + 1].r = type_r[DetectionInfo.rg[k].type];
						//res_ptr[j + 1][i + 1].g = type_g[DetectionInfo.rg[k].type];
						//res_ptr[j + 1][i + 1].b = type_b[DetectionInfo.rg[k].type];

						res_ptr[j][i] = RGB(type_r[type], type_g[type], type_b[type]);
						res_ptr[j - 1][i - 1] = RGB(type_r[type], type_g[type], type_b[type]);
						res_ptr[j + 1][i + 1] = RGB(type_r[type], type_g[type], type_b[type]);

					}
					else if (i == DetectionInfo.rg[k].min_x && (DetectionInfo.rg[k].min_y <= j && DetectionInfo.rg[k].max_y >= j))
					{
						//res_ptr[j][i].r = type_r[DetectionInfo.rg[k].type];
						//res_ptr[j][i].g = type_g[DetectionInfo.rg[k].type];
						//res_ptr[j][i].b = type_b[DetectionInfo.rg[k].type];

						//res_ptr[j - 1][i - 1].r = type_r[DetectionInfo.rg[k].type];
						//res_ptr[j - 1][i - 1].g = type_g[DetectionInfo.rg[k].type];
						//res_ptr[j - 1][i - 1].b = type_b[DetectionInfo.rg[k].type];

						//res_ptr[j + 1][i + 1].r = type_r[DetectionInfo.rg[k].type];
						//res_ptr[j + 1][i + 1].g = type_g[DetectionInfo.rg[k].type];
						//res_ptr[j + 1][i + 1].b = type_b[DetectionInfo.rg[k].type];

						res_ptr[j][i] = RGB(type_r[type], type_g[type], type_b[type]);
						res_ptr[j - 1][i - 1] = RGB(type_r[type], type_g[type], type_b[type]);
						res_ptr[j + 1][i + 1] = RGB(type_r[type], type_g[type], type_b[type]);
					}

					else if (i == DetectionInfo.rg[k].max_x && (DetectionInfo.rg[k].min_y <= j && DetectionInfo.rg[k].max_y >= j))
					{
						//res_ptr[j][i].r = type_r[DetectionInfo.rg[k].type];
						//res_ptr[j][i].g = type_g[DetectionInfo.rg[k].type];
						//res_ptr[j][i].b = type_b[DetectionInfo.rg[k].type];

						//res_ptr[j - 1][i - 1].r = type_r[DetectionInfo.rg[k].type];
						//res_ptr[j - 1][i - 1].g = type_g[DetectionInfo.rg[k].type];
						//res_ptr[j - 1][i - 1].b = type_b[DetectionInfo.rg[k].type];

						//res_ptr[j + 1][i + 1].r = type_r[DetectionInfo.rg[k].type];
						//res_ptr[j + 1][i + 1].g = type_g[DetectionInfo.rg[k].type];
						//res_ptr[j + 1][i + 1].b = type_b[DetectionInfo.rg[k].type];

						res_ptr[j][i] = RGB(type_r[type], type_g[type], type_b[type]);
						res_ptr[j - 1][i - 1] = RGB(type_r[type], type_g[type], type_b[type]);
						res_ptr[j + 1][i + 1] = RGB(type_r[type], type_g[type], type_b[type]);
					}
					else
					{

					}

				}
		
	}
}

void CDetection::RemoveROIExtern(CDib& dib) {

	int i, j;
	int w = dib.GetWidth();
	int h = dib.GetHeight();

	BYTE** ptr = dib.GetPtr();

	for (j = 0; j < h; j++) {
		for (i = 0; i < w; i++) {
			if (ROI_MIN > j)
			{
				ptr[j][i] = 0;
			}

			if (ROI_MAX < j)
			{
				ptr[j][i] = 0;
			}

			if (i < ROI_MIN_X)
			{
				ptr[j][i] = 0;
			}

			if (i > ROI_MAX_X)
			{
				ptr[j][i] = 0;
			}
		}

	}
}

int CDetection::LabelArrangement() {
	//TRACE(_T("Before : %d\n"), DetectionInfo.cnt);
	int i, w, h;
	int flag = 0;

	// 레이블 정리
	for (i = 1; i < DetectionInfo.cnt; i++) {

		//TRACE(_T("%d =>  w:%d / h:%d\n"), i, DetectionInfo.rg[i].w, DetectionInfo.rg[i].h);
		if (flag == 1) {
			i = i - 1;
			flag = 0;
		}

		if ((DetectionInfo.rg[i].w <= 3) || (DetectionInfo.rg[i].h <= 3) || (DetectionInfo.rg[i].w >= LENGTH_TH) || (DetectionInfo.rg[i].h >= LENGTH_TH))  {
			RemoveIdx(i);
			flag = 1;
		}
		else if (((DetectionInfo.rg[i].w / DetectionInfo.rg[i].h) > RATIO_TH) || ((DetectionInfo.rg[i].h / DetectionInfo.rg[i].w) > RATIO_TH ) ||
			((DetectionInfo.rg[i].w * DetectionInfo.rg[i].h) >= PIXEL_TH)) {
			RemoveIdx(i);
			flag = 1;
		}
	}
	
	// 마지막 값 제거
	int tempCnt = DetectionInfo.cnt-1;
	if ((DetectionInfo.rg[tempCnt].w <= 3) || (DetectionInfo.rg[tempCnt].h <= 3) || (DetectionInfo.rg[tempCnt].w >= LENGTH_TH) || (DetectionInfo.rg[tempCnt].h >= LENGTH_TH)) {
		DetectionInfo.cnt -= 1;
	}
	else if (((DetectionInfo.rg[tempCnt].w / DetectionInfo.rg[tempCnt].h) > RATIO_TH) || ((DetectionInfo.rg[tempCnt].h / DetectionInfo.rg[tempCnt].w) > RATIO_TH) ||
		((DetectionInfo.rg[tempCnt].w * DetectionInfo.rg[tempCnt].h) >= PIXEL_TH)) {
		DetectionInfo.cnt -= 1;
	}


	if ((DetectionInfo.rg[1].w <= 3) || (DetectionInfo.rg[1].h <= 3) || (DetectionInfo.rg[1].w >= LENGTH_TH) || (DetectionInfo.rg[1].h >= LENGTH_TH)) {
		RemoveIdx(1);
	}
	else if (((DetectionInfo.rg[1].w / DetectionInfo.rg[1].h) > RATIO_TH) || ((DetectionInfo.rg[1].h / DetectionInfo.rg[1].w) > RATIO_TH) ||
		((DetectionInfo.rg[1].w * DetectionInfo.rg[1].h) >= PIXEL_TH)) {
		RemoveIdx(1);
	}

	//TRACE(_T("After : %d\n"), DetectionInfo.cnt);
	return DetectionInfo.cnt;
}

void CDetection::RemoveIdx(int idx) {
	int i;

	for (i = idx; i < DetectionInfo.cnt-1; i++) {
		DetectionInfo.rg[i] = DetectionInfo.rg[i + 1];
	}

	DetectionInfo.cnt -= 1;
}



void CDetection::LabelAreaExtand(int w, int h) {

	int i, wTh, hTh;

	for (i = 1; i < DetectionInfo.cnt; i++) {
		wTh = DetectionInfo.rg[i].w * LABEL_AREA_EXTAND_RATIO;
		hTh = DetectionInfo.rg[i].h * LABEL_AREA_EXTAND_RATIO;

		if ((DetectionInfo.rg[i].min_x - wTh) >= 0) DetectionInfo.rg[i].min_x -= wTh;
		else DetectionInfo.rg[i].min_x = 1;

		if ((DetectionInfo.rg[i].max_x + wTh) <= w) DetectionInfo.rg[i].max_x += wTh;
		else DetectionInfo.rg[i].max_x = w - 1;

		if ((DetectionInfo.rg[i].min_y - hTh) >= 0) DetectionInfo.rg[i].min_y -= hTh;
		else DetectionInfo.rg[i].min_x = 1;

		if ((DetectionInfo.rg[i].max_y + hTh) <= h) DetectionInfo.rg[i].max_y += hTh;
		else DetectionInfo.rg[i].max_x = h - 1;


		DetectionInfo.rg[i].w = DetectionInfo.rg[i].max_x - DetectionInfo.rg[i].min_x;
		DetectionInfo.rg[i].h = DetectionInfo.rg[i].max_y - DetectionInfo.rg[i].min_y;
	}
}


void CDetection::Classfication(CDib& dib) {




}

void CDetection::ExtractColorFeature(CDib& dib) {
	int i, j, k;

	RGBBYTE** ptr = dib.GetRGBPtr();
	int sumR, sumG, sumB;
	sumR = sumG = sumB = 0;
	float variR, variG, variB;
	variR = variG = variB = 0.0;
	int NumOfPixels = 0;

	

	for (k = 1; k < DetectionInfo.cnt; k++) {
		sumR = sumG = sumB = 0;
		variR = variG = variB = 0.0;

		for (j = DetectionInfo.rg[k].min_y; j < DetectionInfo.rg[k].max_y; j++) {
			for (i = DetectionInfo.rg[k].min_x; i < DetectionInfo.rg[k].max_x; i++) {
				sumR += ptr[j][i].r;
				sumG += ptr[j][i].g;
				sumB += ptr[j][i].b;
			}
		}
		
		NumOfPixels = DetectionInfo.rg[k].h * DetectionInfo.rg[k].w;

		
		if (NumOfPixels != 0) {
			DetectionInfo.rg[k].feature.color.mean_R = sumR / NumOfPixels;
			DetectionInfo.rg[k].feature.color.mean_G = sumG / NumOfPixels;
			DetectionInfo.rg[k].feature.color.mean_B = sumB / NumOfPixels;
		}
		else {
			DetectionInfo.rg[k].feature.color.mean_R = 0;
			DetectionInfo.rg[k].feature.color.mean_G = 0;
			DetectionInfo.rg[k].feature.color.mean_B = 0;
		}

		for (j = DetectionInfo.rg[k].min_y; j < DetectionInfo.rg[k].max_y; j++) {
			for (i = DetectionInfo.rg[k].min_x; i < DetectionInfo.rg[k].max_x; i++) {
				variR += pow(ptr[j][i].r - DetectionInfo.rg[k].feature.color.mean_R, 2);
				variG += pow(ptr[j][i].g - DetectionInfo.rg[k].feature.color.mean_G, 2);
				variB += pow(ptr[j][i].b - DetectionInfo.rg[k].feature.color.mean_B, 2);
			}
		}


		if (NumOfPixels != 0) {
			DetectionInfo.rg[k].feature.color.variance_R = variR / NumOfPixels;
			DetectionInfo.rg[k].feature.color.variance_G = variG / NumOfPixels;
			DetectionInfo.rg[k].feature.color.variance_B = variB / NumOfPixels;
		}
		else {
			DetectionInfo.rg[k].feature.color.variance_R = 0;
			DetectionInfo.rg[k].feature.color.variance_G = 0;
			DetectionInfo.rg[k].feature.color.variance_B = 0;
		}
	}

}

void CDetection::TrainSVM() {

	cv::RNG rng;

	//IplImage *img = cvCreateImage(cvSize(1000, 1000), IPL_DEPTH_8U, 3);
	//cvZero(img);

	//cvNamedWindow("result", CV_WINDOW_AUTOSIZE);

	// 학습 데이터의 총 수
	int sample_count = 423;
	//int sample_count = 601;

	// 학습 데이터와 클래스를 할당할 행렬 생성
	//CvMat *train_data = cvCreateMat(sample_count, 2, CV_32FC1);

	CvMat *train_data = cvCreateMat(sample_count, 6, CV_32FC1);

	//CvMat *train_class = cvCreateMat(sample_count, 1, CV_32SC1);

	CvMat *train_class = cvCreateMat(sample_count, 1, CV_32SC1);



	// 무작위로 학습 데이터를 생성 후 영역 별로 클래스를 할당
	//for (int k = 0; k<sample_count; k++){

	//	float x = (float)rng.uniform(0, img->width);
	//	float y = (float)rng.uniform(0, img->height);

	//	// !!! 학습 데이터는 0~1 사이로 정규화 하여 사용한다.
	//	CV_MAT_ELEM(*train_data, float, k, 0) = x / img->width;
	//	CV_MAT_ELEM(*train_data, float, k, 1) = y / img->height;

	//	int c = (x - 2 * y + 400 > 0) ? ((x > 600) ? 0 : 1) : ((x > 400) ? 1 : 2);
	//	CV_MAT_ELEM(*train_class, long, k, 0) = c;

	//}

	// SVM 학습 매개 변수를 다음과 같이 설정한다.


	//ifstream inputFile("input.txt");

	//string inputString;
	//char* dup = nullptr;
	//char* token = nullptr;
	//int cnt = 0;
	//int tokenCnt = 0;

	//if (inputFile.is_open()) {
	//	cout << "Don't exist file" << endl;
	//}
	//else {
	//	while (!inputFile.eof()) {
	//		cnt++;
	//		tokenCnt = 0;
	//		inputFile >> inputString;
	//		
	//		dup = strdup(inputString.c_str());
	//		token = strtok(dup, "\t");
	//		while (token != NULL) {

	//			
	//			
	//			token = strtok(NULL, "\t");
	//		}
	//		
	//	}
	//}
	




	CString strFileFullPath;

	CString temp = _T(".\\input.txt");
	//strFileFullPath = m_strEditDir + temp;

	TCHAR sep = '\t';
	CStdioFile fRead;
	CFileException fException;
	CString strLine;


	if (!fRead.Open(temp, CFile::modeRead, &fException))
	{
		fException.ReportError();
		return;
	}

	fRead.SeekToBegin();
	int nIndex = 0;

	// 데이터 추출
	while (fRead.ReadString(strLine))
	{
		int i = 0;
		int arrTemp[7];

		CString fileName;
		// protection for multi blank.f
		for (int j = 0; i < 7; j++)	// "i" is correct. don't be confused.
		{
			// push values
			CString strVal;
			AfxExtractSubString(strVal, strLine, j, sep);

			if (strVal == "")
				continue;

			arrTemp[i] = static_cast<float>(_wtof(strVal));

			i++;
		}


		CV_MAT_ELEM(*train_data, float, nIndex, 0) = arrTemp[0];
		CV_MAT_ELEM(*train_data, float, nIndex, 1) = arrTemp[1];
		CV_MAT_ELEM(*train_data, float, nIndex, 2) = arrTemp[2];
		CV_MAT_ELEM(*train_data, float, nIndex, 3) = arrTemp[3];
		CV_MAT_ELEM(*train_data, float, nIndex, 4) = arrTemp[4];
		CV_MAT_ELEM(*train_data, float, nIndex, 5) = arrTemp[5];

		CV_MAT_ELEM(*train_class, long, nIndex, 0) = arrTemp[6];
		//rect.left = arrTemp[0]; rect.right = arrTemp[1]; rect.top = arrTemp[2]; rect.bottom = arrTemp[3];
	

		// push values
		//CString strFileName;
		//AfxExtractSubString(strFileName, strLine, 4, sep);

		nIndex++;

	}

	fRead.Close();


	//1e-6

	CvSVMParams param;
	param.svm_type = CvSVM::C_SVC;
	param.kernel_type = CvSVM::LINEAR;
	param.term_crit = cvTermCriteria(CV_TERMCRIT_ITER, 100, 0.000001);

	// SVM 분류기가 주어진 데이터와 클래스를 학습한다.
	svm.train(train_data, train_class, cv::Mat(), cv::Mat(), param);

	cvReleaseMat(&train_class);
	cvReleaseMat(&train_data);

}

void CDetection::SVMClassification() {

	CvMat *Test_data = cvCreateMat(1, 6, CV_32FC1);


	for (int i = 1; i < DetectionInfo.cnt; i++) {
		CV_MAT_ELEM(*Test_data, float, 0, 0) = DetectionInfo.rg[i].feature.color.mean_R;
		CV_MAT_ELEM(*Test_data, float, 0, 1) = DetectionInfo.rg[i].feature.color.mean_G;
		CV_MAT_ELEM(*Test_data, float, 0, 2) = DetectionInfo.rg[i].feature.color.mean_B;
		CV_MAT_ELEM(*Test_data, float, 0, 3) = DetectionInfo.rg[i].feature.color.variance_R;
		CV_MAT_ELEM(*Test_data, float, 0, 4) = DetectionInfo.rg[i].feature.color.variance_G;
		CV_MAT_ELEM(*Test_data, float, 0, 5) = DetectionInfo.rg[i].feature.color.variance_B;

		DetectionInfo.rg[i].type = svm.predict(Test_data);
	}

	cvReleaseMat(&Test_data);

	//// SVM 분류기가 이미지의 모든 픽셀에 대해 각 픽셀이 
	//// 어느 클래스에 속하는지 추정하여 클래스를 할당한다.
	//for (int x = 0; x<img->width; x++) {
	//	for (int y = 0; y<img->height; y++) {
	//		float sample_[2] = { (float)x / img->width, (float)y / img->height };
	//		CvMat sample = cvMat(1, 2, CV_32FC1, sample_);

	//		// SVM 분류기가 주어진 픽셀이 어느 클래스에 속하는지 추정한다.
	//		float response = svm.predict(&sample);

	//		// 이미지에 추정된 클래스를 색으로 표시한다.
	//		cvSet2D(img, y, x, sample_param[cvRound(response)].color_pt);
	//	}
	//}

	//// 학습 데이터를 이미지에 그린다.
	//for (int k = 0; k<sample_count; k++) {
	//	int x = cvRound(img->width* CV_MAT_ELEM(*train_data, float, k, 0));
	//	int y = cvRound(img->height*CV_MAT_ELEM(*train_data, float, k, 1));
	//	int c = cvRound(CV_MAT_ELEM(*train_class, long, k, 0));

	//	cvCircle(img, cvPoint(x, y), 2, sample_param[c].color_bg, CV_FILLED);
	//}

	//// 서포트 벡터를 그린다.
	//int n = svm.get_support_vector_count();
	//for (int i = 0; i<n; i++) {
	//	const float *support = svm.get_support_vector(i);
	//	int x = cvRound(support[0] * img->width);
	//	int y = cvRound(support[1] * img->height);

	//	cvCircle(img, cvPoint(x, y), 4, CV_RGB(255, 255, 255), 1);
	//}

	//

	//cvShowImage("result", img);

	//// 키를 누르면 종료
	//cvWaitKey(0);

	//cvReleaseMat(&train_class);
	//cvReleaseMat(&train_data);

	//cvDestroyWindow("result");
	//cvReleaseImage(&img);

}

//151028 jwgim 특징 파일 로드해서 rf 학습시키는 함수
//@ param -	1. 특징 파일 경로
//			2. 트리 파일 경로
//			3. 학습된 RF 저장 위치
//			4. 학습시킬 트리 갯수
//			5. 학습시킬 좋은 트리 갯수
//			6. 데이터 선택 비율
//			7. 학습 클래스 갯수

void CDetection::TrainRF(CString featurePath, CString treePath, int treeNum, int goodtreeNum, double sampleRatio, int class_num)
{
	CStdioFile fRead;
	CFileException fException;
	CString strLine;
	TCHAR sep = '\t';

	int info;

	int count = 0;
	int row = 0;
	bool firstLine = true;

	int trainDataRow = 0;
	int trainDataColumn = 0;

	double** trainDataSet = nullptr;

	// 특징 데이터 로드 
	if (!fRead.Open(featurePath, CFile::modeRead, &fException))
	{
		fException.ReportError();
		return;
	}

	while (fRead.ReadString(strLine))
	{
		if (firstLine) // trainData row, column
		{
			CString strTemp;
			AfxExtractSubString(strTemp, strLine, 0, sep);
			trainDataRow = _wtoi(strTemp);
			AfxExtractSubString(strTemp, strLine, 1, sep);
			trainDataColumn = _wtoi(strTemp);

			trainDataSet = new double*[trainDataRow];
			for (int i = 0; i < trainDataRow; i++)
			{
				trainDataSet[i] = new double[trainDataColumn];

				memset(trainDataSet[i], 0, sizeof(double) * trainDataColumn);
			}

			firstLine = false;
		}
		else
		{
			int i = 0;									// protection for multi blank.f
			for (int j = 0; i < trainDataColumn; j++)	// "i" is correct. don't be confused.
			{
				// push values
				CString strVal;
				AfxExtractSubString(strVal, strLine, j, sep);
				if (strVal == "")
				{
					continue;
				}
				trainDataSet[row][i] = _wtof(strVal);
				i++;
			}
			row++;
		}
	}

	// Random Forest 트레이닝
	generateRandomForests(trainDataSet, trainDataRow, trainDataColumn - 1, class_num, treeNum, sampleRatio, goodtreeNum, info, RFInfo);	// 200, 0.85	

	// 메모리 해제
	for (int i = 0; i < trainDataRow; i++)
	{
		delete[] trainDataSet[i];
	}

	delete[] trainDataSet;

	fRead.Close();


	/////////////////////////문자열 유니코드에서 ->멀티바이트로 변환//////////////////////////////////////////
	FILE *fp1;
	wchar_t* path = treePath.GetBuffer(0);
	treePath.ReleaseBuffer();

	char*	pPath;
	int		outputSize = 0;

	outputSize = WideCharToMultiByte(CP_ACP, 0, path, (int)wcslen(path), NULL, 0, NULL, NULL);
	pPath = new char[outputSize + 1];

	WideCharToMultiByte(CP_ACP, 0, path, (int)wcslen(path), pPath, outputSize, NULL, NULL); // 길이만큼 멀티바이트로 변환한다.

	pPath[outputSize] = '\0';

	fopen_s(&fp1, pPath, "wt");

	delete[] pPath;

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// Random Forest 학습 트리 데이터 출력
	fprintf(fp1, "%d\n", RFInfo.bufsize);
	fprintf(fp1, "%d\n", RFInfo.theNumberOfClass);
	fprintf(fp1, "%d\n", RFInfo.theNumberOfTree);
	fprintf(fp1, "%d\n", RFInfo.featureDemension);

	for (int i = 0; i < RFInfo.theNumberOfTree; i++)
	{
		//fprintf(fp1, "%d\t", df.treesFlag[i]);
		fprintf(fp1, "%f\t", RFInfo.treesWeight[i]);
	}

	fprintf(fp1, "\n");

	for (int i = 0; i < RFInfo.bufsize; i++)
	{
		fprintf(fp1, "%f\n", RFInfo.trees[i]);
	}

	fclose(fp1);

}


int CDetection::LoadTreeData(CString treePath, RANDOMFORESTS& rf)
{
	CStdioFile fRead;
	CFileException fException;
	CString strLine;
	TCHAR sep = '\t';

	int line = 0;
	int row = 0;

	if (!fRead.Open(treePath, CFile::modeRead, &fException)) {
		fException.ReportError();
		return FILEOPEN_FAIL;
	}

	while (fRead.ReadString(strLine)) {
		CString strTemp;
		if (line == 0) {
			rf.bufsize = _wtoi(strLine.Trim());

			if (rf.trees != NULL) {
				delete[] rf.trees;
				delete[] rf.treesWeight;
			}

			rf.trees = new double[rf.bufsize + 1];
			memset(rf.trees, 0, sizeof(double) * (rf.bufsize + 1));

			line++;
		}
		else if (line == 1) {
			rf.theNumberOfClass = _wtoi(strLine.Trim());
			line++;
		}
		else if (line == 2) {
			rf.theNumberOfTree = _wtoi(strLine.Trim());
			rf.treesWeight = new double[rf.theNumberOfTree];
			memset(rf.treesWeight, 0, sizeof(double) * rf.theNumberOfTree);

			line++;
		}
		else if (line == 3) {
			rf.featureDemension = _wtoi(strLine.Trim());
			line++;
		}
		else if (line == 4) {
			int i = 0;
			for (int j = 0; i < rf.theNumberOfTree; j++) {	// "i" is correct. don't be confused.
				// push values
				CString strVal;
				AfxExtractSubString(strVal, strLine, j, sep);
				if (strVal == "")
					continue;
				rf.treesWeight[i] = _wtof(strVal);
				i++;
			}
			line++;
		}
		else
			rf.trees[row++] = _wtof(strLine.Trim());
	}
	fRead.Close();	

	return SUCCESS;
}

int CDetection::LoadAllTreeData()
{	
	int result = UNKNOWN;

	result = LoadTreeData(FILE_PATH_DETECT_RF_TREE_WAVELET, RFInfo);
	if( result != SUCCESS )		return result;
	
	result = LoadTreeData(FILE_PATH_DETECT_RF_TREE_HAAR, RFInfo_first);
	if( result != SUCCESS )		return result;
	
	result = LoadTreeData(FILE_PATH_DETECT_RF_TREE_WAVELET_TAIL, RFInfo_first_tail);
	if( result != SUCCESS )		return result;

	result = LoadTreeData(FILE_PATH_DETECT_RF_TREE_COLOR_HISTO, RFInfo_Color);
	if (result != SUCCESS)		return result;

	return SUCCESS;
}


void CDetection::RFColorClassification() {

	double prob = 0.0f;
	double* result = new double[CLASSES_TESTING];
	memset(result, 0, sizeof(double)*CLASSES_TESTING);
	double* feature = new double[7];

	for (int i = 1; i < DetectionInfo.cnt; i++) {

		feature[0] = DetectionInfo.rg[i].feature.color.mean_R;
		feature[1] = DetectionInfo.rg[i].feature.color.mean_G;
		feature[2] = DetectionInfo.rg[i].feature.color.mean_B;
		feature[3] = DetectionInfo.rg[i].feature.color.variance_R;
		feature[4] = DetectionInfo.rg[i].feature.color.variance_G;
		feature[5] = DetectionInfo.rg[i].feature.color.variance_B;
		
		estimateProbabilityUsingRandomForests(RFInfo, feature, false, result);

		double temp = result[0];

		if (temp > 0.6) {
			DetectionInfo.rg[i].type = TAIL_LIGHT;
		}
		else {
			DetectionInfo.rg[i].type = OTHERS;
		}

	}

	delete[] result;
	delete[] feature;
}

//151028 jwgim 영상에서 ocslbp 특징 추출하는 함수
//@ param -	1. 추출할 영상 (gray)
//			2. 추출된 특징 저장할 배열 포인터
//			3. 특징벡터 크기
//			4. 서브 블록 갯수(가로) 8
//			5. 서브 블록 갯수(세로) 8
//			6. OCS-LBP bin 크기 (서브블록당 크기) 8
//			7. OCS-LBP 누적 방법
//			8. 방향 누적 기준 임계값 6

// 서브 블록 2 x 2 , 

void CDetection::GetOCSLBPFeatureFromImage(BYTE** ptr, int height, int width ,double* feature, int featureSize, int blkWidth, int blkHeight, int binSize, int threshold)
{
	//BYTE** ptr = dib.GetPtr();

	int sub_region = blkWidth*blkHeight;
	double total_pixel = 0.0;

	int Height = height;
	int Width = width;
	int i, j, m, n;
	int startM = 0;
	int endM = 0;
	int startN = 0;
	int endN = 0;

	// Building histogram (region)
	int marginY = Height % blkHeight;
	int marginX = Width % blkWidth;

	double** histogram = new double*[sub_region];

	for (i = 0; i < sub_region; i++)
	{
		histogram[i] = new double[binSize];
		memset(histogram[i], 0, sizeof(double)*binSize);
	}


	for (j = 0; j < blkHeight; j++)
	{
		for (i = 0; i < blkWidth; i++)
		{
			startM = j * (Height / blkHeight);
			endM = (j + 1) * (Height / blkHeight);
			startN = i * (Width / blkWidth);
			endN = (i + 1) * (Width / blkWidth);

		
			for (m = startM; m < endM; m++) {
				for (n = startN; n < endN; n++) {
					if (m == 0 || n == 0)
						continue;

					if (m == Height - marginY - 1 || n == Width - marginX - 1)
						continue;

					if (ptr[m][n + 1] - ptr[m][n - 1] >= threshold)										// 0
						histogram[j*blkHeight + i][0] += abs(ptr[m][n + 1] - ptr[m][n - 1]);
					else if (ptr[m][n - 1] - ptr[m][n + 1] >= threshold)									// 4
						histogram[j*blkHeight + i][4] += abs(ptr[m][n + 1] - ptr[m][n - 1]);

					if (ptr[m + 1][n + 1] - ptr[m - 1][n - 1] >= threshold)									// 1
						histogram[j*blkHeight + i][1] += abs(ptr[m + 1][n + 1] - ptr[m - 1][n - 1]);
					else if (ptr[m - 1][n - 1] - ptr[m + 1][n + 1] >= threshold)								// 5
						histogram[j*blkHeight + i][5] += abs(ptr[m + 1][n + 1] - ptr[m - 1][n - 1]);

					if (ptr[m + 1][n] - ptr[m - 1][n] >= threshold)										// 2
						histogram[j*blkHeight + i][2] += abs(ptr[m + 1][n] - ptr[m - 1][n]);
					else if (ptr[m - 1][n] - ptr[m + 1][n] >= threshold)									// 6
						histogram[j*blkHeight + i][6] += abs(ptr[m + 1][n] - ptr[m - 1][n]);

					if (ptr[m + 1][n - 1] - ptr[m - 1][n + 1] >= threshold)									// 3
						histogram[j*blkHeight + i][3] += abs(ptr[m + 1][n - 1] - ptr[m - 1][n + 1]);
					else if (ptr[m - 1][n + 1] - ptr[m + 1][n - 1] >= threshold)								// 7
						histogram[j*blkHeight + i][7] += abs(ptr[m + 1][n - 1] - ptr[m - 1][n + 1]);
				}
			}
			
		}
	}

	for (i = 0; i < sub_region; i++) {
		for (j = 0; j < binSize; j++)
			feature[i * binSize + j] = histogram[i][j];
	}

	// 2014-02-04 JMR L2 Normalize 변경
	//if (accumType == OCSLBP_ACCUM_DIST)
		L2Normalization(feature, featureSize);
	//MinMaxNormalizationForDouble(feature, featureSize);



	// Release memory
	for (i = 0; i < sub_region; i++)
		delete[] histogram[i];
	delete[] histogram;
}

void CDetection::ExtractWaveletFeature(CDib& dib) {

	int i, j, k, l, itr, ii, jj;

	CDib tmp;
	BYTE** ori = dib.GetPtr();


	int image_height = 0;
	int image_width = 0;
	int size = 16;

	for (itr = 1; itr < DetectionInfo.cnt; itr++) {


		image_height = DetectionInfo.rg[itr].h;
		image_width = DetectionInfo.rg[itr].w;
		Trace(_T("w : %d / h : %d\n"), image_width, image_height);
		tmp.CreateGrayImage(image_width, image_height, 0);


		BYTE** ptr = tmp.GetPtr();

		for (i = 0, ii = DetectionInfo.rg[itr].min_y; i < image_height, ii<DetectionInfo.rg[itr].max_y; i++, ii++) {
			for (j = 0, jj = DetectionInfo.rg[itr].min_x; j < image_width, jj < DetectionInfo.rg[itr].max_x; j++, jj++) {
				ptr[i][j] = ori[ii][jj];
			}
		}

		//BYTE **tmpLH = new BYTE*[image_height];
		//for (i = 0; i < image_height; i++) tmpLH[i] = new BYTE[image_width];
		//BYTE **tmpHL = new BYTE*[image_height];
		//for (i = 0; i < image_height; i++) tmpHL[i] = new BYTE[image_width];
		BYTE **tmpHH = new BYTE*[size];
		for (i = 0; i < size; i++) tmpHH[i] = new BYTE[size];
		

		//for (i = 0; i < image_height; i++)
		//	for (j = 0; j < image_width; j++) {
		//		tmpLH[i][j] = tmpHL[i][j] = tmpHH[i][j] = 0;
		//	}

		Wavelet.Daub4b(tmp ,itr, image_height, image_width);

		ptr = tmp.GetPtr();

		/*for (i = 0; i < image_height / 2; i++){
			for (j = 0; j < image_width / 2; j++){
				tmpLH[i * 2][j * 2] = tmpLH[i * 2][j * 2 + 1] = tmpLH[i * 2 + 1][j * 2] = tmpLH[i * 2 + 1][j * 2 + 1] = ptr[i][j + image_width / 2];
				tmpHL[i * 2][j * 2] = tmpHL[i * 2][j * 2 + 1] = tmpHL[i * 2 + 1][j * 2] = tmpHL[i * 2 + 1][j * 2 + 1] = ptr[i + image_height / 2][j];
				tmpHH[i * 2][j * 2] = tmpHH[i * 2][j * 2 + 1] = tmpHH[i * 2 + 1][j * 2] = tmpHH[i * 2 + 1][j * 2 + 1] = ptr[i + image_height / 2][j + image_width / 2];
			}
		}*/

		for (i = 0; i < size; i++){
			for (j = 0; j < size; j++){
				tmpHH[i][j] = ptr[i + size][j + size];
			}
		}

		DetectionInfo.rg[itr].feature.wavelet.size = OCSLBP_SUBBLOCK_SIZE * OCSLBP_SUBBLOCK_SIZE * OCSLBP_BIN_SIZE;
		DetectionInfo.rg[itr].feature.wavelet.OCSLBP_wavelet = new double[DetectionInfo.rg[itr].feature.wavelet.size];
		/*GetOCSLBPFeatureFromImage(tmpHH, image_height, image_width, DetectionInfo.rg[itr].feature.wavelet.OCSLBP_wavelet, DetectionInfo.rg[itr].feature.wavelet.size,
			OCSLBP_SUBBLOCK_SIZE, OCSLBP_SUBBLOCK_SIZE, OCSLBP_BIN_SIZE, 6);*/

		GetOCSLBPFeatureFromImage(tmpHH, 16, 16, DetectionInfo.rg[itr].feature.wavelet.OCSLBP_wavelet, DetectionInfo.rg[itr].feature.wavelet.size,
			OCSLBP_SUBBLOCK_SIZE, OCSLBP_SUBBLOCK_SIZE, OCSLBP_BIN_SIZE, 6);

		for (i = 0; i < size; i++) {
			delete[] tmpHH[i];
		}

		delete[] tmpHH;


		//for (i = 0; i < image_height; i++) {
		//	delete[] tmpLH[i];
		//	delete[] tmpHL[i];
		//	delete[] tmpHH[i];
		//}

		//delete[] tmpLH;
		//delete[] tmpHL;
		//delete[] tmpHH;
	}

	


}


void CDetection::RFWaveletClassification(CDib& dib) 
{
	double prob = 0.0f;
	double* result = new double[CLASSES_TESTING];
	memset(result, 0, sizeof(double)*CLASSES_TESTING);

	double* result_haar = new double[CLASSES_TESTING];
	memset(result_haar, 0, sizeof(double)*CLASSES_TESTING);


	double* result_tail = new double[CLASSES_TESTING];
	memset(result_tail, 0, sizeof(double)*CLASSES_TESTING);


	double* result_color_histo = new double[CLASSES_TESTING];
	memset(result_color_histo, 0, sizeof(double)*CLASSES_TESTING);

	//int size = COLOR_FEATURE_SIZE + WAVELET_FEATURE_SIZE;
	
	int size = WAVELET_FEATURE_SIZE;

	double* feature = new double[size];

	double RednessRes = 0;

	double* feature_haar = new double[60];
	
	double* feature_color_histo = new double[768];

	int x, y;
	
	for (int i = 1; i < DetectionInfo.cnt; i++) {

		//feature[0] = DetectionInfo.rg[i].feature.color.mean_R;
		//feature[1] = DetectionInfo.rg[i].feature.color.mean_G;
		//feature[2] = DetectionInfo.rg[i].feature.color.mean_B;
		//feature[3] = DetectionInfo.rg[i].feature.color.variance_R;
		//feature[4] = DetectionInfo.rg[i].feature.color.variance_G;
		//feature[5] = DetectionInfo.rg[i].feature.color.variance_B;

		//for (x = COLOR_FEATURE_SIZE, y=0; x < size, y<WAVELET_FEATURE_SIZE; x++, y++) {
		//	feature[x] = DetectionInfo.rg[i].feature.wavelet.OCSLBP_wavelet[y];
		//}

		for (x = 0; x < WAVELET_FEATURE_SIZE; x++) {
			feature[x] = DetectionInfo.rg[i].feature.wavelet.OCSLBP_wavelet[x];
		}

		estimateProbabilityUsingRandomForests(RFInfo, feature, false, result);

		estimateProbabilityUsingRandomForests(RFInfo_first_tail, feature, false, result_tail);

		for (x = 0; x < 60; x++){
			feature_haar[x] = DetectionInfo.rg[i].feature.haar.HaarValue[x];
		}

		estimateProbabilityUsingRandomForests(RFInfo_first, feature_haar, false, result_haar);

		for (x = 0; x < 256; x++){
			feature_color_histo[x] = DetectionInfo.rg[i].feature.colorHisto.R[x];
			feature_color_histo[x + (256)] = DetectionInfo.rg[i].feature.colorHisto.G[x];
			feature_color_histo[x + (256 * 2)] = DetectionInfo.rg[i].feature.colorHisto.B[x];
		}

		estimateProbabilityUsingRandomForests(RFInfo_Color, feature_color_histo, false, result_color_histo);

		double temp = result[0];
		double temp_haar = result_haar[0];
		double temp_tail = result_tail[0];
		double temp_color_histo = result_color_histo[0];

		//if (temp > 0.65) {
		//	RednessRes = GetRedness(i, dib);
		//	Trace(_T("TAIL/HEAD Redness : %f\n"), RednessRes);

		//	if (RednessRes < REDNESS_TH) {
		//		DetectionInfo.rg[i].type = TAIL_LIGHT;
		//	}
		//	else {
		//		DetectionInfo.rg[i].type = HEAD_LIGHT;
		//	}
		//}
		//else {
		//	RednessRes = GetRedness(i, dib);
		//	Trace(_T("OTHERS Redness : %f\n"), RednessRes);
		//	DetectionInfo.rg[i].type = OTHERS;
		//}

		RednessRes = GetRedness(i, dib);
	
		
		if ( (RednessRes < REDNESS_TH)) { // tail light



			if (temp_haar > 0.55) {
			//if (temp_tail > 0.55) {
			//	Trace(_T("Redness : %f\n"), RednessRes);

				
				DetectionInfo.rg[i].type = TAIL_LIGHT;
				//DetectionInfo.rg[i].type = HEAD_LIGHT;
				//if (DetectionInfo.rg[i].type == TAIL_LIGHT) Trace(_T("Tail Redness : %f\n"), RednessRes);
			}
			else {
				//Trace(_T("temp : %f\n"), temp_tail);
				//Trace(_T("Red : %f\n"), RednessRes);
				DetectionInfo.rg[i].type = OTHERS;
			}
			//DetectionInfo.rg[i].type = TAIL_LIGHT;

		}
		else { // head / others

			//if (temp > 0.55) {
			if ( ( (temp_color_histo*0.5) + (temp*0.5) ) > 0.55){
				Trace(_T("HEAD LIGHT temp : %f\n"), temp);
				Trace(_T("HEAD LIGHT color temp : %f\n"), temp_color_histo);

				//Trace(_T("temp : %f\n"), temp);
				//Trace(_T("Head Red : %f\n"), RednessRes);
				DetectionInfo.rg[i].type = HEAD_LIGHT;
			}
			else {
				Trace(_T("OTHERS temp : %f\n"), temp);
				Trace(_T("OTHERS color temp : %f\n"), temp_color_histo);
				//Trace(_T("Other Redness : %f\n"), RednessRes);
				DetectionInfo.rg[i].type = OTHERS;
			}
		}

	}

	delete[] result;
	delete[] result_tail;
	delete[] feature;
	delete[] result_haar;
	delete[] feature_haar;

}

double CDetection::GetRedness(int num, CDib& dib) {

	int i, j;
	double sumR, sumG, sumB;
	sumR = sumG = sumB = 0;
	double redness = 0;

	RGBBYTE** ptr = dib.GetRGBPtr();

	//for (j = DetectionInfo.rg[num].min_y; j < DetectionInfo.rg[num].max_y; j++) {
	//	for (i = DetectionInfo.rg[num].min_x; i < DetectionInfo.rg[num].max_x; i++) {
	//		sumR += ptr[j][i].r;
	//		sumG += ptr[j][i].g;
	//		sumB += ptr[j][i].b;
	//	}
	//}

	//redness = (sumG + sumB) / (2 * sumR);

	for (j = DetectionInfo.rg[num].min_y; j < DetectionInfo.rg[num].max_y; j++) {
		for (i = DetectionInfo.rg[num].min_x; i < DetectionInfo.rg[num].max_x; i++) {
			double temp = (ptr[j][i].r + ptr[j][i].g + ptr[j][i].b);
			if (temp != 0) {
				sumR = sumR + (ptr[j][i].r / temp);
				sumG = sumG + (ptr[j][i].g / temp);
				sumB = sumB + (ptr[j][i].b / temp);
			}
		}
	}

	if(sumR == 0) redness = 0;
	else redness = (sumG + sumB) / (2 * sumR);

	return redness;

}



void CDetection::MergeImage(CDib& dib1, CDib& dib2, CDib& result) {
	int w = dib1.GetWidth();
	int h = dib1.GetHeight();

	int i, j;

	result.CreateGrayImage(w, h, 0);

	BYTE** ptr1 = dib1.GetPtr();
	BYTE** ptr2 = dib2.GetPtr();
	
	BYTE** ptr_res = result.GetPtr();

	for (i = 0; i < h; i++) {
		for(j = 0; j < w; j++) {
			if (ptr1[i][j] == 255 || ptr2[i][j] == 255) {
				ptr_res[i][j] = 255;
			}
		}
	}
}






// Haar
void CDetection::generateIntegralImage(CDib& dib, double** integralValueForIntensity) {
	register int h, w, m;

	BYTE** ptr = dib.GetPtr();

	int imgW = dib.GetWidth();
	int imgH = dib.GetHeight();

	for (h = 0; h < imgH; h++)
	{
		for (w = 0; w < imgW; w++)
		{
			integralValueForIntensity[h][w] = ptr[h][w];

			// Intensity를 위한 Integral histogram
			if (h == 0 && w == 0)
				integralValueForIntensity[h][w] = integralValueForIntensity[h][w];
			else if (h == 0)
				integralValueForIntensity[h][w] = integralValueForIntensity[h][w - 1] + integralValueForIntensity[h][w];
			else if (w == 0)
				integralValueForIntensity[h][w] = integralValueForIntensity[h - 1][w] + integralValueForIntensity[h][w];
			else	// B + C - A + D(원래값)
				integralValueForIntensity[h][w] = integralValueForIntensity[h - 1][w] + integralValueForIntensity[h][w - 1] - integralValueForIntensity[h - 1][w - 1] + integralValueForIntensity[h][w];
		}
	}
}


void CDetection::GetHaarlikeFeatureOfWindow(CDib& dib, int labelNum) {
	//블럭 위치 계산을 위한 변수
	int left_b, right_b, top_b, bottom_b;
	left_b = right_b = top_b = bottom_b = 0;

	int count = 0;
	//double dImg_totalSum = 0.0;		// 입력이미지 전체 픽셀 합	
	double dTmp1 = 0.0;
	double dTmp2 = 0.0;
	double dTmpAvg1 = 0.0;
	double dTmpAvg2 = 0.0;

	DetectionInfo.rg[labelNum].feature.haar.HaarValue = new double[60];
	memset(DetectionInfo.rg[labelNum].feature.haar.HaarValue, 0, sizeof(double) * 60);
	DetectionInfo.rg[labelNum].feature.haar.size = 0;

	//블록별 합
	//int* block_sum = new int[mnHaarlike_blk_width *mnHaarlike_blk_height];
	int* block_sum = new int[8 * 8];
	memset(block_sum, 0, sizeof(int)*8 *8);

	int block_width = dib.GetWidth();
	block_width /= 8;
	int block_height = dib.GetHeight();
	block_height /= 8;

	RGBBYTE** ptr = dib.GetRGBPtr();

	int widthOfBlock, heightOfBlock;
	//widthOfBlock = mnWin_size_x / 8;
	//heightOfBlock = mnWin_size_y / 8;

	//블럭별 pixel value 추출
	//for (int i = 0; i < 8; i++)
	//{
	//	for (int j = 0; j < 8; j++)
	//	{
	//		left_b = (j * widthOfBlock) + start.x;
	//		right_b = left_b + widthOfBlock - 1;
	//		top_b = (i * heightOfBlock) + start.y;
	//		bottom_b = top_b + heightOfBlock - 1;


	//		if (left_b == 0 && top_b == 0)	// A = A  0번째
	//			block_sum[count] = (int)(integralValueForIntensity[bottom_b][right_b]);
	//		else if (top_b == 0)	//B - A 1번째
	//			block_sum[count] = (int)(integralValueForIntensity[bottom_b][right_b] - integralValueForIntensity[bottom_b][right_b - widthOfBlock]);
	//		else if (left_b == 0)	// C - A 2번째
	//			block_sum[count] = (int)(integralValueForIntensity[bottom_b][right_b] - integralValueForIntensity[bottom_b - heightOfBlock][right_b]);
	//		else	// D-B-C+A 4번째
	//			block_sum[count] = (int)(integralValueForIntensity[bottom_b][right_b] -
	//			integralValueForIntensity[bottom_b - heightOfBlock][right_b] -
	//			integralValueForIntensity[bottom_b][right_b - widthOfBlock] +
	//			integralValueForIntensity[bottom_b - heightOfBlock][right_b - widthOfBlock]);

	//		//dImg_totalSum += block_sum[count];
	//		count++;
	//	}
	//}

	for (int s = 0; s < 3; s++) {

		for (int i = 0; i < 8; i++) {
			for (int j = 0; j < 8; j++) {

				block_sum[i * 8 + j] = 0;

				for (int bh = (i*block_height); bh < (i*block_height + block_height); bh++) {
					for (int bw = (j*block_width); bw < (j*block_width + block_width); bw++) {

						if (s == 0) {
							block_sum[i * 8 + j] += ptr[bh][bw].r;
						}
						else if (s == 1) {
							block_sum[i * 8 + j] += ptr[bh][bw].g;
						}
						else {
							block_sum[i * 8 + j] += ptr[bh][bw].b;
						}
					}
				}

				block_sum[i * 8 + j] = block_sum[i * 8 + j] / (block_height*block_width);
			}
		}

		if (ComputeHaarLikeFeatureValue(labelNum, block_sum, s) < 0) {

		}
		//	Trace(_T("[%s,%d] extract haarlike failed\n"), _function_, _line_);


	}

	//L2Normalization(DetectionInfo.rg[labelNum].feature.haar.HaarValue, 20);
	//MinMaxNormalizationForDouble(feature, featureSize);

	delete[] block_sum;
}




int CDetection::ComputeHaarLikeFeatureValue(int labelNum, int* block_sum, int num)
{
	int ret = -1;

	if (block_sum == nullptr) {
		Trace(_T("[%s,%d] block_sum nullptr\n"), _function_, _line_);
		return ret;
	}

	//add vector by jwgim 150319
	std::vector<int>positiveIdxList;
	std::vector<int>negativeIdxList;
	int idx = num * 20;


	positiveIdxList.clear();
	negativeIdxList.clear();

	// 1
	negativeIdxList.push_back(0); negativeIdxList.push_back(1); negativeIdxList.push_back(2);
	negativeIdxList.push_back(3); positiveIdxList.push_back(4); positiveIdxList.push_back(5);
	positiveIdxList.push_back(6); positiveIdxList.push_back(7); negativeIdxList.push_back(8);
	negativeIdxList.push_back(9); negativeIdxList.push_back(10); negativeIdxList.push_back(11);
	positiveIdxList.push_back(12); positiveIdxList.push_back(13); positiveIdxList.push_back(14);
	positiveIdxList.push_back(15); negativeIdxList.push_back(16); negativeIdxList.push_back(17);
	negativeIdxList.push_back(18); negativeIdxList.push_back(19); positiveIdxList.push_back(20);
	positiveIdxList.push_back(21); positiveIdxList.push_back(22); positiveIdxList.push_back(23);
	negativeIdxList.push_back(24); negativeIdxList.push_back(25); negativeIdxList.push_back(26);
	negativeIdxList.push_back(27); positiveIdxList.push_back(28); positiveIdxList.push_back(29);
	positiveIdxList.push_back(30); positiveIdxList.push_back(31); negativeIdxList.push_back(32);
	negativeIdxList.push_back(33); negativeIdxList.push_back(34); negativeIdxList.push_back(35);
	positiveIdxList.push_back(36); positiveIdxList.push_back(37); positiveIdxList.push_back(38);
	positiveIdxList.push_back(39); negativeIdxList.push_back(40); negativeIdxList.push_back(41);
	negativeIdxList.push_back(42); negativeIdxList.push_back(43); positiveIdxList.push_back(44);
	positiveIdxList.push_back(45); positiveIdxList.push_back(46); positiveIdxList.push_back(47);
	negativeIdxList.push_back(48); negativeIdxList.push_back(49); negativeIdxList.push_back(50);
	negativeIdxList.push_back(51); positiveIdxList.push_back(52); positiveIdxList.push_back(53);
	positiveIdxList.push_back(54); positiveIdxList.push_back(55); negativeIdxList.push_back(56);
	negativeIdxList.push_back(57); negativeIdxList.push_back(58); negativeIdxList.push_back(59);
	positiveIdxList.push_back(60); positiveIdxList.push_back(61); positiveIdxList.push_back(62);
	positiveIdxList.push_back(63);
	DetectionInfo.rg[labelNum].feature.haar.HaarValue[idx++] = getHaarLikePatternValue(block_sum, &positiveIdxList, &negativeIdxList);
	

	// 2
	positiveIdxList.push_back(0); positiveIdxList.push_back(1); positiveIdxList.push_back(2);
	positiveIdxList.push_back(3); positiveIdxList.push_back(4); positiveIdxList.push_back(5);
	positiveIdxList.push_back(6); positiveIdxList.push_back(7); positiveIdxList.push_back(8);
	positiveIdxList.push_back(9); positiveIdxList.push_back(10); positiveIdxList.push_back(11);
	positiveIdxList.push_back(12); positiveIdxList.push_back(13); positiveIdxList.push_back(14);
	positiveIdxList.push_back(15); positiveIdxList.push_back(16); positiveIdxList.push_back(17);
	positiveIdxList.push_back(18); positiveIdxList.push_back(19); positiveIdxList.push_back(20);
	positiveIdxList.push_back(21); positiveIdxList.push_back(22); positiveIdxList.push_back(23);
	positiveIdxList.push_back(24); positiveIdxList.push_back(25); positiveIdxList.push_back(26);
	positiveIdxList.push_back(27); positiveIdxList.push_back(28); positiveIdxList.push_back(29);
	positiveIdxList.push_back(30); positiveIdxList.push_back(31); negativeIdxList.push_back(32);
	negativeIdxList.push_back(33); negativeIdxList.push_back(34); negativeIdxList.push_back(35);
	negativeIdxList.push_back(36); negativeIdxList.push_back(37); negativeIdxList.push_back(38);
	negativeIdxList.push_back(39); negativeIdxList.push_back(40); negativeIdxList.push_back(41);
	negativeIdxList.push_back(42); negativeIdxList.push_back(43); negativeIdxList.push_back(44);
	negativeIdxList.push_back(45); negativeIdxList.push_back(46); negativeIdxList.push_back(47);
	negativeIdxList.push_back(48); negativeIdxList.push_back(49); negativeIdxList.push_back(50);
	negativeIdxList.push_back(51); negativeIdxList.push_back(52); negativeIdxList.push_back(53);
	negativeIdxList.push_back(54); negativeIdxList.push_back(55); negativeIdxList.push_back(56);
	negativeIdxList.push_back(57); negativeIdxList.push_back(58); negativeIdxList.push_back(59);
	negativeIdxList.push_back(60); negativeIdxList.push_back(61); negativeIdxList.push_back(62);
	negativeIdxList.push_back(63);
	DetectionInfo.rg[labelNum].feature.haar.HaarValue[idx++] = getHaarLikePatternValue(block_sum, &positiveIdxList, &negativeIdxList);

	// 3
	/*negativeIdxList.push_back(0); negativeIdxList.push_back(1); negativeIdxList.push_back(2);
	negativeIdxList.push_back(3); negativeIdxList.push_back(4); negativeIdxList.push_back(5);
	positiveIdxList.push_back(6); positiveIdxList.push_back(7); negativeIdxList.push_back(8);
	negativeIdxList.push_back(9); negativeIdxList.push_back(10); negativeIdxList.push_back(11);
	negativeIdxList.push_back(12); negativeIdxList.push_back(13); positiveIdxList.push_back(14);
	positiveIdxList.push_back(15); negativeIdxList.push_back(16); negativeIdxList.push_back(17);
	negativeIdxList.push_back(18); negativeIdxList.push_back(19); negativeIdxList.push_back(20);
	negativeIdxList.push_back(21); positiveIdxList.push_back(22); positiveIdxList.push_back(23);
	negativeIdxList.push_back(24); negativeIdxList.push_back(25); negativeIdxList.push_back(26);
	negativeIdxList.push_back(27); negativeIdxList.push_back(28); negativeIdxList.push_back(29);
	positiveIdxList.push_back(30); positiveIdxList.push_back(31); negativeIdxList.push_back(32);
	negativeIdxList.push_back(33); negativeIdxList.push_back(34); negativeIdxList.push_back(35);
	negativeIdxList.push_back(36); negativeIdxList.push_back(37); positiveIdxList.push_back(38);
	positiveIdxList.push_back(39); negativeIdxList.push_back(40); negativeIdxList.push_back(41);
	negativeIdxList.push_back(42); negativeIdxList.push_back(43); negativeIdxList.push_back(44);
	negativeIdxList.push_back(45); positiveIdxList.push_back(46); positiveIdxList.push_back(47);
	negativeIdxList.push_back(48); negativeIdxList.push_back(49); negativeIdxList.push_back(50);
	negativeIdxList.push_back(51); negativeIdxList.push_back(52); negativeIdxList.push_back(53);
	positiveIdxList.push_back(54); positiveIdxList.push_back(55); negativeIdxList.push_back(56);
	negativeIdxList.push_back(57); negativeIdxList.push_back(58); negativeIdxList.push_back(59);
	negativeIdxList.push_back(60); negativeIdxList.push_back(61); positiveIdxList.push_back(62);
	positiveIdxList.push_back(63);
	DetectionInfo.rg[labelNum].feature.haar.HaarValue[idx++] = getHaarLikePatternValue(block_sum, &positiveIdxList, &negativeIdxList);*/

	// 4
	/*positiveIdxList.push_back(0); positiveIdxList.push_back(1); negativeIdxList.push_back(2);
	negativeIdxList.push_back(3); negativeIdxList.push_back(4); negativeIdxList.push_back(5);
	negativeIdxList.push_back(6); negativeIdxList.push_back(7); positiveIdxList.push_back(8);
	positiveIdxList.push_back(9); negativeIdxList.push_back(10); negativeIdxList.push_back(11);
	negativeIdxList.push_back(12); negativeIdxList.push_back(13); negativeIdxList.push_back(14);
	negativeIdxList.push_back(15); positiveIdxList.push_back(16); positiveIdxList.push_back(17);
	negativeIdxList.push_back(18); negativeIdxList.push_back(19); negativeIdxList.push_back(20);
	negativeIdxList.push_back(21); negativeIdxList.push_back(22); negativeIdxList.push_back(23);
	positiveIdxList.push_back(24); positiveIdxList.push_back(25); negativeIdxList.push_back(26);
	negativeIdxList.push_back(27); negativeIdxList.push_back(28); negativeIdxList.push_back(29);
	negativeIdxList.push_back(30); negativeIdxList.push_back(31); positiveIdxList.push_back(32);
	positiveIdxList.push_back(33); negativeIdxList.push_back(34); negativeIdxList.push_back(35);
	negativeIdxList.push_back(36); negativeIdxList.push_back(37); negativeIdxList.push_back(38);
	negativeIdxList.push_back(39); positiveIdxList.push_back(40); positiveIdxList.push_back(41);
	negativeIdxList.push_back(42); negativeIdxList.push_back(43); negativeIdxList.push_back(44);
	negativeIdxList.push_back(45); negativeIdxList.push_back(46); negativeIdxList.push_back(47);
	positiveIdxList.push_back(48); positiveIdxList.push_back(49); negativeIdxList.push_back(50);
	negativeIdxList.push_back(51); negativeIdxList.push_back(52); negativeIdxList.push_back(53);
	negativeIdxList.push_back(54); negativeIdxList.push_back(55); positiveIdxList.push_back(56);
	positiveIdxList.push_back(57); negativeIdxList.push_back(58); negativeIdxList.push_back(59);
	negativeIdxList.push_back(60); negativeIdxList.push_back(61); negativeIdxList.push_back(62);
	negativeIdxList.push_back(63);
	DetectionInfo.rg[labelNum].feature.haar.HaarValue[idx++] = getHaarLikePatternValue(block_sum, &positiveIdxList, &negativeIdxList);*/

	// 5
	//negativeIdxList.push_back(0); negativeIdxList.push_back(1); negativeIdxList.push_back(2);
	//negativeIdxList.push_back(3); negativeIdxList.push_back(4); negativeIdxList.push_back(5);
	//negativeIdxList.push_back(6); negativeIdxList.push_back(7); negativeIdxList.push_back(8);
	//negativeIdxList.push_back(9); negativeIdxList.push_back(10); negativeIdxList.push_back(11);
	//negativeIdxList.push_back(12); negativeIdxList.push_back(13); negativeIdxList.push_back(14);
	//negativeIdxList.push_back(15); negativeIdxList.push_back(16); negativeIdxList.push_back(17);
	//negativeIdxList.push_back(18); negativeIdxList.push_back(19); negativeIdxList.push_back(20);
	//negativeIdxList.push_back(21); negativeIdxList.push_back(22); negativeIdxList.push_back(23);
	//negativeIdxList.push_back(24); negativeIdxList.push_back(25); negativeIdxList.push_back(26);
	//negativeIdxList.push_back(27); negativeIdxList.push_back(28); negativeIdxList.push_back(29);
	//negativeIdxList.push_back(30); negativeIdxList.push_back(31); negativeIdxList.push_back(32);
	//negativeIdxList.push_back(33); negativeIdxList.push_back(34); negativeIdxList.push_back(35);
	//negativeIdxList.push_back(36); negativeIdxList.push_back(37); negativeIdxList.push_back(38);
	//negativeIdxList.push_back(39); negativeIdxList.push_back(40); negativeIdxList.push_back(41);
	//negativeIdxList.push_back(42); negativeIdxList.push_back(43); negativeIdxList.push_back(44);
	//negativeIdxList.push_back(45); negativeIdxList.push_back(46); negativeIdxList.push_back(47);
	//positiveIdxList.push_back(48); positiveIdxList.push_back(49); positiveIdxList.push_back(50);
	//positiveIdxList.push_back(51); positiveIdxList.push_back(52); positiveIdxList.push_back(53);
	//positiveIdxList.push_back(54); positiveIdxList.push_back(55); positiveIdxList.push_back(56);
	//positiveIdxList.push_back(57); positiveIdxList.push_back(58); positiveIdxList.push_back(59);
	//positiveIdxList.push_back(60); positiveIdxList.push_back(61); positiveIdxList.push_back(62);
	//positiveIdxList.push_back(63);
	//DetectionInfo.rg[labelNum].feature.haar.HaarValue[idx++] = getHaarLikePatternValue(block_sum, &positiveIdxList, &negativeIdxList);

	// 6
	//positiveIdxList.push_back(0); positiveIdxList.push_back(1); positiveIdxList.push_back(2);
	//positiveIdxList.push_back(3); positiveIdxList.push_back(4); positiveIdxList.push_back(5);
	//positiveIdxList.push_back(6); positiveIdxList.push_back(7); positiveIdxList.push_back(8);
	//positiveIdxList.push_back(9); positiveIdxList.push_back(10); positiveIdxList.push_back(11);
	//positiveIdxList.push_back(12); positiveIdxList.push_back(13); positiveIdxList.push_back(14);
	//positiveIdxList.push_back(15); negativeIdxList.push_back(16); negativeIdxList.push_back(17);
	//negativeIdxList.push_back(18); negativeIdxList.push_back(19); negativeIdxList.push_back(20);
	//negativeIdxList.push_back(21); negativeIdxList.push_back(22); negativeIdxList.push_back(23);
	//negativeIdxList.push_back(24); negativeIdxList.push_back(25); negativeIdxList.push_back(26);
	//negativeIdxList.push_back(27); negativeIdxList.push_back(28); negativeIdxList.push_back(29);
	//negativeIdxList.push_back(30); negativeIdxList.push_back(31); negativeIdxList.push_back(32);
	//negativeIdxList.push_back(33); negativeIdxList.push_back(34); negativeIdxList.push_back(35);
	//negativeIdxList.push_back(36); negativeIdxList.push_back(37); negativeIdxList.push_back(38);
	//negativeIdxList.push_back(39); negativeIdxList.push_back(40); negativeIdxList.push_back(41);
	//negativeIdxList.push_back(42); negativeIdxList.push_back(43); negativeIdxList.push_back(44);
	//negativeIdxList.push_back(45); negativeIdxList.push_back(46); negativeIdxList.push_back(47);
	//negativeIdxList.push_back(48); negativeIdxList.push_back(49); negativeIdxList.push_back(50);
	//negativeIdxList.push_back(51); negativeIdxList.push_back(52); negativeIdxList.push_back(53);
	//negativeIdxList.push_back(54); negativeIdxList.push_back(55); negativeIdxList.push_back(56);
	//negativeIdxList.push_back(57); negativeIdxList.push_back(58); negativeIdxList.push_back(59);
	//negativeIdxList.push_back(60); negativeIdxList.push_back(61); negativeIdxList.push_back(62);
	//negativeIdxList.push_back(63);
	//DetectionInfo.rg[labelNum].feature.haar.HaarValue[idx++] = getHaarLikePatternValue(block_sum, &positiveIdxList, &negativeIdxList);

	// 7
	positiveIdxList.push_back(0); positiveIdxList.push_back(1); positiveIdxList.push_back(2);
	positiveIdxList.push_back(3); positiveIdxList.push_back(4); positiveIdxList.push_back(5);
	positiveIdxList.push_back(6); positiveIdxList.push_back(7); positiveIdxList.push_back(8);
	positiveIdxList.push_back(9); positiveIdxList.push_back(10); positiveIdxList.push_back(11);
	positiveIdxList.push_back(12); positiveIdxList.push_back(13); positiveIdxList.push_back(14);
	positiveIdxList.push_back(15); positiveIdxList.push_back(16); positiveIdxList.push_back(17);
	negativeIdxList.push_back(18); negativeIdxList.push_back(19); negativeIdxList.push_back(20);
	negativeIdxList.push_back(21); positiveIdxList.push_back(22); positiveIdxList.push_back(23);
	positiveIdxList.push_back(24); positiveIdxList.push_back(25); negativeIdxList.push_back(26);
	negativeIdxList.push_back(27); negativeIdxList.push_back(28); negativeIdxList.push_back(29);
	positiveIdxList.push_back(30); positiveIdxList.push_back(31); positiveIdxList.push_back(32);
	positiveIdxList.push_back(33); negativeIdxList.push_back(34); negativeIdxList.push_back(35);
	negativeIdxList.push_back(36); negativeIdxList.push_back(37); positiveIdxList.push_back(38);
	positiveIdxList.push_back(39); positiveIdxList.push_back(40); positiveIdxList.push_back(41);
	negativeIdxList.push_back(42); negativeIdxList.push_back(43); negativeIdxList.push_back(44);
	negativeIdxList.push_back(45); positiveIdxList.push_back(46); positiveIdxList.push_back(47);
	positiveIdxList.push_back(48); positiveIdxList.push_back(49); positiveIdxList.push_back(50);
	positiveIdxList.push_back(51); positiveIdxList.push_back(52); positiveIdxList.push_back(53);
	positiveIdxList.push_back(54); positiveIdxList.push_back(55); positiveIdxList.push_back(56);
	positiveIdxList.push_back(57); positiveIdxList.push_back(58); positiveIdxList.push_back(59);
	positiveIdxList.push_back(60); positiveIdxList.push_back(61); positiveIdxList.push_back(62);
	positiveIdxList.push_back(63);
	DetectionInfo.rg[labelNum].feature.haar.HaarValue[idx++] = getHaarLikePatternValue(block_sum, &positiveIdxList, &negativeIdxList);

	// 8
	//positiveIdxList.push_back(0); negativeIdxList.push_back(1); negativeIdxList.push_back(2);
	//negativeIdxList.push_back(3); negativeIdxList.push_back(4); negativeIdxList.push_back(5);
	//negativeIdxList.push_back(6); positiveIdxList.push_back(7); positiveIdxList.push_back(8);
	//negativeIdxList.push_back(9); negativeIdxList.push_back(10); negativeIdxList.push_back(11);
	//negativeIdxList.push_back(12); negativeIdxList.push_back(13); negativeIdxList.push_back(14);
	//positiveIdxList.push_back(15); positiveIdxList.push_back(16); negativeIdxList.push_back(17);
	//negativeIdxList.push_back(18); negativeIdxList.push_back(19); negativeIdxList.push_back(20);
	//negativeIdxList.push_back(21); negativeIdxList.push_back(22); positiveIdxList.push_back(23);
	//positiveIdxList.push_back(24); negativeIdxList.push_back(25); negativeIdxList.push_back(26);
	//negativeIdxList.push_back(27); negativeIdxList.push_back(28); negativeIdxList.push_back(29);
	//negativeIdxList.push_back(30); positiveIdxList.push_back(31); positiveIdxList.push_back(32);
	//negativeIdxList.push_back(33); negativeIdxList.push_back(34); negativeIdxList.push_back(35);
	//negativeIdxList.push_back(36); negativeIdxList.push_back(37); negativeIdxList.push_back(38);
	//positiveIdxList.push_back(39); positiveIdxList.push_back(40); negativeIdxList.push_back(41);
	//negativeIdxList.push_back(42); negativeIdxList.push_back(43); negativeIdxList.push_back(44);
	//negativeIdxList.push_back(45); negativeIdxList.push_back(46); positiveIdxList.push_back(47);
	//positiveIdxList.push_back(48); negativeIdxList.push_back(49); negativeIdxList.push_back(50);
	//negativeIdxList.push_back(51); negativeIdxList.push_back(52); negativeIdxList.push_back(53);
	//negativeIdxList.push_back(54); positiveIdxList.push_back(55); positiveIdxList.push_back(56);
	//negativeIdxList.push_back(57); negativeIdxList.push_back(58); negativeIdxList.push_back(59);
	//negativeIdxList.push_back(60); negativeIdxList.push_back(61); negativeIdxList.push_back(62);
	//positiveIdxList.push_back(63);
	//DetectionInfo.rg[labelNum].feature.haar.HaarValue[idx++] = getHaarLikePatternValue(block_sum, &positiveIdxList, &negativeIdxList);

	// 9
	positiveIdxList.push_back(0); positiveIdxList.push_back(1); negativeIdxList.push_back(2);
	negativeIdxList.push_back(3); negativeIdxList.push_back(4); negativeIdxList.push_back(5);
	positiveIdxList.push_back(6); positiveIdxList.push_back(7); positiveIdxList.push_back(8);
	positiveIdxList.push_back(9); negativeIdxList.push_back(10); negativeIdxList.push_back(11);
	negativeIdxList.push_back(12); negativeIdxList.push_back(13); positiveIdxList.push_back(14);
	positiveIdxList.push_back(15); positiveIdxList.push_back(16); positiveIdxList.push_back(17);
	negativeIdxList.push_back(18); negativeIdxList.push_back(19); negativeIdxList.push_back(20);
	negativeIdxList.push_back(21); positiveIdxList.push_back(22); positiveIdxList.push_back(23);
	positiveIdxList.push_back(24); positiveIdxList.push_back(25); negativeIdxList.push_back(26);
	negativeIdxList.push_back(27); negativeIdxList.push_back(28); negativeIdxList.push_back(29);
	positiveIdxList.push_back(30); positiveIdxList.push_back(31); positiveIdxList.push_back(32);
	positiveIdxList.push_back(33); negativeIdxList.push_back(34); negativeIdxList.push_back(35);
	negativeIdxList.push_back(36); negativeIdxList.push_back(37); positiveIdxList.push_back(38);
	positiveIdxList.push_back(39); positiveIdxList.push_back(40); positiveIdxList.push_back(41);
	negativeIdxList.push_back(42); negativeIdxList.push_back(43); negativeIdxList.push_back(44);
	negativeIdxList.push_back(45); positiveIdxList.push_back(46); positiveIdxList.push_back(47);
	positiveIdxList.push_back(48); positiveIdxList.push_back(49); negativeIdxList.push_back(50);
	negativeIdxList.push_back(51); negativeIdxList.push_back(52); negativeIdxList.push_back(53);
	positiveIdxList.push_back(54); positiveIdxList.push_back(55); positiveIdxList.push_back(56);
	positiveIdxList.push_back(57); negativeIdxList.push_back(58); negativeIdxList.push_back(59);
	negativeIdxList.push_back(60); negativeIdxList.push_back(61); positiveIdxList.push_back(62);
	positiveIdxList.push_back(63);
	DetectionInfo.rg[labelNum].feature.haar.HaarValue[idx++] = getHaarLikePatternValue(block_sum, &positiveIdxList, &negativeIdxList);

	// 10
	positiveIdxList.push_back(0); positiveIdxList.push_back(1); positiveIdxList.push_back(2);
	positiveIdxList.push_back(3); positiveIdxList.push_back(4); positiveIdxList.push_back(5);
	positiveIdxList.push_back(6); positiveIdxList.push_back(7); positiveIdxList.push_back(8);
	positiveIdxList.push_back(9); positiveIdxList.push_back(10); positiveIdxList.push_back(11);
	positiveIdxList.push_back(12); positiveIdxList.push_back(13); positiveIdxList.push_back(14);
	positiveIdxList.push_back(15); negativeIdxList.push_back(16); negativeIdxList.push_back(17);
	negativeIdxList.push_back(18); negativeIdxList.push_back(19); negativeIdxList.push_back(20);
	negativeIdxList.push_back(21); negativeIdxList.push_back(22); negativeIdxList.push_back(23);
	negativeIdxList.push_back(24); negativeIdxList.push_back(25); negativeIdxList.push_back(26);
	negativeIdxList.push_back(27); negativeIdxList.push_back(28); negativeIdxList.push_back(29);
	negativeIdxList.push_back(30); negativeIdxList.push_back(31); negativeIdxList.push_back(32);
	negativeIdxList.push_back(33); negativeIdxList.push_back(34); negativeIdxList.push_back(35);
	negativeIdxList.push_back(36); negativeIdxList.push_back(37); negativeIdxList.push_back(38);
	negativeIdxList.push_back(39); negativeIdxList.push_back(40); negativeIdxList.push_back(41);
	negativeIdxList.push_back(42); negativeIdxList.push_back(43); negativeIdxList.push_back(44);
	negativeIdxList.push_back(45); negativeIdxList.push_back(46); negativeIdxList.push_back(47);
	positiveIdxList.push_back(48); positiveIdxList.push_back(49); positiveIdxList.push_back(50);
	positiveIdxList.push_back(51); positiveIdxList.push_back(52); positiveIdxList.push_back(53);
	positiveIdxList.push_back(54); positiveIdxList.push_back(55); positiveIdxList.push_back(56);
	positiveIdxList.push_back(57); positiveIdxList.push_back(58); positiveIdxList.push_back(59);
	positiveIdxList.push_back(60); positiveIdxList.push_back(61); positiveIdxList.push_back(62);
	positiveIdxList.push_back(63);
	DetectionInfo.rg[labelNum].feature.haar.HaarValue[idx++] = getHaarLikePatternValue(block_sum, &positiveIdxList, &negativeIdxList);

	// 11
	//positiveIdxList.push_back(0); positiveIdxList.push_back(1); positiveIdxList.push_back(2);
	//positiveIdxList.push_back(3); positiveIdxList.push_back(4); positiveIdxList.push_back(5);
	//positiveIdxList.push_back(6); positiveIdxList.push_back(7); negativeIdxList.push_back(8);
	//negativeIdxList.push_back(9); negativeIdxList.push_back(10); negativeIdxList.push_back(11);
	//negativeIdxList.push_back(12); negativeIdxList.push_back(13); negativeIdxList.push_back(14);
	//negativeIdxList.push_back(15); negativeIdxList.push_back(16); negativeIdxList.push_back(17);
	//negativeIdxList.push_back(18); negativeIdxList.push_back(19); negativeIdxList.push_back(20);
	//negativeIdxList.push_back(21); negativeIdxList.push_back(22); negativeIdxList.push_back(23);
	//negativeIdxList.push_back(24); negativeIdxList.push_back(25); negativeIdxList.push_back(26);
	//negativeIdxList.push_back(27); negativeIdxList.push_back(28); negativeIdxList.push_back(29);
	//negativeIdxList.push_back(30); negativeIdxList.push_back(31); negativeIdxList.push_back(32);
	//negativeIdxList.push_back(33); negativeIdxList.push_back(34); negativeIdxList.push_back(35);
	//negativeIdxList.push_back(36); negativeIdxList.push_back(37); negativeIdxList.push_back(38);
	//negativeIdxList.push_back(39); negativeIdxList.push_back(40); negativeIdxList.push_back(41);
	//negativeIdxList.push_back(42); negativeIdxList.push_back(43); negativeIdxList.push_back(44);
	//negativeIdxList.push_back(45); negativeIdxList.push_back(46); negativeIdxList.push_back(47);
	//negativeIdxList.push_back(48); negativeIdxList.push_back(49); negativeIdxList.push_back(50);
	//negativeIdxList.push_back(51); negativeIdxList.push_back(52); negativeIdxList.push_back(53);
	//negativeIdxList.push_back(54); negativeIdxList.push_back(55); positiveIdxList.push_back(56);
	//positiveIdxList.push_back(57); positiveIdxList.push_back(58); positiveIdxList.push_back(59);
	//positiveIdxList.push_back(60); positiveIdxList.push_back(61); positiveIdxList.push_back(62);
	//positiveIdxList.push_back(63);
	//DetectionInfo.rg[labelNum].feature.haar.HaarValue[idx++] = getHaarLikePatternValue(block_sum, &positiveIdxList, &negativeIdxList);

	// 12
	positiveIdxList.push_back(0); positiveIdxList.push_back(1); negativeIdxList.push_back(2);
	negativeIdxList.push_back(3); positiveIdxList.push_back(4); positiveIdxList.push_back(5);
	negativeIdxList.push_back(6); negativeIdxList.push_back(7); positiveIdxList.push_back(8);
	positiveIdxList.push_back(9); negativeIdxList.push_back(10); negativeIdxList.push_back(11);
	positiveIdxList.push_back(12); positiveIdxList.push_back(13); negativeIdxList.push_back(14);
	negativeIdxList.push_back(15); positiveIdxList.push_back(16); positiveIdxList.push_back(17);
	negativeIdxList.push_back(18); negativeIdxList.push_back(19); positiveIdxList.push_back(20);
	positiveIdxList.push_back(21); negativeIdxList.push_back(22); negativeIdxList.push_back(23);
	positiveIdxList.push_back(24); positiveIdxList.push_back(25); negativeIdxList.push_back(26);
	negativeIdxList.push_back(27); positiveIdxList.push_back(28); positiveIdxList.push_back(29);
	negativeIdxList.push_back(30); negativeIdxList.push_back(31); positiveIdxList.push_back(32);
	positiveIdxList.push_back(33); negativeIdxList.push_back(34); negativeIdxList.push_back(35);
	positiveIdxList.push_back(36); positiveIdxList.push_back(37); negativeIdxList.push_back(38);
	negativeIdxList.push_back(39); positiveIdxList.push_back(40); positiveIdxList.push_back(41);
	negativeIdxList.push_back(42); negativeIdxList.push_back(43); positiveIdxList.push_back(44);
	positiveIdxList.push_back(45); negativeIdxList.push_back(46); negativeIdxList.push_back(47);
	positiveIdxList.push_back(48); positiveIdxList.push_back(49); negativeIdxList.push_back(50);
	negativeIdxList.push_back(51); positiveIdxList.push_back(52); positiveIdxList.push_back(53);
	negativeIdxList.push_back(54); negativeIdxList.push_back(55); positiveIdxList.push_back(56);
	positiveIdxList.push_back(57); negativeIdxList.push_back(58); negativeIdxList.push_back(59);
	positiveIdxList.push_back(60); positiveIdxList.push_back(61); negativeIdxList.push_back(62);
	negativeIdxList.push_back(63);
	DetectionInfo.rg[labelNum].feature.haar.HaarValue[idx++] = getHaarLikePatternValue(block_sum, &positiveIdxList, &negativeIdxList);

	// 13
	positiveIdxList.push_back(0); positiveIdxList.push_back(1); positiveIdxList.push_back(2);
	positiveIdxList.push_back(3); positiveIdxList.push_back(4); positiveIdxList.push_back(5);
	positiveIdxList.push_back(6); positiveIdxList.push_back(7); positiveIdxList.push_back(8);
	positiveIdxList.push_back(9); positiveIdxList.push_back(10); positiveIdxList.push_back(11);
	positiveIdxList.push_back(12); positiveIdxList.push_back(13); positiveIdxList.push_back(14);
	positiveIdxList.push_back(15); negativeIdxList.push_back(16); negativeIdxList.push_back(17);
	negativeIdxList.push_back(18); negativeIdxList.push_back(19); negativeIdxList.push_back(20);
	negativeIdxList.push_back(21); negativeIdxList.push_back(22); negativeIdxList.push_back(23);
	negativeIdxList.push_back(24); negativeIdxList.push_back(25); negativeIdxList.push_back(26);
	negativeIdxList.push_back(27); negativeIdxList.push_back(28); negativeIdxList.push_back(29);
	negativeIdxList.push_back(30); negativeIdxList.push_back(31); positiveIdxList.push_back(32);
	positiveIdxList.push_back(33); positiveIdxList.push_back(34); positiveIdxList.push_back(35);
	positiveIdxList.push_back(36); positiveIdxList.push_back(37); positiveIdxList.push_back(38);
	positiveIdxList.push_back(39); positiveIdxList.push_back(40); positiveIdxList.push_back(41);
	positiveIdxList.push_back(42); positiveIdxList.push_back(43); positiveIdxList.push_back(44);
	positiveIdxList.push_back(45); positiveIdxList.push_back(46); positiveIdxList.push_back(47);
	negativeIdxList.push_back(48); negativeIdxList.push_back(49); negativeIdxList.push_back(50);
	negativeIdxList.push_back(51); negativeIdxList.push_back(52); negativeIdxList.push_back(53);
	negativeIdxList.push_back(54); negativeIdxList.push_back(55); negativeIdxList.push_back(56);
	negativeIdxList.push_back(57); negativeIdxList.push_back(58); negativeIdxList.push_back(59);
	negativeIdxList.push_back(60); negativeIdxList.push_back(61); negativeIdxList.push_back(62);
	negativeIdxList.push_back(63);
	DetectionInfo.rg[labelNum].feature.haar.HaarValue[idx++] = getHaarLikePatternValue(block_sum, &positiveIdxList, &negativeIdxList);

	// 14
	negativeIdxList.push_back(0); negativeIdxList.push_back(1); negativeIdxList.push_back(2);
	negativeIdxList.push_back(3); positiveIdxList.push_back(4); positiveIdxList.push_back(5);
	positiveIdxList.push_back(6); positiveIdxList.push_back(7); negativeIdxList.push_back(8);
	negativeIdxList.push_back(9); negativeIdxList.push_back(10); negativeIdxList.push_back(11);
	positiveIdxList.push_back(12); positiveIdxList.push_back(13); positiveIdxList.push_back(14);
	positiveIdxList.push_back(15); positiveIdxList.push_back(16); positiveIdxList.push_back(17);
	positiveIdxList.push_back(18); positiveIdxList.push_back(19); negativeIdxList.push_back(20);
	negativeIdxList.push_back(21); negativeIdxList.push_back(22); negativeIdxList.push_back(23);
	positiveIdxList.push_back(24); positiveIdxList.push_back(25); positiveIdxList.push_back(26);
	positiveIdxList.push_back(27); negativeIdxList.push_back(28); negativeIdxList.push_back(29);
	negativeIdxList.push_back(30); negativeIdxList.push_back(31); positiveIdxList.push_back(32);
	positiveIdxList.push_back(33); positiveIdxList.push_back(34); positiveIdxList.push_back(35);
	negativeIdxList.push_back(36); negativeIdxList.push_back(37); negativeIdxList.push_back(38);
	negativeIdxList.push_back(39); positiveIdxList.push_back(40); positiveIdxList.push_back(41);
	positiveIdxList.push_back(42); positiveIdxList.push_back(43); negativeIdxList.push_back(44);
	negativeIdxList.push_back(45); negativeIdxList.push_back(46); negativeIdxList.push_back(47);
	positiveIdxList.push_back(48); positiveIdxList.push_back(49); positiveIdxList.push_back(50);
	positiveIdxList.push_back(51); negativeIdxList.push_back(52); negativeIdxList.push_back(53);
	negativeIdxList.push_back(54); negativeIdxList.push_back(55); positiveIdxList.push_back(56);
	positiveIdxList.push_back(57); positiveIdxList.push_back(58); positiveIdxList.push_back(59);
	negativeIdxList.push_back(60); negativeIdxList.push_back(61); negativeIdxList.push_back(62);
	negativeIdxList.push_back(63);
	DetectionInfo.rg[labelNum].feature.haar.HaarValue[idx++] = getHaarLikePatternValue(block_sum, &positiveIdxList, &negativeIdxList);

	// 15
	positiveIdxList.push_back(0); positiveIdxList.push_back(1); negativeIdxList.push_back(2);
	negativeIdxList.push_back(3); negativeIdxList.push_back(4); negativeIdxList.push_back(5);
	negativeIdxList.push_back(6); negativeIdxList.push_back(7); positiveIdxList.push_back(8);
	positiveIdxList.push_back(9); negativeIdxList.push_back(10); negativeIdxList.push_back(11);
	negativeIdxList.push_back(12); negativeIdxList.push_back(13); negativeIdxList.push_back(14);
	negativeIdxList.push_back(15); positiveIdxList.push_back(16); positiveIdxList.push_back(17);
	negativeIdxList.push_back(18); negativeIdxList.push_back(19); negativeIdxList.push_back(20);
	negativeIdxList.push_back(21); negativeIdxList.push_back(22); negativeIdxList.push_back(23);
	positiveIdxList.push_back(24); positiveIdxList.push_back(25); negativeIdxList.push_back(26);
	negativeIdxList.push_back(27); negativeIdxList.push_back(28); negativeIdxList.push_back(29);
	negativeIdxList.push_back(30); negativeIdxList.push_back(31); negativeIdxList.push_back(32);
	negativeIdxList.push_back(33); positiveIdxList.push_back(34); positiveIdxList.push_back(35);
	positiveIdxList.push_back(36); positiveIdxList.push_back(37); positiveIdxList.push_back(38);
	positiveIdxList.push_back(39); negativeIdxList.push_back(40); negativeIdxList.push_back(41);
	positiveIdxList.push_back(42); positiveIdxList.push_back(43); positiveIdxList.push_back(44);
	positiveIdxList.push_back(45); positiveIdxList.push_back(46); positiveIdxList.push_back(47);
	negativeIdxList.push_back(48); negativeIdxList.push_back(49); positiveIdxList.push_back(50);
	positiveIdxList.push_back(51); positiveIdxList.push_back(52); positiveIdxList.push_back(53);
	positiveIdxList.push_back(54); positiveIdxList.push_back(55); negativeIdxList.push_back(56);
	negativeIdxList.push_back(57); positiveIdxList.push_back(58); positiveIdxList.push_back(59);
	positiveIdxList.push_back(60); positiveIdxList.push_back(61); positiveIdxList.push_back(62);
	positiveIdxList.push_back(63);
	DetectionInfo.rg[labelNum].feature.haar.HaarValue[idx++] = getHaarLikePatternValue(block_sum, &positiveIdxList, &negativeIdxList);

	// 16
	negativeIdxList.push_back(0); negativeIdxList.push_back(1); negativeIdxList.push_back(2);
	negativeIdxList.push_back(3); negativeIdxList.push_back(4); negativeIdxList.push_back(5);
	positiveIdxList.push_back(6); positiveIdxList.push_back(7); negativeIdxList.push_back(8);
	negativeIdxList.push_back(9); negativeIdxList.push_back(10); negativeIdxList.push_back(11);
	negativeIdxList.push_back(12); negativeIdxList.push_back(13); positiveIdxList.push_back(14);
	positiveIdxList.push_back(15); negativeIdxList.push_back(16); negativeIdxList.push_back(17);
	negativeIdxList.push_back(18); negativeIdxList.push_back(19); negativeIdxList.push_back(20);
	negativeIdxList.push_back(21); positiveIdxList.push_back(22); positiveIdxList.push_back(23);
	negativeIdxList.push_back(24); negativeIdxList.push_back(25); negativeIdxList.push_back(26);
	negativeIdxList.push_back(27); negativeIdxList.push_back(28); negativeIdxList.push_back(29);
	positiveIdxList.push_back(30); positiveIdxList.push_back(31); positiveIdxList.push_back(32);
	positiveIdxList.push_back(33); positiveIdxList.push_back(34); positiveIdxList.push_back(35);
	positiveIdxList.push_back(36); positiveIdxList.push_back(37); negativeIdxList.push_back(38);
	negativeIdxList.push_back(39); positiveIdxList.push_back(40); positiveIdxList.push_back(41);
	positiveIdxList.push_back(42); positiveIdxList.push_back(43); positiveIdxList.push_back(44);
	positiveIdxList.push_back(45); negativeIdxList.push_back(46); negativeIdxList.push_back(47);
	positiveIdxList.push_back(48); positiveIdxList.push_back(49); positiveIdxList.push_back(50);
	positiveIdxList.push_back(51); positiveIdxList.push_back(52); positiveIdxList.push_back(53);
	negativeIdxList.push_back(54); negativeIdxList.push_back(55); positiveIdxList.push_back(56);
	positiveIdxList.push_back(57); positiveIdxList.push_back(58); positiveIdxList.push_back(59);
	positiveIdxList.push_back(60); positiveIdxList.push_back(61); negativeIdxList.push_back(62);
	negativeIdxList.push_back(63);
	DetectionInfo.rg[labelNum].feature.haar.HaarValue[idx++] = getHaarLikePatternValue(block_sum, &positiveIdxList, &negativeIdxList);

	// 17
	//positiveIdxList.push_back(0); positiveIdxList.push_back(1); positiveIdxList.push_back(2);
	//positiveIdxList.push_back(3); negativeIdxList.push_back(4); negativeIdxList.push_back(5);
	//negativeIdxList.push_back(6); negativeIdxList.push_back(7); positiveIdxList.push_back(8);
	//positiveIdxList.push_back(9); positiveIdxList.push_back(10); positiveIdxList.push_back(11);
	//negativeIdxList.push_back(12); negativeIdxList.push_back(13); negativeIdxList.push_back(14);
	//negativeIdxList.push_back(15); positiveIdxList.push_back(16); positiveIdxList.push_back(17);
	//positiveIdxList.push_back(18); positiveIdxList.push_back(19); negativeIdxList.push_back(20);
	//negativeIdxList.push_back(21); negativeIdxList.push_back(22); negativeIdxList.push_back(23);
	//positiveIdxList.push_back(24); positiveIdxList.push_back(25); positiveIdxList.push_back(26);
	//positiveIdxList.push_back(27); negativeIdxList.push_back(28); negativeIdxList.push_back(29);
	//negativeIdxList.push_back(30); negativeIdxList.push_back(31); positiveIdxList.push_back(32);
	//positiveIdxList.push_back(33); positiveIdxList.push_back(34); positiveIdxList.push_back(35);
	//negativeIdxList.push_back(36); negativeIdxList.push_back(37); negativeIdxList.push_back(38);
	//negativeIdxList.push_back(39); positiveIdxList.push_back(40); positiveIdxList.push_back(41);
	//positiveIdxList.push_back(42); positiveIdxList.push_back(43); negativeIdxList.push_back(44);
	//negativeIdxList.push_back(45); negativeIdxList.push_back(46); negativeIdxList.push_back(47);
	//negativeIdxList.push_back(48); negativeIdxList.push_back(49); negativeIdxList.push_back(50);
	//negativeIdxList.push_back(51); positiveIdxList.push_back(52); positiveIdxList.push_back(53);
	//positiveIdxList.push_back(54); positiveIdxList.push_back(55); negativeIdxList.push_back(56);
	//negativeIdxList.push_back(57); negativeIdxList.push_back(58); negativeIdxList.push_back(59);
	//positiveIdxList.push_back(60); positiveIdxList.push_back(61); positiveIdxList.push_back(62);
	//positiveIdxList.push_back(63);
	//DetectionInfo.rg[labelNum].feature.haar.HaarValue[idx++] = getHaarLikePatternValue(block_sum, &positiveIdxList, &negativeIdxList);

	// 18
	/*negativeIdxList.push_back(0); negativeIdxList.push_back(1); negativeIdxList.push_back(2);
	negativeIdxList.push_back(3); positiveIdxList.push_back(4); positiveIdxList.push_back(5);
	positiveIdxList.push_back(6); positiveIdxList.push_back(7); negativeIdxList.push_back(8);
	negativeIdxList.push_back(9); negativeIdxList.push_back(10); negativeIdxList.push_back(11);
	positiveIdxList.push_back(12); positiveIdxList.push_back(13); positiveIdxList.push_back(14);
	positiveIdxList.push_back(15); negativeIdxList.push_back(16); negativeIdxList.push_back(17);
	negativeIdxList.push_back(18); negativeIdxList.push_back(19); positiveIdxList.push_back(20);
	positiveIdxList.push_back(21); positiveIdxList.push_back(22); positiveIdxList.push_back(23);
	positiveIdxList.push_back(24); positiveIdxList.push_back(25); positiveIdxList.push_back(26);
	positiveIdxList.push_back(27); negativeIdxList.push_back(28); negativeIdxList.push_back(29);
	negativeIdxList.push_back(30); negativeIdxList.push_back(31); positiveIdxList.push_back(32);
	positiveIdxList.push_back(33); positiveIdxList.push_back(34); positiveIdxList.push_back(35);
	negativeIdxList.push_back(36); negativeIdxList.push_back(37); negativeIdxList.push_back(38);
	negativeIdxList.push_back(39); positiveIdxList.push_back(40); positiveIdxList.push_back(41);
	positiveIdxList.push_back(42); positiveIdxList.push_back(43); negativeIdxList.push_back(44);
	negativeIdxList.push_back(45); negativeIdxList.push_back(46); negativeIdxList.push_back(47);
	positiveIdxList.push_back(48); positiveIdxList.push_back(49); positiveIdxList.push_back(50);
	positiveIdxList.push_back(51); negativeIdxList.push_back(52); negativeIdxList.push_back(53);
	negativeIdxList.push_back(54); negativeIdxList.push_back(55); positiveIdxList.push_back(56);
	positiveIdxList.push_back(57); positiveIdxList.push_back(58); positiveIdxList.push_back(59);
	negativeIdxList.push_back(60); negativeIdxList.push_back(61); negativeIdxList.push_back(62);
	negativeIdxList.push_back(63);
	DetectionInfo.rg[labelNum].feature.haar.HaarValue[idx++] = getHaarLikePatternValue(block_sum, &positiveIdxList, &negativeIdxList);*/

	// 19
	positiveIdxList.push_back(0); positiveIdxList.push_back(1); positiveIdxList.push_back(2);
	positiveIdxList.push_back(3); negativeIdxList.push_back(4); negativeIdxList.push_back(5);
	negativeIdxList.push_back(6); negativeIdxList.push_back(7); positiveIdxList.push_back(8);
	positiveIdxList.push_back(9); positiveIdxList.push_back(10); positiveIdxList.push_back(11);
	negativeIdxList.push_back(12); negativeIdxList.push_back(13); negativeIdxList.push_back(14);
	negativeIdxList.push_back(15); positiveIdxList.push_back(16); positiveIdxList.push_back(17);
	positiveIdxList.push_back(18); positiveIdxList.push_back(19); negativeIdxList.push_back(20);
	negativeIdxList.push_back(21); negativeIdxList.push_back(22); negativeIdxList.push_back(23);
	positiveIdxList.push_back(24); positiveIdxList.push_back(25); positiveIdxList.push_back(26);
	positiveIdxList.push_back(27); negativeIdxList.push_back(28); negativeIdxList.push_back(29);
	negativeIdxList.push_back(30); negativeIdxList.push_back(31); positiveIdxList.push_back(32);
	positiveIdxList.push_back(33); positiveIdxList.push_back(34); positiveIdxList.push_back(35);
	negativeIdxList.push_back(36); negativeIdxList.push_back(37); negativeIdxList.push_back(38);
	negativeIdxList.push_back(39); negativeIdxList.push_back(40); negativeIdxList.push_back(41);
	negativeIdxList.push_back(42); negativeIdxList.push_back(43); positiveIdxList.push_back(44);
	positiveIdxList.push_back(45); positiveIdxList.push_back(46); positiveIdxList.push_back(47);
	negativeIdxList.push_back(48); negativeIdxList.push_back(49); negativeIdxList.push_back(50);
	negativeIdxList.push_back(51); positiveIdxList.push_back(52); positiveIdxList.push_back(53);
	positiveIdxList.push_back(54); positiveIdxList.push_back(55); negativeIdxList.push_back(56);
	negativeIdxList.push_back(57); negativeIdxList.push_back(58); negativeIdxList.push_back(59);
	positiveIdxList.push_back(60); positiveIdxList.push_back(61); positiveIdxList.push_back(62);
	positiveIdxList.push_back(63);
	DetectionInfo.rg[labelNum].feature.haar.HaarValue[idx++] = getHaarLikePatternValue(block_sum, &positiveIdxList, &negativeIdxList);

	// 20
	/*positiveIdxList.push_back(0); positiveIdxList.push_back(1); positiveIdxList.push_back(2);
	negativeIdxList.push_back(3); negativeIdxList.push_back(4); negativeIdxList.push_back(5);
	negativeIdxList.push_back(6); negativeIdxList.push_back(7); positiveIdxList.push_back(8);
	positiveIdxList.push_back(9); positiveIdxList.push_back(10); negativeIdxList.push_back(11);
	negativeIdxList.push_back(12); negativeIdxList.push_back(13); negativeIdxList.push_back(14);
	negativeIdxList.push_back(15); positiveIdxList.push_back(16); positiveIdxList.push_back(17);
	positiveIdxList.push_back(18); negativeIdxList.push_back(19); negativeIdxList.push_back(20);
	negativeIdxList.push_back(21); negativeIdxList.push_back(22); negativeIdxList.push_back(23);
	positiveIdxList.push_back(24); positiveIdxList.push_back(25); positiveIdxList.push_back(26);
	negativeIdxList.push_back(27); negativeIdxList.push_back(28); negativeIdxList.push_back(29);
	negativeIdxList.push_back(30); negativeIdxList.push_back(31); negativeIdxList.push_back(32);
	negativeIdxList.push_back(33); negativeIdxList.push_back(34); positiveIdxList.push_back(35);
	positiveIdxList.push_back(36); positiveIdxList.push_back(37); positiveIdxList.push_back(38);
	positiveIdxList.push_back(39); negativeIdxList.push_back(40); negativeIdxList.push_back(41);
	negativeIdxList.push_back(42); positiveIdxList.push_back(43); positiveIdxList.push_back(44);
	positiveIdxList.push_back(45); positiveIdxList.push_back(46); positiveIdxList.push_back(47);
	negativeIdxList.push_back(48); negativeIdxList.push_back(49); negativeIdxList.push_back(50);
	positiveIdxList.push_back(51); positiveIdxList.push_back(52); positiveIdxList.push_back(53);
	positiveIdxList.push_back(54); positiveIdxList.push_back(55); negativeIdxList.push_back(56);
	negativeIdxList.push_back(57); negativeIdxList.push_back(58); positiveIdxList.push_back(59);
	positiveIdxList.push_back(60); positiveIdxList.push_back(61); positiveIdxList.push_back(62);
	positiveIdxList.push_back(63);
	DetectionInfo.rg[labelNum].feature.haar.HaarValue[idx++] = getHaarLikePatternValue(block_sum, &positiveIdxList, &negativeIdxList);*/

	// 21
	/*negativeIdxList.push_back(0); negativeIdxList.push_back(1); negativeIdxList.push_back(2);
	negativeIdxList.push_back(3); negativeIdxList.push_back(4); positiveIdxList.push_back(5);
	positiveIdxList.push_back(6); positiveIdxList.push_back(7); negativeIdxList.push_back(8);
	negativeIdxList.push_back(9); negativeIdxList.push_back(10); negativeIdxList.push_back(11);
	negativeIdxList.push_back(12); positiveIdxList.push_back(13); positiveIdxList.push_back(14);
	positiveIdxList.push_back(15); negativeIdxList.push_back(16); negativeIdxList.push_back(17);
	negativeIdxList.push_back(18); negativeIdxList.push_back(19); negativeIdxList.push_back(20);
	positiveIdxList.push_back(21); positiveIdxList.push_back(22); positiveIdxList.push_back(23);
	negativeIdxList.push_back(24); negativeIdxList.push_back(25); negativeIdxList.push_back(26);
	negativeIdxList.push_back(27); negativeIdxList.push_back(28); positiveIdxList.push_back(29);
	positiveIdxList.push_back(30); positiveIdxList.push_back(31); positiveIdxList.push_back(32);
	positiveIdxList.push_back(33); positiveIdxList.push_back(34); positiveIdxList.push_back(35);
	positiveIdxList.push_back(36); negativeIdxList.push_back(37); negativeIdxList.push_back(38);
	negativeIdxList.push_back(39); positiveIdxList.push_back(40); positiveIdxList.push_back(41);
	positiveIdxList.push_back(42); positiveIdxList.push_back(43); positiveIdxList.push_back(44);
	negativeIdxList.push_back(45); negativeIdxList.push_back(46); negativeIdxList.push_back(47);
	positiveIdxList.push_back(48); positiveIdxList.push_back(49); positiveIdxList.push_back(50);
	positiveIdxList.push_back(51); positiveIdxList.push_back(52); negativeIdxList.push_back(53);
	negativeIdxList.push_back(54); negativeIdxList.push_back(55); positiveIdxList.push_back(56);
	positiveIdxList.push_back(57); positiveIdxList.push_back(58); positiveIdxList.push_back(59);
	positiveIdxList.push_back(60); negativeIdxList.push_back(61); negativeIdxList.push_back(62);
	negativeIdxList.push_back(63);
	DetectionInfo.rg[labelNum].feature.haar.HaarValue[idx++] = getHaarLikePatternValue(block_sum, &positiveIdxList, &negativeIdxList);*/

	// 22
	positiveIdxList.push_back(0); positiveIdxList.push_back(1); positiveIdxList.push_back(2);
	positiveIdxList.push_back(3); negativeIdxList.push_back(4); negativeIdxList.push_back(5);
	negativeIdxList.push_back(6); negativeIdxList.push_back(7); positiveIdxList.push_back(8);
	positiveIdxList.push_back(9); positiveIdxList.push_back(10); positiveIdxList.push_back(11);
	negativeIdxList.push_back(12); negativeIdxList.push_back(13); negativeIdxList.push_back(14);
	negativeIdxList.push_back(15); positiveIdxList.push_back(16); positiveIdxList.push_back(17);
	positiveIdxList.push_back(18); positiveIdxList.push_back(19); negativeIdxList.push_back(20);
	negativeIdxList.push_back(21); negativeIdxList.push_back(22); negativeIdxList.push_back(23);
	positiveIdxList.push_back(24); positiveIdxList.push_back(25); positiveIdxList.push_back(26);
	positiveIdxList.push_back(27); negativeIdxList.push_back(28); negativeIdxList.push_back(29);
	negativeIdxList.push_back(30); negativeIdxList.push_back(31); negativeIdxList.push_back(32);
	negativeIdxList.push_back(33); negativeIdxList.push_back(34); negativeIdxList.push_back(35);
	positiveIdxList.push_back(36); positiveIdxList.push_back(37); positiveIdxList.push_back(38);
	positiveIdxList.push_back(39); negativeIdxList.push_back(40); negativeIdxList.push_back(41);
	negativeIdxList.push_back(42); negativeIdxList.push_back(43); positiveIdxList.push_back(44);
	positiveIdxList.push_back(45); positiveIdxList.push_back(46); positiveIdxList.push_back(47);
	negativeIdxList.push_back(48); negativeIdxList.push_back(49); negativeIdxList.push_back(50);
	negativeIdxList.push_back(51); positiveIdxList.push_back(52); positiveIdxList.push_back(53);
	positiveIdxList.push_back(54); positiveIdxList.push_back(55); negativeIdxList.push_back(56);
	negativeIdxList.push_back(57); negativeIdxList.push_back(58); negativeIdxList.push_back(59);
	positiveIdxList.push_back(60); positiveIdxList.push_back(61); positiveIdxList.push_back(62);
	positiveIdxList.push_back(63);
	DetectionInfo.rg[labelNum].feature.haar.HaarValue[idx++] = getHaarLikePatternValue(block_sum, &positiveIdxList, &negativeIdxList);

	// 23
	negativeIdxList.push_back(0); negativeIdxList.push_back(1); positiveIdxList.push_back(2);
	positiveIdxList.push_back(3); positiveIdxList.push_back(4); positiveIdxList.push_back(5);
	negativeIdxList.push_back(6); negativeIdxList.push_back(7); negativeIdxList.push_back(8);
	negativeIdxList.push_back(9); positiveIdxList.push_back(10); positiveIdxList.push_back(11);
	positiveIdxList.push_back(12); positiveIdxList.push_back(13); negativeIdxList.push_back(14);
	negativeIdxList.push_back(15); positiveIdxList.push_back(16); positiveIdxList.push_back(17);
	negativeIdxList.push_back(18); negativeIdxList.push_back(19); negativeIdxList.push_back(20);
	negativeIdxList.push_back(21); positiveIdxList.push_back(22); positiveIdxList.push_back(23);
	positiveIdxList.push_back(24); positiveIdxList.push_back(25); negativeIdxList.push_back(26);
	negativeIdxList.push_back(27); negativeIdxList.push_back(28); negativeIdxList.push_back(29);
	positiveIdxList.push_back(30); positiveIdxList.push_back(31); positiveIdxList.push_back(32);
	positiveIdxList.push_back(33); negativeIdxList.push_back(34); negativeIdxList.push_back(35);
	negativeIdxList.push_back(36); negativeIdxList.push_back(37); positiveIdxList.push_back(38);
	positiveIdxList.push_back(39); positiveIdxList.push_back(40); positiveIdxList.push_back(41);
	negativeIdxList.push_back(42); negativeIdxList.push_back(43); negativeIdxList.push_back(44);
	negativeIdxList.push_back(45); positiveIdxList.push_back(46); positiveIdxList.push_back(47);
	positiveIdxList.push_back(48); positiveIdxList.push_back(49); negativeIdxList.push_back(50);
	negativeIdxList.push_back(51); negativeIdxList.push_back(52); negativeIdxList.push_back(53);
	positiveIdxList.push_back(54); positiveIdxList.push_back(55); positiveIdxList.push_back(56);
	positiveIdxList.push_back(57); negativeIdxList.push_back(58); negativeIdxList.push_back(59);
	negativeIdxList.push_back(60); negativeIdxList.push_back(61); positiveIdxList.push_back(62);
	positiveIdxList.push_back(63);
	DetectionInfo.rg[labelNum].feature.haar.HaarValue[idx++] = getHaarLikePatternValue(block_sum, &positiveIdxList, &negativeIdxList);

	// 24
	negativeIdxList.push_back(0); negativeIdxList.push_back(1); positiveIdxList.push_back(2);
	positiveIdxList.push_back(3); positiveIdxList.push_back(4); positiveIdxList.push_back(5);
	positiveIdxList.push_back(6); positiveIdxList.push_back(7); negativeIdxList.push_back(8);
	negativeIdxList.push_back(9); positiveIdxList.push_back(10); positiveIdxList.push_back(11);
	positiveIdxList.push_back(12); positiveIdxList.push_back(13); positiveIdxList.push_back(14);
	positiveIdxList.push_back(15); positiveIdxList.push_back(16); positiveIdxList.push_back(17);
	negativeIdxList.push_back(18); negativeIdxList.push_back(19); negativeIdxList.push_back(20);
	negativeIdxList.push_back(21); negativeIdxList.push_back(22); negativeIdxList.push_back(23);
	positiveIdxList.push_back(24); positiveIdxList.push_back(25); negativeIdxList.push_back(26);
	negativeIdxList.push_back(27); negativeIdxList.push_back(28); negativeIdxList.push_back(29);
	negativeIdxList.push_back(30); negativeIdxList.push_back(31); positiveIdxList.push_back(32);
	positiveIdxList.push_back(33); negativeIdxList.push_back(34); negativeIdxList.push_back(35);
	negativeIdxList.push_back(36); negativeIdxList.push_back(37); negativeIdxList.push_back(38);
	negativeIdxList.push_back(39); positiveIdxList.push_back(40); positiveIdxList.push_back(41);
	negativeIdxList.push_back(42); negativeIdxList.push_back(43); negativeIdxList.push_back(44);
	negativeIdxList.push_back(45); negativeIdxList.push_back(46); negativeIdxList.push_back(47);
	negativeIdxList.push_back(48); negativeIdxList.push_back(49); positiveIdxList.push_back(50);
	positiveIdxList.push_back(51); positiveIdxList.push_back(52); positiveIdxList.push_back(53);
	positiveIdxList.push_back(54); positiveIdxList.push_back(55); negativeIdxList.push_back(56);
	negativeIdxList.push_back(57); positiveIdxList.push_back(58); positiveIdxList.push_back(59);
	positiveIdxList.push_back(60); positiveIdxList.push_back(61); positiveIdxList.push_back(62);
	positiveIdxList.push_back(63);
	DetectionInfo.rg[labelNum].feature.haar.HaarValue[idx++] = getHaarLikePatternValue(block_sum, &positiveIdxList, &negativeIdxList);

	// 25
	positiveIdxList.push_back(0); positiveIdxList.push_back(1); negativeIdxList.push_back(2);
	negativeIdxList.push_back(3); negativeIdxList.push_back(4); negativeIdxList.push_back(5);
	positiveIdxList.push_back(6); positiveIdxList.push_back(7); positiveIdxList.push_back(8);
	positiveIdxList.push_back(9); negativeIdxList.push_back(10); negativeIdxList.push_back(11);
	negativeIdxList.push_back(12); negativeIdxList.push_back(13); positiveIdxList.push_back(14);
	positiveIdxList.push_back(15); positiveIdxList.push_back(16); positiveIdxList.push_back(17);
	negativeIdxList.push_back(18); negativeIdxList.push_back(19); negativeIdxList.push_back(20);
	negativeIdxList.push_back(21); positiveIdxList.push_back(22); positiveIdxList.push_back(23);
	positiveIdxList.push_back(24); positiveIdxList.push_back(25); negativeIdxList.push_back(26);
	negativeIdxList.push_back(27); negativeIdxList.push_back(28); negativeIdxList.push_back(29);
	positiveIdxList.push_back(30); positiveIdxList.push_back(31); positiveIdxList.push_back(32);
	positiveIdxList.push_back(33); negativeIdxList.push_back(34); negativeIdxList.push_back(35);
	negativeIdxList.push_back(36); negativeIdxList.push_back(37); positiveIdxList.push_back(38);
	positiveIdxList.push_back(39); positiveIdxList.push_back(40); positiveIdxList.push_back(41);
	negativeIdxList.push_back(42); negativeIdxList.push_back(43); negativeIdxList.push_back(44);
	negativeIdxList.push_back(45); positiveIdxList.push_back(46); positiveIdxList.push_back(47);
	negativeIdxList.push_back(48); negativeIdxList.push_back(49); positiveIdxList.push_back(50);
	positiveIdxList.push_back(51); positiveIdxList.push_back(52); positiveIdxList.push_back(53);
	negativeIdxList.push_back(54); negativeIdxList.push_back(55); negativeIdxList.push_back(56);
	negativeIdxList.push_back(57); positiveIdxList.push_back(58); positiveIdxList.push_back(59);
	positiveIdxList.push_back(60); positiveIdxList.push_back(61); negativeIdxList.push_back(62);
	negativeIdxList.push_back(63);
	DetectionInfo.rg[labelNum].feature.haar.HaarValue[idx++] = getHaarLikePatternValue(block_sum, &positiveIdxList, &negativeIdxList);

	// 26
	/*negativeIdxList.push_back(0); negativeIdxList.push_back(1); negativeIdxList.push_back(2);
	positiveIdxList.push_back(3); positiveIdxList.push_back(4); negativeIdxList.push_back(5);
	negativeIdxList.push_back(6); negativeIdxList.push_back(7); negativeIdxList.push_back(8);
	negativeIdxList.push_back(9); negativeIdxList.push_back(10); positiveIdxList.push_back(11);
	positiveIdxList.push_back(12); negativeIdxList.push_back(13); negativeIdxList.push_back(14);
	negativeIdxList.push_back(15); negativeIdxList.push_back(16); negativeIdxList.push_back(17);
	negativeIdxList.push_back(18); positiveIdxList.push_back(19); positiveIdxList.push_back(20);
	negativeIdxList.push_back(21); negativeIdxList.push_back(22); negativeIdxList.push_back(23);
	positiveIdxList.push_back(24); positiveIdxList.push_back(25); positiveIdxList.push_back(26);
	negativeIdxList.push_back(27); negativeIdxList.push_back(28); positiveIdxList.push_back(29);
	positiveIdxList.push_back(30); positiveIdxList.push_back(31); positiveIdxList.push_back(32);
	positiveIdxList.push_back(33); positiveIdxList.push_back(34); negativeIdxList.push_back(35);
	negativeIdxList.push_back(36); positiveIdxList.push_back(37); positiveIdxList.push_back(38);
	positiveIdxList.push_back(39); positiveIdxList.push_back(40); positiveIdxList.push_back(41);
	positiveIdxList.push_back(42); negativeIdxList.push_back(43); negativeIdxList.push_back(44);
	positiveIdxList.push_back(45); positiveIdxList.push_back(46); positiveIdxList.push_back(47);
	positiveIdxList.push_back(48); positiveIdxList.push_back(49); positiveIdxList.push_back(50);
	negativeIdxList.push_back(51); negativeIdxList.push_back(52); positiveIdxList.push_back(53);
	positiveIdxList.push_back(54); positiveIdxList.push_back(55); positiveIdxList.push_back(56);
	positiveIdxList.push_back(57); positiveIdxList.push_back(58); negativeIdxList.push_back(59);
	negativeIdxList.push_back(60); positiveIdxList.push_back(61); positiveIdxList.push_back(62);
	positiveIdxList.push_back(63);
	DetectionInfo.rg[labelNum].feature.haar.HaarValue[idx++] = getHaarLikePatternValue(block_sum, &positiveIdxList, &negativeIdxList);*/

	// 27
	/*negativeIdxList.push_back(0); negativeIdxList.push_back(1); negativeIdxList.push_back(2);
	positiveIdxList.push_back(3); positiveIdxList.push_back(4); positiveIdxList.push_back(5);
	positiveIdxList.push_back(6); positiveIdxList.push_back(7); negativeIdxList.push_back(8);
	negativeIdxList.push_back(9); negativeIdxList.push_back(10); positiveIdxList.push_back(11);
	positiveIdxList.push_back(12); positiveIdxList.push_back(13); positiveIdxList.push_back(14);
	positiveIdxList.push_back(15); negativeIdxList.push_back(16); negativeIdxList.push_back(17);
	negativeIdxList.push_back(18); positiveIdxList.push_back(19); positiveIdxList.push_back(20);
	positiveIdxList.push_back(21); positiveIdxList.push_back(22); positiveIdxList.push_back(23);
	positiveIdxList.push_back(24); positiveIdxList.push_back(25); positiveIdxList.push_back(26);
	negativeIdxList.push_back(27); negativeIdxList.push_back(28); negativeIdxList.push_back(29);
	negativeIdxList.push_back(30); negativeIdxList.push_back(31); positiveIdxList.push_back(32);
	positiveIdxList.push_back(33); positiveIdxList.push_back(34); negativeIdxList.push_back(35);
	negativeIdxList.push_back(36); negativeIdxList.push_back(37); negativeIdxList.push_back(38);
	negativeIdxList.push_back(39); negativeIdxList.push_back(40); negativeIdxList.push_back(41);
	negativeIdxList.push_back(42); positiveIdxList.push_back(43); positiveIdxList.push_back(44);
	positiveIdxList.push_back(45); positiveIdxList.push_back(46); positiveIdxList.push_back(47);
	negativeIdxList.push_back(48); negativeIdxList.push_back(49); negativeIdxList.push_back(50);
	positiveIdxList.push_back(51); positiveIdxList.push_back(52); positiveIdxList.push_back(53);
	positiveIdxList.push_back(54); positiveIdxList.push_back(55); negativeIdxList.push_back(56);
	negativeIdxList.push_back(57); negativeIdxList.push_back(58); positiveIdxList.push_back(59);
	positiveIdxList.push_back(60); positiveIdxList.push_back(61); positiveIdxList.push_back(62);
	positiveIdxList.push_back(63);
	DetectionInfo.rg[labelNum].feature.haar.HaarValue[idx++] = getHaarLikePatternValue(block_sum, &positiveIdxList, &negativeIdxList);*/

	// 28
	/*positiveIdxList.push_back(0); positiveIdxList.push_back(1); positiveIdxList.push_back(2);
	positiveIdxList.push_back(3); positiveIdxList.push_back(4); negativeIdxList.push_back(5);
	negativeIdxList.push_back(6); negativeIdxList.push_back(7); positiveIdxList.push_back(8);
	positiveIdxList.push_back(9); positiveIdxList.push_back(10); positiveIdxList.push_back(11);
	positiveIdxList.push_back(12); negativeIdxList.push_back(13); negativeIdxList.push_back(14);
	negativeIdxList.push_back(15); positiveIdxList.push_back(16); positiveIdxList.push_back(17);
	positiveIdxList.push_back(18); positiveIdxList.push_back(19); positiveIdxList.push_back(20);
	negativeIdxList.push_back(21); negativeIdxList.push_back(22); negativeIdxList.push_back(23);
	negativeIdxList.push_back(24); negativeIdxList.push_back(25); negativeIdxList.push_back(26);
	negativeIdxList.push_back(27); negativeIdxList.push_back(28); positiveIdxList.push_back(29);
	positiveIdxList.push_back(30); positiveIdxList.push_back(31); negativeIdxList.push_back(32);
	negativeIdxList.push_back(33); negativeIdxList.push_back(34); negativeIdxList.push_back(35);
	negativeIdxList.push_back(36); positiveIdxList.push_back(37); positiveIdxList.push_back(38);
	positiveIdxList.push_back(39); positiveIdxList.push_back(40); positiveIdxList.push_back(41);
	positiveIdxList.push_back(42); positiveIdxList.push_back(43); positiveIdxList.push_back(44);
	negativeIdxList.push_back(45); negativeIdxList.push_back(46); negativeIdxList.push_back(47);
	positiveIdxList.push_back(48); positiveIdxList.push_back(49); positiveIdxList.push_back(50);
	positiveIdxList.push_back(51); positiveIdxList.push_back(52); negativeIdxList.push_back(53);
	negativeIdxList.push_back(54); negativeIdxList.push_back(55); positiveIdxList.push_back(56);
	positiveIdxList.push_back(57); positiveIdxList.push_back(58); positiveIdxList.push_back(59);
	positiveIdxList.push_back(60); negativeIdxList.push_back(61); negativeIdxList.push_back(62);
	negativeIdxList.push_back(63);
	DetectionInfo.rg[labelNum].feature.haar.HaarValue[idx++] = getHaarLikePatternValue(block_sum, &positiveIdxList, &negativeIdxList);*/

	// 29
	/*negativeIdxList.push_back(0); negativeIdxList.push_back(1); negativeIdxList.push_back(2);
	negativeIdxList.push_back(3); positiveIdxList.push_back(4); positiveIdxList.push_back(5);
	positiveIdxList.push_back(6); positiveIdxList.push_back(7); negativeIdxList.push_back(8);
	negativeIdxList.push_back(9); negativeIdxList.push_back(10); negativeIdxList.push_back(11);
	positiveIdxList.push_back(12); positiveIdxList.push_back(13); positiveIdxList.push_back(14);
	positiveIdxList.push_back(15); negativeIdxList.push_back(16); negativeIdxList.push_back(17);
	negativeIdxList.push_back(18); negativeIdxList.push_back(19); positiveIdxList.push_back(20);
	positiveIdxList.push_back(21); positiveIdxList.push_back(22); positiveIdxList.push_back(23);
	positiveIdxList.push_back(24); positiveIdxList.push_back(25); positiveIdxList.push_back(26);
	positiveIdxList.push_back(27); negativeIdxList.push_back(28); negativeIdxList.push_back(29);
	negativeIdxList.push_back(30); negativeIdxList.push_back(31); positiveIdxList.push_back(32);
	positiveIdxList.push_back(33); positiveIdxList.push_back(34); positiveIdxList.push_back(35);
	negativeIdxList.push_back(36); negativeIdxList.push_back(37); negativeIdxList.push_back(38);
	negativeIdxList.push_back(39); negativeIdxList.push_back(40); negativeIdxList.push_back(41);
	negativeIdxList.push_back(42); negativeIdxList.push_back(43); positiveIdxList.push_back(44);
	positiveIdxList.push_back(45); positiveIdxList.push_back(46); positiveIdxList.push_back(47);
	negativeIdxList.push_back(48); negativeIdxList.push_back(49); negativeIdxList.push_back(50);
	negativeIdxList.push_back(51); positiveIdxList.push_back(52); positiveIdxList.push_back(53);
	positiveIdxList.push_back(54); positiveIdxList.push_back(55); negativeIdxList.push_back(56);
	negativeIdxList.push_back(57); negativeIdxList.push_back(58); negativeIdxList.push_back(59);
	positiveIdxList.push_back(60); positiveIdxList.push_back(61); positiveIdxList.push_back(62);
	positiveIdxList.push_back(63);
	DetectionInfo.rg[labelNum].feature.haar.HaarValue[idx++] = getHaarLikePatternValue(block_sum, &positiveIdxList, &negativeIdxList);*/

	// 30
	/*positiveIdxList.push_back(0); positiveIdxList.push_back(1); negativeIdxList.push_back(2);
	negativeIdxList.push_back(3); negativeIdxList.push_back(4); negativeIdxList.push_back(5);
	positiveIdxList.push_back(6); positiveIdxList.push_back(7); positiveIdxList.push_back(8);
	positiveIdxList.push_back(9); negativeIdxList.push_back(10); negativeIdxList.push_back(11);
	negativeIdxList.push_back(12); negativeIdxList.push_back(13); positiveIdxList.push_back(14);
	positiveIdxList.push_back(15); positiveIdxList.push_back(16); positiveIdxList.push_back(17);
	negativeIdxList.push_back(18); negativeIdxList.push_back(19); negativeIdxList.push_back(20);
	negativeIdxList.push_back(21); positiveIdxList.push_back(22); positiveIdxList.push_back(23);
	positiveIdxList.push_back(24); positiveIdxList.push_back(25); negativeIdxList.push_back(26);
	negativeIdxList.push_back(27); negativeIdxList.push_back(28); negativeIdxList.push_back(29);
	positiveIdxList.push_back(30); positiveIdxList.push_back(31); negativeIdxList.push_back(32);
	negativeIdxList.push_back(33); positiveIdxList.push_back(34); positiveIdxList.push_back(35);
	positiveIdxList.push_back(36); positiveIdxList.push_back(37); negativeIdxList.push_back(38);
	negativeIdxList.push_back(39); negativeIdxList.push_back(40); negativeIdxList.push_back(41);
	positiveIdxList.push_back(42); positiveIdxList.push_back(43); positiveIdxList.push_back(44);
	positiveIdxList.push_back(45); negativeIdxList.push_back(46); negativeIdxList.push_back(47);
	negativeIdxList.push_back(48); negativeIdxList.push_back(49); positiveIdxList.push_back(50);
	positiveIdxList.push_back(51); positiveIdxList.push_back(52); positiveIdxList.push_back(53);
	negativeIdxList.push_back(54); negativeIdxList.push_back(55); negativeIdxList.push_back(56);
	negativeIdxList.push_back(57); positiveIdxList.push_back(58); positiveIdxList.push_back(59);
	positiveIdxList.push_back(60); positiveIdxList.push_back(61); negativeIdxList.push_back(62);
	negativeIdxList.push_back(63);
	DetectionInfo.rg[labelNum].feature.haar.HaarValue[idx++] = getHaarLikePatternValue(block_sum, &positiveIdxList, &negativeIdxList);*/

	// 31
	/*positiveIdxList.push_back(0); positiveIdxList.push_back(1); positiveIdxList.push_back(2);
	negativeIdxList.push_back(3); negativeIdxList.push_back(4); positiveIdxList.push_back(5);
	positiveIdxList.push_back(6); positiveIdxList.push_back(7); positiveIdxList.push_back(8);
	positiveIdxList.push_back(9); positiveIdxList.push_back(10); negativeIdxList.push_back(11);
	negativeIdxList.push_back(12); positiveIdxList.push_back(13); positiveIdxList.push_back(14);
	positiveIdxList.push_back(15); positiveIdxList.push_back(16); positiveIdxList.push_back(17);
	positiveIdxList.push_back(18); negativeIdxList.push_back(19); negativeIdxList.push_back(20);
	positiveIdxList.push_back(21); positiveIdxList.push_back(22); positiveIdxList.push_back(23);
	positiveIdxList.push_back(24); positiveIdxList.push_back(25); positiveIdxList.push_back(26);
	negativeIdxList.push_back(27); negativeIdxList.push_back(28); positiveIdxList.push_back(29);
	positiveIdxList.push_back(30); positiveIdxList.push_back(31); positiveIdxList.push_back(32);
	positiveIdxList.push_back(33); positiveIdxList.push_back(34); negativeIdxList.push_back(35);
	negativeIdxList.push_back(36); positiveIdxList.push_back(37); positiveIdxList.push_back(38);
	positiveIdxList.push_back(39); negativeIdxList.push_back(40); negativeIdxList.push_back(41);
	negativeIdxList.push_back(42); positiveIdxList.push_back(43); positiveIdxList.push_back(44);
	negativeIdxList.push_back(45); negativeIdxList.push_back(46); negativeIdxList.push_back(47);
	negativeIdxList.push_back(48); negativeIdxList.push_back(49); negativeIdxList.push_back(50);
	positiveIdxList.push_back(51); positiveIdxList.push_back(52); negativeIdxList.push_back(53);
	negativeIdxList.push_back(54); negativeIdxList.push_back(55); negativeIdxList.push_back(56);
	negativeIdxList.push_back(57); negativeIdxList.push_back(58); positiveIdxList.push_back(59);
	positiveIdxList.push_back(60); negativeIdxList.push_back(61); negativeIdxList.push_back(62);
	negativeIdxList.push_back(63);
	DetectionInfo.rg[labelNum].feature.haar.HaarValue[idx++] = getHaarLikePatternValue(block_sum, &positiveIdxList, &negativeIdxList);*/

	// 32
	/*positiveIdxList.push_back(0); positiveIdxList.push_back(1); positiveIdxList.push_back(2);
	negativeIdxList.push_back(3); negativeIdxList.push_back(4); positiveIdxList.push_back(5);
	positiveIdxList.push_back(6); positiveIdxList.push_back(7); positiveIdxList.push_back(8);
	positiveIdxList.push_back(9); positiveIdxList.push_back(10); negativeIdxList.push_back(11);
	negativeIdxList.push_back(12); positiveIdxList.push_back(13); positiveIdxList.push_back(14);
	positiveIdxList.push_back(15); positiveIdxList.push_back(16); positiveIdxList.push_back(17);
	positiveIdxList.push_back(18); negativeIdxList.push_back(19); negativeIdxList.push_back(20);
	positiveIdxList.push_back(21); positiveIdxList.push_back(22); positiveIdxList.push_back(23);
	positiveIdxList.push_back(24); positiveIdxList.push_back(25); positiveIdxList.push_back(26);
	negativeIdxList.push_back(27); negativeIdxList.push_back(28); positiveIdxList.push_back(29);
	positiveIdxList.push_back(30); positiveIdxList.push_back(31); negativeIdxList.push_back(32);
	negativeIdxList.push_back(33); negativeIdxList.push_back(34); positiveIdxList.push_back(35);
	positiveIdxList.push_back(36); negativeIdxList.push_back(37); negativeIdxList.push_back(38);
	negativeIdxList.push_back(39); negativeIdxList.push_back(40); negativeIdxList.push_back(41);
	negativeIdxList.push_back(42); positiveIdxList.push_back(43); positiveIdxList.push_back(44);
	negativeIdxList.push_back(45); negativeIdxList.push_back(46); negativeIdxList.push_back(47);
	negativeIdxList.push_back(48); negativeIdxList.push_back(49); negativeIdxList.push_back(50);
	positiveIdxList.push_back(51); positiveIdxList.push_back(52); negativeIdxList.push_back(53);
	negativeIdxList.push_back(54); negativeIdxList.push_back(55); negativeIdxList.push_back(56);
	negativeIdxList.push_back(57); negativeIdxList.push_back(58); positiveIdxList.push_back(59);
	positiveIdxList.push_back(60); negativeIdxList.push_back(61); negativeIdxList.push_back(62);
	negativeIdxList.push_back(63);
	DetectionInfo.rg[labelNum].feature.haar.HaarValue[idx++] = getHaarLikePatternValue(block_sum, &positiveIdxList, &negativeIdxList);*/

	// 33
	/*negativeIdxList.push_back(0); negativeIdxList.push_back(1); negativeIdxList.push_back(2);
	negativeIdxList.push_back(3); positiveIdxList.push_back(4); positiveIdxList.push_back(5);
	positiveIdxList.push_back(6); positiveIdxList.push_back(7); negativeIdxList.push_back(8);
	negativeIdxList.push_back(9); negativeIdxList.push_back(10); negativeIdxList.push_back(11);
	positiveIdxList.push_back(12); positiveIdxList.push_back(13); positiveIdxList.push_back(14);
	positiveIdxList.push_back(15); positiveIdxList.push_back(16); positiveIdxList.push_back(17);
	positiveIdxList.push_back(18); positiveIdxList.push_back(19); negativeIdxList.push_back(20);
	negativeIdxList.push_back(21); negativeIdxList.push_back(22); negativeIdxList.push_back(23);
	positiveIdxList.push_back(24); positiveIdxList.push_back(25); positiveIdxList.push_back(26);
	positiveIdxList.push_back(27); negativeIdxList.push_back(28); negativeIdxList.push_back(29);
	negativeIdxList.push_back(30); negativeIdxList.push_back(31); positiveIdxList.push_back(32);
	positiveIdxList.push_back(33); positiveIdxList.push_back(34); positiveIdxList.push_back(35);
	negativeIdxList.push_back(36); negativeIdxList.push_back(37); negativeIdxList.push_back(38);
	negativeIdxList.push_back(39); positiveIdxList.push_back(40); positiveIdxList.push_back(41);
	positiveIdxList.push_back(42); positiveIdxList.push_back(43); negativeIdxList.push_back(44);
	negativeIdxList.push_back(45); negativeIdxList.push_back(46); negativeIdxList.push_back(47);
	negativeIdxList.push_back(48); negativeIdxList.push_back(49); negativeIdxList.push_back(50);
	negativeIdxList.push_back(51); positiveIdxList.push_back(52); positiveIdxList.push_back(53);
	positiveIdxList.push_back(54); positiveIdxList.push_back(55); negativeIdxList.push_back(56);
	negativeIdxList.push_back(57); negativeIdxList.push_back(58); negativeIdxList.push_back(59);
	positiveIdxList.push_back(60); positiveIdxList.push_back(61); positiveIdxList.push_back(62);
	positiveIdxList.push_back(63);
	DetectionInfo.rg[labelNum].feature.haar.HaarValue[idx++] = getHaarLikePatternValue(block_sum, &positiveIdxList, &negativeIdxList);*/

	// 34
	/*negativeIdxList.push_back(0); negativeIdxList.push_back(1); positiveIdxList.push_back(2);
	positiveIdxList.push_back(3); positiveIdxList.push_back(4); positiveIdxList.push_back(5);
	negativeIdxList.push_back(6); negativeIdxList.push_back(7); negativeIdxList.push_back(8);
	negativeIdxList.push_back(9); positiveIdxList.push_back(10); positiveIdxList.push_back(11);
	positiveIdxList.push_back(12); positiveIdxList.push_back(13); negativeIdxList.push_back(14);
	negativeIdxList.push_back(15); negativeIdxList.push_back(16); negativeIdxList.push_back(17);
	positiveIdxList.push_back(18); positiveIdxList.push_back(19); positiveIdxList.push_back(20);
	positiveIdxList.push_back(21); negativeIdxList.push_back(22); negativeIdxList.push_back(23);
	positiveIdxList.push_back(24); positiveIdxList.push_back(25); negativeIdxList.push_back(26);
	negativeIdxList.push_back(27); negativeIdxList.push_back(28); negativeIdxList.push_back(29);
	positiveIdxList.push_back(30); positiveIdxList.push_back(31); positiveIdxList.push_back(32);
	positiveIdxList.push_back(33); negativeIdxList.push_back(34); negativeIdxList.push_back(35);
	negativeIdxList.push_back(36); negativeIdxList.push_back(37); positiveIdxList.push_back(38);
	positiveIdxList.push_back(39); positiveIdxList.push_back(40); positiveIdxList.push_back(41);
	negativeIdxList.push_back(42); negativeIdxList.push_back(43); negativeIdxList.push_back(44);
	negativeIdxList.push_back(45); positiveIdxList.push_back(46); positiveIdxList.push_back(47);
	positiveIdxList.push_back(48); positiveIdxList.push_back(49); negativeIdxList.push_back(50);
	negativeIdxList.push_back(51); negativeIdxList.push_back(52); negativeIdxList.push_back(53);
	positiveIdxList.push_back(54); positiveIdxList.push_back(55); positiveIdxList.push_back(56);
	positiveIdxList.push_back(57); negativeIdxList.push_back(58); negativeIdxList.push_back(59);
	negativeIdxList.push_back(60); negativeIdxList.push_back(61); positiveIdxList.push_back(62);
	positiveIdxList.push_back(63);
	DetectionInfo.rg[labelNum].feature.haar.HaarValue[idx++] = getHaarLikePatternValue(block_sum, &positiveIdxList, &negativeIdxList);*/

	// 35
	positiveIdxList.push_back(0); positiveIdxList.push_back(1); positiveIdxList.push_back(2);
	positiveIdxList.push_back(3); positiveIdxList.push_back(4); negativeIdxList.push_back(5);
	negativeIdxList.push_back(6); negativeIdxList.push_back(7); positiveIdxList.push_back(8);
	positiveIdxList.push_back(9); positiveIdxList.push_back(10); positiveIdxList.push_back(11);
	positiveIdxList.push_back(12); negativeIdxList.push_back(13); negativeIdxList.push_back(14);
	negativeIdxList.push_back(15); negativeIdxList.push_back(16); negativeIdxList.push_back(17);
	negativeIdxList.push_back(18); negativeIdxList.push_back(19); negativeIdxList.push_back(20);
	positiveIdxList.push_back(21); positiveIdxList.push_back(22); positiveIdxList.push_back(23);
	negativeIdxList.push_back(24); negativeIdxList.push_back(25); negativeIdxList.push_back(26);
	negativeIdxList.push_back(27); negativeIdxList.push_back(28); positiveIdxList.push_back(29);
	positiveIdxList.push_back(30); positiveIdxList.push_back(31); negativeIdxList.push_back(32);
	negativeIdxList.push_back(33); negativeIdxList.push_back(34); negativeIdxList.push_back(35);
	negativeIdxList.push_back(36); positiveIdxList.push_back(37); positiveIdxList.push_back(38);
	positiveIdxList.push_back(39); negativeIdxList.push_back(40); negativeIdxList.push_back(41);
	negativeIdxList.push_back(42); negativeIdxList.push_back(43); negativeIdxList.push_back(44);
	positiveIdxList.push_back(45); positiveIdxList.push_back(46); positiveIdxList.push_back(47);
	positiveIdxList.push_back(48); positiveIdxList.push_back(49); positiveIdxList.push_back(50);
	positiveIdxList.push_back(51); positiveIdxList.push_back(52); negativeIdxList.push_back(53);
	negativeIdxList.push_back(54); negativeIdxList.push_back(55); positiveIdxList.push_back(56);
	positiveIdxList.push_back(57); positiveIdxList.push_back(58); positiveIdxList.push_back(59);
	positiveIdxList.push_back(60); negativeIdxList.push_back(61); negativeIdxList.push_back(62);
	negativeIdxList.push_back(63);
	DetectionInfo.rg[labelNum].feature.haar.HaarValue[idx++] = getHaarLikePatternValue(block_sum, &positiveIdxList, &negativeIdxList);

	// 36
	/*positiveIdxList.push_back(0); positiveIdxList.push_back(1); negativeIdxList.push_back(2);
	negativeIdxList.push_back(3); negativeIdxList.push_back(4); negativeIdxList.push_back(5);
	positiveIdxList.push_back(6); positiveIdxList.push_back(7); positiveIdxList.push_back(8);
	positiveIdxList.push_back(9); negativeIdxList.push_back(10); negativeIdxList.push_back(11);
	negativeIdxList.push_back(12); negativeIdxList.push_back(13); positiveIdxList.push_back(14);
	positiveIdxList.push_back(15); positiveIdxList.push_back(16); positiveIdxList.push_back(17);
	negativeIdxList.push_back(18); negativeIdxList.push_back(19); negativeIdxList.push_back(20);
	negativeIdxList.push_back(21); positiveIdxList.push_back(22); positiveIdxList.push_back(23);
	positiveIdxList.push_back(24); positiveIdxList.push_back(25); negativeIdxList.push_back(26);
	negativeIdxList.push_back(27); negativeIdxList.push_back(28); negativeIdxList.push_back(29);
	positiveIdxList.push_back(30); positiveIdxList.push_back(31); positiveIdxList.push_back(32);
	positiveIdxList.push_back(33); negativeIdxList.push_back(34); negativeIdxList.push_back(35);
	negativeIdxList.push_back(36); negativeIdxList.push_back(37); positiveIdxList.push_back(38);
	positiveIdxList.push_back(39); negativeIdxList.push_back(40); negativeIdxList.push_back(41);
	positiveIdxList.push_back(42); positiveIdxList.push_back(43); positiveIdxList.push_back(44);
	positiveIdxList.push_back(45); negativeIdxList.push_back(46); negativeIdxList.push_back(47);
	negativeIdxList.push_back(48); negativeIdxList.push_back(49); positiveIdxList.push_back(50);
	positiveIdxList.push_back(51); positiveIdxList.push_back(52); positiveIdxList.push_back(53);
	negativeIdxList.push_back(54); negativeIdxList.push_back(55); negativeIdxList.push_back(56);
	negativeIdxList.push_back(57); positiveIdxList.push_back(58); positiveIdxList.push_back(59);
	positiveIdxList.push_back(60); positiveIdxList.push_back(61); negativeIdxList.push_back(62);
	negativeIdxList.push_back(63);
	DetectionInfo.rg[labelNum].feature.haar.HaarValue[idx++] = getHaarLikePatternValue(block_sum, &positiveIdxList, &negativeIdxList);*/

	// 37
	negativeIdxList.push_back(0); negativeIdxList.push_back(1); negativeIdxList.push_back(2);
	positiveIdxList.push_back(3); positiveIdxList.push_back(4); positiveIdxList.push_back(5);
	positiveIdxList.push_back(6); positiveIdxList.push_back(7); negativeIdxList.push_back(8);
	negativeIdxList.push_back(9); negativeIdxList.push_back(10); positiveIdxList.push_back(11);
	positiveIdxList.push_back(12); positiveIdxList.push_back(13); positiveIdxList.push_back(14);
	positiveIdxList.push_back(15); positiveIdxList.push_back(16); positiveIdxList.push_back(17);
	positiveIdxList.push_back(18); negativeIdxList.push_back(19); negativeIdxList.push_back(20);
	negativeIdxList.push_back(21); negativeIdxList.push_back(22); negativeIdxList.push_back(23);
	positiveIdxList.push_back(24); positiveIdxList.push_back(25); positiveIdxList.push_back(26);
	negativeIdxList.push_back(27); negativeIdxList.push_back(28); negativeIdxList.push_back(29);
	negativeIdxList.push_back(30); negativeIdxList.push_back(31); positiveIdxList.push_back(32);
	positiveIdxList.push_back(33); positiveIdxList.push_back(34); negativeIdxList.push_back(35);
	negativeIdxList.push_back(36); negativeIdxList.push_back(37); negativeIdxList.push_back(38);
	negativeIdxList.push_back(39); positiveIdxList.push_back(40); positiveIdxList.push_back(41);
	positiveIdxList.push_back(42); negativeIdxList.push_back(43); negativeIdxList.push_back(44);
	negativeIdxList.push_back(45); negativeIdxList.push_back(46); negativeIdxList.push_back(47);
	negativeIdxList.push_back(48); negativeIdxList.push_back(49); negativeIdxList.push_back(50);
	positiveIdxList.push_back(51); positiveIdxList.push_back(52); positiveIdxList.push_back(53);
	positiveIdxList.push_back(54); positiveIdxList.push_back(55); negativeIdxList.push_back(56);
	negativeIdxList.push_back(57); negativeIdxList.push_back(58); positiveIdxList.push_back(59);
	positiveIdxList.push_back(60); positiveIdxList.push_back(61); positiveIdxList.push_back(62);
	positiveIdxList.push_back(63);
	DetectionInfo.rg[labelNum].feature.haar.HaarValue[idx++] = getHaarLikePatternValue(block_sum, &positiveIdxList, &negativeIdxList);

	// 38
	/*positiveIdxList.push_back(0); positiveIdxList.push_back(1); positiveIdxList.push_back(2);
	positiveIdxList.push_back(3); positiveIdxList.push_back(4); positiveIdxList.push_back(5);
	negativeIdxList.push_back(6); negativeIdxList.push_back(7); positiveIdxList.push_back(8);
	positiveIdxList.push_back(9); positiveIdxList.push_back(10); positiveIdxList.push_back(11);
	positiveIdxList.push_back(12); positiveIdxList.push_back(13); negativeIdxList.push_back(14);
	negativeIdxList.push_back(15); negativeIdxList.push_back(16); negativeIdxList.push_back(17);
	negativeIdxList.push_back(18); negativeIdxList.push_back(19); negativeIdxList.push_back(20);
	negativeIdxList.push_back(21); positiveIdxList.push_back(22); positiveIdxList.push_back(23);
	negativeIdxList.push_back(24); negativeIdxList.push_back(25); negativeIdxList.push_back(26);
	negativeIdxList.push_back(27); negativeIdxList.push_back(28); negativeIdxList.push_back(29);
	positiveIdxList.push_back(30); positiveIdxList.push_back(31); negativeIdxList.push_back(32);
	negativeIdxList.push_back(33); negativeIdxList.push_back(34); negativeIdxList.push_back(35);
	negativeIdxList.push_back(36); negativeIdxList.push_back(37); positiveIdxList.push_back(38);
	positiveIdxList.push_back(39); negativeIdxList.push_back(40); negativeIdxList.push_back(41);
	negativeIdxList.push_back(42); negativeIdxList.push_back(43); negativeIdxList.push_back(44);
	negativeIdxList.push_back(45); positiveIdxList.push_back(46); positiveIdxList.push_back(47);
	positiveIdxList.push_back(48); positiveIdxList.push_back(49); positiveIdxList.push_back(50);
	positiveIdxList.push_back(51); positiveIdxList.push_back(52); positiveIdxList.push_back(53);
	negativeIdxList.push_back(54); negativeIdxList.push_back(55); positiveIdxList.push_back(56);
	positiveIdxList.push_back(57); positiveIdxList.push_back(58); positiveIdxList.push_back(59);
	positiveIdxList.push_back(60); positiveIdxList.push_back(61); negativeIdxList.push_back(62);
	negativeIdxList.push_back(63);
	DetectionInfo.rg[labelNum].feature.haar.HaarValue[idx++] = getHaarLikePatternValue(block_sum, &positiveIdxList, &negativeIdxList);*/

	// 39
	/*positiveIdxList.push_back(0); positiveIdxList.push_back(1); negativeIdxList.push_back(2);
	negativeIdxList.push_back(3); positiveIdxList.push_back(4); positiveIdxList.push_back(5);
	positiveIdxList.push_back(6); positiveIdxList.push_back(7); positiveIdxList.push_back(8);
	positiveIdxList.push_back(9); negativeIdxList.push_back(10); negativeIdxList.push_back(11);
	positiveIdxList.push_back(12); positiveIdxList.push_back(13); positiveIdxList.push_back(14);
	positiveIdxList.push_back(15); negativeIdxList.push_back(16); negativeIdxList.push_back(17);
	positiveIdxList.push_back(18); positiveIdxList.push_back(19); negativeIdxList.push_back(20);
	negativeIdxList.push_back(21); negativeIdxList.push_back(22); negativeIdxList.push_back(23);
	negativeIdxList.push_back(24); negativeIdxList.push_back(25); positiveIdxList.push_back(26);
	positiveIdxList.push_back(27); negativeIdxList.push_back(28); negativeIdxList.push_back(29);
	negativeIdxList.push_back(30); negativeIdxList.push_back(31); negativeIdxList.push_back(32);
	negativeIdxList.push_back(33); positiveIdxList.push_back(34); positiveIdxList.push_back(35);
	negativeIdxList.push_back(36); negativeIdxList.push_back(37); negativeIdxList.push_back(38);
	negativeIdxList.push_back(39); negativeIdxList.push_back(40); negativeIdxList.push_back(41);
	positiveIdxList.push_back(42); positiveIdxList.push_back(43); negativeIdxList.push_back(44);
	negativeIdxList.push_back(45); negativeIdxList.push_back(46); negativeIdxList.push_back(47);
	positiveIdxList.push_back(48); positiveIdxList.push_back(49); negativeIdxList.push_back(50);
	negativeIdxList.push_back(51); positiveIdxList.push_back(52); positiveIdxList.push_back(53);
	positiveIdxList.push_back(54); positiveIdxList.push_back(55); positiveIdxList.push_back(56);
	positiveIdxList.push_back(57); negativeIdxList.push_back(58); negativeIdxList.push_back(59);
	positiveIdxList.push_back(60); positiveIdxList.push_back(61); positiveIdxList.push_back(62);
	positiveIdxList.push_back(63);
	DetectionInfo.rg[labelNum].feature.haar.HaarValue[idx++] = getHaarLikePatternValue(block_sum, &positiveIdxList, &negativeIdxList);*/

	// 40
	/*positiveIdxList.push_back(0); positiveIdxList.push_back(1); positiveIdxList.push_back(2);
	positiveIdxList.push_back(3); negativeIdxList.push_back(4); negativeIdxList.push_back(5);
	positiveIdxList.push_back(6); positiveIdxList.push_back(7); positiveIdxList.push_back(8);
	positiveIdxList.push_back(9); positiveIdxList.push_back(10); positiveIdxList.push_back(11);
	negativeIdxList.push_back(12); negativeIdxList.push_back(13); positiveIdxList.push_back(14);
	positiveIdxList.push_back(15); negativeIdxList.push_back(16); negativeIdxList.push_back(17);
	negativeIdxList.push_back(18); negativeIdxList.push_back(19); positiveIdxList.push_back(20);
	positiveIdxList.push_back(21); negativeIdxList.push_back(22); negativeIdxList.push_back(23);
	negativeIdxList.push_back(24); negativeIdxList.push_back(25); negativeIdxList.push_back(26);
	negativeIdxList.push_back(27); positiveIdxList.push_back(28); positiveIdxList.push_back(29);
	negativeIdxList.push_back(30); negativeIdxList.push_back(31); negativeIdxList.push_back(32);
	negativeIdxList.push_back(33); negativeIdxList.push_back(34); negativeIdxList.push_back(35);
	positiveIdxList.push_back(36); positiveIdxList.push_back(37); negativeIdxList.push_back(38);
	negativeIdxList.push_back(39); negativeIdxList.push_back(40); negativeIdxList.push_back(41);
	negativeIdxList.push_back(42); negativeIdxList.push_back(43); positiveIdxList.push_back(44);
	positiveIdxList.push_back(45); negativeIdxList.push_back(46); negativeIdxList.push_back(47);
	positiveIdxList.push_back(48); positiveIdxList.push_back(49); positiveIdxList.push_back(50);
	positiveIdxList.push_back(51); negativeIdxList.push_back(52); negativeIdxList.push_back(53);
	positiveIdxList.push_back(54); positiveIdxList.push_back(55); positiveIdxList.push_back(56);
	positiveIdxList.push_back(57); positiveIdxList.push_back(58); positiveIdxList.push_back(59);
	negativeIdxList.push_back(60); negativeIdxList.push_back(61); positiveIdxList.push_back(62);
	positiveIdxList.push_back(63);
	DetectionInfo.rg[labelNum].feature.haar.HaarValue[idx++] = getHaarLikePatternValue(block_sum, &positiveIdxList, &negativeIdxList);*/

	// 41
	/*positiveIdxList.push_back(0); positiveIdxList.push_back(1); negativeIdxList.push_back(2);
	negativeIdxList.push_back(3); positiveIdxList.push_back(4); positiveIdxList.push_back(5);
	positiveIdxList.push_back(6); positiveIdxList.push_back(7); positiveIdxList.push_back(8);
	positiveIdxList.push_back(9); negativeIdxList.push_back(10); negativeIdxList.push_back(11);
	positiveIdxList.push_back(12); positiveIdxList.push_back(13); positiveIdxList.push_back(14);
	positiveIdxList.push_back(15); positiveIdxList.push_back(16); positiveIdxList.push_back(17);
	negativeIdxList.push_back(18); negativeIdxList.push_back(19); positiveIdxList.push_back(20);
	positiveIdxList.push_back(21); positiveIdxList.push_back(22); positiveIdxList.push_back(23);
	negativeIdxList.push_back(24); negativeIdxList.push_back(25); positiveIdxList.push_back(26);
	positiveIdxList.push_back(27); negativeIdxList.push_back(28); negativeIdxList.push_back(29);
	negativeIdxList.push_back(30); negativeIdxList.push_back(31); negativeIdxList.push_back(32);
	negativeIdxList.push_back(33); positiveIdxList.push_back(34); positiveIdxList.push_back(35);
	negativeIdxList.push_back(36); negativeIdxList.push_back(37); negativeIdxList.push_back(38);
	negativeIdxList.push_back(39); positiveIdxList.push_back(40); positiveIdxList.push_back(41);
	negativeIdxList.push_back(42); negativeIdxList.push_back(43); positiveIdxList.push_back(44);
	positiveIdxList.push_back(45); positiveIdxList.push_back(46); positiveIdxList.push_back(47);
	positiveIdxList.push_back(48); positiveIdxList.push_back(49); negativeIdxList.push_back(50);
	negativeIdxList.push_back(51); positiveIdxList.push_back(52); positiveIdxList.push_back(53);
	positiveIdxList.push_back(54); positiveIdxList.push_back(55); positiveIdxList.push_back(56);
	positiveIdxList.push_back(57); negativeIdxList.push_back(58); negativeIdxList.push_back(59);
	positiveIdxList.push_back(60); positiveIdxList.push_back(61); positiveIdxList.push_back(62);
	positiveIdxList.push_back(63);
	DetectionInfo.rg[labelNum].feature.haar.HaarValue[idx++] = getHaarLikePatternValue(block_sum, &positiveIdxList, &negativeIdxList);*/

	// 42
	/*positiveIdxList.push_back(0); positiveIdxList.push_back(1); negativeIdxList.push_back(2);
	negativeIdxList.push_back(3); negativeIdxList.push_back(4); negativeIdxList.push_back(5);
	positiveIdxList.push_back(6); positiveIdxList.push_back(7); positiveIdxList.push_back(8);
	positiveIdxList.push_back(9); negativeIdxList.push_back(10); negativeIdxList.push_back(11);
	negativeIdxList.push_back(12); negativeIdxList.push_back(13); positiveIdxList.push_back(14);
	positiveIdxList.push_back(15); negativeIdxList.push_back(16); negativeIdxList.push_back(17);
	positiveIdxList.push_back(18); positiveIdxList.push_back(19); positiveIdxList.push_back(20);
	positiveIdxList.push_back(21); negativeIdxList.push_back(22); negativeIdxList.push_back(23);
	negativeIdxList.push_back(24); negativeIdxList.push_back(25); positiveIdxList.push_back(26);
	positiveIdxList.push_back(27); positiveIdxList.push_back(28); positiveIdxList.push_back(29);
	negativeIdxList.push_back(30); negativeIdxList.push_back(31); positiveIdxList.push_back(32);
	positiveIdxList.push_back(33); negativeIdxList.push_back(34); negativeIdxList.push_back(35);
	negativeIdxList.push_back(36); negativeIdxList.push_back(37); positiveIdxList.push_back(38);
	positiveIdxList.push_back(39); positiveIdxList.push_back(40); positiveIdxList.push_back(41);
	negativeIdxList.push_back(42); negativeIdxList.push_back(43); negativeIdxList.push_back(44);
	negativeIdxList.push_back(45); positiveIdxList.push_back(46); positiveIdxList.push_back(47);
	positiveIdxList.push_back(48); positiveIdxList.push_back(49); negativeIdxList.push_back(50);
	negativeIdxList.push_back(51); negativeIdxList.push_back(52); negativeIdxList.push_back(53);
	positiveIdxList.push_back(54); positiveIdxList.push_back(55); positiveIdxList.push_back(56);
	positiveIdxList.push_back(57); negativeIdxList.push_back(58); negativeIdxList.push_back(59);
	negativeIdxList.push_back(60); negativeIdxList.push_back(61); positiveIdxList.push_back(62);
	positiveIdxList.push_back(63);
	DetectionInfo.rg[labelNum].feature.haar.HaarValue[idx++] = getHaarLikePatternValue(block_sum, &positiveIdxList, &negativeIdxList);*/

	// 43
	/*positiveIdxList.push_back(0); positiveIdxList.push_back(1); negativeIdxList.push_back(2);
	negativeIdxList.push_back(3); negativeIdxList.push_back(4); negativeIdxList.push_back(5);
	positiveIdxList.push_back(6); positiveIdxList.push_back(7); positiveIdxList.push_back(8);
	positiveIdxList.push_back(9); negativeIdxList.push_back(10); negativeIdxList.push_back(11);
	negativeIdxList.push_back(12); negativeIdxList.push_back(13); positiveIdxList.push_back(14);
	positiveIdxList.push_back(15); positiveIdxList.push_back(16); positiveIdxList.push_back(17);
	negativeIdxList.push_back(18); negativeIdxList.push_back(19); negativeIdxList.push_back(20);
	negativeIdxList.push_back(21); positiveIdxList.push_back(22); positiveIdxList.push_back(23);
	positiveIdxList.push_back(24); positiveIdxList.push_back(25); negativeIdxList.push_back(26);
	negativeIdxList.push_back(27); negativeIdxList.push_back(28); negativeIdxList.push_back(29);
	positiveIdxList.push_back(30); positiveIdxList.push_back(31); negativeIdxList.push_back(32);
	negativeIdxList.push_back(33); positiveIdxList.push_back(34); positiveIdxList.push_back(35);
	positiveIdxList.push_back(36); positiveIdxList.push_back(37); negativeIdxList.push_back(38);
	negativeIdxList.push_back(39); negativeIdxList.push_back(40); negativeIdxList.push_back(41);
	positiveIdxList.push_back(42); positiveIdxList.push_back(43); positiveIdxList.push_back(44);
	positiveIdxList.push_back(45); negativeIdxList.push_back(46); negativeIdxList.push_back(47);
	positiveIdxList.push_back(48); positiveIdxList.push_back(49); negativeIdxList.push_back(50);
	negativeIdxList.push_back(51); negativeIdxList.push_back(52); negativeIdxList.push_back(53);
	positiveIdxList.push_back(54); positiveIdxList.push_back(55); positiveIdxList.push_back(56);
	positiveIdxList.push_back(57); negativeIdxList.push_back(58); negativeIdxList.push_back(59);
	negativeIdxList.push_back(60); negativeIdxList.push_back(61); positiveIdxList.push_back(62);
	positiveIdxList.push_back(63);
	DetectionInfo.rg[labelNum].feature.haar.HaarValue[idx++] = getHaarLikePatternValue(block_sum, &positiveIdxList, &negativeIdxList);*/

	// 44
	/*positiveIdxList.push_back(0); positiveIdxList.push_back(1); positiveIdxList.push_back(2);
	negativeIdxList.push_back(3); negativeIdxList.push_back(4); positiveIdxList.push_back(5);
	positiveIdxList.push_back(6); positiveIdxList.push_back(7); positiveIdxList.push_back(8);
	positiveIdxList.push_back(9); positiveIdxList.push_back(10); negativeIdxList.push_back(11);
	negativeIdxList.push_back(12); positiveIdxList.push_back(13); positiveIdxList.push_back(14);
	positiveIdxList.push_back(15); positiveIdxList.push_back(16); positiveIdxList.push_back(17);
	positiveIdxList.push_back(18); negativeIdxList.push_back(19); negativeIdxList.push_back(20);
	positiveIdxList.push_back(21); positiveIdxList.push_back(22); positiveIdxList.push_back(23);
	positiveIdxList.push_back(24); positiveIdxList.push_back(25); positiveIdxList.push_back(26);
	negativeIdxList.push_back(27); negativeIdxList.push_back(28); positiveIdxList.push_back(29);
	positiveIdxList.push_back(30); positiveIdxList.push_back(31); negativeIdxList.push_back(32);
	negativeIdxList.push_back(33); negativeIdxList.push_back(34); positiveIdxList.push_back(35);
	positiveIdxList.push_back(36); negativeIdxList.push_back(37); negativeIdxList.push_back(38);
	negativeIdxList.push_back(39); negativeIdxList.push_back(40); negativeIdxList.push_back(41);
	negativeIdxList.push_back(42); positiveIdxList.push_back(43); positiveIdxList.push_back(44);
	negativeIdxList.push_back(45); negativeIdxList.push_back(46); negativeIdxList.push_back(47);
	positiveIdxList.push_back(48); positiveIdxList.push_back(49); positiveIdxList.push_back(50);
	negativeIdxList.push_back(51); negativeIdxList.push_back(52); positiveIdxList.push_back(53);
	positiveIdxList.push_back(54); positiveIdxList.push_back(55); positiveIdxList.push_back(56);
	positiveIdxList.push_back(57); positiveIdxList.push_back(58); negativeIdxList.push_back(59);
	negativeIdxList.push_back(60); positiveIdxList.push_back(61); positiveIdxList.push_back(62);
	positiveIdxList.push_back(63);
	DetectionInfo.rg[labelNum].feature.haar.HaarValue[idx++] = getHaarLikePatternValue(block_sum, &positiveIdxList, &negativeIdxList);*/

	// 45
	/*positiveIdxList.push_back(0); positiveIdxList.push_back(1); positiveIdxList.push_back(2);
	positiveIdxList.push_back(3); negativeIdxList.push_back(4); negativeIdxList.push_back(5);
	positiveIdxList.push_back(6); positiveIdxList.push_back(7); positiveIdxList.push_back(8);
	positiveIdxList.push_back(9); positiveIdxList.push_back(10); positiveIdxList.push_back(11);
	negativeIdxList.push_back(12); negativeIdxList.push_back(13); positiveIdxList.push_back(14);
	positiveIdxList.push_back(15); positiveIdxList.push_back(16); positiveIdxList.push_back(17);
	positiveIdxList.push_back(18); positiveIdxList.push_back(19); negativeIdxList.push_back(20);
	negativeIdxList.push_back(21); positiveIdxList.push_back(22); positiveIdxList.push_back(23);
	negativeIdxList.push_back(24); negativeIdxList.push_back(25); negativeIdxList.push_back(26);
	negativeIdxList.push_back(27); positiveIdxList.push_back(28); positiveIdxList.push_back(29);
	negativeIdxList.push_back(30); negativeIdxList.push_back(31); negativeIdxList.push_back(32);
	negativeIdxList.push_back(33); negativeIdxList.push_back(34); negativeIdxList.push_back(35);
	positiveIdxList.push_back(36); positiveIdxList.push_back(37); negativeIdxList.push_back(38);
	negativeIdxList.push_back(39); positiveIdxList.push_back(40); positiveIdxList.push_back(41);
	positiveIdxList.push_back(42); positiveIdxList.push_back(43); negativeIdxList.push_back(44);
	negativeIdxList.push_back(45); positiveIdxList.push_back(46); positiveIdxList.push_back(47);
	positiveIdxList.push_back(48); positiveIdxList.push_back(49); positiveIdxList.push_back(50);
	positiveIdxList.push_back(51); negativeIdxList.push_back(52); negativeIdxList.push_back(53);
	positiveIdxList.push_back(54); positiveIdxList.push_back(55); positiveIdxList.push_back(56);
	positiveIdxList.push_back(57); positiveIdxList.push_back(58); positiveIdxList.push_back(59);
	negativeIdxList.push_back(60); negativeIdxList.push_back(61); positiveIdxList.push_back(62);
	positiveIdxList.push_back(63);
	DetectionInfo.rg[labelNum].feature.haar.HaarValue[idx++] = getHaarLikePatternValue(block_sum, &positiveIdxList, &negativeIdxList);*/

	// 46
	/*positiveIdxList.push_back(0); positiveIdxList.push_back(1); positiveIdxList.push_back(2);
	negativeIdxList.push_back(3); negativeIdxList.push_back(4); positiveIdxList.push_back(5);
	positiveIdxList.push_back(6); positiveIdxList.push_back(7); positiveIdxList.push_back(8);
	positiveIdxList.push_back(9); positiveIdxList.push_back(10); negativeIdxList.push_back(11);
	negativeIdxList.push_back(12); positiveIdxList.push_back(13); positiveIdxList.push_back(14);
	positiveIdxList.push_back(15); negativeIdxList.push_back(16); negativeIdxList.push_back(17);
	negativeIdxList.push_back(18); positiveIdxList.push_back(19); positiveIdxList.push_back(20);
	negativeIdxList.push_back(21); negativeIdxList.push_back(22); negativeIdxList.push_back(23);
	negativeIdxList.push_back(24); negativeIdxList.push_back(25); negativeIdxList.push_back(26);
	positiveIdxList.push_back(27); positiveIdxList.push_back(28); negativeIdxList.push_back(29);
	negativeIdxList.push_back(30); negativeIdxList.push_back(31); positiveIdxList.push_back(32);
	positiveIdxList.push_back(33); positiveIdxList.push_back(34); negativeIdxList.push_back(35);
	negativeIdxList.push_back(36); positiveIdxList.push_back(37); positiveIdxList.push_back(38);
	positiveIdxList.push_back(39); positiveIdxList.push_back(40); positiveIdxList.push_back(41);
	positiveIdxList.push_back(42); negativeIdxList.push_back(43); negativeIdxList.push_back(44);
	positiveIdxList.push_back(45); positiveIdxList.push_back(46); positiveIdxList.push_back(47);
	positiveIdxList.push_back(48); positiveIdxList.push_back(49); positiveIdxList.push_back(50);
	negativeIdxList.push_back(51); negativeIdxList.push_back(52); positiveIdxList.push_back(53);
	positiveIdxList.push_back(54); positiveIdxList.push_back(55); positiveIdxList.push_back(56);
	positiveIdxList.push_back(57); positiveIdxList.push_back(58); negativeIdxList.push_back(59);
	negativeIdxList.push_back(60); positiveIdxList.push_back(61); positiveIdxList.push_back(62);
	positiveIdxList.push_back(63);
	DetectionInfo.rg[labelNum].feature.haar.HaarValue[idx++] = getHaarLikePatternValue(block_sum, &positiveIdxList, &negativeIdxList);*/

	// 47
	positiveIdxList.push_back(0); positiveIdxList.push_back(1); positiveIdxList.push_back(2);
	positiveIdxList.push_back(3); positiveIdxList.push_back(4); positiveIdxList.push_back(5);
	negativeIdxList.push_back(6); negativeIdxList.push_back(7); positiveIdxList.push_back(8);
	positiveIdxList.push_back(9); positiveIdxList.push_back(10); positiveIdxList.push_back(11);
	positiveIdxList.push_back(12); positiveIdxList.push_back(13); negativeIdxList.push_back(14);
	negativeIdxList.push_back(15); negativeIdxList.push_back(16); negativeIdxList.push_back(17);
	negativeIdxList.push_back(18); negativeIdxList.push_back(19); negativeIdxList.push_back(20);
	negativeIdxList.push_back(21); negativeIdxList.push_back(22); negativeIdxList.push_back(23);
	negativeIdxList.push_back(24); negativeIdxList.push_back(25); negativeIdxList.push_back(26);
	negativeIdxList.push_back(27); negativeIdxList.push_back(28); negativeIdxList.push_back(29);
	negativeIdxList.push_back(30); negativeIdxList.push_back(31); negativeIdxList.push_back(32);
	negativeIdxList.push_back(33); negativeIdxList.push_back(34); negativeIdxList.push_back(35);
	negativeIdxList.push_back(36); negativeIdxList.push_back(37); negativeIdxList.push_back(38);
	negativeIdxList.push_back(39); negativeIdxList.push_back(40); negativeIdxList.push_back(41);
	negativeIdxList.push_back(42); negativeIdxList.push_back(43); negativeIdxList.push_back(44);
	negativeIdxList.push_back(45); negativeIdxList.push_back(46); negativeIdxList.push_back(47);
	negativeIdxList.push_back(48); negativeIdxList.push_back(49); positiveIdxList.push_back(50);
	positiveIdxList.push_back(51); positiveIdxList.push_back(52); positiveIdxList.push_back(53);
	positiveIdxList.push_back(54); positiveIdxList.push_back(55); negativeIdxList.push_back(56);
	negativeIdxList.push_back(57); positiveIdxList.push_back(58); positiveIdxList.push_back(59);
	positiveIdxList.push_back(60); positiveIdxList.push_back(61); positiveIdxList.push_back(62);
	positiveIdxList.push_back(63);
	DetectionInfo.rg[labelNum].feature.haar.HaarValue[idx++] = getHaarLikePatternValue(block_sum, &positiveIdxList, &negativeIdxList);

	// 48
	negativeIdxList.push_back(0); negativeIdxList.push_back(1); positiveIdxList.push_back(2);
	positiveIdxList.push_back(3); positiveIdxList.push_back(4); positiveIdxList.push_back(5);
	positiveIdxList.push_back(6); positiveIdxList.push_back(7); negativeIdxList.push_back(8);
	negativeIdxList.push_back(9); positiveIdxList.push_back(10); positiveIdxList.push_back(11);
	positiveIdxList.push_back(12); positiveIdxList.push_back(13); positiveIdxList.push_back(14);
	positiveIdxList.push_back(15); negativeIdxList.push_back(16); negativeIdxList.push_back(17);
	negativeIdxList.push_back(18); negativeIdxList.push_back(19); negativeIdxList.push_back(20);
	negativeIdxList.push_back(21); negativeIdxList.push_back(22); negativeIdxList.push_back(23);
	negativeIdxList.push_back(24); negativeIdxList.push_back(25); negativeIdxList.push_back(26);
	negativeIdxList.push_back(27); negativeIdxList.push_back(28); negativeIdxList.push_back(29);
	negativeIdxList.push_back(30); negativeIdxList.push_back(31); negativeIdxList.push_back(32);
	negativeIdxList.push_back(33); negativeIdxList.push_back(34); negativeIdxList.push_back(35);
	negativeIdxList.push_back(36); negativeIdxList.push_back(37); negativeIdxList.push_back(38);
	negativeIdxList.push_back(39); negativeIdxList.push_back(40); negativeIdxList.push_back(41);
	negativeIdxList.push_back(42); negativeIdxList.push_back(43); negativeIdxList.push_back(44);
	negativeIdxList.push_back(45); negativeIdxList.push_back(46); negativeIdxList.push_back(47);
	positiveIdxList.push_back(48); positiveIdxList.push_back(49); positiveIdxList.push_back(50);
	positiveIdxList.push_back(51); positiveIdxList.push_back(52); positiveIdxList.push_back(53);
	negativeIdxList.push_back(54); negativeIdxList.push_back(55); positiveIdxList.push_back(56);
	positiveIdxList.push_back(57); positiveIdxList.push_back(58); positiveIdxList.push_back(59);
	positiveIdxList.push_back(60); positiveIdxList.push_back(61); negativeIdxList.push_back(62);
	negativeIdxList.push_back(63);
	DetectionInfo.rg[labelNum].feature.haar.HaarValue[idx++] = getHaarLikePatternValue(block_sum, &positiveIdxList, &negativeIdxList);

	// 49
	/*positiveIdxList.push_back(0); positiveIdxList.push_back(1); negativeIdxList.push_back(2);
	negativeIdxList.push_back(3); negativeIdxList.push_back(4); negativeIdxList.push_back(5);
	negativeIdxList.push_back(6); negativeIdxList.push_back(7); positiveIdxList.push_back(8);
	positiveIdxList.push_back(9); negativeIdxList.push_back(10); negativeIdxList.push_back(11);
	negativeIdxList.push_back(12); negativeIdxList.push_back(13); negativeIdxList.push_back(14);
	negativeIdxList.push_back(15); positiveIdxList.push_back(16); positiveIdxList.push_back(17);
	negativeIdxList.push_back(18); negativeIdxList.push_back(19); negativeIdxList.push_back(20);
	negativeIdxList.push_back(21); positiveIdxList.push_back(22); positiveIdxList.push_back(23);
	positiveIdxList.push_back(24); positiveIdxList.push_back(25); negativeIdxList.push_back(26);
	negativeIdxList.push_back(27); negativeIdxList.push_back(28); negativeIdxList.push_back(29);
	positiveIdxList.push_back(30); positiveIdxList.push_back(31); positiveIdxList.push_back(32);
	positiveIdxList.push_back(33); negativeIdxList.push_back(34); negativeIdxList.push_back(35);
	negativeIdxList.push_back(36); negativeIdxList.push_back(37); positiveIdxList.push_back(38);
	positiveIdxList.push_back(39); positiveIdxList.push_back(40); positiveIdxList.push_back(41);
	negativeIdxList.push_back(42); negativeIdxList.push_back(43); negativeIdxList.push_back(44);
	negativeIdxList.push_back(45); positiveIdxList.push_back(46); positiveIdxList.push_back(47);
	negativeIdxList.push_back(48); negativeIdxList.push_back(49); negativeIdxList.push_back(50);
	negativeIdxList.push_back(51); negativeIdxList.push_back(52); negativeIdxList.push_back(53);
	positiveIdxList.push_back(54); positiveIdxList.push_back(55); negativeIdxList.push_back(56);
	negativeIdxList.push_back(57); negativeIdxList.push_back(58); negativeIdxList.push_back(59);
	negativeIdxList.push_back(60); negativeIdxList.push_back(61); positiveIdxList.push_back(62);
	positiveIdxList.push_back(63);
	DetectionInfo.rg[labelNum].feature.haar.HaarValue[idx++] = getHaarLikePatternValue(block_sum, &positiveIdxList, &negativeIdxList);*/

	// 50
	/*negativeIdxList.push_back(0); negativeIdxList.push_back(1); negativeIdxList.push_back(2);
	negativeIdxList.push_back(3); negativeIdxList.push_back(4); negativeIdxList.push_back(5);
	positiveIdxList.push_back(6); positiveIdxList.push_back(7); negativeIdxList.push_back(8);
	negativeIdxList.push_back(9); negativeIdxList.push_back(10); negativeIdxList.push_back(11);
	negativeIdxList.push_back(12); negativeIdxList.push_back(13); positiveIdxList.push_back(14);
	positiveIdxList.push_back(15); positiveIdxList.push_back(16); positiveIdxList.push_back(17);
	negativeIdxList.push_back(18); negativeIdxList.push_back(19); negativeIdxList.push_back(20);
	negativeIdxList.push_back(21); positiveIdxList.push_back(22); positiveIdxList.push_back(23);
	positiveIdxList.push_back(24); positiveIdxList.push_back(25); negativeIdxList.push_back(26);
	negativeIdxList.push_back(27); negativeIdxList.push_back(28); negativeIdxList.push_back(29);
	positiveIdxList.push_back(30); positiveIdxList.push_back(31); positiveIdxList.push_back(32);
	positiveIdxList.push_back(33); negativeIdxList.push_back(34); negativeIdxList.push_back(35);
	negativeIdxList.push_back(36); negativeIdxList.push_back(37); positiveIdxList.push_back(38);
	positiveIdxList.push_back(39); positiveIdxList.push_back(40); positiveIdxList.push_back(41);
	negativeIdxList.push_back(42); negativeIdxList.push_back(43); negativeIdxList.push_back(44);
	negativeIdxList.push_back(45); positiveIdxList.push_back(46); positiveIdxList.push_back(47);
	positiveIdxList.push_back(48); positiveIdxList.push_back(49); negativeIdxList.push_back(50);
	negativeIdxList.push_back(51); negativeIdxList.push_back(52); negativeIdxList.push_back(53);
	negativeIdxList.push_back(54); negativeIdxList.push_back(55); positiveIdxList.push_back(56);
	positiveIdxList.push_back(57); negativeIdxList.push_back(58); negativeIdxList.push_back(59);
	negativeIdxList.push_back(60); negativeIdxList.push_back(61); negativeIdxList.push_back(62);
	negativeIdxList.push_back(63);
	DetectionInfo.rg[labelNum].feature.haar.HaarValue[idx++] = getHaarLikePatternValue(block_sum, &positiveIdxList, &negativeIdxList);*/

	// 51
	positiveIdxList.push_back(0); positiveIdxList.push_back(1); negativeIdxList.push_back(2);
	negativeIdxList.push_back(3); negativeIdxList.push_back(4); negativeIdxList.push_back(5);
	positiveIdxList.push_back(6); positiveIdxList.push_back(7); positiveIdxList.push_back(8);
	positiveIdxList.push_back(9); negativeIdxList.push_back(10); negativeIdxList.push_back(11);
	negativeIdxList.push_back(12); negativeIdxList.push_back(13); positiveIdxList.push_back(14);
	positiveIdxList.push_back(15); negativeIdxList.push_back(16); negativeIdxList.push_back(17);
	positiveIdxList.push_back(18); positiveIdxList.push_back(19); positiveIdxList.push_back(20);
	positiveIdxList.push_back(21); negativeIdxList.push_back(22); negativeIdxList.push_back(23);
	negativeIdxList.push_back(24); negativeIdxList.push_back(25); positiveIdxList.push_back(26);
	positiveIdxList.push_back(27); positiveIdxList.push_back(28); positiveIdxList.push_back(29);
	negativeIdxList.push_back(30); negativeIdxList.push_back(31); negativeIdxList.push_back(32);
	negativeIdxList.push_back(33); positiveIdxList.push_back(34); positiveIdxList.push_back(35);
	positiveIdxList.push_back(36); positiveIdxList.push_back(37); negativeIdxList.push_back(38);
	negativeIdxList.push_back(39); negativeIdxList.push_back(40); negativeIdxList.push_back(41);
	positiveIdxList.push_back(42); positiveIdxList.push_back(43); positiveIdxList.push_back(44);
	positiveIdxList.push_back(45); negativeIdxList.push_back(46); negativeIdxList.push_back(47);
	positiveIdxList.push_back(48); positiveIdxList.push_back(49); negativeIdxList.push_back(50);
	negativeIdxList.push_back(51); negativeIdxList.push_back(52); negativeIdxList.push_back(53);
	positiveIdxList.push_back(54); positiveIdxList.push_back(55); positiveIdxList.push_back(56);
	positiveIdxList.push_back(57); negativeIdxList.push_back(58); negativeIdxList.push_back(59);
	negativeIdxList.push_back(60); negativeIdxList.push_back(61); positiveIdxList.push_back(62);
	positiveIdxList.push_back(63);
	DetectionInfo.rg[labelNum].feature.haar.HaarValue[idx++] = getHaarLikePatternValue(block_sum, &positiveIdxList, &negativeIdxList);

	// 52
	//negativeIdxList.push_back(0); negativeIdxList.push_back(1); negativeIdxList.push_back(2);
	//negativeIdxList.push_back(3); positiveIdxList.push_back(4); positiveIdxList.push_back(5);
	//positiveIdxList.push_back(6); positiveIdxList.push_back(7); negativeIdxList.push_back(8);
	//negativeIdxList.push_back(9); negativeIdxList.push_back(10); negativeIdxList.push_back(11);
	//positiveIdxList.push_back(12); positiveIdxList.push_back(13); positiveIdxList.push_back(14);
	//positiveIdxList.push_back(15); positiveIdxList.push_back(16); positiveIdxList.push_back(17);
	//positiveIdxList.push_back(18); positiveIdxList.push_back(19); negativeIdxList.push_back(20);
	//negativeIdxList.push_back(21); negativeIdxList.push_back(22); negativeIdxList.push_back(23);
	//positiveIdxList.push_back(24); positiveIdxList.push_back(25); positiveIdxList.push_back(26);
	//positiveIdxList.push_back(27); negativeIdxList.push_back(28); negativeIdxList.push_back(29);
	//negativeIdxList.push_back(30); negativeIdxList.push_back(31); negativeIdxList.push_back(32);
	//negativeIdxList.push_back(33); negativeIdxList.push_back(34); negativeIdxList.push_back(35);
	//positiveIdxList.push_back(36); positiveIdxList.push_back(37); positiveIdxList.push_back(38);
	//positiveIdxList.push_back(39); negativeIdxList.push_back(40); negativeIdxList.push_back(41);
	//negativeIdxList.push_back(42); negativeIdxList.push_back(43); positiveIdxList.push_back(44);
	//positiveIdxList.push_back(45); positiveIdxList.push_back(46); positiveIdxList.push_back(47);
	//positiveIdxList.push_back(48); positiveIdxList.push_back(49); positiveIdxList.push_back(50);
	//positiveIdxList.push_back(51); negativeIdxList.push_back(52); negativeIdxList.push_back(53);
	//negativeIdxList.push_back(54); negativeIdxList.push_back(55); positiveIdxList.push_back(56);
	//positiveIdxList.push_back(57); positiveIdxList.push_back(58); positiveIdxList.push_back(59);
	//negativeIdxList.push_back(60); negativeIdxList.push_back(61); negativeIdxList.push_back(62);
	//negativeIdxList.push_back(63);
	//DetectionInfo.rg[labelNum].feature.haar.HaarValue[idx++] = getHaarLikePatternValue(block_sum, &positiveIdxList, &negativeIdxList);

	//// 53
	//positiveIdxList.push_back(0); positiveIdxList.push_back(1); negativeIdxList.push_back(2);
	//negativeIdxList.push_back(3); positiveIdxList.push_back(4); positiveIdxList.push_back(5);
	//negativeIdxList.push_back(6); negativeIdxList.push_back(7); positiveIdxList.push_back(8);
	//positiveIdxList.push_back(9); negativeIdxList.push_back(10); negativeIdxList.push_back(11);
	//positiveIdxList.push_back(12); positiveIdxList.push_back(13); negativeIdxList.push_back(14);
	//negativeIdxList.push_back(15); positiveIdxList.push_back(16); positiveIdxList.push_back(17);
	//negativeIdxList.push_back(18); negativeIdxList.push_back(19); positiveIdxList.push_back(20);
	//positiveIdxList.push_back(21); negativeIdxList.push_back(22); negativeIdxList.push_back(23);
	//positiveIdxList.push_back(24); positiveIdxList.push_back(25); negativeIdxList.push_back(26);
	//negativeIdxList.push_back(27); positiveIdxList.push_back(28); positiveIdxList.push_back(29);
	//negativeIdxList.push_back(30); negativeIdxList.push_back(31); negativeIdxList.push_back(32);
	//negativeIdxList.push_back(33); positiveIdxList.push_back(34); positiveIdxList.push_back(35);
	//negativeIdxList.push_back(36); negativeIdxList.push_back(37); positiveIdxList.push_back(38);
	//positiveIdxList.push_back(39); negativeIdxList.push_back(40); negativeIdxList.push_back(41);
	//positiveIdxList.push_back(42); positiveIdxList.push_back(43); negativeIdxList.push_back(44);
	//negativeIdxList.push_back(45); positiveIdxList.push_back(46); positiveIdxList.push_back(47);
	//negativeIdxList.push_back(48); negativeIdxList.push_back(49); positiveIdxList.push_back(50);
	//positiveIdxList.push_back(51); negativeIdxList.push_back(52); negativeIdxList.push_back(53);
	//positiveIdxList.push_back(54); positiveIdxList.push_back(55); negativeIdxList.push_back(56);
	//negativeIdxList.push_back(57); positiveIdxList.push_back(58); positiveIdxList.push_back(59);
	//negativeIdxList.push_back(60); negativeIdxList.push_back(61); positiveIdxList.push_back(62);
	//positiveIdxList.push_back(63);
	//DetectionInfo.rg[labelNum].feature.haar.HaarValue[idx++] = getHaarLikePatternValue(block_sum, &positiveIdxList, &negativeIdxList);

	//// 54
	//negativeIdxList.push_back(0); positiveIdxList.push_back(1); positiveIdxList.push_back(2);
	//positiveIdxList.push_back(3); positiveIdxList.push_back(4); positiveIdxList.push_back(5);
	//positiveIdxList.push_back(6); negativeIdxList.push_back(7); negativeIdxList.push_back(8);
	//positiveIdxList.push_back(9); positiveIdxList.push_back(10); positiveIdxList.push_back(11);
	//positiveIdxList.push_back(12); positiveIdxList.push_back(13); positiveIdxList.push_back(14);
	//negativeIdxList.push_back(15); positiveIdxList.push_back(16); negativeIdxList.push_back(17);
	//negativeIdxList.push_back(18); negativeIdxList.push_back(19); negativeIdxList.push_back(20);
	//negativeIdxList.push_back(21); negativeIdxList.push_back(22); positiveIdxList.push_back(23);
	//positiveIdxList.push_back(24); negativeIdxList.push_back(25); negativeIdxList.push_back(26);
	//negativeIdxList.push_back(27); negativeIdxList.push_back(28); negativeIdxList.push_back(29);
	//negativeIdxList.push_back(30); positiveIdxList.push_back(31); positiveIdxList.push_back(32);
	//negativeIdxList.push_back(33); negativeIdxList.push_back(34); negativeIdxList.push_back(35);
	//negativeIdxList.push_back(36); negativeIdxList.push_back(37); negativeIdxList.push_back(38);
	//positiveIdxList.push_back(39); positiveIdxList.push_back(40); negativeIdxList.push_back(41);
	//negativeIdxList.push_back(42); negativeIdxList.push_back(43); negativeIdxList.push_back(44);
	//negativeIdxList.push_back(45); negativeIdxList.push_back(46); positiveIdxList.push_back(47);
	//negativeIdxList.push_back(48); positiveIdxList.push_back(49); positiveIdxList.push_back(50);
	//positiveIdxList.push_back(51); positiveIdxList.push_back(52); positiveIdxList.push_back(53);
	//positiveIdxList.push_back(54); negativeIdxList.push_back(55); negativeIdxList.push_back(56);
	//positiveIdxList.push_back(57); positiveIdxList.push_back(58); positiveIdxList.push_back(59);
	//positiveIdxList.push_back(60); positiveIdxList.push_back(61); positiveIdxList.push_back(62);
	//negativeIdxList.push_back(63);
	//DetectionInfo.rg[labelNum].feature.haar.HaarValue[idx++] = getHaarLikePatternValue(block_sum, &positiveIdxList, &negativeIdxList);

	//// 55
	//negativeIdxList.push_back(0); negativeIdxList.push_back(1); positiveIdxList.push_back(2);
	//positiveIdxList.push_back(3); positiveIdxList.push_back(4); positiveIdxList.push_back(5);
	//negativeIdxList.push_back(6); negativeIdxList.push_back(7); positiveIdxList.push_back(8);
	//positiveIdxList.push_back(9); negativeIdxList.push_back(10); negativeIdxList.push_back(11);
	//negativeIdxList.push_back(12); negativeIdxList.push_back(13); positiveIdxList.push_back(14);
	//positiveIdxList.push_back(15); positiveIdxList.push_back(16); positiveIdxList.push_back(17);
	//negativeIdxList.push_back(18); negativeIdxList.push_back(19); negativeIdxList.push_back(20);
	//negativeIdxList.push_back(21); positiveIdxList.push_back(22); positiveIdxList.push_back(23);
	//positiveIdxList.push_back(24); positiveIdxList.push_back(25); negativeIdxList.push_back(26);
	//negativeIdxList.push_back(27); negativeIdxList.push_back(28); negativeIdxList.push_back(29);
	//positiveIdxList.push_back(30); positiveIdxList.push_back(31); positiveIdxList.push_back(32);
	//positiveIdxList.push_back(33); negativeIdxList.push_back(34); negativeIdxList.push_back(35);
	//negativeIdxList.push_back(36); negativeIdxList.push_back(37); positiveIdxList.push_back(38);
	//positiveIdxList.push_back(39); positiveIdxList.push_back(40); positiveIdxList.push_back(41);
	//negativeIdxList.push_back(42); negativeIdxList.push_back(43); negativeIdxList.push_back(44);
	//negativeIdxList.push_back(45); positiveIdxList.push_back(46); positiveIdxList.push_back(47);
	//positiveIdxList.push_back(48); positiveIdxList.push_back(49); negativeIdxList.push_back(50);
	//negativeIdxList.push_back(51); negativeIdxList.push_back(52); negativeIdxList.push_back(53);
	//positiveIdxList.push_back(54); positiveIdxList.push_back(55); negativeIdxList.push_back(56);
	//negativeIdxList.push_back(57); positiveIdxList.push_back(58); positiveIdxList.push_back(59);
	//positiveIdxList.push_back(60); positiveIdxList.push_back(61); negativeIdxList.push_back(62);
	//negativeIdxList.push_back(63);
	//DetectionInfo.rg[labelNum].feature.haar.HaarValue[idx++] = getHaarLikePatternValue(block_sum, &positiveIdxList, &negativeIdxList);

	//// 56
	//negativeIdxList.push_back(0); negativeIdxList.push_back(1); positiveIdxList.push_back(2);
	//positiveIdxList.push_back(3); positiveIdxList.push_back(4); positiveIdxList.push_back(5);
	//negativeIdxList.push_back(6); negativeIdxList.push_back(7); negativeIdxList.push_back(8);
	//negativeIdxList.push_back(9); positiveIdxList.push_back(10); positiveIdxList.push_back(11);
	//positiveIdxList.push_back(12); positiveIdxList.push_back(13); negativeIdxList.push_back(14);
	//negativeIdxList.push_back(15); positiveIdxList.push_back(16); positiveIdxList.push_back(17);
	//negativeIdxList.push_back(18); negativeIdxList.push_back(19); negativeIdxList.push_back(20);
	//negativeIdxList.push_back(21); positiveIdxList.push_back(22); positiveIdxList.push_back(23);
	//positiveIdxList.push_back(24); positiveIdxList.push_back(25); negativeIdxList.push_back(26);
	//negativeIdxList.push_back(27); negativeIdxList.push_back(28); negativeIdxList.push_back(29);
	//positiveIdxList.push_back(30); positiveIdxList.push_back(31); positiveIdxList.push_back(32);
	//positiveIdxList.push_back(33); negativeIdxList.push_back(34); negativeIdxList.push_back(35);
	//negativeIdxList.push_back(36); negativeIdxList.push_back(37); positiveIdxList.push_back(38);
	//positiveIdxList.push_back(39); positiveIdxList.push_back(40); positiveIdxList.push_back(41);
	//negativeIdxList.push_back(42); negativeIdxList.push_back(43); negativeIdxList.push_back(44);
	//negativeIdxList.push_back(45); positiveIdxList.push_back(46); positiveIdxList.push_back(47);
	//negativeIdxList.push_back(48); negativeIdxList.push_back(49); positiveIdxList.push_back(50);
	//positiveIdxList.push_back(51); positiveIdxList.push_back(52); positiveIdxList.push_back(53);
	//negativeIdxList.push_back(54); negativeIdxList.push_back(55); negativeIdxList.push_back(56);
	//negativeIdxList.push_back(57); positiveIdxList.push_back(58); positiveIdxList.push_back(59);
	//positiveIdxList.push_back(60); positiveIdxList.push_back(61); negativeIdxList.push_back(62);
	//negativeIdxList.push_back(63);
	//DetectionInfo.rg[labelNum].feature.haar.HaarValue[idx++] = getHaarLikePatternValue(block_sum, &positiveIdxList, &negativeIdxList);

	//// 57
	//positiveIdxList.push_back(0); positiveIdxList.push_back(1); negativeIdxList.push_back(2);
	//negativeIdxList.push_back(3); negativeIdxList.push_back(4); negativeIdxList.push_back(5);
	//positiveIdxList.push_back(6); positiveIdxList.push_back(7); positiveIdxList.push_back(8);
	//positiveIdxList.push_back(9); negativeIdxList.push_back(10); negativeIdxList.push_back(11);
	//negativeIdxList.push_back(12); negativeIdxList.push_back(13); positiveIdxList.push_back(14);
	//positiveIdxList.push_back(15); negativeIdxList.push_back(16); negativeIdxList.push_back(17);
	//positiveIdxList.push_back(18); positiveIdxList.push_back(19); positiveIdxList.push_back(20);
	//positiveIdxList.push_back(21); negativeIdxList.push_back(22); negativeIdxList.push_back(23);
	//negativeIdxList.push_back(24); negativeIdxList.push_back(25); positiveIdxList.push_back(26);
	//positiveIdxList.push_back(27); positiveIdxList.push_back(28); positiveIdxList.push_back(29);
	//negativeIdxList.push_back(30); negativeIdxList.push_back(31); positiveIdxList.push_back(32);
	//positiveIdxList.push_back(33); negativeIdxList.push_back(34); negativeIdxList.push_back(35);
	//negativeIdxList.push_back(36); negativeIdxList.push_back(37); positiveIdxList.push_back(38);
	//positiveIdxList.push_back(39); positiveIdxList.push_back(40); positiveIdxList.push_back(41);
	//negativeIdxList.push_back(42); negativeIdxList.push_back(43); negativeIdxList.push_back(44);
	//negativeIdxList.push_back(45); positiveIdxList.push_back(46); positiveIdxList.push_back(47);
	//negativeIdxList.push_back(48); negativeIdxList.push_back(49); positiveIdxList.push_back(50);
	//positiveIdxList.push_back(51); positiveIdxList.push_back(52); positiveIdxList.push_back(53);
	//negativeIdxList.push_back(54); negativeIdxList.push_back(55); negativeIdxList.push_back(56);
	//negativeIdxList.push_back(57); positiveIdxList.push_back(58); positiveIdxList.push_back(59);
	//positiveIdxList.push_back(60); positiveIdxList.push_back(61); negativeIdxList.push_back(62);
	//negativeIdxList.push_back(63);
	//DetectionInfo.rg[labelNum].feature.haar.HaarValue[idx++] = getHaarLikePatternValue(block_sum, &positiveIdxList, &negativeIdxList);

	//// 58
	//negativeIdxList.push_back(0); negativeIdxList.push_back(1); positiveIdxList.push_back(2);
	//positiveIdxList.push_back(3); negativeIdxList.push_back(4); negativeIdxList.push_back(5);
	//positiveIdxList.push_back(6); positiveIdxList.push_back(7); negativeIdxList.push_back(8);
	//negativeIdxList.push_back(9); positiveIdxList.push_back(10); positiveIdxList.push_back(11);
	//negativeIdxList.push_back(12); negativeIdxList.push_back(13); positiveIdxList.push_back(14);
	//positiveIdxList.push_back(15); negativeIdxList.push_back(16); negativeIdxList.push_back(17);
	//positiveIdxList.push_back(18); positiveIdxList.push_back(19); negativeIdxList.push_back(20);
	//negativeIdxList.push_back(21); positiveIdxList.push_back(22); positiveIdxList.push_back(23);
	//positiveIdxList.push_back(24); positiveIdxList.push_back(25); negativeIdxList.push_back(26);
	//negativeIdxList.push_back(27); positiveIdxList.push_back(28); positiveIdxList.push_back(29);
	//negativeIdxList.push_back(30); negativeIdxList.push_back(31); positiveIdxList.push_back(32);
	//positiveIdxList.push_back(33); negativeIdxList.push_back(34); negativeIdxList.push_back(35);
	//positiveIdxList.push_back(36); positiveIdxList.push_back(37); negativeIdxList.push_back(38);
	//negativeIdxList.push_back(39); negativeIdxList.push_back(40); negativeIdxList.push_back(41);
	//positiveIdxList.push_back(42); positiveIdxList.push_back(43); negativeIdxList.push_back(44);
	//negativeIdxList.push_back(45); positiveIdxList.push_back(46); positiveIdxList.push_back(47);
	//negativeIdxList.push_back(48); negativeIdxList.push_back(49); positiveIdxList.push_back(50);
	//positiveIdxList.push_back(51); negativeIdxList.push_back(52); negativeIdxList.push_back(53);
	//positiveIdxList.push_back(54); positiveIdxList.push_back(55); negativeIdxList.push_back(56);
	//negativeIdxList.push_back(57); positiveIdxList.push_back(58); positiveIdxList.push_back(59);
	//negativeIdxList.push_back(60); negativeIdxList.push_back(61); positiveIdxList.push_back(62);
	//positiveIdxList.push_back(63);
	//DetectionInfo.rg[labelNum].feature.haar.HaarValue[idx++] = getHaarLikePatternValue(block_sum, &positiveIdxList, &negativeIdxList);

	//// 59
	//negativeIdxList.push_back(0); negativeIdxList.push_back(1); positiveIdxList.push_back(2);
	//positiveIdxList.push_back(3); negativeIdxList.push_back(4); negativeIdxList.push_back(5);
	//positiveIdxList.push_back(6); positiveIdxList.push_back(7); negativeIdxList.push_back(8);
	//negativeIdxList.push_back(9); positiveIdxList.push_back(10); positiveIdxList.push_back(11);
	//negativeIdxList.push_back(12); negativeIdxList.push_back(13); positiveIdxList.push_back(14);
	//positiveIdxList.push_back(15); positiveIdxList.push_back(16); positiveIdxList.push_back(17);
	//negativeIdxList.push_back(18); negativeIdxList.push_back(19); positiveIdxList.push_back(20);
	//positiveIdxList.push_back(21); negativeIdxList.push_back(22); negativeIdxList.push_back(23);
	//positiveIdxList.push_back(24); positiveIdxList.push_back(25); negativeIdxList.push_back(26);
	//negativeIdxList.push_back(27); positiveIdxList.push_back(28); positiveIdxList.push_back(29);
	//negativeIdxList.push_back(30); negativeIdxList.push_back(31); positiveIdxList.push_back(32);
	//positiveIdxList.push_back(33); negativeIdxList.push_back(34); negativeIdxList.push_back(35);
	//positiveIdxList.push_back(36); positiveIdxList.push_back(37); negativeIdxList.push_back(38);
	//negativeIdxList.push_back(39); positiveIdxList.push_back(40); positiveIdxList.push_back(41);
	//negativeIdxList.push_back(42); negativeIdxList.push_back(43); positiveIdxList.push_back(44);
	//positiveIdxList.push_back(45); negativeIdxList.push_back(46); negativeIdxList.push_back(47);
	//negativeIdxList.push_back(48); negativeIdxList.push_back(49); positiveIdxList.push_back(50);
	//positiveIdxList.push_back(51); negativeIdxList.push_back(52); negativeIdxList.push_back(53);
	//positiveIdxList.push_back(54); positiveIdxList.push_back(55); negativeIdxList.push_back(56);
	//negativeIdxList.push_back(57); positiveIdxList.push_back(58); positiveIdxList.push_back(59);
	//negativeIdxList.push_back(60); negativeIdxList.push_back(61); positiveIdxList.push_back(62);
	//positiveIdxList.push_back(63);
	//DetectionInfo.rg[labelNum].feature.haar.HaarValue[idx++] = getHaarLikePatternValue(block_sum, &positiveIdxList, &negativeIdxList);

	//// 60
	//negativeIdxList.push_back(0); negativeIdxList.push_back(1); negativeIdxList.push_back(2);
	//positiveIdxList.push_back(3); positiveIdxList.push_back(4); negativeIdxList.push_back(5);
	//negativeIdxList.push_back(6); negativeIdxList.push_back(7); negativeIdxList.push_back(8);
	//negativeIdxList.push_back(9); negativeIdxList.push_back(10); positiveIdxList.push_back(11);
	//positiveIdxList.push_back(12); negativeIdxList.push_back(13); negativeIdxList.push_back(14);
	//negativeIdxList.push_back(15); positiveIdxList.push_back(16); positiveIdxList.push_back(17);
	//positiveIdxList.push_back(18); negativeIdxList.push_back(19); negativeIdxList.push_back(20);
	//positiveIdxList.push_back(21); positiveIdxList.push_back(22); positiveIdxList.push_back(23);
	//positiveIdxList.push_back(24); positiveIdxList.push_back(25); positiveIdxList.push_back(26);
	//negativeIdxList.push_back(27); negativeIdxList.push_back(28); positiveIdxList.push_back(29);
	//positiveIdxList.push_back(30); positiveIdxList.push_back(31); negativeIdxList.push_back(32);
	//negativeIdxList.push_back(33); negativeIdxList.push_back(34); positiveIdxList.push_back(35);
	//positiveIdxList.push_back(36); negativeIdxList.push_back(37); negativeIdxList.push_back(38);
	//negativeIdxList.push_back(39); negativeIdxList.push_back(40); negativeIdxList.push_back(41);
	//negativeIdxList.push_back(42); positiveIdxList.push_back(43); positiveIdxList.push_back(44);
	//negativeIdxList.push_back(45); negativeIdxList.push_back(46); negativeIdxList.push_back(47);
	//positiveIdxList.push_back(48); positiveIdxList.push_back(49); positiveIdxList.push_back(50);
	//negativeIdxList.push_back(51); negativeIdxList.push_back(52); positiveIdxList.push_back(53);
	//positiveIdxList.push_back(54); positiveIdxList.push_back(55); positiveIdxList.push_back(56);
	//positiveIdxList.push_back(57); positiveIdxList.push_back(58); negativeIdxList.push_back(59);
	//negativeIdxList.push_back(60); positiveIdxList.push_back(61); positiveIdxList.push_back(62);
	//positiveIdxList.push_back(63);
	//DetectionInfo.rg[labelNum].feature.haar.HaarValue[idx++] = getHaarLikePatternValue(block_sum, &positiveIdxList, &negativeIdxList);

	//// 61
	//positiveIdxList.push_back(0); positiveIdxList.push_back(1); negativeIdxList.push_back(2);
	//negativeIdxList.push_back(3); positiveIdxList.push_back(4); positiveIdxList.push_back(5);
	//negativeIdxList.push_back(6); negativeIdxList.push_back(7); positiveIdxList.push_back(8);
	//positiveIdxList.push_back(9); negativeIdxList.push_back(10); negativeIdxList.push_back(11);
	//positiveIdxList.push_back(12); positiveIdxList.push_back(13); negativeIdxList.push_back(14);
	//negativeIdxList.push_back(15); negativeIdxList.push_back(16); negativeIdxList.push_back(17);
	//positiveIdxList.push_back(18); positiveIdxList.push_back(19); negativeIdxList.push_back(20);
	//negativeIdxList.push_back(21); positiveIdxList.push_back(22); positiveIdxList.push_back(23);
	//negativeIdxList.push_back(24); negativeIdxList.push_back(25); positiveIdxList.push_back(26);
	//positiveIdxList.push_back(27); negativeIdxList.push_back(28); negativeIdxList.push_back(29);
	//positiveIdxList.push_back(30); positiveIdxList.push_back(31); positiveIdxList.push_back(32);
	//positiveIdxList.push_back(33); negativeIdxList.push_back(34); negativeIdxList.push_back(35);
	//positiveIdxList.push_back(36); positiveIdxList.push_back(37); negativeIdxList.push_back(38);
	//negativeIdxList.push_back(39); positiveIdxList.push_back(40); positiveIdxList.push_back(41);
	//negativeIdxList.push_back(42); negativeIdxList.push_back(43); positiveIdxList.push_back(44);
	//positiveIdxList.push_back(45); negativeIdxList.push_back(46); negativeIdxList.push_back(47);
	//negativeIdxList.push_back(48); negativeIdxList.push_back(49); positiveIdxList.push_back(50);
	//positiveIdxList.push_back(51); negativeIdxList.push_back(52); negativeIdxList.push_back(53);
	//positiveIdxList.push_back(54); positiveIdxList.push_back(55); negativeIdxList.push_back(56);
	//negativeIdxList.push_back(57); positiveIdxList.push_back(58); positiveIdxList.push_back(59);
	//negativeIdxList.push_back(60); negativeIdxList.push_back(61); positiveIdxList.push_back(62);
	//positiveIdxList.push_back(63);
	//DetectionInfo.rg[labelNum].feature.haar.HaarValue[idx++] = getHaarLikePatternValue(block_sum, &positiveIdxList, &negativeIdxList);


	ret = 0;
	return ret;
}

double CDetection::getHaarLikePatternValue(int* block_sum, std::vector<int>* positiveList, std::vector<int>* negativeList)
{
	//예외처리
	if (block_sum == nullptr) {
		Trace(_T("[%s,%d] nullptr\n"), _function_, _line_);
		positiveList->clear();
		negativeList->clear();
		return (double)0.0f;
	}
	double feature = 0.0f;
	int sum1, sum2;
	sum1 = sum2 = 0;
	int psize, nsize;
	psize = (int)positiveList->size();
	nsize = (int)negativeList->size();

	for (int i = 0; i < psize; i++)
		sum1 += block_sum[positiveList->at(i)];
	for (int i = 0; i < nsize; i++)
		sum2 += block_sum[negativeList->at(i)];
	feature = abs(((double)sum1 / (double)psize) - ((double)sum2 / (double)nsize));

	positiveList->clear();
	negativeList->clear();

	return feature;
}

void CDetection::ExtractHaarFeatrue(CDib& dib) {

	int i, j, k;
	int ii, jj;
	RGBBYTE** ptr = dib.GetRGBPtr();
	
	int NumOfPixels = 0;

	CDib temp;
	int w, h;

	for (k = 1; k < DetectionInfo.cnt; k++) {

		temp.CreateRGBImage(DetectionInfo.rg[k].w, DetectionInfo.rg[k].h, RGB(0,0,0));

		RGBBYTE** temp_ptr = temp.GetRGBPtr();

		for (j = DetectionInfo.rg[k].min_y, jj = 0; j < DetectionInfo.rg[k].max_y, jj<DetectionInfo.rg[k].h; j++, jj++) {
			for (i = DetectionInfo.rg[k].min_x, ii = 0; i < DetectionInfo.rg[k].max_x, ii<DetectionInfo.rg[k].w; i++, ii++) {
				temp_ptr[jj][ii].r = ptr[j][i].r;
				temp_ptr[jj][ii].g = ptr[j][i].g;
				temp_ptr[jj][ii].b = ptr[j][i].b;
			}
		}


		DibResizeBilinear_FOR_RGBBYTE(temp, 32, 32);

		GetHaarlikeFeatureOfWindow(temp, k);

		L2Normalization(DetectionInfo.rg[k].feature.haar.HaarValue, 60);


	}

}

int CDetection::GetDetectionInfo(lb& detectionInfo)
{
	detectionInfo = DetectionInfo; 

	return detectionInfo.cnt;	
}

void CDetection::SaveRegion(CDib& dib, int frameNum) {

	int i, j, k;
	int ii, jj;
	RGBBYTE** ptr = dib.GetRGBPtr();

	CString filename;

	CString str;
	CString fileNum;
	

	CDib temp;
	int w, h;

	for (k = 1; k < DetectionInfo.cnt; k++) {

		temp.CreateRGBImage(DetectionInfo.rg[k].w, DetectionInfo.rg[k].h, RGB(0,0,0));

		RGBBYTE** temp_ptr = temp.GetRGBPtr();

		for (j = DetectionInfo.rg[k].min_y, jj = 0; j < DetectionInfo.rg[k].max_y, jj<DetectionInfo.rg[k].h; j++, jj++) {
			for (i = DetectionInfo.rg[k].min_x, ii = 0; i < DetectionInfo.rg[k].max_x, ii<DetectionInfo.rg[k].w; i++, ii++) {
				temp_ptr[jj][ii].r = ptr[j][i].r;
				temp_ptr[jj][ii].g = ptr[j][i].g;
				temp_ptr[jj][ii].b = ptr[j][i].b;
			}
		}
		
		if(DetectionInfo.rg[k].type == OTHERS || DetectionInfo.rg[k].type == HEAD_LIGHT) {
			str = "./DATA/";
			fileNum.Format(_T("%d_%d"), frameNum, k);
			fileNum += ".bmp";
			str += fileNum;

			temp.Save(str);
		}
	}

}

void CDetection::ExtractColorHistoFeature(CDib& dib) {

	int i, j, k, s;
	int ii, jj;
	RGBBYTE** ptr = dib.GetRGBPtr();

	int w, h;
	
	for (k = 1; k < DetectionInfo.cnt; k++) {

		int max = 0;

		for (s = 0; s < 256; s++) {
			DetectionInfo.rg[k].feature.colorHisto.R[s] = 0;
			DetectionInfo.rg[k].feature.colorHisto.G[s] = 0;
			DetectionInfo.rg[k].feature.colorHisto.B[s] = 0;
		}

		for (j = DetectionInfo.rg[k].min_y; j < DetectionInfo.rg[k].max_y; j++) {
			for (i = DetectionInfo.rg[k].min_x; i < DetectionInfo.rg[k].max_x; i++) {
				DetectionInfo.rg[k].feature.colorHisto.R[ptr[j][i].r]++;
				DetectionInfo.rg[k].feature.colorHisto.G[ptr[j][i].g]++;
				DetectionInfo.rg[k].feature.colorHisto.B[ptr[j][i].b]++;
			}
		}

		for (i = 0; i < 3; i++) {
			if (i == 0) {
				for (j = 0; j < 256; j++) {
					if (max < DetectionInfo.rg[k].feature.colorHisto.R[j]) {
						max = DetectionInfo.rg[k].feature.colorHisto.R[j];
					}
				}
			} 
			else if (i == 1) {
				for (j = 0; j < 256; j++) {
					if (max < DetectionInfo.rg[k].feature.colorHisto.G[j]) {
						max = DetectionInfo.rg[k].feature.colorHisto.G[j];
					}
				}
			}
			else {
				for (j = 0; j < 256; j++) {
					if (max < DetectionInfo.rg[k].feature.colorHisto.B[j]) {
						max = DetectionInfo.rg[k].feature.colorHisto.B[j];
					}
				}
			}
		}

		for (i = 0; i < 256; i++) {
			DetectionInfo.rg[k].feature.colorHisto.R[i] = DetectionInfo.rg[k].feature.colorHisto.R[i] / max;
			DetectionInfo.rg[k].feature.colorHisto.G[i] = DetectionInfo.rg[k].feature.colorHisto.G[i] / max;
			DetectionInfo.rg[k].feature.colorHisto.B[i] = DetectionInfo.rg[k].feature.colorHisto.B[i] / max;
		}

		//L2Normalization(DetectionInfo.rg[k].feature.colorHisto, 256);

		//double temp_r = 0.0;
		//double temp_g = 0.0;
		//double temp_b = 0.0;

		//double dL2norm_r = 0.0;
		//double dL2norm_g = 0.0;
		//double dL2norm_b = 0.0;

		//for (i = 0; i < 256; i++) {
		//	temp_r += DetectionInfo.rg[k].feature.colorHisto.R[i] * DetectionInfo.rg[k].feature.colorHisto.R[i];
		//	temp_g += DetectionInfo.rg[k].feature.colorHisto.G[i] * DetectionInfo.rg[k].feature.colorHisto.G[i];
		//	temp_b += DetectionInfo.rg[k].feature.colorHisto.B[i] * DetectionInfo.rg[k].feature.colorHisto.B[i];
		//}

		//dL2norm_r = sqrt(temp_r);
		//dL2norm_g = sqrt(temp_g);
		//dL2norm_b = sqrt(temp_b);

		//for (i = 0; i < 256; i++) {
		//	DetectionInfo.rg[k].feature.colorHisto.R[i] = DetectionInfo.rg[k].feature.colorHisto.R[i] / (dL2norm_r + 0.2);
		//	DetectionInfo.rg[k].feature.colorHisto.G[i] = DetectionInfo.rg[k].feature.colorHisto.G[i] / (dL2norm_g + 0.2);
		//	DetectionInfo.rg[k].feature.colorHisto.B[i] = DetectionInfo.rg[k].feature.colorHisto.B[i] / (dL2norm_b + 0.2);
		//}


	}

}