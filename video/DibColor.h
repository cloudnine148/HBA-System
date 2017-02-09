#pragma once

#include "Dib.h"
#include "Dib.h"
#include "RGBBYTE.h"
#include <math.h>






typedef struct _LineParam
{
	double rho;
	double ang;
} LineParam;

void DibGrayscale(CDib& dib);
void DibHSIscale(CDib& dib);
void DibYCbCrscale(CDib& dib);

void DibResizeBilinear(CDib& dib, int nw, int nh);
void DibResizeBilinear(CDib& dib, int nw, int nh, int roi_star, int roi_end, int window_height);
void DibResizeBilinear_FOR_RGBBYTE(CDib& dib, int nw, int nh);

void DibMorphologyGrayErosion(CDib& dib);
void DibMorphologyGrayDilation(CDib& dib);

void DibResizeBilinear_For_BYTE(BYTE** img, BYTE** resultimg, int origW, int origH, int ResultW, int ResultH, int roi_star, int roi_end, int window_height=0.0);
//void DibResizeBilinear_For_BYTE(BYTE** img, BYTE** resultimg, int origW, int origH, int ResultW, int ResultH, double roi_start_ratio, double roi_end_ratio, int window_height);

void DidRotate(CDib& dib, double angle);
void DidTranslation(CDib& dib, int sx, int sy);


void DibHistogramStretching(CDib& dib);

void DibEdgeSobel(CDib& dib);
void DibFilterLaplacian(CDib& dib);
void DibFilterUnsharpMask(CDib& dib);

int DibHoughLine(CDib& dib, LineParam* line, int lineNum);
int DibHoughLine(CDib& dib, LineParam* line, int lineNum, int angle1, int angle2, int arrMax);

void DibDrawLine(CDib& dib, LineParam* line, int lineNum, BYTE C);

void DibDrawLine(CDib& dib, int x1, int y1, int x2, int y2, BYTE C);

void BlackHatTransform(CDib &dib, CDib& result, CDib& ori);