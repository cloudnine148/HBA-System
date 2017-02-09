#pragma once

#include "Dib.h"
#include "RGBBYTE.h"
#include <opencv/highgui.h>


int Get24bitDibFromIplImage(IplImage* img, CDib& dib);
int GetIplImageFrom24bitDib(CDib& dib, IplImage** img);

int Get8bitDibFromIplImage(IplImage* img, CDib&dib);
int GetIplImageFrom8bitDib(CDib& dib, IplImage** img);

int GetDibFromIplImage(IplImage* img, CDib& dib);
int GetIplImageFromDib(CDib& dib, IplImage** img);

void ShowImage(CDib& dib, char *name, int nWaitTime=0);		// For Test : 2016-02-26

void arrowedLine(IplImage* img, CvPoint pt1, CvPoint pt2, const CvScalar& color, int thickness=1, int line_type=8, int shift=0, double tipLength=0.1);