#include "StdAfx.h"
#include "DibColor.h"


//#define RGB2GRAY(r, g, b) (0.25*(r) + 0.25*(g) - 0.45*(b)) //s component of YES model
#define RGB2GRAY(r, g, b) (0.299*(r) + 0.587*(g) + 0.114*(b))
//#define SWAP(a, b)  do { a ^= b; b ^= a; a ^= b; } while ( 0 )
#define SWAP(a, b)  { a ^= b; b ^= a; a ^= b; }

const double PI = acos(-1.0);

//#include "common.h"

void DibGrayscale(CDib& dib)
{
	register int i, j;

	int w = dib.GetWidth();
	int h = dib.GetHeight();

	CDib cpy = dib;
	dib.CreateGrayImage(w, h);

	BYTE** ptr1 = dib.GetPtr();
	RGBBYTE** ptr2 = cpy.GetRGBPtr();

	BYTE r, g, b;
	for(j = 0; j < h; j++)
	{
		for(i = 0; i < w; i++)
		{
			r = ptr2[j][i].r;
			g = ptr2[j][i].g;
			b = ptr2[j][i].b;

			ptr1[j][i] = (BYTE)limit(RGB2GRAY(r, g, b));
		}
	}
}

void DibYCbCrscale(CDib& dib)
{
	register int i, j;

	double R = 0;
	double G = 0;
	double B = 0;

	int w = dib.GetWidth();
	int h = dib.GetHeight();

	CDib cpy = dib;
	dib.CreateRGBImage(w, h, RGB(0, 0, 0));

	RGBBYTE** ptr = dib.GetRGBPtr();
	RGBBYTE** ptr2 = cpy.GetRGBPtr();

	for(i = 0; i < h; i++)
	{
		for(j = 0; j < w; j++)
		{
			R = ptr2[i][j].r;
			G = ptr2[i][j].g;
			B = ptr2[i][j].b;

			// RGB 입력 : [ 0~255 ]

			//// Y
			//ptr[i][j].r = (int)(0.257*R + 0.504*G + 0.098*B + 16.0 + 0.5);

			//// Cb
			//ptr[i][j].g = (int)(-0.148*R - 0.291*G + 0.43*B + 128.0 + 0.5);

			//// Cr
			//ptr[i][j].b = (int)(0.439*R - 0.368*G - 0.071*B + 128.0 + 0.5);

			// Y
			ptr[i][j].r = limit((int)(0.299*R + 0.587*G + 0.114*B + 0.5));

			// Cb
			ptr[i][j].g = limit((int)(-0.169*R - 0.331*G + 0.5*B + 128.0 + 0.5));

			// Cr
			ptr[i][j].b = limit((int)(0.5*R - 0.419*G - 0.081*B + 128.0 + 0.5));
			
		}
	}
}

void DibHSIscale(CDib& dib)
{
	register int i, j;

	double R = 0;
	double G = 0;
	double B = 0;

	int w = dib.GetWidth();
	int h = dib.GetHeight();

	CDib cpy = dib;
	dib.CreateRGBImage(w, h, RGB(0, 0, 0));

	RGBBYTE** ptr = dib.GetRGBPtr();
	RGBBYTE** ptr2 = cpy.GetRGBPtr();

	for(i = 0; i < h; i++)
	{
		for(j = 0; j < w; j++)
		{
			R = ptr2[i][j].r / 255.0;
			G = ptr2[i][j].g / 255.0;
			B = ptr2[i][j].b / 255.0;

			// I
			ptr[i][j].r = (int)(((R + G + B) / 3.0) * 255.0);

			if((R == G) && (G == B))
			{
				// S
				ptr[i][j].g = 0;
				// H
				ptr[i][j].b = 0;
			}
			else
			{
				double min_rgb;
				double temp;
				double temp_h;

				min_rgb = min(min(R, G), B);

				// S
				ptr[i][j].g = (int)((1.0 - (min_rgb / ptr[i][j].r)) * 255.0);

				temp = ((R - G) + (R - B)) / (2 * sqrt((R - G) * (R - G) + (R - B) * (G - B)));

				temp_h = acos(temp) * 180 / PI;

				if(B > G)
				{
					temp_h = 360 - temp_h;
				}

				// H
				ptr[i][j].b = (int)((temp_h / 360.0) * 255.0);
			}
		}
	}
}

void DibResizeBilinear(CDib& dib, int nw, int nh)
{
	CDib cpy = dib;
	
	register int i, j;

	int w = cpy.GetWidth();
	int h = cpy.GetHeight();

	dib.CreateGrayImage(nw, nh);

	BYTE** ptr1 = cpy.GetPtr();
	BYTE** ptr2 = dib.GetPtr();

	int x1, y1, x2, y2;
	double rx, ry, p, q, temp;

	i=j=0;
	x1=y1=x2=y2=0;
	rx=ry=p=q=temp=0.0;

	

	for( j = 0 ; j < nh ; j++ )
		for( i = 0 ; i < nw ; i++ )
		{
			rx = (double)w*i/nw;
			ry = (double)h*j/nh;

			x1 = (int)rx;
			y1 = (int)ry;

			x2 = x1 + 1; if( x2 == w ) x2 = w - 1;
			y2 = y1 + 1; if( y2 == h ) y2 = h - 1;

			p = rx - x1;
			q = ry - y1;


			temp = (1.0-p)*(1.0-q)*ptr1[y1][x1] + p*(1.0-q)*ptr1[y1][x2]
			+ (1.0-p)*q*ptr1[y2][x1] + p*q*ptr1[y2][x2];

			ptr2[j][i] = (BYTE)limit(temp);
		}
}

void DibResizeBilinear_FOR_RGBBYTE(CDib& dib, int nw, int nh)
{
	CDib cpy = dib;

	register int i, j;

	int w = cpy.GetWidth();
	int h = cpy.GetHeight();

	dib.CreateRGBImage(nw, nh, RGB(0,0,0));

	RGBBYTE** ptr1 = cpy.GetRGBPtr();
	RGBBYTE** ptr2 = dib.GetRGBPtr();

	int x1, y1, x2, y2;
	double rx, ry, p, q, temp;

	i = j = 0;
	x1 = y1 = x2 = y2 = 0;
	rx = ry = p = q = temp = 0.0;



	for (j = 0; j < nh; j++)
		for (i = 0; i < nw; i++)
		{
			rx = (double)w*i / nw;
			ry = (double)h*j / nh;

			x1 = (int)rx;
			y1 = (int)ry;

			x2 = x1 + 1; if (x2 == w) x2 = w - 1;
			y2 = y1 + 1; if (y2 == h) y2 = h - 1;

			p = rx - x1;
			q = ry - y1;


			temp = (1.0 - p)*(1.0 - q)*ptr1[y1][x1].r + p*(1.0 - q)*ptr1[y1][x2].r
				+ (1.0 - p)*q*ptr1[y2][x1].r + p*q*ptr1[y2][x2].r;

			ptr2[j][i].r = (BYTE)limit(temp);

			temp = (1.0 - p)*(1.0 - q)*ptr1[y1][x1].g + p*(1.0 - q)*ptr1[y1][x2].g
				+ (1.0 - p)*q*ptr1[y2][x1].g + p*q*ptr1[y2][x2].g;

			ptr2[j][i].g = (BYTE)limit(temp);

			temp = (1.0 - p)*(1.0 - q)*ptr1[y1][x1].b + p*(1.0 - q)*ptr1[y1][x2].b
				+ (1.0 - p)*q*ptr1[y2][x1].b + p*q*ptr1[y2][x2].b;

			ptr2[j][i].b = (BYTE)limit(temp);
		}
}


void DibResizeBilinear(CDib& dib, int nw, int nh, int roi_star, int roi_end, int window_height)
{
	CDib cpy = dib;
	
	register int i, j;

	int w = cpy.GetWidth();
	int h = cpy.GetHeight();

	dib.CreateGrayImage(nw, nh);

	BYTE** ptr1 = cpy.GetPtr();
	BYTE** ptr2 = dib.GetPtr();

	int x1, y1, x2, y2;
	double rx, ry, p, q, temp;

	i=j=0;
	x1=y1=x2=y2=0;
	rx=ry=p=q=temp=0.0;		

	for( j = roi_star ; j < roi_end ; j++ )
		for( i = 0 ; i < nw ; i++ )
		{
			rx = (double)w*i/nw;
			ry = (double)h*j/nh;

			x1 = (int)rx;
			y1 = (int)ry;

			x2 = x1 + 1; if( x2 == w ) x2 = w - 1;
			y2 = y1 + 1; if( y2 == h ) y2 = h - 1;

			p = rx - x1;
			q = ry - y1;


			temp = (1.0-p)*(1.0-q)*ptr1[y1][x1] + p*(1.0-q)*ptr1[y1][x2]
			+ (1.0-p)*q*ptr1[y2][x1] + p*q*ptr1[y2][x2];

			ptr2[j][i] = (BYTE)limit(temp);
		}
}


void DibMorphologyGrayErosion(CDib& dib)
{
	register int i, j, m, n;

	int w = dib.GetWidth();
	int h = dib.GetHeight();

	CDib cpy = dib;;

	BYTE** ptr1 = dib.GetPtr();
	BYTE** ptr2 = cpy.GetPtr();

	int x, y, pmin;
	for( j = 0 ; j < h ; j++ )
	for( i = 0 ; i < w ; i++ )
	{
		pmin = 255;
		
		for( n = -4 ; n <= 4 ; n++ )
		for( m = -4 ; m <= 4 ; m++ )
		{
			x = i + m;
			y = j + n;

			if( x >= 0 && x < w && y >= 0 && y < h )
			{
				if( ptr2[y][x] < pmin )
					pmin = ptr2[y][x];
			}
		}

		ptr1[j][i] = pmin;
	}
}

void DibMorphologyGrayDilation(CDib& dib)
{
	register int i, j, m, n;

	int w = dib.GetWidth();
	int h = dib.GetHeight();

	CDib cpy = dib;;

	BYTE** ptr1 = dib.GetPtr();
	BYTE** ptr2 = cpy.GetPtr();

	int x, y, pmax;
	for( j = 0 ; j < h ; j++ )
	for( i = 0 ; i < w ; i++ )
	{
		pmax = 0;

		for( n = -4 ; n <= 4 ; n++ )
		for( m = -4 ; m <= 4 ; m++ )
		{
			x = i + m;
			y = j + n;

			if( x >= 0 && x < w && y >= 0 && y < h )
			{
				if( ptr2[y][x] > pmax )
					pmax = ptr2[y][x];
			}
		}

		ptr1[j][i] = pmax;
	}
}

void DibResizeBilinear_For_BYTE(BYTE** img, BYTE** resultimg, int origW, int origH, int ResultW, int ResultH, int roi_start, int roi_end, int window_height)
{
	register int i, j;

	int x1, y1, x2, y2;
	double rx, ry, p, q, temp;
	
	i=j=0;
	x1=y1=x2=y2=0;
	rx=ry=p=q=temp=0.0;
	
	for( j = roi_start ; j < roi_end ; j++ )
	{
		for( i = 0 ; i < ResultW ; i++ )
		{
			rx = (double)origW*i/ResultW;
			ry = (double)origH*j/ResultH;

			x1 = (int)rx;
			y1 = (int)ry;

			x2 = x1 + 1; if( x2 == origW ) x2 = origW - 1;
			y2 = y1 + 1; if( y2 == origH ) y2 = origH - 1;

			p = rx - x1;
			q = ry - y1;	

			if( origW <= x1 )	x1 = origW;
			if( origH <= y1 )	y1 = origH;
			if( origW <= x2 )	x2 = origW;
			if( origH <= y2 )	y2 = origH;

			temp = (1.0-p)*(1.0-q)*img[y1][x1] + p*(1.0-q)*img[y1][x2]
			+ (1.0-p)*q*img[y2][x1] + p*q*img[y2][x2];

			resultimg[j][i] = (BYTE)limit(temp);

		}
	}

	/*
	for( j=0 ; j<ResultH ; j++ )
	{
		for( i=0 ; i<ResultW ; i++ )
		{
			rx = (double)origW*i/ResultW;
			ry = (double)origH*j/ResultH;

			x1 = (int)rx;
			y1 = (int)ry;

			x2 = x1 + 1; if( x2 == origW ) x2 = origW - 1;
			y2 = y1 + 1; if( y2 == origH ) y2 = origH - 1;

			p = rx - x1;
			q = ry - y1;


			temp = (1.0-p)*(1.0-q)*img[y1][x1] + p*(1.0-q)*img[y1][x2]
			+ (1.0-p)*q*img[y2][x1] + p*q*img[y2][x2];

			resultimg[j][i] = (BYTE)limit(temp);

		}
	}
	*/

}
//void DibResizeBilinear_For_BYTE(BYTE** img, BYTE** resultimg, int origW, int origH, int ResultW, int ResultH, double roi_start_ratio, double roi_end_ratio, int window_height)
//{
//	register int i, j;
//
//	int x1, y1, x2, y2;
//	double rx, ry, p, q, temp;
//	
//	i=j=0;
//	x1=y1=x2=y2=0;
//	rx=ry=p=q=temp=0.0;
//
//	int sh, eh;
//	if( roi_start_ratio == 0.0 && roi_end_ratio )
//	{
//		sh = 0;
//		eh = ResultH;
//	}else
//	{
//		sh = (int)((ResultH * roi_start_ratio) - window_height);
//		eh = (int)(ResultH * roi_end_ratio);
//	}
//
//	for( j = sh ; j < eh ; j++ )
//	{
//		for( i = 0 ; i < ResultW ; i++ )
//		{
//			rx = (double)origW*i/ResultW;
//			ry = (double)origH*j/ResultH;
//
//			x1 = (int)rx;
//			y1 = (int)ry;
//
//			x2 = x1 + 1; if( x2 == origW ) x2 = origW - 1;
//			y2 = y1 + 1; if( y2 == origH ) y2 = origH - 1;
//
//			p = rx - x1;
//			q = ry - y1;	
//
//			temp = (1.0-p)*(1.0-q)*img[y1][x1] + p*(1.0-q)*img[y1][x2]
//			+ (1.0-p)*q*img[y2][x1] + p*q*img[y2][x2];
//
//			resultimg[j][i] = (BYTE)limit(temp);
//
//		}
//	}
//
//	/*
//	for( j=0 ; j<ResultH ; j++ )
//	{
//		for( i=0 ; i<ResultW ; i++ )
//		{
//			rx = (double)origW*i/ResultW;
//			ry = (double)origH*j/ResultH;
//
//			x1 = (int)rx;
//			y1 = (int)ry;
//
//			x2 = x1 + 1; if( x2 == origW ) x2 = origW - 1;
//			y2 = y1 + 1; if( y2 == origH ) y2 = origH - 1;
//
//			p = rx - x1;
//			q = ry - y1;
//
//
//			temp = (1.0-p)*(1.0-q)*img[y1][x1] + p*(1.0-q)*img[y1][x2]
//			+ (1.0-p)*q*img[y2][x1] + p*q*img[y2][x2];
//
//			resultimg[j][i] = (BYTE)limit(temp);
//
//		}
//	}
//	*/
//
//}

void DidRotate(CDib& dib, double angle)
{
	register int i, j;

	int w = dib.GetWidth();
	int h = dib.GetHeight();

	CDib cpy = dib;

	double rad = (angle*PI)/180.0f;
	double cos_value = cos(rad);
	double sin_value = sin(rad);

	int nx, ny, minx, miny, maxx, maxy, nw, nh;

	int interval_x = 0;
	int interval_y = 0;

	minx = maxx = 0;
	miny = maxy = 0;

	nx = (int)floor(w*cos_value + 0.5f);
	ny = (int)floor(w*sin_value + 0.5f);
	minx = (minx<nx)?minx:nx; maxx = (maxx>nx)?maxx:nx;
	miny = (miny<ny)?miny:ny; maxy = (maxy>ny)?maxy:ny;

	nx = (int)floor(-h*sin_value + 0.5f);
	ny = (int)floor(h*cos_value + 0.5f);
	minx = (minx<nx)?minx:nx; maxx = (maxx>nx)?maxx:nx;
	miny = (miny<ny)?miny:ny; maxy = (maxy>ny)?maxy:ny;

	nx = (int)floor(w*cos_value -h*sin_value + 0.5f);
	ny = (int)floor(w*sin_value +h*cos_value + 0.5f);
	minx = (minx<nx)?minx:nx; maxx = (maxx>nx)?maxx:nx;
	miny = (miny<ny)?miny:ny; maxy = (maxy>ny)?maxy:ny;

	nw = maxx-minx;
	nh = maxy-miny;

	dib.CreateGrayImage(nw,nh,0);

	interval_x = abs((w / 2) - (nw / 2));
	interval_y = abs((h / 2) - (nh / 2));

	BYTE** ptrCpy = cpy.GetPtr();
	BYTE** ptrOrigin = dib.GetPtr();

	int x1, x2, y1, y2;
	double rx, ry, p, q, temp;

	for(j=miny; j<maxy; ++j)
	{
		for(i=minx; i<maxx; ++i)
		{
			rx = i*cos_value + j*sin_value;
			ry = -i*sin_value + j*cos_value;

			if(rx<0 ||rx>w-1|| ry<0|| ry>h-1)
				continue;

			x1 = (int)rx;
			y1 = (int)ry;

			x2 = x1+1; if(x2==w) x2=w-1;
			y2 = y1+1; if(y2==h) y2=h-1;

			p = rx -x1;
			q = ry -y1;

			temp = (1.0f - p)*(1.0f - q)*ptrCpy[y1][x1] + p*(1.0f - q)*ptrCpy[y1][x2]+
				(1.0f - p)*q*ptrCpy[y2][x1] + p*q*ptrCpy[y2][x2];

			//if(0 <= j - miny - interval_y && j - miny - interval_y < h 
			//	&& 0 <= i - minx - interval_x && i - minx - interval_x < w)
			//{
			//	ptrOrigin[j - miny - interval_y][i - minx - interval_x] = (BYTE)limit(temp);
			//}

			ptrOrigin[j - miny][i - minx] = (BYTE)limit(temp);
		}
	}

	return;
}

void DidTranslation(CDib& dib, int sx, int sy)
{
	register int i, j;
	int x = 0, y = 0;

	CDib cpy = dib;

	int w = cpy.GetWidth();
	int h = cpy.GetHeight();

	dib.CreateGrayImage(w, h, 0);

	BYTE** ptr1 = cpy.GetPtr();
	BYTE** ptr2 = dib.GetPtr();

	for(i = 0; i < h; i++)
	{
		for(j = 0; j < w; j++)
		{
			x = j - sx;
			y = i - sy;

			if(x >= 0 && x < w && y >= 0 && y < h)
			{
				ptr2[i][j] = ptr1[y][x];
			}
		}
	}

	TRACE("");
}



void DibHistogramStretching(CDib& dib)
{
	register int i, j;

	int max, min, Nmax, Nmin;
	Nmax = 255;
	Nmin = 0;
	int histogram[256] = { 0, };

	int count = 0;

	int w = dib.GetWidth();
	int h = dib.GetHeight();

	int scaling_const = (int)((double)w * (double)h * 0.01);

	BYTE** ptr = dib.GetPtr();

	for (i = 0; i < h; i++)
	{
		for (j = 0; j < w; j++)
		{
			histogram[ptr[i][j]]++;
		}
	}

	for (i = 0; i < 256; i++)
	{
		count += histogram[i];

		if (count > scaling_const)
		{
			min = i;
			break;
		}
	}

	count = 0;
	for (i = 255; i >= 0; i--)
	{
		count += histogram[i];

		if (count > scaling_const)
		{
			max = i;
			break;
		}
	}

	for (i = 0; i < h; i++)
	{
		for (j = 0; j < w; j++)
		{
			if (ptr[i][j] < min)
			{
				ptr[i][j] = min;
			}
			if (ptr[i][j] > max)
			{
				ptr[i][j] = max;
			}

			ptr[i][j] = (int)((double)(ptr[i][j] - min) * ((double)(Nmax - Nmin) / (double)(max - min)) + Nmin);
		}
	}
}

void DibEdgeSobel(CDib& dib)
{
	register int i, j;

	int w = dib.GetWidth();
	int h = dib.GetHeight();

	CDib cpy = dib;
	dib.CreateGrayImage(w, h, 0);

	BYTE** ptr1 = dib.GetPtr();
	BYTE** ptr2 = cpy.GetPtr();

	int h1, h2;
	double hval;

	for (i = 1; i < h - 1; i++)
	{
		for (j = 1; j < w - 1; j++)
		{
			h1 = -ptr2[i - 1][j - 1] - 2 * ptr2[i - 1][j] - ptr2[i - 1][j + 1]
				+ ptr2[i + 1][j - 1] + 2 * ptr2[i + 1][j] + ptr2[i + 1][j + 1];

			h2 = -ptr2[i - 1][j - 1] - 2 * ptr2[i][j - 1] - ptr2[i + 1][j - 1]
				+ ptr2[i - 1][j + 1] + 2 * ptr2[i][j + 1] + ptr2[i + 1][j + 1];

			hval = sqrt((double)h1 * h1 + h2 * h2);

			if (hval > 100)
			{
				ptr1[i][j] = (BYTE)limit(hval);
			}
		}
	}
}

void DibFilterLaplacian(CDib& dib)
{
	register int i, j;

	int w = dib.GetWidth();
	int h = dib.GetHeight();

	CDib cpy = dib;
	dib.CreateGrayImage(w, h, 0);

	BYTE** ptr1 = dib.GetPtr();
	BYTE** ptr2 = cpy.GetPtr();

	int temp;
	for (i = 1; i < h - 1; i++)
	{
		for (j = 1; j < w - 1; j++)
		{
			temp = 4 * ptr2[i][j] - ptr2[i - 1][j] - ptr2[i][j + 1] - ptr2[i + 1][j] - ptr2[i][j - 1];

			ptr1[i][j] = (BYTE)limit(temp);
		}
	}
}

void DibFilterUnsharpMask(CDib& dib)
{
	register int i, j;

	int w = dib.GetWidth();
	int h = dib.GetHeight();

	CDib cpy = dib;

	BYTE** ptr1 = dib.GetPtr();
	BYTE** ptr2 = cpy.GetPtr();

	int temp;
	for (i = 1; i < h - 1; i++)
	{
		for (j = 1; j < w - 1; j++)
		{
			ptr1[i][j] = (BYTE)limit(5 * ptr2[i][j] - ptr2[i - 1][j] - ptr2[i][j - 1] - ptr2[i + 1][j] - ptr2[i][j + 1]);
		}
	}
}

int DibHoughLine(CDib& dib, LineParam* line, int lineNum)
{
	register int i, j;

	int w = dib.GetWidth();
	int h = dib.GetHeight();

	BYTE** ptr = dib.GetPtr();

	int num_rho = (int)(sqrt((double)w * w + h * h) * 2);
	int num_ang = 360;

	double* tsin = new double[num_ang];
	double* tcos = new double[num_ang];

	for (i = 0; i < num_ang; i++)
	{
		tsin[i] = (double)sin(i * PI / num_ang);
		tcos[i] = (double)cos(i * PI / num_ang);
	}

	int** arr = new int*[num_rho];

	for (i = 0; i < num_rho; i++)
	{
		arr[i] = new int[num_ang];
		memset(arr[i], 0, sizeof(int) * num_ang);
	}

	int m, n;

	for (i = 0; i < h; i++)
	{
		for (j = 0; j < w; j++)
		{
			if (ptr[i][j] > 60)
			{
				//// 0. 전체
				//for (n = 0; n < num_ang; n++)
				//{
				//	m = (int)floor(j * tsin[n] + i * tcos[n] + 0.5);
				//	m += (num_rho / 2);

				//	arr[m][n]++;
				//}

				//// 1. 왼쪽
				//for (n = 30; n < 150; n++)
				//{
				//	m = (int)floor(j * tsin[n] + i * tcos[n] + 0.5);
				//	m += (num_rho / 2);

				//	arr[m][n]++;
				//}

				//// 1. 오른쪽
				//for (n = 210; n < 335; n++)
				//{
				//	m = (int)floor(j * tsin[n] + i * tcos[n] + 0.5);
				//	m += (num_rho / 2);

				//	arr[m][n]++;
				//}

				// 2. 왼쪽 0
				for (n = 40; n < 60; n++)
				{
					m = (int)floor(j * tsin[n] + i * tcos[n] + 0.5);
					m += (num_rho / 2);

					arr[m][n]++;
				}
				// 2. 왼쪽 1
				//for (n = 120; n < 150; n++)
				//{
				//	m = (int)floor(j * tsin[n] + i * tcos[n] + 0.5);
				//	m += (num_rho / 2);

				//	arr[m][n]++;
				//}

				// 2. 오른쪽 0
				//for (n = 210; n < 240; n++)
				//{
				//	m = (int)floor(j * tsin[n] + i * tcos[n] + 0.5);
				//	m += (num_rho / 2);

				//	arr[m][n]++;
				//}
				// 2. 오른쪽 1
				for (n = 290; n < 305; n++)
				{
					m = (int)floor(j * tsin[n] + i * tcos[n] + 0.5);
					m += (num_rho / 2);

					arr[m][n]++;
				}
			}
		}
	}

	for (i = 0; i < lineNum; i++)
	{
		line[i].ang = 0;
		line[i].rho = 0;
	}

	int arr_max = 128;
	int arr_min = 55;
	int lineCount = 0;

	for (m = 0; m < num_rho; m++)
	{
		for (n = 0; n < num_ang; n++)
		{
			if (arr_min < arr[m][n] && arr[m][n] < arr_max)
		//	if (arr[m][n] > arr_max)
			{
				line[lineCount].rho = m - (num_rho / 2);
				line[lineCount].ang = n * 180.0 / num_ang;

				lineCount++;

				if (lineCount == lineNum)
				{
					//Trace(_T("라인 수가 다 참\n"));
					break;
				}
			}
		}
	}

	delete[] tsin;
	delete[] tcos;

	for (i = 0; i < num_rho; i++)
	{
		delete[] arr[i];
	}
	delete[] arr;

	return lineCount;
}

int DibHoughLine(CDib& dib, LineParam* line, int lineNum, int angle1, int angle2, int arrMax)
{
	register int i, j;

	int w = dib.GetWidth();
	int h = dib.GetHeight();

	BYTE** ptr = dib.GetPtr();

	int num_rho = (int)(sqrt((double)w * w + h * h) * 2);
	int num_ang = 360;

	double* tsin = new double[num_ang];
	double* tcos = new double[num_ang];

	for (i = 0; i < num_ang; i++)
	{
		tsin[i] = (double)sin(i * PI / num_ang);
		tcos[i] = (double)cos(i * PI / num_ang);
	}

	int** arr = new int*[num_rho];

	for (i = 0; i < num_rho; i++)
	{
		arr[i] = new int[num_ang];
		memset(arr[i], 0, sizeof(int) * num_ang);
	}

	int m, n;

	for (i = 0; i < h; i++)
	{
		for (j = 0; j < w; j++)
		{
			if (ptr[i][j] > 60)
			{
				for (n = angle1; n < angle2; n++)
				{
					m = (int)floor(j * tsin[n] + i * tcos[n] + 0.5);
					m += (num_rho / 2);

					arr[m][n]++;
				}
			}
		}
	}

	for (i = 0; i < lineNum; i++)
	{
		line[i].ang = 0;
		line[i].rho = 0;
	}

//	int arr_max = arrMax;
	int arr_max = 0;
	int arr_min = 55;
	int lineCount = 0;

	for (m = 0; m < num_rho; m++)
	{
		for (n = 0; n < num_ang; n++)
		{
		//	if (arr_min < arr[m][n] && arr[m][n] < arr_max)
			if (arr[m][n] > arr_max)
			{
				line[lineCount].rho = m - (num_rho / 2);
				line[lineCount].ang = n * 180.0 / num_ang;

		//		lineCount++;

				arr_max = arr[m][n];

				if (lineCount == lineNum)
				{
					//Trace(_T("라인 수가 다 참\n"));
					break;
				}
			}
		}
	}

	delete[] tsin;
	delete[] tcos;

	for (i = 0; i < num_rho; i++)
	{
		delete[] arr[i];
	}
	delete[] arr;

//	return lineCount;
	return 1;
}

void DibDrawLine(CDib& dib, LineParam* line, int lineNum, BYTE C)
{
	register int x, y, i;

	int w = dib.GetWidth();
	int h = dib.GetHeight();

	BYTE** ptr = dib.GetPtr();

	for (i = 0; i < lineNum; i++)
	{
		if (line[i].ang == 90)
		{
			x = (int)(line[i].rho + 0.5);

			for (y = 0; y < h; y++)
			{
				ptr[y][x] = C;
			}
		}
		else
		{
			int x1 = 0;
			int y1 = (int)floor(line[i].rho / cos(line[i].ang * PI / 180) + 0.5);
			int x2 = dib.GetWidth() - 1;
			int y2 = (int)floor((line[i].rho - x2 * sin(line[i].ang * PI / 180)) / cos(line[i].ang * PI / 180) + 0.5);
			
			DibDrawLine(dib, x1, y1, x2, y2, C);
		}
	}
}

void DibDrawLine(CDib& dib, int x1, int y1, int x2, int y2, BYTE C)
{
	register int x, y;
	double m;

	int w = dib.GetWidth();
	int h = dib.GetHeight();

	BYTE** ptr = dib.GetPtr();

	if (x1 == x2)
	{
		if (y1 > y2)
		{
			SWAP(y1, y2);
		}

		for (y = y1; y <= y2; y++)
		{
			ptr[y][x1] = C;
		}
	}
	else
	{
		m = (double)(y2 - y1) / (x2 - x1);

		if ((m > -1) && (m < 1))
		{
			if (x1 > x2)
			{
				SWAP(x1, x2);
				SWAP(y1, y2);
			}

			for (x = x1; x <= x2; x++)
			{
				y = (int)floor(m * (x - x1) + y1 + 0.5);

				if (y >= 0 && y < h)
				{
					ptr[y][x] = C;
				}
			}
		}
		else
		{
			if (y1 > y2)
			{
				SWAP(x1, x2);
				SWAP(y1, y2);
			}

			for (y = y1; y <= y2; y++)
			{
				x = (int)floor((y - y1) / m + x1 + 0.5);

				if (y >= 0 && y < h)
				{
					ptr[y][x] = C;
				}
			}
		}
	}
}

void BlackHatTransform(CDib& dib, CDib& result, CDib& ori) {
	
	int x, y;
	CDib tempOri = ori;

	DibGrayscale(tempOri);
	BYTE** ori_ptr = tempOri.GetPtr();

	int h = dib.GetHeight();
	int w = dib.GetWidth();

	result.CreateGrayImage(w, h);

	BYTE** ptr = dib.GetPtr();
	BYTE** res_ptr = result.GetPtr();

	CDib tempDib;
	tempDib.CreateGrayImage(w, h);
	tempDib.Copy(&tempOri);
	BYTE** temp_ptr = tempDib.GetPtr();

	//tempDib.Save(_T("dib.bmp"));

	//DibMorphologyGrayDilation(tempDib); // 팽창 
	//DibMorphologyGrayDilation(tempDib); // 팽창 
	DibMorphologyGrayDilation(tempDib); // 팽창 
	DibMorphologyGrayErosion(tempDib); // 침식
	
	CDib A1;
	CDib A2;

	A1.CreateGrayImage(w, h);
	A2.CreateGrayImage(w, h);

	BYTE** a1_ptr = A1.GetPtr();
	BYTE** a2_ptr = A2.GetPtr();

	//for (int i = 0; i < h; i++) {
	//	for (int j = 0; j < w; j++) {
	//		if (ptr[i][j] == 255) a1_ptr[i][j] = ori_ptr[i][j]; // 이진화 영상에서 255부분의 원본영상의 값
	//		if (temp_ptr[i][j] == 255) a2_ptr[i][j] = ori_ptr[i][j]; // 모폴로지 영상에서 255부분의 원본영상의 값
	//	}
	//}

	for (int i = 0; i < h; i++) {
		for (int j = 0; j < w; j++) {
			res_ptr[i][j] = temp_ptr[i][j] - ori_ptr[i][j];
			//res_ptr[i][j] = a2_ptr[i][j] - a1_ptr[i][j];
		}
	}

	int iSum = 0;

	int min = INT_MAX;
	int max = 0;

	for (int i = 0; i < h; i++) {
		for (int j = 0; j < w; j++) {
			if (res_ptr[i][j] != 0) {
				iSum += res_ptr[i][j];
				if (min > res_ptr[i][j]) min = res_ptr[i][j];
				if (max < res_ptr[i][j]) max = res_ptr[i][j];
			}		
		}
	}

	//int th = iSum / (w*h);
	int th = (max - min) / 2;

	for (int i = 0; i < h; i++) {
		for (int j = 0; j < w; j++) {
			if (res_ptr[i][j] > th) {
				res_ptr[i][j] = 255;
			}
		}
	}
	
	
}