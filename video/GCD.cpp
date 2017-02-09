#include "GCd.h"
#include "stdafx.h"
int gcdSubstract(int x, int y)
{
	int temp;

	if (y > x)
	{
		temp = x;
		x = y;
		y = temp;
	}
	if (y == 0)
		return x;
	return gcdSubstract(y, x - y);
}
