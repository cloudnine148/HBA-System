#include "stdafx.h"
#include "Pairing.h"

#define TEMPLATE_WIDTH 30
#define TEMPLATE_HEIGHT 30
#define BLOB_DISTANCE 150

CPairing::CPairing()
{
	pair.pairinfo = new Pair[9999];
}


CPairing::~CPairing()
{
	delete[] pair.pairinfo;
}

/**
@brief			��ü���� Pairing ����
�ʱ�ȭ
Inner-width üũ
Association Check
Pairing

boolean globalPairing �� �Ű������� �޾ƿ����� ����� �����������. ������ true�� ����.
*/
void CPairing::SetPairing(CDib& dib, lb& DetectionInfo, boolean globalPairing)
{
	BYTE** ptr = dib.GetPtr();

	/*
	first_width = ���� �ʺ�
	second_width = ������ �ʺ�
	first_height = ���� ����
	second_height = ������ ����
	*/
	int first_width = 0, first_height = 0, second_width = 0, second_height = 0;

	// first_area = ���� ����, second_area = ������ ����
	int first_Area = 0, second_Area = 0;

	// first_Y_Vertical = ���� y �� ��ǥ, second_Y_Vertical = ������ y �� ��ǥ
	int first_Y_Vertical = 0, second_Y_Vertical = 0;

	// �� blob�� minx �� maxx, Boxratio ���� �� ���
	int first_min_x, first_max_x, first_min_y, first_max_y, second_min_x, second_max_x, second_min_y, second_max_y;

	// Distance ���� �� ���� ����
	int Inner_width = 0, max_y = 0;

	// ���ؼ� cnt�� 1�ϰ�� ����� ����
	int size = 0;

	//ǥ������
	double first_deviation = 0.0, second_deviation = 0.0;

	//���
	double first_mean = 0.0, second_mean = 0.0;

	if (DetectionInfo.cnt <= 0)		// 2016-12-26 JMR
		return;

	// size�� 2�� ����� �迭�� ����� �Ҵ��� �ȴ�.
	else if (DetectionInfo.cnt == 1)
		size = 2;

	else
		size = DetectionInfo.cnt;

	// Area �� ���� 2���� �迭
	double** Area = new double*[size];
	// Y�� ��ħ�� ���� 2���� �迭
	double** Y_overlap = new double*[size];
	// Width�� Height���� ���� ���� 2���� �迭
	double** Ratio_WH = new double*[size];
	// Distance ���� ���� 2���� �迭
	double** Distance = new double*[size];

	// �ʱ�ȭ
	for (int i = 0; i < size; i++)
	{
		Area[i] = new double[size];
		memset(Area[i], 0, sizeof(double)*size);

		Y_overlap[i] = new double[size];
		memset(Y_overlap[i], 0, sizeof(double)*size);

		Ratio_WH[i] = new double[size];
		memset(Ratio_WH[i], 0, sizeof(double)*size);

		Distance[i] = new double[size];
		memset(Distance[i], 0, sizeof(double)*size);
	}

	int passcnt = 0;
	//
	//	
	//	int** testtable = new int*[DetectionInfo.cnt];
	//	for (int i = 0; i < DetectionInfo.cnt; i++)
	//	{
	//		testtable[i] = new int[DetectionInfo.cnt];
	//		memset(testtable[i], 0, sizeof(int)*DetectionInfo.cnt);
	//	}
	//
	//
#pragma region Calculate threshold and Set pair
	if (globalPairing)
	{
		initPairing(DetectionInfo);

		//dib.Save(_T(".//DATA//origin.bmp"));

		for (int i = 1; i < DetectionInfo.cnt; i++)
		{
			if (-1 < DetectionInfo.rg[i].type && DetectionInfo.rg[i].type < 2)
			{
				if (DetectionInfo.rg[i].w > 1 && DetectionInfo.rg[i].h > 1)
				{
					// ù ��° blob ����
					first_width = DetectionInfo.rg[i].w;
					first_height = DetectionInfo.rg[i].h;
					first_Area = first_width * first_height;

					first_min_x = DetectionInfo.rg[i].min_x;
					first_max_x = DetectionInfo.rg[i].max_x;
					first_min_y = DetectionInfo.rg[i].min_y;
					first_max_y = DetectionInfo.rg[i].max_y;

					first_Y_Vertical = first_min_y + (first_height / 2);

					// Left Blob �缱�� ������
					CDib leftTempDib;
					leftTempDib.CreateGrayImage(first_width, first_height, 0);

					BYTE** left_ptr = leftTempDib.GetPtr();

					for (int i1 = 0, i2 = first_min_y; i2 < first_max_y; i1++, i2++)
					{
						for (int j1 = 0, j2 = first_min_x; j2 < first_max_x; j1++, j2++)
						{
							left_ptr[i1][j1] = ptr[i2][j2];
						}
					}

					DibResizeBilinear(leftTempDib, TEMPLATE_WIDTH, TEMPLATE_HEIGHT);

					//leftTempDib.Save(_T(".\\DATA\\left1.bmp"));

					// Y�� ����
					//DibMirror(leftTempDib);

					//leftTempDib.Save(_T(".\\DATA\\left1.bmp"));

					//��� ���
					first_mean = CalculateMeanValue_FOR_BYTE(leftTempDib, TEMPLATE_WIDTH, TEMPLATE_HEIGHT);

					//ǥ������ ���
					first_deviation = CalculateStandardDeviation_FOR_BYTE(leftTempDib, first_mean, TEMPLATE_WIDTH, TEMPLATE_HEIGHT);

					if (first_mean == 0 || first_deviation == 0)
					{
						continue;
					}


					for (int j = i + 1; j < DetectionInfo.cnt; j++)
					{
						CDib RightTempDib;
						if (-1 < DetectionInfo.rg[j].type && DetectionInfo.rg[j].type < 2)
						{
							if (DetectionInfo.rg[j].w > 1 && DetectionInfo.rg[j].h > 1)
							{
								// �� ��° blob ����
								second_width = DetectionInfo.rg[j].w;
								second_height = DetectionInfo.rg[j].h;
								second_Area = second_width * second_height;

								second_min_x = DetectionInfo.rg[j].min_x;
								second_max_x = DetectionInfo.rg[j].max_x;
								second_min_y = DetectionInfo.rg[j].min_y;
								second_max_y = DetectionInfo.rg[j].max_y;

								second_Y_Vertical = second_min_y + (second_height / 2);

								Inner_width = 0;
					
								// Right Blob �缱�� ������
								RightTempDib.CreateGrayImage(second_width, second_height, 0);

								BYTE** Right_ptr = RightTempDib.GetPtr();

								for (int i1 = 0, i2 = second_min_y; i2 < second_max_y; i1++, i2++)
								{
									for (int j1 = 0, j2 = second_min_x; j2 < second_max_x; j1++, j2++)
									{
										Right_ptr[i1][j1] = ptr[i2][j2];
									}
								}

								DibResizeBilinear(RightTempDib, TEMPLATE_WIDTH, TEMPLATE_HEIGHT);
								//RightTempDib.Save(_T(".\\DATA\\right1.bmp"));

								second_mean = CalculateMeanValue_FOR_BYTE(RightTempDib, TEMPLATE_WIDTH, TEMPLATE_HEIGHT);
								second_deviation = CalculateStandardDeviation_FOR_BYTE(RightTempDib, second_mean, TEMPLATE_WIDTH, TEMPLATE_HEIGHT);

								if (second_mean == 0 || second_deviation == 0)
								{
									continue;
								}

								/*// pair ���� ���� �Ǵ� - score �̻��
								if (Compare_Area(first_Area, second_Area, first_Y_Vertical, second_Y_Vertical, globalPairing))
								{
								if (Compare_Y_Overlap(first_min_y, second_min_y, first_max_y, second_max_y))
								{
								if (Compare_width_and_box_ratio(first_min_x, first_max_x, first_min_y, first_max_y, second_min_x, second_max_x, second_min_y, second_max_y, globalPairing))
								{
								if (Compare_correlation(leftTempDib, RightTempDib, first_mean, second_mean, first_deviation, second_deviation, globalPairing))
								{
								if (DetectionInfo.rg[i].pairnum == 0 && DetectionInfo.rg[j].pairnum == 0)
								{
								//Trace(_T("Pair!"));
								}
								else
								continue;
								}
								else
								{
								//TRACE("�ǳʶ� 1\n");
								continue;
								}
								}
								else
								{
								//TRACE("�ǳʶ� 2\n");
								continue;
								}
								}
								else
								{
								continue;
								}
								}
								else
								{
								//TRACE("�ǳʶ� 3\n");
								continue;
								}*/

								/* // ���� pair �δ� ���
								if (first_min_x < second_min_x)
								{
								DetectionInfo.rg[i].pairnum = passcnt + 1;
								pair.pairinfo[passcnt].LeftBlobInfo = DetectionInfo.rg[i];
								DetectionInfo.rg[j].pairnum = passcnt + 1;
								pair.pairinfo[passcnt].RightBlobInfo = DetectionInfo.rg[j];
								pair.pairinfo[passcnt].LeftBlobDib = leftTempDib;
								pair.pairinfo[passcnt].RightBlobDib = RightTempDib;


								pair.pairinfo[passcnt].box_min_x = first_min_x;
								pair.pairinfo[passcnt].box_max_x = second_max_x;

								if (first_min_y < second_min_y)
								pair.pairinfo[passcnt].box_min_y = first_min_y;

								else
								pair.pairinfo[passcnt].box_min_y = second_min_y;

								if (first_max_y < second_max_y)
								pair.pairinfo[passcnt].box_max_y = second_max_y;

								else
								pair.pairinfo[passcnt].box_max_y = first_max_y;

								pair.pairinfo[passcnt].box_center_x = (pair.pairinfo[passcnt].box_min_x + pair.pairinfo[passcnt].box_max_x) / 2;
								pair.pairinfo[passcnt].box_center_y = (pair.pairinfo[passcnt].box_min_y + pair.pairinfo[passcnt].box_max_y) / 2;


								pair.pairinfo[passcnt].leftArea = first_Area;
								pair.pairinfo[passcnt].rightArea = second_Area;

								pair.pairinfo[passcnt].leftVertical = first_Y_Vertical;
								pair.pairinfo[passcnt].rightVertical = second_Y_Vertical;

								pair.pairinfo[passcnt].boxWidth = pair.pairinfo[passcnt].box_max_x - pair.pairinfo[passcnt].box_min_x;
								pair.pairinfo[passcnt].boxHeight = pair.pairinfo[passcnt].box_max_y - pair.pairinfo[passcnt].box_min_y;
								pair.pairinfo[passcnt].boxRatio = (pair.pairinfo[passcnt].boxWidth) / (pair.pairinfo[passcnt].boxHeight);
								}

								else
								{
								DetectionInfo.rg[j].pairnum = passcnt + 1;
								pair.pairinfo[passcnt].LeftBlobInfo = DetectionInfo.rg[j];
								DetectionInfo.rg[i].pairnum = passcnt + 1;
								pair.pairinfo[passcnt].RightBlobInfo = DetectionInfo.rg[i];

								pair.pairinfo[passcnt].box_min_x = second_min_x;
								pair.pairinfo[passcnt].box_max_x = first_max_x;

								if (first_min_y < second_min_y)
								pair.pairinfo[passcnt].box_min_y = first_min_y;

								else
								pair.pairinfo[passcnt].box_min_y = second_min_y;

								if (first_max_y < second_max_y)
								pair.pairinfo[passcnt].box_max_y = second_max_y;

								else
								pair.pairinfo[passcnt].box_max_y = first_max_y;

								pair.pairinfo[passcnt].box_center_x = (pair.pairinfo[passcnt].box_min_x + pair.pairinfo[passcnt].box_max_x) / 2;
								pair.pairinfo[passcnt].box_center_y = (pair.pairinfo[passcnt].box_min_y + pair.pairinfo[passcnt].box_max_y) / 2;

								pair.pairinfo[passcnt].leftArea = second_Area;
								pair.pairinfo[passcnt].rightArea = first_Area;

								pair.pairinfo[passcnt].leftVertical = second_Y_Vertical;
								pair.pairinfo[passcnt].rightVertical = first_Y_Vertical;

								pair.pairinfo[passcnt].boxWidth = pair.pairinfo[passcnt].box_max_x - pair.pairinfo[passcnt].box_min_x;
								pair.pairinfo[passcnt].boxHeight = pair.pairinfo[passcnt].box_max_y - pair.pairinfo[passcnt].box_min_y;
								pair.pairinfo[passcnt].boxRatio = (pair.pairinfo[passcnt].boxWidth) / (pair.pairinfo[passcnt].box_max_y - pair.pairinfo[passcnt].box_min_y);
								}

								passcnt++;
								pair.pair_cnt++;

								}*/

								// Y���� ��ġ�� �ʴ� ���� Pair�� ��������
								if (first_max_y <= second_min_y || second_max_y <= first_min_y)
								{
									continue;
								}

								else
								{
									// ��ġ��� �ϴµ� �� �Ʒ��� ��ġ�� ���� ����
									if (first_min_x <= second_min_x && second_max_x <= first_max_x)
										continue;
									else if (first_min_x >= second_min_x && second_max_x >= first_max_x)
										continue;
									else if (second_min_x <= first_max_x && first_max_x <= second_max_x)
										continue;
									else if (first_min_x <= second_max_x && first_max_x >= second_max_x)
										continue;
								}

								// �� �� �� �ؿ� �ִ� blob�� y�� ��ǥ�� max�� ����
								if (first_max_y < second_max_y)
									max_y = second_max_y;
								else
									max_y = first_max_y;


								//�ʹ� ���� ������ ���� Pair�� ��������. ����̰� �����̰� ������� BLOB_DISTANCE�� ����� ���x
								//�Ÿ� �����ʿ�. ���϶���Ʈ�� ��쿡�� �̰ź��� �� �������� ��찡 ���� �� �� ����.
								if (first_max_x < second_min_x)
								{
									if (second_min_x - first_max_x > BLOB_DISTANCE)
										continue;
								}

								else
								{
									if (first_min_x - second_max_x > BLOB_DISTANCE)
										continue;
								}

								//�� Blob������ ���� �Ÿ��� ����
								if (first_min_x < second_min_x)
								{
									//������ �������� �� �� �� �Ʒ��� ��ġ�� ���.
									if (first_max_x > second_min_x)
									{
										continue; 
									}
									else
									{
										Inner_width = second_min_x - first_max_x;
									}
								}
									
								else
								{
									if (second_max_x > first_min_x)
									{
										continue;//Inner_width = second_max_x - first_min_x;
									}
									else
									{
										Inner_width = first_min_x - second_max_x;
									}
								}
									
								//�� �� ���϶���Ʈ��� Ȯ���ϰ� �������� ���� ��쿡��.
								if (DetectionInfo.rg[i].type == 1 && DetectionInfo.rg[j].type == 1)
								{
									if (max_y <= ROI_FIRST_END)
									{
										if (PAIR_BLOB_TAIL_FIRST_DISTANCE_MIN > Inner_width || Inner_width > PAIR_BLOB_TAIL_FIRST_DISTANCE_MAX)
										{
											continue;
										}
									}
									else if (max_y <= ROI_SECOND_END)
									{
										if (PAIR_BLOB_TAIL_SECOND_DISTANCE_MIN > Inner_width || Inner_width > PAIR_BLOB_TAIL_SECOND_DISTANCE_MAX)
										{
											continue;
										}
									}
									else if (max_y <= ROI_THIRD_END)
									{
										if (PAIR_BLOB_TAIL_THIRD_DISTANCE_MIN > Inner_width || Inner_width > PAIR_BLOB_TAIL_THIRD_DISTANCE_MAX)
										{
											continue;
										}
									}
									else if (max_y <= ROI_FOURTH_END)
									{
										if (PAIR_BLOB_TAIL_FOURTH_DISTANCE_MIN > Inner_width || Inner_width > PAIR_BLOB_TAIL_FOURTH_DISTANCE_MAX)
										{
											continue;
										}

									}
									else if (max_y <= ROI_MAX)
									{
										if (PAIR_BLOB_TAIL_FIFTH_DISTANCE_MIN > Inner_width || Inner_width > PAIR_BLOB_TAIL_FIFTH_DISTANCE_MAX)
										{
											continue;
										}
									}

									else
									{
										Trace(_T("Something wrong!"));
										continue;
									}

								}
								// ��� - ������ ���� ����.
								else if((DetectionInfo.rg[i].type == 1 && DetectionInfo.rg[j].type == 0) || (DetectionInfo.rg[i].type == 0 && DetectionInfo.rg[j].type == 1) )
								{
									continue;
								}
								// ������ ���-���
								else
								{
									if (max_y <= ROI_FIRST_END)
									{
										if (PAIR_BLOB_FIRST_DISTANCE_MIN > Inner_width || Inner_width > PAIR_BLOB_FIRST_DISTANCE_MAX)
										{
											continue;
										}
									}
									else if (max_y <= ROI_SECOND_END)
									{
										if (PAIR_BLOB_SECOND_DISTANCE_MIN > Inner_width || Inner_width > PAIR_BLOB_SECOND_DISTANCE_MAX)
										{
											continue;
										}
									}
									else if (max_y <= ROI_THIRD_END)
									{
										if (PAIR_BLOB_THIRD_DISTANCE_MIN > Inner_width || Inner_width > PAIR_BLOB_THIRD_DISTANCE_MAX)
										{
											continue;
										}
									}
									else if (max_y <= ROI_FOURTH_END)
									{
										if (PAIR_BLOB_FOURTH_DISTANCE_MIN > Inner_width || Inner_width > PAIR_BLOB_FOURTH_DISTANCE_MAX)
										{
											continue;
										}

									}
									else if (max_y <= ROI_MAX)
									{
										if (PAIR_BLOB_FIFTH_DISTANCE_MIN > Inner_width || Inner_width > PAIR_BLOB_FIFTH_DISTANCE_MAX)
										{
											continue;
										}
									}

									else
									{
										Trace(_T("Something wrong!"));
										continue;
									}

								}

								// Pair ���� ���� �Ǵ� - score ��뿡 �� Value��.
								Area[i][j] = Compare_Area(first_Area, second_Area);
								Y_overlap[i][j] = Compare_Y_Overlap(first_min_y, second_min_y, first_max_y, second_max_y);
								Ratio_WH[i][j] = Compare_Width_and_Height(first_min_x, first_max_x, first_min_y, first_max_y, second_min_x, second_max_x, second_min_y, second_max_y);
								Distance[i][j] = Compare_Distance_Two_blobs(Inner_width, max_y);
							}
						}
						else
						{
							continue;
						}

						//Trace(_T("%d,%d\n"), passcnt, pair.pair_cnt);
					}
				}
			}
			else
			{
				continue;
			}
		}

		// Association Check
		Matching_Pairing_With_Detection(DetectionInfo.cnt, Area, Y_overlap, Ratio_WH, Distance, DetectionInfo);

		//TRACE("����\n");
	}

	/*else
	{
		for (int i = 1; i < pair.pair_cnt; i++)
		{
			first_mean = CalculateMeanValue(dib, pair.pairinfo[i].LeftBlobInfo.min_x, pair.pairinfo[i].LeftBlobInfo.max_x, pair.pairinfo[i].LeftBlobInfo.min_y, pair.pairinfo[i].LeftBlobInfo.max_y);
			second_mean = CalculateMeanValue(dib, pair.pairinfo[i].RightBlobInfo.min_x, pair.pairinfo[i].RightBlobInfo.max_x, pair.pairinfo[i].RightBlobInfo.min_y, pair.pairinfo[i].RightBlobInfo.max_y);
			first_deviation = CalculateStandardDeviation(dib, first_mean, pair.pairinfo[i].LeftBlobInfo.min_x, pair.pairinfo[i].LeftBlobInfo.max_x, pair.pairinfo[i].LeftBlobInfo.min_y, pair.pairinfo[i].LeftBlobInfo.max_y);
			second_deviation = CalculateStandardDeviation(dib, second_mean, pair.pairinfo[i].RightBlobInfo.min_x, pair.pairinfo[i].RightBlobInfo.max_x, pair.pairinfo[i].RightBlobInfo.min_y, pair.pairinfo[i].RightBlobInfo.max_y);

			// pair ���� ���� �Ǵ�
			if (Compare_Area_and_Y(pair.pairinfo[i].leftArea, pair.pairinfo[i].rightArea, pair.pairinfo[i].leftVertical, pair.pairinfo[i].rightVertical, globalPairing))
			{
				if (Compare_width_and_box_ratio(pair.pairinfo[i].LeftBlobInfo.min_x, pair.pairinfo[i].LeftBlobInfo.max_x, pair.pairinfo[i].LeftBlobInfo.min_y, pair.pairinfo[i].LeftBlobInfo.max_y,
					pair.pairinfo[i].RightBlobInfo.min_x, pair.pairinfo[i].RightBlobInfo.max_x, pair.pairinfo[i].RightBlobInfo.min_y, pair.pairinfo[i].RightBlobInfo.max_y, globalPairing))
				{

					if (Compare_correlation(pair.pairinfo[i].LeftBlobDib, pair.pairinfo[i].RightBlobDib, first_mean, second_mean, first_deviation, second_deviation, globalPairing))
					{

					}
					else
						continue;
				}
				else
					continue;
			}
			else
				continue;
		}
	}*/

	/*for (int i = 0; i < DetectionInfo.cnt; i++)
	{
	for (int j = i + 1; j < DetectionInfo.cnt; j++)
	{
	Trace(_T(" %d "), testtable[i][j]);
	}
	Trace(_T("\n"));
	}*/

	//DrawPairBox(dib, resultDib);

	for (int i = 0; i < size; i++)
	{
		delete[] Area[i];
		delete[] Y_overlap[i];
		delete[] Ratio_WH[i];
		delete[] Distance[i];
	}

	delete[] Area;
	delete[] Y_overlap;
	delete[] Ratio_WH;
	delete[] Distance;
#pragma endregion
}

/**
@brief			�� Blob���� ���� ��
return			�� Blob�� ũ���� ���� Ȯ�� 0 ~ 1
*/
double CPairing::Compare_Area(int firstArea, int secondArea)
{
	// �ʱ� pairing�� ���� ���µ� ��� ����� �ұ�?
	//int TH_dCy = 0;
	//double TH_MinA = 0, TH_MaxA, dividedValue = 0.0;
	double dividedValue = 0.0;

	/*if (globalPairing)
	{
		TH_MinA = 0.7;
		TH_MaxA = 1.9;
	}
		*/

	/*else
		TH_MaxA = max(6 * abs(firstArea - secondArea), 1);*/

	/*if (globalPairing)
		TH_dCy = 7;
	else
		TH_dCy = abs(first_Y_Vertical - second_Y_Vertical) + 5;*/

	// �� ������ �� ū ������ �и�� �ִ´�. �Ϻ��ϰ� ������ return 1
	if (firstArea < secondArea)
		dividedValue = firstArea / (double)secondArea;

	else
		dividedValue = secondArea / (double)firstArea;

	return dividedValue;
}

/**
@brief			�� Blob���� Y�� ��ħ�� ��
return			�� Blob�� Y�� ��ħ Ȯ�� 0 ~ 1
*/
double CPairing::Compare_Y_Overlap(int min_y1, int min_y2, int max_y1, int max_y2)
{
	int max_yValue = 0;
	int min_yValue = 0;
	int overlap_max_yValue = 0;
	int overlap_min_yValue = 0;

	int total_y_length = 0;
	int overlap_yValue = 0;

	double overlap_yValue_rate = 0.0;

	// �� ū y���� ��ĥ �� �ִ� �ִ� y���� ã�´�.
	if (max_y1 < max_y2)
	{
		max_yValue = max_y2;
		overlap_max_yValue = max_y1;
	}
	else
	{
		max_yValue = max_y1;
		overlap_max_yValue = max_y2;
	}

	// �� ���� y���� ��ĥ �� �ִ� �ּ� y���� ã�´�.
	if (min_y1 < min_y2)
	{
		min_yValue = min_y1;
		overlap_min_yValue = min_y2;
	}
	else
	{
		min_yValue = min_y2;
		overlap_min_yValue = min_y1;
	}

	// �� Blob�� ������ y���� ���̸� ���Ѵ�.
	total_y_length = max_yValue - min_yValue;

	// �� �߿����� �����ִ� y���� ���̸� ���Ѵ�.
	overlap_yValue = overlap_max_yValue - overlap_min_yValue;

	if (total_y_length == 0 || overlap_yValue < 0)
		return false;

	// �� Blob�� ������ ���̿� �����ִ� ���̸� ���Ѵ�. ���� ��ħ�̸�  return 1.
	overlap_yValue_rate = ((double)overlap_yValue / (double)total_y_length);

	return overlap_yValue_rate;

	/*if (overlap_yValue_rate > 0.62)
		return true;
	else
		return false;*/
}

/**
@brief			�� Blob���� Width �� Height�� ���� ���� ��
return			�� Blob�� ���� ������ Ȯ�� 0 ~1
*/
double CPairing::Compare_Width_and_Height(int r_min_x, int r_max_x, int r_min_y, int r_max_y, int l_min_x, int l_max_x, int l_min_y, int l_max_y)
{
	// �� Blob�� width�� heigth�� ���� ���Ѵ�.
	int width1 = r_max_x - r_min_x;
	int height1 = r_max_y - r_min_y;
	int width2 = l_max_x - l_min_x;
	int height2 = l_max_y - l_min_y;

	// �ش� Blob�� ������ width-height ratio�� ���Ѵ�.
	double ratio1 = width1 / (double)height1;
	double ratio2 = width2 / (double)height2;

	double result = 0.0;

	// ū ���� �и�. ratio�� ���� ������ return 1
	if (ratio1 < ratio2)
		return result = ratio1 / ratio2;
	else
		return result = ratio2 / ratio1;
}

/**
@brief			�� blob ���� �Ÿ� �� 12-05 �߰�
���� �ڵ忡���� Association Checking�� �Ű������� ������ ���� ����� �ϰ� ���� ����.
*/
double CPairing::Compare_Distance_Two_blobs(int inner_width, int max_y)
{
	double Pro_distance = 0.0;
	
	// �ּ� �ִ� �Ӱ谪 �ο�.
	if (max_y <= ROI_FIRST_END)
	{
		if (PAIR_BLOB_FIRST_DISTANCE_MIN <= inner_width && inner_width <= PAIR_BLOB_FIRST_DISTANCE_MAX)
		{
			if (((double)inner_width / PAIR_BLOB_FIRST_DISTANCE_MAX) > 1)
				Pro_distance = 0.0;
			
			else
				Pro_distance = 1 - ((double)inner_width / PAIR_BLOB_FIRST_DISTANCE_MAX);
		}
	}
	else if (max_y <= ROI_SECOND_END)
	{
		if (PAIR_BLOB_SECOND_DISTANCE_MIN <= inner_width && inner_width <= PAIR_BLOB_SECOND_DISTANCE_MAX)
		{
			if (((double)inner_width / PAIR_BLOB_SECOND_DISTANCE_MAX) > 1)
				Pro_distance = 0.0;

			else
				Pro_distance = ((double)inner_width / PAIR_BLOB_SECOND_DISTANCE_MAX);
		}
	}
	else if (max_y <= ROI_THIRD_END)
	{
		if (PAIR_BLOB_THIRD_DISTANCE_MIN <= inner_width && inner_width <= PAIR_BLOB_THIRD_DISTANCE_MAX)
		{
			if (((double)inner_width / PAIR_BLOB_THIRD_DISTANCE_MAX) > 1)
				Pro_distance = 0.0;

			else
				Pro_distance = ((double)inner_width / PAIR_BLOB_THIRD_DISTANCE_MAX);
		}
	}
	else if (max_y <= ROI_FOURTH_END)
	{
		if (PAIR_BLOB_FOURTH_DISTANCE_MIN <= inner_width && inner_width <= PAIR_BLOB_FOURTH_DISTANCE_MAX)
		{
			if (((double)inner_width / PAIR_BLOB_FOURTH_DISTANCE_MAX) > 1)
				Pro_distance = 0.0;

			else
				Pro_distance = ((double)inner_width / PAIR_BLOB_FOURTH_DISTANCE_MAX);
		}

	}
	else if (max_y <= ROI_MAX)
	{
		if (PAIR_BLOB_FIFTH_DISTANCE_MIN <= inner_width && inner_width <= PAIR_BLOB_FIFTH_DISTANCE_MAX)
		{
			if (((double)inner_width / PAIR_BLOB_FIFTH_DISTANCE_MAX) > 1)
				Pro_distance = 0.0;

			else
				Pro_distance = ((double)inner_width / PAIR_BLOB_FIFTH_DISTANCE_MAX);
		}
	}

	else
	{
		Trace(_T("Something wrong!"));
	}

	return Pro_distance;
}

// �ʺ�� 2���� blob�� ���������� box ratio. **���� Ư¡. ����� ������
boolean CPairing::Compare_width_and_box_ratio(int r_min_x, int r_max_x, int r_min_y, int r_max_y, int l_min_x, int l_max_x, int l_min_y, int l_max_y, boolean globalPairing)
{
	// box width Thresholds
	int TH_w1 = 0, TH_w2 = 0;

	// Box ratio, TH_RATIO1 = WIDTH, RATIO2 = HEIGHT
	double TH_Ratio1 = 0.0, TH_Ratio2 = 0.0;

	int width = 0, height = 0;

	width = max(r_max_x, l_max_x) - min(r_min_x, l_min_x);
	height = (max(r_max_y, l_max_y) - min(r_min_y, l_min_y));

	if (globalPairing)
	{
		TH_w1 = 15;
		TH_w2 = 180;

		TH_Ratio1 = 2.0;
		TH_Ratio2 = 8;
	}

	else
	{
		TH_w1 = max(r_max_x, l_max_x) - min(r_min_x, l_min_x) - 5;
		TH_w2 = max(r_max_x, l_max_x) - min(r_min_x, l_min_x) + 5;

		TH_Ratio1 = 0.5 * ((max(r_max_x, l_max_x) - min(r_min_x, l_min_x)) / (max(r_max_y, l_max_y) - min(r_min_y, l_min_y)));
		TH_Ratio2 = 2.5 * ((max(r_max_x, l_max_x) - min(r_min_x, l_min_x)) / (max(r_max_y, l_max_y) - min(r_min_y, l_min_y)));
	}

	/*if (TH_w1 <= width && width <= TH_w2)
	{*/
		if (TH_Ratio1 <= width / height && width / height <= TH_Ratio2)
			return TRUE;
		else
			return FALSE;
	/*}
	else
		return FALSE;*/
}

/*
dev_x = ���ʰ� �����ʰ��� x ����
dev_y = ���ʰ� �����ʰ��� y ����
l_mean = ���� blob�� ���
r_mean = ������ blob�� ���
l_sdev = ���� blob�� ǥ������
r_sdev = ������ blob�� ǥ������

**����Ư¡. ����� ������
*/
boolean CPairing::Compare_correlation(CDib& Left_dib, CDib& Right_dib, double l_mean, double r_mean, double l_sdev, double r_sdev, boolean globalPairing)
{
	CString str = _T(".\\DATA\\");
	CString str2 = _T(".\\DATA\\");
	BYTE** left_ptr = Left_dib.GetPtr();
	str.AppendFormat(_T("left2.bmp"));


	BYTE** right_ptr = Right_dib.GetPtr();
	str2.AppendFormat(_T("right2.bmp"));


	double TH_corr = 0.0, corr = 0.0;

	if (r_sdev == 0 || l_sdev == 0)
		return FALSE;

	Left_dib.Save(str);
	Right_dib.Save(str2);

	for (int i = 0; i < TEMPLATE_HEIGHT; i++)
	{
		for (int j = 0; j < TEMPLATE_WIDTH; j++)
		{
			TH_corr += ((left_ptr[i][j] - l_mean) * (right_ptr[i][j] - r_mean)) / (l_sdev * r_sdev);
		}
	}

	TH_corr /= TEMPLATE_HEIGHT * TEMPLATE_WIDTH;

	/*for (int i = 0; i < TEMPLATE_HEIGHT; i++)
	{
	for (int j = 0; j < TEMPLATE_WIDTH; j++)
	{
	corr += ((left_ptr[i][j] - l_mean) * (right_ptr[i][j] - r_mean)) / (l_sdev * r_sdev);
	}
	}
	*/
	if (TH_corr >= 0.6)
		return TRUE;
	else
		return FALSE;
}

/**
@brief		�� blob ���� ������ �˻�
count1 : detection�� ������ ����
area : blob�� ���� ��
y_over : blob�� y�� ��ħ ��
ratio : blob�� width�� height ��
Distance : �� blob ������ �Ÿ� ��
pair : pair���� ����
DetectionInfo : Detection�� ����
*/
void CPairing::Matching_Pairing_With_Detection(int count1, double** area, double** y_over, double** ratio, double** Distance, lb& DetectionInfo)
{
	register int i, j;

	int sCount = 0;
	int size = 0;
		
	double threshold = PAIR_SCORE_THRESHOLD;

	/*
	first_width = ���� �ʺ�
	second_width = ������ �ʺ�
	first_height = ���� ����
	second_height = ������ ����
	*/
	int first_width = 0, first_height = 0, second_width = 0, second_height = 0;

	// first_area = ���� ����, second_area = ������ ����
	int first_Area = 0, second_Area = 0;

	// first_Y_Vertical = ���� y �� ��ǥ, second_Y_Vertical = ������ y �� ��ǥ
	int first_Y_Vertical = 0, second_Y_Vertical = 0;

	// �� blob�� minx �� maxx, Boxratio ���� �� ���
	int first_min_x, first_max_x, first_min_y, first_max_y, second_min_x, second_max_x, second_min_y, second_max_y;
		
	// Pair ����
	int paircnt = 0;
	
	//int temp_paircnt = 0;

	/*
	index1 = i��° Blob�� pair_Number
	index2 = i��°�� �����ϴ� Blob�� Pair_Number
	*/
	int index1 = 0, index2 = 0;

	double score = 0.0;

	if (count1 > 2)
	{
#pragma region hungarian method�� ����� score matrix �����
		// ���� ��ķ� ����� ���� ��
		sCount = count1;

		// ��� detection�� tracking ���� score�� ����Ǵ� �迭
		// ���� ����� ����� ���ؼ� ���ڰ� ���ڶ� ���� 0�� �����
		// �� ����� ������ ��Ī�� ������
		// ��� ũ�Ⱑ 1 x 1 �� ��� 2 x 2�� �÷���
		if (sCount == 1)
		{
			size = 2;
		}
		else
		{
			size = sCount;
		}

		double** squareScoreMatrix = new double*[size];
		for (i = 0; i < size; i++)
		{
			squareScoreMatrix[i] = new double[size];
		}

		// score ��Ʈ���� �ʱ�ȭ
		for (i = 0; i < size; i++)
		{
			for (j = 0; j < size; j++)
			{
				//	squareScoreMatrix[i][j] = ((rand() % (int)((SCORE_THRESHOLD - 0.1) * 10)) * 0.1) + 0.1;
				squareScoreMatrix[i][j] = 0.0;
			}
		}

		// score ���� ��� �� ��
		// ��Ŀ� ����	
		// �տ� ������ ����ġ�� �ٲٸ�, �ش� Ư¡�� �켱���� �ٲ�.
		for (i = 0; i < count1; i++)
		{
			for (j = i + 1; j < count1; j++)
			{
				squareScoreMatrix[i][j] = (0.2 * area[i][j]) + (0.4 * y_over[i][j]) + (0.4 * ratio[i][j]);// + (0.25 * Distance[i][j]);
			}
		}
#pragma endregion
				
		// hungarian method
		x_y = matching.matchingProcessing(sCount, squareScoreMatrix);
		/*
		//Hungarian method�� ��������ʰ� Score_Matrix�� ���� ���� �������� �����ϸ� ����ɱ�?
		double max = 0;
		double* table = new double[count1];
		for (i = 0; i < count1; i++)
		{
			table[i] = 0;
		}

		for (i = 0; i < count1; i++)

		{
			max = 0;
			for (j = 0; j < count1; j++)
			{
				if (max < squareScoreMatrix[i][j])
				{
					max = squareScoreMatrix[i][j];
					table[i] = j;
				}
			}
		}
*/
		for (i = 0; i < count1; i++)
		{
			//int temp = table[i];
			score = squareScoreMatrix[i][x_y[i]];

			// matching�� �� ���. ��� ���� ����
			if (score > threshold /*&& dist < distThreshold*/)
			{		
				index1 = DetectionInfo.rg[i].pairNumber;
				index2 = DetectionInfo.rg[x_y[i]].pairNumber;
				
				// �� �� �ϳ��� ������ �� ���� ���� �ִٸ�
				// i��° detection pair number Ȯ��
				if (index1 != 0)
				{
					// ���� score�� ���� score���� �� ���ٸ�
					if (pair.pairinfo[index1 - 1].score < score)
					{
						// ���� �ξ��� Pair�� ����
						DetectionInfo.rg[pair.pairinfo[index1 - 1].right_index].pairNumber = 0;
						DetectionInfo.rg[pair.pairinfo[index1 - 1].left_index].pairNumber = 0;
						//temp_paircnt = paircnt;
						paircnt = index1 - 1;
					}
					else
						continue;
				}
				// i��° Blob�� pair�� �ξ��� Blob�� pair number Ȯ��
				else if (index2 != 0)
				{
					if (pair.pairinfo[index2 - 1].score < score)
					{
						DetectionInfo.rg[pair.pairinfo[index2 - 1].right_index].pairNumber = 0;
						DetectionInfo.rg[pair.pairinfo[index2 - 1].left_index].pairNumber = 0;
						//temp_paircnt = paircnt;
						paircnt = index2 - 1;
					}
					else
						continue;
				}
				else if (index1 != 0 && index2 != 0)
				{
					// ���� score�� ���� score���� �� ���ٸ�
					if (pair.pairinfo[index1 - 1].score < score)
					{
						if (pair.pairinfo[index2 - 1].score < score)
						{
							DetectionInfo.rg[pair.pairinfo[index1 - 1].right_index].pairNumber = 0;
							DetectionInfo.rg[pair.pairinfo[index1 - 1].left_index].pairNumber = 0;

							DetectionInfo.rg[pair.pairinfo[index2 - 1].right_index].pairNumber = 0;
							DetectionInfo.rg[pair.pairinfo[index2 - 1].left_index].pairNumber = 0;

							paircnt--;
							pair.pair_cnt--;

							if (index1 < index2)
							{
								//temp_paircnt = paircnt;
								paircnt = index2 - 1;
							}
							else
							{
								//temp_paircnt = paircnt;
								paircnt = index1 - 1;
							}							
						}
						else
							continue;

						for (int order = paircnt + 1; order < count1; order++)
						{
							pair.pairinfo[order] = pair.pairinfo[order + 1];
						}
					}
				}
				
				// �� ������ ��� �����Ѵٸ� ������ �����ϰ�, Pairing ����.

				first_width = DetectionInfo.rg[i].w;
				first_height = DetectionInfo.rg[i].h;
				first_Area = first_width * first_height;

				first_min_x = DetectionInfo.rg[i].min_x;
				first_max_x = DetectionInfo.rg[i].max_x;
				first_min_y = DetectionInfo.rg[i].min_y;
				first_max_y = DetectionInfo.rg[i].max_y;

				second_width = DetectionInfo.rg[x_y[i]].w;
				second_height = DetectionInfo.rg[x_y[i]].h;
				second_Area = second_width * second_height;

				second_min_x = DetectionInfo.rg[x_y[i]].min_x;
				second_max_x = DetectionInfo.rg[x_y[i]].max_x;
				second_min_y = DetectionInfo.rg[x_y[i]].min_y;
				second_max_y = DetectionInfo.rg[x_y[i]].max_y;
										
				if (first_min_x < second_min_x)
				{
					pair.pairinfo[paircnt].LeftBlobInfo = DetectionInfo.rg[i];
					pair.pairinfo[paircnt].RightBlobInfo = DetectionInfo.rg[x_y[i]];

					pair.pairinfo[paircnt].left_index = i;
					pair.pairinfo[paircnt].right_index = x_y[i];

					pair.pairinfo[paircnt].box_min_x = first_min_x;
					pair.pairinfo[paircnt].box_max_x = second_max_x;

					if (first_min_y < second_min_y)
						pair.pairinfo[paircnt].box_min_y = first_min_y;

					else
						pair.pairinfo[paircnt].box_min_y = second_min_y;

					if (first_max_y < second_max_y)
						pair.pairinfo[paircnt].box_max_y = second_max_y;

					else
						pair.pairinfo[paircnt].box_max_y = first_max_y;

					pair.pairinfo[paircnt].box_center_x = (pair.pairinfo[paircnt].box_min_x + pair.pairinfo[paircnt].box_max_x) / 2;
					pair.pairinfo[paircnt].box_center_y = (pair.pairinfo[paircnt].box_min_y + pair.pairinfo[paircnt].box_max_y) / 2;


					pair.pairinfo[paircnt].leftArea = first_Area;
					pair.pairinfo[paircnt].rightArea = second_Area;

					pair.pairinfo[paircnt].leftVertical = first_Y_Vertical;
					pair.pairinfo[paircnt].rightVertical = second_Y_Vertical;

					pair.pairinfo[paircnt].boxWidth = pair.pairinfo[paircnt].box_max_x - pair.pairinfo[paircnt].box_min_x;
					pair.pairinfo[paircnt].boxHeight = pair.pairinfo[paircnt].box_max_y - pair.pairinfo[paircnt].box_min_y;
					pair.pairinfo[paircnt].boxRatio = (pair.pairinfo[paircnt].boxWidth) / (pair.pairinfo[paircnt].boxHeight);
				} 
				else
				{					
					pair.pairinfo[paircnt].LeftBlobInfo = DetectionInfo.rg[x_y[i]];
					pair.pairinfo[paircnt].RightBlobInfo = DetectionInfo.rg[i];

					pair.pairinfo[paircnt].left_index = x_y[i];
					pair.pairinfo[paircnt].right_index = i;

					pair.pairinfo[paircnt].box_min_x = second_min_x;
					pair.pairinfo[paircnt].box_max_x = first_max_x;

					if (first_min_y < second_min_y)
						pair.pairinfo[paircnt].box_min_y = first_min_y;

					else
						pair.pairinfo[paircnt].box_min_y = second_min_y;

					if (first_max_y < second_max_y)
						pair.pairinfo[paircnt].box_max_y = second_max_y;

					else
						pair.pairinfo[paircnt].box_max_y = first_max_y;

					pair.pairinfo[paircnt].box_center_x = (pair.pairinfo[paircnt].box_min_x + pair.pairinfo[paircnt].box_max_x) / 2;
					pair.pairinfo[paircnt].box_center_y = (pair.pairinfo[paircnt].box_min_y + pair.pairinfo[paircnt].box_max_y) / 2;

					pair.pairinfo[paircnt].leftArea = second_Area;
					pair.pairinfo[paircnt].rightArea = first_Area;

					pair.pairinfo[paircnt].leftVertical = second_Y_Vertical;
					pair.pairinfo[paircnt].rightVertical = first_Y_Vertical;

					pair.pairinfo[paircnt].boxWidth = pair.pairinfo[paircnt].box_max_x - pair.pairinfo[paircnt].box_min_x;
					pair.pairinfo[paircnt].boxHeight = pair.pairinfo[paircnt].box_max_y - pair.pairinfo[paircnt].box_min_y;
					pair.pairinfo[paircnt].boxRatio = (pair.pairinfo[paircnt].boxWidth) / (pair.pairinfo[paircnt].box_max_y - pair.pairinfo[paircnt].box_min_y);
				}

				if (DetectionInfo.rg[i].type == 1 && DetectionInfo.rg[x_y[i]].type == 1)
				{
					// ���϶���Ʈ
					pair.pairinfo[paircnt].pairType = 1;
				}
				else
				{
					// ������Ʈ
					pair.pairinfo[paircnt].pairType = 0;
				}


				DetectionInfo.rg[i].pairNumber = paircnt + 1;
				DetectionInfo.rg[x_y[i]].pairNumber = paircnt + 1;

				pair.pairinfo[paircnt].score = squareScoreMatrix[i][x_y[i]];
				pair.pairinfo[paircnt].pairNumber = paircnt + 1;

				paircnt = pair.pair_cnt;
				paircnt++;
				pair.pair_cnt++;

			}
			else
			{
				int a = 0;
			}
		}

		// �޸� ����
		if (sCount != 1)
		{
			for (i = 0; i < sCount; i++)
			{
				delete[] squareScoreMatrix[i];
			}

			delete[] squareScoreMatrix;
		}
		else
		{
			for (i = 0; i < 2; i++)
			{
				delete[] squareScoreMatrix[i];
			}

			delete[] squareScoreMatrix;
		}
		//delete[] table;
		x_y = NULL;
	}
}

/*
@brief			���� �� �ִ� PairBox�� ������ ����.
Detection Blob�� ���� ���� �޶�����.
*/
void CPairing::SetPairInfo(lb& DetectionInfo)
{
	pair.pairinfo = new Pair[DetectionInfo.cnt / 2];
}

/*
@brief			 Pairing�� ���� Blob�� �ִ� Box�� Draw
*/
void CPairing::DrawPairBox(CDib& dib, CDib& result)
{
	if( pair.pair_cnt <= 0 )			// 2016-12-26 JMR
		return; 

	int i, j, k;
	int w = dib.GetWidth();
	int h = dib.GetHeight();


	RGBBYTE** res_ptr = result.GetRGBPtr();

	for (k = 0; k < pair.pair_cnt; k++) 
	{
		for (j = 1; j < h - 1; j++)
			for (i = 1; i < w - 1; i++)
			{
				// ���϶���Ʈ
				if (pair.pairinfo[k].pairType == 1)
				{
					if ((j == pair.pairinfo[k].box_min_y || j == pair.pairinfo[k].box_max_y) && i >= pair.pairinfo[k].box_min_x && i <= pair.pairinfo[k].box_max_x)
					{
						res_ptr[j - 1][i - 1].r = 244;
						res_ptr[j - 1][i - 1].g = 100;
						res_ptr[j - 1][i - 1].b = 34;

						res_ptr[j][i].r = 244;
						res_ptr[j][i].g = 100;
						res_ptr[j][i].b = 34;

						res_ptr[j + 1][i + 1].r = 244;
						res_ptr[j + 1][i + 1].g = 100;
						res_ptr[j + 1][i + 1].b = 34;
					}

					/*if (j == pair.pairinfo[k].box_max_y && i >= pair.pairinfo[k].box_min_x &&i <= pair.pairinfo[k].box_max_x)
					{
					res_ptr[j][i].r = 0;
					res_ptr[j][i].g = 255;
					res_ptr[j][i].b = 0;
					}*/

					if (j <= pair.pairinfo[k].box_max_y && j >= pair.pairinfo[k].box_min_y && (i == pair.pairinfo[k].box_min_x || i == pair.pairinfo[k].box_max_x))
					{
						res_ptr[j - 1][i - 1].r = 244;
						res_ptr[j - 1][i - 1].g = 100;
						res_ptr[j - 1][i - 1].b = 34;

						res_ptr[j][i].r = 244;
						res_ptr[j][i].g = 100;
						res_ptr[j][i].b = 34;

						res_ptr[j + 1][i + 1].r = 244;
						res_ptr[j + 1][i + 1].g = 100;
						res_ptr[j + 1][i + 1].b = 34;
					}
				}

				// ������Ʈ
				else
				{
					if ((j == pair.pairinfo[k].box_min_y || j == pair.pairinfo[k].box_max_y) && i >= pair.pairinfo[k].box_min_x && i <= pair.pairinfo[k].box_max_x)
					{
						res_ptr[j - 1][i - 1].r = 0;
						res_ptr[j - 1][i - 1].g = 255;
						res_ptr[j - 1][i - 1].b = 0;

						res_ptr[j][i].r = 0;
						res_ptr[j][i].g = 255;
						res_ptr[j][i].b = 0;

						res_ptr[j + 1][i + 1].r = 0;
						res_ptr[j + 1][i + 1].g = 255;
						res_ptr[j + 1][i + 1].b = 0;
					}

					/*if (j == pair.pairinfo[k].box_max_y && i >= pair.pairinfo[k].box_min_x &&i <= pair.pairinfo[k].box_max_x)
					{
					res_ptr[j][i].r = 0;
					res_ptr[j][i].g = 255;
					res_ptr[j][i].b = 0;
					}*/

					if (j <= pair.pairinfo[k].box_max_y && j >= pair.pairinfo[k].box_min_y && (i == pair.pairinfo[k].box_min_x || i == pair.pairinfo[k].box_max_x))
					{
						res_ptr[j - 1][i - 1].r = 0;
						res_ptr[j - 1][i - 1].g = 255;
						res_ptr[j - 1][i - 1].b = 0;

						res_ptr[j][i].r = 0;
						res_ptr[j][i].g = 255;
						res_ptr[j][i].b = 0;

						res_ptr[j + 1][i + 1].r = 0;
						res_ptr[j + 1][i + 1].g = 255;
						res_ptr[j + 1][i + 1].b = 0;
					}
				}
			}
	}

#pragma region Ȯ�ο� Draw ROI
	/*
	for (j = 1; j < h - 1; j++)
		for (i = 1; i < w - 1; i++)
		{
			if (j == ROI_MIN)
			{
				res_ptr[j - 1][i - 1].r = 255;
				res_ptr[j - 1][i - 1].g = 0;
				res_ptr[j - 1][i - 1].b = 255;

				res_ptr[j][i].r = 255;
				res_ptr[j][i].g = 0;
				res_ptr[j][i].b = 255;

				res_ptr[j + 1][i + 1].r = 255;
				res_ptr[j + 1][i + 1].g = 0;
				res_ptr[j + 1][i + 1].b = 255;
			}
			else if (j == ROI_FIRST_END)
			{
				res_ptr[j - 1][i - 1].r = 255;
				res_ptr[j - 1][i - 1].g = 0;
				res_ptr[j - 1][i - 1].b = 255;

				res_ptr[j][i].r = 255;
				res_ptr[j][i].g = 0;
				res_ptr[j][i].b = 255;

				res_ptr[j + 1][i + 1].r = 255;
				res_ptr[j + 1][i + 1].g = 0;
				res_ptr[j + 1][i + 1].b = 255;
			}
			else if (j == ROI_SECOND_END)
			{
				res_ptr[j - 1][i - 1].r = 255;
				res_ptr[j - 1][i - 1].g = 0;
				res_ptr[j - 1][i - 1].b = 255;

				res_ptr[j][i].r = 255;
				res_ptr[j][i].g = 0;
				res_ptr[j][i].b = 255;

				res_ptr[j + 1][i + 1].r = 255;
				res_ptr[j + 1][i + 1].g = 0;
				res_ptr[j + 1][i + 1].b = 255;
			}
			else if (j == ROI_THIRD_END)
			{
				res_ptr[j - 1][i - 1].r = 255;
				res_ptr[j - 1][i - 1].g = 0;
				res_ptr[j - 1][i - 1].b = 255;

				res_ptr[j][i].r = 255;
				res_ptr[j][i].g = 0;
				res_ptr[j][i].b = 255;

				res_ptr[j + 1][i + 1].r = 255;
				res_ptr[j + 1][i + 1].g = 0;
				res_ptr[j + 1][i + 1].b = 255;
			}
			else if (j == ROI_FOURTH_END)
			{
				res_ptr[j - 1][i - 1].r = 255;
				res_ptr[j - 1][i - 1].g = 0;
				res_ptr[j - 1][i - 1].b = 255;

				res_ptr[j][i].r = 255;
				res_ptr[j][i].g = 0;
				res_ptr[j][i].b = 255;

				res_ptr[j + 1][i + 1].r = 255;
				res_ptr[j + 1][i + 1].g = 0;
				res_ptr[j + 1][i + 1].b = 255;
			}
			else if (j == ROI_MAX)
			{
				res_ptr[j - 1][i - 1].r = 255;
				res_ptr[j - 1][i - 1].g = 0;
				res_ptr[j - 1][i - 1].b = 255;

				res_ptr[j][i].r = 255;
				res_ptr[j][i].g = 0;
				res_ptr[j][i].b = 255;

				res_ptr[j + 1][i + 1].r = 255;
				res_ptr[j + 1][i + 1].g = 0;
				res_ptr[j + 1][i + 1].b = 255;
			}
		}		
	*/
#pragma endregion
}

/*
@brief			 Pairing�� ���Ǵ� ������ ����ü �ʱ�ȭ
*/
void CPairing::initPairing(lb& DetectionInfo)
{
	CDib empty;
	empty.CreateGrayImage(0, 0, 0);
	sRegion emptysr;
	emptysr.type = -1;
//	emptysr.pairnum = 0;

	
	for (int passcnt = 0; passcnt < DetectionInfo.cnt; passcnt++)
	{
		pair.pairinfo[passcnt].LeftBlobDib = empty;
		pair.pairinfo[passcnt].RightBlobDib = empty;
		pair.pairinfo[passcnt].LeftBlobInfo = emptysr;
		pair.pairinfo[passcnt].RightBlobInfo = emptysr;

		pair.pairinfo[passcnt].left_index = 0;
		pair.pairinfo[passcnt].right_index = 0;

		pair.pairinfo[passcnt].box_min_x = 0;
		pair.pairinfo[passcnt].box_max_x = 0;
		pair.pairinfo[passcnt].box_min_y = 0;
		pair.pairinfo[passcnt].box_max_y = 0;
		pair.pairinfo[passcnt].box_center_x = 0;
		pair.pairinfo[passcnt].box_center_y = 0;

		pair.pairinfo[passcnt].leftArea = 0;
		pair.pairinfo[passcnt].rightArea = 0;

		pair.pairinfo[passcnt].leftVertical = 0;
		pair.pairinfo[passcnt].rightVertical = 0;

		pair.pairinfo[passcnt].boxWidth = 0;
		pair.pairinfo[passcnt].boxHeight = 0;
		pair.pairinfo[passcnt].boxRatio = 0;

		pair.pairinfo[passcnt].score = 0.0;
		DetectionInfo.rg[passcnt].pairNumber = 0;
	}
	
	beamControl = false;
	pair.pair_cnt = 0;

}

// Y�� ����
void CPairing::DibMirror(CDib& dib)
{
	register int i, j;
	int w = dib.GetWidth();
	int h = dib.GetHeight();

	CDib cpy = dib;

	//dib.CreateGrayImage(w, h);

	BYTE** ptr1 = cpy.GetPtr();
	BYTE** ptr2 = dib.GetPtr();

	for (j = 0; j < h; j++)
	{
		for (i = 0; i < w; i++)
		{
			ptr2[j][i] = ptr1[j][w - 1 - i];
		}
	}
}

/*
@brief			 Pair���� ������ �˾� �� �� �ִ�.
*/
PairInfo CPairing::getpairinfo()
{
	return pair;
}

// ROI �޾ƿ���
void CPairing::set_ROI_In_pairing(int ROI_MIN, int ROI_MAX)
{
	this->ROI_MIN = ROI_MIN;
	this->ROI_MAX = ROI_MAX;

	ROI_HEIGHT = this->ROI_MAX - this->ROI_MIN;

	ROI_FIRST_END = ROI_MIN + ROI_HEIGHT * 0.1 * PAIR_ROI_CROP_FIRST;
	ROI_SECOND_END = ROI_MIN + ROI_HEIGHT * 0.1 * PAIR_ROI_CROP_SECOND;
	ROI_THIRD_END = ROI_MIN + ROI_HEIGHT * 0.1 * PAIR_ROI_CROP_THIRD;
	ROI_FOURTH_END = ROI_MIN + ROI_HEIGHT * 0.1 * PAIR_ROI_CROP_FOURTH;

	/*
	ROI_FIRST_END_OVER = ROI_FIRST_END + ROI_HEIGHT * 0.1 * PAIR_ROI_CROP_OVERLAP_RATIO;
	ROI_SECOND_END_OVER = ROI_SECOND_END + ROI_HEIGHT * 0.1 * PAIR_ROI_CROP_OVERLAP_RATIO;
	ROI_THIRD_END_OVER = ROI_THIRD_END + ROI_HEIGHT * 0.1 * PAIR_ROI_CROP_OVERLAP_RATIO;
	ROI_FOURTH_END_OVER = ROI_FOURTH_END + ROI_HEIGHT * 0.1 * PAIR_ROI_CROP_OVERLAP_RATIO;

	PAIR_BLOB_FIRST_DISTANCE = PAIR_BLOB_FIRST_DISTANCE_MAX + PAIR_BLOB_FIRST_DISTANCE_MIN;
	PAIR_BLOB_SECOND_DISTANCE = PAIR_BLOB_SECOND_DISTANCE_MAX + PAIR_BLOB_SECOND_DISTANCE_MIN;
	PAIR_BLOB_THIRD_DISTANCE = PAIR_BLOB_THIRD_DISTANCE_MAX + PAIR_BLOB_THIRD_DISTANCE_MIN;
	PAIR_BLOB_FOURTH_DISTANCE = PAIR_BLOB_FOURTH_DISTANCE_MAX + PAIR_BLOB_FOURTH_DISTANCE_MIN;
	PAIR_BLOB_FIFTH_DISTANCE = PAIR_BLOB_FIFTH_DISTANCE_MAX + PAIR_BLOB_FIFTH_DISTANCE_MIN;
	*/
}

/*
@brief			 Beam - Control.
Text ���
HIGH - BEAM, LOW-BEAM ����
*/
void CPairing::setbeamText(CDC* pDC)
{
	int h_count = 0, l_count = 0 ;
	pDC->SetTextColor(RGB(0, 255, 0));
	pDC->SetBkMode(TRANSPARENT);

	/* 
	CHECK FRAME���� ���� ��, ����� ���翩�ο� ���� HIGH-BEAM�� LOW-BEAM�� �״�� ���.
	���ÿ� CHECK FRAME�� ������ �� ���� ����ؼ� HIGH-BEAM �� LOW-BEAM�� ���� Ƚ���� �����ϸ�, CHECK FRAME�� ��ŭ QUEUE�� �ִ´�.
	*/
	if(currentframe < HL_FRAME_CHECK)
	{
		// 1������ �۴�. �� �� ����. ���̺�
		if (pair.pair_cnt < 1)
		{
			pDC->TextOutW(560, 74, _T("High-Beam"));
			beamControl = true;
			histo_HL[HIGH_BEAM]++;
			histo_HL_Queue[currentframe] = HIGH_BEAM;
		}
		else
		{
			pDC->TextOutW(560, 74, _T("Low-Beam"));
			beamControl = false;
			histo_HL[LOW_BEAM]++;
			histo_HL_Queue[currentframe] = LOW_BEAM;
		}
	}
	/* 
	���� CHECKFRAME�� �Ѿ�ٸ�, ���ݱ��� �����ؿ� HL ���ÿ� ���� BEAM���¸� ���ѵڿ� �� ���� ���� BEAM�� ������ش�.
	�׿� ���ÿ� �ش� BEAM�� QUEUE�� �ִ´�. �������� Adjust_HL_Queue �Լ��� ȣ���ؼ� QUEUE�� ���� ó�� BEAM ���� �����ϰ�, �׿� ���ÿ� HL���ÿ����� �ش� BEAM�� �����Ѵ�.
	�� CHECK FRAME ��ŭ�� ������ QUEUE�� HL ���ÿ� �Ҵ�ȴ�.
	*/
	else
	{
		if (pair.pair_cnt < 1)
		{
			histo_HL[HIGH_BEAM]++;
			histo_HL_Queue[HL_FRAME_CHECK] = HIGH_BEAM;

			if(histo_HL[HIGH_BEAM] > histo_HL[LOW_BEAM])
			{	
				pDC->TextOutW(560, 74, _T("High-Beam"));
				beamControl = true;
			}
			else
			{
				pDC->TextOutW(560, 74, _T("Low-Beam"));
				beamControl = false;
			}
		}
		else
		{
			histo_HL[LOW_BEAM]++;
			histo_HL_Queue[HL_FRAME_CHECK] = LOW_BEAM;

			if(histo_HL[HIGH_BEAM] < histo_HL[LOW_BEAM])
			{	
				pDC->TextOutW(560, 74, _T("Low-Beam"));
				beamControl = false;
			}
			else
			{
				pDC->TextOutW(560, 74, _T("High-Beam"));
				beamControl = true;
			}
		}

		// HIGH-BEAM�� LOW-BEAM�� ���������� ��Ÿ���� Histogram�� Queue�� ����
		Adjust_HL_Queue();
	}
	
	pDC->SetTextColor(RGB(255, 255, 0));
	pDC->SetBkMode(TRANSPARENT);
	pDC->TextOutW(220,ROI_MIN - 50, _T("ROI"));
	pair.pair_cnt = 0;
}

/*
@brief			 Beam - Control.
���� Beam�� HIGH���� LOW����
*/
boolean CPairing::getBeamresult()
{
	return beamControl;
}

/*
@brief			 Beam - Control.
���� ������ ����
*/
void CPairing::setCurrentFrame(int m_currentframe)
{
	this->currentframe = m_currentframe - 1;
}

/*
@brief			 Beam - Control.
HIGH_BEAM + 1
*/
void CPairing:: plusHB_Count()
{
	histo_HL[HIGH_BEAM]++;
}

/*
@brief			 Beam - Control.
HIGH_BEAM - 1
*/
void CPairing:: MinusHB_Count()
{
	histo_HL[HIGH_BEAM]--;
}

/*
@brief			 Beam - Control.
LOW_BEAM + 1
*/
void CPairing:: PlusLB_Count()
{
	histo_HL[LOW_BEAM]++;
}

/*
@brief			 Beam - Control.
LOW_BEAM - 1
*/
void CPairing:: MinusLB_Count()
{
	histo_HL[LOW_BEAM]--;
}

/*
@brief			 Beam - Control.
������ QUEUE�� HIGH_BEAM���� set
����� ������� ����
*/
void CPairing:: set_Histo_Queue_HIGH(int currentFrame)
{
	histo_HL_Queue[currentFrame] = HIGH_BEAM;
}

/*
@brief			 Beam - Control.
������ QUEUE�� LOW_BEAM���� set
����� ������� ����
*/
void CPairing:: set_Histo_Queue_LOW(int currentFrame)
{
	histo_HL_Queue[currentFrame] = LOW_BEAM;
}

/*
@brief			 Beam - Control.
QUEUE�� ���� �տ� �Ҵ� ��Ұ� LOW���� HIGH���� �Ǵ��ؼ� QUEUE�� ����.
*/
void CPairing::Adjust_HL_Queue()
{
	if(histo_HL_Queue[0] == HIGH_BEAM)
	{
		histo_HL[HIGH_BEAM]--;
	}
	else
	{
		histo_HL[LOW_BEAM]--;
	}

	for(int i =0; i < HL_FRAME_CHECK; i++)
	{
		histo_HL_Queue[i] = histo_HL_Queue[i + 1];
	}
}

/*
@brief			 Beam - Control.
HIGH_BEAM�� LOW_BEAM�� ���� ���� �ʱ�ȭ.
*/
void CPairing::initPairingValue()
{
	for(int i=0; i<2; i++)
	{
		histo_HL[i] = 0;
	}

	for(int i=0; i<= HL_FRAME_CHECK; i++)
	{
		histo_HL_Queue[i] = -1;
	}
}