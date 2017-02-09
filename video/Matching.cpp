#include "StdAfx.h"
#include "Matching.h"

#define _CRTDBG_MAP_ALLOC

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif 


CMatching::CMatching(void)
{
}


CMatching::~CMatching(void)
{
}



// public
int* CMatching::matchingProcessing(int n, double** cost_mat)
{
	hungarian(n, cost_mat);

	return x_y;
}

// 헝가리안 알고리즘 : O(n^3)
void CMatching::hungarian(int n, double** cost_mat)
{
	// step 1
	init_labels(n, cost_mat);

	// step 2
	while(!is_perfect(n))
	{
		init_S_T(n); // O(n)

		// step 3,4 
		augment(n, cost_mat); // O(n^2)	
	}
}	

// 라벨링 초기화 : O(n^2)	
void CMatching::init_labels(int n, double** cost_mat)
{
	register int i, j;
	int index = 0;

	cnt_match = 0;

	// Matching graph 초기화
	memset(x_y, FREE, sizeof(int) * n);
	memset(y_x, FREE, sizeof(int) * n); 

	for(i = 0; i < n; i++)
	{
		lx[i] = -1 * INF;
		ly[i] = 0;

		// 각 열에서 max값을 찾아 라벨링, 인덱스를 알아낸다	
		for(j = 0; j < n; j++)
		{
			if(lx[i] < cost_mat[i][j])
			{
				lx[i] = cost_mat[i][j];

				if(lx[i] < 0)
				{
					lx[i] = 0;
				}
				
				index = j;
			}
		}

		// 초기 Matching graph 생성
		if(y_x[index] == FREE)
		{
			x_y[i] = index;
			y_x[index] = i;

			cnt_match++;
		}
		/*
		매칭 그래프에서 y가 이미 점을 가지고 있으면 (FREE하지 않으면)
		기존에 연결된 것이므로 alternating tree가 반드시 존재한다
		*/
	}

}

// M graph가 perfect인지 판별 : O(1)	
int CMatching::is_perfect(int n)
{
	if(cnt_match < n)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}	

// S집합과 T집합을 초기화 : O(n)
void CMatching::init_S_T(int n)
{
	register int i;

	// S 집합은 큐이므로 ,read지점과 write지점만 초기화
	s_read = s_write = 0;

	// T 집합은 bool값을 지니므로 0으로 초기화
	memset(T, 0, sizeof(int) * n);

	for(i = 0; i < n; i++)
	{
		if(x_y[i] == FREE)
		{
			S[s_write++] = i; // free한 x 점 하나를 선택하여 S집합에 추가한다	
			break;
		}
	}
}

// augmenting path를 찾아 추가하는 과정 (핵심) : O(n^2)	
void CMatching::augment(int n, double** cost_mat)
{
	register int i;
	int x = 0;
	int y = 0;

	// init slack
	for(i=0; i<n; i++)
	{
		slacky[i] = INF;
	}
	memset(slack_x, FREE, sizeof(int) * n);

	// 모든 S 집합에 대하여 계산 : 큐 , 최악의 경우 n-1 만큼 반복된다
	while(s_read < s_write)
	{
		// S 집합에서 하나를 읽어와 slack 계산 : O(n)
		slack_update(S[s_read++], n, cost_mat); 

		// slack에서 가장 작은 비용을 갖는 간선을 찾는다	 : O(n)
		y = find_in_slack(n); 
		
		x = slack_x[y];

		// 그 해당 y점이 free하면 augmenting path를 찾은 것이다!!
		if(y_x[y] == FREE)
		{
			break;
		}

		// 라벨 재조정 한다	 : 3 x O(n)
		update_labels(slacky[y], n);

		// alternating tree 확장 : 해당 y점이 matched 이기 때문
		S[s_write++] = y_x[y]; // 기존에 매치된 y점과 연결되어 있는 x점을 S집합에 추가
		T[y] = 1; // T집합에 y 추가
	}

	// 찾아낸 augmenting path를 기준으로 Matching graph 재조정 : O(n)
	reconstruct_matching_graph(x, y, n);
	cnt_match++;
}	

// 주어진 x 점에서 계산되는 T집합에 포함되지 않는 y와 연결되는	
// 모든 간선비용(delta)에 대하여 기존 slack y값과 비교하여 	
// 더 작은 값이면 업데이트 하고 그 x 점을 기록한다	 : O(n)	
void CMatching::slack_update(int x, int n, double** cost_mat)
{
	register int i;
	double delta;

	for(i = 0; i < n; i++)
	{
		// double 타입이라 그런지 0을 넣어도 -0.000018 같은 음수값이 들어가기 때문에
		// 0보다 작으면 수정해준다
		// 문제는 ly는 lx와 반대로 양의 0.000018 같이 같은 값을 가진다
		if((int)lx[x] < 0)
		{
			lx[x] = 0;
		}

		delta = lx[x] + ly[i] - cost_mat[x][i];

		if(!T[i] && slacky[i] > delta)
		{
			slacky[i] = delta;
			slack_x[i] = x;
		}
	}
}

// T 집합에 포함되지 않는 y점중 slack에서 가장 작은 값을 찾아	
// 인덱스를 반환한다	 : O(n)	
int CMatching::find_in_slack(int n)
{
	register int i;
	double min = INF;
	double max = 0;
	int index = FREE;
	
	for(i = 0; i < n; i++)
	{
		if(slacky[i] < 0.0)
		{
			slacky[i] = 0.0;
		}


		if (!T[i] && slacky[i] >= 0.0 && min > slacky[i])
		{
			min = slacky[i];
			index = i;
		}
	}

	return index;
}	

// 라벨 업데이트 및 slack y값 조정 : 3 * O(n)		
void CMatching::update_labels(double delta, int n)
{
	register int i;

	// S 집합에 속하는 x의 라벨 업데이트
	for(i = 0; i < n; i++)
	{
		if(S[i])
		{
			lx[i] -= delta;
		}
	}
	// T 집합에 속하는 y의 라벨 업데이트	
	for(i = 0; i < n; i++)
	{
		if(T[i])
		{
			ly[i] += delta;
		}
	}

	// slacky 에다가도 delta 값을 빼주는 이유는	
	// T집합에 속하지 않는 y들에 대해 slacky는 어차피 	
	// 최소비용 후보들이기 때문에 label x 가 delta만큼 	
	// 줄어들어 차이가 생긴다	 이것을 Equality 속성이 
	// 만족하게 보정한다	
	// 바로 직전에 선택한 간선의 slacky 값은 0이 될 것이다	 (직후에 T집합에 포함된다)
	for(i = 0; i < n; i++)
	{
		if(!T[i])
		{
			slacky[i] -= delta;
		}
	}
}

// matching graph 재 생성 : O(n) , tail recursion		
void CMatching::reconstruct_matching_graph(int x, int y, int n)
{	
	int tmp;
	
	if(x_y[x] != FREE)
	{
		tmp = x_y[x];
		x_y[x] = y;
		y_x[y] = x;

		reconstruct_matching_graph( slack_x[tmp], tmp, n);
	}
	else
	{
		x_y[x] = y;
		y_x[y] = x;
	}
}