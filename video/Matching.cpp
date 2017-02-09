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

// �밡���� �˰��� : O(n^3)
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

// �󺧸� �ʱ�ȭ : O(n^2)	
void CMatching::init_labels(int n, double** cost_mat)
{
	register int i, j;
	int index = 0;

	cnt_match = 0;

	// Matching graph �ʱ�ȭ
	memset(x_y, FREE, sizeof(int) * n);
	memset(y_x, FREE, sizeof(int) * n); 

	for(i = 0; i < n; i++)
	{
		lx[i] = -1 * INF;
		ly[i] = 0;

		// �� ������ max���� ã�� �󺧸�, �ε����� �˾Ƴ���	
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

		// �ʱ� Matching graph ����
		if(y_x[index] == FREE)
		{
			x_y[i] = index;
			y_x[index] = i;

			cnt_match++;
		}
		/*
		��Ī �׷������� y�� �̹� ���� ������ ������ (FREE���� ������)
		������ ����� ���̹Ƿ� alternating tree�� �ݵ�� �����Ѵ�
		*/
	}

}

// M graph�� perfect���� �Ǻ� : O(1)	
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

// S���հ� T������ �ʱ�ȭ : O(n)
void CMatching::init_S_T(int n)
{
	register int i;

	// S ������ ť�̹Ƿ� ,read������ write������ �ʱ�ȭ
	s_read = s_write = 0;

	// T ������ bool���� ���ϹǷ� 0���� �ʱ�ȭ
	memset(T, 0, sizeof(int) * n);

	for(i = 0; i < n; i++)
	{
		if(x_y[i] == FREE)
		{
			S[s_write++] = i; // free�� x �� �ϳ��� �����Ͽ� S���տ� �߰��Ѵ�	
			break;
		}
	}
}

// augmenting path�� ã�� �߰��ϴ� ���� (�ٽ�) : O(n^2)	
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

	// ��� S ���տ� ���Ͽ� ��� : ť , �־��� ��� n-1 ��ŭ �ݺ��ȴ�
	while(s_read < s_write)
	{
		// S ���տ��� �ϳ��� �о�� slack ��� : O(n)
		slack_update(S[s_read++], n, cost_mat); 

		// slack���� ���� ���� ����� ���� ������ ã�´�	 : O(n)
		y = find_in_slack(n); 
		
		x = slack_x[y];

		// �� �ش� y���� free�ϸ� augmenting path�� ã�� ���̴�!!
		if(y_x[y] == FREE)
		{
			break;
		}

		// �� ������ �Ѵ�	 : 3 x O(n)
		update_labels(slacky[y], n);

		// alternating tree Ȯ�� : �ش� y���� matched �̱� ����
		S[s_write++] = y_x[y]; // ������ ��ġ�� y���� ����Ǿ� �ִ� x���� S���տ� �߰�
		T[y] = 1; // T���տ� y �߰�
	}

	// ã�Ƴ� augmenting path�� �������� Matching graph ������ : O(n)
	reconstruct_matching_graph(x, y, n);
	cnt_match++;
}	

// �־��� x ������ ���Ǵ� T���տ� ���Ե��� �ʴ� y�� ����Ǵ�	
// ��� �������(delta)�� ���Ͽ� ���� slack y���� ���Ͽ� 	
// �� ���� ���̸� ������Ʈ �ϰ� �� x ���� ����Ѵ�	 : O(n)	
void CMatching::slack_update(int x, int n, double** cost_mat)
{
	register int i;
	double delta;

	for(i = 0; i < n; i++)
	{
		// double Ÿ���̶� �׷��� 0�� �־ -0.000018 ���� �������� ���� ������
		// 0���� ������ �������ش�
		// ������ ly�� lx�� �ݴ�� ���� 0.000018 ���� ���� ���� ������
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

// T ���տ� ���Ե��� �ʴ� y���� slack���� ���� ���� ���� ã��	
// �ε����� ��ȯ�Ѵ�	 : O(n)	
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

// �� ������Ʈ �� slack y�� ���� : 3 * O(n)		
void CMatching::update_labels(double delta, int n)
{
	register int i;

	// S ���տ� ���ϴ� x�� �� ������Ʈ
	for(i = 0; i < n; i++)
	{
		if(S[i])
		{
			lx[i] -= delta;
		}
	}
	// T ���տ� ���ϴ� y�� �� ������Ʈ	
	for(i = 0; i < n; i++)
	{
		if(T[i])
		{
			ly[i] += delta;
		}
	}

	// slacky ���ٰ��� delta ���� ���ִ� ������	
	// T���տ� ������ �ʴ� y�鿡 ���� slacky�� ������ 	
	// �ּҺ�� �ĺ����̱� ������ label x �� delta��ŭ 	
	// �پ��� ���̰� �����	 �̰��� Equality �Ӽ��� 
	// �����ϰ� �����Ѵ�	
	// �ٷ� ������ ������ ������ slacky ���� 0�� �� ���̴�	 (���Ŀ� T���տ� ���Եȴ�)
	for(i = 0; i < n; i++)
	{
		if(!T[i])
		{
			slacky[i] -= delta;
		}
	}
}

// matching graph �� ���� : O(n) , tail recursion		
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