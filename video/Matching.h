#pragma once

#define INF		100000	
#define NUM		1024
#define FREE	-1	

class CMatching
{
public:
	CMatching(void);
	~CMatching(void);


public:
	int* matchingProcessing(int n, double** cost_mat);

protected:
	// �밡���� �˰��� : O(n^3)	
	void hungarian(int n, double** cost_mat);

	// ���̺� �ʱ�ȭ : O(n^2)	
	void init_labels(int n, double** cost_mat);

	// M graph�� perfect���� �Ǻ� : O(1)		
	int is_perfect(int n);

	// S���հ� T������ �ʱ�ȭ : O(n)	
	void init_S_T(int n);

	// augmenting path�� ã�� �߰��ϴ� ���� (�ٽ�) : O(n^2)		
	void augment(int n, double** cost_mat);

	// �־��� x ������ ���Ǵ� T���տ� ���Ե��� �ʴ� y�� ����Ǵ�	
	// ��� �������(delta)�� ���Ͽ� ���� slack y���� ���Ͽ� 	
	// �� ���� ���̸� ������Ʈ �ϰ� �� x ���� ����Ѵ�	 : O(n)	
	void slack_update(int x, int n, double** cost_mat);

	// T ���տ� ���Ե��� �ʴ� y���� slack���� ���� ���� ���� ã��	
	// �ε����� ��ȯ�Ѵ�	 : O(n)	
	int find_in_slack(int n);

	// �� ������Ʈ �� slack y�� ���� : 3 * O(n)	
	void update_labels(double delta, int n);

	// matching graph �� ���� : O(n) , tail recursion	
	void reconstruct_matching_graph(int x, int y, int n);



protected:
	double distance[50];
	double probability[50];

	// hungarian method�� ���Ǵ� ����
	double lx[NUM];		// label x (��갪)	
	double ly[NUM];		// label y (��갪)

	int S[NUM];			// S ����, ť �ڷᱸ��, x���� �ش�Ǵ� (�ε�����) 	
	int s_read;			// dequeue�� ���� ����
	int s_write;		// enqueue�� ���� ����
	int T[NUM];			// T ����, �������� ������ 0�� ����, �����ϸ� 1�� ���� ���Ѵ�	 (bool��)

	// Matching graph	
	int x_y[NUM];			// x ������ ��Ī�Ǵ� y���� ��Ÿ����, (�ε�����)	
	int y_x[NUM];			// y ������ ��Ī�Ǵ� x���� ��Ÿ����, (�ε�����)

	// O(n^4) �� O(n^3)���� ���̱� ���� ����ϴ� slack ����	
	double slacky[NUM];		// �ش� y���� ���ϴ� ���� ���� delta ���� ��Ÿ����	 (��갪)
	int slack_x[NUM];		// �ش� y������ ���� ���� delta������ ����� x���� ��Ÿ����	 (�ε�����)

	int cnt_match;		// �� ��Ī�� ��, �̰��� n�� �������� ���� ��Ī�� ���̴�
};