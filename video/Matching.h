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
	// 헝가리안 알고리즘 : O(n^3)	
	void hungarian(int n, double** cost_mat);

	// 레이블링 초기화 : O(n^2)	
	void init_labels(int n, double** cost_mat);

	// M graph가 perfect인지 판별 : O(1)		
	int is_perfect(int n);

	// S집합과 T집합을 초기화 : O(n)	
	void init_S_T(int n);

	// augmenting path를 찾아 추가하는 과정 (핵심) : O(n^2)		
	void augment(int n, double** cost_mat);

	// 주어진 x 점에서 계산되는 T집합에 포함되지 않는 y와 연결되는	
	// 모든 간선비용(delta)에 대하여 기존 slack y값과 비교하여 	
	// 더 작은 값이면 업데이트 하고 그 x 점을 기록한다	 : O(n)	
	void slack_update(int x, int n, double** cost_mat);

	// T 집합에 포함되지 않는 y점중 slack에서 가장 작은 값을 찾아	
	// 인덱스를 반환한다	 : O(n)	
	int find_in_slack(int n);

	// 라벨 업데이트 및 slack y값 조정 : 3 * O(n)	
	void update_labels(double delta, int n);

	// matching graph 재 생성 : O(n) , tail recursion	
	void reconstruct_matching_graph(int x, int y, int n);



protected:
	double distance[50];
	double probability[50];

	// hungarian method에 사용되는 변수
	double lx[NUM];		// label x (계산값)	
	double ly[NUM];		// label y (계산값)

	int S[NUM];			// S 집합, 큐 자료구조, x점에 해당되는 (인덱스값) 	
	int s_read;			// dequeue를 위한 변수
	int s_write;		// enqueue를 위한 변수
	int T[NUM];			// T 집합, 존재하지 않으면 0의 값을, 존재하면 1의 값을 지닌다	 (bool값)

	// Matching graph	
	int x_y[NUM];			// x 점에서 매칭되는 y점을 나타낸다, (인덱스값)	
	int y_x[NUM];			// y 점에서 매칭되는 x점을 나타낸다, (인덱스값)

	// O(n^4) 를 O(n^3)으로 줄이기 위해 사용하는 slack 관련	
	double slacky[NUM];		// 해당 y점이 지니는 가장 작은 delta 값을 나타낸다	 (계산값)
	int slack_x[NUM];		// 해당 y점에서 가장 작은 delta값으로 연결된 x점을 나타낸다	 (인덱스값)

	int cnt_match;		// 총 매칭된 수, 이것이 n과 같아지면 전부 매칭된 것이다
};