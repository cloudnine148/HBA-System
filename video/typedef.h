#pragma once

//150903 for thread
#define EV_NAME_TEST_PLAY		_T("test_play")
#define EV_NAME_TEST_PAUSE		_T("test_pause")
#define EV_NAME_TEST_STOP		_T("test_stop")
#define EV_NAME_TEST_TERM		_T("test_terminate")

extern HANDLE gevTestPlay;
extern HANDLE gevTestPause;
extern HANDLE gevTestStop;
extern HANDLE gevTestTerm;


//�Լ� ��ȯ �� �ڵ�
enum returnValEnum {
	SUCCESS=0,
	INVALID_VALUE,
	INVALID_STATE,
	FILEOPEN_FAIL,
	FILELOAD_FAIL,
	FILESAVE_FAIL,
	THREAD_TERM,
	UNKNOWN,
	KEEPGOING,
};

extern returnValEnum returnVal;

struct Point {
	int x;
	int y;
};

//���� ������ ���� ������ ���� ����ü
struct stc_detectionWindow {
	Point startp;		//������ ������
	Point endp;			//������ ������
	Point centerp;		//������ �߽���
	double prob;		//�з� Ȯ����
	bool isValid;		//flag
};
