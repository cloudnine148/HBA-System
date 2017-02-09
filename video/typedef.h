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


//함수 반환 값 코드
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

//검출 윈도우 정보 저장을 위한 구조체
struct stc_detectionWindow {
	Point startp;		//윈도우 시작점
	Point endp;			//윈도우 종료점
	Point centerp;		//윈도우 중심정
	double prob;		//분류 확률값
	bool isValid;		//flag
};
