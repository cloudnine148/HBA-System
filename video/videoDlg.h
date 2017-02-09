
// videoDlg.h : 헤더 파일
//

#pragma once
#include "Avi.h"
#include <string>
#include "afxwin.h"
#include "MainClass.h"

using namespace std;

// CvideoDlg 대화 상자
class CvideoDlg : public CDialogEx
{
// 생성입니다.
public:
	CvideoDlg(CWnd* pParent = NULL);	// 표준 생성자입니다.

// 대화 상자 데이터입니다.
	enum { IDD = IDD_VIDEO_DIALOG };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV 지원입니다.


// 구현입니다.
protected:
	HICON m_hIcon;

	// 생성된 메시지 맵 함수
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnTimer(UINT_PTR nIDEvent);
	afx_msg void OnBnClickedLoad();



public:
	CDib m_Dib;
	CDib m_trackingDib;
	CAvi m_Avi;

	CMainClass m_MAIN;

	CString temp_path; // 파일 여는 위치 저장 (DIR)
	CString load_path; // 파일 로드하는 위치 저장(Load)
	CString save_path; // 파일 저장할 위치 저장(SAVE)
	CString num_key; // 자릿수만큼 0붙임.
	CString eb_curFrame; // 현재프레임
	CString eb_totalFrame; // 전체프레임
	CString eb_processtime; // 처리시간
	CListBox m_lb_fname; // 리스트박스 멤버
	string* bmpsaved; // bmp의 이름이 저장된다.

	LARGE_INTEGER Frequency; // 시스템 시작 시간.

	int m_nCurrentFrame; // 현재 프레임	
	int count; // 현재 가리키는 프레임

	int count_bmp;
	int cur_bmp;
	int num_length;
	//Picture Control - Local Adaptive thresholding Binarization
	int lo_width;
	int lo_height;
	int lo_left;
	int lo_top;
	//Picture Control - Global Adaptive thresholding Binarization
	int gl_width;
	int gl_height;
	int gl_left;
	int gl_top;
	//Picture Control -	Original 
	int or_width;
	int or_height;
	int or_left;
	int or_top;
	//Picture Control - Detection
	int dt_width;
	int dt_height;
	int dt_left;
	int dt_top;
	//Picture Control - BeamControl
	int bc_width;
	int bc_height;
	int bc_left;
	int bc_top;

	bool m_bPlay; // 재생여부
	bool m_load; // 로드여부
	bool ischecked; // 체크박스 활성여부
	//bool globalPairingflag;		
	bool m_bTreefileloading;		// tree file 로드 여부	
	
public:
	afx_msg void OnBnClickedPlay();
	afx_msg void OnBnClickedStop();
	afx_msg void OnBnClickedPause();
	afx_msg void OnBnClickedNext();
	afx_msg void OnBnClickedPrev();
	afx_msg void OnBnClickedOpendir();
	afx_msg void OnLbnSelchangeList1();
	afx_msg void OnBnClickedCheck1();
	afx_msg void OnBnClickedSavedir();

public:
	void PreProcessing(CDib& dib);
	void processImage();
	void drawImage(CDib dib);
	void drawTrackingImage();	
	void gl_drawImage(CDib dib);
	void or_drawImage(CDib dib);
	void lo_drawImage(CDib dib);
	void dt_drawImage(CDib dib);
	void OriginalImage();
	void bc_drawBMP();
	afx_msg void OnBnClickedTestBtn();
	
	
	// ROI
	void DrawROI(CDib& dib);
	afx_msg void OnBnClickedLoadData();
	afx_msg void OnStnClickedOriginal();
	afx_msg void OnDestroy();

	void drawImages();
	afx_msg void OnStnClickedBeamControl();
};
