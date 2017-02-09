//////////////////////////////////////////////////////////////////////
//                                                                  //
// Avi.h: header file of the CAvi class.                            //
//                                                                  //
// Copyright (c) 2003~<current> Sun-Kyoo Hwang                      //
//                              sunkyoo.ippbook@gmail.com           //
//                                                                  //
// 영상처리 프로그래밍 By Visual C++                                //
//                                                                  //
// Last modified: 20/04/2007                                        //
//                                                                  //
//////////////////////////////////////////////////////////////////////

#pragma once
#pragma comment(lib,"vfw32.lib")


#include <VFW.h>
#include "Dib.h"

class CAvi  
{
public:
	// 생성자
	CAvi();
	~CAvi();

public:
	// 멤버 함수
	BOOL    Open(LPCTSTR lpszPathName);
	void    Close();

	// 그리기 함수
	void    DrawFrame(HDC hDC, int nFrame);
	void    DrawFrame(HDC hDC, int nFrame, int dx, int dy);
	void    DrawFrame(HDC hDC, int nFrame, int dx, int dy, int dw, int dh, DWORD dwRop = SRCCOPY);

	// 현재 프레임 캡쳐 함수
	BOOL    GetFrame(int nFrame, CDib& dib);

	// AVI 파일 정보 반환 함수
	int     GetFrameRate() { return m_nFrameRate; }
	int     GetTotalFrame() { return m_nTotalFrame; }
	int     GetHeight() { return m_nHeight; }
	int     GetWidth() { return m_nWidth; }
	int     GetBitCount() { return m_nBitCount; }
	BOOL    IsValid() { return (m_pAviFile != NULL); }; //파일 가리키냐_

protected:
	// 멤버 변수
	PAVIFILE    m_pAviFile;			// AVI 파일 인터페이스 포인터
	PAVISTREAM  m_pVideoStream;		// Video 스트림 포인터
	PGETFRAME   m_pVideoFrame;		// 프레임을 받는 포인터

    int         m_nWidth;			// 영상의 화면 사이즈
	int         m_nRWidth;
	int         m_nHeight;

	int         m_nBitCount;

	int         m_nTotalFrame;      // 전체 프레임 개수
	int         m_nFrameRate;       // 초당 프레임 비율
};

