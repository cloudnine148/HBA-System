// videoDlg.cpp : 구현 파일

#include "stdafx.h"
#include "afxdialogex.h"

#include <opencv2\opencv.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>

#include "video.h"
#include "videoDlg.h"
#include <io.h>
#include <windows.h>
#include "DibEnhancement.h"
#include "DibColor.h"
#include "DibCv.h"
#include "GCD.h"
#include "common.h"



#pragma warning(disable:4996)

//#ifdef _DEBUG
//#define new DEBUG_NEW
//#endif

#define D_MAX_ARRAY_SIZE 1000

//// ROI
//#define ROI_MIN_RATIO 3
//#define ROI_MAX_RATIO 8

using namespace std;


// 응용 프로그램 정보에 사용되는 CAboutDlg 대화 상자입니다.

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// 대화 상자 데이터입니다.
	enum { IDD = IDD_ABOUTBOX };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 지원입니다.

// 구현입니다.
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(CAboutDlg::IDD)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()


// CvideoDlg 대화 상자



CvideoDlg::CvideoDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(CvideoDlg::IDD, pParent)
	, eb_curFrame(_T(""))
	, eb_totalFrame(_T(""))
	, eb_processtime(_T(""))
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CvideoDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_LIST1, m_lb_fname);
	DDX_Text(pDX, IDC_EB_CF, eb_curFrame);
	DDX_Text(pDX, IDC_EB_TF, eb_totalFrame);
	DDX_Text(pDX, IDC_EB_PT, eb_processtime);
}

BEGIN_MESSAGE_MAP(CvideoDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_WM_TIMER()
	ON_BN_CLICKED(IDC_LOAD, &CvideoDlg::OnBnClickedLoad)
	ON_BN_CLICKED(IDC_PLAY, &CvideoDlg::OnBnClickedPlay)
	ON_BN_CLICKED(IDC_STOP, &CvideoDlg::OnBnClickedStop)
	ON_BN_CLICKED(IDC_PAUSE, &CvideoDlg::OnBnClickedPause)
	ON_BN_CLICKED(IDC_NEXT, &CvideoDlg::OnBnClickedNext)
	ON_BN_CLICKED(IDC_PREV, &CvideoDlg::OnBnClickedPrev)
	ON_BN_CLICKED(IDC_OPENDIR, &CvideoDlg::OnBnClickedOpendir)
	ON_LBN_SELCHANGE(IDC_LIST1, &CvideoDlg::OnLbnSelchangeList1)
	ON_BN_CLICKED(IDC_CHECK1, &CvideoDlg::OnBnClickedCheck1)
	ON_BN_CLICKED(IDC_SAVEDIR, &CvideoDlg::OnBnClickedSavedir)
	ON_BN_CLICKED(IDC_TEST_BTN, &CvideoDlg::OnBnClickedTestBtn)
	ON_BN_CLICKED(BTN_LOAD_DATA, &CvideoDlg::OnBnClickedLoadData)
	ON_WM_DESTROY()
END_MESSAGE_MAP()


// CvideoDlg 메시지 처리기

BOOL CvideoDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// 시스템 메뉴에 "정보..." 메뉴 항목을 추가합니다.

	// IDM_ABOUTBOX는 시스템 명령 범위에 있어야 합니다.
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// 이 대화 상자의 아이콘을 설정합니다. 응용 프로그램의 주 창이 대화 상자가 아닐 경우에는
	//  프레임워크가 이 작업을 자동으로 수행합니다.
	SetIcon(m_hIcon, TRUE);			// 큰 아이콘을 설정합니다.
	SetIcon(m_hIcon, FALSE);		// 작은 아이콘을 설정합니다.

	// TODO: 여기에 추가 초기화 작업을 추가합니다.
	
	QueryPerformanceFrequency(&Frequency);//시스템 시작 시간.
	count = 0; // 영상 저장 번호 초기화

	m_bPlay = FALSE; // 재생중인지
	m_load = FALSE; // 로드버튼이 눌렸는지
	ischecked = FALSE; //SAVE가 체크 되었는지
	

	//////////////////////////////////
	//Picture Control 크기 받아오기 // 
	//////////////////////////////////
	CStatic* lo_staticSize = (CStatic*)GetDlgItem(IDC_LOCAL);
	CStatic* gl_staticSize = (CStatic*)GetDlgItem(IDC_GLOBAL);
	CStatic* or_staticSize = (CStatic*)GetDlgItem(IDC_ORIGIN);
	CStatic* dt_staticSize = (CStatic*)GetDlgItem(IDC_DETECTION);

	CRect lo_rect,gl_rect,or_rect, dt_rect, bc_rect;

	lo_staticSize->GetClientRect(lo_rect);
	gl_staticSize->GetClientRect(gl_rect);
	or_staticSize->GetClientRect(or_rect);
	dt_staticSize->GetClientRect(dt_rect);

	lo_width = lo_rect.Width();
	lo_height = lo_rect.Height();
	gl_width = gl_rect.Width();
	gl_height = gl_rect.Height();
	or_width = or_rect.Width();
	or_height = or_rect.Height(); 
	dt_width = dt_rect.Width();
	dt_height = dt_rect.Height();



	lo_staticSize->GetWindowRect(lo_rect);
	gl_staticSize->GetWindowRect(gl_rect);
	or_staticSize->GetWindowRect(or_rect);
	dt_staticSize->GetWindowRect(dt_rect);

	ScreenToClient(lo_rect);
	ScreenToClient(gl_rect);
	ScreenToClient(or_rect);
	ScreenToClient(dt_rect);

	lo_left = lo_rect.left;
	lo_top = lo_rect.top;

	gl_left = gl_rect.left;
	gl_top = gl_rect.top;

	or_left = or_rect.left;
	or_top = or_rect.top;

	dt_left = dt_rect.left;
	dt_top = dt_rect.top;	

	m_bTreefileloading = false;

	return TRUE;  // 포커스를 컨트롤에 설정하지 않으면 TRUE를 반환합니다.
}

void CvideoDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}

// 대화 상자에 최소화 단추를 추가할 경우 아이콘을 그리려면
//  아래 코드가 필요합니다. 문서/뷰 모델을 사용하는 MFC 응용 프로그램의 경우에는
//  프레임워크에서 이 작업을 자동으로 수행합니다.

void CvideoDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // 그리기를 위한 디바이스 컨텍스트입니다.

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// 클라이언트 사각형에서 아이콘을 가운데에 맞춥니다.
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// 아이콘을 그립니다.
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

// 사용자가 최소화된 창을 끄는 동안에 커서가 표시되도록 시스템에서
//  이 함수를 호출합니다.
HCURSOR CvideoDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}



void CvideoDlg::OnTimer(UINT_PTR nIDEvent) // 타이머. 계속해서 불린다.
{
	CString save; // 저장할 파일 이름.
	if(nIDEvent ==1 && !m_load)
	{
		m_nCurrentFrame++;
		if(m_nCurrentFrame >= m_Avi.GetTotalFrame()) // 영상이 끝나면
		{
			m_nCurrentFrame = m_Avi.GetTotalFrame() - 1;
			KillTimer(1);
			m_bPlay = FALSE;
		}
		else
		{			
			m_Avi.GetFrame(m_nCurrentFrame, m_Dib);

			processImage();
			drawImages();

			if(ischecked)
			{
				if(save_path=="") // 경로 지정을 안했으면 C:\\test에 저장.
				{
					::CreateDirectory(_T("C:\\test"),NULL);
					save_path = "C:\\test";
					AfxMessageBox(_T("경로설정을 하지 않았습니다. \n기본 경로인 C:\\test에 저장이 됩니다."));
				}

				save.Format(_T("%d"),++count);//몇 번째인지
				int len=1;//현재 저장되는 bmp의 자리수
				len = save.GetLength();
				CString temp_num_key = num_key.Left(num_key.GetLength()-len);
				m_Dib.Save(save_path+"\\"+temp_num_key+save+".bmp");
			}

			//현재 몇 번째 프레임인지.
			CString temp; 
			temp.Format(_T("%d"),m_nCurrentFrame+1);
			eb_curFrame = temp;
		}
	}

	else if(nIDEvent ==1 && m_load) // 로드버튼이 눌렸으면
	{
		cur_bmp++;
		if(cur_bmp == count_bmp) // 영상이 끝나면
		{
			cur_bmp = count_bmp-1;
			KillTimer(1);
			m_bPlay = FALSE;
		}

		else
		{
			CString fir;
			fir = load_path;
			fir += "\\";
			fir += bmpsaved[cur_bmp].c_str();

			m_Dib.Load(fir);
			
			processImage();
			drawImages();

			CString temp;
			temp.Format(_T("%d"),cur_bmp+1);
			eb_curFrame = temp;
		}
	}
	UpdateData(FALSE); 
	Invalidate(FALSE);
	CDialogEx::OnTimer(nIDEvent);
}

int GetDirFilesNum(CString dirName) // 파일선택시 내부에 bmp파일 갯수를 세는 함수
{
        int count = 1;
        CFileFind finder;
       
        BOOL bWorking = finder.FindFile(dirName + "/*.bmp");
 
        while(bWorking)
        {
                bWorking = finder.FindNextFile();
                if(finder.IsDots())
                {
                        continue;
                }
               
                count++;
 
        }
        finder.Close();
 
        return count;
}


void CvideoDlg::OnBnClickedLoad() // 파일 로드. 폴더내의 이미지파일만 읽어오기.
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.	
	if(m_bPlay) // 재생중인데 load버튼 누르면 화면을 멈추어 주자.
	{
		m_bPlay = FALSE;
		KillTimer(1);
	}

	TCHAR cpath[MAX_PATH] = _T("");
	LPITEMIDLIST pDirList;
	BROWSEINFO browseInfo;
	browseInfo.hwndOwner = NULL;
	browseInfo.pidlRoot = NULL;
	browseInfo.lpszTitle = _T("이미지를 불러올 폴더를 선택해 주세요.");
	browseInfo.pszDisplayName = cpath;
	browseInfo.ulFlags = BIF_RETURNONLYFSDIRS;
	browseInfo.lpfn = NULL;
	browseInfo.lParam = 0;
	pDirList = SHBrowseForFolder(&browseInfo);
	if(pDirList == NULL) return;
	SHGetPathFromIDList(pDirList, cpath);

	TCHAR *return_path = cpath;

	load_path = return_path;

	count_bmp = GetDirFilesNum(load_path); // 폴더 내의 bmp파일이 몇개인가!
	
	if(count_bmp == 1)
	{
		AfxMessageBox(_T("bmp파일이 없습니다."));
		return ;
	}

	cur_bmp = 0; //현재 몇번째 bmp인가

	if(count_bmp >0){ //bmp파일이 있으면
	bmpsaved = new string[count_bmp];
	memset(bmpsaved,NULL,count_bmp-1); // 파일 갯수만큼 초기화
	}

	struct _finddata_t c_file;
	intptr_t hFile = _findfirst(load_path+"\\*.bmp", &c_file);
	if(hFile==-1) return;      // no files
	do
	{
		if(c_file.attrib & _A_SUBDIR)
		{
				// directory 처리
		}
		else if(c_file.attrib & _A_HIDDEN)
		{
				// hidden file 처리
		}
		else
		{
				// 일반 파일 처리
			char* temp;
			temp = c_file.name;
			
			bmpsaved[cur_bmp] = temp;
			cur_bmp++;
		}
	} while(_findnext(hFile, &c_file)==0);
	_findclose(hFile);

	CString fir;
	fir = load_path;
	fir += "\\";
	fir += bmpsaved[0].c_str();

	m_Dib.Load(fir);

	processImage();
	drawImages();

	//전체 bmp가 몇개인지
	CString temp;
	temp.Format(_T("%d"),count_bmp);
	eb_totalFrame = temp;
	eb_curFrame = "1";
	UpdateData(FALSE);
	
	m_load = TRUE; // 로드버튼이 눌림.
	cur_bmp = 0;
}


void CvideoDlg::OnBnClickedPlay() // 재생
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.

	if( !m_bTreefileloading )
	{
		OnBnClickedLoadData();
		TRACE(_T("Loading the tree file...\n"));
	}

	if(m_Avi.IsValid() && !m_load) // 로드버튼 안눌렸을 때
	{		 
		if(!m_bPlay) // 재생중이 아닐 때
		{
			SetTimer(1, 1000/m_Avi.GetFrameRate(), NULL);
			m_bPlay = TRUE;
		}
		else// 재생중일 때
		{
			m_bPlay = FALSE;
			KillTimer(1);
		}
	}
	else if(m_load) // 로드버튼 눌렀을 때
	{
		if(!m_bPlay) // 재생중이 아닐 때
		{
			SetTimer(1,70,NULL);
			m_bPlay = TRUE;
		}
		else
		{
			m_bPlay = FALSE;
			KillTimer(1);
		}
	}
}


void CvideoDlg::OnBnClickedStop() // 정지
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if(m_Avi.IsValid()&&!m_load){ // 로드중이 아닐때
		count =0;
		m_bPlay = FALSE;
		KillTimer(1);
		m_nCurrentFrame = 0;
		m_Avi.GetFrame(m_nCurrentFrame,m_Dib); //영상의 첫 번째 프레임을 표시
		
		m_MAIN.InitValues(m_Dib.GetHeight(), m_Dib.GetWidth());

		processImage();
		drawImages();
		
		Invalidate(FALSE);

		eb_curFrame = "1";
		UpdateData(FALSE); 
	}

	else if(m_load)// 로드버튼 눌림
	{
		m_bPlay = FALSE;
		KillTimer(1);
		cur_bmp = 0;

		CString fir;
		fir = load_path;
		fir += "\\";
		fir += bmpsaved[cur_bmp].c_str(); // 첫 화면 경로

		m_Dib.Load(fir);

		processImage();
		drawImages();

		Invalidate(FALSE);

		eb_curFrame = "1";
		UpdateData(FALSE); 
	}
}


void CvideoDlg::OnBnClickedPause() // 일시정지
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	m_bPlay = FALSE;
	KillTimer(1);
}


void CvideoDlg::OnBnClickedNext() // 한 프레임 뒤로
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if(m_Avi.IsValid() && !m_load){
		if(m_nCurrentFrame < m_Avi.GetTotalFrame() -1)
		{
			m_nCurrentFrame++;
			m_Avi.GetFrame(m_nCurrentFrame,m_Dib);

			processImage();
			drawImages();

			CString temp;
			temp.Format(_T("%d"),m_nCurrentFrame+1);
			eb_curFrame = temp;
			UpdateData(FALSE); 
		}
	}
	else
	{
		if(cur_bmp < count_bmp-1)
		{
			cur_bmp++;
			CString fir;
			fir = load_path;
			fir += "\\";
			fir += bmpsaved[cur_bmp].c_str(); // 한 프레임 뒤로

			m_Dib.Load(fir);

			processImage();
			drawImages();

			CString temp;
			temp.Format(_T("%d"),cur_bmp+1);
			eb_curFrame = temp;
			UpdateData(FALSE); 
		}
	}
	Invalidate(FALSE);
}


void CvideoDlg::OnBnClickedPrev() // 한 프레임 앞으로
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if(m_Avi.IsValid() && !m_load)
	{
		if(m_nCurrentFrame > 0)
		{
			m_nCurrentFrame--;
			m_Avi.GetFrame(m_nCurrentFrame,m_Dib);

			processImage();
			drawImages();

			CString temp;
			temp.Format(_T("%d"),m_nCurrentFrame+1);
			eb_curFrame = temp;
			UpdateData(FALSE); 
		}
	}
	else
	{
		if(cur_bmp > 0)
		{
			cur_bmp--;
			CString fir;
			fir = load_path;
			fir += "\\";
			fir += bmpsaved[cur_bmp].c_str(); // 한 프레임 뒤로

			m_Dib.Load(fir);

			processImage();
			drawImages();

			CString temp;
			temp.Format(_T("%d"),cur_bmp+1);
			eb_curFrame = temp;
			UpdateData(FALSE); 
		}
	}
	Invalidate(FALSE);
}


void CvideoDlg::OnBnClickedOpendir() // 폴더내에 .avi 전체 열기
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if(m_bPlay)
	{
		m_bPlay = FALSE;
		KillTimer(1);
	}
	TCHAR cpath[MAX_PATH] = _T("");
	LPITEMIDLIST pDirList;
	BROWSEINFO browseInfo;
	browseInfo.hwndOwner = NULL;
	browseInfo.pidlRoot = NULL;
	browseInfo.lpszTitle = _T("Avi를 불러올 폴더를 선택해 주세요.");
	browseInfo.pszDisplayName = cpath;
	browseInfo.ulFlags = BIF_RETURNONLYFSDIRS;
	browseInfo.lpfn = NULL;
	browseInfo.lParam = 0;
	pDirList = SHBrowseForFolder(&browseInfo);
	if(pDirList == NULL) return;
	SHGetPathFromIDList(pDirList, cpath);

	TCHAR *return_path = cpath;

	if(return_path != NULL)
		m_lb_fname.ResetContent();

	CString temp;
	//CString temp_path;
	temp_path = return_path; // 폴더 경로 저장

	struct _finddata_t c_file;
	intptr_t hFile = _findfirst(temp_path+"\\*.avi", &c_file);
	if(hFile==-1) 
	{
		AfxMessageBox(_T("Avi파일이 없습니다."));
		return;      // no files
	}
	do
	{
		if(c_file.attrib & _A_SUBDIR)
		{
				// directory 처리
		}
		else if(c_file.attrib & _A_HIDDEN)
		{
				// hidden file 처리
		}
		else
		{
				// 일반 파일 처리
			temp = c_file.name;
			m_lb_fname.AddString(temp);
		}
	} while(_findnext(hFile, &c_file)==0);
	_findclose(hFile);

}

void CvideoDlg::OnLbnSelchangeList1() // 리스트박스에서 .avi파일 선택
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	m_load = FALSE;
	count =0;
	num_key = "";
	int level = m_lb_fname.GetCurSel(); // 현재 선택된 인덱스?
	CString tempname;

	if(m_bPlay) // 재생중인데 avi 선택을 하면 화면을 멈추어 주자.
	{
		m_bPlay = FALSE;
		KillTimer(1);
	}

	if(level != -1) //선택된 것이 있다면
	{
		m_lb_fname.GetText(level,tempname);
		m_nCurrentFrame = 0;
		m_bPlay = FALSE;

		m_Avi.Open(temp_path+"\\"+tempname); // 해당경로 avi파일 오픈.
		m_Avi.GetFrame(m_nCurrentFrame, m_Dib);
	
		m_MAIN.InitValues(m_Dib.GetHeight(), m_Dib.GetWidth());

		drawImage(m_Dib);

		CString temp;
		temp.Format(_T("%d"),m_Avi.GetTotalFrame());
		eb_totalFrame = temp;
		eb_curFrame = "1";
		num_length = temp.GetLength(); // 총 프레임의 길이. 몇자리?
		for(int i=0; i< num_length; i++)
			num_key += "0"; // 자리만큼 0붙여주기		

		UpdateData(FALSE);
	}
}


void CvideoDlg::OnBnClickedCheck1() // 체크박스 눌렀을 때
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if(!ischecked) // 안눌려있으면
		ischecked = TRUE;

	else
		ischecked = FALSE;
}


void CvideoDlg::OnBnClickedSavedir() // 파일 저장 경로 설정
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if(m_bPlay) // 다른 작업으로 재생이 멈추면 타이머를 끝내주자.
	{
		m_bPlay = FALSE;
		KillTimer(1);
	}

	TCHAR cpath[MAX_PATH] = _T("");
	LPITEMIDLIST pDirList;
	BROWSEINFO browseInfo;
	browseInfo.hwndOwner = NULL;
	browseInfo.pidlRoot = NULL;
	browseInfo.lpszTitle = _T("이미지를 저장 할 폴더를 선택해 주세요.");
	browseInfo.pszDisplayName = cpath;
	//browseInfo.ulFlags = BIF_RETURNONLYFSDIRS;
	browseInfo.ulFlags = BIF_RETURNONLYFSDIRS|BIF_NEWDIALOGSTYLE ;
	browseInfo.lpfn = NULL;
	browseInfo.lParam = 0;
	pDirList = SHBrowseForFolder(&browseInfo);
	if(pDirList == NULL) return;
	SHGetPathFromIDList(pDirList, cpath);

	TCHAR *return_path = cpath;

	save_path = return_path;
}


void CvideoDlg::processImage()
{
	LARGE_INTEGER BeginTime;
	LARGE_INTEGER Endtime;
	CDib dib = m_Dib;

	QueryPerformanceCounter(&BeginTime); // 시간 측정 시작
	
	m_MAIN.mainProcessing(dib, m_nCurrentFrame);

	QueryPerformanceCounter(&Endtime); // 시간 측정 끝.

	CString temp;
	//처리시간. 단위 ms
	temp.Format(_T("%lf"),((double)(Endtime.QuadPart - BeginTime.QuadPart)/(double)Frequency.QuadPart)*1000);
	eb_processtime = temp;	
}

void CvideoDlg::drawImage(CDib dib)
{
	CDC* pDC = GetDC();
	dib.Draw(pDC->m_hDC,lo_left,lo_top,lo_width,lo_height);
	ReleaseDC(pDC);
}


void CvideoDlg::gl_drawImage(CDib dib)
{
	CDC* pDC = GetDC();
	dib.Draw(pDC->m_hDC, gl_left, gl_top, gl_width, gl_height);
	ReleaseDC(pDC);
}


void CvideoDlg::or_drawImage(CDib dib)
{
	CDC* pDC = GetDC();
	dib.Draw(pDC->m_hDC, or_left, or_top, or_width, or_height);
	ReleaseDC(pDC);
}


void CvideoDlg::lo_drawImage(CDib dib)
{
	CDC* pDC = GetDC();
	dib.Draw(pDC->m_hDC, lo_left, lo_top, lo_width, lo_height);
			
	m_MAIN.Draw_BeamText(pDC);
	ReleaseDC(pDC);
}

void CvideoDlg::dt_drawImage(CDib dib)
{
	CDC* pDC = GetDC();
	dib.Draw(pDC->m_hDC, dt_left, dt_top, dt_width, dt_height);
	ReleaseDC(pDC);
}

void CvideoDlg::OriginalImage()
{
	CDib dib = m_Dib;
	or_drawImage(dib);
}

void CvideoDlg::bc_drawBMP()
{
	CPaintDC dc(this);
	CDC *pDC = GetDC();
	CDC mDC;
	BITMAP bmpinfo;
	
	mDC.CreateCompatibleDC(&dc);

	CBitmap bmp;
	CBitmap* pOldBmp = NULL;

	boolean bc_result;

	bc_result = m_MAIN.GetPairResult();

	if(bc_result)
		bmp.LoadBitmapW(IDB_HIGHBEAM);

	else
		bmp.LoadBitmapW(IDB_LOWBEAM);
	
	bmp.GetBitmap(&bmpinfo);

	pOldBmp = mDC.SelectObject(&bmp);

	pDC->TransparentBlt(510,69,bmpinfo.bmWidth/5,bmpinfo.bmHeight/5, &mDC,0,0,bmpinfo.bmWidth,bmpinfo.bmHeight, RGB(255,255,255));

	mDC.SelectObject(pOldBmp);

	ReleaseDC(pDC);
	ReleaseDC(&mDC);
}


void CvideoDlg::OnBnClickedTestBtn()
{	
	m_MAIN.TrainRF_Detection(FILE_PATH_DETECT_FEATURE_WAVELET, FILE_PATH_DETECT_RF_TREE_WAVELET, RF_NUMBER_OF_TREE_F_WIN, RF_NUMBER_OF_GOODTREE_F_WIN, RF_RATIO_OF_SAMPLEDATA_F_WIN, 2);
}


void CvideoDlg::drawTrackingImage()
{
	CDC *pDC = GetDC();

	m_trackingDib = m_Dib;

	m_MAIN.GetTrackingResult(m_trackingDib);
	m_trackingDib.Draw(pDC->m_hDC, or_left, or_top, or_width, or_height);	
	m_MAIN.Draw_trackingText(pDC);

	ReleaseDC(pDC);
}

void CvideoDlg::OnBnClickedLoadData()
{
	if( !m_bTreefileloading )
	{
		int result;
		result = m_MAIN.LoadRFTreeDataForDetection();
		if( result != SUCCESS ){
			Trace(_T("[%s, %d] Loading of the RF's tree is failed.(%d)\n"), _function_, _line_, result);
			AfxMessageBox(_T("Loading of the RF's tree is failed."));
		}

		m_bTreefileloading = true;
	}	
}

void CvideoDlg::OnDestroy()
{
	CDialogEx::OnDestroy();
	// TODO: 여기에 메시지 처리기 코드를 추가합니다.
	
}


void CvideoDlg::drawImages()
{
	CDib roiDib = m_Dib;
	CDib segDib = m_Dib;	
	CDib dtcDlb = m_Dib;
	CDib prDib = m_Dib;

	// Original img + ROI
	m_MAIN.DrawROI(roiDib);
	or_drawImage(roiDib);

	// Segmentation Result
	m_MAIN.GetSegmentationResult(segDib);
	gl_drawImage(segDib);

	// detection Result
	m_MAIN.GetDetectionResult(m_Dib, dtcDlb);
	m_MAIN.DrawROI(dtcDlb);
	dt_drawImage(dtcDlb);

	// Pairing Result
	m_MAIN.GetPairingResult(m_Dib, prDib);
	m_MAIN.DrawROI(prDib);
	lo_drawImage(prDib);

	// Draw HIGHBEAM, LOWBEAM BMP
	bc_drawBMP();

	// DIB 저장
	/*CString str_dt, str_pr;
	CString filenum_dt, filenum_pr;
	str_dt = str_pr = ".\\DATA\\";

	filenum_dt.Format(_T("%d"), m_nCurrentFrame);
	filenum_dt += "_dt.bmp";
	str_dt += filenum_dt;

	filenum_pr.Format(_T("%d"), m_nCurrentFrame);
	filenum_pr += "_pr.bmp";
	str_pr += filenum_pr;

	roiDib.Save(str_dt);
	prDib.Save(str_pr);*/
}