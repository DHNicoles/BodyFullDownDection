#ifndef CAMERACTRL_H
#define CAMERACTRL_H

#include <Windows.h>
#include "GeneralDef.h"

#include	<string>
#include	<opencv/highgui.h>
#include	<opencv/cvaux.h>
#include	<opencv/cxcore.h>
#include	"opencv2/core/core.hpp"
#include	"opencv2/imgproc/imgproc.hpp"
#include	"opencv2/video/background_segm.hpp"
#include	"opencv2/objdetect/objdetect.hpp"
#include	"opencv2/highgui/highgui.hpp"
#include	"Track.h"
#include	"cv.h"  
#include	"highgui.h"   
#include	<memory>

using namespace std;
using namespace cv;


class __declspec(dllexport) HIKPlay{
private://read from camera
	bool IsVideo;
	string IPstr, username, password;
	long port;
	LOCAL_DEVICE_INFO m_struDeviceInfo;
	LONG m_lPlayHandle;
	static string result[MaxThread];//����ļ����
	static unsigned short in[MaxThread]; //��Ʒ��������ʱ�Ļ������±�
	static unsigned short out[MaxThread]; //��Ʒ��������ʱ�Ļ������±�
	static HANDLE g_RetMutex[MaxThread]; //�����̼߳�Ļ���
	static HANDLE g_hMutex[MaxThread]; //�����̼߳�Ļ���
	static HANDLE g_hFullSemaphore[MaxThread]; //����������ʱ��ʹ�����ߵȴ�
	static HANDLE g_hEmptySemaphore[MaxThread]; //����������ʱ��ʹ�����ߵȴ�
	static HANDLE hand[MaxThread];//�̵߳ľ��
	static unsigned short SIZE_OF_BUFFER; //����������
	static bool g_continue[MaxThread]; //���Ƴ������
	static int hik_width_org[MaxThread];
	static int hik_height_org[MaxThread];
	static char * imagesData[MaxThread][100];
	static IplImage* images[MaxThread][100];//�������Ǹ�ѭ������
	static long lport[MaxThread];
	static long Port[MaxThread]; // ȫ�ֵĲ��ſ�port��
	static vector<bool> IDpool;
	static int ValidIndex;
	int ID;
public:

	HIKPlay(string IPstr, string username, string password, long port,string confpath);
	
	//�������
	bool start();
	//ֹͣ���
	bool stop();
	//����������-1��ʾֹͣ��0��ʾδ������Ա��1��ʾ������Ա�����ڸ��٣�2��ʾ������Ա,�ҿ��ܵ���
	string getLastResult();

	
private:
	//��ʼ��ָ��
	bool StartPlay(const LOCAL_DEVICE_INFO &m_struDeviceInfo);

	void DoGetDeviceResoureCfg(LOCAL_DEVICE_INFO &m_struDeviceInfo);

	bool DoLogin(string IPstr, string userID, string psw, LOCAL_DEVICE_INFO &m_struDeviceInfo);

	//static void CALLBACK g_RealDataCallBack_V30(LONG lRealHandle, DWORD dwDataType, BYTE *pBuffer, DWORD dwBufSize, void* dwUser);

	// void CALLBACK DecCBFun(long nPort, char * pBuf, long nSize, FRAME_INFO * pFrameInfo, long nReserved1, long /*nReserved2*/); //YV12(YUV��һ��)-->IplImag
//image process
	static unsigned int __stdcall playThreadProc(LPVOID lpParameter);//camera

	static unsigned int __stdcall fileThreadProc(LPVOID lpParameter);//video

	static bool YV12_To_BGR24(unsigned char* pYV12, unsigned char* pRGB24, int width, int height);

	static IplImage* YV12_To_IplImage(IplImage* pImage, unsigned char* pYV12, int width, int height);

	void Thread_init();
	static void(CALLBACK *g_RealDataCallBack_V30[MaxThread])(LONG lRealHandle, DWORD dwDataType, BYTE *pBuffer, DWORD dwBufSize, void* dwUser);
	static void(CALLBACK *DecCBFun[MaxThread])(long, char *, long, FRAME_INFO *, long, long /*nReserved2*/);
	//
	static void CALLBACK g_RealDataCallBack_V30_0(LONG lRealHandle, DWORD dwDataType, BYTE *pBuffer, DWORD dwBufSize, void* dwUser);
	static void CALLBACK g_RealDataCallBack_V30_1(LONG lRealHandle, DWORD dwDataType, BYTE *pBuffer, DWORD dwBufSize, void* dwUser);
	static void CALLBACK g_RealDataCallBack_V30_2(LONG lRealHandle, DWORD dwDataType, BYTE *pBuffer, DWORD dwBufSize, void* dwUser);
	static void CALLBACK g_RealDataCallBack_V30_3(LONG lRealHandle, DWORD dwDataType, BYTE *pBuffer, DWORD dwBufSize, void* dwUser);
	static void CALLBACK g_RealDataCallBack_V30_4(LONG lRealHandle, DWORD dwDataType, BYTE *pBuffer, DWORD dwBufSize, void* dwUser);
	static void CALLBACK g_RealDataCallBack_V30_5(LONG lRealHandle, DWORD dwDataType, BYTE *pBuffer, DWORD dwBufSize, void* dwUser);
	static void CALLBACK g_RealDataCallBack_V30_6(LONG lRealHandle, DWORD dwDataType, BYTE *pBuffer, DWORD dwBufSize, void* dwUser);
	static void CALLBACK g_RealDataCallBack_V30_7(LONG lRealHandle, DWORD dwDataType, BYTE *pBuffer, DWORD dwBufSize, void* dwUser);
	static void CALLBACK g_RealDataCallBack_V30_8(LONG lRealHandle, DWORD dwDataType, BYTE *pBuffer, DWORD dwBufSize, void* dwUser);
	static void CALLBACK g_RealDataCallBack_V30_9(LONG lRealHandle, DWORD dwDataType, BYTE *pBuffer, DWORD dwBufSize, void* dwUser);

	static void CALLBACK DecCBFun_0(long nPort, char * pBuf, long nSize, FRAME_INFO * pFrameInfo, long nReserved1, long /*nReserved2*/); //YV12(YUV��һ��)-->IplImag
	static void CALLBACK DecCBFun_1(long nPort, char * pBuf, long nSize, FRAME_INFO * pFrameInfo, long nReserved1, long /*nReserved2*/); //YV12(YUV��һ��)-->IplImag
	static void CALLBACK DecCBFun_2(long nPort, char * pBuf, long nSize, FRAME_INFO * pFrameInfo, long nReserved1, long /*nReserved2*/); //YV12(YUV��һ��)-->IplImag
	static void CALLBACK DecCBFun_3(long nPort, char * pBuf, long nSize, FRAME_INFO * pFrameInfo, long nReserved1, long /*nReserved2*/); //YV12(YUV��һ��)-->IplImag
	static void CALLBACK DecCBFun_4(long nPort, char * pBuf, long nSize, FRAME_INFO * pFrameInfo, long nReserved1, long /*nReserved2*/); //YV12(YUV��һ��)-->IplImag
	static void CALLBACK DecCBFun_5(long nPort, char * pBuf, long nSize, FRAME_INFO * pFrameInfo, long nReserved1, long /*nReserved2*/); //YV12(YUV��һ��)-->IplImag
	static void CALLBACK DecCBFun_6(long nPort, char * pBuf, long nSize, FRAME_INFO * pFrameInfo, long nReserved1, long /*nReserved2*/); //YV12(YUV��һ��)-->IplImag
	static void CALLBACK DecCBFun_7(long nPort, char * pBuf, long nSize, FRAME_INFO * pFrameInfo, long nReserved1, long /*nReserved2*/); //YV12(YUV��һ��)-->IplImag
	static void CALLBACK DecCBFun_8(long nPort, char * pBuf, long nSize, FRAME_INFO * pFrameInfo, long nReserved1, long /*nReserved2*/); //YV12(YUV��һ��)-->IplImag
	static void CALLBACK DecCBFun_9(long nPort, char * pBuf, long nSize, FRAME_INFO * pFrameInfo, long nReserved1, long /*nReserved2*/); //YV12(YUV��һ��)-->IplImag
private://read from file
	VideoCapture cap;
	string VideoPath;
	Track track;
public:
	string filepath;
	HIKPlay(const char* path, string confpath);
public:
	static const int  defaultNbSamples;		//ÿ�����ص����������
private:
	static const int  defaultReqMatches;		//#minָ��
	static const int  defaultRadius;		//Sqthere�뾶
	static const int  defaultSubsamplingFactor;	//�Ӳ�������
	static const int  background;	//��������
	static const int  foreground;		//ǰ������
	static int c_xoff[9];//x���ھӵ�
	static int c_yoff[9];//y���ھӵ�
	static Size scale;
	//����һ�������������
	RNG rng;
	Mat frame;
	IplImage* pFrame;
	IplImage* pAfter;
	IplImage* segMap;
	CvMat *   pAfterMat;
	CvMat *   segMat;
	
	void Initialize(CvMat* pFrameMat, RNG rng);
	void update(CvMat* pFrameMat, CvMat* segMat, RNG rng);

};

#endif // !CAMERACTRL_H