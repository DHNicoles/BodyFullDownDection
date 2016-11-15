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
	static string result[MaxThread];//相机的检测结果
	static unsigned short in[MaxThread]; //产品进缓冲区时的缓冲区下标
	static unsigned short out[MaxThread]; //产品出缓冲区时的缓冲区下标
	static HANDLE g_RetMutex[MaxThread]; //用于线程间的互斥
	static HANDLE g_hMutex[MaxThread]; //用于线程间的互斥
	static HANDLE g_hFullSemaphore[MaxThread]; //当缓冲区满时迫使生产者等待
	static HANDLE g_hEmptySemaphore[MaxThread]; //当缓冲区空时迫使消费者等待
	static HANDLE hand[MaxThread];//线程的句柄
	static unsigned short SIZE_OF_BUFFER; //缓冲区长度
	static bool g_continue[MaxThread]; //控制程序结束
	static int hik_width_org[MaxThread];
	static int hik_height_org[MaxThread];
	static char * imagesData[MaxThread][100];
	static IplImage* images[MaxThread][100];//缓冲区是个循环队列
	static long lport[MaxThread];
	static long Port[MaxThread]; // 全局的播放库port号
	static vector<bool> IDpool;
	static int ValidIndex;
	int ID;
public:

	HIKPlay(string IPstr, string username, string password, long port,string confpath);
	
	//启动相机
	bool start();
	//停止相机
	bool stop();
	//相机检测结果：-1表示停止，0表示未发现人员，1表示发现人员，正在跟踪，2表示发现人员,且可能倒地
	string getLastResult();

	
private:
	//初始化指定
	bool StartPlay(const LOCAL_DEVICE_INFO &m_struDeviceInfo);

	void DoGetDeviceResoureCfg(LOCAL_DEVICE_INFO &m_struDeviceInfo);

	bool DoLogin(string IPstr, string userID, string psw, LOCAL_DEVICE_INFO &m_struDeviceInfo);

	//static void CALLBACK g_RealDataCallBack_V30(LONG lRealHandle, DWORD dwDataType, BYTE *pBuffer, DWORD dwBufSize, void* dwUser);

	// void CALLBACK DecCBFun(long nPort, char * pBuf, long nSize, FRAME_INFO * pFrameInfo, long nReserved1, long /*nReserved2*/); //YV12(YUV的一种)-->IplImag
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

	static void CALLBACK DecCBFun_0(long nPort, char * pBuf, long nSize, FRAME_INFO * pFrameInfo, long nReserved1, long /*nReserved2*/); //YV12(YUV的一种)-->IplImag
	static void CALLBACK DecCBFun_1(long nPort, char * pBuf, long nSize, FRAME_INFO * pFrameInfo, long nReserved1, long /*nReserved2*/); //YV12(YUV的一种)-->IplImag
	static void CALLBACK DecCBFun_2(long nPort, char * pBuf, long nSize, FRAME_INFO * pFrameInfo, long nReserved1, long /*nReserved2*/); //YV12(YUV的一种)-->IplImag
	static void CALLBACK DecCBFun_3(long nPort, char * pBuf, long nSize, FRAME_INFO * pFrameInfo, long nReserved1, long /*nReserved2*/); //YV12(YUV的一种)-->IplImag
	static void CALLBACK DecCBFun_4(long nPort, char * pBuf, long nSize, FRAME_INFO * pFrameInfo, long nReserved1, long /*nReserved2*/); //YV12(YUV的一种)-->IplImag
	static void CALLBACK DecCBFun_5(long nPort, char * pBuf, long nSize, FRAME_INFO * pFrameInfo, long nReserved1, long /*nReserved2*/); //YV12(YUV的一种)-->IplImag
	static void CALLBACK DecCBFun_6(long nPort, char * pBuf, long nSize, FRAME_INFO * pFrameInfo, long nReserved1, long /*nReserved2*/); //YV12(YUV的一种)-->IplImag
	static void CALLBACK DecCBFun_7(long nPort, char * pBuf, long nSize, FRAME_INFO * pFrameInfo, long nReserved1, long /*nReserved2*/); //YV12(YUV的一种)-->IplImag
	static void CALLBACK DecCBFun_8(long nPort, char * pBuf, long nSize, FRAME_INFO * pFrameInfo, long nReserved1, long /*nReserved2*/); //YV12(YUV的一种)-->IplImag
	static void CALLBACK DecCBFun_9(long nPort, char * pBuf, long nSize, FRAME_INFO * pFrameInfo, long nReserved1, long /*nReserved2*/); //YV12(YUV的一种)-->IplImag
private://read from file
	VideoCapture cap;
	string VideoPath;
	Track track;
public:
	string filepath;
	HIKPlay(const char* path, string confpath);
public:
	static const int  defaultNbSamples;		//每个像素点的样本个数
private:
	static const int  defaultReqMatches;		//#min指数
	static const int  defaultRadius;		//Sqthere半径
	static const int  defaultSubsamplingFactor;	//子采样概率
	static const int  background;	//背景像素
	static const int  foreground;		//前景像素
	static int c_xoff[9];//x的邻居点
	static int c_yoff[9];//y的邻居点
	static Size scale;
	//创建一个随机数生成器
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