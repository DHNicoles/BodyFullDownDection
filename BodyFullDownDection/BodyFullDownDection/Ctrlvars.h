#ifndef CTRLVARS_H
#define CTRLVARS_H


#include	<Windows.h>
#include	"GeneralDef.h"
#include	<iostream>
#include	 <string>
#include	<opencv/highgui.h>
#include	<opencv/cvaux.h>
#include	<opencv/cxcore.h>
#include	"opencv2/core/core.hpp"
#include	"opencv2/imgproc/imgproc.hpp"
#include	"opencv2/video/background_segm.hpp"
#include	"opencv2/objdetect/objdetect.hpp"
#include	"opencv2/highgui/highgui.hpp"
/*****************************************/
#define MaxThread 10

/*****************************************/
//camrea�̵߳Ĳ����ṹ
class HIKPlay;
typedef struct
{
	HIKPlay* HPptr;
	HWND hHwnd;
	int index;
	int procType;
	std::string type;	//����ͷ����
	std::string devicename; //����ͷ����
	LONG lPlayHandle;
	LONG lUserID;
}threadParam;
/*****************************************/
//file�̵߳Ĳ����ṹ
typedef struct
{
	HIKPlay* HPptr;
	cv::VideoCapture* pcap;
	HANDLE* h;
}fileParam;
/*****************************************/

extern cv::Size recvSize;
extern int frameCnt;
extern bool InitFlag;
extern cv::Mat preFrame, mask;
extern volatile bool shootFlag[MaxThread];
/*****************************************/
//initial all
void initall();
/*
//config �ļ�·��
*/

#endif // !CTRLVARS_H
