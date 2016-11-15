#ifndef VIBESUB_H
#define VIBESUB__H
#include	"Track.h"
#include	"cv.h"  
#include	"highgui.h"   
#include	<memory>
using namespace std;
using namespace cv;


class __declspec(dllexport)  VibeSub{
public:
	static const int  defaultNbSamples;		//每个像素点的样本个数
private:
	static const int  defaultReqMatches;		//#min指数
	static const int  defaultRadius;		//Sqthere半径
	static const int  defaultSubsamplingFactor;	//子采样概率
	static const int  background;	//背景像素
	static const int  foreground;		//前景像素
	int c_xoff[9];//x的邻居点
	int c_yoff[9];//y的邻居点
	
	Size scale;
	//创建一个随机数生成器
	RNG rng;
	Mat frame;
	IplImage*pFrame;
	//tr1::shared_ptr<IplImage>pAfter, segMap;
	//tr1::shared_ptr<CvMat> pAfterMat, segMat;
	IplImage *pAfter, *segMap;
	CvMat *pAfterMat, *segMat;
	VideoCapture cap;
	Track track;
	void Initialize(CvMat* pFrameMat, RNG rng);
	void update(CvMat* pFrameMat, CvMat* segMat, RNG rng);
public:
	VibeSub(const char* filename, const char* xmlname);
	void run();
	~VibeSub();
};

#endif // !VIBESUB_H
