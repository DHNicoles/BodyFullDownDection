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
	static const int  defaultNbSamples;		//ÿ�����ص����������
private:
	static const int  defaultReqMatches;		//#minָ��
	static const int  defaultRadius;		//Sqthere�뾶
	static const int  defaultSubsamplingFactor;	//�Ӳ�������
	static const int  background;	//��������
	static const int  foreground;		//ǰ������
	int c_xoff[9];//x���ھӵ�
	int c_yoff[9];//y���ھӵ�
	
	Size scale;
	//����һ�������������
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
