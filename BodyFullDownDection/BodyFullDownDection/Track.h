#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/ml/ml.hpp> 
#include	"cv.h"  
#include	"Global.h"
#include	"Ctrlvars.h"
#include	"highgui.h" 
#include	<vector>
#include	<memory>
#include	<set>
#include	"BDLOG.h"
#include	<direct.h>
#include	 "time.h"
using namespace cv;

class Track{

private:
	CascadeClassifier  cascade;
	std::vector<Rect> StandBox;
	std::vector<vector<Point> > contours;
	Mat pre_Mask, preFrameGray, CurGray;
	Mat ROI_preFrameMask, ROI_CurMask;
	Rect CornersRoi;
	vector<Point2f> points[2];        // point0为特征点的原来位置，point1为特征点的新位置

	int maxCount;        // 检测的最大特征数

	double qLevel;        // 特征检测的等级

	double minDist;        // 两特征点之间的最小距离

	vector<uchar> status;        // 跟踪特征的状态，特征的流发现为1，否则为0

	vector<float> err;

	vector<int> available;

	bool TriggerFlag;

	int CountStatic;

	int frameCnt;
	
	const int MaxStatic, MaxAvailabelCorners, MinUpdateCorner;

	time_t timeBegin;

	string homepath;
	string PersonnelState;
	Rect prePosition;
	int FrameBegin;
	Point PossiblePt;
	bool FallFlag;
protected:
	string getStateAndSaveImg(Mat& src);
public:
	Track();
	void clear();
	void initialXml(const char* path);
	bool standDetect(Mat& roi, Rect& position);
	string proc(Mat& src, IplImage*mask,int ID);
};