#include <opencv2/video/video.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <vector>
using namespace std;
using namespace cv;
class LightF{
private:
	vector<Point2f> points[2];        // point0为特征点的原来位置，point1为特征点的新位置

	int maxCount;        // 检测的最大特征数

	double qLevel ;        // 特征检测的等级

	double minDist ;        // 两特征点之间的最小距离

	vector<uchar> status;        // 跟踪特征的状态，特征的流发现为1，否则为0
	
	vector<float> err;

	Mat frame, gray_prev, gray;

	VideoCapture cap;

	Size scale;
public:
	LightF(const char *);
	void run();
};