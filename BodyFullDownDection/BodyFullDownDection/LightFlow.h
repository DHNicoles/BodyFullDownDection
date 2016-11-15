#include <opencv2/video/video.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <vector>
using namespace std;
using namespace cv;
class LightF{
private:
	vector<Point2f> points[2];        // point0Ϊ�������ԭ��λ�ã�point1Ϊ���������λ��

	int maxCount;        // �������������

	double qLevel ;        // �������ĵȼ�

	double minDist ;        // ��������֮�����С����

	vector<uchar> status;        // ����������״̬��������������Ϊ1������Ϊ0
	
	vector<float> err;

	Mat frame, gray_prev, gray;

	VideoCapture cap;

	Size scale;
public:
	LightF(const char *);
	void run();
};