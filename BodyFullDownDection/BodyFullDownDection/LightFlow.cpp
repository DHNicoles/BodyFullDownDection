#include	"LightFlow.h"

LightF::LightF(const char *path)
:maxCount(100), qLevel(0.01), minDist(1.0), cap(path), scale(600, 300)
{
	
}
void LightF::run(){
	if (!cap.isOpened())return;
	cap >> frame;
	while (!frame.empty()){
		resize(frame, frame, scale);
		cvtColor(frame, gray, CV_BGR2GRAY);
		if (gray_prev.empty()){
			gray.copyTo(gray_prev);
		}
		else{
			goodFeaturesToTrack(gray_prev, points[0], maxCount, qLevel, minDist);
			// l-k光流法运动估计
			calcOpticalFlowPyrLK(gray_prev, gray, points[0], points[1], status, err);
			int k = 0;
			for (size_t i = 0; i < points[1].size(); i++)
			{

				if (status[i])
				{
					points[0][k] = points[0][i];
					points[1][k++] = points[1][i];

				}
			}
			points[1].resize(k);//point1保存着一些流动的点
			for (size_t i = 0; i < points[1].size(); i++)
			{
				line(frame, points[0][i], points[1][i], Scalar(0, 0, 255), 1);
				circle(frame, points[1][i], 1, Scalar(255, 0, 0), 3);
			}
			imshow("frame", frame);
			waitKey(10);
			swap(gray_prev, gray);
			swap(points[0], points[1]);
		}
		cap >> frame;
	}
}