#include	"Track.h"
#include	<string>

using namespace std;
using namespace cv;
Track::Track() :
	maxCount(20), qLevel(0.1), minDist(10.0), TriggerFlag(false), CountStatic(0), MaxAvailabelCorners(10), MaxStatic(130)/*倒地持续最少检测帧数*/,
	MinUpdateCorner(800), FallFlag(false), CornersRoi(0, 0, 0, 0), frameCnt(0), prePosition(0, 0, 0, 0)
{
	
	//_mkdir((homepath).c_str());
}
void Track::clear(){
	TriggerFlag = false;
	FallFlag = false;
	frameCnt = CountStatic = 0;
	pre_Mask.release();
	points[0].clear(); points[1].clear();
	available.clear();
	PossiblePt = Point(0, 0);
	StandBox.clear();
	prePosition = Rect(0,0,0,0);
}
void Track::initialXml(const char* path){
	//cascade
	cascade.load(path); 
}
Rect getRightOneRect(vector<Rect>& StandBox){
	if (StandBox.empty())return Rect(0, 0, 0, 0);
	if (StandBox.size() == 1)return StandBox.back();
	vector<Rect>::iterator rit = StandBox.begin();
	Rect ret(0,0,0,0);
	for (; rit != StandBox.end();++rit){
		Rect r = *rit;
		ret = ret.area() < rit->area() ? *rit : ret;		
	}
	return ret;
}
bool Track::standDetect(Mat& roi, Rect& position){
	Mat gray_Roi;
	cvtColor(roi, gray_Roi, CV_BGR2GRAY);
	cascade.detectMultiScale(roi, StandBox, 1.45, 4, 0, Size(16, 16));
	Rect temp(prePosition);
	if (!prePosition.width && !StandBox.empty()){
		prePosition = getRightOneRect(StandBox);//init
		FrameBegin = frameCnt;
		PossiblePt = Point(prePosition.x + prePosition.width / 2, prePosition.y + prePosition.height / 2);
	}
	else if (!StandBox.empty()){
		int dist =  (frameCnt - FrameBegin) * 2;
		vector<Rect>::iterator rit = StandBox.begin();
		int pcx = prePosition.x + prePosition.width / 2;
		int pcy = prePosition.y + prePosition.height / 2;

		Rect ret(0, 0, 0, 0);
		for (; rit!= StandBox.end();++rit){
			Rect r = *rit;
			int scx = r.x + r.width / 2;
			int scy = r.y + r.height / 2;
			//cout << "distance=" << abs(pcx - scx) + abs(pcy - scy) << "\t,距离=" << dist << endl;
			if (abs(pcx - scx) + abs(pcy - scy) < dist){
				ret = ret.area() < rit->area() ? *rit : ret;
				FrameBegin = frameCnt;	
			}
		}
		if (FrameBegin == frameCnt){
			prePosition = ret;//更新
			PossiblePt = Point(prePosition.x + prePosition.width / 2, prePosition.y + prePosition.height / 2);
			
			//cout << "prePosition = ret;//更新" << endl;
		}
	}
	//for (int i = 0; i < StandBox.size(); i++){
	//	//有发现
	//	
	//	Rect rect = StandBox[i];
	//	rectangle(roi, rect, Scalar(0, 0, 255));
	//	vector<Point2f> newcorners;
	//	goodFeaturesToTrack(gray_Roi(rect), newcorners, maxCount, qLevel, minDist);
	//	for (size_t j = 0; j != newcorners.size(); ++j){
	//		newcorners[j] = Point2f(newcorners[j].x + rect.x + position.x, newcorners[j].y + rect.y + position.y);
	//	}
	//	
	//	points[1].insert(points[1].end(), newcorners.begin(), newcorners.end());
	//	available.insert(available.end(), newcorners.size(), 0);
	//}
	
	if (FrameBegin == frameCnt && ( abs(prePosition.area() - temp.area())>10 || available.size()<1000)){//找到新的位置
		rectangle(roi, prePosition, Scalar(0, 0, 255));
		vector<Point2f> newcorners;
		goodFeaturesToTrack(gray_Roi(prePosition), newcorners, maxCount, qLevel, minDist);
		for (size_t j = 0; j != newcorners.size(); ++j){
			newcorners[j] = Point2f(newcorners[j].x + prePosition.x + position.x, newcorners[j].y + prePosition.y + position.y);
		}
		
		points[1].insert(points[1].end(), newcorners.begin(), newcorners.end());
		available.insert(available.end(), newcorners.size(), 0);
		//清除矩形框之外的光流点
		vector<Point2f>::iterator pit = points[1].begin();
		vector<int>::iterator  ait = available.begin();
		for (; pit != points[1].end();){	
			Rect rect = prePosition;
			if (pit->x >= rect.x + position.x&&pit->y >= rect.y + position.y&&pit->x <= rect.x + rect.width + position.x&&pit->y <= rect.y + rect.height + position.y|| (*ait)<-5){
				++pit, ++ait; 
			}
			else{
				pit = points[1].erase(pit);
				ait = available.erase(ait);
			}
			
		}
	}
	return FrameBegin == frameCnt;
}
string Track::proc(Mat& src, IplImage *mask,int ID){
	++frameCnt;
	Mat foreg = Mat(mask);
	//先开运算再闭运算
	erode(foreg, foreg, Mat(), Point(), 2);
	dilate(foreg, foreg, Mat(), Point(), 12);

	if (pre_Mask.empty()){
		foreg.copyTo(pre_Mask);
		cvtColor(src, preFrameGray, CV_BGR2GRAY);
		return string();
	}
	else{

		cvtColor(src, CurGray, CV_BGR2GRAY);
		// l-k光流法运动估计
		
		if (!points[0].empty())calcOpticalFlowPyrLK(preFrameGray, CurGray, points[0], points[1], status, err);
		//寻找运动很快的点
		long sum_x = 0, sum_y = 0, n = 0;
		for (size_t i = 0; i < points[0].size(); i++){
			int diff = (abs(points[0][i].x - points[1][i].x) + abs(points[0][i].y - points[1][i].y));
			if (diff){
				sum_x += diff*points[0][i].x;
				sum_y += diff*points[0][i].y;
				n += diff;
			}
		}
		if (n){//如果有运动很快的点，则计算其平均中心位置；若中心位置在边界上，则将之删除
			PossiblePt = Point(sum_x / n, sum_y / n);
			if (PossiblePt.x<10 || PossiblePt.y<10 || PossiblePt.x>(src.cols-10) || PossiblePt.y>(src.rows-10))points[1].clear();
		}
		
		//cout << "n=" << n << "+" << PossiblePt.x << "+" << PossiblePt.y << endl;
		//imshow("detect", foreg);
		/*Mat diff;
		absdiff(pre_Mask, foreg, diff);
		int sumNon = countNonZero(diff);*/
		//cout << "Change = " << sumNon << "\t，size = " << points[0].size() << "\t，CountStatic = " << CountStatic << endl;

		//alarm
		if (FallFlag){
			putText(src, "Fall Alarm!", cvPoint(CornersRoi.x, CornersRoi.y), 1, 2, Scalar(0, 0, 255), 2);
			//返回状态
			string filename = getStateAndSaveImg(src);
			PersonnelState = "2|FG|" + filename;
		}
		if (!points[1].empty()){//如果已经有跟踪目标
			CornersRoi = boundingRect(points[0]);
			CornersRoi.x = CornersRoi.x >= 0 && CornersRoi.x < src.cols ? CornersRoi.x : (CornersRoi.x < 0 ? 0 : src.cols - 1);
			CornersRoi.y = CornersRoi.y >= 0 && CornersRoi.y < src.rows ? CornersRoi.y : (CornersRoi.y < 0 ? 0 : src.rows - 1);
			CornersRoi.width = (CornersRoi.x + CornersRoi.width) < src.cols ? CornersRoi.width : (src.cols - CornersRoi.x - 1);
			CornersRoi.height = (CornersRoi.y + CornersRoi.height) < src.rows ? CornersRoi.height : (src.rows - CornersRoi.y - 1);
			ROI_preFrameMask = pre_Mask(CornersRoi);
			ROI_CurMask = foreg(CornersRoi);
			//imshow("ROI_preFrameGray", ROI_preFrameMask); 
			imshow("ROI_CurGray", ROI_CurMask);
			Mat Roi_diff;
			absdiff(ROI_preFrameMask, ROI_CurMask, Roi_diff);
			int Roi_sumNon = countNonZero(Roi_diff);
			//cout << "RoiChange = " << Roi_sumNon << "\t，size = " << points[0].size() << "\t，CountStatic = " << CountStatic << endl;;
			if (Roi_sumNon < 10){
				if (points[0].size() < 20);//焦点太少或者没有人
				else{
					//辨别倒地
					//开始计时
					if (TriggerFlag == false){
						CountStatic = 0;
						TriggerFlag = true;
						
					}
					else if (CountStatic++>MaxStatic){
						FallFlag = true;
						/*time(&timeBegin);
						string logs = ctime(&timeBegin);
						if (!logs.empty()){
							logs.resize(logs.size() - 1);
							for (int i = 0; i < logs.size(); ++i)
							if (logs[i] == ':')logs[i] = '.';
						}*/
					}
					putText(src, to_string(CountStatic), cvPoint(CornersRoi.x + CornersRoi.width/2, CornersRoi.y), 1, 1, Scalar(0, 255, 255), 2);
				}
			}
			else if (Roi_sumNon>500){//变化很快
				FallFlag=TriggerFlag = false;
				int k = 0;
				for (size_t i = 0; i < points[0].size(); i++){
					if (!status[i])continue;
					if ((abs(points[0][i].x - points[1][i].x) + abs(points[0][i].y - points[1][i].y))<1){
						if (available[i]++ >= MaxAvailabelCorners)continue;
						else{
							points[0][k] = points[0][i];
							points[1][k] = points[1][i];
							available[k++] = available[i];
						}
					}
					else{
						
						points[0][k] = points[0][i];
						points[1][k] = points[1][i];
						available[k++] = -10;
					}
				}
				
				//point1保存着一些流动的点
				points[0].resize(k); points[1].resize(k); available.resize(k);
			}
			else FallFlag = TriggerFlag = false;
		}
		else{
			
		}
		
		
		foreg.copyTo(pre_Mask);
		if (!standDetect(src, Rect(0, 0, 0, 0))){//需要删掉距离PossiblePt过远的点 >100
			int k = 0;
			for (size_t i = 0; i < points[1].size(); i++){
				if ((abs(PossiblePt.x - points[1][i].x) + abs(PossiblePt.y - points[1][i].y)) < 100)points[1][k++] = points[1][i];
			}
			points[1].resize(k);
		}
		//if (sumNon > MinUpdateCorner){//前景变化大于阈值
		//	//foreg.copyTo(pre_Mask);
		//	TriggerFlag = false;
		//	FallFlag = false;
		//	contours.clear();
		//	findContours(foreg, contours, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
		//	for (size_t i = 0; i != contours.size(); ++i){
		//		Rect r = boundingRect(contours[i]);
		//		if (r.width < 16 || r.height < 32)continue;
		//		standDetect(src(r), r);
		//	}
		//	//if (!contours.empty() && points[0].size()>3000 && frameCnt % 20 == 0){// 1/20的概率去重 
		//	//	//remove duplicate ele  
		//	//	vector<Point2f>::iterator pit = points[1].begin();
		//	//	vector<int>::iterator  ait = available.begin();
		//	//	for (; pit != points[1].end(); ++pit, ++ait){
		//	//		vector<int>::iterator  aptr = ait + 1;
		//	//		vector<Point2f>::iterator ptr = pit + 1;
		//	//		for (; ptr != points[1].end();){
		//	//			double dist = abs(ptr->x - pit->x) + abs(ptr->y - pit->y);
		//	//			if (dist < 1){
		//	//				ptr = points[1].erase(ptr);
		//	//				aptr = available.erase(aptr);
		//	//			}
		//	//			else{
		//	//				ptr++; aptr++;
		//	//			}
		//	//		}
		//	//	}
		//	//}
		//}
		//for (size_t i = 0; i < points[1].size(); i++)circle(src, points[1][i], 1, Scalar(0, 0, 255), 2);
		for (size_t i = 0; i < points[1].size(); i++)circle(src, points[1][i], 1, Scalar(0, 0, 255), 2);
		circle(src, PossiblePt, 1, Scalar(0, 255, 0), 20);
		swap(points[0], points[1]);
		points[1].clear();
		swap(CurGray, preFrameGray);
	}
	string filename;
	if (shootFlag[ID]){
		filename = getStateAndSaveImg(src);
		shootFlag[ID] = false;
	}
	if (FallFlag){
		//返回状态
		PersonnelState = "2|FG|" + filename;
	}
	else if (!points[0].empty()){
		//返回状态
		PersonnelState = "1|FG|" + filename;
	}
	else {
		//返回状态
		PersonnelState = "0|FG|" + filename;
	}
	return PersonnelState;
}
string Track::getStateAndSaveImg(Mat& src){
	homepath = ConfigPath + "//img//";
	SYSTEMTIME sys;
	GetLocalTime(&sys);
	string times = to_string(sys.wYear) + to_string(sys.wMonth) + to_string(sys.wDay);
	string directory = homepath + times;
	CreatDir(directory.data());
	string filename = directory + "//" + to_string(sys.wHour) + to_string(sys.wMinute) + to_string(sys.wSecond) + "[shoot].jpg";
	string logs = "\nSavePath：" + filename;
	BDLOG(logs.c_str());
	imwrite(filename, src);
	return filename;
}