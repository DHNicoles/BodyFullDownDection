#include	"VibeSub.h"

const int  VibeSub::defaultNbSamples = 20;
const int  VibeSub::defaultReqMatches = 2;//2// 6;		//#minָ��
const int  VibeSub::defaultRadius = 8;//20//8;		//sqthere�뾶
const int  VibeSub::defaultSubsamplingFactor = 16;	//�Ӳ�������
const int  VibeSub::background = 0;	//��������
const int  VibeSub::foreground = 255;		//ǰ������

float samples[2000][2000][VibeSub::defaultNbSamples + 1];

VibeSub::VibeSub(const char* filename, const char* xmlname)
:rng(0xFFFFFFFF), scale(600, 300), cap(filename), track(xmlname),
//pFrame(cvCreateImage(scale, IPL_DEPTH_8U, 3)),
segMap(cvCreateImage(scale, IPL_DEPTH_8U, 1)),
pAfter(cvCreateImage(scale, IPL_DEPTH_8U, 1)),
segMat(cvCreateMat(scale.height, scale.width, CV_32FC1)),
pAfterMat(cvCreateMat(scale.height, scale.width, CV_32FC1))
{
	int xoff[9] = { -1, 0, 1, -1, 1, -1, 0, 1, 0 };//x���ھӵ�
	int yoff[9] = { -1, 0, 1, -1, 1, -1, 0, 1, 0 };//y���ھӵ�
	for (int i = 0; i < 9; ++i){
		c_xoff[i] = xoff[i];
		c_yoff[i] = yoff[i];
	}
}
VibeSub::~VibeSub(){
	cap.release();
	cvReleaseImage(&segMap);
	cvReleaseImage(&pAfter);
	cvReleaseMat(&segMat);
	cvReleaseMat(&pAfterMat);
}
void VibeSub::run(){
	if (!cap.isOpened()){
		cout << "no file" << endl;
	}
	cap >> frame;
	resize(frame, frame, scale);
	//ת���ɵ�ͨ��ͼ���ٴ���
	pFrame = (IplImage*)cvClone(&IplImage(frame));//??��debug��ֱ�Ӿ���pFrame=&IplImage(frame);���ɣ�������release��Ҫ�ĳ��������
	cout << pFrame->nChannels << endl;
	cvCvtColor(pFrame, pAfter, CV_BGR2GRAY);
	cvConvert(pAfter, pAfterMat);
	//init
	Initialize(pAfterMat, rng);
	cap >> frame;
	while (!frame.empty()){
		//if (ii++ == 300)waitKey(0);
		resize(frame, frame, scale);
		pFrame = &IplImage(frame);
		cvCvtColor(pFrame, pAfter, CV_BGR2GRAY);
		cvConvert(pAfter, pAfterMat);
		update(pAfterMat, segMat, rng);//���±���
		cvConvert(segMat, segMap);

		//blur(frame, frame, Size(5, 5), Point(-1, -1));
		medianBlur(frame, frame, 3);
		//GaussianBlur(frame, frame, Size(3, 3), 0, 0);

		track.proc(frame, segMap);
		//cvNamedWindow("segMap");
		//cvShowImage("segMap", segMap.get());
		imshow("frame", frame);
		if (waitKey(10) > 0){
			waitKey(0);
			//break;
		}
		cap >> frame;
	}
	//cvReleaseImage(&pFrame);
	return;
}
void VibeSub::Initialize(CvMat* pFrameMat, RNG rng){

	//��¼������ɵ� ��(r) �� ��(c)
	int rand, r, c;

	//��ÿ�������������г�ʼ��
	for (int y = 0; y < pFrameMat->rows; y++){//Height
		for (int x = 0; x < pFrameMat->cols; x++){//Width
			for (int k = 0; k < defaultNbSamples; k++){
				//�����ȡ��������ֵ
				rand = rng.uniform(0, 9);
				r = y + c_yoff[rand]; if (r < 0) r = 0; if (r >= pFrameMat->rows) r = pFrameMat->rows - 1;	//��
				c = x + c_xoff[rand]; if (c < 0) c = 0; if (c >= pFrameMat->cols) c = pFrameMat->cols - 1;	//��
				//�洢��������ֵ
				samples[y][x][k] = CV_MAT_ELEM(*pFrameMat, float, r, c);
			}
			samples[y][x][defaultNbSamples] = 0;
		}
	}
}


//���º���
void VibeSub::update(CvMat* pFrameMat, CvMat* segMat, RNG rng){

	for (int y = 0; y < pFrameMat->rows; y++){	//Height
		for (int x = 0; x < pFrameMat->cols; x++){	//Width

			//�����ж�һ�����Ƿ��Ǳ�����,index��¼�ѱȽϵ�����������count��ʾƥ�����������
			int count = 0, index = 0; float dist = 0;
			//
			while ((count < defaultReqMatches) && (index < defaultNbSamples)){
				dist = CV_MAT_ELEM(*pFrameMat, float, y, x) - samples[y][x][index];
				if (dist < 0) dist = -dist;
				if (dist < defaultRadius) count++;
				index++;
			}
			if (count >= defaultReqMatches){

				//�ж�Ϊ��������,ֻ�б�������ܱ����������͸��´洢����ֵ
				samples[y][x][defaultNbSamples] = 0;

				*((float *)CV_MAT_ELEM_PTR(*segMat, y, x)) = background;

				int rand = rng.uniform(0, defaultSubsamplingFactor);
				if (rand == 0){
					rand = rng.uniform(0, defaultNbSamples);
					samples[y][x][rand] = CV_MAT_ELEM(*pFrameMat, float, y, x);
				}
				rand = rng.uniform(0, defaultSubsamplingFactor);
				if (rand == 0){
					int xN, yN;
					rand = rng.uniform(0, 9); yN = y + c_yoff[rand]; if (yN < 0) yN = 0; if (yN >= pFrameMat->rows) yN = pFrameMat->rows - 1;
					rand = rng.uniform(0, 9); xN = x + c_xoff[rand]; if (xN<0) xN = 0; if (xN >= pFrameMat->cols) xN = pFrameMat->cols - 1;
					rand = rng.uniform(0, defaultNbSamples);
					samples[yN][xN][rand] = CV_MAT_ELEM(*pFrameMat, float, y, x);
				}
			}
			else {
				//�ж�Ϊǰ������
				*((float *)CV_MAT_ELEM_PTR(*segMat, y, x)) = foreground;

				samples[y][x][defaultNbSamples]++;
				if (samples[y][x][defaultNbSamples]>100){
					int rand = rng.uniform(0, defaultNbSamples);
					if (rand == 0){
						rand = rng.uniform(0, defaultNbSamples);
						samples[y][x][rand] = CV_MAT_ELEM(*pFrameMat, float, y, x);
					}
				}
			}

		}
	}

}