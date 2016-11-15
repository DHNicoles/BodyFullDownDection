
#include <Windows.h>
#include "Ctrlvars.h"
#include "GeneralDef.h"
#include <iostream>
#include <string>
#include	<opencv/highgui.h>
#include	<opencv/cvaux.h>
#include	<opencv/cxcore.h>
#include	"opencv2/core/core.hpp"
#include	"opencv2/imgproc/imgproc.hpp"
#include	"opencv2/video/background_segm.hpp"
#include	"opencv2/objdetect/objdetect.hpp"
#include	"opencv2/highgui/highgui.hpp"
/*****************************************/


cv::Size recvSize(800, 500);
int frameCnt = 0;
bool InitFlag = true;
cv::Mat preFrame, mask;
volatile bool shootFlag[MaxThread] = { false };
/*****************************************/
//initial all
void initall(){
	frameCnt = 0;
}