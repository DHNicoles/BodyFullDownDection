#include	<stdio.h>
#include	<process.h>  
#include	"CameraCtrl.h"
#include	<iostream>
#include	"RGB.h"
#include	"BDLOG.h"
#include	"Global.h"
///////////////////////////////////////
//string HIKPlay::result = "-1|FG|NO";
//HANDLE HIKPlay::hand = NULL;
//bool HIKPlay::g_continue = true;
//unsigned short HIKPlay::SIZE_OF_BUFFER = 100;
////�������������ź�
//HANDLE HIKPlay::g_RetMutex = CreateMutex(NULL, FALSE, NULL);
//HANDLE HIKPlay::g_hMutex = CreateMutex(NULL, FALSE, NULL);
//HANDLE HIKPlay::g_hEmptySemaphore = CreateSemaphore(NULL, SIZE_OF_BUFFER - 1, SIZE_OF_BUFFER - 1, NULL);
//HANDLE HIKPlay::g_hFullSemaphore = CreateSemaphore(NULL, 0, SIZE_OF_BUFFER - 1, NULL);
//unsigned short HIKPlay::in = 0;
//unsigned short HIKPlay::out = 0;
//int HIKPlay::hik_width_org = 1280;
//int HIKPlay::hik_height_org = 720;
//char* HIKPlay::imagesData[100];
//IplImage* HIKPlay::images[100];//�������Ǹ�ѭ������
string HIKPlay::result[MaxThread];//����ļ����
unsigned short HIKPlay::in[MaxThread]; //��Ʒ��������ʱ�Ļ������±�
unsigned short HIKPlay::out[MaxThread]; //��Ʒ��������ʱ�Ļ������±�
HANDLE HIKPlay::g_RetMutex[MaxThread]; //�����̼߳�Ļ���
HANDLE HIKPlay::g_hMutex[MaxThread]; //�����̼߳�Ļ���
HANDLE HIKPlay::g_hFullSemaphore[MaxThread]; //����������ʱ��ʹ�����ߵȴ�
HANDLE HIKPlay::g_hEmptySemaphore[MaxThread]; //����������ʱ��ʹ�����ߵȴ�
HANDLE HIKPlay::hand[MaxThread];//�̵߳ľ��
unsigned short HIKPlay::SIZE_OF_BUFFER = 100; //����������
bool HIKPlay::g_continue[MaxThread]; //���Ƴ������
int HIKPlay::hik_width_org[MaxThread];
int HIKPlay::hik_height_org[MaxThread];
char * HIKPlay::imagesData[MaxThread][100];
IplImage* HIKPlay::images[MaxThread][100];//�������Ǹ�ѭ������
long HIKPlay::lport[MaxThread];
long HIKPlay::Port[MaxThread] = { 0 };
vector<bool> HIKPlay::IDpool(MaxThread);
int HIKPlay::ValidIndex = -1;
///////////////////////////////////////
//
void (CALLBACK *HIKPlay::g_RealDataCallBack_V30[MaxThread])(LONG lRealHandle, DWORD dwDataType, BYTE *pBuffer, DWORD dwBufSize, void* dwUser) = {
	HIKPlay::g_RealDataCallBack_V30_0,
	HIKPlay::g_RealDataCallBack_V30_1,
	HIKPlay::g_RealDataCallBack_V30_2,
	HIKPlay::g_RealDataCallBack_V30_3,
	HIKPlay::g_RealDataCallBack_V30_4,
	HIKPlay::g_RealDataCallBack_V30_5,
	HIKPlay::g_RealDataCallBack_V30_6,
	HIKPlay::g_RealDataCallBack_V30_7,
	HIKPlay::g_RealDataCallBack_V30_8,
	HIKPlay::g_RealDataCallBack_V30_9

};
void (CALLBACK *HIKPlay::DecCBFun[MaxThread])(long, char *, long, FRAME_INFO *, long, long /*nReserved2*/) = {
	HIKPlay::DecCBFun_0,
	HIKPlay::DecCBFun_1,
	HIKPlay::DecCBFun_2,
	HIKPlay::DecCBFun_3,
	HIKPlay::DecCBFun_4,
	HIKPlay::DecCBFun_5,
	HIKPlay::DecCBFun_6,
	HIKPlay::DecCBFun_7,
	HIKPlay::DecCBFun_8,
	HIKPlay::DecCBFun_9
};
const int  HIKPlay::defaultNbSamples = 20;//ÿ�����ص����������
const int  HIKPlay::defaultReqMatches = 2;//2// 6;		//#minָ��
const int  HIKPlay::defaultRadius = 8;//20//8;		//sqthere�뾶
const int  HIKPlay::defaultSubsamplingFactor = 16;	//�Ӳ�������
const int  HIKPlay::background = 0;	//��������
const int  HIKPlay::foreground = 255;		//ǰ������

float samples[2000][2000][HIKPlay::defaultNbSamples + 1];

int HIKPlay::c_xoff[9] = { -1, 0, 1, -1, 1, -1, 0, 1, 0 };//x���ھӵ�
int HIKPlay::c_yoff[9] = { -1, 0, 1, -1, 1, -1, 0, 1, 0 };//y���ھӵ�
Size HIKPlay::scale(600, 300);

void HIKPlay::Initialize(CvMat* pFrameMat, RNG rng){
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
void HIKPlay::update(CvMat* pFrameMat, CvMat* segMat, RNG rng){
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
///////////////////////////////////////


HIKPlay::HIKPlay(string IPstr, string username, string password, long port, string confpath) :
IPstr(IPstr), username(username), password(password), m_struDeviceInfo(), m_lPlayHandle(0), IsVideo(false)
, filepath(""){
	ConfigPath = confpath;
	//����һ��ID
	ValidIndex = 0;
	for (int i = 0; i < IDpool.size(); ++i){
		if (!IDpool[i]){
			ValidIndex = i;
			IDpool[i] = true;
			break;
		}
	}
	ID = ValidIndex;
	HIKPlay::lport[ID] = port;
	result[ID] = "-1|FG|NO";
	track.initialXml(string(confpath + "//configs//hogcascade_pedestrians.xml").c_str());
}
HIKPlay::HIKPlay(const char* path, string confpath) :
VideoPath(path), IsVideo(true){
	ConfigPath = confpath;
	//����һ��ID
	ValidIndex = 0;
	for (int i = 0; i < IDpool.size(); ++i){
		if (!IDpool[i]){
			ValidIndex = i;
			IDpool[i] = true;
			break;
		}
	}
	ID = ValidIndex;
	result[ID] = "-1|FG|NO";
	track.initialXml(string(confpath + "//configs//hogcascade_pedestrians.xml").c_str());
	if (!cap.isOpened())BDLOG("file open error!");
}


bool HIKPlay::start(){
	initall();
	if (!IsVideo){//camera
		//login
		bool ret = this->DoLogin(IPstr, username, password, m_struDeviceInfo);
		if (!ret)return false;
		return StartPlay(m_struDeviceInfo);
	}
	else{//files
		cap.open(VideoPath);
		if (!cap.isOpened())return false;
		HIKPlay::Thread_init();//�߳��ź���������ĳ�ʼ��
		//�����̣߳�����������YUV��ʽ��ͼ��
		fileParam* pam = new fileParam();
		pam->pcap = &cap;
		pam->h = &hand[ID];
		pam->HPptr = this;
		hand[ID] = (HANDLE)_beginthreadex(NULL, 0, fileThreadProc, (LPVOID)pam, 0, NULL);
		CloseHandle(hand[ID]);
		return true;
	}
}
bool HIKPlay::DoLogin(string IPstr, string userID, string psw, LOCAL_DEVICE_INFO &m_struDeviceInfo){

	NET_DVR_Init();

	char *sDVRIP = (char *)IPstr.data();
	char *sUserName = (char *)userID.data();
	char *sPassword = (char *)psw.data();
	UINT	m_nDevPort = HIKPlay::lport[ID];
	NET_DVR_DEVICEINFO_V30 DeviceInfoTmp;
	memset(&DeviceInfoTmp, 0, sizeof(NET_DVR_DEVICEINFO_V30));
	//�豸ID
	int m_lDecID = NET_DVR_Login_V30(sDVRIP, m_nDevPort, sUserName, sPassword, &DeviceInfoTmp);
	if (m_lDecID == -1)
	{
		//��ʾ��¼����
		//MessageBox("Login to Device failed!\n");
		DWORD err = NET_DVR_GetLastError();
		//���������ʾ��Ϣ

		cout << "err:" << err << endl;
		NET_DVR_Logout(m_struDeviceInfo.lLoginID);
		NET_DVR_Cleanup();
		return FALSE;
	}
	m_struDeviceInfo.lLoginID = m_lDecID;
	m_struDeviceInfo.iDeviceChanNum = DeviceInfoTmp.byChanNum;
	m_struDeviceInfo.iIPChanNum = DeviceInfoTmp.byIPChanNum;
	m_struDeviceInfo.iStartChan = DeviceInfoTmp.byStartChan;
	DoGetDeviceResoureCfg(m_struDeviceInfo);  //��ȡ�豸��Դ��Ϣ	
	return TRUE;
}
void HIKPlay::DoGetDeviceResoureCfg(LOCAL_DEVICE_INFO &m_struDeviceInfo){
	NET_DVR_IPPARACFG IpAccessCfg;
	memset(&IpAccessCfg, 0, sizeof(IpAccessCfg));
	DWORD  dwReturned;

	m_struDeviceInfo.bIPRet = NET_DVR_GetDVRConfig(m_struDeviceInfo.lLoginID, NET_DVR_GET_IPPARACFG, 0, &IpAccessCfg, sizeof(NET_DVR_IPPARACFG), &dwReturned);
	int i;
	if (!m_struDeviceInfo.bIPRet)   //��֧��ip����,9000�����豸��֧�ֽ���ģ��ͨ��
	{
		for (i = 0; i<MAX_ANALOG_CHANNUM; i++)
		{
			if (i < m_struDeviceInfo.iDeviceChanNum)
			{
				std::sprintf(m_struDeviceInfo.struChanInfo[i].chChanName, "camera%d", i + m_struDeviceInfo.iStartChan);
				m_struDeviceInfo.struChanInfo[i].iChanIndex = i + m_struDeviceInfo.iStartChan;  //ͨ����
				m_struDeviceInfo.struChanInfo[i].bEnable = TRUE;

			}
			else
			{
				m_struDeviceInfo.struChanInfo[i].iChanIndex = -1;
				m_struDeviceInfo.struChanInfo[i].bEnable = FALSE;
				std::sprintf(m_struDeviceInfo.struChanInfo[i].chChanName, "");
			}
		}
	}
	else        //֧��IP���룬9000�豸
	{
		for (i = 0; i<MAX_ANALOG_CHANNUM; i++)  //ģ��ͨ��
		{
			if (i < m_struDeviceInfo.iDeviceChanNum)
			{
				sprintf(m_struDeviceInfo.struChanInfo[i].chChanName, "camera%d", i + m_struDeviceInfo.iStartChan);
				m_struDeviceInfo.struChanInfo[i].iChanIndex = i + m_struDeviceInfo.iStartChan;
				if (IpAccessCfg.byAnalogChanEnable[i])
				{
					m_struDeviceInfo.struChanInfo[i].bEnable = TRUE;
				}
				else
				{
					m_struDeviceInfo.struChanInfo[i].bEnable = FALSE;
				}

			}
			else//clear the state of other channel
			{
				m_struDeviceInfo.struChanInfo[i].iChanIndex = -1;
				m_struDeviceInfo.struChanInfo[i].bEnable = FALSE;
				sprintf(m_struDeviceInfo.struChanInfo[i].chChanName, "");
			}
		}

		//����ͨ��
		for (i = 0; i<MAX_IP_CHANNEL; i++)
		{
			if (IpAccessCfg.struIPChanInfo[i].byEnable)  //ipͨ������
			{
				m_struDeviceInfo.struChanInfo[i + MAX_ANALOG_CHANNUM].bEnable = TRUE;
				m_struDeviceInfo.struChanInfo[i + MAX_ANALOG_CHANNUM].iChanIndex = IpAccessCfg.struIPChanInfo[i].byChannel;
				sprintf(m_struDeviceInfo.struChanInfo[i + MAX_ANALOG_CHANNUM].chChanName, "IP Camera %d", IpAccessCfg.struIPChanInfo[i].byChannel);

			}
			else
			{
				m_struDeviceInfo.struChanInfo[i + MAX_ANALOG_CHANNUM].bEnable = FALSE;
				m_struDeviceInfo.struChanInfo[i + MAX_ANALOG_CHANNUM].iChanIndex = -1;
			}
		}


	}
}
typedef HWND(WINAPI *PROCGETCONSOLEWINDOW)();

bool HIKPlay::StartPlay(const LOCAL_DEVICE_INFO &m_struDeviceInfo){
	HIKPlay::Thread_init();//�߳��ź���������ĳ�ʼ��
	int m_iCurChanIndex = 0;
	NET_DVR_CLIENTINFO ClientInfo;
	ClientInfo.hPlayWnd = NULL;//GetDlgItem(IDC_STATIC_PLAY)->m_hWnd;
	ClientInfo.lChannel = m_struDeviceInfo.struChanInfo[m_iCurChanIndex].iChanIndex;
	ClientInfo.lLinkMode = 0;
	ClientInfo.sMultiCastIP = NULL;;
	//�ص�����g_RealDataCallBack_V30
	//wocao ����ص�����ʼ��û�н�ȥ���Ͱ�sdk���е�����dll������debug�� made
	int *param = new int(ID);
	LONG m_lPlayHandle = NET_DVR_RealPlay_V30(m_struDeviceInfo.lLoginID, &ClientInfo, g_RealDataCallBack_V30[ID], param, FALSE);
	if (-1 == m_lPlayHandle)
	{
		//BDLOG("����������ص���������ʧ��");
		DWORD err = NET_DVR_GetLastError();
		//���������ʾ��Ϣ
		//MessageBox(m_csErr);
		//cout << "err:"<<err << endl;
		NET_DVR_Logout(m_struDeviceInfo.lLoginID);
		NET_DVR_Cleanup();
		return false;
	}

	//�����̣߳�����������YUV��ʽ��ͼ��
	threadParam* pam = new threadParam();
	pam->HPptr = this;
	hand[ID] = (HANDLE)_beginthreadex(NULL, 0, playThreadProc, (LPVOID)pam, 0, NULL);
	CloseHandle(hand[ID]);
	Sleep(2000);
	return true;
}
bool HIKPlay::stop(){
	g_continue[ID] = false;
	shootFlag[ID] = false;
	IDpool[ID] = false;
	result[ID] = "-1|FG|NO";
	track.clear();
	if (!IsVideo){
		NET_DVR_StopRealPlay(m_struDeviceInfo.lLoginID);
		BOOL ret1 = NET_DVR_Logout(m_struDeviceInfo.lLoginID);
		BOOL ret2 = NET_DVR_Cleanup();
		return ret1&&ret2;
	}
	Sleep(2000);
	return CloseHandle(g_RetMutex[ID]);
	//return true;
}
string HIKPlay::getLastResult(){
	string ret;
	shootFlag[ID] = true;
	while (shootFlag[ID] && g_continue[ID]){}//�ȴ�ͼ����proc����
	while (WaitForSingleObject(g_RetMutex[ID], 100) == WAIT_TIMEOUT){}
	ret = result[ID];
	ReleaseMutex(g_RetMutex[ID]);
	cout << "getLastResult" << endl;
	return ret;
}
void HIKPlay::Thread_init()
{
	//BDLOG("�̳߳�ʼ����Thread_init");
	hand[ID] = NULL;
	g_continue[ID] = true;

	//�������������ź�
	g_RetMutex[ID] = CreateMutex(NULL, FALSE, NULL);
	g_hMutex[ID] = CreateMutex(NULL, FALSE, NULL);
	g_hEmptySemaphore[ID] = CreateSemaphore(NULL, SIZE_OF_BUFFER - 1, SIZE_OF_BUFFER - 1, NULL);
	g_hFullSemaphore[ID] = CreateSemaphore(NULL, 0, SIZE_OF_BUFFER - 1, NULL);
	in[ID] = 0;
	out[ID] = 0;
	hik_width_org[ID] = 1280;
	hik_height_org[ID] = 720;
}

unsigned int __stdcall HIKPlay::fileThreadProc(LPVOID lpParameter){
	fileParam *playInfo = (fileParam*)lpParameter;
	int ID = playInfo->HPptr->ID;
	HIKPlay* ObjPtr = playInfo->HPptr;
	VideoCapture vc = *playInfo->pcap;
	HANDLE h = playInfo->h;
	delete playInfo;

	vc >> ObjPtr->frame;
	resize(ObjPtr->frame, ObjPtr->frame, scale);
	//ת���ɵ�ͨ��ͼ���ٴ���
	ObjPtr->pFrame = (IplImage*)cvClone(&IplImage(ObjPtr->frame));//??��debug��ֱ�Ӿ���pFrame=&IplImage(frame);���ɣ�������release��Ҫ�ĳ��������
	cout << ObjPtr->pFrame->nChannels << endl;
	//init
	ObjPtr->rng = RNG(0xFFFFFFFF);
	ObjPtr->pAfter = cvCreateImage(scale, IPL_DEPTH_8U, 1);
	ObjPtr->segMap = cvCreateImage(scale, IPL_DEPTH_8U, 1);
	ObjPtr->pAfterMat = cvCreateMat(scale.height, scale.width, CV_32FC1);
	ObjPtr->segMat = cvCreateMat(scale.height, scale.width, CV_32FC1);
	cvCvtColor(ObjPtr->pFrame, ObjPtr->pAfter, CV_BGR2GRAY);
	cvConvert(ObjPtr->pAfter, ObjPtr->pAfterMat);
	ObjPtr->Initialize(ObjPtr->pAfterMat, ObjPtr->rng);
	vc >> ObjPtr->frame;

	string WinName = "video channel : "+to_string(ID);
	cvNamedWindow(WinName.c_str(), CV_WINDOW_AUTOSIZE);
	while (g_continue[ID] && !ObjPtr->frame.empty())
	{
		resize(ObjPtr->frame, ObjPtr->frame, scale);
		ObjPtr->pFrame = &IplImage(ObjPtr->frame);
		cvCvtColor(ObjPtr->pFrame, ObjPtr->pAfter, CV_BGR2GRAY);
		cvConvert(ObjPtr->pAfter, ObjPtr->pAfterMat);
		ObjPtr->update(ObjPtr->pAfterMat, ObjPtr->segMat, ObjPtr->rng);//���±���
		cvConvert(ObjPtr->segMat, ObjPtr->segMap);

		//blur(frame, frame, Size(5, 5), Point(-1, -1));
		//medianBlur(ObjPtr->frame, ObjPtr->frame, 3);
		//GaussianBlur(frame, frame, Size(3, 3), 0, 0);
		if (WaitForSingleObject(g_RetMutex[ID], 100) == WAIT_TIMEOUT)continue;
		string ret = (ObjPtr->track).proc(ObjPtr->frame, ObjPtr->segMap, ID);
		result[ID] = ret.empty() ? result[ID] : ret;
		//cout << result << endl;
		ReleaseMutex(g_RetMutex[ID]);
		cvShowImage(WinName.c_str(), &IplImage(ObjPtr->frame));
		if (waitKey(10)>0)break;
		vc >> ObjPtr->frame;
	}
	cvDestroyWindow(WinName.c_str());
	return 0;
}
//DWORD WINAPI playThreadProc(LPVOID lpParameter)
unsigned int __stdcall HIKPlay::playThreadProc(LPVOID lpParameter)
{

	threadParam *playInfo = (threadParam*)lpParameter;
	int ID = playInfo->HPptr->ID;
	HIKPlay* ObjPtr = playInfo->HPptr;
	int prePointNumber = -1, number = -1;
	
	int winplus = 0;
	string WinName = "HIKIMG_CHANNEL: " + to_string(ID);
	cvNamedWindow(WinName.c_str(), CV_WINDOW_AUTOSIZE);
	bool InitialFlag = false;
	while (g_continue[ID] == true)
	{
		HANDLE h[2] = { g_hFullSemaphore[ID], g_hMutex[ID] };
		if (WaitForMultipleObjects(2, h, TRUE, 100) == WAIT_TIMEOUT)continue;
		//ת��YV12��RGB
		IplImage* imageresize = cvCreateImage(scale, IPL_DEPTH_8U, 3);
		IplImage* src = YV12_To_IplImage(images[ID][out[ID]], (unsigned char *)imagesData[ID][out[ID]], hik_width_org[ID], hik_height_org[ID]);
		cvResize(src, imageresize, CV_INTER_LINEAR);		//���¶���ͼ��Ĵ�С��recvSize
		cvReleaseImage(&images[ID][out[ID]]);//�ͷ�ԭ���±괦��ͼ���ڴ�
		images[ID][out[ID]] = imageresize;
		cvReleaseImage(&src);
		delete[] imagesData[ID][out[ID]];//�ͷ���ǰ��ʽͼ���ڴ�
		out[ID] = (out[ID] + 1) % SIZE_OF_BUFFER;
		ReleaseMutex(g_hMutex[ID]);
		ReleaseSemaphore(g_hEmptySemaphore[ID], 1, NULL);
		if (imageresize){
			if (!InitialFlag){
				//����һ�������������
				
				ObjPtr->rng = RNG(0xFFFFFFFF);	
				ObjPtr->pAfter = cvCreateImage(scale, IPL_DEPTH_8U, 1);
				ObjPtr->segMap = cvCreateImage(scale, IPL_DEPTH_8U, 1);
				ObjPtr->pAfterMat = cvCreateMat(scale.height, scale.width, CV_32FC1);
				ObjPtr->segMat = cvCreateMat(scale.height, scale.width, CV_32FC1);
				ObjPtr->pFrame = imageresize;//??��debug��ֱ�Ӿ���pFrame=&IplImage(frame);���ɣ�������release��Ҫ�ĳ��������
				cvCvtColor(ObjPtr->pFrame, ObjPtr->pAfter, CV_BGR2GRAY);
				cvConvert(ObjPtr->pAfter, ObjPtr->pAfterMat);
				//init
				ObjPtr->Initialize(ObjPtr->pAfterMat, ObjPtr->rng);
				InitialFlag = true;
			}
			else{
				ObjPtr->pFrame = imageresize;
				cvCvtColor(ObjPtr->pFrame, ObjPtr->pAfter, CV_BGR2GRAY);
				cvConvert(ObjPtr->pAfter, ObjPtr->pAfterMat);
				ObjPtr->update(ObjPtr->pAfterMat, ObjPtr->segMat, ObjPtr->rng);//���±���
				cvConvert(ObjPtr->segMat, ObjPtr->segMap);
				ObjPtr->frame = Mat(ObjPtr->pFrame);
				//blur(frame, frame, Size(5, 5), Point(-1, -1));
				//medianBlur(ObjPtr->frame, ObjPtr->frame, 3);
				//GaussianBlur(frame, frame, Size(3, 3), 0, 0);

				if (WaitForSingleObject(g_RetMutex[ID], 100) == WAIT_TIMEOUT)continue;
				string ret = (ObjPtr->track).proc(ObjPtr->frame, ObjPtr->segMap, ID);
				result[ID] = ret.empty() ? result[ID] : ret;
				ReleaseMutex(g_RetMutex[ID]);
				resize(ObjPtr->frame, ObjPtr->frame, Size(1280, 720));
				cvShowImage(WinName.c_str(), &IplImage(ObjPtr->frame));
				waitKey(1);
			}
		}
	}
	cvDestroyWindow(WinName.c_str());
	delete playInfo;
	CloseHandle(g_hMutex[ID]);
	CloseHandle(g_hEmptySemaphore[ID]);
	CloseHandle(g_hFullSemaphore[ID]);
	return 0;
}

//YV12תΪBGR24����
bool HIKPlay::YV12_To_BGR24(unsigned char* pYV12, unsigned char* pRGB24, int width, int height)
{
	if (!pYV12 || !pRGB24)
	{
		return false;
	}

	const long nYLen = long(height * width);
	const int halfWidth = (width >> 1);

	if (nYLen<1 || halfWidth<1)
	{
		return false;
	}

	// yv12's data structure
	// |WIDTH |
	// y......y--------
	// y......y   HEIGHT
	// y......y
	// y......y--------
	// v..v
	// v..v
	// u..u
	// u..u
	unsigned char* yData = pYV12;
	unsigned char* vData = &yData[nYLen];
	unsigned char* uData = &vData[nYLen >> 2];

	if (!uData || !vData)
	{
		return false;
	}

	// Convert YV12 to RGB24
	int rgb[3];
	int i, j, m, n, x, y;
	m = -width;
	n = -halfWidth;
	for (y = 0; y<height; y++)
	{
		m += width;
		if (!(y % 2))
			n += halfWidth;
		for (x = 0; x<width; x++)
		{
			i = m + x;
			j = n + (x >> 1);
			rgb[2] = yv2r_table[yData[i]][vData[j]];
			rgb[1] = yData[i] - uv2ig_table[uData[j]][vData[j]];
			rgb[0] = yu2b_table[yData[i]][uData[j]];
			//rgb[2] = int(yData[i] + 1.370705 * (vData[j] - 128)); // r
			//rgb[1] = int(yData[i] - 0.698001 * (uData[j] - 128)  - 0.703125 * (vData[j] - 128));   // g
			//rgb[0] = int(yData[i] + 1.732446 * (uData[j] - 128)); // b		

			//j = nYLen - iWidth - m + x;
			//i = (j<<1) + j;    //ͼ�������µߵ���

			j = m + x;
			i = (j << 1) + j;

			for (j = 0; j<3; j++)
			{
				if (rgb[j] >= 0 && rgb[j] <= 255)
					pRGB24[i + j] = rgb[j];
				else
					pRGB24[i + j] = (rgb[j] < 0) ? 0 : 255;
			}
		}
	}

	if (pRGB24 == NULL)
	{
		return false;
	}

	return true;
}

IplImage* HIKPlay::YV12_To_IplImage(IplImage* pImage, unsigned char* pYV12, int width, int height)
{
	if (!pYV12)
	{
		return NULL;
	}

	int sizeRGB = width* height * 3;
	unsigned char* pRGB24 = new unsigned char[sizeRGB];
	memset(pRGB24, 0, sizeRGB);

	if (YV12_To_BGR24(pYV12, pRGB24, width, height) == false || (!pRGB24))
	{
		return NULL;
	}

	pImage = cvCreateImage(cvSize(width, height), 8, 3);
	pImage->origin = 1;
	if (!pImage)
	{
		return NULL;
	}

	memcpy(pImage->imageData, pRGB24, sizeRGB);
	if (!(pImage->imageData))
	{
		return NULL;
	}

	delete[] pRGB24;
	return pImage;
}

void CALLBACK HIKPlay::g_RealDataCallBack_V30_0(LONG lRealHandle, DWORD dwDataType, BYTE *pBuffer, DWORD dwBufSize, void* dwUser){
	//global var
	//cout << "g_RealDataCallBack_V30" << endl;
	//cout << "g_RealDataCallBack_V30_" << endl;

	LONG m_iPort = 0;
	HWND hWnd = GetConsoleWindow();
	int *pint = (int*)dwUser;
	int ID = *pint;
	switch (dwDataType)
	{
	case NET_DVR_SYSHEAD: //ϵͳͷ

		if (!PlayM4_GetPort(&Port[ID]))  //��ȡ���ſ�δʹ�õ�ͨ����
		{
			break;
		}
		m_iPort = Port[ID]; //��һ�λص�����ϵͳͷ������ȡ�Ĳ��ſ�port�Ÿ�ֵ��ȫ��port���´λص�����ʱ��ʹ�ô�port�Ų���
		if (dwBufSize > 0)
		{
			if (!PlayM4_SetStreamOpenMode(Port[ID], STREAME_REALTIME))  //����ʵʱ������ģʽ
			{
				break;
			}

			if (!PlayM4_OpenStream(Port[ID], pBuffer, dwBufSize, 1024 * 1024)) //�����ӿ�
			{
				break;
			}
			PlayM4_SetDecCallBack(Port[ID], DecCBFun[ID]);
			if (!PlayM4_Play(Port[ID], hWnd)) //���ſ�ʼ
			{
				break;
			}
		}
	case NET_DVR_STREAMDATA:   //��������
		if (dwBufSize > 0 && Port[ID] != -1)
		{

			if (!PlayM4_InputData(Port[ID], pBuffer, dwBufSize))
			{
				break;
			}
		}
	}
	////delete pint;
}
void CALLBACK HIKPlay::g_RealDataCallBack_V30_1(LONG lRealHandle, DWORD dwDataType, BYTE *pBuffer, DWORD dwBufSize, void* dwUser){
	//global var
	//cout << "g_RealDataCallBack_V30" << endl;

	LONG m_iPort = 0;
	HWND hWnd = GetConsoleWindow();
	int *pint = (int*)dwUser;
	int ID = *pint;

	switch (dwDataType)
	{
	case NET_DVR_SYSHEAD: //ϵͳͷ

		if (!PlayM4_GetPort(&Port[ID]))  //��ȡ���ſ�δʹ�õ�ͨ����
		{
			break;
		}
		m_iPort = Port[ID]; //��һ�λص�����ϵͳͷ������ȡ�Ĳ��ſ�port�Ÿ�ֵ��ȫ��port���´λص�����ʱ��ʹ�ô�port�Ų���
		if (dwBufSize > 0)
		{
			if (!PlayM4_SetStreamOpenMode(Port[ID], STREAME_REALTIME))  //����ʵʱ������ģʽ
			{
				break;
			}

			if (!PlayM4_OpenStream(Port[ID], pBuffer, dwBufSize, 1024 * 1024)) //�����ӿ�
			{
				break;
			}
			PlayM4_SetDecCallBack(Port[ID], DecCBFun[ID]);
			if (!PlayM4_Play(Port[ID], hWnd)) //���ſ�ʼ
			{
				break;
			}
		}
	case NET_DVR_STREAMDATA:   //��������
		if (dwBufSize > 0 && Port[ID] != -1)
		{

			if (!PlayM4_InputData(Port[ID], pBuffer, dwBufSize))
			{
				break;
			}
		}
	}

}
void CALLBACK HIKPlay::g_RealDataCallBack_V30_2(LONG lRealHandle, DWORD dwDataType, BYTE *pBuffer, DWORD dwBufSize, void* dwUser){
	//global var
	//cout << "g_RealDataCallBack_V30" << endl;

	LONG m_iPort = 0;
	HWND hWnd = GetConsoleWindow();
	int *pint = (int*)dwUser;
	int ID = *pint;
	//delete pint;
	switch (dwDataType)
	{
	case NET_DVR_SYSHEAD: //ϵͳͷ

		if (!PlayM4_GetPort(&Port[ID]))  //��ȡ���ſ�δʹ�õ�ͨ����
		{
			break;
		}
		m_iPort = Port[ID]; //��һ�λص�����ϵͳͷ������ȡ�Ĳ��ſ�Port[ID]�Ÿ�ֵ��ȫ��Port[ID]���´λص�����ʱ��ʹ�ô�Port[ID]�Ų���
		if (dwBufSize > 0)
		{
			if (!PlayM4_SetStreamOpenMode(Port[ID], STREAME_REALTIME))  //����ʵʱ������ģʽ
			{
				break;
			}

			if (!PlayM4_OpenStream(Port[ID], pBuffer, dwBufSize, 1024 * 1024)) //�����ӿ�
			{
				break;
			}
			PlayM4_SetDecCallBack(Port[ID], DecCBFun[ID]);
			if (!PlayM4_Play(Port[ID], hWnd)) //���ſ�ʼ
			{
				break;
			}
		}
	case NET_DVR_STREAMDATA:   //��������
		if (dwBufSize > 0 && Port[ID] != -1)
		{

			if (!PlayM4_InputData(Port[ID], pBuffer, dwBufSize))
			{
				break;
			}
		}
	}

}
void CALLBACK HIKPlay::g_RealDataCallBack_V30_3(LONG lRealHandle, DWORD dwDataType, BYTE *pBuffer, DWORD dwBufSize, void* dwUser){
	//global var
	//cout << "g_RealDataCallBack_V30" << endl;

	LONG m_iPort = 0;
	HWND hWnd = GetConsoleWindow();
	int *pint = (int*)dwUser;
	int ID = *pint;
	//delete pint;
	switch (dwDataType)
	{
	case NET_DVR_SYSHEAD: //ϵͳͷ

		if (!PlayM4_GetPort(&Port[ID]))  //��ȡ���ſ�δʹ�õ�ͨ����
		{
			break;
		}
		m_iPort = Port[ID]; //��һ�λص�����ϵͳͷ������ȡ�Ĳ��ſ�Port[ID]�Ÿ�ֵ��ȫ��Port[ID]���´λص�����ʱ��ʹ�ô�Port[ID]�Ų���
		if (dwBufSize > 0)
		{
			if (!PlayM4_SetStreamOpenMode(Port[ID], STREAME_REALTIME))  //����ʵʱ������ģʽ
			{
				break;
			}

			if (!PlayM4_OpenStream(Port[ID], pBuffer, dwBufSize, 1024 * 1024)) //�����ӿ�
			{
				break;
			}
			PlayM4_SetDecCallBack(Port[ID], DecCBFun[ID]);
			if (!PlayM4_Play(Port[ID], hWnd)) //���ſ�ʼ
			{
				break;
			}
		}
	case NET_DVR_STREAMDATA:   //��������
		if (dwBufSize > 0 && Port[ID] != -1)
		{

			if (!PlayM4_InputData(Port[ID], pBuffer, dwBufSize))
			{
				break;
			}
		}
	}

}
void CALLBACK HIKPlay::g_RealDataCallBack_V30_4(LONG lRealHandle, DWORD dwDataType, BYTE *pBuffer, DWORD dwBufSize, void* dwUser){
	//global var
	//cout << "g_RealDataCallBack_V30" << endl;

	LONG m_iPort = 0;
	HWND hWnd = GetConsoleWindow();
	int *pint = (int*)dwUser;
	int ID = *pint;
	//delete pint;
	switch (dwDataType)
	{
	case NET_DVR_SYSHEAD: //ϵͳͷ

		if (!PlayM4_GetPort(&Port[ID]))  //��ȡ���ſ�δʹ�õ�ͨ����
		{
			break;
		}
		m_iPort = Port[ID]; //��һ�λص�����ϵͳͷ������ȡ�Ĳ��ſ�Port[ID]�Ÿ�ֵ��ȫ��Port[ID]���´λص�����ʱ��ʹ�ô�Port[ID]�Ų���
		if (dwBufSize > 0)
		{
			if (!PlayM4_SetStreamOpenMode(Port[ID], STREAME_REALTIME))  //����ʵʱ������ģʽ
			{
				break;
			}

			if (!PlayM4_OpenStream(Port[ID], pBuffer, dwBufSize, 1024 * 1024)) //�����ӿ�
			{
				break;
			}
			PlayM4_SetDecCallBack(Port[ID], DecCBFun[ID]);
			if (!PlayM4_Play(Port[ID], hWnd)) //���ſ�ʼ
			{
				break;
			}
		}
	case NET_DVR_STREAMDATA:   //��������
		if (dwBufSize > 0 && Port[ID] != -1)
		{

			if (!PlayM4_InputData(Port[ID], pBuffer, dwBufSize))
			{
				break;
			}
		}
	}

}
void CALLBACK HIKPlay::g_RealDataCallBack_V30_5(LONG lRealHandle, DWORD dwDataType, BYTE *pBuffer, DWORD dwBufSize, void* dwUser){
	//global var
	//cout << "g_RealDataCallBack_V30" << endl;

	LONG m_iPort = 0;
	HWND hWnd = GetConsoleWindow();
	int *pint = (int*)dwUser;
	int ID = *pint;
	//delete pint;
	switch (dwDataType)
	{
	case NET_DVR_SYSHEAD: //ϵͳͷ

		if (!PlayM4_GetPort(&Port[ID]))  //��ȡ���ſ�δʹ�õ�ͨ����
		{
			break;
		}
		m_iPort = Port[ID]; //��һ�λص�����ϵͳͷ������ȡ�Ĳ��ſ�Port[ID]�Ÿ�ֵ��ȫ��Port[ID]���´λص�����ʱ��ʹ�ô�Port[ID]�Ų���
		if (dwBufSize > 0)
		{
			if (!PlayM4_SetStreamOpenMode(Port[ID], STREAME_REALTIME))  //����ʵʱ������ģʽ
			{
				break;
			}

			if (!PlayM4_OpenStream(Port[ID], pBuffer, dwBufSize, 1024 * 1024)) //�����ӿ�
			{
				break;
			}
			PlayM4_SetDecCallBack(Port[ID], DecCBFun[ID]);
			if (!PlayM4_Play(Port[ID], hWnd)) //���ſ�ʼ
			{
				break;
			}
		}
	case NET_DVR_STREAMDATA:   //��������
		if (dwBufSize > 0 && Port[ID] != -1)
		{

			if (!PlayM4_InputData(Port[ID], pBuffer, dwBufSize))
			{
				break;
			}
		}
	}

}
void CALLBACK HIKPlay::g_RealDataCallBack_V30_6(LONG lRealHandle, DWORD dwDataType, BYTE *pBuffer, DWORD dwBufSize, void* dwUser){
	//global var
	//cout << "g_RealDataCallBack_V30" << endl;

	LONG m_iPort = 0;
	HWND hWnd = GetConsoleWindow();
	int *pint = (int*)dwUser;
	int ID = *pint;
	//delete pint;
	switch (dwDataType)
	{
	case NET_DVR_SYSHEAD: //ϵͳͷ

		if (!PlayM4_GetPort(&Port[ID]))  //��ȡ���ſ�δʹ�õ�ͨ����
		{
			break;
		}
		m_iPort = Port[ID]; //��һ�λص�����ϵͳͷ������ȡ�Ĳ��ſ�Port[ID]�Ÿ�ֵ��ȫ��Port[ID]���´λص�����ʱ��ʹ�ô�Port[ID]�Ų���
		if (dwBufSize > 0)
		{
			if (!PlayM4_SetStreamOpenMode(Port[ID], STREAME_REALTIME))  //����ʵʱ������ģʽ
			{
				break;
			}

			if (!PlayM4_OpenStream(Port[ID], pBuffer, dwBufSize, 1024 * 1024)) //�����ӿ�
			{
				break;
			}
			PlayM4_SetDecCallBack(Port[ID], DecCBFun[ID]);
			if (!PlayM4_Play(Port[ID], hWnd)) //���ſ�ʼ
			{
				break;
			}
		}
	case NET_DVR_STREAMDATA:   //��������
		if (dwBufSize > 0 && Port[ID] != -1)
		{

			if (!PlayM4_InputData(Port[ID], pBuffer, dwBufSize))
			{
				break;
			}
		}
	}

}
void CALLBACK HIKPlay::g_RealDataCallBack_V30_7(LONG lRealHandle, DWORD dwDataType, BYTE *pBuffer, DWORD dwBufSize, void* dwUser){
	//global var
	//cout << "g_RealDataCallBack_V30" << endl;

	LONG m_iPort = 0;
	HWND hWnd = GetConsoleWindow();
	int *pint = (int*)dwUser;
	int ID = *pint;
	//delete pint;
	switch (dwDataType)
	{
	case NET_DVR_SYSHEAD: //ϵͳͷ

		if (!PlayM4_GetPort(&Port[ID]))  //��ȡ���ſ�δʹ�õ�ͨ����
		{
			break;
		}
		m_iPort = Port[ID]; //��һ�λص�����ϵͳͷ������ȡ�Ĳ��ſ�Port[ID]�Ÿ�ֵ��ȫ��Port[ID]���´λص�����ʱ��ʹ�ô�Port[ID]�Ų���
		if (dwBufSize > 0)
		{
			if (!PlayM4_SetStreamOpenMode(Port[ID], STREAME_REALTIME))  //����ʵʱ������ģʽ
			{
				break;
			}

			if (!PlayM4_OpenStream(Port[ID], pBuffer, dwBufSize, 1024 * 1024)) //�����ӿ�
			{
				break;
			}
			PlayM4_SetDecCallBack(Port[ID], DecCBFun[ID]);
			if (!PlayM4_Play(Port[ID], hWnd)) //���ſ�ʼ
			{
				break;
			}
		}
	case NET_DVR_STREAMDATA:   //��������
		if (dwBufSize > 0 && Port[ID] != -1)
		{

			if (!PlayM4_InputData(Port[ID], pBuffer, dwBufSize))
			{
				break;
			}
		}
	}

}
void CALLBACK HIKPlay::g_RealDataCallBack_V30_8(LONG lRealHandle, DWORD dwDataType, BYTE *pBuffer, DWORD dwBufSize, void* dwUser){
	//global var
	//cout << "g_RealDataCallBack_V30" << endl;

	LONG m_iPort = 0;
	HWND hWnd = GetConsoleWindow();
	int *pint = (int*)dwUser;
	int ID = *pint;
	//delete pint;
	switch (dwDataType)
	{
	case NET_DVR_SYSHEAD: //ϵͳͷ

		if (!PlayM4_GetPort(&Port[ID]))  //��ȡ���ſ�δʹ�õ�ͨ����
		{
			break;
		}
		m_iPort = Port[ID]; //��һ�λص�����ϵͳͷ������ȡ�Ĳ��ſ�Port[ID]�Ÿ�ֵ��ȫ��Port[ID]���´λص�����ʱ��ʹ�ô�Port[ID]�Ų���
		if (dwBufSize > 0)
		{
			if (!PlayM4_SetStreamOpenMode(Port[ID], STREAME_REALTIME))  //����ʵʱ������ģʽ
			{
				break;
			}

			if (!PlayM4_OpenStream(Port[ID], pBuffer, dwBufSize, 1024 * 1024)) //�����ӿ�
			{
				break;
			}
			PlayM4_SetDecCallBack(Port[ID], DecCBFun[ID]);
			if (!PlayM4_Play(Port[ID], hWnd)) //���ſ�ʼ
			{
				break;
			}
		}
	case NET_DVR_STREAMDATA:   //��������
		if (dwBufSize > 0 && Port[ID] != -1)
		{

			if (!PlayM4_InputData(Port[ID], pBuffer, dwBufSize))
			{
				break;
			}
		}
	}

}
void CALLBACK HIKPlay::g_RealDataCallBack_V30_9(LONG lRealHandle, DWORD dwDataType, BYTE *pBuffer, DWORD dwBufSize, void* dwUser){
	//global var
	//cout << "g_RealDataCallBack_V30" << endl;

	LONG m_iPort = 0;
	HWND hWnd = GetConsoleWindow();
	int *pint = (int*)dwUser;
	int ID = *pint;
	//delete pint;
	switch (dwDataType)
	{
	case NET_DVR_SYSHEAD: //ϵͳͷ

		if (!PlayM4_GetPort(&Port[ID]))  //��ȡ���ſ�δʹ�õ�ͨ����
		{
			break;
		}
		m_iPort = Port[ID]; //��һ�λص�����ϵͳͷ������ȡ�Ĳ��ſ�Port[ID]�Ÿ�ֵ��ȫ��Port[ID]���´λص�����ʱ��ʹ�ô�Port[ID]�Ų���
		if (dwBufSize > 0)
		{
			if (!PlayM4_SetStreamOpenMode(Port[ID], STREAME_REALTIME))  //����ʵʱ������ģʽ
			{
				break;
			}

			if (!PlayM4_OpenStream(Port[ID], pBuffer, dwBufSize, 1024 * 1024)) //�����ӿ�
			{
				break;
			}
			PlayM4_SetDecCallBack(Port[ID], DecCBFun[ID]);
			if (!PlayM4_Play(Port[ID], hWnd)) //���ſ�ʼ
			{
				break;
			}
		}
	case NET_DVR_STREAMDATA:   //��������
		if (dwBufSize > 0 && Port[ID] != -1)
		{

			if (!PlayM4_InputData(Port[ID], pBuffer, dwBufSize))
			{
				break;
			}
		}
	}

}

//�˴��ǽ���֮���YUV��ʽ�ַ���
void CALLBACK HIKPlay::DecCBFun_0(long nPort, char * pBuf, long nSize, FRAME_INFO * pFrameInfo, long nReserved1, long /*nReserved2*/) //YV12(YUV��һ��)-->IplImage
{
	int ID = 0;
	//cout << "DecCBFun_0" << endl;
	HANDLE h[2] = { g_hEmptySemaphore[ID], g_hMutex[ID] };
	if (WaitForMultipleObjects(2, h, TRUE, 10) == WAIT_TIMEOUT)
		return;
	int len = hik_width_org[ID] * hik_height_org[ID] * 1.5;
	char * imgData = new char[len];
	memcpy(imgData, pBuf, len);
	imagesData[ID][in[ID]] = imgData;

	in[ID] = (in[ID] + 1) % SIZE_OF_BUFFER;
	ReleaseMutex(g_hMutex[ID]);
	ReleaseSemaphore(g_hFullSemaphore[ID], 1, NULL);


}
void CALLBACK HIKPlay::DecCBFun_1(long nPort, char * pBuf, long nSize, FRAME_INFO * pFrameInfo, long nReserved1, long /*nReserved2*/) //YV12(YUV��һ��)-->IplImage
{
	int ID = 1;

	HANDLE h[2] = { g_hEmptySemaphore[ID], g_hMutex[ID] };
	if (WaitForMultipleObjects(2, h, TRUE, 10) == WAIT_TIMEOUT)
		return;
	int len = hik_width_org[ID] * hik_height_org[ID] * 1.5;
	char * imgData = new char[len];
	memcpy(imgData, pBuf, len);
	imagesData[ID][in[ID]] = imgData;

	in[ID] = (in[ID] + 1) % SIZE_OF_BUFFER;
	ReleaseMutex(g_hMutex[ID]);
	ReleaseSemaphore(g_hFullSemaphore[ID], 1, NULL);


}
void CALLBACK HIKPlay::DecCBFun_2(long nPort, char * pBuf, long nSize, FRAME_INFO * pFrameInfo, long nReserved1, long /*nReserved2*/) //YV12(YUV��һ��)-->IplImage
{
	int ID = 2;

	HANDLE h[2] = { g_hEmptySemaphore[ID], g_hMutex[ID] };
	if (WaitForMultipleObjects(2, h, TRUE, 10) == WAIT_TIMEOUT)
		return;
	int len = hik_width_org[ID] * hik_height_org[ID] * 1.5;
	char * imgData = new char[len];
	memcpy(imgData, pBuf, len);
	imagesData[ID][in[ID]] = imgData;

	in[ID] = (in[ID] + 1) % SIZE_OF_BUFFER;
	ReleaseMutex(g_hMutex[ID]);
	ReleaseSemaphore(g_hFullSemaphore[ID], 1, NULL);
}
void CALLBACK HIKPlay::DecCBFun_3(long nPort, char * pBuf, long nSize, FRAME_INFO * pFrameInfo, long nReserved1, long /*nReserved2*/) //YV12(YUV��һ��)-->IplImage
{
	int ID = 3;

	HANDLE h[2] = { g_hEmptySemaphore[ID], g_hMutex[ID] };
	if (WaitForMultipleObjects(2, h, TRUE, 10) == WAIT_TIMEOUT)
		return;
	int len = hik_width_org[ID] * hik_height_org[ID] * 1.5;
	char * imgData = new char[len];
	memcpy(imgData, pBuf, len);
	imagesData[ID][in[ID]] = imgData;

	in[ID] = (in[ID] + 1) % SIZE_OF_BUFFER;
	ReleaseMutex(g_hMutex[ID]);
	ReleaseSemaphore(g_hFullSemaphore[ID], 1, NULL);
}
void CALLBACK HIKPlay::DecCBFun_4(long nPort, char * pBuf, long nSize, FRAME_INFO * pFrameInfo, long nReserved1, long /*nReserved2*/) //YV12(YUV��һ��)-->IplImage
{
	int ID = 4;

	HANDLE h[2] = { g_hEmptySemaphore[ID], g_hMutex[ID] };
	if (WaitForMultipleObjects(2, h, TRUE, 10) == WAIT_TIMEOUT)
		return;
	int len = hik_width_org[ID] * hik_height_org[ID] * 1.5;
	char * imgData = new char[len];
	memcpy(imgData, pBuf, len);
	imagesData[ID][in[ID]] = imgData;

	in[ID] = (in[ID] + 1) % SIZE_OF_BUFFER;
	ReleaseMutex(g_hMutex[ID]);
	ReleaseSemaphore(g_hFullSemaphore[ID], 1, NULL);
}
void CALLBACK HIKPlay::DecCBFun_5(long nPort, char * pBuf, long nSize, FRAME_INFO * pFrameInfo, long nReserved1, long /*nReserved2*/) //YV12(YUV��һ��)-->IplImage
{
	int ID = 5;

	HANDLE h[2] = { g_hEmptySemaphore[ID], g_hMutex[ID] };
	if (WaitForMultipleObjects(2, h, TRUE, 10) == WAIT_TIMEOUT)
		return;
	int len = hik_width_org[ID] * hik_height_org[ID] * 1.5;
	char * imgData = new char[len];
	memcpy(imgData, pBuf, len);
	imagesData[ID][in[ID]] = imgData;

	in[ID] = (in[ID] + 1) % SIZE_OF_BUFFER;
	ReleaseMutex(g_hMutex[ID]);
	ReleaseSemaphore(g_hFullSemaphore[ID], 1, NULL);
}
void CALLBACK HIKPlay::DecCBFun_6(long nPort, char * pBuf, long nSize, FRAME_INFO * pFrameInfo, long nReserved1, long /*nReserved2*/) //YV12(YUV��һ��)-->IplImage
{
	int ID = 6;

	HANDLE h[2] = { g_hEmptySemaphore[ID], g_hMutex[ID] };
	if (WaitForMultipleObjects(2, h, TRUE, 10) == WAIT_TIMEOUT)
		return;
	int len = hik_width_org[ID] * hik_height_org[ID] * 1.5;
	char * imgData = new char[len];
	memcpy(imgData, pBuf, len);
	imagesData[ID][in[ID]] = imgData;

	in[ID] = (in[ID] + 1) % SIZE_OF_BUFFER;
	ReleaseMutex(g_hMutex[ID]);
	ReleaseSemaphore(g_hFullSemaphore[ID], 1, NULL);
}
void CALLBACK HIKPlay::DecCBFun_7(long nPort, char * pBuf, long nSize, FRAME_INFO * pFrameInfo, long nReserved1, long /*nReserved2*/) //YV12(YUV��һ��)-->IplImage
{
	int ID = 7;

	HANDLE h[2] = { g_hEmptySemaphore[ID], g_hMutex[ID] };
	if (WaitForMultipleObjects(2, h, TRUE, 10) == WAIT_TIMEOUT)
		return;
	int len = hik_width_org[ID] * hik_height_org[ID] * 1.5;
	char * imgData = new char[len];
	memcpy(imgData, pBuf, len);
	imagesData[ID][in[ID]] = imgData;

	in[ID] = (in[ID] + 1) % SIZE_OF_BUFFER;
	ReleaseMutex(g_hMutex[ID]);
	ReleaseSemaphore(g_hFullSemaphore[ID], 1, NULL);
}
void CALLBACK HIKPlay::DecCBFun_8(long nPort, char * pBuf, long nSize, FRAME_INFO * pFrameInfo, long nReserved1, long /*nReserved2*/) //YV12(YUV��һ��)-->IplImage
{
	int ID = 8;

	HANDLE h[2] = { g_hEmptySemaphore[ID], g_hMutex[ID] };
	if (WaitForMultipleObjects(2, h, TRUE, 10) == WAIT_TIMEOUT)
		return;
	int len = hik_width_org[ID] * hik_height_org[ID] * 1.5;
	char * imgData = new char[len];
	memcpy(imgData, pBuf, len);
	imagesData[ID][in[ID]] = imgData;

	in[ID] = (in[ID] + 1) % SIZE_OF_BUFFER;
	ReleaseMutex(g_hMutex[ID]);
	ReleaseSemaphore(g_hFullSemaphore[ID], 1, NULL);
}
void CALLBACK HIKPlay::DecCBFun_9(long nPort, char * pBuf, long nSize, FRAME_INFO * pFrameInfo, long nReserved1, long /*nReserved2*/) //YV12(YUV��һ��)-->IplImage
{
	int ID = 9;

	HANDLE h[2] = { g_hEmptySemaphore[ID], g_hMutex[ID] };
	if (WaitForMultipleObjects(2, h, TRUE, 10) == WAIT_TIMEOUT)
		return;
	int len = hik_width_org[ID] * hik_height_org[ID] * 1.5;
	char * imgData = new char[len];
	memcpy(imgData, pBuf, len);
	imagesData[ID][in[ID]] = imgData;

	in[ID] = (in[ID] + 1) % SIZE_OF_BUFFER;
	ReleaseMutex(g_hMutex[ID]);
	ReleaseSemaphore(g_hFullSemaphore[ID], 1, NULL);
}