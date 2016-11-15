#include	<iostream>
#include	<string>
#include	"CameraCtrl.h"

using namespace std;
#define CAMERA


//void main(){
//#ifdef CAMERA
//	string IPstr("10.103.241.189"), userID("admin"), psw("632911632");
//
//	long port = 8000;
//	string confpath = "H://abc";
//	HIKPlay hik1(IPstr, userID, psw, port, confpath);
//
//	HIKPlay hik2(IPstr, userID, psw, port, confpath);
//	cout << hik1.getLastResult() << endl;
//	cout << hik2.getLastResult() << endl;
//	if (hik1.start()&& hik2.start()){
//		//让主线程阻塞
//
//		int i = 0;
//		while (i++ < 10000){
//			cout << " CHN 1-- " << i << " = " << hik1.getLastResult() << endl;
//			cout << " CHN 2-- " << i << " = " << hik2.getLastResult() << endl;
//			Sleep(2000);
//		}
//		hik1.stop();
//		hik2.stop();
//	}
//	if (hik1.start() && hik2.start()){
//		//让主线程阻塞
//
//		int i = 0;
//		while (i++ < 500){
//			cout << " CHN 1-- " << i << " = " << hik1.getLastResult() << endl;
//			cout << " CHN 2-- " << i << " = " << hik2.getLastResult() << endl;
//			Sleep(2000);
//		}
//		hik1.stop();
//		hik2.stop();
//	}
//	if (hik1.start() && hik2.start()){
//		//让主线程阻塞
//
//		int i = 0;
//		while (i++ < 5){
//			cout << " CHN 1-- " << i << " = " << hik1.getLastResult() << endl;
//			cout << " CHN 2-- " << i << " = " << hik2.getLastResult() << endl;
//			Sleep(2000);
//		}
//		hik1.stop();
//		hik2.stop();
//	}
//	if (hik1.start() && hik2.start()){
//		//让主线程阻塞
//		int i = 0;
//		while (i++ < 10){
//			cout << " CHN 1-- " << i << " = " << hik1.getLastResult() << endl;
//			cout << " CHN 2-- " << i << " = " << hik2.getLastResult() << endl;
//			Sleep(1000);
//		}
//		hik1.stop();
//		hik2.stop();
//	}
//	cout << hik1.getLastResult() << endl;
//	if (hik1.start()){
//		//让主线程阻塞
//		int i = 0;
//		while (i++ < 20){
//			Sleep(1000);
//			cout << i << " = " << hik1.getLastResult() << endl;
//		}
//		hik1.stop();
//	}
//#else
//	using std::cout;
//	string filename = "J://right//1.mp4";
//	HIKPlay hik(filename.c_str(), string("c://test"));
//	if (hik.start()){
//		//让主线程阻塞
//		int i = 0;
//		Sleep(-1);
//	}
//	hik.stop();
//#endif
//}
string IPstr("10.103.241.189"), userID("admin"), psw("632911632");
//
//	long port = 8000;
//	string confpath = "H://abc";
//	HIKPlay hik1(IPstr, userID, psw, port, confpath);
void main(int argc, char* argv[]){
	string IP, userID, psw, portStr, confpath;
	cout << "Tips:\n\tEnter:\n\tIPadd  username  password  port  file-path\n\tExample:\n\t10.103.241.189  admin  12345  8000  c://monitor" << endl;
	//cin >> IP >> userID >> psw >> portStr >> confpath;
	IP = "10.103.241.191", userID = "admin", psw = "632911632", portStr = "8000", confpath = "j://monitor";
	long port = atoi(portStr.c_str());
	HIKPlay hik(IPstr, userID, psw, port, confpath);
	if (hik.start()){
		//让主线程阻塞
		Sleep(-1);
	}

}
