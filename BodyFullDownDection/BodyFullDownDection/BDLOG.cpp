#include	"BDLOG.h"
#include	"Global.h"
#include	<string>
using namespace std;
void BDLOG(const char * strLog){
	string path = ConfigPath + "/monitor_Personnel_Safety.log";
	std::ofstream outfile(path, std::ios::app);
	if (!outfile){
		exit(0);
	}
	time_t stime;
	time(&stime);
	string SysTime = ::ctime(&stime);
	SysTime.resize(SysTime.length() - 1);
	outfile << SysTime << "   "<<strLog  <<  std::endl;
}