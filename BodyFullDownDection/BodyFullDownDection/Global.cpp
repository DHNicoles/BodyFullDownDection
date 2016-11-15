#include	"Global.h"

using namespace std;

string ConfigPath;


int CreatDir(const char *pDir)
{
	int i = 0;
	int iRet;
	int iLen;
	char* pszDir;

	if (NULL == pDir)
	{
		return 0;
	}

	pszDir = _strdup(pDir);
	iLen = strlen(pszDir);

	// �����м�Ŀ¼
	for (i = 0; i < iLen; i++)
	{
		if (pszDir[i] == '\\' || pszDir[i] == '/' || pszDir[i] == '//')
		{
			pszDir[i] = '\0';

			//���������,����
			iRet = ACCESS(pszDir, 0);

			if (iRet != 0)
			{
				iRet = MKDIR(pszDir);
				if (iRet != 0)
				{
					return -1;
				}
			}
			//֧��linux,������\����/
			pszDir[i] = '/';
		}
	}

	iRet = MKDIR(pszDir);
	free(pszDir);
	return iRet;
}