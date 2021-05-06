// 解析后的数据可以在 data_process 分析
// 请根据实际连接方式修改main函数中对应参数


#include <stdio.h>
#include <stdlib.h>
#include <string.h> 
#include <sys/types.h> 
#include <errno.h>
#include <math.h>
#include <vector>
#include <windows.h>

#include "data.h"

// serial port handle

int uart_talk(HANDLE hCom, int n, const char* cmd, 
		int nhdr, const char* hdr_str, 
		int nfetch, char* fetch)
{
	printf("send command : %s\n", cmd);
	DWORD dw;
	WriteFile(hCom, cmd, n, &dw, NULL);
			
	char buf[2048];
	DWORD nr = 0;
	ReadFile(hCom, buf, sizeof(buf), &nr, NULL);

	while (nr < (int)sizeof(buf))
	{
		DWORD n;
		ReadFile(hCom, buf + nr, sizeof(buf) - nr, &n, NULL);
		if (n > 0) nr += n;
	}

	for (int i=0; i<(int)sizeof(buf)-nhdr-nfetch; i++) 
	{
		if (memcmp(buf+i, hdr_str, nhdr) == 0) 
		{
			memcpy(fetch, buf+i+nhdr, nfetch);
			fetch[nfetch] = 0;
			return 0;
		}
	}

	char path[256];
	sprintf_s(path, 250, "./tmp/%s.dat", hdr_str);
	FILE* fp;
	if (fopen_s(&fp, path, "wb") == 0)
	{
		fwrite(buf, 1, sizeof(buf), fp);
		fclose(fp);
	}

	printf("read %d bytes, not found %s\n", nr, hdr_str);
	return -1;
}

int setup_lidar(HANDLE hCom, int unit_is_mm, int with_confidence, int resample, int with_deshadow, int with_smooth)
{
	//char buf[] = "LUUIDH";
	//write(g_port, buf, strlen(buf));
	char buf[32];
	DWORD nr = 0;
	for (int i=0; i<300 && nr<=0; i++) { 
		Sleep(10);

		ReadFile(hCom, buf, sizeof(buf), &nr, NULL);
	}
	if (nr <= 0) {
		printf("serial port seem not working\n");
		return -1;
	}

	if (uart_talk(hCom, 6, "LUUIDH", 12, "PRODUCT SN: ", 9, g_uuid) == 0) 
	{
			printf("get product SN : %s\n", g_uuid);
	}

	if (uart_talk(hCom, 6, unit_is_mm == 0 ? "LMDCMH" : "LMDMMH", 
				10, "SET LiDAR ", 9, buf) == 0)
	{
		printf("set LiDAR unit to %s\n", buf);
	}

	if (uart_talk(hCom, 6, with_confidence == 0 ? "LNCONH" : "LOCONH", 
				6, "LiDAR ", 5, buf) == 0)
	{
		printf("set LiDAR confidence to %s\n", buf);
	}

	if (uart_talk(hCom, 6, with_deshadow == 0 ? "LFFF0H" : "LFFF1H", 
				6, "LiDAR ", 5, buf) == 0)
	{
		printf("set deshadow to %d\n", with_deshadow);
	}

	if (uart_talk(hCom, 6, with_smooth == 0 ? "LSSS0H" : "LSSS1H", 
				6, "LiDAR ", 5, buf) == 0)
	{
		printf("set smooth to %d\n", with_smooth);
	}

	if (resample == 0)
		strcpy_s(buf, 30, "LSRES:000H");
	else if (resample == 1)
		strcpy_s(buf, 30, "LSRES:001H");
	else if (resample > 100 && resample < 1000)
		sprintf_s(buf, 30, "LSRES:%03dH", resample);
	else
		buf[0] = 0;

	if (buf[0]) {
		char buf2[32];
		if (uart_talk(hCom, 10, buf, 15, "set resolution ", 1, buf2) == 0)
		{
			printf("set LiDAR resample to %d\n", resample);
		}
	}

	return 0;
}

HANDLE OpenPort(const char* name, int speed)
{
	char path[32];
	sprintf_s(path, 30, "\\\\.\\%s", name);
	// Open the serial port.
	HANDLE hPort = CreateFile(path,
		GENERIC_READ | GENERIC_WRITE, // Access (read-write) mode
		0,            // Share mode
		NULL,         // Pointer to the security attribute
		OPEN_EXISTING,// How to open the serial port
		0,            // Port attributes
		NULL);        // Handle to port with attribute

	if (hPort == NULL || hPort == INVALID_HANDLE_VALUE)
	{
		//MessageBox(0, "can not open port", name, MB_OK);
		return 0;
	}
	DCB PortDCB;
	// Initialize the DCBlength member. 
	PortDCB.DCBlength = sizeof(DCB);
	// Get the default port setting information.
	GetCommState(hPort, &PortDCB);

	// Change the DCB structure settings.
	PortDCB.BaudRate = speed;// 115200;              // Current baud 
	PortDCB.fBinary = TRUE;               // Binary mode; no EOF check 
	PortDCB.fParity = TRUE;               // Enable parity checking 
	PortDCB.fOutxCtsFlow = FALSE;         // No CTS output flow control 
	PortDCB.fOutxDsrFlow = FALSE;         // No DSR output flow control 
	PortDCB.fDtrControl = DTR_CONTROL_ENABLE;
	// DTR flow control type 
	PortDCB.fDsrSensitivity = FALSE;      // DSR sensitivity 
	PortDCB.fTXContinueOnXoff = TRUE;     // XOFF continues Tx 
	PortDCB.fOutX = FALSE;                // No XON/XOFF out flow control 
	PortDCB.fInX = FALSE;                 // No XON/XOFF in flow control 
	PortDCB.fErrorChar = FALSE;           // Disable error replacement 
	PortDCB.fNull = FALSE;                // Disable null stripping 
	PortDCB.fRtsControl = RTS_CONTROL_ENABLE;
	// RTS flow control 
	PortDCB.fAbortOnError = FALSE;        // Do not abort reads/writes on 
										  // error
	PortDCB.ByteSize = 8;                 // Number of bits/byte, 4-8 
	PortDCB.Parity = NOPARITY;            // 0-4=no,odd,even,mark,space 
	PortDCB.StopBits = ONESTOPBIT;        // 0,1,2 = 1, 1.5, 2 

										  // Configure the port according to the specifications of the DCB 
										  // structure.
	if (!SetCommState(hPort, &PortDCB))
	{
		//MessageBox(0, "Unable to configure the serial port", "error", MB_OK);
		CloseHandle(hPort);
		return NULL;
	}
	// Retrieve the timeout parameters for all read and write operations
	// on the port. 
	COMMTIMEOUTS CommTimeouts;
	GetCommTimeouts(hPort, &CommTimeouts);

	// Change the COMMTIMEOUTS structure settings.
	CommTimeouts.ReadIntervalTimeout = MAXDWORD;
	CommTimeouts.ReadTotalTimeoutMultiplier = 0;
	CommTimeouts.ReadTotalTimeoutConstant = 0;
	CommTimeouts.WriteTotalTimeoutMultiplier = 0;//10;  
	CommTimeouts.WriteTotalTimeoutConstant = 0;//1000;    

											   // Set the timeout parameters for all read and write operations
											   // on the port. 
	if (!SetCommTimeouts(hPort, &CommTimeouts))
	{
		// Could not set the timeout parameters.
		//MessageBox(0, "Unable to set the timeout parameters", "error", MB_OK);
		CloseHandle(hPort);
		return NULL;
	}

	return hPort;
}

int main(int argc, char **argv)
{
	int with_chk = 1; 		// 使能数据校验

	if (argc < 8) {
		printf("usage : ./lidar 串口名称 波特率 单位是毫米 数据中带有强度 分辨率[0,1,200,225,250,300,333...] 去拖点 平滑\n");
		return -1;
	}
	
	char* port = argv[1];			//串口名称 "/dev/ttyUSB0"; 

	int baud_rate = atoi(argv[2]); 		// 串口波特率

	int unit_is_mm = atoi(argv[3]); 	// 数据是毫米为单位,厘米时为0

	int with_confidence = atoi(argv[4]); 	// 数据中带有强度

	int resample = atoi(argv[5]); // 分辨率，0：原始数据，1：角度修正数据，200：0.2°，333：0.3°。。。。

	int with_deshadow = atoi(argv[6]); // 去拖点，0：关闭，1：开启

	int with_smooth = atoi(argv[7]); // 数据平滑， 0：关闭， 1：开启

	
	// open serial port
	HANDLE hCom = OpenPort(port, baud_rate);
	if (hCom == NULL) {
		printf("Open port error ");
		return -1;
	}

	// 
	setup_lidar(hCom, unit_is_mm, with_confidence, resample, with_deshadow, with_smooth);

	unsigned char* buf = new unsigned char[BUF_SIZE];
	int buf_len = 0;

	FILE* fp_rec = NULL;// fopen("/tmp/rec.dat", "ab");

	int fan_span = 360;
	while (1)
	{ 
		int new_data = -1;

		DWORD nr = 0;
		if (ReadFile(hCom, buf+buf_len, BUF_SIZE - buf_len, &nr, NULL))
		{
			if (nr == 0) continue;
			if (nr > 0 && fp_rec) { 
				fwrite(buf+buf_len, 1, nr, fp_rec);
				fflush(fp_rec);
			}

			new_data = nr;
		}

		if (new_data > 0)
		{
			buf_len += new_data;

			int consume = 0; 
			RawData dat;
			bool is_pack;
			if (unit_is_mm)// && with_confidence)
			{
				is_pack = parse_data_x(buf_len, buf, 
					fan_span,unit_is_mm, with_confidence,
					dat, consume, with_chk);
			}
			else {
				is_pack = parse_data(buf_len, buf, 
					fan_span, unit_is_mm, with_confidence, 
					dat, consume, with_chk);
			}
			if (is_pack)
			{
				data_process(dat);
			}

			if (consume > 0) 
			{
				if (!is_pack) {
					FILE* fp;
					if (fopen_s(&fp, "/tmp/bad.dat", "ab") == 0)
					{
						fwrite(buf, 1, consume, fp);
						fclose(fp);
					}
					printf("drop %d bytes: %02x %02x %02x %02x %02x %02x", 
							consume,
							buf[0], buf[1], buf[2],
						       	buf[3], buf[4], buf[5]);
				}


				for (int i=consume; i<buf_len; i++) 
					buf[i - consume] = buf[i];
				buf_len -= consume; 
			}
		}
	}

	//close(fd);
	return 0;
}

