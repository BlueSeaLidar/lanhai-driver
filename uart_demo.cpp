// 解析后的数据可以在 data_process 分析
// 请根据实际连接方式修改main函数中对应参数


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>
#include <pthread.h>
#include <stdio.h> 
#include <stdlib.h> 
#include <unistd.h> 
#include <string.h> 
#include <sys/types.h> 
#include <netinet/in.h> 
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/wait.h> 
#include <arpa/inet.h> 
#include <stdarg.h>
#include <pthread.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <errno.h>
#include <netinet/in.h> 
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <math.h>
#include <vector>

#include "data.h"

extern "C" int change_baud(int fd, int baud);

// serial port handle
int g_port = -1;

int open_serial_port(const char* port, int baudrate) 
{
       	int fd = open(port,  O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd < 0) { 
		printf("open %s error", port); 
		return -1; 
       	}
	
	int   		ret; 
	struct termios	attrs;

	tcflush(fd, TCIOFLUSH);

	/* get current attrs */
	ret = tcgetattr(fd, &attrs);
	if(ret < 0) {
		printf("get attrs failed");
		return -1;
	}

	/* set speed */
	int speed = B230400;
	//if (baudrate == 115200) speed = B115200;

	ret = cfsetispeed(&attrs, speed);//[baudrate]);  
	ret |= cfsetospeed(&attrs, speed);//[baudrate]);

	/* enable recieve and set as local line */
	attrs.c_cflag |= (CLOCAL | CREAD);

	/* set data bits */
	attrs.c_cflag &= ~CSIZE;
	attrs.c_cflag |= CS8;

	/* set parity */
	if (1) {//parity == UART_POFF) {
		attrs.c_cflag &= ~PARENB;			//disable parity
	       	attrs.c_iflag &= ~INPCK;
	} else {
		attrs.c_cflag |= (PARENB | PARODD);	//enable parity
	       	attrs.c_iflag |= INPCK;
		//if(parity == UART_PEVEN) attrs.c_cflag &= ~PARODD;
	}

	/* set stop bits */
	attrs.c_cflag &= ~CSTOPB;	// 1 stop bit
	//attrs.c_cflag |= CSTOPB;	// 2 stop bits

	// Disable Hardware flowcontrol
        attrs.c_cflag &= ~CRTSCTS;

	/* set to raw mode, disable echo, signals */
	attrs.c_lflag &= ~(ICANON | ECHO | ECHOE | IEXTEN | ISIG);

	/* set no output process, raw mode */
	attrs.c_oflag &= ~OPOST;
	attrs.c_oflag &= ~(ONLCR | OCRNL);

	/* disable CR map  */
	attrs.c_iflag &= ~(ICRNL | INLCR);
	/* disable software flow control */
	attrs.c_iflag &= ~(IXON | IXOFF | IXANY);

//	attrs.c_cc[VMIN] = 0;
//	attrs.c_cc[VTIME] = 10;

	/* flush driver buf */
	tcflush(fd, TCIFLUSH);

	/* update attrs now */
	if(tcsetattr(fd, TCSANOW, &attrs) < 0) 
	{
		close(fd);
	       	printf("tcsetattr err");
	       	return -1;
	}

	if ( change_baud(fd, baudrate) )
	{
		close(fd);
	       	printf("fail to set baudrate %d", baudrate);
	       	return -1;
	}

	return fd;
}

int uart_talk(int fd, int n, const char* cmd, 
		int nhdr, const char* hdr_str, 
		int nfetch, char* fetch)
{
	printf("send command : %s\n", cmd);
	write(fd, cmd, n);
			
	char buf[2048];
	int nr = read(fd, buf, sizeof(buf));

	while (nr < (int)sizeof(buf))
	{
		int n = read(fd, buf+nr, sizeof(buf)-nr);
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
	sprintf(path, "/tmp/%s.dat", hdr_str);
	FILE* fp = fopen(path, "wb");
	if (fp) {
		fwrite(buf, 1, sizeof(buf), fp);
		fclose(fp);
	}

	printf("read %d bytes, not found %s\n", nr, hdr_str);
	return -1;
}

int setup_lidar(int fd_uart, int unit_is_mm, int with_confidence, int resample, int with_deshadow, int with_smooth)
{
	//char buf[] = "LUUIDH";
	//write(g_port, buf, strlen(buf));
	char buf[32];
	int nr = 0;
	for (int i=0; i<300 && nr<=0; i++) { 
		usleep(10000);
		nr = read(fd_uart, buf, sizeof(buf));
	}
	if (nr <= 0) {
		printf("serial port seem not working\n");
		return -1;
	}

	if (uart_talk(fd_uart, 6, "LUUIDH", 12, "PRODUCT SN: ", 9, g_uuid) == 0) 
	{
			printf("get product SN : %s\n", g_uuid);
	}

	if (uart_talk(fd_uart, 6, unit_is_mm == 0 ? "LMDCMH" : "LMDMMH", 
				10, "SET LiDAR ", 9, buf) == 0)
	{
		printf("set LiDAR unit to %s\n", buf);
	}

	if (uart_talk(fd_uart, 6, with_confidence == 0 ? "LNCONH" : "LOCONH", 
				6, "LiDAR ", 5, buf) == 0)
	{
		printf("set LiDAR confidence to %s\n", buf);
	}

	if (uart_talk(fd_uart, 6, with_deshadow == 0 ? "LFFF0H" : "LFFF1H", 
				6, "LiDAR ", 5, buf) == 0)
	{
		printf("set deshadow to %d\n", with_deshadow);
	}

	if (uart_talk(fd_uart, 6, with_smooth == 0 ? "LSSS0H" : "LSSS1H", 
				6, "LiDAR ", 5, buf) == 0)
	{
		printf("set smooth to %d\n", with_smooth);
	}

	if (resample == 0)
		strcpy(buf, "LSRES:000H");
	else if (resample == 1)
		strcpy(buf, "LSRES:001H");
	else if (resample > 100 && resample < 1000)
		sprintf(buf, "LSRES:%03dH", resample);
	else
		buf[0] = 0;

	if (buf[0]) {
		char buf2[32];
		if (uart_talk(fd_uart, 10, buf, 15, "set resolution ", 1, buf2) == 0)
		{
			printf("set LiDAR resample to %d\n", resample);
		}
	}
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
	int fd_uart = open_serial_port(port, baud_rate);
	if (fd_uart < 0) {
		printf("Open port error ");
		return -1;
	}
	g_port = fd_uart;

	// 
	setup_lidar(fd_uart, unit_is_mm, with_confidence, resample, with_deshadow, with_smooth);

	unsigned char* buf = new unsigned char[BUF_SIZE];
	int buf_len = 0;

	FILE* fp_rec = NULL;// fopen("/tmp/rec.dat", "ab");

	int fan_span = 360;
	while (1)
	{ 
		fd_set fds;
		FD_ZERO(&fds); 

		FD_SET(fd_uart, &fds); 

		struct timeval to = { 1, 1 };
		int ret = select(fd_uart+1, &fds, NULL, NULL, &to); 

		if (ret == 0) 
		{
			printf("read data timeout\n");
			continue;
		}
		
		if (ret < 0) {
			printf("select error\n");
			return -1;
		}

		int new_data = -1;
		if (fd_uart > 0 && FD_ISSET(fd_uart, &fds)) 
		{
			int nr = read(fd_uart, buf+buf_len, BUF_SIZE - buf_len);
			if (nr <= 0) {
				printf("read port %d error %d\n", buf_len, nr);
				break;
			}

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
			if (unit_is_mm）// && with_confidence)
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
					FILE* fp = fopen("/tmp/bad.dat", "ab");
					if (fp) {
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

