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

extern "C" int change_baud(int fd, int baud);

//#include <linux/termios.h>
struct RawDataHdr
{
	unsigned short code;
	unsigned short N;
	unsigned short angle;
};

struct RawData
{
	unsigned short code;
	unsigned short N;
	unsigned short angle;
	unsigned short distance[1000];
	unsigned char confidence[1000];
};

struct DataPoint
{
	float angle; // 弧度
	float distance; // 米
};


#define PI 3.14159265

#define HDR_SIZE 6

#define BUF_SIZE 512*1024

struct CmdHeader
{
	unsigned short sign;
	unsigned short cmd;
	unsigned short sn;
	unsigned short len;
};

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

char g_uuid[32] = "";
// translate lidar raw data to ROS laserscan message
bool parse_data_3(int len, unsigned char* buf, RawData& dat, int& consume, int with_chk) 
{
	int idx = 0;
	while (idx < len-18)
	{
		if (buf[idx] == 'S' && buf[idx+1] == 'T' && buf[idx+6] == 'E' && buf[idx+7] == 'D')
		{
			idx += 8;
		}
		if (buf[idx] != 0xce || buf[idx+1] != 0xfa)
		{
			idx++;
			continue;
		}
	
		if (idx > 24) for (int i=0; i<idx-22; i++) 
		{
			// get product SN
			if (memcmp(buf+i, "PRODUCT SN: ", 12) == 0)
		       	{
				memcpy(g_uuid, buf+i+12, 9);
				g_uuid[9] = 0;
				printf("found product SN : %s\n", g_uuid);
			}
		}

		RawDataHdr hdr;
		memcpy(&hdr, buf+idx, HDR_SIZE);

		if ((hdr.angle % 360) != 0) 
		{
			printf("bad angle %d\n", hdr.angle);
			idx += 2;
			continue; 
		}

		if (hdr.N > 300 || hdr.N < 30) 
		{
			printf("points number %d seem not correct\n", hdr.N);
			idx += 2;
			continue;
		}

		if (idx + HDR_SIZE + hdr.N*3 + 2 > len)
		{
			// data packet not complete
			break;
		}

		// calc checksum
		unsigned short sum = hdr.angle + hdr.N, chk;
		unsigned char* pdat = buf+idx+HDR_SIZE;
		for (int i=0; i<hdr.N; i++)
		{
			dat.confidence[i] = *pdat++;
			sum += dat.confidence[i];

			unsigned short v = *pdat++;
			unsigned short v2 = *pdat++;
			dat.distance[i] = (v2<<8) | v;

			sum += dat.distance[i];
		}

		memcpy(&chk, pdat, 2);

		if (with_chk != 0 && chk != sum) 
		{
			printf("chksum3 error");
			consume = idx + HDR_SIZE + 3*hdr.N + 2;
			return 0;
		}

		memcpy(&dat, &hdr, HDR_SIZE);
		//memcpy(dat.data, buf+idx+HDR_SIZE, 2*hdr.N);
		//printf("get3 %d(%d)\n", hdr.angle, hdr.N);
		
		idx += HDR_SIZE + 3*hdr.N + 2;
		consume = idx;
		return true;
	}

	if (idx > 1024) consume = idx/2;
	return false;
}

bool parse_data(int len, unsigned char* buf, RawData& dat, int is_mm, int with_conf, int& consume, int with_chk) 
{
	int idx = 0;
	while (idx < len-180)
	{
		if (buf[idx] == 'S' && buf[idx+1] == 'T' && buf[idx+6] == 'E' && buf[idx+7] == 'D')
		{
			idx+=8;
		}
		if (buf[idx] != 0xce || buf[idx+1] != 0xfa)
		{
			idx++;
			continue;
		}
			
		if (idx > 24) for (int i=0; i<idx-22; i++) 
		{
			// get product SN
			if (memcmp(buf+i, "PRODUCT SN: ", 12) == 0)
		       	{
				memcpy(g_uuid, buf+i+12, 9);
				g_uuid[9] = 0;
				printf("found product SN : %s\n", g_uuid);
			}
		}

		RawDataHdr hdr;
		memcpy(&hdr, buf+idx, HDR_SIZE);

		if ((hdr.angle % 360) != 0) 
		{
			printf("bad angle %d\n", hdr.angle);
			idx += 2;
			continue; 
		}

		if (hdr.N > 300 || hdr.N < 30) 
		{
			printf("points number %d seem not correct\n", hdr.N);
			idx += 2;
			continue;
		}

		if (idx + HDR_SIZE + hdr.N*sizeof(short) + 2 > len)
		{
			// data packet not complete
			break;
		}

		// calc checksum
		unsigned short sum = hdr.angle + hdr.N, chk;
		unsigned char* pdat = buf+idx+HDR_SIZE;
		for (int i=0; i<hdr.N; i++)
		{
			unsigned short v = *pdat++;
			unsigned short v2 = *pdat++;
			unsigned short val = (v2<<8) | v;

			if (with_conf)
			{
				dat.confidence[i] = val >> 13;
			       	dat.distance[i] = val & 0x1fff;
				if (is_mm == 0) dat.distance[i] *= 10;
			} else {
				dat.confidence[i] = is_mm ? val : val*10;
				dat.confidence[i] = 0;
			}

			sum += val;
		}
		memcpy(&chk, buf+idx+HDR_SIZE+hdr.N*2, 2);

		if (with_chk != 0 && chk != sum) 
		{
			printf("chksum error");
			consume = idx + HDR_SIZE + 2*hdr.N + 2;
			return 0;
		}

		memcpy(&dat, &hdr, HDR_SIZE);
		// memcpy(dat.data, buf+idx+HDR_SIZE, 2*hdr.N);

		idx += HDR_SIZE + 2*hdr.N + 2;
		consume = idx;
		return true;
	}

	if (idx > 1024) consume = idx/2;
	return false;
}

int uart_talk(int fd, int n, const char* cmd, 
		int nhdr, const char* hdr_str, 
		int nfetch, char* fetch)
{
	printf("send command : %s\n", cmd);
	write(fd, cmd, n);
			
	char buf[1024];
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


void data_process(int n, DataPoint* points)
{
	printf("360°数据点数 %d\n", n);
}

RawData dat360[10];
// 每次获得36°扇区数据
void data_process(RawData* data)
{
	// int mi = 100000;
	// for (int i=0; i<data->N; i++) {
	// if (data->distance[i] > 0 && mi > data->distance[i])
	// mi = data->distance[i];
	// }
	//printf("角度 %d, 数据点数 %d\n", data->angle/10, data->N);

	dat360[data->angle/360] = *data;
	if (data->angle != 3600-360) return ;
	
	int count = 0, n = 0;
	for (int i=0; i<10; i++) {
		if (dat360[i].N > 0)
			n++;
	       	count += dat360[i].N;
	}
	if (n != 10) return;

	DataPoint* points = new DataPoint[count];
	count = 0;
	for (int i=0; i<10; i++) 
	{
		for (int j=0; j<dat360[i].N; j++)
		{
			points[count].angle = (36*i + j*36.0/dat360[i].N) * PI / 360;
			points[count].distance = dat360[i].distance[j] / 1000.0;
			count++;
		}
		dat360[i].N = 0;
	}
	data_process(count, points);
	delete points;
}

int main(int argc, char **argv)
{
	const char* type = "uart"; 		// 连接方式："uart" 串口， "tcp" 网络方法
	const char* dev_ip = "192.168.158.91"; 	// 雷达的网络地址
	int tcp_port = 5000; 			// 雷达的TCP端口
	int with_chk = 1; 		// 使能数据校验

	if (argc < 5) {
		printf("usage : ./lidar 串口名称 波特率 单位是毫米 数据中带有强度\n");
		return -1;
	}
	
	char* port = argv[1];			//串口名称 "/dev/ttyUSB0"; 
	int baud_rate = atoi(argv[2]); 		// 串口波特率
	int unit_is_mm = atoi(argv[3]); 	// 数据是毫米为单位,厘米时为0
	int with_confidence = atoi(argv[4]); 	// 数据中带有强度
	
	//int mirror =  0;
	//int from_zero = 0;
	//int angle_patch = 1;
	
	// open serial port
	int fd_uart = -1, fd_tcp = -1;

	if (type == "uart") {
		fd_uart = open_serial_port(port, baud_rate);
		if (fd_uart < 0) {
			printf("Open port error ");
			return -1;
		}
		g_port = fd_uart;

		//send UUID reading request 
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
	}

	unsigned char* buf = new unsigned char[BUF_SIZE];
	int buf_len = 0;

	memset(dat360, 0, sizeof(RawData)*10);

	FILE* fp_rec = NULL;// fopen("/tmp/rec.dat", "ab");

	bool should_publish = false;
	while (1)
	{ 
		if (type == "tcp" && fd_tcp < 0) 
		{
			// open TCP port
			int sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
			if (sock < 0) { printf("socket TCP failed\n"); return 0; }

			struct sockaddr_in addr;
			memset(&addr, 0, sizeof(addr));     /* Zero out structure */
			addr.sin_family      = AF_INET;             /* Internet address family */

			addr.sin_addr.s_addr = inet_addr(dev_ip);
		       	addr.sin_port = htons(tcp_port);

			int ret = connect(sock, (struct sockaddr *) &addr, sizeof(addr)); 
			
			if (ret != 0) 
			{
				printf("connect (%s:%d) failed", dev_ip, tcp_port);
			       	close(sock); 
				sleep(15);
			       	continue;
		       	}
			fd_tcp = sock;
			printf("connect (%s:%d) ok", dev_ip, tcp_port);
		}

		fd_set fds;
	       	FD_ZERO(&fds); 

		int fd_max = -1;
		if (fd_uart > 0) 
		{
			FD_SET(fd_uart, &fds); 
			if (fd_max < fd_uart) fd_max = fd_uart;
		}

		if (fd_tcp > 0) 
		{
			FD_SET(fd_tcp, &fds); 
			if (fd_max < fd_tcp) fd_max = fd_tcp;
		}
		
		
		struct timeval to = { 1, 1 };
	       	int ret = select(fd_max+1, &fds, NULL, NULL, &to); 

		if (ret == 0) 
		{
			printf("read data timeout");
			if (fd_tcp > 0) {
				close(fd_tcp);
				fd_tcp = -1;
			}
			continue;
		}
		
		if (ret < 0) {
			printf("select error");
			return -1;
		}

		int new_data = -1;
		if (fd_uart > 0 && FD_ISSET(fd_uart, &fds)) 
		{
			int nr = read(fd_uart, buf+buf_len, BUF_SIZE - buf_len);
			if (nr <= 0) {
				printf("read port %d error %d", buf_len, nr);
				break;
			}

			if (nr == 0) continue;
			if (nr > 0 && fp_rec) { 
				fwrite(buf+buf_len, 1, nr, fp_rec);
				fflush(fp_rec);
		       	}

			new_data = nr;
		}

		if (fd_tcp > 0 && FD_ISSET(fd_tcp, &fds)) 
		{
			int nr = recv(fd_tcp,  buf+buf_len, BUF_SIZE - buf_len, 0);
			if (nr < 0) {
				printf("tcp error");
				close(fd_tcp);
				fd_tcp = -1;
				continue;
			}

			new_data = nr;
		}

		if (new_data > 0)
		{
			buf_len += new_data;

			int consume = 0; 
			RawData dat;
			bool is_pack;
			if (unit_is_mm && with_confidence)
				is_pack = parse_data_3(buf_len, buf, dat, consume, with_chk);
			else
				is_pack = parse_data(buf_len, buf, dat, 
						unit_is_mm, with_confidence, consume,
						with_chk);
			if (is_pack)
			{
				data_process(&dat);
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

