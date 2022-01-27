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
#include "data.h"

int uart_talk(int fd, int n, const char* cmd, 
		int nhdr, const char* hdr_str, 
		int nfetch, char* fetch)
{
	return -1;
}

int main(int argc, char **argv)
{
	int with_chk = 1; 		// 使能数据校验

	if (argc < 8) {
		printf("usage : ./lidar 雷达地址 端口 单位是毫米 数据中带有强度 分辨率[0,1,200,225,250,300,333...] 去拖点 平滑\n");
		return -1;
	}
	
	char dev_ip[256]; 	// 雷达的网络地址
	strcpy(dev_ip, argv[1]);
	int tcp_port = atoi(argv[2]); 		// 雷达的TCP端口
	int unit_is_mm = atoi(argv[3]); 	// 数据是毫米为单位,厘米时为0
	int with_confidence = atoi(argv[4]); 	// 数据中带有强度
	int resample = atoi(argv[5]); // 分辨率，0：原始数据，1：角度修正数据，200：0.2°，333：0.3°。。。。
	int with_deshadow = atoi(argv[6]); // 去拖点，0：关闭，1：开启
	int with_smooth = atoi(argv[7]); // 数据平滑， 0：关闭， 1：开启
	
	int fd_tcp = -1;

	unsigned char* buf = new unsigned char[BUF_SIZE];
	int buf_len = 0;

	FILE* fp_rec = NULL;// fopen("/tmp/rec.dat", "ab");

	bool should_publish = false;
	int fan_span = 360;
	while (1)
	{ 
		if (fd_tcp < 0) 
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

		if (fd_tcp > 0) 
		{
			FD_SET(fd_tcp, &fds); 
			if (fd_max < fd_tcp) fd_max = fd_tcp;
		}
		
		struct timeval to = { 1, 1 };
	    int ret = select(fd_max+1, &fds, NULL, NULL, &to); 

		if (ret == 0) 
		{
			printf("read data timeout\n");
			if (fd_tcp > 0) {
				close(fd_tcp);
				fd_tcp = -1;
			}
			continue;
		}
		
		if (ret < 0) {
			printf("select error\n");
			return -1;
		}

		int new_data = -1;

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

