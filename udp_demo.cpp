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

struct CmdHeader
{
	unsigned short sign;
	unsigned short cmd;
	unsigned short sn;
	unsigned short len;
};
struct KeepAlive {
	uint32_t world_clock;
	uint32_t mcu_hz;
	uint32_t arrive;
	uint32_t delay;
	uint32_t reserved[4];
};
// CRC32
unsigned int stm32crc(unsigned int *ptr, unsigned int len)
{
	unsigned int xbit, data;
	unsigned int crc32 = 0xFFFFFFFF;
	const unsigned int polynomial = 0x04c11db7;

	for (unsigned int i = 0; i < len; i++)
	{
		xbit = 1 << 31;
		data = ptr[i];
		for (unsigned int bits = 0; bits < 32; bits++)
		{
			if (crc32 & 0x80000000)
			{
				crc32 <<= 1;
				crc32 ^= polynomial;
			}
			else
				crc32 <<= 1;

			if (data & xbit)
				crc32 ^= polynomial;

			xbit >>= 1;
		}
	}
	return crc32;
}

// 
bool send_cmd_udp_f(int fd_udp, const char* dev_ip, int dev_port,
	       	int cmd, int sn, 
		int len, const void* snd_buf, bool bpr)
{
	char buffer[2048];
	CmdHeader* hdr = (CmdHeader*)buffer;
	hdr->sign = 0x484c;
	hdr->cmd = cmd;
	hdr->sn = sn;

	len = ((len + 3) >> 2) * 4;

	hdr->len = len;

	memcpy(buffer + sizeof(CmdHeader), snd_buf, len);

	int n = sizeof(CmdHeader);
	unsigned int* pcrc = (unsigned int*)(buffer + sizeof(CmdHeader) + len);
	pcrc[0] = stm32crc((unsigned int*)(buffer + 0), len/4 + 2);

	sockaddr_in to;
	to.sin_family = AF_INET;
	to.sin_addr.s_addr = inet_addr(dev_ip);
	to.sin_port = htons(dev_port);

	int len2 = len + sizeof(CmdHeader) + 4;

	sendto(fd_udp, buffer, len2, 0, (struct sockaddr*)&to, sizeof(struct sockaddr));

	if (bpr) {
		char s[3096];
		for (int i = 0; i < len2; i++) 
			sprintf(s + 3 * i, "%02x ", (unsigned char)buffer[i]);

		printf("send to %s:%d 0x%04x sn[%d] L=%d : %s\n", 
				dev_ip, dev_port, cmd, sn, len, s);
	}
	return true;
}

bool send_cmd_udp(int fd_udp, const char* dev_ip, int dev_port,
	       	int cmd, int sn, 
		int len, const void* snd_buf)
{
	return send_cmd_udp_f(fd_udp, dev_ip, dev_port, cmd, sn, len, snd_buf, true);
}

char lidar_ip[256];
int lidar_port = 5000;

bool udp_talk(int fd_udp, 
		 int n, const char* cmd, 
		 int nhdr, const char* hdr_str, 
		 int nfetch, char* fetch)
{
	printf("send command : \'%s\' \n", cmd);

	unsigned short sn = rand();
	int rt = send_cmd_udp(fd_udp, lidar_ip, lidar_port, 0x0043, sn, n, cmd);

	int nr = 0;
	for (int i=0; i<100; i++)
	{
		fd_set fds;
		FD_ZERO(&fds); 

		FD_SET(fd_udp, &fds); 
	
		struct timeval to = { 1, 0 }; 
		int ret = select(fd_udp+1, &fds, NULL, NULL, &to); 

		if (ret <= 0) {
			return false;
		}
		
		// read UDP data
		if (FD_ISSET(fd_udp, &fds)) 
		{ 
			nr++;
			sockaddr_in addr;
			socklen_t sz = sizeof(addr);
	
			char buf[1024] = {0};
			int nr = recvfrom(fd_udp, buf, sizeof(buf), 0, (struct sockaddr *)&addr, &sz);
			if (nr > 0) 
			{	
				CmdHeader* hdr = (CmdHeader*)buf;
				if (hdr->sign != 0x484c || hdr->sn != sn) continue;
					
				for (int i=0; i<nr-nhdr-nfetch; i++) 
				{
					if (memcmp(buf+i, hdr_str, nhdr) == 0) 
					{ 
						memcpy(fetch, buf+i+nhdr, nfetch);
					       	fetch[nfetch] = 0;
					       	return true;
				       	}
			       	}

				memcpy(fetch, "ok", 2);
				fetch[2] = 0;
				return true;
			}
		}
	}

	printf("read %d packets, not response\n", nr);
	return false;
}

int setup_lidar(int fd_udp, int unit_is_mm, int with_confidence, int resample, int with_deshadow, int with_smooth)
{
	//char buf[] = "LUUIDH";
	//write(g_port, buf, strlen(buf));
	char buf[32];
	int nr = 0;
	
	if (udp_talk(fd_udp, 6, "LUUIDH", 12, "PRODUCT SN: ", 9, g_uuid) == 0) 
	{
			printf("get product SN : %s\n", g_uuid);
	}

	if (udp_talk(fd_udp, 6, unit_is_mm == 0 ? "LMDCMH" : "LMDMMH", 
				10, "SET LiDAR ", 9, buf) == 0)
	{
		printf("set LiDAR unit to %s\n", buf);
	}

	if (udp_talk(fd_udp, 6, with_confidence == 0 ? "LNCONH" : "LOCONH", 
				6, "LiDAR ", 5, buf) == 0)
	{
		printf("set LiDAR confidence to %s\n", buf);
	}

	if (udp_talk(fd_udp, 6, with_deshadow == 0 ? "LFFF0H" : "LFFF1H", 
				6, "LiDAR ", 5, buf) == 0)
	{
		printf("set deshadow to %d\n", with_deshadow);
	}

	if (udp_talk(fd_udp, 6, with_smooth == 0 ? "LSSS0H" : "LSSS1H", 
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
		if (udp_talk(fd_udp, 10, buf, 15, "set resolution ", 1, buf2) == 0)
		{
			printf("set LiDAR resample to %d\n", resample);
		}
	}
	return 0;
}

int main(int argc, char **argv)
{
	int with_chk = 1; 		// 使能数据校验

	if (argc < 9) {
		printf("usage : ./udp_lidar_demo 雷达地址 雷达端口 本地端口 单位是毫米 数据中带有强度 去拖点 平滑 重采样\n");
		return -1;
	}
	
	strcpy(lidar_ip, argv[1]); // 雷达的网络地址
	lidar_port = atoi(argv[2]); 			// 雷达的端口
	int local_port = atoi(argv[3]); 			// 雷达的端口
	int unit_is_mm = atoi(argv[4]); 	// 数据是毫米为单位,厘米时为0
	int with_confidence = atoi(argv[5]); 	// 数据中带有强度
	int with_deshadow = atoi(argv[6]); // 去拖点，0：关闭，1：开启
	int with_smooth = atoi(argv[7]); // 数据平滑， 0：关闭， 1：开启
	int resample = atoi(argv[8]); // 分辨率，0：原始数据，1：角度修正数据，200：0.2°，333：0.3°。。。。
	
	// open UDP port
	int fd_udp  = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	sockaddr_in addr;
	addr.sin_family = AF_INET;
	addr.sin_port = htons(local_port);
	addr.sin_addr.s_addr = htonl(INADDR_ANY);

	int rt = ::bind(fd_udp, (struct sockaddr *)&addr, sizeof(addr));
	if (rt != 0)
	{
		printf("bind port %d failed\n", local_port);
		return -1;
	}
	
	printf("start udp %s:%d udp %d\n", lidar_ip, lidar_port, fd_udp);

	char cmd[12] = "LGCPSH";
	rt = send_cmd_udp(fd_udp, lidar_ip, lidar_port, 0x0043, rand(), 6, cmd);

	int fd_cmd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	setup_lidar(fd_cmd, unit_is_mm, with_confidence, resample, with_deshadow, with_smooth);

	unsigned char* buf = new unsigned char[BUF_SIZE];
	int buf_len = 0;

	FILE* fp_rec = NULL;// fopen("/tmp/rec.dat", "ab");


	struct timeval tv;
	gettimeofday(&tv, NULL);
	time_t tto = tv.tv_sec + 1;
	uint32_t delay = 0;

	bool should_publish = false;
	int fan_span = 360;

	while (1)
	{ 
		fd_set fds;
		FD_ZERO(&fds); 
		FD_SET(fd_udp, &fds); 
		FD_SET(fd_cmd, &fds); 
		int fd_max = fd_udp > fd_cmd ? fd_udp : fd_cmd;
		
		struct timeval to = { 10, 1 };
		int ret = select(fd_max+1, &fds, NULL, NULL, &to); 

		gettimeofday(&tv, NULL);
		if (tv.tv_sec > tto) 
		{
			KeepAlive alive;
			gettimeofday(&tv, NULL);
			alive.world_clock = (tv.tv_sec % 3600) * 1000 + tv.tv_usec/1000;
			alive.delay = delay;

			// acknowlege device 
			//int rt = send_cmd_udp(fd_udp, info->lidar_ip, info->lidar_port, 0x4753, rand(), 0, NULL);
			send_cmd_udp_f(fd_cmd, lidar_ip, lidar_port, 0x4b41, rand(), sizeof(alive), &alive, false);

			tto = tv.tv_sec + 1;
		}

		if (ret == 0) 
		{
			rt = send_cmd_udp(fd_cmd, lidar_ip, lidar_port, 0x0043, rand(), 6, cmd);
			continue;
		}
		
		if (ret < 0) {
			printf("select error\n");
			return -1;
		}

		
		// read UDP command response
		if (FD_ISSET(fd_cmd, &fds)) 
		{
			sockaddr_in addr;
			socklen_t sz = sizeof(addr);

			int len = recvfrom(fd_cmd, buf, BUF_SIZE, 0, (struct sockaddr *)&addr, &sz);
			if (len > 0) {
			}
		}
		
		// read UDP data
		if (FD_ISSET(fd_udp, &fds)) 
		{ 
			sockaddr_in addr;
			socklen_t sz = sizeof(addr);

			int len = recvfrom(fd_udp, buf, BUF_SIZE, 0, (struct sockaddr *)&addr, &sz);
			if (len > 0) {
				RawData dat;
				bool is_pack;
				int consume;
				if (unit_is_mm && with_confidence)
				{
					is_pack = parse_data_x(len, buf, 
						fan_span,unit_is_mm, with_confidence,
						dat, consume, with_chk);
				}
				else {
					is_pack = parse_data(len, buf, 
						fan_span, unit_is_mm, with_confidence, 
						dat, consume, with_chk);
				}
				if (is_pack)
				{
					data_process(dat);
				}
			}
		}
	}

	return 0;
}

