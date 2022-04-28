// 解析后的数据可以在 data_process 分析
// 请根据实际连接方式修改main函数中对应参数

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>
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
#include <iostream>
#include <fstream>
#include <sstream>
#include "data.h"

struct CmdHeader
{
	unsigned short sign;
	unsigned short cmd;
	unsigned short sn;
	unsigned short len;
};

struct KeepAlive
{
	uint32_t world_clock;
	uint32_t mcu_hz;
	uint32_t arrive;
	uint32_t delay;
	uint32_t reserved[4];
};

struct RunConfig
{
	// paramters
	int local_port;
	int unit_is_mm;
	int with_confidence;
	int resample;
	int with_deshadow;
	int with_smooth;
	int with_chk;
	int data_bytes;
	int rpm;
	int output_scan;
	int output_360;
	int from_zero;
	char output_file[256];
	char lidar_ip[256];
	int lidar_port;
	int is_group_listener;
	char group_ip[256];
	// control
	bool should_quit;
	bool should_run;
	pthread_t thread;
};

bool read_config(const char *cfg_file_name, RunConfig &cfg)
{
	std::ifstream infile;
	infile.open(cfg_file_name);
	std::string s;
	std::string lidar_ip_s, lidar_port_s, local_port_s;
	std::string unit_is_mm_s, with_confidence_s, resample_s;
	std::string with_deshadow_s, with_smooth_s, with_chk_s;
	std::string raw_bytes_s, rpm_s, output_scan_s, output_360_s;
	std::string from_zero_s;
	std::string output_file;
	std::string is_group_listener;
	std::string group_ip;

	while (getline(infile, s))
	{
		std::string tmp;
		std::stringstream linestream(s);
		getline(linestream, tmp, ':');

		if (tmp == "lidar_ip")
		{
			getline(linestream, lidar_ip_s, ':');
			strcpy(cfg.lidar_ip, lidar_ip_s.c_str());
		}
		else if (tmp == "lidar_port")
		{
			getline(linestream, lidar_port_s, ':');
			cfg.lidar_port = atoi(lidar_port_s.c_str());
		}
		else if (tmp == "local_port")
		{
			getline(linestream, local_port_s, ':');
			cfg.local_port = atoi(local_port_s.c_str());
		}
		else if (tmp == "with_confidence")
		{
			getline(linestream, with_confidence_s, ':');
			cfg.with_confidence = atoi(with_confidence_s.c_str());
		}
		else if (tmp == "raw_bytes")
		{
			getline(linestream, raw_bytes_s, ':');
			cfg.data_bytes = atoi(raw_bytes_s.c_str());
		}
		else if (tmp == "unit_is_mm")
		{
			getline(linestream, unit_is_mm_s, ':');
			cfg.unit_is_mm = atoi(unit_is_mm_s.c_str());
		}
		else if (tmp == "with_chk")
		{
			getline(linestream, with_chk_s, ':');
			cfg.with_chk = atoi(with_chk_s.c_str());
		}
		else if (tmp == "with_smooth")
		{
			getline(linestream, with_smooth_s, ':');
			cfg.with_smooth = atoi(with_smooth_s.c_str());
		}
		else if (tmp == "with_deshadow")
		{
			getline(linestream, with_deshadow_s, ':');
			cfg.with_deshadow = atoi(with_deshadow_s.c_str());
		}
		else if (tmp == "resample")
		{
			getline(linestream, resample_s, ':');
			cfg.resample = atoi(resample_s.c_str());
		}
		else if (tmp == "rpm")
		{
			getline(linestream, rpm_s, ':');
			cfg.rpm = atoi(rpm_s.c_str());
		}
		else if (tmp == "output_scan")
		{
			getline(linestream, output_scan_s, ':');
			cfg.output_scan = atoi(output_scan_s.c_str());
		}
		else if (tmp == "output_360")
		{
			getline(linestream, output_360_s, ':');
			cfg.output_360 = atoi(output_360_s.c_str());
		}
		else if (tmp == "from_zero")
		{
			getline(linestream, from_zero_s, ':');
			cfg.from_zero = atoi(from_zero_s.c_str());
		}
		else if (tmp == "output_file")
		{
			getline(linestream, output_file, ':');
			strcpy(cfg.output_file, output_file.c_str());
		}
		else if (tmp == "is_group_listener")
		{
			getline(linestream, is_group_listener, ':');
			cfg.is_group_listener = atoi(is_group_listener.c_str());
		}
		else if (tmp == "group_ip")
		{
			getline(linestream, group_ip, ':');
			strcpy(cfg.group_ip, group_ip.c_str());
		}
	}
	return true;
}

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
bool send_cmd_udp_f(int fd_udp, const char *dev_ip, int dev_port,
					int cmd, int sn,
					int len, const void *snd_buf, bool bpr)
{
	char buffer[2048];
	CmdHeader *hdr = (CmdHeader *)buffer;
	hdr->sign = 0x484c;
	hdr->cmd = cmd;
	hdr->sn = sn;

	len = ((len + 3) >> 2) * 4;

	hdr->len = len;

	memcpy(buffer + sizeof(CmdHeader), snd_buf, len);

	int n = sizeof(CmdHeader);
	unsigned int *pcrc = (unsigned int *)(buffer + sizeof(CmdHeader) + len);
	pcrc[0] = stm32crc((unsigned int *)(buffer + 0), len / 4 + 2);

	sockaddr_in to;
	to.sin_family = AF_INET;
	to.sin_addr.s_addr = inet_addr(dev_ip);
	to.sin_port = htons(dev_port);

	int len2 = len + sizeof(CmdHeader) + 4;

	sendto(fd_udp, buffer, len2, 0, (struct sockaddr *)&to, sizeof(struct sockaddr));

	if (bpr)
	{
		char s[3096];
		for (int i = 0; i < len2; i++)
			sprintf(s + 3 * i, "%02x ", (unsigned char)buffer[i]);

		printf("send to %s:%d 0x%04x sn[%d] L=%d : %s\n",
			   dev_ip, dev_port, cmd, sn, len, s);
	}
	return true;
}

bool send_cmd_udp(int fd_udp, const char *dev_ip, int dev_port,
				  int cmd, int sn,
				  int len, const void *snd_buf)
{
	return send_cmd_udp_f(fd_udp, dev_ip, dev_port, cmd, sn, len, snd_buf, true);
}

bool udp_talk(int fd_udp, const char *lidar_ip, int lidar_port,
			  int n, const char *cmd,
			  int nhdr, const char *hdr_str,
			  int nfetch, char *fetch)
{
	printf("send command : \'%s\' \n", cmd);

	unsigned short sn = rand();
	int rt = send_cmd_udp(fd_udp, lidar_ip, lidar_port, 0x0043, sn, n, cmd);

	time_t t0 = time(NULL);
	int ntry = 0;
	while (time(NULL) < t0 + 3 && ntry < 1000)
	{
		fd_set fds;
		FD_ZERO(&fds);
		FD_SET(fd_udp, &fds);

		struct timeval to = {1, 0};
		int ret = select(fd_udp + 1, &fds, NULL, NULL, &to);

		if (ret < 0)
		{
			printf("select error\n");
			return false;
		}
		if (ret == 0)
		{
			continue;
		}

		// read UDP data
		if (FD_ISSET(fd_udp, &fds))
		{
			ntry++;
			sockaddr_in addr;
			socklen_t sz = sizeof(addr);

			char buf[1024] = {0};
			int nr = recvfrom(fd_udp, buf, sizeof(buf), 0, (struct sockaddr *)&addr, &sz);
			if (nr > 0)
			{
				CmdHeader *hdr = (CmdHeader *)buf;
				if (hdr->sign != 0x484c || hdr->sn != sn)
					continue;

				char *payload = buf + sizeof(CmdHeader);
				for (int i = 0; i < nr - nhdr - 1; i++)
				{
					if (memcmp(payload + i, hdr_str, nhdr) == 0)
					{
						// memcpy(fetch, buf+i+nhdr, nfetch);
						// fetch[nfetch] = 0;
						if (nfetch > 0)
						{
							memset(fetch, 0, nfetch);
							for (int j = 0; j < nfetch && i + nhdr + j < nr; j++)
								fetch[j] = payload[i + nhdr + j];
						}
						return true;
					}
				}
				// memcpy(fetch, "ok", 2);
				// fetch[2] = 0;
				// return true;
			}
		}
	}
	printf("read %d packets, not response\n", ntry);
	return false;
}
int setup_lidar(int fd_udp, const char *ip, int port,
				int unit_is_mm, int with_confidence, int resample, int with_deshadow, int with_smooth,char*groupip,int local_port)
{
	// char buf[] = "LUUIDH";
	// write(g_port, buf, strlen(buf));
	char buf[32];
	int nr = 0;

	if (udp_talk(fd_udp, ip, port, 6, "LUUIDH", 12, "PRODUCT SN: ", 9, g_uuid))
	{
		printf("get product SN : %s\n", g_uuid);
	}

	if (udp_talk(fd_udp, ip, port, 6, "LOCONH", 2, "OK", 0, NULL))
	{
	}

	if (!udp_talk(fd_udp, ip, port, 6, "LSTARH", 2, "OK", 0, NULL))
	{
		printf("start Lidar fail!\n");
	}

	if (!udp_talk(fd_udp, ip, port,
				  6, with_deshadow == 0 ? "LFFF0H" : "LFFF1H",
				  2, "OK", 0, NULL))
	{
		printf("set deshadow to %d fail!\n", with_deshadow);
	}

	if (!udp_talk(fd_udp, ip, port, 6,
				  with_smooth == 0 ? "LSSS0H" : "LSSS1H",
				  2, "OK", 0, NULL))
	{
		printf("set smooth to %d fail!\n", with_smooth);
	}
	if (resample == 0)
		strcpy(buf, "LSRES:000H");
	else if (resample == 1)
		strcpy(buf, "LSRES:001H");
	else if (resample > 100 && resample < 1000)
		sprintf(buf, "LSRES:%03dH", resample);
	else
		buf[0] = 0;

	if (buf[0])
	{
		if (!udp_talk(fd_udp, ip, port, 10, buf, 0, "OK", 0, NULL))
		{
			printf("set LiDAR resample to %d fail!\n", resample);
		}
	}
	return 0;
}
int m_flag = 0;
void *lidar_thread_proc(void *param)
{	

	RunConfig *cfg = (RunConfig *)param;
	char cmd[12] = "LUUIDH";
	// open UDP port
	int fd_udp = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	sockaddr_in addr;
	addr.sin_family = AF_INET;
	addr.sin_port = htons(cfg->local_port);
	addr.sin_addr.s_addr = htonl(INADDR_ANY);

	int rt = ::bind(fd_udp, (struct sockaddr *)&addr, sizeof(addr));
	if (rt != 0)
	{
		printf("\033[1;31m----> bind port %d failed.\033[0m\n", cfg->local_port);
		return NULL;
	}

	printf("\033[1;32m----> start udp %s:%d udp %d\033[0m\n", cfg->lidar_ip, cfg->lidar_port, fd_udp);

	if (cfg->is_group_listener == 1)
	{
		ip_mreq group;
		memset(&group, 0, sizeof(group));
		group.imr_multiaddr.s_addr = inet_addr(cfg->group_ip);
		group.imr_interface.s_addr = INADDR_ANY;

		int rt = setsockopt(fd_udp, IPPROTO_IP,
							IP_ADD_MEMBERSHIP, (char *)&group,
							sizeof(group));

		printf("Adding to multicast group %s %s\n", cfg->group_ip, rt < 0 ? "fail!" : "ok");
	}

	if (cfg->is_group_listener == 0)
	{
		char cmd[12] = "LUUIDH";
		rt = send_cmd_udp(fd_udp, cfg->lidar_ip, cfg->lidar_port, 0x0043, rand(), 6, cmd);

		setup_lidar(fd_udp, cfg->lidar_ip, cfg->lidar_port,
					cfg->unit_is_mm, cfg->with_confidence,
					cfg->resample, cfg->with_deshadow, cfg->with_smooth,cfg->group_ip,cfg->local_port);
	}
	unsigned char *buf = new unsigned char[BUF_SIZE];
	int buf_len = 0;

	struct timeval tv;
	gettimeofday(&tv, NULL);
	time_t tto = tv.tv_sec + 1;
	uint32_t delay = 0;

	bool should_publish = false;
	int fan_span = 360;
	int idle = 0;
	bool runing = true;
	
	while (!cfg->should_quit)
	{
		if (!cfg->is_group_listener)
		{
			// 启停雷达测距
			if (runing != cfg->should_run)
			{
				char str_start[] = "LSTARH";
				char str_stop[] = "LSTOPH";

				if (udp_talk(fd_udp, cfg->lidar_ip, cfg->lidar_port,
							 6, cfg->should_run ? str_start : str_stop,
							 2, "OK", 0, NULL))
				{
					runing = cfg->should_run;
				}
			}
		}
		fd_set fds;
		FD_ZERO(&fds);
		FD_SET(fd_udp, &fds);
	
		struct timeval to = {1, 1};
		int ret = select(fd_udp + 1, &fds, NULL, NULL, &to);
		if (!cfg->is_group_listener)
		{
			if (runing != cfg->should_run)
			{
				gettimeofday(&tv, NULL);
				if (tv.tv_sec > tto)
				{
					KeepAlive alive;
					gettimeofday(&tv, NULL);
					alive.world_clock = (tv.tv_sec % 3600) * 1000 + tv.tv_usec / 1000;
					alive.delay = delay;

					// acknowlege device
					// int rt = send_cmd_udp(fd_udp, info->lidar_ip, info->lidar_port, 0x4753, rand(), 0, NULL);0x4b41
					send_cmd_udp_f(fd_udp, cfg->lidar_ip, cfg->lidar_port, 0x4b41, rand(), sizeof(alive), &alive, false);
					// The host sends time synchronization information to the radar
					// printf("udp ip:%s  port:%d lidar:%d  delay:%d\n", cfg->lidar_ip,cfg->lidar_port,alive.world_clock, delay);
					g_flag = 1;
					tto = tv.tv_sec + 1;
				}

				if (ret == 0)
				{
					if (!runing)
						continue;

					if (idle++ > 10)
					{
						rt = send_cmd_udp(fd_udp, cfg->lidar_ip, cfg->lidar_port, 0x0043, rand(), 6, cmd);
						idle = 0;
					}
					continue;
				}

				if (ret < 0)
				{
					printf("select error\n");
					break;
				}
			}
		}
		// read UDP data
		if (FD_ISSET(fd_udp, &fds))
		{
			idle = 0;
			sockaddr_in addr;
			socklen_t sz = sizeof(addr);
			
			int len = recvfrom(fd_udp, buf, BUF_SIZE, 0, (struct sockaddr *)&addr, &sz);
			if (len > 0)
			{
				RawData dat;
				bool is_pack;
				int consume;
				if (cfg->unit_is_mm && cfg->with_confidence)
				{
					is_pack = parse_data_x(len, buf,
										   fan_span, cfg->unit_is_mm, cfg->with_confidence,
										   dat, consume, cfg->with_chk);

				}
				else
				{
					is_pack = parse_data(len, buf,
										 fan_span, cfg->unit_is_mm, cfg->with_confidence,
										 dat, consume, cfg->with_chk);
				}

				if (is_pack)
				{
					if (cfg->output_scan)
					{
						if (cfg->output_360)
						{
							fan_data_process(dat, cfg->output_file);
						}
						else
						{
							whole_data_process(dat, cfg->from_zero, cfg->output_file);
						}
					}
				}
			}
		}
	}

	close(fd_udp);

	delete buf;
	return NULL;
}

// 连接雷达，开始接收数据
RunConfig *StartDrv(const RunConfig &cfg)
{
	RunConfig *run = new RunConfig;
	memcpy(run, &cfg, sizeof(RunConfig));

	run->should_quit = false;
	run->should_run = true;
	pthread_create(&run->thread, NULL, lidar_thread_proc, run);

	return run;
}

// 释放连接
void StopDrv(RunConfig *run)
{
	run->should_quit = true;
	sleep(2);
}

// 启停雷达测距
void ControlDrv(RunConfig *run, bool start_motor)
{
	run->should_run = start_motor;
}

int main(int argc, char **argv)
{
	/*
	 * 1,params init
	 */
	if (argc < 2)
	{
		printf("usage : ./lidar ../params/xxx.txt\n");
		return -1;
	}

	const char *cfg_file_name = argv[1];

	RunConfig cfg;
	read_config(cfg_file_name, cfg);

	while (1)
	{
		RunConfig *round = StartDrv(cfg);

		while (1)
		{
			printf("\n=======================================\n");
			printf("input 'start' to restart LiDAR\n");
			printf("input 'stop' to stop LiDAR\n");
			printf("input 'exit' to disconnect\n");
			printf("=======================================\n");

			char line[80];
			fgets(line, 80, stdin);

			if (strncmp(line, "start", 5) == 0)
			{
				ControlDrv(round, true);
				continue;
			}
			else if (strncmp(line, "stop", 4) == 0)
			{
				ControlDrv(round, false);
				continue;
			}
			else if (strncmp(line, "exit", 4) == 0)
			{
				break;
			}
		}

		StopDrv(round);
	}

	return 0;
}
