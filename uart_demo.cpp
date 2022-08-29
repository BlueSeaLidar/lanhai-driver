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
#include <iostream>
#include <fstream>
#include <sstream>
#include "data.h"

extern "C" int change_baud(int fd, int baud);

// serial port handle
int g_port = -1;

int open_serial_port(const char* port, int baudrate) 
{
       	int fd = open(port,  O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd < 0) { 
		printf("\033[1;31m----> Open %s error\033[0m\n", port); 
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
#if 0
	char path[256];
	sprintf(path, "/tmp/%s.dat", hdr_str);
	FILE* fp = fopen(path, "wb");
	if (fp) {
		fwrite(buf, 1, sizeof(buf), fp);
		fclose(fp);
	}
#endif

	printf("read %d bytes, not found %s\n", nr, hdr_str);
	return -1;
}
int strip(const char* s, char* buf)
{
        int len = 0;
        for (int i=0; s[i] != 0; i++)
        {
                if (s[i] >= 'a' && s[i] <= 'z')
                        buf[len++] = s[i];
                else if (s[i] >= 'A' && s[i] <= 'Z')
                        buf[len++] = s[i];
                else if (s[i] >= '0' && s[i] <= '9')
                        buf[len++] = s[i];
		else if (len > 0)
			break;
        }
        buf[len] = 0;
        return len;
}
int setup_lidar(int fd_uart, int unit_is_mm, int with_confidence, int resample, int with_deshadow, int with_smooth, int init_rpm)
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
	//get product sn (Compatible with multiple models)
	if (uart_talk(fd_uart, 6, "LUUIDH", 11, "PRODUCT SN:", 16, buf) == 0) 
	{
		strip(buf, g_uuid);
		printf("get product SN : %s\n", g_uuid);
	}
	else if (uart_talk(fd_uart, 6, "LUUIDH", 10, "VENDOR ID:", 16, buf) == 0) 
	{
		strip(buf, g_uuid);
		printf("get product SN : %s\n", g_uuid);
	}
	//Set the lidar returned data unit   CM or MM
	if (uart_talk(fd_uart, 6, unit_is_mm == 0 ? "LMDCMH" : "LMDMMH", 
				10, "SET LiDAR ", 9, buf) == 0)
	{
		printf("set LiDAR unit to %s\n", buf);
	}
	//set lidar confidence state   LNCONH close   LOCONH open
	if (uart_talk(fd_uart, 6, with_confidence == 0 ? "LNCONH" : "LOCONH", 
				6, "LiDAR ", 5, buf) == 0)
	{
		printf("set LiDAR confidence to %s\n", buf);
	}
	//set  de-deshadow state    LFFF0H:close  LFFF1H:open
	if (uart_talk(fd_uart, 6, with_deshadow == 0 ? "LFFF0H" : "LFFF1H", 
				6, "LiDAR ", 5, buf) == 0)
	{
		printf("set deshadow to %d\n", with_deshadow);
	}
	//set  de-smooth     LSSS0H:close   LSSS1H:open
	if (uart_talk(fd_uart, 6, with_smooth == 0 ? "LSSS0H" : "LSSS1H", 
				6, "LiDAR ", 5, buf) == 0)
	{
		printf("set smooth to %d\n", with_smooth);
	}
	//LSRES:000H :set default Angular resolution  LSRES:001H :fix Angular resolution  
	if (resample == 0)
		strcpy(buf, "LSRES:000H");
	else if (resample == 1)
		strcpy(buf, "LSRES:001H");
	else
		buf[0] = 0;

	if (buf[0]) {
		char buf2[32];
		if (uart_talk(fd_uart, 10, buf, 15, "set resolution ", 1, buf2) == 0)
		{
			printf("set LiDAR resample to %d\n", resample);
		}
	}

	// setup rpm  (The specific model range is different)
	if (init_rpm > 300 && init_rpm < 3000) {
		for (int i=0; i<10; i++) 
		{
			char cmd[32];
			sprintf(cmd, "LSRPM:%dH", init_rpm);
			if (uart_talk(fd_uart, strlen(cmd), cmd, 3, "RPM", 5, buf) == 0)
			{
				printf("set RPM to %s\n", buf);
				break;
			}
		}
	}
	return 0;
}

int main(int argc, char **argv)
{
	/*
     * 1,params init
     */
	if (argc < 2) {
        printf("usage : ./lidar ../params/xxx.txt\n");
        return -1;
    }
    std::string file_name = argv[1];
    std::ifstream infile;
    infile.open(file_name.c_str());
    std::string s;
    std::string port,baud_rate_s,unit_is_mm_s,with_confidence_s,resample_s,with_deshadow_s,with_smooth_s,with_chk_s,raw_bytes_s,rpm_s,output_scan_s,output_360_s,from_zero_s,output_file;
    int baud_rate,unit_is_mm,with_confidence,resample,with_deshadow,with_smooth,with_chk,data_bytes,rpm,output_scan,output_360,from_zero;
    while (getline(infile, s)){
        std::string tmp;
        std::stringstream linestream(s);
        getline(linestream, tmp, ':');

        if(tmp == "port"){
            getline(linestream, port, ':');
        }else if(tmp == "baud_rate"){
            getline(linestream, baud_rate_s, ':');
            baud_rate = atoi(baud_rate_s.c_str());
        }else if(tmp == "with_confidence"){
            getline(linestream, with_confidence_s, ':');
            with_confidence = atoi(with_confidence_s.c_str());
        }else if(tmp == "raw_bytes"){
            getline(linestream, raw_bytes_s, ':');
            data_bytes = atoi(raw_bytes_s.c_str());
        }else if(tmp == "unit_is_mm"){
            getline(linestream, unit_is_mm_s, ':');
            unit_is_mm = atoi(unit_is_mm_s.c_str());
        }else if(tmp == "with_chk"){
            getline(linestream, with_chk_s, ':');
            with_chk = atoi(with_chk_s.c_str());
        }else if(tmp == "with_smooth"){
            getline(linestream, with_smooth_s, ':');
            with_smooth = atoi(with_smooth_s.c_str());
        }else if(tmp == "with_deshadow"){
            getline(linestream, with_deshadow_s, ':');
            with_deshadow = atoi(with_deshadow_s.c_str());
        }else if(tmp == "resample"){
            getline(linestream, resample_s, ':');
            resample = atoi(resample_s.c_str());
        }else if(tmp == "rpm"){
            getline(linestream, rpm_s, ':');
            rpm = atoi(rpm_s.c_str());
        }else if(tmp == "output_scan"){
            getline(linestream, output_scan_s, ':');
            output_scan = atoi(output_scan_s.c_str());
        }else if(tmp == "output_360"){
            getline(linestream, output_360_s, ':');
            output_360 = atoi(output_360_s.c_str());
        }else if(tmp == "from_zero"){
            getline(linestream, from_zero_s, ':');
            from_zero = atoi(from_zero_s.c_str());
        }else if(tmp == "output_file"){
            getline(linestream, output_file, ':');
        }
    }

	/*
	 * 2, open uart port
	 */
	int fd_uart = open_serial_port(port.c_str(), baud_rate);
	if (fd_uart < 0) {
		printf("\033[1;31m----> Open port error \033[0m\n");
		return -1;
	}
	g_port = fd_uart;
	
	/*
	 * 3, send cmd to lidar to set params
	 */
	setup_lidar(fd_uart, unit_is_mm, with_confidence, resample, with_deshadow, with_smooth, rpm);
    printf("\033[1;32m----> All params set OK ! Start parser data.\033[0m\n");

    /*
     * 4, read and parser data
     */
	unsigned char* buf = new unsigned char[BUF_SIZE];
	int buf_len = 0;

	FILE* fp_rec = NULL;// fopen("/tmp/rec.dat", "ab");

	int fan_span = 360;//36 degrees
	while (1)
	{
		/*
	     * check fd_uart is ok
	     */
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

		//read data process
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

			new_data = nr;//recevied data length
		}

		/*
		 * do parser process
		 */
		if (new_data > 0)
		{
			buf_len += new_data;

			int consume = 0; //in order to compute the rest of data after every parser process
			RawData dat;
			bool is_pack;
			//if (unit_is_mm)// && with_confidence)
			if (data_bytes == 3)
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
			//data output
			if (is_pack)
			{
				if(output_scan){
					if(output_360){
						fan_data_process(dat, output_file.c_str());
					}else{
						whole_data_process(dat,from_zero,output_file.c_str());
					}
				}
			}

			if (consume > 0) 
			{
				//data is not whole fan,drop it
				if (!is_pack) {
#if 0
					FILE* fp = fopen("/tmp/bad.dat", "ab");
					if (fp) {
						fwrite(buf, 1, consume, fp);
						fclose(fp);
					}
#endif
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

