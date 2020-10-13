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

int main(int argc, char **argv)
{
	if (argc < 4) 
	{
		printf("usage : ./record /dev/ttyUSBx 波特率 文件名\n");
		return -1;
	}

	const char* port = argv[1];
	int baud_rate = atoi(argv[2]);
	const char* filename = argv[3];

	int fd_uart = open_serial_port(port, baud_rate);
       	if (fd_uart < 0) 
	{
	       	printf("Open port %s error\n", port);
	       	return -1;
	}
	
#define BUF_SIZE 4096
	unsigned char* buf = new unsigned char[BUF_SIZE];
	FILE* fp_rec = fopen(filename, "wb");

	printf("Ctrl-C to quit\n");
	while (1)
	{ 
		fd_set fds;
	       	FD_ZERO(&fds); 

		int fd_max = -1;
		FD_SET(fd_uart, &fds); 
		if (fd_max < fd_uart) fd_max = fd_uart;

		struct timeval to = { 1, 1 };
	       	int ret = select(fd_max+1, &fds, NULL, NULL, &to); 

		if (ret == 0) 
		{
			printf("read data timeout\n");
			continue;
		}
		
		if (ret < 0) {
			printf("select error");
			return -1;
		}

		if (fd_uart > 0 && FD_ISSET(fd_uart, &fds)) 
		{
			int nr = read(fd_uart, buf, BUF_SIZE);
			if (nr <= 0) {
				printf("read port error %d", nr);
				break;
			}

			if (nr > 0 && fp_rec) { 
				fwrite(buf, 1, nr, fp_rec);
				fflush(fp_rec);
		       	}
		}
	}

	//close(fd);
       	return 0;
}

