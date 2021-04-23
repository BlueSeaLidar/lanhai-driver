#ifndef _LIDAR_DATA
#define  _LIDAR_DATA

#define PI 3.1415926535898

#define HDR_SIZE 6
#define HDR2_SIZE 8
#define HDR3_SIZE 16 

#define MAX_POINTS 500

#define BUF_SIZE 8*1024


//#include <linux/termios.h>
struct RawDataHdr
{
	unsigned short code;
	unsigned short N;
	unsigned short angle;
};

struct RawDataHdr2
{
	unsigned short code;
	unsigned short N;
	unsigned short angle;
	unsigned short span;
};

struct RawDataHdr3
{
	unsigned short code;
	unsigned short N;
	unsigned short angle;
	unsigned short span;
	unsigned short fbase;
	unsigned short first;
	unsigned short last;
	unsigned short fend;
};

struct DataPoint
{
	float angle; // 弧度
	float distance; // 米
	unsigned char confidence;
};

struct RawData
{
	unsigned short code;
	unsigned short N;
	unsigned short angle;
	unsigned short span;

	DataPoint points[MAX_POINTS];
	// unsigned short distance[1000];
	// unsigned char confidence[1000];
};

extern int pack_format;
extern char g_uuid[32];

// 固定格式解析
bool parse_data(int len, unsigned char* buf, 
	int& span, int& is_mm, int& with_conf, 
	RawData& dat, int& consume, int with_chk);

// 多种格式
bool parse_data_x(int len, unsigned char* buf, 
	int& span, int& is_mm, int& with_conf, 
	RawData& dat, int& consume, int with_chk);

// 用户的数据处理
void data_process(const RawData& raw);

#endif