#ifndef _LIDAR_DATA
#define  _LIDAR_DATA

#include <stdint.h>
#include <string>
#define PI 3.1415926535898

#define HDR_SIZE 6
#define HDR2_SIZE 8
#define HDR3_SIZE 16 
#define HDR7_SIZE 28 
#define HDR99_SIZE 32 
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


struct RawDataHdr7 {
	uint16_t code;
	uint16_t N;
	uint16_t whole_fan;
	uint16_t ofset;
	uint32_t beg_ang;
	uint32_t end_ang;
	uint32_t flags;
	uint32_t timestamp;
	uint32_t dev_id;
};


struct FanSegment
{
	RawDataHdr7 hdr;

	uint16_t dist[MAX_POINTS];
	uint16_t angle[MAX_POINTS];
	uint8_t energy[MAX_POINTS];

	struct FanSegment* next;
};

struct RawDataHdr99 {
	uint16_t code;
	uint16_t N;
	uint16_t from;
	uint16_t total;
	uint32_t flags;
	uint32_t timestamp;
	uint32_t dev_no;
	uint32_t reserved[3];
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
	unsigned short angle; // 0.1 degree
	unsigned short span; // 0.1 degree
	unsigned short fbase;
	unsigned short first;
	unsigned short last;
	unsigned short fend;
	// short ros_angle;	// 0.1 degree
	DataPoint points[MAX_POINTS];
	uint32_t ts[2];

};

extern int pack_format;
extern char g_uuid[32];
extern uint32_t g_timestamp[2];
extern int  g_flag;
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
void fan_data_process(const RawData& raw, const char* output_file);
void whole_data_process(const RawData& raw, bool from_zero, const char* output_file);
#endif
