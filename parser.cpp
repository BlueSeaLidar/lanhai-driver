// 解析后的数据可以在 data_process 分析
// 请根据实际连接方式修改main函数中对应参数

#include <stdio.h>
#include <string.h>
#include <time.h>
#include "data.h"

#ifdef _WIN32

#include <windows.h>
void gettimeofday(timeval* tv, void*)
{
	SYSTEMTIME st;
	GetLocalTime(&st);

	tv->tv_sec = time(NULL);
	tv->tv_usec = st.wMilliseconds;
}

#else

#include <sys/time.h>

#endif



static FanSegment* GetFanSegment(const RawDataHdr7& hdr, uint8_t* pdat, bool with_chk)
{
	FanSegment* fan_seg = new FanSegment;
	if (!fan_seg) {
		printf("out of memory\n");
		return NULL;
	}
	fan_seg->hdr = hdr;
	fan_seg->next = NULL;

	uint16_t sum = 0;
	//if (with_chk)
	{
		uint16_t* pchk = (uint16_t*)pdat;
		for (int i = 1; i < HDR7_SIZE / 2; i++)
			sum += pchk[i];
	}


	uint8_t* pDist = pdat + HDR7_SIZE;
	uint8_t* pAngle = pdat + HDR7_SIZE + 2 * hdr.N;
	uint8_t* energy = pdat + HDR7_SIZE + 4 * hdr.N;

	for (int i = 0; i < hdr.N; i++, pDist += 2, pAngle += 2)
	{
		fan_seg->dist[i] = ((uint16_t)(pDist[1]) << 8) | pDist[0];
		fan_seg->angle[i] = ((uint16_t)(pAngle[1]) << 8) | pAngle[0];
		fan_seg->energy[i] = energy[i];

		sum += fan_seg->dist[i];
		sum += fan_seg->angle[i];
		sum += energy[i];
	}

	uint8_t* pchk = pdat + HDR7_SIZE + 5 * hdr.N;
	uint16_t chksum = ((uint16_t)(pchk[1]) << 8) | pchk[0];
	if (chksum != sum) {
		printf("checksum error\n");
		delete fan_seg;
		return NULL;
	}

	return fan_seg;
}

void DecTimestamp(uint32_t ts, uint32_t* ts2)
{	
	timeval tv;
	gettimeofday(&tv, NULL);

	uint32_t sec = tv.tv_sec % 3600;
	if (sec < 5 && ts/1000 > 3595)
	{
		ts2[0] = (tv.tv_sec / 3600 -1) * 3600 + ts/1000;
	}
	else {
		ts2[0] = (tv.tv_sec / 3600) * 3600 + ts/1000;
	}

	ts2[1] = (ts%1000)*1000;
}

static void PackFanData(FanSegment* seg, RawData& rdat)
{
	//RawData* dat = new RawData;
	//if (!dat) { printf("out of memory\n"); return NULL; }
	RawData* dat = &rdat;

	dat->code = 0xfac7;
	dat->N = seg->hdr.whole_fan;
	dat->angle = seg->hdr.beg_ang / 100; // 0.1 degree
	dat->span = (seg->hdr.end_ang - seg->hdr.beg_ang) / 100; // 0.1 degree
	dat->fbase = 0;
	dat->first = 0;
	dat->last = 0;
	dat->fend = 0;

	DecTimestamp(seg->hdr.timestamp, dat->ts);
	//printf("%d %d.%d\n", dat->angle, dat->ts[0], dat->ts[1]);

	int count = 0;
	while (seg)
	{
		for (int i = 0; i < seg->hdr.N; i++, count++)
		{
			dat->points[count].confidence = seg->energy[i];
			dat->points[count].distance = seg->dist[i];
			dat->points[count].angle = (seg->angle[i] + seg->hdr.beg_ang) / 1000.0;
		}

		seg = seg->next;
	}
	//return dat;
}

static int GetFanPointCount(FanSegment* seg)
{
	int n = 0;

	while (seg) { n += seg->hdr.N;  seg = seg->next; }

	return n;
}

FanSegment* g_fan_segs = NULL;

static bool GetData0xC7(const RawDataHdr7& hdr, uint8_t* pdat, bool with_chk, RawData& dat)
{
	bool got = false;
#if 0
	if (parser->dev_id != ANYONE && hdr.dev_id != parser->dev_id) {
		static time_t last = 0;
		time_t t = time(NULL);
		if (t > last) {
			printf("device id [%d] != my id [%d]\n", hdr.dev_id, parser->dev_id);
			last = t;
		}
		// not my data
		return NULL;
	}
#endif

	FanSegment* fan_seg = GetFanSegment(hdr, pdat, with_chk);
	if (!fan_seg) {
		return NULL;
	}

	//printf("fan %d %d\n", fan_seg->hdr.beg_ang, fan_seg->hdr.ofset);

	if (g_fan_segs != NULL)
	{
		FanSegment* seg = g_fan_segs;

		if (seg->hdr.timestamp != fan_seg->hdr.timestamp)
		{
			printf("drop old fan segments\n");
			while (seg) {
				g_fan_segs = seg->next;
				delete seg;
				seg = g_fan_segs;
			}
			g_fan_segs = fan_seg;
		}
		else {
			while (seg) {
				if (seg->hdr.ofset == fan_seg->hdr.ofset) {
					printf("drop duplicated segment\n");
					delete fan_seg;
					fan_seg = NULL;
					break;
				}
				if (seg->next == NULL) {
					seg->next = fan_seg;
					break;
				}
				seg = seg->next;
			}
		}
	}

	if (g_fan_segs == NULL && fan_seg != NULL)
	{
		g_fan_segs = fan_seg;
	}

	// if (parser->fan_segs == NULL) { return NULL; }

	int N = GetFanPointCount(g_fan_segs);

	if (N >= g_fan_segs->hdr.whole_fan)
	{
		if (N == g_fan_segs->hdr.whole_fan)
		{
			if (N > sizeof(dat.points) / sizeof(dat.points[0]))
			{
				printf("too many %d points in 1 fan\n", N);
			}
			else
			{
				PackFanData(g_fan_segs, dat);
				got = true;
			}
		}

		// remove segments
		FanSegment* seg = g_fan_segs;
		while (seg) {
			g_fan_segs = seg->next;
			delete seg;
			seg = g_fan_segs;
		}

		//if (dat) {
			//SetTimeStamp(dat, );
				//dat->ros_angle = LidarAng2ROS(dat->angle + dat->span);
		//}
		//return dat;
	}
	return got;
}


bool GetData0xCE(const RawDataHdr& hdr, unsigned char* pdat, int span, int with_chk, RawData& dat)
{
	// calc checksum
	unsigned short sum = hdr.angle + hdr.N, chk;
	for (int i=0; i<hdr.N; i++)
	{
		dat.points[i].confidence = *pdat++;
		sum += dat.points[i].confidence;

		unsigned short v = *pdat++;
		unsigned short v2 = *pdat++;
		unsigned short vv = (v2<<8) | v;
		dat.points[i].distance = vv / 1000.0;
		sum += vv;
		dat.points[i].angle = (hdr.angle + (double)span * i / hdr.N) * PI / 1800;
	}

	memcpy(&chk, pdat, 2);

	if (with_chk != 0 && chk != sum) 
	{
		printf("chksum ce error");
		// consume = idx + HDR_SIZE + 3*hdr.N + 2;
		return false;
	}

	memcpy(&dat, &hdr, HDR_SIZE);
	dat.span = span;
	//memcpy(dat.data, buf+idx+HDR_SIZE, 2*hdr.N);
	//printf("get3 %d(%d)\n", hdr.angle, hdr.N);
	
	return true;
}

bool GetData0x9D(const RawDataHdr2& hdr, unsigned char* pdat, int with_chk, RawData& dat)
{
	unsigned short sum = hdr.angle + hdr.N + hdr.span, chk;

	for (int i = 0; i < hdr.N; i++)
	{
		dat.points[i].confidence = 1;// *pdat++;
		//sum += dat.points[i].confidence;

		unsigned short v = *pdat++;
		unsigned short v2 = *pdat++;

		unsigned short vv = (v2 << 8) | v;

		sum += vv;
		dat.points[i].distance = vv / 1000.0;
		dat.points[i].angle = (hdr.angle + hdr.span * i / (double)hdr.N) * PI / 1800;
	}

	memcpy(&chk, pdat, 2);

	if (with_chk != 0 && chk != sum)
	{
		printf("chksum cf error");
		return 0;
	}

	memcpy(&dat, &hdr, sizeof(hdr));
	//memcpy(dat.data, buf+idx+HDR_SIZE, 2*hdr.N);
	//printf("get3 %d(%d)\n", hdr.angle, hdr.N);

	return true;

}



bool GetData0xCF(const RawDataHdr2& hdr, unsigned char* pdat, int with_chk, RawData& dat)
{
	unsigned short sum = hdr.angle + hdr.N + hdr.span, chk;

	for (int i = 0; i < hdr.N; i++)
	{
		dat.points[i].confidence = *pdat++;
		sum += dat.points[i].confidence;

		unsigned short v = *pdat++;
		unsigned short v2 = *pdat++;

		unsigned short vv = (v2 << 8) | v;

		sum += vv;
		dat.points[i].distance = vv / 1000.0;
		dat.points[i].angle = (hdr.angle + hdr.span * i / (double)hdr.N) * PI / 1800;
	}

	memcpy(&chk, pdat, 2);

	if (with_chk != 0 && chk != sum)
	{
		printf("chksum cf error");
		return 0;
	}

	memcpy(&dat, &hdr, sizeof(hdr));
	//memcpy(dat.data, buf+idx+HDR_SIZE, 2*hdr.N);
	//printf("get3 %d(%d)\n", hdr.angle, hdr.N);

	return true;

}

bool GetData0xDF(const RawDataHdr3& hdr, unsigned char* pdat, int with_chk, RawData& dat)
{
	unsigned short sum = hdr.angle + hdr.N + hdr.span, chk;

	sum += hdr.fbase;
	sum += hdr.first;
	sum += hdr.last;
	sum += hdr.fend;

	double dan = (hdr.last - hdr.first) / double(hdr.N - 1);

	for (int i = 0; i < hdr.N; i++)
	{
		dat.points[i].confidence = *pdat++;
		sum += dat.points[i].confidence;

		unsigned short v = *pdat++;
		unsigned short v2 = *pdat++;
		unsigned short vv = (v2 << 8) | v;
		sum += vv;
		dat.points[i].distance = vv / 1000.0;
		dat.points[i].angle = (hdr.first + dan * i)*PI / 18000; 
	}

	memcpy(&chk, pdat, 2);

	if (with_chk != 0 && chk != sum)
	{
		printf("chksum df error");
		return 0;
	}

	memcpy(&dat, &hdr, HDR2_SIZE);
	//memcpy(dat.data, buf+idx+HDR_SIZE, 2*hdr.N);
	//printf("get3 %d(%d)\n", hdr.angle, hdr.N);

	return true;

}

int pack_format = 0xce;

char g_uuid[32] = "";
// translate lidar raw data to ROS laserscan message
bool parse_data_x(int len, unsigned char* buf, 
	int& span, int& is_mm, int& with_conf, 
	RawData& dat, int& consume, int with_chk) 
{
	int idx = 0;
	while (idx < len-18)
	{
		if (buf[idx] == 'S' && buf[idx+1] == 'T' && buf[idx+6] == 'E' && buf[idx+7] == 'D')
		{
			unsigned char flag = buf[idx + 2];
			span = 360;
			if (flag & 0x10) span = 180;
			if (flag & 0x20) span = 90;
			with_conf = flag & 2;
			is_mm = flag & 1;
			idx += 8;
		}

		if (buf[idx + 1] == 0xfa && (buf[idx] == 0xdf || buf[idx] == 0xce || buf[idx] == 0xcf || buf[idx] == 0xc7 || buf[idx] == 0x9d))
		{
			// found;
			pack_format = buf[idx];
		}
		else
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

		if (buf[idx] != 0xc7 && (hdr.angle % 90) != 0) 
		{
			printf("bad angle %d\n", hdr.angle);
			idx += 2;
			continue; 
		}

		if (hdr.N > MAX_POINTS || hdr.N < 10) 
		{
			printf("points number %d seem not correct\n", hdr.N);
			idx += 2;
			continue;
		}

		bool got;
		if (buf[idx] == 0xce && idx + HDR_SIZE + hdr.N * 3 + 2 <= len)
		{
			got = GetData0xCE(hdr, buf + idx + HDR_SIZE, 
				hdr.angle == 3420 ? span*2 : span, 
				with_chk, dat);
			consume = idx + HDR_SIZE + 3 * hdr.N + 2;
		}
		else if (buf[idx] == 0x9d && idx + hdr.N * 2 + 10 <= len)
		{
			RawDataHdr2 hdr2;
			memcpy(&hdr2, buf + idx, HDR2_SIZE);
			got = GetData0x9D(hdr2, buf + idx + HDR2_SIZE, with_chk, dat);
			consume = idx + HDR2_SIZE + 2 * hdr.N + 2;
		}
		else if (buf[idx] == 0xdf && idx + hdr.N * 3 + 18 <= len)
		{
			RawDataHdr3 hdr3;
			memcpy(&hdr3, buf + idx, HDR3_SIZE);
			got = GetData0xDF(hdr3, buf + idx + HDR3_SIZE, with_chk, dat);
			consume = idx + HDR3_SIZE + 3 * hdr.N + 2;
		}
		else if (buf[idx] == 0xcf && idx + HDR2_SIZE + hdr.N * 3 + 2 <= len)
		{
			RawDataHdr2 hdr2;
			memcpy(&hdr2, buf + idx, HDR2_SIZE);
			got = GetData0xCF(hdr2, buf + idx + HDR2_SIZE, with_chk, dat);
			consume = idx + HDR2_SIZE + 3 * hdr.N + 2;
		}
		else if (buf[idx] == 0xc7 && idx + HDR7_SIZE + hdr.N * 5 + 2 <= len)
		{
			RawDataHdr7 hdr7;
			memcpy(&hdr7, buf + idx, HDR7_SIZE);

			got = GetData0xC7(hdr7, buf + idx, with_chk, dat);

			consume = idx + HDR7_SIZE + 5 * hdr.N + 2;
		} 
		
		else {
			// data packet not complete
			break;
		}
		return got;
	}

	if (idx > 1024) consume = idx/2;
	return false;
}

bool parse_data(int len, unsigned char* buf, 
	int& span, int& is_mm, int& with_conf, 
	RawData& dat, int& consume, int with_chk) 
{
	int idx = 0;
	while (idx < len-180)
	{
		if (buf[idx] == 'S' && buf[idx+1] == 'T' && buf[idx+6] == 'E' && buf[idx+7] == 'D')
		{
			unsigned char flag = buf[idx + 2];
			span = 360;
			if (flag & 0x10) span = 180;
			if (flag & 0x20) span = 90;
			with_conf = flag & 2;
			is_mm = flag & 1;
			idx += 8;
		}
		
		if (buf[idx] != 0xce || buf[idx+1] != 0xfa)
		{
			idx++;
			continue;
		}
		pack_format = buf[idx];
			
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
				dat.points[i].confidence = val >> 13;
				dat.points[i].distance = val & 0x1fff;
				dat.points[i].distance /= (is_mm ? 1000.0 : 100.0) ;
			} else {
				dat.points[i].confidence = is_mm ? val : val*10;
				dat.points[i].confidence = 0;
			}

			dat.points[i].angle = (hdr.angle + 360.0 * i / hdr.N) * PI / 1800;

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


