// 解析后的数据可以在 data_process 分析
// 请根据实际连接方式修改main函数中对应参数
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include "data.h"

using namespace std;

//from_zero true时从[0，pi），false时[-pi,pi)
bool from_zero = false;
int factor = from_zero ? 2 : 1;


// 360 数据
void data_process(int n, DataPoint* points, uint32_t* timestamp = NULL)
{
	if (timestamp != NULL)
		printf("%d.%d ", timestamp[0], timestamp[1]);
	printf("%x : 360 data points %d\n", pack_format, n);
#if 0
	FILE* fp = fopen("last.txt", "w");
	if (fp) {
		for (int i = 0; i < n; i++)
		{
			fprintf(fp, "%.5f\t%.3f\t%d\n",
				points[i].angle > factor*PI ? points[i].angle-2*PI : points[i].angle, 
				points[i].distance, points[i].confidence);
		}
		fclose(fp);
	}
#endif
}

#ifndef FILL_FAN_ZERO
vector<RawData*> datas;
// 每次获得一个扇区（9°/ 36°)数据
void data_process(const RawData& raw)
{
	// int mi = 100000;
	// for (int i=0; i<data->N; i++) {
	// if (data->distance[i] > 0 && mi > data->distance[i])
	// mi = data->distance[i];
	// }
	//printf("角度 %d, 数据点数 %d + %d\n", raw.angle/10, raw.N, raw.span);

	RawData* data = new RawData;
	memcpy(data, &raw, sizeof(RawData));
	datas.push_back(data);

	if (raw.angle + raw.span != factor*1800)
	{
		return;
	}
	
	bool bfirst = true;
	uint32_t timestamp[2] = {0};
	int count = 0, n = 0, angles = 0;
	for (vector<RawData*>::iterator it = datas.begin(); it != datas.end(); ++it)
	{
		data = *it;
		angles += data->span;
		count += data->N;
		if (bfirst) {
			timestamp[0] = data->ts[0];
			timestamp[1] = data->ts[1];
			bfirst = false;
		}
		n++;
	}
	
	if (angles != 3600)
	{
		printf("angle sum %d, drop %d fans %d points\n", angles, n, count);
		for (vector<RawData*>::iterator it = datas.begin(); it != datas.end(); ++it)
		{
			data = *it;
			delete data;
		}
		datas.clear();
	}

	DataPoint* points = new DataPoint[count];
	count = 0;

	for (vector<RawData*>::iterator it = datas.begin(); it != datas.end(); ++it)
	{
		data = *it;
		for (int i = 0; i < data->N; i++) {
			points[count++] = data->points[i];
		}
		delete data;
	}
	datas.clear();

	data_process(count, points, timestamp);
	delete points;
}

#else

#define MAX_FANS 40
RawData* datas[MAX_FANS] = {NULL};

double calc_res()
{
	double sum = 0; 
	int cnt = 0;
	for (int i=0; i<MAX_FANS; i++) 
	{
		if (datas[i] != NULL) {
			sum += datas[i]->span;
			cnt += datas[i]->N;
		}
	}
	return sum/cnt;
}

int fill_zero(int from, int to, double res, DataPoint* points)
{
	printf("fill hole %d ~ %d : %f\n", from, to, res);
	int N = (to - from) / res;
	for (int j=0; j<N; j++) 
	{
		points[j].angle = (from + j * res) * PI / 1800;
		points[j].distance = 0;
		points[j].confidence = 0;
	}
	return N;
}

int data_publish(int from, int to, double res, DataPoint* points)
{
	int M = from / 90, N = to/90, count = 0;
	
	for (int i=M; i<N; i++)
	{
		RawData* data = datas[i];
		if (data == NULL) continue;

		if (from < data->angle) 
		{
			count += fill_zero(from, data->angle, res, points+count);
		}
			
		for (int j = 0; j < data->N; j++) 
		{
			 points[count++] = data->points[j];
		}
		from = data->angle + data->span;
	}

	if (from < to) 
	{
		count += fill_zero(from, to, res, points+count);
	}

	return count;
}

void data_publish(DataPoint* points)
{
	double res = calc_res();

	int count = data_publish(1800, 3600, res, points);

	count += data_publish(0, 1800, res, points+count);

	for (int i=0; i<MAX_FANS; i++) {
		delete datas[i];
		datas[i] = NULL;
	}
		
	data_process(count, points);
}

DataPoint g_points[10000];
// 每次获得一个扇区（9°/ 36°)数据
void data_process(const RawData& raw)
{
	//if ((rand() % 100) == 50) {
	//printf("角度 %d, 数据点数 %d + %d\n", raw.angle/10, raw.N, raw.span);
	//return ;
	//}

	RawData* data = new RawData;
	memcpy(data, &raw, sizeof(RawData));

	int idx = raw.angle / 90;

	if (datas[idx] != NULL) 
	{
		if (data->angle >= 1800)
		{
			data_publish(g_points);
		}
		else {
			delete datas[idx];
			datas[idx] = NULL;
		}
	}
		
	datas[idx] = data;
	if (raw.angle + raw.span == 1800)
	{
		data_publish(g_points);
	}
}


#endif



