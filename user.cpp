// 解析后的数据可以在 data_process 分析
// 请根据实际连接方式修改main函数中对应参数
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include "data.h"
#include <string>
using namespace std;

//from_zero true时从[0，pi），false时[-pi,pi)
bool from_zero = false;
int factor = from_zero ? 2 : 1;
uint32_t g_timestamp[2]={0};

// 360 数据
void data_process(int n, DataPoint* points, uint32_t* timestamp = NULL)
{
	if (timestamp != NULL)
		printf("%d.%d ", timestamp[0], timestamp[1]);
	//printf("%x : 360 data points %d\n", pack_format, n);
	for (int i = 0; i < n; i++)
		{
			printf("%.5f\t%.3f\t%d\n",
				points[i].angle > factor*PI ? points[i].angle-2*PI : points[i].angle, 
				points[i].distance, points[i].confidence);
		}
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
	delete[] points;
}

int g_flag=0;
int m_oldtime=0;
void fan_data_process(const RawData& raw, const char* output_file)
{
	RawData* data = new RawData;
	memcpy(data, &raw, sizeof(RawData));
	uint32_t timestamp[2] = {0};
	timestamp[0] = data->ts[0];
	timestamp[1] = data->ts[1];
	if(g_flag==1)
	{
		int temp = (timestamp[0] % 3600) * 1000 + timestamp[1] / 1000;
		if(temp-m_oldtime>10000)
		{
		  //printf("after time:%d  error: \033[%d\033[0m\n",temp,temp-m_oldtime);
		  printf("\033[0;31m after time:%d  error:%d\n",temp,temp-m_oldtime);
		}
   		else
   		{
   			printf("\033[0m after time:%d \n",temp);
   		}
   		m_oldtime=temp;
   		g_flag=0;
   	}
    	// for (int i = 0; i < data->N; i++)
		 //	{
		 		//printf("%.5f\t%.3f\t%d\n", data->points[i].angle, data->points[i].distance, data->points[i].confidence);
		 //	}
	printf("single span data points %d  time:%d.%d\n",data->N,timestamp[0],timestamp[1]);
	if (output_file != NULL)
	{
		FILE* fp = fopen(output_file, "w");
		if (fp) {
			// for (int i = 0; i < data->N; i++)
			// {
			// 	fprintf(fp, "%.5f\t%.3f\t%d\n", data->points[i].angle, data->points[i].distance, data->points[i].confidence);
			// }
			int diff = timestamp[0]*1000+timestamp[1]-g_timestamp[0]*1000-g_timestamp[1];
			fprintf(fp, "[diff]:%d [new0]:%d  [new1]:%d \n",diff,timestamp[0], timestamp[1]);
			fclose(fp);
		}
	}
	memcpy(g_timestamp,timestamp,sizeof(timestamp));
	delete data;
}

vector<RawData*> whole_datas;
void whole_data_process(const RawData& raw, bool from_zero, const char* output_file) 
{
    RawData *data = new RawData;
    memcpy(data, &raw, sizeof(RawData));
    whole_datas.push_back(data);

    int factor = from_zero ? 2 : 1;
    if (raw.angle + raw.span != factor * 1800) {
        return;
    }

    bool bfirst = true;
    uint32_t timestamp[2] = {0};
    int count = 0, n = 0, angles = 0;

    for (std::vector<RawData *>::iterator it = whole_datas.begin(); it != whole_datas.end(); ++it) {
        data = *it;
        //accumulate point's angle and counts
        angles += data->span;
        count += data->N;
        //record first point timestamp
        if (bfirst) {
            timestamp[0] = data->ts[0];
            timestamp[1] = data->ts[1];
            bfirst = false;
        }
        //record fan counts
        n++;
    }

    //not whole circle data, so clear it
    if (angles != 3600) {
        printf("angle sum %d, drop %d fans %d points\n", angles, n, count);
        for (std::vector<RawData *>::iterator it = whole_datas.begin(); it != whole_datas.end(); ++it) {
            data = *it;
            delete data;
        }
        whole_datas.clear();
    }

    DataPoint *points = new DataPoint[count];//store whole circle points
    count = 0;
    for (std::vector<RawData *>::iterator it = whole_datas.begin(); it != whole_datas.end(); ++it) {
        data = *it;
        for (int i = 0; i < data->N; i++) {
            points[count++] = data->points[i];
        }
        delete data;
    }
    whole_datas.clear();

    //printf("\r%d.%d : Data frame head = %x, 360 degrees contains %d spans", timestamp[0], timestamp[1], pack_format, n);
	printf("single span data points %d  time:%d.%d\n",count,timestamp[0],timestamp[1]);
	//g_timestamp=timestamp;
	
	if (output_file != NULL)
	{
		FILE *fp = fopen(output_file, "w");
		if (fp) {
			// for (int i = 0; i < count; i++) {
			// 	fprintf(fp, "%.5f\t%.3f\t%d\n",
			// 				points[i].angle > factor * PI ? points[i].angle - 2 * PI : points[i].angle,
			// 				points[i].distance, points[i].confidence);
			// }
			int diff = timestamp[0]*1000+timestamp[1]-g_timestamp[0]*1000-g_timestamp[1];
			fprintf(fp, "[diff]:%d [new0]:%d  [new1]:%d \n",diff,timestamp[0], timestamp[1]);
			fclose(fp);
		}
	}
	memcpy(g_timestamp,timestamp,sizeof(timestamp));
	delete[] points;
    
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



