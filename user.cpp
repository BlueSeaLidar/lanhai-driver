// 解析后的数据可以在 data_process 分析
// 请根据实际连接方式修改main函数中对应参数
#include <stdio.h>
#include <string.h>
#include <vector>
#include "data.h"

using namespace std;
vector<RawData*> datas;

// 360 数据
void data_process(int n, DataPoint* points)
{
	printf("%x : 360 data points %d\n", pack_format, n);
#if 0
	FILE* fp = fopen("last.txt", "w");
	if (fp) {
		for (int i = 0; i < n; i++)
		{
			fprintf(fp, "%.5f\t%.3f\t%d\n",
				points[i].angle > PI ? points[i].angle-2*PI : points[i].angle, 
				points[i].distance, points[i].confidence);
		}
		fclose(fp);
	}
#endif
}

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

	if (raw.angle + raw.span != 1800)
	{
		return;
	}
	
	int count = 0, n = 0, angles = 0;
	for (vector<RawData*>::iterator it = datas.begin(); it != datas.end(); ++it)
	{
		data = *it;
		angles += data->span;
		count += data->N;
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

	data_process(count, points);
	delete points;
}





