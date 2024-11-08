
#include <iostream>
#include <vector>
#include <algorithm>
#include <string>
#include <cctype>
#include <cstring>
#include <cmath>
#include <map>

#include "rtklib.h"

/* show message --------------------------------------------------------------*/
extern int showmsg(const char *format, ...)
{
    va_list arg;
    va_start(arg,format); vfprintf(stderr,format,arg); va_end(arg);
    fprintf(stderr,*format?"\r":"\n");
    return 0;
}

#define MAX_HEADER_BUFF (64)
#define MAX_HEADER_SEGM (10)

typedef struct
{
	gtime_t time;
	uint8_t dat[MAX_HEADER_BUFF];
	int slen;
	int nlen;
	int nseg;
	int nloc[MAX_HEADER_SEGM];
}buf_t;

static int add_buff_data(buf_t* buff, int data, int nseg)
{
	int ret = 0;
	double ms_time = 0;
	int wk = 0;
	double ep[6] = { 0 };
	double ws = 0;
	if (buff->nlen >= MAX_HEADER_BUFF)
		buff->nlen = 0;
	if (buff->nlen == 0)
	{
		buff->nseg = 0;
		buff->slen = 0;
	}
	buff->dat[buff->nlen++] = data;
	if (data == ',')
	{
		if (buff->nseg < MAX_HEADER_SEGM)
		{
			buff->nloc[buff->nseg++] = buff->nlen;
		}
		else
		{
			buff->nlen = 0;
		}
	}
	if (buff->nseg < 2 || buff->nlen < 2) return 0;
	if (data == ',' && buff->nseg >= nseg)
	{
		buff->slen = atoi((char*)(buff->dat + buff->nloc[buff->nseg - 2]));
		if (buff->nseg > 2)
			ms_time = atof((char*)(buff->dat + buff->nloc[buff->nseg - 3]));
		else
			ms_time = atof((char*)(buff->dat));
		ms_time += 18000.0;
		buff->time.time = (time_t)floor(ms_time / 1000);
		buff->time.sec = ms_time / 1000 - buff->time.time;
		ws = time2gpst(buff->time, &wk);
		wk = wk;
		time2epoch(buff->time, ep);
		ret = 1;
	}
	return ret;
}

static int read_buff_from_file(FILE* fBIN, buf_t* buff, int nseg)
{
	int ret = 0;
	int data = 0;
	while (fBIN && !feof(fBIN) && (data=fgetc(fBIN))!=EOF)
	{
		if (ret = add_buff_data(buff, data, nseg)) break;
	}
	return ret;
}

static FILE* set_output_file(const char* fname, const char* key)
{
	char filename[255] = { 0 }, outfilename[255] = { 0 };
	strcpy(filename, fname);
	char* temp = strrchr(filename, '.');
	if (temp) temp[0] = '\0';
	sprintf(outfilename, "%s-%s", filename, key);
	return fopen(outfilename, "wb");
}

static int output_status(std::vector<double>& vDt, std::map<int, int>& mGAP, int numofmsg, int numofcrc, int numofepoch, gtime_t stime, gtime_t etime, int v1, int v2, int v3, int v4, const char *fname, int is_time)
{
	double dt_avg = 0;
	double dt_68 = 0;
	double dt_95 = 0;
	double dt_99 = 0;
	double dt_min = 0;
	double dt_max = 0;
	int i = 0;
	int numofepoch_3s = 0;
	if (vDt.size() > 0)
	{
		std::sort(vDt.begin(), vDt.end());
		dt_min = vDt.front();
		for (std::vector<double>::iterator pDt = vDt.begin(); pDt != vDt.end(); ++pDt)
		{
			if (!is_time)
				*pDt -= dt_min;
			if (*pDt > 3.0)
			{
				++numofepoch_3s;
			}
		}
		for (i = 0; i < (int)vDt.size(); ++i)
			dt_avg += vDt[i];

		dt_avg /= (int)vDt.size();
		int loc68 = (int)(vDt.size() * 0.6827);
		int loc95 = (int)(vDt.size() * 0.9545);
		int loc99 = (int)(vDt.size() * 0.9973);

		dt_68 = vDt[loc68];
		dt_95 = vDt[loc95];
		dt_99 = vDt[loc99];
		dt_max = vDt.back();
	}
	FILE* fSUM = fopen("sum.txt", "r");
	char buffer[255] = { 0 };
	int line = 0;
	while (fSUM && fgets(buffer, sizeof(buffer), fSUM))
	{
		char* temp = strrchr(buffer, '\n');
		if (temp) temp[0] = '\0';
		if (strlen(buffer) > 0)
			++line;
	}
	if (fSUM) fclose(fSUM);
	fSUM = fopen("sum.txt", "a");
	if (!line)
	{
		fprintf(fSUM, "filename,numofmsg,numofcrc_failure,numofepoch,numofepoch_3s,dt_avg,dt_68,dt_95,dt_99,dt_min,dt_max,dt_count,time_duration,data_interval_counts\n");
	}
	printf("%s,%6i,%6i,%6i,%6i,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%6i,%10.4f,%7.2f,%i,%i,%i", fname, numofmsg, numofcrc, numofepoch, numofepoch_3s, dt_avg, dt_68, dt_95, dt_99, dt_min, dt_max, (int)vDt.size(), timediff(etime, stime), v1 / 3600.0 / 24.0, -v2, v3, v4);
	if (fSUM) fprintf(fSUM, "%s,%6i,%6i,%6i,%6i,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%6i,%10.4f,%7.2f,%i,%i,%i", fname, numofmsg, numofcrc, numofepoch, numofepoch_3s, dt_avg, dt_68, dt_95, dt_99, dt_min, dt_max, (int)vDt.size(), timediff(etime, stime), v1 / 3600.0 / 24.0, -v2, v3, v4);
	for (std::map<int, int>::iterator pGAP = mGAP.begin(); pGAP != mGAP.end(); ++pGAP)
	{
		if (fSUM) fprintf(fSUM, ",%i(%i)", pGAP->first, pGAP->second);
		printf(",%i(%i)", pGAP->first, pGAP->second);
	}
	printf("\n");
	if (fSUM) fprintf(fSUM, "\n");
	if (fSUM) fclose(fSUM);
	return 0;
}

static int log2rtcm(const char *fname, int is_geod, int is_out, int is_time)
{
	int ret = 0;
	FILE *fLOG = fopen(fname, "rb");
	FILE *fRTCM = nullptr;
	rtcm_t* rtcm = new rtcm_t;
	init_rtcm(rtcm);
	
	buf_t buf = { 0 };
	int i = 0;
	std::vector<double> vDt;
	gtime_t stime = { 0 };
	gtime_t etime = { 0 };
	int numofepoch = 0;
	std::map<int, int> mGAP;
	int numofcrc = 0;
	int numofmsg = 0;
	int v1 = 0;
	int v2 = 0;
	int v3 = 0;
	int v4 = 0;
	while (fLOG && !feof(fLOG))
	{
		ret = read_buff_from_file(fLOG, &buf, is_geod ? 3 : 2);
		if (ret)
		{
			int slen = buf.slen + 2; /* \r\n */
			int ret = 0;
			if (slen > 2)
			{
				if (!fRTCM) fRTCM = set_output_file(fname, "log.rtcm");
				char* tempBuff = new char[slen];
				if (fread((char*)tempBuff, sizeof(char), slen, fLOG) == slen)
				{
					rtcm->time_s = buf.time;
					for (i = 0; i < slen - 2; ++i)
					{
						int ret1 = input_rtcm3(rtcm, tempBuff[i]);
						if (rtcm->nbyte == 0 && rtcm->len > 0)
						{
							++numofmsg;
							if (rtk_crc24q(rtcm->buff, rtcm->len) != getbitu(rtcm->buff, rtcm->len * 8, 24))
							{
								++numofcrc;
							}
							int vers = getbitu(rtcm->buff, 24 + 12, 3);
							int stype = getbitu(rtcm->buff, 24 + 12 + 3, 9);
							if (stype == 300)
							{
								v1 = getbitu(rtcm->buff, 85, 25);
								v2 = getbitu(rtcm->buff, 124, 7);
								v3 = getbitu(rtcm->buff, 131, 2);
								v4 = getbitu(rtcm->buff, 135, 3);
							}
						}
						if (ret1==1)
						{
							if (!numofepoch)
							{
								stime = rtcm->time;
							}
							else
							{
								int mm_dt = (int)(timediff(rtcm->time, etime)*1000.0);
								mGAP[mm_dt]++;
							}
							etime = rtcm->time;
							++numofepoch;
							double dt = timediff(buf.time, rtcm->time);

							vDt.push_back(dt);
						}
					}
					if (fRTCM) fwrite(tempBuff, sizeof(char), slen - 2, fRTCM);
				}
				delete[]tempBuff;
			}
			memset(&buf, 0, sizeof(buf_t)); /* reset buffer */
		}
	}

	output_status(vDt, mGAP, numofmsg, numofcrc, numofepoch, stime, etime, v1, v2, v3, v4, fname, is_time);

	if (fLOG) fclose(fLOG);
	if (fRTCM) fclose(fRTCM);

	free_rtcm(rtcm);
	delete rtcm;

	return ret;
}

static int procrtcm(const char* fname, int is_time)
{
	int ret = 0;
	FILE* fLOG = fopen(fname, "rb");
	FILE* fRTCM = nullptr;
	rtcm_t* rtcm = new rtcm_t;
	init_rtcm(rtcm);

	int i = 0;
	std::vector<double> vDt;
	gtime_t stime = { 0 };
	gtime_t etime = { 0 };
	int numofepoch = 0;
	std::map<int, int> mGAP;
	int numofcrc = 0;
	int numofmsg = 0;
	int data = 0;
	gtime_t time_rcv = { 0 };
	int numof4054_1 = 0;
	int v1 = 0;
	int v2 = 0;
	int v3 = 0;
	int v4 = 0;
	while (fLOG && !feof(fLOG) && (data = fgetc(fLOG)) != EOF)
	{
		int ret1 = input_rtcm3(rtcm, data);
		if (rtcm->nbyte == 0 && rtcm->len > 0)
		{
			++numofmsg;
			if (rtk_crc24q(rtcm->buff, rtcm->len) != getbitu(rtcm->buff, rtcm->len * 8, 24))
			{
				++numofcrc;
			}
			int type = getbitu(rtcm->buff, 24, 12);
			if (type == 4054)
			{
				int vers = getbitu(rtcm->buff, 24 + 12, 3);
				int stype = getbitu(rtcm->buff, 24 + 12 + 3, 9);
				//printf("%4i,%3i\n", type, stype);
				if (stype == 1) {
					int week = getbitu(rtcm->buff, 24 + 12 + 3 + 9, 16);
					double ws = getbitu(rtcm->buff, 24 + 12 + 3 + 9 + 16, 32) * 0.001;
					time_rcv = gpst2time(week, ws);
					rtcm->time_s = time_rcv;
					++numof4054_1;
				}
				if (stype == 300)
				{
					v1 = getbitu(rtcm->buff, 85, 25);
					v2 = getbitu(rtcm->buff, 124, 7);
					v3 = getbitu(rtcm->buff, 131, 2);
					v4 = getbitu(rtcm->buff, 135, 3);
				}
			}

		}
		if (ret1 == 1)
		{
			if (!numofepoch)
			{
				stime = rtcm->time;
			}
			else
			{
				int mm_dt = (int)(timediff(rtcm->time, etime) * 1000.0);
				mGAP[mm_dt]++;
			}
			if (numof4054_1)
			{
				double dt = timediff(time_rcv, rtcm->time);
				if (dt < 0 && numofepoch>0) dt = timediff(time_rcv, etime);

				if (dt > 0)
				{
					vDt.push_back(dt);
				}
			}
			etime = rtcm->time;
			++numofepoch;
		}
	}

	output_status(vDt, mGAP, numofmsg, numofcrc, numofepoch, stime, etime, v1, v2, v3, v4, fname, is_time);

	if (fLOG) fclose(fLOG);
	if (fRTCM) fclose(fRTCM);

	free_rtcm(rtcm);
	delete rtcm;

	return ret;
}

int main(int argc, char* argv[])
{
	if (argc < 2)
	{
		printf("log2rtcm file");
	}
	else
	{
		int is_rtcm = 0;
		int is_time = 1;
		int is_geod = 1;
		int is_outp = 1;
		const char* temp = nullptr;
		for (int i = 2; i < argc; ++i)
		{
			if (strstr(argv[i], "rtcm") && (temp = strrchr(argv[i], '=')))
				is_rtcm = atoi(temp + 1);
			if (strstr(argv[i], "time") && (temp = strrchr(argv[i], '=')))
				is_time = atoi(temp + 1);
			if (strstr(argv[i], "geod") && (temp = strrchr(argv[i], '=')))
				is_geod = atoi(temp + 1);
			if (strstr(argv[i], "outp") && (temp = strrchr(argv[i], '=')))
				is_outp = atoi(temp + 1);
		}
		if (is_rtcm)
			procrtcm(argv[1], is_time);
		else
			log2rtcm(argv[1], is_geod, is_outp, is_time);
	}
	return 0;
}
