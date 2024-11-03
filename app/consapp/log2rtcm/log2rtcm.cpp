
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

typedef struct
{
	gtime_t time;
	uint8_t dat[MAX_HEADER_BUFF];
	int slen;
	int nlen;
	int nseg;
	int nloc[4];
}buf_t;

static int add_buff_data(buf_t* buff, int data)
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
		if (buff->nseg < 2)
			buff->nloc[buff->nseg++] = buff->nlen;
		return 0;
	}
	if (buff->nseg < 2) return 0;
	if (data != 0xD3)
	{
		buff->nlen = 0;
		return 0;
	}
	if (buff->slen == 0)
	{
		buff->slen = atoi((char*)(buff->dat + buff->nloc[0]));
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

static int read_buff_from_file(FILE* fBIN, buf_t* buff)
{
	int ret = 0;
	int data = 0;
	while (fBIN && !feof(fBIN) && (data=fgetc(fBIN))!=EOF)
	{
		if (ret = add_buff_data(buff, data)) break;
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

static int log2rtcm(const char *fname)
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
	int numofepoch_3000 = 0;
	std::map<int, int> mGAP;
	int numofcrc = 0;
	int numofmsg = 0;
	int v1 = 0;
	int v2 = 0;
	int v3 = 0;
	int v4 = 0;
	while (fLOG && !feof(fLOG))
	{
		ret = read_buff_from_file(fLOG, &buf);
		if (ret)
		{
			int slen = buf.slen + 2; /* \r\n */
			int ret = 0;
			buf.dat[buf.nloc[1]-1] = '\0';
			//printf("%s,%s\n", time_str(buf.time, 2), (char*)buf.dat);
			if (slen > 2)
			{
				if (!fRTCM)
				{
					double ep[6] = { 0 };
					time2epoch(buf.time, ep);
					char timestr[255] = { 0 };
					sprintf(timestr, "%04i-%02i-%02i-%02i-%02i-%02i.rtcm3", (int)ep[0], (int)ep[1], (int)ep[2], (int)ep[3], (int)ep[4], (int)ep[5]);
					fRTCM = set_output_file(fname, timestr);
				}
				char* tempBuff = new char[slen];
				tempBuff[0] = buf.dat[buf.nloc[1]];
				if (fread((char*)tempBuff+1, sizeof(char), slen-1, fLOG) == (slen-1))
				{
					rtcm->time_s = buf.time;
					for (i = 0; i < slen - 1; ++i)
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

							if (dt > 0)
							{

								if (dt >= 3.0)
									numofepoch_3000++;

								//printf("%s,%10.4f,%3i\n", time_str(buf.time, 3), dt, rtcm->obs.n);
								vDt.push_back(dt);
							}
						}
					}
					if (fRTCM) fwrite(tempBuff, sizeof(char), slen - 2, fRTCM);
				}
				buf.nlen = 0; /* reset the counter */
				delete[]tempBuff;
			}
			buf.dat[buf.nloc[3]] = '\0';
		}
	}
	double dt_avg = 0;
	double dt_68 = 0;
	double dt_95 = 0;
	double dt_99 = 0;
	if (vDt.size() > 0)
	{
		std::sort(vDt.begin(), vDt.end());
		for (i = 0; i < (int)vDt.size(); ++i)
			dt_avg += vDt[i];

		dt_avg /= (int)vDt.size();
		int loc68 = (int)(vDt.size() * 0.6827);
		int loc95 = (int)(vDt.size() * 0.9545);
		int loc99 = (int)(vDt.size() * 0.9973);

		dt_68 = vDt[loc68];
		dt_95 = vDt[loc95];
		dt_99 = vDt[loc99];
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
	printf("%s,%6i,%6i,%6i,%6i,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%6i,%10.4f,%7.2f,%i,%i,%i", fname, numofmsg, numofcrc, numofepoch, numofepoch_3000, dt_avg, dt_68, dt_95, dt_99, vDt.front(), vDt.back(), (int)vDt.size(), timediff(etime, stime), v1 / 3600.0 / 24.0, -v2, v3, v4);
	if (fSUM) fprintf(fSUM, "%s,%6i,%6i,%6i,%6i,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%6i,%10.4f,%7.2f,%i,%i,%i", fname, numofmsg, numofcrc, numofepoch, numofepoch_3000, dt_avg, dt_68, dt_95, dt_99, vDt.front(), vDt.back(), (int)vDt.size(), timediff(etime, stime), v1 / 3600.0 / 24.0, -v2, v3, v4);
	for (std::map<int,int>::iterator pGAP=mGAP.begin();pGAP!=mGAP.end();++pGAP)
	{
		if (fSUM) fprintf(fSUM,",%i(%i)", pGAP->first, pGAP->second);
		printf(",%i(%i)", pGAP->first, pGAP->second);
	}
	printf("\n");
	if (fSUM) fprintf(fSUM, "\n");
	if (fSUM) fclose(fSUM);
	if (fLOG) fclose(fLOG);
	if (fRTCM) fclose(fRTCM);

	free_rtcm(rtcm);
	delete rtcm;

	return ret;
}

static int procrtcm(const char* fname)
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
	int numofepoch_3000 = 0;
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
					if (dt >= 3.0) numofepoch_3000++;
					//printf("%s,%10.4f,%3i\n", time_str(buf.time, 3), dt, rtcm->obs.n);

					vDt.push_back(dt);
				}
			}
			etime = rtcm->time;
			++numofepoch;
		}
	}
	double dt_avg = 0;
	double dt_68 = 0;
	double dt_95 = 0;
	double dt_99 = 0;
	if (vDt.size() > 0)
	{
		std::sort(vDt.begin(), vDt.end());
		for (i = 0; i < (int)vDt.size(); ++i)
			dt_avg += vDt[i];

		dt_avg /= (int)vDt.size();
		int loc68 = (int)(vDt.size() * 0.6827);
		int loc95 = (int)(vDt.size() * 0.9545);
		int loc99 = (int)(vDt.size() * 0.9973);

		dt_68 = vDt[loc68];
		dt_95 = vDt[loc95];
		dt_99 = vDt[loc99];
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
		fprintf(fSUM, "filename,numofmsg,numofcrc_failure,numofepoch,numofepoch_3s,dt_avg,dt_68,dt_95,dt_99,dt_min,dt_max,dt_count,time_duration,v1,v2,v3,v4,data_interval_counts\n");
	}
	printf("%s,%6i,%6i,%6i,%6i,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%6i,%10.4f,%7.2f,%i,%i,%i", fname, numofmsg, numofcrc, numofepoch, numofepoch_3000, dt_avg, dt_68, dt_95, dt_99, vDt.front(), vDt.back(), (int)vDt.size(), timediff(etime, stime), v1/3600.0/24.0, -v2, v3, v4);
	if (fSUM) fprintf(fSUM, "%s,%6i,%6i,%6i,%6i,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%6i,%10.4f,%7.2f,%i,%i,%i", fname, numofmsg, numofcrc, numofepoch, numofepoch_3000, dt_avg, dt_68, dt_95, dt_99, vDt.front(), vDt.back(), (int)vDt.size(), timediff(etime, stime), v1 / 3600.0 / 24.0, -v2, v3, v4);
	for (std::map<int, int>::iterator pGAP = mGAP.begin(); pGAP != mGAP.end(); ++pGAP)
	{
		if (fSUM) fprintf(fSUM, ",%i(%i)", pGAP->first, pGAP->second);
		printf(",%i(%i)", pGAP->first, pGAP->second);
	}
	printf("\n");
	if (fSUM) fprintf(fSUM, "\n");
	if (fSUM) fclose(fSUM);
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
	}
	else if (argc < 3)
	{
		log2rtcm(argv[1]);
	}
	else if (strstr(argv[1], "rtcm"))
	{
		procrtcm(argv[2]);
	}
}
