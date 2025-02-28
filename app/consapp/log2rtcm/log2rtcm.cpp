
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

int parse_fields(char* const buffer, char** val, char key, int max_field)
{
	char* p, * q;
	int n = 0;

	/* parse fields */
	for (p = buffer; *p && n < max_field; p = q + 1) {
		if (p == NULL) break;
		if ((q = strchr(p, key)) || (q = strchr(p, '\n')) || (q = strchr(p, '\r'))) {
			val[n++] = p; *q = '\0';
		}
		else break;
	}
	if (p) val[n++] = p;
	return n;
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

static int output_status(std::vector<double>& vDt, std::map<int, int>& mGAP, int numofmsg, int numofcrc, int numofepoch, gtime_t stime, gtime_t etime, int v1, int v2, int v3, int v4, const char *fname, int is_time, double* rate, char *ver, char *gnss)
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
		fprintf(fSUM, "filename,numofmsg,numofcrc_failure,numofepoch,numofepoch_3s,dt_avg,dt_68,dt_95,dt_99,dt_min,dt_max,dt_count,time_duration,sat_avail_0deg,sat_avail_5deg,sat_avail_7deg,sat_avail_10deg,data_interval_counts\n");
	}
	printf("%s,%6i,%6i,%6i,%6i,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%6i,%10.4f,%7.2f,%4i,%i,%i,%8.4f,%8.4f,%8.4f,%8.4f,%s,%s", fname, numofmsg, numofcrc, numofepoch, numofepoch_3s, dt_avg, dt_68, dt_95, dt_99, dt_min, dt_max, (int)vDt.size(), timediff(etime, stime), v1 / 3600.0 / 24.0, -v2, v3, v4, rate[0], rate[1], rate[2], rate[3], ver, gnss);
	if (fSUM) fprintf(fSUM, "%s,%6i,%6i,%6i,%6i,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%6i,%10.4f,%7.2f,%4i,%i,%i,%8.4f,%8.4f,%8.4f,%8.4f,%s,%s", fname, numofmsg, numofcrc, numofepoch, numofepoch_3s, dt_avg, dt_68, dt_95, dt_99, dt_min, dt_max, (int)vDt.size(), timediff(etime, stime), v1 / 3600.0 / 24.0, -v2, v3, v4, rate[0], rate[1], rate[2], rate[3], ver, gnss);
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

#ifndef SQ	/* square */
#define SQ(x) ( (x)*(x) )
#endif

const char* sys2char3(int sys, int prn)
{
	if (sys == SYS_GPS) return "GPS";
	else if (sys == SYS_GLO) return "GLO";
	else if (sys == SYS_GAL) return "GAL";
	else if (sys == SYS_CMP) return prn > 0 ? (prn <= 16 ? "BD2" : "BD3") : "BDS";
	else if (sys == SYS_QZS) return "QZS";
	else if (sys == SYS_SBS) return "SBS";
	else if (sys == SYS_IRN) return "IRN";
	else return "UKN";
}
char sys2char(int sys, int prn)
{
	if (sys == SYS_GPS) return 'G';
	else if (sys == SYS_GLO) return 'R';
	else if (sys == SYS_GAL) return 'E';
	else if (sys == SYS_CMP) return prn > 0 ? (prn <= 16 ? '2' : 'C') : 'C';
	else if (sys == SYS_QZS) return 'J';
	else if (sys == SYS_SBS) return 'S';
	else if (sys == SYS_IRN) return 'I';
	else return 'U';
}

typedef struct {
	int sat;
	gtime_t time;
	double rs[6];
	double dts[2];
	double var;
	double age;
	int svh;
}spvt_t;

bool operator==(const spvt_t& t1, int t2) { return t1.sat == t2; }
bool operator==(const spvt_t& t1, const spvt_t& t2) { return t1.sat==t2.sat; }

bool operator==(const gtime_t& t1, const gtime_t& t2) { return fabs(timediff(t1, t2)) < DTTOL; }
bool operator!=(const gtime_t& t1, const gtime_t& t2) { return !(t1 == t2); }
bool operator< (const gtime_t& t1, const gtime_t& t2) { return timediff(t1, t2) < -DTTOL; }

bool operator==(const obsd_t& t1, int sat) { return t1.sat == sat; }
bool operator==(const obsd_t& t1, const obsd_t& t2) { return t1.time == t2.time && t1.sat == t2.sat; }
bool operator!=(const obsd_t& t1, const obsd_t& t2) { return !(t1 == t2); }
bool operator< (const obsd_t& t1, const obsd_t& t2) { return t1.time == t2.time ? t1.sat < t2.sat : t1.time < t2.time; }
bool valid_pos(const double* pos) { return !(sqrt(pos[0] * pos[0] + pos[1] * pos[1] + pos[2] * pos[2]) < 0.0001); }

bool operator==(const eph_t& t1, const eph_t& t2) { return t1.sat == t2.sat && t1.toe == t2.toe; }
bool operator==(const geph_t& t1, const geph_t& t2) { return t1.sat == t2.sat && t1.toe == t2.toe; }
bool operator< (const eph_t& t1, const eph_t& t2) { return t1.sat == t2.sat ? t1.toe < t2.toe : false; }
bool operator< (const geph_t& t1, const geph_t& t2) { return t1.sat == t2.sat ? t1.toe < t2.toe : false; }

bool operator< (const eph_t& t1, const gtime_t& t2) { return t1.toe < t2; }
bool operator< (const geph_t& t1, const gtime_t& t2) { return t1.toe < t2; }

#define DTGPS_MEO (0.067213)     /* transimition time delay for GPS MEO */
#define DTGLO_MEO (0.077230)     /* transimition time delay for GLO MEO */
#define DTQZS_GEO (0.119652)     /* transimition time delay for QZS GEO & IGSO */
#define DTGAL_MEO (0.077730)     /* transimition time delay for GAL MEO */
#define DTBD2_MEO (0.071776)     /* transimition time delay for BD2 MEO 11,12,14 */
#define DTBD3_MEO (0.071840)     /* transimition time delay for BD3 MEO */
#define DTBD2_GEO (0.119258)     /* transimition time delay for BD2 GEO & IGSO 1,2,3,4,5,6,7,8,9,10,13,16 */
#define DTBD3_GEO (0.119347)     /* transimition time delay for BD3 GEO & IGSO 38,39,40,59,60,62 */

#ifndef STD_BRDCCLK
#define STD_BRDCCLK 30.0          /* error of broadcast clock (m) */
#endif

double default_sat_dt(int sat)
{
	int prn = 0;
	int sys = satsys(sat, &prn);
	double dt = DTGPS_MEO;
	if (sys == SYS_GPS)
		dt = DTGPS_MEO;
	else if (sys == SYS_GLO)
		dt = DTGLO_MEO;
	else if (sys == SYS_GAL)
		dt = DTGAL_MEO;
	else if (sys == SYS_QZS)
		dt = DTQZS_GEO;
	else if (sys == SYS_CMP)
	{
		if (prn == 11 || prn == 12 || prn == 14)
			dt = DTBD2_MEO;
		else if (prn == 38 || prn == 39 || prn == 40 || prn == 59 || prn == 60 || prn == 62)
			dt = DTBD3_GEO;
		else if (prn <= 16)
			dt = DTBD2_GEO;
		else
			dt = DTBD3_MEO;
	}
	return dt;
}

static int cal_sat_pos_vel(gtime_t time, int sat, nav_t* nav, spvt_t* spvt)
{
	int ret = 0;
	gtime_t teph = time;
	double dt = default_sat_dt(sat);

	spvt->sat = sat;

	/* transmission time by satellite clock */
	time = timeadd(time, -dt);

	/* satellite clock bias by broadcast ephemeris */
	if (!ephclk(time, teph, sat, nav, &dt)) {
		trace(3, "no broadcast clock %s sat=%2d\n", time_str(time, 3), sat);
		return ret;
	}
	time = timeadd(time, -dt);
	/* ephpos     (time,teph,sat,nav,-1,rs,dts,var,svh) */

	/* satellite position and clock at transmission time */
	if (!ephpos(time, teph, sat, nav, -1, spvt->rs, spvt->dts, &spvt->var, &spvt->svh)) {
		trace(3, "no ephemeris %s sat=%2d\n", time_str(time, 3), sat);
		return ret;
	}
	/* if no precise clock available, use broadcast clock instead */
	if (spvt->dts[0] == 0.0) {
		if (!ephclk(time, teph, sat, nav, spvt->dts)) return ret;
		spvt->dts[1] = 0.0;
		spvt->var = SQ(STD_BRDCCLK);
	}
	spvt->time = time;
	ret = 1;
	return ret;
}

static int cal_sat_pos(obsd_t* obs, int n, nav_t* nav, double* rr, FILE* fCSV)
{
	int ret = 0;
	int i = 0;
	int j = 0;

	double* rs = nullptr;
	double* dts = nullptr;
	double* var = nullptr;
	double* azel = nullptr;
	int* svh = nullptr;
	double pos[3] = { 0 };
	double r = 0;
	double e[3] = { 0 };
	int sat = 0;
	int prn = 0;
	int sys = 0;
	obsd_t tobs = { 0 };

	for (i = 0; i < n; ++i)
	{
		for (j = i + 1; j < n; ++j)
		{
			if (obs[j].sat < obs[i].sat)
			{
				/* switch */
				tobs = obs[j];
				obs[j] = obs[i];
				obs[i] = tobs;
			}
		}
	}

	if (n > 0 && !(fabs(rr[0]) < 0.001 || fabs(rr[1]) < 0.001 || fabs(rr[2]) < 0.001))
	{
		rs = new double[n * 6];
		dts = new double[n * 2];
		var = new double[n];
		azel = new double[n * 2];
		svh = new int[n];

		ecef2pos(rr, pos);

		satposs(obs[0].time, obs, n, nav, EPHOPT_BRDC, rs, dts, var, svh);

		if (fCSV)
		{
			for (i = 0; i < n; ++i)
			{
				if (fabs(rs[i * 6 + 0]) < 0.001 || fabs(rs[i * 6 + 1]) < 0.001 || fabs(rs[i * 6 + 2]) < 0.001) continue;
				sat = obs[i].sat;
				sys = satsys(sat, &prn);
				r = geodist(rs + i * 6, rr, e);
				if (r < 0.01) continue;
				satazel(pos, e, azel + i * 2);
				fprintf(fCSV, "%3i,%2i,%2i,%7.3f,%7.3f\n", sat, sys, prn, azel[i * 2 + 0] * R2D, azel[i * 2 + 1] * R2D);
			}
		}

		delete[] rs;
		delete[] dts;
		delete[] var;
		delete[] azel;
		delete[] svh;
	}
	return ret;
}

struct brdc_t
{
	std::map<int, std::vector<eph_t>> mSatEph;
	std::map<int, std::vector<eph_t>> mGalEph;
	std::map<int, std::vector<geph_t>> mGloEph;
	brdc_t()
	{

	}
	~brdc_t()
	{

	}
	int add_sat_eph(const eph_t& eph)
	{
		int ret = 0;
		std::map<int, std::vector<eph_t> >::iterator pEphMap = mSatEph.find(eph.sat);
		if (pEphMap != mSatEph.end())
		{
			std::vector<eph_t>::iterator pSatEph = std::find(pEphMap->second.begin(), pEphMap->second.end(), eph);
			if (pSatEph == pEphMap->second.end())
			{
				ret = 1;
				pEphMap->second.push_back(eph);
				std::sort(pEphMap->second.begin(), pEphMap->second.end());
			}
			else
			{
				//*pSatEph = eph;
				ret =-1;
			}
		}
		else if (eph.sat > 0)
		{
			ret = 2;
			mSatEph[eph.sat].push_back(eph);
		}
		return ret;
	}
	int add_gal_eph(const eph_t& eph)
	{
		int ret = 0;
		std::map<int, std::vector<eph_t> >::iterator pEphMap = mGalEph.find(eph.sat);
		if (pEphMap != mGalEph.end())
		{
			std::vector<eph_t>::iterator pSatEph = std::find(pEphMap->second.begin(), pEphMap->second.end(), eph);
			if (pSatEph == pEphMap->second.end())
			{
				ret = 1;
				pEphMap->second.push_back(eph);
				std::sort(pEphMap->second.begin(), pEphMap->second.end());
			}
			else
			{
				//*pSatEph = eph;
				ret = -1;
			}
		}
		else if (eph.sat > 0)
		{
			ret = 2;
			mGalEph[eph.sat].push_back(eph);
		}
		return ret;
	}
	int add_glo_eph(const geph_t& eph)
	{
		int ret = 0;
		std::map<int, std::vector<geph_t> >::iterator pEphMap = mGloEph.find(eph.sat);
		if (pEphMap != mGloEph.end())
		{
			std::vector<geph_t>::iterator pSatEph = std::find(pEphMap->second.begin(), pEphMap->second.end(), eph);
			if (pSatEph == pEphMap->second.end())
			{
				ret = 1;
				pEphMap->second.push_back(eph);
				std::sort(pEphMap->second.begin(), pEphMap->second.end());
			}
			else
			{
				//*pSatEph = eph;
				ret = -1;
			}
		}
		else if (eph.sat > 0)
		{
			ret = 2;
			mGloEph[eph.sat].push_back(eph);
		}
		return ret;
	}
	int add_nav_eph(rtcm_t* rtcm)
	{
		int ret = 0;
		int prn = 0;
		int sys = 0;
		int sat = rtcm->ephsat;
		if (sat > 0 && (sys = satsys(sat, &prn)))
		{
			if (sys == SYS_GLO)
				ret = add_glo_eph(rtcm->nav.geph[prn - 1]);
			else if (rtcm->ephset && sys == SYS_GAL)
				ret = add_gal_eph(rtcm->nav.eph[sat - 1 + MAXSAT]);
			else
				ret = add_sat_eph(rtcm->nav.eph[sat - 1]);
		}
		return ret;
	}
	int update_sat_eph_nav(gtime_t time, nav_t& nav)
	{
		int ret = 0;
		int sat = 0;
		std::map<int, std::vector<eph_t> >::iterator pEphMap = mSatEph.begin();
		while (pEphMap != mSatEph.end())
		{
			sat = pEphMap->first;
			if (pEphMap->second.size() > 0)
			{
				std::vector<eph_t>::iterator pSatEph = std::lower_bound(pEphMap->second.begin(), pEphMap->second.end(), time);
				if (pSatEph == pEphMap->second.end())
				{
					if (pSatEph != pEphMap->second.begin())
					{
						pSatEph--;
						nav.eph[sat - 1] = *pSatEph;
					}
				}
				else if (pSatEph == pEphMap->second.begin())
				{
					nav.eph[sat - 1] = *pSatEph;
				}
				else
				{
					double dt0 = timediff(time, (pSatEph - 1)->toe);
					double dt1 = timediff(time, pSatEph->toe);
					if (fabs(dt0) < fabs(dt1))
					{
						nav.eph[sat - 1] = *(pSatEph-1);
					}
					else
					{
						nav.eph[sat - 1] = *pSatEph;
					}
				}
			}
			++pEphMap;
		}
		return ret;
	}
	int update_gal_eph_nav(gtime_t time, nav_t& nav)
	{
		int ret = 0;
		int sat = 0;
		int prn = 0;
		int sys = 0;
		std::map<int, std::vector<eph_t> >::iterator pEphMap = mGalEph.begin();
		while (pEphMap != mGalEph.end())
		{
			sat = pEphMap->first;
			sys = satsys(sat, &prn);
			if (sys==SYS_GAL && pEphMap->second.size() > 0)
			{
				std::vector<eph_t>::iterator pSatEph = std::lower_bound(pEphMap->second.begin(), pEphMap->second.end(), time);
				if (pSatEph == pEphMap->second.end())
				{
					if (pSatEph != pEphMap->second.begin())
					{
						pSatEph--;
						nav.eph[sat - 1 + MAXSAT] = *pSatEph;
					}
				}
				else if (pSatEph == pEphMap->second.begin())
				{
					nav.eph[sat - 1 + MAXSAT] = *pSatEph;
				}
				else
				{
					double dt0 = timediff(time, (pSatEph - 1)->toe);
					double dt1 = timediff(time, pSatEph->toe);
					if (fabs(dt0) < fabs(dt1))
					{
						nav.eph[sat - 1 + MAXSAT] = *(pSatEph - 1);
					}
					else
					{
						nav.eph[sat - 1 + MAXSAT] = *pSatEph;
					}
				}
			}
			++pEphMap;
		}
		return ret;
	}
	int update_glo_eph_nav(gtime_t time, nav_t& nav)
	{
		int ret = 0;
		int sat = 0;
		int sys = 0;
		int prn = 0;
		std::map<int, std::vector<geph_t> >::iterator pEphMap = mGloEph.begin();
		while (pEphMap != mGloEph.end())
		{
			sat = pEphMap->first;
			sys = satsys(sat, &prn);
			if (sys==SYS_GLO && pEphMap->second.size() > 0)
			{
				std::vector<geph_t>::iterator pSatEph = std::lower_bound(pEphMap->second.begin(), pEphMap->second.end(), time);
				if (pSatEph == pEphMap->second.end())
				{
					if (pSatEph != pEphMap->second.begin())
					{
						pSatEph--;
						nav.geph[prn - 1] = *pSatEph;
					}
				}
				else if (pSatEph == pEphMap->second.begin())
				{
					nav.geph[prn - 1] = *pSatEph;
				}
				else
				{
					double dt0 = timediff(time, (pSatEph - 1)->toe);
					double dt1 = timediff(time, pSatEph->toe);
					if (fabs(dt0) < fabs(dt1))
					{
						nav.geph[prn - 1] = *(pSatEph - 1);
					}
					else
					{
						nav.geph[prn - 1] = *pSatEph;
					}
				}
			}
			++pEphMap;
		}
		return ret;
	}
	int update_eph_nav(gtime_t time, nav_t& nav)
	{
		int ret = 0;
		if (update_sat_eph_nav(time, nav)) ret++;
		if (update_gal_eph_nav(time, nav)) ret++;
		if (update_glo_eph_nav(time, nav)) ret++;
		return ret;
	}
};

struct epoch_t
{
	gtime_t time;
	double  pos[3];
	std::vector<obsd_t> obsd;
	epoch_t()
	{
		memset(&time, 0, sizeof(gtime_t));
		pos[0] = pos[1] = pos[2] = 0;
	}
	~epoch_t()
	{

	}
	bool operator<(const epoch_t& src)
	{
		return time < src.time;
	}
	bool operator==(const epoch_t& src)
	{
		return time == src.time;
	}
	bool operator==(const gtime_t& src)
	{
		return time == src;
	}
	int nsat() { return (int)obsd.size(); }
	int add(const obsd_t* obs, int n)
	{
		int ret = 0;
		int i = 0;
		if (n > 0)
		{
			if (obs[0].time != time)
			{
				obsd.clear();
				time = obs[0].time;
			}
			for (i = 0; i < n; ++i)
			{
				if (std::find(obsd.begin(), obsd.end(), obs[i]) == obsd.end())
				{
					obsd.push_back(obs[i]);
					ret++;
				}
			}
		}
		if (ret) std::sort(obsd.begin(), obsd.end());
		return ret;
	}
	int add(rtcm_t* rtcm)
	{
		int ret = 0;
		return ret;
	}
	int cal_sat_pos_all(nav_t* nav, std::map<int, spvt_t>& mSatPVT)
	{
		int ret = 0;
		int i = 0;
		eph_t* eph = nullptr;
		geph_t *geph = nullptr;
		/* GPS, GAL, BDS, CMP */
		for (i = 0, eph = nav->eph + i; i < MAXSAT + MAXSAT; ++i, ++eph)
		{
			spvt_t spvt = { 0 };
			if (cal_sat_pos_vel(time, eph->sat, nav, &spvt))
			{
				mSatPVT[spvt.sat] = spvt;
				ret++;
			}
		}
		/* GLO */
		for (i = 0, geph = nav->geph + i; i < MAXPRNGLO; ++i, ++geph)
		{
			spvt_t spvt = { 0 };
			if (cal_sat_pos_vel(time, geph->sat, nav, &spvt))
			{
				mSatPVT[spvt.sat] = spvt;
				ret++;
			}
		}
		return ret;
	}
};
static int update_epoch_obs(std::vector<epoch_t>& vEpoch, rtcm_t* rtcm)
{
	int ret = 0;
	/* store the data for further process */
	if (rtcm->obs.n > 0)
	{
		std::vector<epoch_t>::iterator pEpoch = std::find(vEpoch.begin(), vEpoch.end(), rtcm->obs.data[0].time);
		if (pEpoch == vEpoch.end())
		{
			epoch_t epoch;
			if (epoch.add(rtcm->obs.data, rtcm->obs.n))
			{
				if (valid_pos(rtcm->sta.pos))
				{
					epoch.pos[0] = rtcm->sta.pos[0];
					epoch.pos[1] = rtcm->sta.pos[1];
					epoch.pos[2] = rtcm->sta.pos[2];
				}
				vEpoch.push_back(epoch);
				ret++;
			}
		}
		else
		{
			if (pEpoch->add(rtcm->obs.data, rtcm->obs.n))
			{
				if (valid_pos(rtcm->sta.pos))
				{
					if (!valid_pos(pEpoch->pos))
					{
						pEpoch->pos[0] = rtcm->sta.pos[0];
						pEpoch->pos[1] = rtcm->sta.pos[1];
						pEpoch->pos[2] = rtcm->sta.pos[2];
					}
					else
					{

					}
				}
				ret++;
			}
		}
		rtcm->obs.n = 0;
	}
	return ret;
}
static int update_epoch_pos(std::vector<epoch_t>& vEpoch, rtcm_t* rtcm)
{
	int ret = 0;
	if (valid_pos(rtcm->sta.pos))
	{
		for (std::vector<epoch_t>::reverse_iterator pEpoch = vEpoch.rbegin(); pEpoch != vEpoch.rend(); ++pEpoch)
		{
			if (valid_pos(pEpoch->pos))
			{
				break;
			}
			else
			{
				pEpoch->pos[0] = rtcm->sta.pos[0];
				pEpoch->pos[1] = rtcm->sta.pos[1];
				pEpoch->pos[2] = rtcm->sta.pos[2];
			}
		}
	}
	return ret;
}
static int proc_epoch_data(std::vector<epoch_t>& vEpoch, brdc_t &brdc, nav_t *nav, FILE *fSAT, FILE *fCSV, double *rate, int is_heph)
{
	int ret = 0;
	double ep[6] = { 0 };
	std::vector<double> rate0, rate5, rate7, rate10;
	for (std::vector<epoch_t>::iterator pEpoch = vEpoch.begin(); pEpoch != vEpoch.end(); ++pEpoch)
	{
		time2epoch(pEpoch->time, ep);
		int wk = 0;
		double ws = time2gpst(pEpoch->time, &wk);

		brdc.update_eph_nav(pEpoch->time, *nav);
		/* compute satellite computer */
		std::map<int, spvt_t> mSatPVT;
		if (valid_pos(pEpoch->pos) && pEpoch->cal_sat_pos_all(nav, mSatPVT))
		{
			double* rr = pEpoch->pos;
			double pos[3] = { 0 }; /* blh */
			double e[3] = { 0 };
			double azel[2] = { 0 };
			ecef2pos(rr, pos);
			std::vector<int> mSatElev0, mSatElev5, mSatElev7, mSatElev10;
			std::vector<int> mSatElev0_, mSatElev5_, mSatElev7_, mSatElev10_;
			int sat0 = 0, sat5 = 0, sat7 = 0, sat10 = 0, sat15 = 0;
			for (std::map<int, spvt_t>::iterator pSatPVT = mSatPVT.begin(); pSatPVT != mSatPVT.end(); ++pSatPVT)
			{
				spvt_t* spvt = &pSatPVT->second;
				double* rs = spvt->rs;
				if (!valid_pos(rs)) continue;
				int prn = 0;
				int sat = spvt->sat;
				int sys = satsys(sat, &prn);
				double r = geodist(rs, rr, e);
				if (r > 0.01)
				{
					satazel(pos, e, azel);
					//printf("%3i,%s,%2i,%7.3f,%8.3f\n", sat, sys2char3(sys,prn), prn, azel[0] * R2D, azel[1] * R2D);
					std::vector<obsd_t>::iterator pobs = std::find(pEpoch->obsd.begin(), pEpoch->obsd.end(), sat);
					if (fCSV) fprintf(fCSV, "%04i,%11.4f,%04i,%02i,%02i,%02i,%02i,%7.4f,%3i,%s,%3i,%8.3f,%8.3f,%3i,%c\n", wk, ws, (int)ep[0], (int)ep[1], (int)ep[2], (int)ep[3], (int)ep[4], ep[5], sat, sys2char3(sys, prn), prn, azel[0] * R2D, azel[1] * R2D, spvt->svh, pobs != pEpoch->obsd.end() ? '+' : '-');
					if ((sys == SYS_GPS || sys == SYS_GLO || sys == SYS_GAL || sys == SYS_CMP) && ((is_heph && !spvt->svh) || !is_heph))
					{
						if (azel[1] >= (0.0 * D2R))
						{
							mSatElev0.push_back(sat);
							if (pobs != pEpoch->obsd.end()) ++sat0; else mSatElev0_.push_back(sat);
						}
						if (azel[1] >= (5.0 * D2R))
						{
							mSatElev5.push_back(sat);
							if (pobs != pEpoch->obsd.end()) ++sat5; else mSatElev5_.push_back(sat);
						}
						if (azel[1] >= (7.0 * D2R))
						{
							mSatElev7.push_back(sat);
							if (pobs != pEpoch->obsd.end()) ++sat7; else mSatElev7_.push_back(sat);
						}
						if (azel[1] >= (10.0 * D2R))
						{
							mSatElev10.push_back(sat);
							if (pobs != pEpoch->obsd.end()) ++sat10; else mSatElev10_.push_back(sat);
						}
					}
				}
			}

			if (fSAT) fprintf(fSAT, "%04i,%11.4f,%02i,%02i,%7.4f,%3i,%3i,%3i,%3i,%3i,%3i,%3i,%3i,%8.4f,%8.4f,%8.4f,%8.4f", wk, ws, (int)ep[3], (int)ep[4], ep[5], (int)mSatElev0.size(), sat0, (int)mSatElev5.size(), sat5, (int)mSatElev7.size(), sat7, (int)mSatElev10.size(), sat10
				, (int)mSatElev0.size() > 0 ? sat0 * 100.0 / (int)mSatElev0.size() : 0
				, (int)mSatElev5.size() > 0 ? sat5 * 100.0 / (int)mSatElev5.size() : 0
				, (int)mSatElev7.size() > 0 ? sat7 * 100.0 / (int)mSatElev7.size() : 0
				, (int)mSatElev10.size() > 0 ? sat10 * 100.0 / (int)mSatElev10.size() : 0
			);

			if (fSAT) fprintf(fSAT, ",");
			for (int i = 0; i < (int)mSatElev0_.size(); ++i)
			{
				int sat = mSatElev0_[i];
				int prn = 0;
				int sys = satsys(sat, &prn);
				if (i == 0)
				{
					if (fSAT) fprintf(fSAT, "%c%02i", sys2char(sys, prn), prn);
				}
				else
				{
					if (fSAT) fprintf(fSAT, ";%c%02i", sys2char(sys, prn), prn);
				}
			}
			if (fSAT) fprintf(fSAT, ",");
			for (int i = 0; i < (int)mSatElev5_.size(); ++i)
			{
				int sat = mSatElev5_[i];
				int prn = 0;
				int sys = satsys(sat, &prn);
				if (i == 0)
				{
					if (fSAT) fprintf(fSAT, "%c%02i", sys2char(sys, prn), prn);
				}
				else
				{
					if (fSAT) fprintf(fSAT, ";%c%02i", sys2char(sys, prn), prn);
				}
			}
			if (fSAT) fprintf(fSAT, ",");
			for (int i = 0; i < (int)mSatElev7_.size(); ++i)
			{
				int sat = mSatElev7_[i];
				int prn = 0;
				int sys = satsys(sat, &prn);
				if (i == 0)
				{
					if (fSAT) fprintf(fSAT, "%c%02i", sys2char(sys, prn), prn);
				}
				else
				{
					if (fSAT) fprintf(fSAT, ";%c%02i", sys2char(sys, prn), prn);
				}
			}
			if (fSAT) fprintf(fSAT, ",");
			for (int i = 0; i < (int)mSatElev10_.size(); ++i)
			{
				int sat = mSatElev10_[i];
				int prn = 0;
				int sys = satsys(sat, &prn);
				if (i == 0)
				{
					if (fSAT) fprintf(fSAT, "%c%02i", sys2char(sys, prn), prn);
				}
				else
				{
					if (fSAT) fprintf(fSAT, ";%c%02i", sys2char(sys, prn), prn);
				}
			}
			if (fSAT) fprintf(fSAT, "\n");
			if ((int)mSatElev0.size() > 0) rate0.push_back(sat0 * 100.0 / (int)mSatElev0.size());
			if ((int)mSatElev5.size() > 0) rate5.push_back(sat5 * 100.0 / (int)mSatElev5.size());
			if ((int)mSatElev7.size() > 0) rate7.push_back(sat7 * 100.0 / (int)mSatElev7.size());
			if ((int)mSatElev10.size() > 0) rate10.push_back(sat10 * 100.0 / (int)mSatElev10.size());

		}
	}
	std::sort(rate0.begin(), rate0.end());
	std::sort(rate5.begin(), rate5.end());
	std::sort(rate7.begin(), rate7.end());
	std::sort(rate10.begin(), rate10.end());
	if (rate0.size() > 0) rate[0] = rate0[(int)(rate0.size() * 0.68)]; else rate[0] = 0;
	if (rate5.size() > 0) rate[1] = rate5[(int)(rate5.size() * 0.68)]; else rate[1] = 0;
	if (rate7.size() > 0) rate[2] = rate7[(int)(rate7.size() * 0.68)]; else rate[2] = 0;
	if (rate10.size() > 0) rate[3] = rate10[(int)(rate10.size() * 0.68)]; else rate[3] = 0;
	if (fSAT) fflush(fSAT);
	if (fCSV) fflush(fCSV);
	return ret;
}
static int log2rtcm(const char *fname, int flag, std::string brdcfname)
{
	int ret = 0;
	FILE *fLOG = fopen(fname, "rb");
	FILE *fRTCM = nullptr;
	rtcm_t* rtcm = new rtcm_t;
	init_rtcm(rtcm);
	
	buf_t buf = { 0 };
	int i = 0;
	int j = 0;
	std::vector<double> vDt;
	gtime_t stime = { 0 };
	gtime_t etime = { 0 };
	int numofepoch = 0;
	std::map<int, int> mGAP;
	int numofbyte = 0;
	int numofcrc = 0;
	int numofmsg = 0;
	int v1 = 0;
	int v2 = 0;
	int v3 = 0;
	int v4 = 0;
	brdc_t brdc;
	std::vector<epoch_t> vEpoch;
	/* read brdc data first */
	FILE* fBRDC = fopen(brdcfname.c_str(), "rb");
	FILE* fCSV = flag & (1 << 2) ? set_output_file(fname, "sat-pos.csv") : NULL;
	FILE* fSAT = flag & (1 << 3) ? set_output_file(fname, "sat-inf.csv") : NULL;
	while (fBRDC && !feof(fBRDC))
	{
		ret = read_buff_from_file(fBRDC, &buf, flag & (1 << 6) ? 3 : 2);
		if (ret)
		{
			int slen = buf.slen + 2; /* \r\n */
			int ret = 0;
			if (slen > 2)
			{
				char* tempBuff = new char[slen];
				if (fread((char*)tempBuff, sizeof(char), slen, fBRDC) == slen)
				{
					rtcm->time_s = buf.time;
					for (i = 0; i < slen - 2; ++i)
					{
						int ret1 = input_rtcm3(rtcm, tempBuff[i]);
						if (ret1 == 2) brdc.add_nav_eph(rtcm);
					}
				}
				delete[]tempBuff;
			}
			memset(&buf, 0, sizeof(buf_t)); /* reset buffer */
		}
	}
	/* read log file */
	numofcrc = 0;
	while (fLOG && !feof(fLOG))
	{
		ret = read_buff_from_file(fLOG, &buf, flag & (1 << 6) ? 3 : 2);
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
						if (ret1 == 2) brdc.add_nav_eph(rtcm);
						numofbyte++;
						if (rtcm->nbyte == 0 && rtcm->len > 0)
						{
							if (rtk_crc24q(rtcm->buff, rtcm->len) != getbitu(rtcm->buff, rtcm->len * 8, 24))
							{
								numofcrc++;
							}
							else
							{
								++numofmsg;
							}
							rtcm->len = 0;
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
							etime = rtcm->time;
							++numofepoch;
							double dt = timediff(buf.time, rtcm->time);

							vDt.push_back(dt);
						}
						/* store the data for further process */
						update_epoch_obs(vEpoch, rtcm);
						if (ret1 == 5) update_epoch_pos(vEpoch, rtcm);
						/* done */
					}
					if (fRTCM) fwrite(tempBuff, sizeof(char), slen - 2, fRTCM);
				}
				delete[]tempBuff;
			}
			memset(&buf, 0, sizeof(buf_t)); /* reset buffer */
		}
	}
	double rate[4] = { 0 };

	proc_epoch_data(vEpoch, brdc, &rtcm->nav, fSAT, fCSV, rate, flag & (1 << 1));

	output_status(vDt, mGAP, numofmsg, numofcrc, numofepoch, stime, etime, v1, v2, v3, v4, fname, flag & (1 << 5), rate, rtcm->sta.recver, rtcm->sta.rectype);

	if (fLOG) fclose(fLOG);
	if (fRTCM) fclose(fRTCM);
	if (fBRDC) fclose(fBRDC);
	if (fCSV) fclose(fCSV);

	free_rtcm(rtcm);
	delete rtcm;

	return ret;
}

static int procrtcm(const char* fname, int flag, std::string brdcfname, int year, int mon, int day)
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
	std::vector<epoch_t> vEpoch;
	epoch_t epoch;
	/* read brdc data first */
	FILE* fBRDC = fopen(brdcfname.c_str(), "rb");
	FILE* fCSV = flag & (1 << 2) ? set_output_file(fname, "sat-pos.csv") : NULL;
	FILE* fSAT = flag & (1 << 3) ? set_output_file(fname, "sat-inf.csv") : NULL;
	brdc_t brdc;
	double ep[6] = { (double)year, (double)mon, (double)day, 0, 0, 0 };
	rtcm->time_s = rtcm->time = epoch2time(ep);
	while (fBRDC && !feof(fBRDC) && (data = fgetc(fBRDC)) != EOF)
	{
		int ret = input_rtcm3(rtcm, data);
		if (ret==2) brdc.add_nav_eph(rtcm);
	}
	while (fLOG && !feof(fLOG) && (data = fgetc(fLOG)) != EOF)
	{
		int ret1 = input_rtcm3(rtcm, data);
		if (ret1 == 2) brdc.add_nav_eph(rtcm);
		if (rtcm->nbyte == 0 && rtcm->len > 0)
		{
			if (rtk_crc24q(rtcm->buff, rtcm->len) != getbitu(rtcm->buff, rtcm->len * 8, 24))
			{
				++numofcrc;
			}
			else
			{
				++numofmsg;
			}
			rtcm->len = 0;
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
		/* store the data for further process */
		update_epoch_obs(vEpoch, rtcm);
		if (ret1 == 5) update_epoch_pos(vEpoch, rtcm);
		/* done */
	}

	double rate[4] = { 0 };

	proc_epoch_data(vEpoch, brdc, &rtcm->nav, fSAT, fCSV, rate, flag & (1 << 1));

	output_status(vDt, mGAP, numofmsg, numofcrc, numofepoch, stime, etime, v1, v2, v3, v4, fname, flag & (1 << 5), rate, rtcm->sta.recver, rtcm->sta.rectype);

	if (fLOG) fclose(fLOG);
	if (fRTCM) fclose(fRTCM);
	if (fBRDC) fclose(fBRDC);
	if (fSAT) fclose(fSAT);
	if (fCSV) fclose(fCSV);

	free_rtcm(rtcm);
	delete rtcm;

	return ret;
}

int main(int argc, char* argv[])
{
	if (argc < 2)
	{
		printf("log2rtcm file [rtcm=1] [date=yyyy,mm,dd] [brdc=file] [outsatpos=1] [outsatinf=1]");
	}
	else
	{
		int flag = 0;
		const char* temp = nullptr;
		std::string brdcfname;
		int year = 0;
		int mon = 0;
		int day = 0;
		time_t now = time(0);
		tm* gmtm = gmtime(&now);
		year = gmtm->tm_year + 1900;
		mon = gmtm->tm_mon + 1;
		day = gmtm->tm_mday;
		
		flag |= 1 << 0;	/* set output flag as default */
		flag |= 1 << 1;	/* set Health flag as default */
		flag |= 1 << 5;	/* set time flag as default */
		flag |= 1 << 6;	/* set geod flag as default */

		for (int i = 2; i < argc; ++i)
		{
			if (strstr(argv[i], "outp") && (temp = strrchr(argv[i], '=')))
			{
				if (atoi(temp + 1))
				{
					flag |= 1 << 0;		/* set output flag */
				}
				else
				{
					flag &= ~(1 << 0);	/* clear output flag */
				}
			}
			if (strstr(argv[i], "health") && (temp = strrchr(argv[i], '=')))
			{
				if (atoi(temp + 1))
				{
					flag |= 1 << 1;		/* set Health flag */
				}
				else
				{
					flag &= ~(1 << 1);	/* clear Health flag */
				}
			}
			if (strstr(argv[i], "outsatpos") && (temp = strrchr(argv[i], '=')))
			{
				if (atoi(temp + 1))
				{
					flag |= 1 << 2;		/* set output satellite pos flag */
				}
				else
				{
					flag &= ~(1 << 2);	/* clear output satellite pos flag */
				}
			}
			if (strstr(argv[i], "outsatinf") && (temp = strrchr(argv[i], '=')))
			{
				if (atoi(temp + 1))
				{
					flag |= 1 << 3;		/* set output satellite info flag */
				}
				else
				{
					flag &= ~(1 << 3);	/* clear output satellite info flag */
				}
			}	
			if (strstr(argv[i], "rtcm") && (temp = strrchr(argv[i], '=')))
			{
				if (atoi(temp + 1))
				{
					flag |= 1 << 4;		/* set rtcm format flag */
				}
				else
				{
					flag &= ~(1 << 4);	/* clear rtcm format flag */
				}
			}
			if (strstr(argv[i], "time") && (temp = strrchr(argv[i], '=')))
			{
				if (atoi(temp + 1))
				{
					flag |= 1 << 5;		/* set time flag */
				}
				else
				{
					flag &= ~(1 << 5);	/* clear time flag */
				}
			}
			if (strstr(argv[i], "geod") && (temp = strrchr(argv[i], '=')))
			{
				if (atoi(temp + 1))
				{
					flag |= 1 << 6;		/* set geod flag */
				}
				else
				{
					flag &= ~(1 << 6);	/* clear geod flag */
				}
			}
			if (strstr(argv[i], "brdc") && (temp = strrchr(argv[i], '=')))
				brdcfname = std::string(temp + 1);
			if (strstr(argv[i], "date") && (temp = strrchr(argv[i], '=')))
			{
				char* val[100];
				int num = parse_fields((char*)(temp + 1), val, ',', 100);
				if (num > 2)
				{
					year = atoi(val[0]);
					mon = atoi(val[1]);
					day = atoi(val[2]);
				}
			}
		}
		if (flag & (1<<4))
			procrtcm(argv[1], flag, brdcfname, year, mon, day);
		else
			log2rtcm(argv[1], flag, brdcfname);
	}
	return 0;
}
