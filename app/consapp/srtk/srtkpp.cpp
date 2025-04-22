#include "srtk.h"
#include <map>
#include <vector>
#include <algorithm>

/* help text -----------------------------------------------------------------*/
static const char* help[] = {
"",
" usage: srtkpp rove=rovefilename base=basefilename brdc=brdcfname date=yyyy/mm/dd",
"",
};

/* show message --------------------------------------------------------------*/
extern int showmsg(const char* format, ...)
{
    va_list arg;
    va_start(arg, format); vfprintf(stderr, format, arg); va_end(arg);
    fprintf(stderr, "\r");
    return 0;
}
extern void settspan(gtime_t ts, gtime_t te) {}
extern void settime(gtime_t time) {}

/* print help ----------------------------------------------------------------*/
static void printhelp(void)
{
    int i;
    for (i = 0; i < (int)(sizeof(help) / sizeof(*help)); i++) fprintf(stderr, "%s\n", help[i]);
    exit(0);
}

static void set_output_file_name(const char* fname, const char* key, char* outfname)
{
    char filename[255] = { 0 }, outfilename[255] = { 0 };
    strcpy(filename, fname);
    char* temp = strrchr(filename, '.');
    if (temp) temp[0] = '\0';
    sprintf(outfname, "%s%s", filename, key);
}

static FILE* set_output_file(const char* fname, const char* key)
{
    char filename[255] = { 0 };
    set_output_file_name(fname, key, filename);
    return fopen(filename, "w");
}

inline bool operator==(const gtime_t& l, const gtime_t& r) {
    return fabs(timediff(l, r)) <= DTTOL;
}
inline bool operator<(const gtime_t& l, const gtime_t& r) {
    return timediff(l, r) < -DTTOL;
}
inline bool operator<=(const gtime_t& l, const gtime_t& r) {
    return l < r || l == r;
}
inline bool operator==(const eph_t& l, const eph_t& r) {
    return l.sat == r.sat && l.toe == r.toe;
}
inline bool operator==(const geph_t& l, const geph_t& r) {
    return l.sat == r.sat && l.toe == r.toe;
}
inline bool operator<(const eph_t& l, const eph_t& r) {
    return l.sat == r.sat ? l.toe < r.toe : l.sat < r.sat;
}
inline bool operator<(const eph_t& l, const gtime_t& r) {
    return l.toe < r;
}
inline bool operator<(const geph_t& l, const geph_t& r) {
    return l.sat == r.sat ? l.toe < r.toe : l.sat < r.sat;
}
inline bool operator<(const geph_t& l, const gtime_t& r) {
    return l.toe < r;
}

static int get_current_date(int& year, int& mon)
{
    time_t now = time(0);
    tm* gmtm = gmtime(&now);
    year = gmtm->tm_year + 1900;
    mon = gmtm->tm_mon + 1;
    return gmtm->tm_mday;
}

struct brdc_t
{
    std::map<int, std::vector< eph_t>> mSatEph;
    std::map<int, std::vector<geph_t>> mGloEph;
    brdc_t()
    {

    }
    int add_sat_eph(int key, const eph_t& eph)
    {
        int ret = 0;
        std::vector<eph_t>::iterator pEph = std::find(mSatEph[key].begin(), mSatEph[key].end(), eph);
        if (pEph == mSatEph[key].end())
        {
            mSatEph[key].push_back(eph);
            std::sort(mSatEph[key].begin(), mSatEph[key].end());
            ret = 1;
        }
        return ret;
    }
    int add_glo_eph(int key, const geph_t& eph)
    {
        int ret = 0;
        std::vector<geph_t>::iterator pEph = std::find(mGloEph[key].begin(), mGloEph[key].end(), eph);
        if (pEph == mGloEph[key].end())
        {
            mGloEph[key].push_back(eph);
            std::sort(mGloEph[key].begin(), mGloEph[key].end());
            ret = 1;
        }
        return ret;
    }
    int read_rtcm_data(const char* fname, int year, int mon, int day)
    {
        int ret = 0;
        int data = 0;
        int sat = 0;
        int sys = 0;
        int prn = 0;
        int sel = 0;
        int count = 0;
        double ep[6] = { 0 };
        if (year <= 0 || mon <= 0 || day <= 0) day = get_current_date(year, mon);
        ep[0] = year;
        ep[1] = mon;
        ep[2] = day;
        FILE* fRTCM = fname ? fopen(fname, "rb") : NULL;
        rtcm_t* rtcm = new rtcm_t;
        init_rtcm(rtcm);
        rtcm->time = rtcm->time_s = epoch2time(ep);
        while (fRTCM && !feof(fRTCM) && (data = fgetc(fRTCM)) != EOF)
        {
            ret = input_rtcm3(rtcm, data);
            if (ret == 2 && (sat = rtcm->ephsat) > 0)
            {
                sys = satsys(sat, &prn);
                if (sys == SYS_GLO)
                {
                    int loc = prn - 1;
                    if (add_glo_eph(loc, rtcm->nav.geph[loc]))
                        ++count;
                }
                else if (sys == SYS_GPS || sys == SYS_GAL || sys == SYS_CMP || sys == SYS_QZS || sys == SYS_IRN)
                {
                    int loc = sat + MAXSAT * rtcm->ephset - 1;
                    if (add_sat_eph(loc, rtcm->nav.eph[loc]))
                        ++count;
                }
            }
        }
        if (fRTCM) fclose(fRTCM);
        free_rtcm(rtcm);
        delete rtcm;
        return ret;
    }
    int get_brdc_data(gtime_t time, nav_t* nav)
    {
        int ret = 1;
        double dt0 = 0;
        double dt1 = 0;
        int is_new = 0;
        /* */
        for (std::map<int, std::vector< eph_t>>::iterator pSatEph = mSatEph.begin(); pSatEph != mSatEph.end(); ++pSatEph)
        {
            if (nav->eph[pSatEph->first].sat > 0)
            {
                dt0 = fabs(timediff(nav->eph[pSatEph->first].toe, time));
            }
            else
            {
                dt0 = 24 * 3600.0;
            }
            std::vector<eph_t>::iterator pEph = std::lower_bound(pSatEph->second.begin(), pSatEph->second.end(), time);
            if (pEph != pSatEph->second.end())
            {
                is_new = 0;
                if ((dt1 = fabs(timediff(pEph->toe, time))) < dt0)
                {
                    dt0 = dt1;
                    nav->eph[pSatEph->first] = *pEph;
                    is_new++;
                }
                if (pEph != pSatEph->second.begin())
                {
                    dt1 = fabs(timediff((pEph - 1)->toe, time));
                    if (dt1 < dt0)
                    {
                        dt0 = dt1;
                        nav->eph[pSatEph->first] = *(pEph - 1);
                        is_new++;
                    }
                }
                if ((pEph + 1) != pSatEph->second.end())
                {
                    dt1 = fabs(timediff((pEph + 1)->toe, time));
                    if (dt1 < dt0)
                    {
                        dt0 = dt1;
                        nav->eph[pSatEph->first] = *(pEph - 1);
                        is_new++;
                    }
                }
                if (is_new) ++ret;
            }
            else if (pSatEph->second.size() > 0)
            {
                --pEph;
                if (pEph != pSatEph->second.end())
                {
                    is_new = 0;
                    if ((dt1 = fabs(timediff(pEph->toe, time))) < dt0)
                    {
                        dt0 = dt1;
                        nav->eph[pSatEph->first] = *pEph;
                        is_new++;
                    }
                    if (is_new) ++ret;
                }
            }
        }
        for (std::map<int, std::vector<geph_t>>::iterator pSatEph = mGloEph.begin(); pSatEph != mGloEph.end(); ++pSatEph)
        {
            if (nav->geph[pSatEph->first].sat > 0)
            {
                dt0 = fabs(timediff(nav->geph[pSatEph->first].toe, time));
            }
            else
            {
                dt0 = 24 * 3600.0;
            }
            std::vector<geph_t>::iterator pEph = std::lower_bound(pSatEph->second.begin(), pSatEph->second.end(), time);
            if (pEph != pSatEph->second.end())
            {
                is_new = 0;
                if ((dt1 = fabs(timediff(pEph->toe, time))) < dt0)
                {
                    dt0 = dt1;
                    nav->geph[pSatEph->first] = *pEph;
                    is_new++;
                }
                if (pEph != pSatEph->second.begin())
                {
                    dt1 = fabs(timediff((pEph - 1)->toe, time));
                    if (dt1 < dt0)
                    {
                        dt0 = dt1;
                        nav->geph[pSatEph->first] = *(pEph - 1);
                        is_new++;
                    }
                }
                if ((pEph + 1) != pSatEph->second.end())
                {
                    dt1 = fabs(timediff((pEph + 1)->toe, time));
                    if (dt1 < dt0)
                    {
                        dt0 = dt1;
                        nav->geph[pSatEph->first] = *(pEph - 1);
                        is_new++;
                    }
                }
                if (is_new) ++ret;
            }
            else if (pSatEph->second.size() > 0)
            {
                --pEph;
                if (pEph != pSatEph->second.end())
                {
                    is_new = 0;
                    if ((dt1 = fabs(timediff(pEph->toe, time))) < dt0)
                    {
                        dt0 = dt1;
                        nav->geph[pSatEph->first] = *pEph;
                        is_new++;
                    }
                    if (is_new) ++ret;
                }
            }
        }
        return 1;
    }
};

struct station_t
{
    FILE* fRTCM;
    rtcm_t* rtcm;
    station_t()
    {
        fRTCM = NULL;
        rtcm = new rtcm_t;
        init_rtcm(rtcm);
    }
    int open(const char* fname, int year, int mon, int day)
    {
        double ep[6] = { 0 };
        if (year <= 0 || mon <= 0 || day <= 0) day = get_current_date(year, mon);
        ep[0] = year;
        ep[1] = mon;
        ep[2] = day;
        rtcm->time = rtcm->time_s = epoch2time(ep);
        if (fRTCM) fclose(fRTCM); fRTCM = NULL;
        fRTCM = fname ? fopen(fname, "rb") : NULL;
        return fRTCM != NULL;
    }
    ~station_t()
    {
        if (fRTCM) fclose(fRTCM);
        free_rtcm(rtcm);
        delete rtcm;
    }
    int get_obs(obsd_t* obs, double* pos)
    {
        int ret = 0;
        int data = 0;
        int nsat = 0;
        while (fRTCM && !feof(fRTCM) && (data = fgetc(fRTCM)) != EOF)
        {
            ret = input_rtcm3(rtcm, data);
            if (ret == 1)
            {
                for (int i = 0; i < rtcm->obs.n; ++i)
                {
                    obs[nsat] = rtcm->obs.data[i];
                    ++nsat;
                }
                pos[0] = rtcm->sta.pos[0];
                pos[1] = rtcm->sta.pos[1];
                pos[2] = rtcm->sta.pos[2];
                break;
            }
        }
        return nsat;
    }
};


static int process_log(const char* rovefname, const char* basefname, const char* brdcfname, int year, int mm, int dd)
{
    int ret = 0;
    artk_t* artk = new artk_t;

    brdc_t* brdc = new brdc_t;
    brdc->read_rtcm_data(rovefname, year, mm, dd);
    brdc->read_rtcm_data(basefname, year, mm, dd);
    brdc->read_rtcm_data(brdcfname, year, mm, dd);

    station_t* base = new station_t;
    station_t* rove = new station_t;

    rove->open(rovefname, year, mm, dd);
    base->open(basefname, year, mm, dd);

    obsd_t* obsd = new obsd_t[MAXOBS + MAXOBS];
    memset(obsd, 0, sizeof(obsd_t) * (MAXOBS + MAXOBS));
    int nsat[2] = { 0 };
    double pos[6] = { 0 };
    obsd_t* obs_rove = obsd;
    obsd_t* obs_base = obsd + MAXOBS;
    double* pos_rove = pos;
    double* pos_base = pos + 3;


    double dt = 0;

    int count = 0;
    int count_spp = 0;

    FILE* fSOL = set_output_file(rovefname, ".csv");
    if (fSOL)
    {
        fprintf(fSOL, "wk,ws,lat,lon,ht,clk,res,lat1,lon1,ht1,clk1,res1\n");
    }

    while (true)
    {
        nsat[0] = rove->get_obs(obs_rove, pos_rove);
        if (!nsat[0]) break;
        while (true)
        {
            if (nsat[1] > 0)
            {
                if (obs_rove->time == obs_base->time || obs_rove->time < obs_base->time) break;
                if ((dt = timediff(obs_rove->time, obs_base->time)) > 0.5)
                {
                    nsat[1] = base->get_obs(obs_base, pos_base);
                }
            }
            else
            {
                nsat[1] = base->get_obs(obs_base, pos_base);
            }
            if (!nsat[1]) break;
        }
        /* base and rove data ready */
        if (nsat[0] > 0 && artk->add_rove_obs(obs_rove, nsat[0], pos_rove))
        {
            brdc->get_brdc_data(obs_rove[0].time, &artk->rtcm_nav->nav);
            if (nsat[1] > 0 && artk->add_base_obs(obs_base, nsat[1], pos_base))
            {

            }
            else
            {
            }
            if (artk->proc())
            {

            }
            count++;
        }
    }

    delete brdc;
    delete base;
    delete rove;
    delete[] obsd;

    delete artk;

    if (fSOL) fclose(fSOL);

    return ret;
}

/* srtkpp main -------------------------------------------------------------*/
int main(int argc, char** argv)
{
    int ret = 0;
    if (argc < 2)
    {
        printhelp();
    }
    else
    {

        int yyyy = 0;
        int mm = 0;
        int dd = 0;
        std::string rovefname;
        std::string basefname;
        std::string brdcfname;
        char* temp = nullptr;
        for (int i = 1; i < argc; ++i)
        {
            if (temp = strchr(argv[i], '='))
            {
                temp[0] = '\0';
                if (strstr(argv[i], "rove"))
                {
                    rovefname = std::string(temp + 1);
                }
                else if (strstr(argv[i], "base"))
                {
                    basefname = std::string(temp + 1);
                }
                else if (strstr(argv[i], "brdc"))
                {
                    brdcfname = std::string(temp + 1);
                }
                else if (strstr(argv[i], "date"))
                {
                    char* temp1 = nullptr;
                    while (temp1 = strchr(temp + 1, '/')) temp1[0] = ' ';
                    while (temp1 = strchr(temp + 1, ':')) temp1[0] = ' ';
                    while (temp1 = strchr(temp + 1, '-')) temp1[0] = ' ';
                    int num = sscanf(temp + 1, "%i %i %i", &yyyy, &mm, &dd);
                    if (num < 3)
                    {
                        yyyy = mm = dd = 0;
                    }
                    else
                    {
                    }
                }
            }

        }
        clock_t stime = clock();


        ret = process_log(rovefname.c_str(), basefname.c_str(), brdcfname.c_str(), yyyy, mm, dd);

        clock_t etime = clock();
        double cpu_time_used = ((double)(etime - stime)) / CLOCKS_PER_SEC;
        printf("rove=%s\n", rovefname.c_str());
        printf("base=%s\n", basefname.c_str());
        printf("brdc=%s\n", brdcfname.c_str());
        printf("time=%.3f[s]\n", cpu_time_used);
    }
    return ret;
}
