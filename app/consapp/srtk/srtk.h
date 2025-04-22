#pragma once

#include "rtklib.h"
#include <mutex>

struct artk_t
{
	rtcm_t* rtcm_obs;	/* rtcm decoder to base stream */
	rtcm_t* rtcm_nav;	/* rtcm decoder to brdc stream */
	rtk_t* rtk;			/* rtk */
	obsd_t* obs;		/* for rover and base */
	prcopt_t* popt;		/* options */
	std::mutex status;
	artk_t()
	{
		rtcm_obs = new rtcm_t;
		rtcm_nav = new rtcm_t;
		rtk = new rtk_t;
		obs = new obsd_t[MAXOBS + MAXOBS];
		popt = new prcopt_t;
		memset(rtcm_obs, 0, sizeof(rtcm_t));
		memset(rtcm_nav, 0, sizeof(rtcm_t));
		init_rtcm(rtcm_obs);
		init_rtcm(rtcm_nav);
		*popt = prcopt_default;
		popt->mode = PMODE_KINEMA;
		popt->navsys = SYS_GPS | SYS_GLO | SYS_GAL | SYS_CMP;
		popt->elmin = 10.0 * D2R;
		popt->refpos = POSOPT_RTCM;
		popt->glomodear = 0;
		popt->bdsmodear = 0;
		popt->dynamics = 2;
		popt->modear = 0;
		popt->nf = 2;/* default triple band L1+L2+L5 */
		rtkinit(rtk, popt);
	}
	~artk_t()
	{
		free_rtcm(rtcm_obs);
		free_rtcm(rtcm_nav);
		delete rtcm_obs;
		delete rtcm_nav;
		delete rtk;
		delete obs;
		delete popt;
	}
	int add_rove_obs(obsd_t* obsd, int n, double *pos)
	{
		int ret = 0;
		int i = 0;
		if (status.try_lock())
		{
			if (n > 0)
			{
				memset(obs, 0, sizeof(obsd_t) * MAXOBS);
				for (i = 0; i < n; ++i)
				{
					obs[i] = obsd[i];
					obs[i].rcv = 1;
				}
				rtk->sol.rr[0] = pos[0];
				rtk->sol.rr[1] = pos[1];
				rtk->sol.rr[2] = pos[2];
				rtk->sol.time = obs[0].time;
			}
			status.unlock();
			ret = 1;
		}
		return ret;
	}
	int add_base_obs(obsd_t* obsd, int n, double *pos)
	{
		int ret = 0;
		int i = 0;
		if (status.try_lock())
		{
			memset(obs + MAXOBS, 0, sizeof(obsd_t) * MAXOBS);
			for (i = 0; i < n; ++i)
			{
				obs[MAXOBS + i] = obsd[i];
				obs[MAXOBS + i].rcv = 2;
			}
			rtk->rb[0] = pos[0];
			rtk->rb[1] = pos[1];
			rtk->rb[2] = pos[2];
			status.unlock();
			ret = 1;
		}
		return ret;
	}
	int add_base_buf(char* buff, int nlen)
	{
		int ret = 0;
		int i = 0;
		int j = 0;
		int sat = 0;
		int sys = 0;
		int prn = 0;
		if (status.try_lock())
		{
			for (i = 0; i < nlen; ++i)
			{
				ret = input_rtcm3(rtcm_obs, (uint8_t)buff[i]);
				if (ret==1)
				{
					for (j = 0; j < rtcm_obs->obs.n; ++j)
					{
						obs[MAXOBS + j] = rtcm_obs->obs.data[j];
						obs[MAXOBS + j].rcv = 2;
					}
				}
				else if (ret == 5)
				{
					rtk->rb[0] = rtcm_obs->sta.pos[0];
					rtk->rb[1] = rtcm_obs->sta.pos[1];
					rtk->rb[2] = rtcm_obs->sta.pos[2];
				}
				else if (ret == 2 && (sat = rtcm_nav->ephsat) > 0)
				{
					sys = satsys(sat, &prn);
					if (sys == SYS_GLO)
					{
						int loc = prn - 1;
						rtcm_nav->nav.geph[loc] = rtcm_obs->nav.geph[loc];
					}
					else if (sys == SYS_GPS || sys == SYS_GAL || sys == SYS_CMP || sys == SYS_QZS || sys == SYS_IRN)
					{
						int loc = sat + MAXSAT * rtcm_nav->ephset - 1;
						rtcm_nav->nav.eph[loc] = rtcm_obs->nav.eph[loc];
					}
				}
			}
			status.unlock();
			ret = 1;
		}
		return ret;
	}
	int add_brdc(nav_t* nav)
	{
		int i = 0;
		int ret = 0;

		if (status.try_lock())
		{
			for (i = 0; i < MAXSAT+MAXSAT; ++i)
			{
				if (nav->eph[i].sat > 0)
				{
					rtcm_nav->nav.eph[i] = nav->eph[i];
				}
			}
			for (i = 0; i < MAXPRNGLO; ++i)
			{
				if (nav->geph[i].sat > 0)
				{
					rtcm_nav->nav.geph[i] = nav->geph[i];
				}
			}
			status.unlock();
			ret = 1;
		}
		return ret;
	}
	int add_brdc(char* buff, int nlen) 
	{
		int i = 0;
		int ret = 0;

		if (status.try_lock())
		{
			for (i = 0; i < nlen; ++i)
			{
				ret = input_rtcm3(rtcm_nav, (uint8_t)buff[i]);
			}
			status.unlock();
			ret = 1;
		}
		return ret;
	}
	int proc()
	{
		int ret = 0;
		int i = 0;
		if (status.try_lock())
		{
			obsd_t* cur_obs = new obsd_t[MAXOBS + MAXOBS];
			memset(cur_obs, 0, sizeof(obsd_t) * (MAXOBS + MAXOBS));
			int nobs = 0;
			for (i = 0; i < MAXOBS; ++i)
			{
				if (obs[i].rcv == 1) cur_obs[nobs++] = obs[i];
			}
			for (i = MAXOBS; i < MAXOBS + MAXOBS; ++i)
			{
				if (obs[i].rcv == 2) cur_obs[nobs++] = obs[i];
			}
			if (fabs(rtk->sol.rr[0]) < 0.001 || fabs(rtk->sol.rr[1]) < 0.001 || fabs(rtk->sol.rr[2]) < 0.001)
			{
				rtk->sol.rr[0] = rtk->rb[0];
				rtk->sol.rr[1] = rtk->rb[1];
				rtk->sol.rr[2] = rtk->rb[2];
			}
			rtk->opt.outsingle = 1;
			rtkpos(rtk, cur_obs, nobs, &rtcm_nav->nav);
			double diff[3] = { rtk->sol.rr[0] - rtk->rb[0], rtk->sol.rr[1] - rtk->rb[1], rtk->sol.rr[2] - rtk->rb[2] };
			double dist = sqrt(diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2]);

			printf("%s,%10.4f,%14.4f,%14.4f,%14.4f,%10.4f,%10.4f,%10.4f,%10.6f,%10.6f,%10.6f,%3i,%10.3f,%10.3f\n", time_str(rtk->sol.time, 3), dist/1000.0, rtk->sol.rr[0], rtk->sol.rr[1], rtk->sol.rr[2], rtk->sol.rr[3], rtk->sol.rr[4], rtk->sol.rr[5], sqrt(rtk->sol.qr[0]), sqrt(rtk->sol.qr[1]), sqrt(rtk->sol.qr[2]), rtk->sol.ns, rtk->sol.age, rtk->sol.ratio);

			delete[] cur_obs;
			status.unlock();
			ret = 1;
		}
		return ret;
	}
};
