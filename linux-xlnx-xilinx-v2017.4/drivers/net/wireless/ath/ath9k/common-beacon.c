/*
 * Copyright (c) 2008-2011 Atheros Communications Inc.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include "common.h"

#define FUDGE 2

static u32 ath9k_get_next_tbtt(struct ath_hw *ah, u64 tsf,
			       unsigned int interval)
{
	unsigned int offset, divisor;

	tsf += TU_TO_USEC(FUDGE + ah->config.sw_beacon_response_time);
	divisor = TU_TO_USEC(interval);
	div_u64_rem(tsf, divisor, &offset);

	return (u32) tsf + divisor - offset;
}

/*
 * This sets up the beacon timers according to the timestamp of the last
 * received beacon and the current TSF, configures PCF and DTIM
 * handling, programs the sleep registers so the hardware will wakeup in
 * time to receive beacons, and configures the beacon miss handling so
 * we'll receive a BMISS interrupt when we stop seeing beacons from the AP
 * we've associated with.
 这根据最后收到的信标的时间戳和当前的 TSF 设置信标定时器，
 配置 PCF 和 DTIM 处理，对睡眠寄存器进行编程，
 以便硬件及时唤醒以接收信标，并配置信标未命中处理，
 因此我们 当我们停止看到来自我们关联的 AP 的信标时，将收到 BMISS 中断。
 */
int ath9k_cmn_beacon_config_sta(struct ath_hw *ah,
				 struct ath_beacon_config *conf,
				 struct ath9k_beacon_state *bs)
{
	struct ath_common *common = ath9k_hw_common(ah);
	//int dtim_intval;  //rt---sub
	u64 tsf;

//rt---add
	int dtimperiod, dtimcount, sleepduration;
	int cfpperiod, cfpcount, bmiss_timeout;
	u32 tsftu;
	int num_beacons, offset, dtim_dec_count, cfp_dec_count;
//end
	

	/* No need to configure beacon if we are not associated
		如果我们没有关联，则无需配置信标
	*/
	if (!test_bit(ATH_OP_PRIM_STA_VIF, &common->op_flags)) {
		ath_dbg(common, BEACON,
			"STA is not yet associated..skipping beacon config\n");
		return -EPERM;
	}

	memset(bs, 0, sizeof(*bs));
	conf->intval = conf->beacon_interval;
	bmiss_timeout = (ATH_DEFAULT_BMISS_LIMIT * conf->beacon_interval);//rt---add
	/*
	 * Setup dtim parameters according to
	 * last beacon we received (which may be none).
	 */
//	dtim_intval = conf->intval * conf->dtim_period;	//rt---sub

//rt---add
	dtimperiod = conf->dtim_period;
	if (dtimperiod <= 0)		/* NB: 0 if not known */
		dtimperiod = 1;
	dtimcount = 1;
	if (dtimcount >= dtimperiod)	/* NB: sanity check */
		dtimcount = 0;
	cfpperiod = 1;			/* NB: no PCF support yet */
	cfpcount = 0;

	sleepduration = conf->intval;
	if (sleepduration <= 0)
		sleepduration = conf->intval;

//end


	/*
	 * Pull nexttbtt forward to reflect the current
	 * TSF and calculate dtim state for the result.
	 向前拉 nexttbtt 以反映当前的 TSF 并计算结果的 dtim+cfp 状态。
	 */
/* //rt---modify

	tsf = ath9k_hw_gettsf64(ah);
	conf->nexttbtt = ath9k_get_next_tbtt(ah, tsf, conf->intval);

	bs->bs_intval = TU_TO_USEC(conf->intval);
	bs->bs_dtimperiod = conf->dtim_period * bs->bs_intval;
	bs->bs_nexttbtt = conf->nexttbtt;
	bs->bs_nextdtim = conf->nexttbtt;
	if (conf->dtim_period > 1)
		bs->bs_nextdtim = ath9k_get_next_tbtt(ah, tsf, dtim_intval);

	/*
	 * Calculate the number of consecutive beacons to miss* before taking
	 * a BMISS interrupt. The configuration is specified in TU so we only
	 * need calculate based	on the beacon interval.  Note that we clamp the
	 * result to at most 15 beacons.
	 *
	bs->bs_bmissthreshold = DIV_ROUND_UP(conf->bmiss_timeout, conf->intval);
	if (bs->bs_bmissthreshold > 15)
		bs->bs_bmissthreshold = 15;
	else if (bs->bs_bmissthreshold <= 0)
		bs->bs_bmissthreshold = 1;
*/

//rt---modify
	tsf = ath9k_hw_gettsf64(ah);
	tsftu = TSF_TO_TU(tsf>>32, tsf) + FUDGE;

	num_beacons = tsftu / conf->intval + 1;
	offset = tsftu % conf->intval;
	conf->nexttbtt = tsftu - offset;
	if (offset)
		conf->nexttbtt += conf->intval;

	/* DTIM Beacon every dtimperiod Beacon */
	dtim_dec_count = num_beacons % dtimperiod;
	/* CFP every cfpperiod DTIM Beacon */
	cfp_dec_count = (num_beacons / dtimperiod) % cfpperiod;
	if (dtim_dec_count)
		cfp_dec_count++;

	dtimcount -= dtim_dec_count;
	if (dtimcount < 0)
		dtimcount += dtimperiod;

	cfpcount -= cfp_dec_count;
	if (cfpcount < 0)
		cfpcount += cfpperiod;

	bs.bs_intval = conf->intval;
	bs.bs_nexttbtt = conf->nexttbtt;
	bs.bs_dtimperiod = dtimperiod*conf->intval;
	bs.bs_nextdtim = bs.bs_nexttbtt + dtimcount*conf->intval;
	bs.bs_cfpperiod = cfpperiod*bs.bs_dtimperiod;
	bs.bs_cfpnext = bs.bs_nextdtim + cfpcount*bs.bs_dtimperiod;
	bs.bs_cfpmaxduration = 0;

	/*
	 * Calculate the number of consecutive beacons to miss* before taking
	 * a BMISS interrupt. The configuration is specified in TU so we only
	 * need calculate based on the beacon interval.  Note that we clamp the
	 * result to at most 15 beacons.
	 */
	if (sleepduration > conf->intval) {
		bs.bs_bmissthreshold = ATH_DEFAULT_BMISS_LIMIT / 2;
	} else {
		bs.bs_bmissthreshold = DIV_ROUND_UP(bmiss_timeout, conf->intval);
		if (bs.bs_bmissthreshold > 15)
			bs.bs_bmissthreshold = 15;
		else if (bs.bs_bmissthreshold <= 0)
			bs.bs_bmissthreshold = 1;
	}
//end
	/*
	 * Calculate sleep duration. The configuration is given in ms.
	 * We ensure a multiple of the beacon period is used. Also, if the sleep
	 * duration is greater than the DTIM period then it makes senses
	 * to make it a multiple of that.
	 *
	 * XXX fixed at 100ms
	 */

//	bs->bs_sleepduration = TU_TO_USEC(roundup(IEEE80211_MS_TO_TU(100), conf->intval));//rt--modify

	bs.bs_sleepduration = roundup(IEEE80211_MS_TO_TU(100), sleepduration);//rt--modify

	if (bs->bs_sleepduration > bs->bs_dtimperiod)
		bs->bs_sleepduration = bs->bs_dtimperiod;

	/* TSF out of range threshold fixed at 1 second */
	bs->bs_tsfoor_threshold = ATH9K_TSFOOR_THRESHOLD;

//	ath_dbg(common, BEACON, "bmiss: %u sleep: %u\n",
//		bs->bs_bmissthreshold, bs->bs_sleepduration);  //rt---modify

//rt---add
	ath_dbg(common, CONFIG, "intval: %u tsf: %llu tsftu: %u\n",
		conf->intval, tsf, tsftu);
	ath_dbg(common, CONFIG,
		"bmiss: %u sleep: %u cfp-period: %u maxdur: %u next: %u\n",
		bs.bs_bmissthreshold, bs.bs_sleepduration,
		bs.bs_cfpperiod, bs.bs_cfpmaxduration, bs.bs_cfpnext);
//end

	return 0;
}
EXPORT_SYMBOL(ath9k_cmn_beacon_config_sta);

void ath9k_cmn_beacon_config_adhoc(struct ath_hw *ah,
				   struct ath_beacon_config *conf)
{
	struct ath_common *common = ath9k_hw_common(ah);

	conf->intval = TU_TO_USEC(conf->beacon_interval);

	if (conf->ibss_creator)
		conf->nexttbtt = conf->intval;
	else
		conf->nexttbtt = ath9k_get_next_tbtt(ah, ath9k_hw_gettsf64(ah),
					       conf->beacon_interval);

	if (conf->enable_beacon)
		ah->imask |= ATH9K_INT_SWBA;
	else
		ah->imask &= ~ATH9K_INT_SWBA;

	ath_dbg(common, BEACON,
		"IBSS (%s) nexttbtt: %u intval: %u conf_intval: %u\n",
		(conf->enable_beacon) ? "Enable" : "Disable",
		conf->nexttbtt, conf->intval, conf->beacon_interval);
}
EXPORT_SYMBOL(ath9k_cmn_beacon_config_adhoc);

/*
 * For multi-bss ap support beacons are either staggered evenly over N slots or
 * burst together.  For the former arrange for the SWBA to be delivered for each
 * slot. Slots that are not occupied will generate nothing.
 对于多 bss ap 支持，信标要么在 N 个时隙上均匀交错，要么一起突发。 
 对于前者，请安排为每个插槽交付 SWBA。 未被占用的插槽不会产生任何东西。
 */
void ath9k_cmn_beacon_config_ap(struct ath_hw *ah,
				struct ath_beacon_config *conf,
				unsigned int bc_buf)
{
	struct ath_common *common = ath9k_hw_common(ah);

	/* NB: the beacon interval is kept internally in TU's */
	conf->intval = TU_TO_USEC(conf->beacon_interval);
	conf->intval /= bc_buf;
	conf->nexttbtt = ath9k_get_next_tbtt(ah, ath9k_hw_gettsf64(ah),
				       conf->beacon_interval);

	if (conf->enable_beacon)
		ah->imask |= ATH9K_INT_SWBA;
	else
		ah->imask &= ~ATH9K_INT_SWBA;

	ath_dbg(common, BEACON,
		"AP (%s) nexttbtt: %u intval: %u conf_intval: %u\n",
		(conf->enable_beacon) ? "Enable" : "Disable",
		conf->nexttbtt, conf->intval, conf->beacon_interval);
	


}
EXPORT_SYMBOL(ath9k_cmn_beacon_config_ap);
