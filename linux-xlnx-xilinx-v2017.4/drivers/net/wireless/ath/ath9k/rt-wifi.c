/*
 * Copyright (c) 2015, The University of Texas at Austin,
 * Department of Computer Science, Cyberphysical Group
 * http://www.cs.utexas.edu/~cps/ All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the copyright holder nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <linux/slab.h>
#include <linux/log2.h>
#include "ath9k.h"
#include "hw.h"
#include "rt-wifi-sched.h"

static struct ath_buf* ath_rt_wifi_get_buf_ap_tx(struct ath_softc *sc, u8 sta_id);
static struct ath_buf* ath_rt_wifi_get_buf_ap_shared(struct ath_softc *sc);

static u32 rt_wifi_get_slot_len(u8 time_slot)
{
	if (time_slot == RT_WIFI_TIME_SLOT_64TU) {
		return TU_TO_USEC(64);
	} else if (time_slot == RT_WIFI_TIME_SLOT_32TU) {
		return TU_TO_USEC(32);
	} else if (time_slot == RT_WIFI_TIME_SLOT_16TU) {
		return TU_TO_USEC(16);
	} else if (time_slot == RT_WIFI_TIME_SLOT_4TU) {
		return TU_TO_USEC(4);
	} else if (time_slot == RT_WIFI_TIME_SLOT_2TU) {
		return TU_TO_USEC(2);
	} else if (time_slot == RT_WIFI_TIME_SLOT_1TU) {
		return TU_TO_USEC(1);
	} else if (time_slot == RT_WIFI_TIME_SLOT_512us) {
		return 512;
	} else if (time_slot == RT_WIFI_TIME_SLOT_256us) {
		return 256;
	} else {
		RT_WIFI_ALERT("Unknown time slot!!!\n");
		return 512;
	}
}

static void rt_wifi_config_superframe(struct ath_softc *sc)
{
	int idx;
	//sc->rt_wifi_superframe������������֡���ڴ�ռ�//GFP_ATOMIC �����������жϴ���ͽ���������֮������������з����ڴ�. �Ӳ�˯��.//
	sc->rt_wifi_superframe = kmalloc(
		sizeof(struct rt_wifi_sched) * sc->rt_wifi_superframe_size,
		GFP_ATOMIC);

	if (!sc->rt_wifi_superframe) {
		RT_WIFI_ALERT("Cannot get enough memory for superframe!!!\n");
		return;
	}
	//������û�ж�Ϊɶ��Ҫ��������//???????????????//����
	//�Ҳ���ԭ������RT-WIFI����Ū��̬ʱ϶���ֵ��뷨��//
	for (idx = 0; idx < sc->rt_wifi_superframe_size; ++idx) {
		sc->rt_wifi_superframe[idx].type = RT_WIFI_SHARED;
	}

	for (idx = 0; idx < ARRAY_SIZE(RT_WIFI_PRE_CONFIG_SCHED); ++idx) {
		if (RT_WIFI_PRE_CONFIG_SCHED[idx].offset < sc->rt_wifi_superframe_size) {
			sc->rt_wifi_superframe[RT_WIFI_PRE_CONFIG_SCHED[idx].offset].type = RT_WIFI_PRE_CONFIG_SCHED[idx].type;
			sc->rt_wifi_superframe[RT_WIFI_PRE_CONFIG_SCHED[idx].offset].sta_id = RT_WIFI_PRE_CONFIG_SCHED[idx].sta_id;
		} else {
			RT_WIFI_ALERT("Wrong pre-config scheudle index\n");
		}
	}
}


void ath_rt_wifi_ap_start_timer(struct ath9k_htc_priv *priv, u32 bcon_intval, u32 nexttbtt)//rt: ath9k_htc_priv *priv-->ath_softc *sc
{
	struct ath_hw *ah;
	u32 next_timer;

	/* The new design align superframe with the init_beacon(), cur_tsf is 0 */
	next_timer = nexttbtt - RT_WIFI_TIMER_OFFSET;
	RT_WIFI_DEBUG("%s cur_tsf: %u, nexttbtt: %u, next_timer: %u\n", __FUNCTION__, 0, nexttbtt, next_timer);
	
	priv->rt_wifi_slot_len = rt_wifi_get_slot_len(RT_WIFI_TIME_SLOT_LEN);

	
	//#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0])) //��ȡ���������Ԫ�صĸ���
	priv->rt_wifi_superframe_size = ARRAY_SIZE(RT_WIFI_PRE_CONFIG_SCHED);
	RT_WIFI_DEBUG("time slot len: %u, superframe size: %u\n",
		priv->rt_wifi_slot_len, priv->rt_wifi_superframe_size);
	rt_wifi_config_superframe(priv);


	ath9k_hw_gen_timer_start_absolute(priv->ah, priv->rt_wifi_timer, next_timer, priv->rt_wifi_slot_len);

	ah = priv->ah;	//rt: ah-->ac_ah
	if ((ah->imask & ATH9K_INT_GENTIMER) == 0) {
		ath9k_hw_disable_interrupts(ah);
		ah->imask |= ATH9K_INT_GENTIMER;
		ath9k_hw_set_interrupts(ah);
		ath9k_hw_enable_interrupts(ah);
	}

	priv->rt_wifi_virt_start_tsf = 0;
	priv->rt_wifi_asn = (nexttbtt/priv->rt_wifi_slot_len) - 1;
	priv->rt_wifi_bc_tsf = 0;
	priv->rt_wifi_bc_asn = 0;
}

void ath_rt_wifi_sta_start_timer(struct ath_softc *sc)
{
	struct ath_hw *ah;
	u64 next_tsf;
	u32 next_timer;

	/* To compensate tsf drfit because of data rate of beacon frame */
	sc->rt_wifi_virt_start_tsf = RT_WIFI_BEACON_DELAY_OFFSET;//wq:����beacon֡ʱ��Ϊ60us
	/* start TDMA machine in the beginning of the next beacon */
	//rt_wifi_cur_tsf��ֵ�Ǵ�beacon֡���棬��һ�仰˵����Ӧ����beacon.c�����ath9k_beacon_generate�����rt_wifi_bc_tsf = rt_wifi_bc_tsf + RT_WIFI_BEACON_INTVAL
	//2019.3.13���ж�����Ļ��޸�//ע��rt_wifi_bc_tsfֻ��һ��ʼΪ0���������ڵ��ӵ�//
	next_tsf = sc->rt_wifi_cur_tsf + RT_WIFI_BEACON_INTVAL - RT_WIFI_TIMER_OFFSET + RT_WIFI_BEACON_DELAY_OFFSET;
	next_timer = next_tsf & 0xffffffff;
	//�������ath9k�ṩ�Ķ�ʱ������������	ath9k_hw_gen_timer_start_absolute//
	//����û��û�в´�next_time���Ƕ�ʱ���ĳ�ֵ����׼ȷ��˵�Ǵ�����ʱ��������ʱ��//����ath9kû���ֲᣬ����Ҫ�Լ�����//
	ath9k_hw_gen_timer_start_absolute(sc->sc_ah, sc->rt_wifi_timer, next_timer, sc->rt_wifi_slot_len);

	ah = sc->sc_ah;
	//rt-wifi�õ�ath9k��ʱ��,����Ϊ"ATH9K_INT_GENTIMER"//
	if ((ah->imask & ATH9K_INT_GENTIMER) == 0) {
		ath9k_hw_disable_interrupts(ah);
		ah->imask |= ATH9K_INT_GENTIMER;
		//�������������������ж�ʹ��
		ath9k_hw_set_interrupts(ah);
		ath9k_hw_enable_interrupts(ah);
	}
	
	sc->rt_wifi_asn = sc->rt_wifi_asn + (RT_WIFI_BEACON_INTVAL / sc->rt_wifi_slot_len)  - 1;
	RT_WIFI_DEBUG("%s cur_tsf: %llu, next_tsf: %llu, next_timer: %u, next_asn - 1: %d\n", __FUNCTION__, sc->rt_wifi_cur_tsf, next_tsf, next_timer, sc->rt_wifi_asn);
}

static struct ath_buf* ath_rt_wifi_get_buf_from_queue(struct ath_softc *sc, u8 sta_id)
{
	struct ath_buf *bf_itr, *bf_tmp = NULL;
	struct ieee80211_hdr *hdr;

	spin_lock(&sc->rt_wifi_q_lock);
	if (!list_empty(&sc->rt_wifi_q)) {
		list_for_each_entry(bf_itr, &sc->rt_wifi_q, list) {
			//�ۣ�ֱ�Ӵ�buf��sk_buf�����ݲ��ֻ�ȡmacͷ��//
			hdr = (struct ieee80211_hdr*)bf_itr->bf_mpdu->data;
			//������Ƕ�mac��ַ��ɸѡ//
			if (rt_wifi_dst_sta(hdr->addr1, sta_id) == true) {
				bf_tmp = bf_itr;
				break;
			}
		}

		if (bf_tmp != NULL) {
			list_del_init(&bf_tmp->list);
		}
	}
	spin_unlock(&sc->rt_wifi_q_lock);

	return  bf_tmp;
}

struct ath_buf* ath_rt_wifi_get_buf_sta(struct ath_softc *sc)
{
	struct ath_buf *bf_tmp = NULL;
	//��������kfifo_len�ǲ����ô��ˣ���Ϊkfifo_len�ǲ�����������õĸ���//
	//�������Ҳ鵽���kfifo_len�Ƿ��ػ�������ʵ��ʹ�õ��ֽ���//
	if (kfifo_len(&sc->rt_wifi_fifo) >= sizeof(struct ath_buf*)) {
		kfifo_out(&sc->rt_wifi_fifo, &bf_tmp, sizeof(struct ath_buf*));
	}

	return bf_tmp;
}

struct ath_buf* ath_rt_wifi_get_buf_ap_tx(struct ath_softc *sc, u8 sta_id)
{
	struct ath_buf *bf_itr, *bf_tmp = NULL;
	struct ieee80211_hdr *hdr;
	//������������ɶ����Ŷ��//�������
	bf_tmp = ath_rt_wifi_get_buf_from_queue(sc, sta_id);
	if (bf_tmp != NULL)
		return bf_tmp;

	while (kfifo_len(&sc->rt_wifi_fifo) >= sizeof(struct ath_buf*)) {
		kfifo_out(&sc->rt_wifi_fifo, &bf_itr, sizeof(struct ath_buf*));

		hdr = (struct ieee80211_hdr*)bf_itr->bf_mpdu->data;
		//ɸѡsta��ͨ��mac��ַ//
		if (rt_wifi_dst_sta(hdr->addr1, sta_id) == true) {
			bf_tmp = bf_itr;
			break;
		} else {
			spin_lock(&sc->rt_wifi_q_lock);
			list_add_tail(&bf_itr->list, &sc->rt_wifi_q);
			spin_unlock(&sc->rt_wifi_q_lock);
		}
	}

	if (bf_tmp == NULL) {
		bf_tmp = ath_rt_wifi_get_buf_ap_shared(sc);
	}

	return bf_tmp;
}

static struct ath_buf* ath_rt_wifi_get_buf_ap_shared(struct ath_softc *sc)
{
	struct ath_buf *bf_tmp = NULL;

	spin_lock(&sc->rt_wifi_q_lock);
	if (!list_empty(&sc->rt_wifi_q)) {
		bf_tmp = list_first_entry(&sc->rt_wifi_q, struct ath_buf, list);
		list_del_init(&bf_tmp->list);
	} else if (kfifo_len(&sc->rt_wifi_fifo) >= sizeof(struct ath_buf*)) {
		kfifo_out(&sc->rt_wifi_fifo, &bf_tmp, sizeof(struct ath_buf*));
	}
	spin_unlock(&sc->rt_wifi_q_lock);

	return bf_tmp;
}

void ath_rt_wifi_tx(struct ath_softc *sc, struct ath_buf *new_buf)
{
	struct ath_hw *ah = sc->sc_ah;
	struct ath_common *common = ath9k_hw_common(ah);
	struct ath_buf *bf, *bf_last;
	bool puttxbuf = false;
	bool edma;

	/* rt-wifi added */
	bool internal = false;
	struct ath_txq *txq;
	struct list_head head_tmp, *head;

	if(new_buf == NULL)
		return;
	REG_SET_BIT(ah, AR_DIAG_SW, AR_DIAG_RX_DIS | AR_DIAG_RX_ABORT);

	/* mainpulate list to fit original dirver code. 
		�����б����ʺ�ԭʼ����������롣
	*/
	head = &head_tmp;
								//��Ҫ��ʾ�����Ǵ�����һ���µ��б�ṹ����head��Ϊ��ʼָ��
	INIT_LIST_HEAD(head);		//	��ʼ���б�ͷ	
	list_add_tail(&new_buf->list, head);		//list �� new_buf ��ͷ���� ������뵽���ڹ������е� LIST ��ͷ��֮ǰ
	//��������õ���ath_buf�����Լ������qnum//
	txq = &(sc->tx.txq[new_buf->qnum]);

	/* original dirver code */
	edma = !!(ah->caps.hw_caps & ATH9K_HW_CAP_EDMA);
	//2018.7.13:�Ҵ�ӡ�ˣ�����bf �� bf_last ������new_buf
	/*
		bf ��ʾ�������� ������������������ȡָ�����MPDU�Ľṹͷ��ָ��ĵ�ַ
	*/
	bf = list_first_entry(head, struct ath_buf, list);			//bf:buffer ��ȡLIST�ṹ�е�һ��Ԫ�أ�ָ��Ϊ��head��
																//�����Ŀ����MPDU��MPDU��ͷ�����б�
	bf_last = list_entry(head->prev, struct ath_buf, list);		//��ǰһ���ڵ��ȡ����ָ���MPDU
																//bf_last=list_last_entry(head, struct ath_buf, list)

	ath_dbg(common, QUEUE, "qnum: %d, txq depth: %d\n",
			txq->axq_qnum, txq->axq_depth);
	/*
		���²������ڽ�MPDU��txq����������
   		��������� edma ���� fifo Ϊ�գ��� MPDU �� txq_fifo �����
   		��� fifo ��Ϊ��ͬʱ edma δ���ã��� MPDU �����ַ�� txq_axq_link ��������� MPDU ������ axq_q �����
  
   		����������������������κ������puttxbuf=false��������
	*/
	if (edma && list_empty(&txq->txq_fifo[txq->txq_headidx])) {
		list_splice_tail_init(head, &txq->txq_fifo[txq->txq_headidx]); //����list��������headǰ��//���������б����³�ʼ������б�
																/*
																    list_splice_tail_init����head�ṹ�����ݲ���txq_headidx�ṹ��Ȼ�����head�ṹ��
																    ���� txq_headidx Ϊ�գ���������ͷճ���ڿյ� txq_headidx
																*/						
		INCR(txq->txq_headidx, ATH_TXFIFO_DEPTH);
		puttxbuf = true;				//�ж���Ϣ�Ƿ��Ѿ�����buffer
	} 
	else {
		list_splice_tail_init(head, &txq->axq_q);   //axq ��ʾ ath9k Ӳ�����С� ��� edma �� fifo �����ǿյģ��򽫶��нṹ��ͷ���� axq_q �ϲ�

		if (txq->axq_link) {
			ath9k_hw_set_desc_link(ah, txq->axq_link, bf->bf_daddr);
			ath_dbg(common, XMIT, "link[%u] (%p)=%llx (%p)\n",
					txq->axq_qnum, txq->axq_link,
					ito64(bf->bf_daddr), bf->bf_desc);
		} else if (!edma)		//txq->axq_link is false and we do not use edma
			puttxbuf = true;

		txq->axq_link = bf_last->bf_desc;				// bf_desc: ���һ�����ݰ�desc�������ַ
	}

	if (puttxbuf) {
		TX_STAT_INC(txq->axq_qnum, puttxbuf);
		ath9k_hw_puttxbuf(ah, txq->axq_qnum, bf->bf_daddr);//u32 axq_qnum: ath9k Ӳ�����кš����������Ӳ�����кͰ��������������ַ֮�佨����һ������
		ath_dbg(common, XMIT, "TXDP[%u] = %llx (%p)\n",
				txq->axq_qnum, ito64(bf->bf_daddr), bf->bf_desc);
	}

	/* rt-wifi, disable carrier sense random backoff �����ز��������˱��㷨*/
	#if RT_WIFI_ENABLE_COEX == 0
	//AR_DIAG_FORCE_CH_IDLE_HIGH����������ز������Ƿ�������//��Ҳ����˵AR_DIAG_FORCE_RX_CLEAR���ǣ�//wq:AR_DIAG_FORCE_RX_CLEAR�����һ�㼸�ʱ���
	//������AR_DIAG_FORCE_CH_IDLE_HIGHΪ��1ʱ�������ز���������ֵΪ����󣬴Ӷ��ﵽ���������ز�������Ŀ��
	REG_SET_BIT(ah, AR_DIAG_SW, AR_DIAG_FORCE_CH_IDLE_HIGH);
	//AR_D_GBL_IFS_MISC_IGNORE_BACKOFF��ֹ�˱�
	REG_SET_BIT(ah, AR_D_GBL_IFS_MISC, AR_D_GBL_IFS_MISC_IGNORE_BACKOFF);
	#endif
	//AR_DIAG_IGNORE_VIRT_CS���������ز������Ƿ���//wq:�����ز�����
	REG_SET_BIT(ah, AR_DIAG_SW, AR_DIAG_IGNORE_VIRT_CS);

	if (!edma || sc->tx99_state) {
		TX_STAT_INC(txq->axq_qnum, txstart);
		ath9k_hw_txstart(ah, txq->axq_qnum);//wq дһ���Ĵ����Կ�ʼ����Ӳ��//
	}

	if (!internal) {
		while (bf) {
			txq->axq_depth++;
			if (bf_is_ampdu_not_probing(bf))
				txq->axq_ampdu_depth++;

			bf_last = bf->bf_lastbf;
			bf = bf_last->bf_next;
			bf_last->bf_next = NULL;
		}
	}

	REG_CLR_BIT(ah, AR_DIAG_SW, AR_DIAG_RX_DIS | AR_DIAG_RX_ABORT);
}
//2020-8-6
/* 
	unsigned int  cur_len;
 */

void ath_rt_wifi_tasklet(struct ath_softc *sc)
{
	int sched_offset;
	struct ath_buf *new_buf = NULL;
	u64 cur_hw_tsf;

	if (sc->rt_wifi_enable == 0) {
		if (sc->sc_ah->opmode == NL80211_IFTYPE_AP) {
			sc->rt_wifi_enable = 1;
		} else {
			RT_WIFI_DEBUG("RT_WIFI: not enable\n");
			return;
		}
	}
	//wq:2020-7-30
	/* 
	extern u16 my_aid;
	printk(��my aid is %d��,my_aid);
	 */

	/* House keeping */
	sc->rt_wifi_asn++;	
	
	cur_hw_tsf = ath9k_hw_gettsf64(sc->sc_ah);
	//sc->rt_wifi_virt_start_tsf = RT_WIFI_BEACON_DELAY_OFFSET//
	cur_hw_tsf = cur_hw_tsf - sc->rt_wifi_virt_start_tsf + (RT_WIFI_TIMER_OFFSET * 2);
	//�����rt_wifi_asn���Ѿ��ӱ���������Ӳ���ϻ�ȡ//ĳ������������local_tsf���//
	//sc->rt_wifi_asn = cur_hw_tsf >> ilog2((sc->rt_wifi_slot_len));//��λ����������//
	//ilog2��Ϊһ���꣬������������ֵ��2�Ķ���//����˵ilog2(4),�������ص���2//
	//ĳ����������˵���������д����ҷǳ��������ྫ�£���������//��λ���㣬���ų��˸��ţ��ּ���������//������˵���Щ����㷨���㣬���̫����
	//�Բۣ�ԭ����������������ǳ�Ϯԭ��ath9k����beacon֡������������ʱ϶��ע��ath9k_beacon_choose_slot��
	sc->rt_wifi_asn = cur_hw_tsf >> ilog2((sc->rt_wifi_slot_len));//wq:���˿�����cur_hw_tsf /(sc->rt_wifi_slot_len),
	sched_offset = sc->rt_wifi_asn % sc->rt_wifi_superframe_size;

	if (sc->sc_ah->opmode == NL80211_IFTYPE_AP) {
		//wq:2020-8-6 
		/* 
		cur_len=kfifo_len(&sc->rt_wifi_fifo)//�ֽڳ���

		 */
		if (sc->rt_wifi_superframe[sched_offset].type == RT_WIFI_RX) {
			RT_WIFI_DEBUG("RT_WIFI_RX(%d)\n", sched_offset);
			new_buf = ath_rt_wifi_get_buf_ap_tx(sc,
					sc->rt_wifi_superframe[sched_offset].sta_id);
		} else if (sc->rt_wifi_superframe[sched_offset].type == RT_WIFI_SHARED) {
			RT_WIFI_DEBUG("RT_WIFI_SHARED(%d)\n", sched_offset);
			new_buf = ath_rt_wifi_get_buf_ap_shared(sc);
		}
	} else if (sc->sc_ah->opmode == NL80211_IFTYPE_STATION) {
		if (sc->rt_wifi_superframe[sched_offset].type == RT_WIFI_TX
		&& sc->rt_wifi_superframe[sched_offset].sta_id == RT_WIFI_LOCAL_ID) {
			RT_WIFI_DEBUG("RT_WIFI_TX(%d)\n", sched_offset);
			new_buf = ath_rt_wifi_get_buf_sta(sc);
		}
	}

	ath_rt_wifi_tx(sc, new_buf);
	
}

bool rt_wifi_authorized_sta(u8 *addr) {
	int i;

	for (i = 0; i < RT_WIFI_STAS_NUM; i++) {
		if ( addr[0]== RT_WIFI_STAS[i].mac_addr[0] &&
			addr[1] == RT_WIFI_STAS[i].mac_addr[1] &&
			addr[2] == RT_WIFI_STAS[i].mac_addr[2] &&
			addr[3] == RT_WIFI_STAS[i].mac_addr[3] &&
			addr[4] == RT_WIFI_STAS[i].mac_addr[4] &&
			addr[5] == RT_WIFI_STAS[i].mac_addr[5] ) {
			return true;
		}
	}
	return false;
}

bool rt_wifi_dst_sta(u8 *addr, u8 sta_id) {

	if (addr[0] == RT_WIFI_STAS[sta_id].mac_addr[0] &&
		addr[1] == RT_WIFI_STAS[sta_id].mac_addr[1] &&
		addr[2] == RT_WIFI_STAS[sta_id].mac_addr[2] &&
		addr[3] == RT_WIFI_STAS[sta_id].mac_addr[3] &&
		addr[4] == RT_WIFI_STAS[sta_id].mac_addr[4] &&
		addr[5] == RT_WIFI_STAS[sta_id].mac_addr[5] ) {
		return true;
	}
	return false;
}

//STA�������յ�beacon֡//
void ath_rt_wifi_rx_beacon(struct ath_softc *sc, struct sk_buff *skb)
{
	struct ath_hw *ah = sc->sc_ah;
	unsigned char *data;
	unsigned char slot_size;
	u64 local_tsf;
	__le16 stype;
	struct ieee80211_mgmt *mgmt;
	static u8 first_beacon = 1;

	mgmt = (void*)skb->data;
	stype = mgmt->frame_control & cpu_to_le16(IEEE80211_FCTL_STYPE);

	data = (unsigned char*)skb->data;
	data = data + skb->len - RT_WIFI_BEACON_VEN_EXT_SIZE - BEACON_FCS_SIZE;

	if (*data != RT_WIFI_BEACON_TAG) {
		RT_WIFI_ALERT("TAG mismatch %x\n", *data);
	}
	else {
		// skip the first beacon for TSF synchronization
		if (first_beacon == 1) {
			first_beacon = 0;
			return;
		}
		/*
		rt_wifi_cur_tsf��ֵ�Ǵ�beacon֡���棬��һ�仰˵����Ӧ����beacon.c�����ath9k_beacon_generate�����rt_wifi_bc_tsf = RT_WIFI_BEACON_INTVAL=102400��
		rt_wifi_asnͬ��Ҳ����beacon֡�����rt_wifi_bc_asn = RT_WIFI_BEACON_INTVAL / sc->rt_wifi_slot_len��
		local_tsf�Ǵӱ�����Ӳ�����������ȡ��
		*/
		memcpy((unsigned char*)(&sc->rt_wifi_cur_tsf), (data+6), sizeof(u64));
		RT_WIFI_DEBUG("beacon current tsf: %llu\n", sc->rt_wifi_cur_tsf);
		local_tsf = ath9k_hw_gettsf64(ah); 

		if(local_tsf >= (sc->rt_wifi_cur_tsf - RT_WIFI_TSF_SYNC_OFFSET)) {
			// Process beacon information
			memcpy((unsigned char*)(&sc->rt_wifi_asn), (data+2), sizeof(int));
			RT_WIFI_DEBUG("beacon asn: %u\n", sc->rt_wifi_asn);

			memcpy(&slot_size, (data+14), sizeof(u8));
			sc->rt_wifi_slot_len = rt_wifi_get_slot_len(slot_size);
			RT_WIFI_DEBUG("slot len: %u\n", sc->rt_wifi_slot_len);

			memcpy((unsigned char*)(&sc->rt_wifi_superframe_size), (data+15), sizeof(u16));
			RT_WIFI_DEBUG("sf size: %u\n", sc->rt_wifi_superframe_size);

			rt_wifi_config_superframe(sc);

			ath_rt_wifi_sta_start_timer(sc);
			sc->rt_wifi_enable = 1;
		} else {
			RT_WIFI_DEBUG("local_tsf: %llu < rt_wifi_cur_tsf: %llu - TSF_SYNC_OFFSET\n",
				local_tsf, sc->rt_wifi_cur_tsf);
		}
	}
}
