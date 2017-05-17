/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2013 by Embedded and Real-Time Systems Laboratory
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 * 
 *  上記著作権者は，以下の(1)～(4)の条件を満たす場合に限り，本ソフトウェ
 *  ア（本ソフトウェアを改変したものを含む．以下同じ）を使用・複製・改
 *  変・再配布（以下，利用と呼ぶ）することを無償で許諾する．
 *  (1) 本ソフトウェアをソースコードの形で利用する場合には，上記の著作
 *      権表示，この利用条件および下記の無保証規定が，そのままの形でソー
 *      スコード中に含まれていること．
 *  (2) 本ソフトウェアを，ライブラリ形式など，他のソフトウェア開発に使
 *      用できる形で再配布する場合には，再配布に伴うドキュメント（利用
 *      者マニュアルなど）に，上記の著作権表示，この利用条件および下記
 *      の無保証規定を掲載すること．
 *  (3) 本ソフトウェアを，機器に組み込むなど，他のソフトウェア開発に使
 *      用できない形で再配布する場合には，次のいずれかの条件を満たすこ
 *      と．
 *    (a) 再配布に伴うドキュメント（利用者マニュアルなど）に，上記の著
 *        作権表示，この利用条件および下記の無保証規定を掲載すること．
 *    (b) 再配布の形態を，別に定める方法によって，TOPPERSプロジェクトに
 *        報告すること．
 *  (4) 本ソフトウェアの利用により直接的または間接的に生じるいかなる損
 *      害からも，上記著作権者およびTOPPERSプロジェクトを免責すること．
 *      また，本ソフトウェアのユーザまたはエンドユーザからのいかなる理
 *      由に基づく請求からも，上記著作権者およびTOPPERSプロジェクトを
 *      免責すること．
 * 
 *  本ソフトウェアは，無保証で提供されているものである．上記著作権者お
 *  よびTOPPERSプロジェクトは，本ソフトウェアに関して，特定の使用目的
 *  に対する適合性も含めて，いかなる保証も行わない．また，本ソフトウェ
 *  アの利用により直接的または間接的に生じたいかなる損害に関しても，そ
 *  の責任を負わない．
 * 
 *  @(#) $Id: pl310.c 2742 2016-01-09 04:25:18Z ertl-honda $
 */

#include "pl310.h"

/* Bitmask of active ways */
static uint32_t pl310_way_mask;  

static inline void cache_wait_way(uint32_t *reg, unsigned long mask)
{
	/* wait for cache operation by line or way to complete */
	while ( sil_rew_mem((void*)(reg)) & mask );
}

static inline void cache_sync(void)
{
	sil_wrw_mem((void*)(RMA1_L2CACHE_BASE + PL310_CACHE_SYNC), 0);
}

static inline void pl310_inv_all(void)
{
	/* invalidate all ways */
	sil_wrw_mem((void*)(RMA1_L2CACHE_BASE + PL310_INV_WAY), pl310_way_mask);
	cache_wait_way((uint32_t *)(RMA1_L2CACHE_BASE+PL310_INV_WAY), pl310_way_mask);
	cache_sync();
}

void
pl310_init(uint32_t aux_val, uint32_t aux_mask)
{
	uint32_t tmp;
	uint32_t aux;
	uint32_t cache_id;
	uint32_t prefetch;
	uint32_t prefetch_val = 0;    
	uint32_t power;
	int ways;

	/* L2キャッシュがすでにオンになっているか確認する */
	tmp = sil_rew_mem((void*)(RMA1_L2CACHE_BASE + PL310_CTRL));

	/* L2キャッシュが無効の場合のみ初期化を実施する */
	if ( !(tmp & 1) ) {
		cache_id = sil_rew_mem((void*)(RMA1_L2CACHE_BASE + PL310_CACHE_ID));
		aux = sil_rew_mem((void*)(RMA1_L2CACHE_BASE + PL310_AUX_CTRL));
		prefetch = sil_rew_mem((void*)(RMA1_L2CACHE_BASE + PL310_PREFETCH_CTRL));
		power = sil_rew_mem((void*)(RMA1_L2CACHE_BASE + PL310_POWER_CTRL));

		if (aux & (1 << 16)) {
			ways = 16;
		}
		else {
			ways = 8;
		}

		aux_val |= 1 << 22;
        
		aux_val |= 1 << 29;
		prefetch_val |= 1 << 29;

		aux_val |= 1 << 28;
		prefetch_val |= 1 << 28;


		if ((cache_id & 0x3f) > 0x6) {
			prefetch_val |= 1 << 30;
		}
                
		pl310_way_mask = (1 << ways) - 1;

		aux &= aux_mask;
		aux |= aux_val;
		prefetch |= prefetch_val;

		sil_wrw_mem((void*)(RMA1_L2CACHE_BASE + PL310_AUX_CTRL), aux);
		sil_wrw_mem((void*)(RMA1_L2CACHE_BASE + PL310_PREFETCH_CTRL), prefetch);
		sil_wrw_mem((void*)(RMA1_L2CACHE_BASE + PL310_POWER_CTRL), power);
        
		pl310_inv_all();
        
		sil_wrw_mem((void*)(RMA1_L2CACHE_BASE + PL310_CTRL), 1);
	}    
}

void
pl310_debug_set(uint32_t val)
{
	sil_wrw_mem((void*)(RMA1_L2CACHE_BASE + PL310_DEBUG_CTRL), val);
}
