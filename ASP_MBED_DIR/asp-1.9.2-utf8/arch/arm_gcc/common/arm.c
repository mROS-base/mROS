/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 *
 *  Copyright (C) 2006-2012 by Embedded and Real-Time Systems Laboratory
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
 *  @(#) $Id: arm.c 2742 2016-01-09 04:25:18Z ertl-honda $
 */

#include "kernel_impl.h"

#if (__TARGET_ARCH_ARM == 6) || (__TARGET_ARCH_ARM == 7)
/*
 *  Dキャッシュを開始
 */
void
dcache_enable(void)
{
	uint32_t bits;

	CP15_CONTROL_READ(bits);

	/* すでにONならリターン */
	if (bits & CP15_CONTROL_C_BIT){
		return;
	}

	dcache_invalidate();

	bits |= CP15_CONTROL_C_BIT;
	CP15_CONTROL_WRITE(bits);
}

/*
 *  Dキャッシュを停止して無効とする．
 *  CA9では，Dキャッシュが無効な状態でClean and Invalidate()を実行すると，
 *  暴走するため，Dキャッシュの状態を判断して，無効な場合は，Invalidate
 *  のみを行う． 
 */
void
dcache_disable(void)
{
	uint32_t bits;

	CP15_CONTROL_READ(bits);
	if( bits & CP15_CONTROL_C_BIT ){
		bits &= ~CP15_CONTROL_C_BIT;
		CP15_CONTROL_WRITE(bits);
		dcache_clean_and_invalidate();
	}
	else{
		dcache_invalidate();
	}
}

/*
 *  Iキャッシュの開始
 */
void
icache_enable(void)
{
	uint32_t bits;

	CP15_CONTROL_READ(bits);

	/*
	 *  すでに有効ならリターン
	 */
	if(bits & CP15_CONTROL_I_BIT){
		return;
	}

	icache_invalidate();

	bits |= CP15_CONTROL_I_BIT;
	CP15_CONTROL_WRITE(bits);
}

/*
 *  Iキャッシュを停止
 */
void
icache_disable(void)
{
	uint32_t bits;

	CP15_CONTROL_READ(bits);
	bits &= ~CP15_CONTROL_I_BIT;
	CP15_CONTROL_WRITE(bits);

	icache_invalidate();
}

/*
 *  I/Dキャッシュを両方を有効に
 */
void
cache_enable(void)
{
	dcache_enable();
	icache_enable();
}

/*
 *  I/Dキャッシュを両方を無効に
 */
void
cache_disable(void)
{
	dcache_disable();
	icache_disable();
}

/*
 *  MMU関連のドライバ
 */
#define DOMAIN_NO  1
#define SIZE_1M    0x00100000
#define PAGE_TABLE_SIZE  0x00004000 /* 4KB */

/*
 * ページテーブル
 */
static uint8_t PageTable[PAGE_TABLE_SIZE]  __attribute__ ((aligned (0x4000)));

/*
 * 指定に従い,メモリのマッピングを行う
 * マッピングの単位は1MB 
 */
void
mmu_map_memory(MEMORY_ATTRIBUTE *m_attribute)
{
	uint32_t size;
	uintptr_t va;
	uintptr_t pa;
	uint32_t ttb;
	uint32_t *sptr;
	uint32_t fix_val;
	
	size = m_attribute->size;
	va   = m_attribute->va;
	pa   = m_attribute->pa;

	/* MB単位になるようにサイズを設定 */
	size = (size + 0x000FFFFF) & ~0x000FFFFF;

	/*
	 * ページテーブルのベースアドレスを取得
	 */
	CP15_TTB0_READ(ttb);

	fix_val =  (m_attribute->s << 16) | (m_attribute->tex << 12) | (m_attribute->ap << 10)
		        | (m_attribute->c << 3) | (m_attribute->b << 2) | (1 << 1);
#if __TARGET_ARCH_ARM == 7
	fix_val |= (m_attribute->ns << 19);
#endif /* __TARGET_ARCH_ARM == 7 */
	sptr  = (uint32_t *)((ttb & 0xFFFFC000) | (((va & 0xFFF00000) >> 20) << 2));

	while(size > 0) {
		*sptr = pa | fix_val;

		sptr++;
		va   += SIZE_1M;
		pa   += SIZE_1M;
		size -= SIZE_1M;
	}
}

/*
 *  MMUの初期化
 */
void
mmu_init(void)
{
	uint32_t bits = 0;

	/* プリフェッチバッファをクリア */
	pbuffer_flash();

	/* TTBR0を用いるように指定 */
	CP15_TTBCR_WRITE(0);

	/*
	 * 変換テーブル(TT)として，PageTableを使用する
	 * Sharedビットをセット
	 */
#if __TARGET_ARCH_ARM == 6
	CP15_TTB0_WRITE((((uintptr_t)PageTable)|CP15_TTB0_RGN_S|CP15_TTB0_RGN_WBWA));
#else /* __TARGET_ARCH_ARM == 7 */
	CP15_TTB0_WRITE((((uintptr_t)PageTable)|CP15_TTB0_RGN_S|CP15_TTB0_RGN_WBWA|CP15_TTB0_IRGN_WBWA));
#endif /* __TARGET_ARCH_ARM == 7 */

	/* プリフェッチバッファをクリア */
	pbuffer_flash();

	/*
	 *  ターゲット依存部でのMMUの初期化
	 */
	target_mmu_init();

	/*
	 * ドメイン番号をセット
	 */
	CP15_DOMAINS_WRITE(DOMAIN_NO);

	/*
	 *  CONTROLコプロセッサの Mビット，XPビットをセットして，
	 *  MMUを有効にする
	 */
	CP15_CONTROL_READ(bits);
	bits |= CP15_CONTROL_M_BIT | CP15_CONTROL_XP_BIT;
	CP15_CONTROL_WRITE(bits);
}
#endif /* (__TARGET_ARCH_ARM == 6) || (__TARGET_ARCH_ARM == 7) */
