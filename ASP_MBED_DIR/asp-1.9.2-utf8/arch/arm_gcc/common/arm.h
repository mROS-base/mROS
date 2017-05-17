/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel  
 * 
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
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
 *  @(#) $Id: arm.h 2757 2016-03-10 15:03:12Z ertl-honda $
 */

/*
 *  ARMのハードウェア資源の定義
 */

#ifndef TOPPERS_ARM_H
#define TOPPERS_ARM_H

/* 
 *  ARM のハードウェア資源のツール依存部の定義 
 */ 
#include <arm_tool.h>

/*
 *  ARM 例外ベクタ
 */
#define SVC_Vector    UINT_C(0x00)
#define UND_Vector    UINT_C(0x04)
#define SWI_Vector    UINT_C(0x08)
#define PRFA_Vector   UINT_C(0x0C)
#define DATAA_Vector  UINT_C(0x10)
#define IRQ_Vector    UINT_C(0x18)
#define FIQ_Vector    UINT_C(0x1C)

/*
 * ARM 例外ベクタ番号
 */
#define SVC_Number    UINT_C(0)
#define UND_Number    UINT_C(1)
#define SWI_Number    UINT_C(2)
#define PRFA_Number   UINT_C(3)
#define DATAA_Number  UINT_C(4)
#define UNNOWN_Number UINT_C(5)
#define IRQ_Number    UINT_C(6)
#define FIQ_Number    UINT_C(7)

/*
 *  CPSR 割込み禁止ビット
 */
#define CPSR_INT_MASK UINT_C(0xC0)
#define CPSR_IRQ_BIT  UINT_C(0x80)
#define CPSR_FIQ_BIT  UINT_C(0x40)

/*
 *  CPSR のモードビット
 */
#define CPSR_MODE_MASK   UINT_C(0x1f)
#define CPSR_USER        UINT_C(0x10)
#define CPSR_FIQ         UINT_C(0x11)
#define CPSR_IRQ         UINT_C(0x12)
#define CPSR_SVC         UINT_C(0x13)
#define CPSR_ABT         UINT_C(0x17)
#define CPSR_UND         UINT_C(0x1B)
#define CPSR_SYS         UINT_C(0x1F)

/*
 *  コプロのビット定義
 */
#define CP15_CONTROL_XP_BIT   (1 << 23)
#define CP15_CONTROL_V_BIT    (1 << 13)
#define CP15_CONTROL_I_BIT    (1 << 12)
#define CP15_CONTROL_Z_BIT    (1 << 11)
#define CP15_CONTROL_C_BIT    (1 <<  2)
#define CP15_CONTROL_M_BIT    (1 <<  0)

#if __TARGET_ARCH_ARM == 6
#define CP15_AUXILIARY_SA_BIT (1 << 5)
#define CP15_CPUID_BIT        (0x0f)
#else /* __TARGET_ARCH_ARM == 7 */
#define CP15_AUXILIARY_SA_BIT (1 << 6)
#define CP15_AUXILIARY_EX_BIT (1 << 7)
#define CP15_CPUID_BIT        (0x03)
#endif /* __TARGET_ARCH_ARM == 7 */

#define CP15_TTB0_RGN_S       (1 << 1)
#define CP15_TTB0_RGN_WBWA    (1 << 3)

#if __TARGET_ARCH_ARM == 7
#define CP15_TTB0_IRGN_WBWA   ((1 << 6)|(0))
#endif /* __TARGET_ARCH_ARM == 7 */

#ifndef TOPPERS_MACRO_ONLY

/*
 *  コプロを用いたルーチン
 */
#if (__TARGET_ARCH_ARM == 6) || (__TARGET_ARCH_ARM == 7)
/*
 *  High exception vector を使うかの設定
 */
Inline void
set_high_vector(bool_t enable)
{
	uint32_t control;

	CP15_CONTROL_READ(control);

	if (enable) {
		control |= CP15_CONTROL_V_BIT;
	}
	else {
		control &= ~CP15_CONTROL_V_BIT;
	}

	CP15_CONTROL_WRITE(control);
}

/*
 *  プロセッサINDEX（0オリジン）の取得
 */
Inline uint32_t
x_prc_index(void)
{
	uint32_t index;

	CP15_CPUID_READ(index);
	return((index & 0x0fU));
}

/*
 *  Data Synchronization Barrier
 *  ・先に発行された read と write の終了を待つ
 *  ・キャッシュ，ブランチプリディクション，TLBの操作の終了を待つ
 *  ・この命令の後に書かれた命令は実行されない 
 */
Inline void
data_sync_barrier(void)
{
	CP15_DATA_SYNC_BARRIER();
}

/*
 *  Data Memory Barrier
 *  ・プログラムの記述に従って，先に書かれた命令でのメモリアクセスが
 *     終了するまで待つ．
 */
Inline void
data_memory_barrier(void)
{
	CP15_DATA_MEMORY_BARRIER();
}


/* 
 *  キャッシュ関連
 */
/*
 *  TLBの無効化
 */
Inline void
invalidate_unfied_tlb(void)
{
	CP15_DATA_SYNC_BARRIER();
}

/*
 *  Dキャッシュの無効化
 */
Inline void
dcache_invalidate(void)
{
#if __TARGET_ARCH_ARM == 6
	CP15_DCACHE_INVALIDATE();
#elif __TARGET_ARCH_ARM == 7
	uint32_t bits = 0;
	uint32_t ways = 0;
	uint32_t sets = 0;

	CP15_CACHE_SIZE_SELECTION_WRITE(0);
	CP15_PBUFFER_FLUSH();
	for (ways = 0; ways < 4; ways++){
		for (sets = 0; sets < 256; sets++){
			bits = ways << 30 | sets << 5;
			CP15_DCACHE_INVALIDATE(bits);
		}
	}
#endif /* __TARGET_ARCH_ARM == 7 */
}

/*
 *  Dキャッシュのクリーン
 */
Inline void
dcache_clean(void)
{
#if __TARGET_ARCH_ARM == 6
#error
#elif __TARGET_ARCH_ARM == 7
	uint32_t bits = 0;
	uint32_t  ways = 0;
	uint32_t sets = 0;

	CP15_CACHE_SIZE_SELECTION_WRITE(0);
	CP15_PBUFFER_FLUSH();
	for (ways = 0; ways < 4; ways++){
		for (sets = 0; sets < 256; sets++){
			bits = ways << 30 | sets << 5;
			CP15_DCACHE_CLEAN(bits);
		}
	}
#endif /* __TARGET_ARCH_ARM == 7 */
}

/*
 *  Dキャッシュのクリーンと無効化
 */
Inline void
dcache_clean_and_invalidate(void)
{
#if __TARGET_ARCH_ARM == 6
	CP15_DCACHE_CLEAN_AND_INVALIDATE();
#elif __TARGET_ARCH_ARM == 7
	uint32_t bits = 0;
	uint32_t  ways = 0;
	uint32_t sets = 0;

	CP15_CACHE_SIZE_SELECTION_WRITE(0);
	CP15_PBUFFER_FLUSH();
	for (ways = 0; ways < 4; ways++){
		for (sets = 0; sets < 256; sets++){
			bits = ways << 30 | sets << 5;
			CP15_DCACHE_CLEAN_AND_INVALIDATE(bits);
		}
	}
#endif /* __TARGET_ARCH_ARM == 7 */
}

/*
 *  Iキャッシュの無効化
 */
Inline void
icache_invalidate(void)
{
	CP15_ICACHE_INVALIDATE();
}

/*
 *  プリフェッチバッファをクリア
 */
Inline void
pbuffer_flash(void)
{
	CP15_PBUFFER_FLUSH();
}

/*
 *  ブランチプリディクターの全無効化
 */
Inline void
bpi_invalidate_all(void)
{
	CP15_BPI_INVALIDATEALL(0);
	CP15_DATA_SYNC_BARRIER();
	CP15_PBUFFER_FLUSH();
}

/*
 *  Dキャッシュを開始
 */
extern void dcache_enable(void);

/*
 *  Dキャッシュを停止
 */
extern void dcache_disable(void);

/*
 *  Iキャッシュの開始
 */
extern void icache_enable(void);

/*
 *  Iキャッシュを停止
 */
extern void icache_disable(void);
	 
/*
 *  I/Dキャッシュを両方を有効に
 */
extern void cache_enable(void);

/*
 *  I/Dキャッシュを両方を無効に
 */
extern void cache_disable(void);

/*
 *  ブランチプリディクターを有効に
 */
Inline void
btac_enable(void)
{
	uint32_t tmp;

	CP15_CONTROL_READ(tmp);
	tmp |= CP15_CONTROL_Z_BIT;
	CP15_CONTROL_WRITE(tmp);
}

/*
 *  MMU関連
 */
/*
 * 変換テーブルへの設定内容
 * va   : 仮想アドレス
 * pa   : 物理アドレス
 * size : サイズ
 * ns   : Secure/Non-Secure
 * s    : 共有指定
 * tex  : C Bとの組み合わせで変化
 * ap   : アクセス権
 * c    : キャッシュ
 * b    : バッファ
 */
typedef struct{
	uintptr_t   va;
	uintptr_t   pa;
	uint32_t    size;
#if __TARGET_ARCH_ARM == 7
	uint8_t     ns;
#endif /* __TARGET_ARCH_ARM == 7 */
	uint8_t     s;
	uint8_t     tex;
	uint8_t     ap;
	uint8_t     c;
	uint8_t     b;
}MEMORY_ATTRIBUTE;

/*
 *  MMUの初期化
 */
extern void mmu_init(void);

/*
 *  MMUによるメモリのマッピング
 */
extern void mmu_map_memory(MEMORY_ATTRIBUTE *m_attribute);
#endif /* (__TARGET_ARCH_ARM == 6) || (__TARGET_ARCH_ARM == 7) */

#endif  /* TOPPERS_MACRO_ONLY */
#endif /* TOPPERS_ARM_H */
