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
 *  @(#) $Id: gic.h 2766 2016-03-13 00:24:24Z ertl-honda $
 */

/*
 *  ARM Generic Interrupt Controller ドライバ
 */

#ifndef TOPPERS_GIC_H
#define TOPPERS_GIC_H

#include <sil.h>
#include <arm.h>

/*
 *  CPU Interface
 *
 *  優先度は，0x0～0xF の16段階．
 *  0x0が最高優先度で，0xFが最低優先度．
 */
#define GICC_CTLR_EN      0x01
#define GICC_CTLR_ENABLEGRP0 0x01
#define GICC_CTLR_ENABLEGRP1 0x02
#define GICC_CTLR_CBPR       0x10
#define GICC_CTLR_FIQEn      0x08

#if GIC_PRI_LEVEL == 16
#define GICC_PMR_OFFSET 0x04
#define GICC_PMR_MASK   0x0f
#elif GIC_PRI_LEVEL == 32
#define GICC_PMR_OFFSET 0x03
#define GICC_PMR_MASK   0x1f
#elif GIC_PRI_LEVEL == 64
#define GICC_PMR_OFFSET 0x02
#define GICC_PMR_MASK   0x3f
#elif GIC_PRI_LEVEL == 128
#define GICC_PMR_OFFSET 0x01
#define GICC_PMR_MASK   0x7f
#elif GIC_PRI_LEVEL == 256
#define GICC_PMR_OFFSET 0x00
#define GICC_PMR_MASK   0xff
#else
#error
#endif /* GIC_PRI_LEVEL == 16 */

/*
 *  Distributor
 */
#define GICD_CTLR_ENABLE 0x01

/*
 *  割込み先のプロセッサの指定
 */
#define GICD_ITARGETSRn_CPU0  0x01
#define GICD_ITARGETSRn_CPU1  0x02
#define GICD_ITARGETSRn_CPU2  0x04
#define GICD_ITARGETSRn_CPU3  0x08

#define GICD_SGIR_CPU0  0x01
#define GICD_SGIR_CPU1  0x02
#define GICD_SGIR_CPU2  0x04
#define GICD_SGIR_CPU3  0x08
#define GICD_SGIR_CPUS  0x0f

#define GICD_SGIR_CPU_OFFSET 16

/*
 *  割込み優先度
 */
#define GIC_MAX_PRIORITY  0x00
#define GIC_MIN_PRIORITY  GICC_PMR_MASK

/*
 *  コンフィギュレーションレジスタの設定値
 */
#define GICD_ICFGRn_EDGE     0x02   /* エッジ割込み */
#define GICD_ICFGRn_LEVEL    0x00   /* レベル割込み */
#define GICD_ICFGRn_N_N      0x00   /* N-Nモデル    */
#define GICD_ICFGRn_1_N      0x01   /* 1-Nモデル    */

/*
 *  GICでサポートしている割込み数
 */
#define GIC_TMIN_INTNO      0U



/*
 *  グローバル割込みの開始番号
 */
#define TMIN_GLOBAL_INTNO   32U

/*
 * 割込み番号(GICでの番号)
 */
#define GIC_IRQNO_IPI0     0
#define GIC_IRQNO_IPI1     1




#define ASP_GICD_BASE           0xE8201000
#define ASP_GICD_ICDDCR         (ASP_GICD_BASE)             /* ICDDCR, 32bit */
#define ASP_GICD_ICDICTR        (ASP_GICD_BASE + 0x004)     /* ICDICTR, 32bit */
#define ASP_GICD_ICDIIDR        (ASP_GICD_BASE + 0x008)     /* ICDIIDR, 32bit */
#define ASP_GICD_ICDISR         (ASP_GICD_BASE + 0x080)     /* ICDISR, offset 0x80 - 0xfc, 32bit */
#define ASP_GICD_ICDISER        (ASP_GICD_BASE + 0x100)     /* ICDISER, offset 0x100-0x17c, 32bit */
#define ASP_GICD_ICDICER        (ASP_GICD_BASE + 0x180)     /* ICDICER, offset 0x180-0x1fc, 32bit */
#define ASP_GICD_ICDISPR        (ASP_GICD_BASE + 0x200)     /* ICDISPR, offset 0x200-0x27c, 32bit */
#define ASP_GICD_ICDICPR        (ASP_GICD_BASE + 0x280)     /* ICDICPR, offset 0x280-0x2fc, 32bit */
#define ASP_GICD_ICDABR         (ASP_GICD_BASE + 0x300)     /* ICDABR, offset 0x300-0x37c, 32bit */
#define ASP_GICD_ICDIPR         (ASP_GICD_BASE + 0x400)     /* ICDIPR, offset 0x400-0x7fb, 8bit */
#define ASP_GICD_ICDIPTR        (ASP_GICD_BASE + 0x800)     /* ICDIPTR, offset 0x800-0xbfb, 8bit */
#define ASP_GICD_ICDICFR        (ASP_GICD_BASE + 0xC00)     /* ICDICR, offset 0xc00-0xcfc, 32bit */
#define ASP_GICD_PPIS           (ASP_GICD_BASE + 0xD00)     /* PPIステータスレジスタ, offset 0xd00, 32bit */
#define ASP_GICD_SPIS           (ASP_GICD_BASE + 0xD04)     /* SPIステータスレジスタ, offset 0xd00-0xd44, 32bit */
#define ASP_GICD_ICDSGIR        (ASP_GICD_BASE + 0xF00)     /* ICDSGIR, 32bit */

#define GICD_CTLR               ASP_GICD_ICDDCR
#define GICD_TYPER              ASP_GICD_ICDICTR
#define GICD_IIDR               ASP_GICD_ICDIIDR
#define GICD_IGROUPRn           ASP_GICD_ICDISR
#define GICD_ISENABLERn         ASP_GICD_ICDISER
#define GICD_ICENABLERn         ASP_GICD_ICDICER
#define GICD_ISPENDRn           ASP_GICD_ICDISPR
#define GICD_ICPENDRn           ASP_GICD_ICDICPR
#define GICD_ISACTIVERn         ASP_GICD_ICDABR
#define GICD_IPRIORITYRn        ASP_GICD_ICDIPR
#define GICD_ITARGETSRn         ASP_GICD_ICDIPTR
#define GICD_ICFGRn             ASP_GICD_ICDICFR
#define GICD_SGIR               ASP_GICD_ICDSGIR

#define GICD_PPIS               ASP_GICD_PPIS
#define GICD_SPIS               ASP_GICD_SPIS

#define ASP_NUM_ICDISR_ENTRY    ((GIC_TNUM_INT + 31) / 32)
#define ASP_NUM_ICDISER_ENTRY   ((GIC_TNUM_INT + 31) / 32)
#define ASP_NUM_ICDICER_ENTRY   ((GIC_TNUM_INT + 31) / 32)
#define ASP_NUM_ICDISPR_ENTRY   ((GIC_TNUM_INT + 31) / 32)
#define ASP_NUM_ICDICPR_ENTRY   ((GIC_TNUM_INT + 31) / 32)
#define ASP_NUM_ICDABR_ENTRY    ((GIC_TNUM_INT + 31) / 32)
#define ASP_NUM_ICDIPR_ENTRY    ((GIC_TNUM_INT + 3) / 4)
#define ASP_NUM_ICDIPTR_ENTRY   ((GIC_TNUM_INT + 3) / 4)
#define ASP_NUM_ICDICFR_ENTRY   ((GIC_TNUM_INT + 15) / 16)

#define ASP_GICC_BASE           0xE8202000
#define ASP_GICC_ICCICR         (ASP_GICC_BASE)             /* ICCICR, 32bit */
#define ASP_GICC_ICCPMR         (ASP_GICC_BASE + 0x004)     /* ICCPMR, 32bit */
#define ASP_GICC_ICCBPR         (ASP_GICC_BASE + 0x008)     /* ICCBPR, 32bit */
#define ASP_GICC_ICCIAR         (ASP_GICC_BASE + 0x00C)     /* ICCIAR, 32bit */
#define ASP_GICC_ICCEOIR        (ASP_GICC_BASE + 0x010)     /* ICCEOIR, 32bit */
#define ASP_GICC_ICCRPR         (ASP_GICC_BASE + 0x014)     /* ICCRPR, 32bit */
#define ASP_GICC_ICCHPIR        (ASP_GICC_BASE + 0x018)     /* ICCHPIR, 32bit */
#define ASP_GICC_ICCABPR        (ASP_GICC_BASE + 0x01C)     /* ICCABPR, 32bit */
#define ASP_GICC_ICCIIDR        (ASP_GICC_BASE + 0x0FC)     /* ICCIIDR, 32bit */

#define GICC_CTLR               ASP_GICC_ICCICR
#define GICC_PMR                ASP_GICC_ICCPMR
#define GICC_BPR                ASP_GICC_ICCBPR
#define GICC_IAR                ASP_GICC_ICCIAR
#define GICC_EOIR               ASP_GICC_ICCEOIR
#define GICC_RPR                ASP_GICC_ICCRPR


#ifndef TOPPERS_MACRO_ONLY

/*
 *  GIC CPU Interface 関連のドライバ
 */
/*
 *  CPUの割込み優先度マスクを設定
 */
Inline void
gicc_set_priority(int pri)
{
	sil_wrw_mem((void *)ASP_GICC_ICCPMR,
				(pri << GICC_PMR_OFFSET));
}

/*
 *  CPUの割込み優先度マスクを取得
 */
Inline int
gicc_current_priority(void)
{
	return (sil_rew_mem((void *)(ASP_GICC_ICCPMR)) >>
			GICC_PMR_OFFSET);
}

/*
 *  GICのプロセッサの割込み優先度のどのビットを使用するか
 */
Inline void
gicc_set_bp(int mask_bit)
{
	sil_wrw_mem((void *)ASP_GICC_ICCBPR, mask_bit);
}

/*
 *  GIC CPU Interface の初期化
 */
extern void gicc_init(void);

/*
  *  GIC CPU Interface の終了
 */
extern void gicc_stop(void);

/*
 *  Distributor 関連のドライバ
 */

/*
 *  割込み禁止
 */
extern void gicd_disable_int(uint8_t id);

/*
 *  割込み許可
 */
extern void gicd_enable_int(uint8_t id);

/*
 *  割込みペンディングクリア
 */
extern void gicd_clear_pending(uint8_t id);

/*
 *  割込みペンディングセット
 */
extern void gicd_set_pending(uint8_t id);

/*
 *  割込み要求のチェック
 */
extern bool_t gicd_probe_int(uint8_t id);

/*
 *  割込み設定のセット
 */
extern void gicd_config(uint8_t id, bool_t is_edge, bool_t is_1_n);

/*
 *  割込み優先度のセット
 *  内部表現で渡す．
 */
extern void gicd_set_priority(uint8_t id, int pri);

/*
 *  割込みターゲットの設定
 *  CPUはORで指定
 */
extern void gicd_set_target(uint8_t id, uint8_t cpus);

/*
 *  Ditoributor の初期化
 */
extern void gicd_init(void);

/*
 *  Ditoributor の初期化の終了
 */
extern void gicd_stop(void);

/*
 *  ソフトウェア割込みを発行
 */
Inline void
gic_raise_sgi(int cpu, int id)
{
	CP15_DATA_SYNC_BARRIER();
	sil_wrw_mem((void *)ASP_GICD_ICDSGIR, (cpu << GICD_SGIR_CPU_OFFSET)|id);
	CP15_DATA_SYNC_BARRIER();
}

/*
 *  ソフトウェア割込みをクリア
 */
Inline void
gic_clear_sgi(int cpu, int id)
{

}

#endif /* TOPPERS_MACRO_ONLY */
#endif /* TOPPERS_GIC_H */
