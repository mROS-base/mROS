/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2006-2011 by Embedded and Real-Time Systems Laboratory
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
 *  @(#) $Id:
 */

#ifndef TOPPERS_RMA1_H
#define TOPPERS_RMA1_H

/*
 *  サポートする機能の定義
 */
#define TOPPERS_TARGET_SUPPORT_ENA_INT		/* ena_intをサポートする */
#define TOPPERS_TARGET_SUPPORT_DIS_INT		/* dis_intをサポートする */
#define TOPPERS_TARGET_SUPPORT_GET_UTM		/* get_utmをサポートする */

/*
 *  SCIFのベースアドレス
 */
#define SCIFA0_BASE  0xE6C40000
#define SCIFA1_BASE  0xE6C50000
#define SCIFA2_BASE  0xE6C60000
#define SCIFA3_BASE  0xE6C70000
#define SCIFA4_BASE  0xE6C80000
#define SCIFA5_BASE  0xE6CB0000
#define SCIFA6_BASE  0xE6CC0000
#define SCIFA7_BASE  0xE6CD0000
#define SCIFB_BASE   0xE6C30000

/*
 *  SCIFの割込み番号
 */
#define SCIFA0_INTNO 132
#define SCIFA1_INTNO 133
#define SCIFA2_INTNO 134
#define SCIFA3_INTNO 135
#define SCIFA4_INTNO 136
#define SCIFA5_INTNO 137
#define SCIFA6_INTNO 138
#define SCIFA7_INTNO 139
#define SCIFB_INTNO  140

/*
 *  GICに関する設定
 */
#define GICC_BASE     0xC2000000 //CPUインタフェースベースアドレス
#define GICD_BASE     0xC2800000 //DISTインタフェースベースアドレス

#define GIC_PRI_LEVEL         16 //優先度の個数,256,128,64,32,16
#define GIC_TNUM_INT         352 //割込み数

/*
 *  割込み優先度に関する設定
 */
#define TMIN_INTPRI   (-(GIC_PRI_LEVEL - 1))   /* 割込み優先度の最小値（最高値）*/
#define TMAX_INTPRI   (-1)    /* 割込み優先度の最大値（最低値）*/

/*
 *  reset/wdt
 */
#define MODEMR           0xFFCC0020
#define MODEMR_MD11      (1 << 11)
#define MODEMR_MD12      (1 << 12)

/*
 *  タイムティックの定義
 */
#define TIC_NUME     1U            /* タイムティックの周期の分子 */
#define TIC_DENO     1U            /* タイムティックの周期の分母 */

#endif /* TOPPERS_RMA1_H */
