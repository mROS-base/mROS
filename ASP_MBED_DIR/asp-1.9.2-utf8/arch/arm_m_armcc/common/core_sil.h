/*
 *  TOPPERS Software
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 * 
 *  Copyright (C) 2008-2011 by Embedded and Real-Time Systems Laboratory
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
 *  @(#) $Id: core_sil.h 2728 2015-12-30 01:46:11Z ertl-honda $
 */

/*
 *   sil.hのコア依存部（ARM-M用）
 */

#ifndef TOPPERS_CORE_SIL_H
#define TOPPERS_CORE_SIL_H

#ifndef TOPPERS_MACRO_ONLY

/*
 *  NMIを除くすべての割込みの禁止
 */
Inline bool_t
TOPPERS_disint(void)
{
	uint32_t val;

	val = __disable_fiq();

	if (val == 1) {
		return (true);
	}
	else {
		return (false);
	}
}

Inline void
TOPPERS_enaint(bool_t locked)
{
	if (!locked) {
		__enable_fiq();
	}
}

/*
 *  全割込みロック状態の制御
 */
#define SIL_PRE_LOC      bool_t  TOPPERS_locked
#define SIL_LOC_INT()    ((void)(TOPPERS_locked = TOPPERS_disint()))
#define SIL_UNL_INT()    (TOPPERS_enaint(TOPPERS_locked))

#endif /* TOPPERS_MACRO_ONLY */

#endif /* TOPPERS_CORE_SIL_H */
