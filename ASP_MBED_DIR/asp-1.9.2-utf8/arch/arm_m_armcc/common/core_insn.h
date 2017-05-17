/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
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
 *  @(#) $Id: core_insn.h 2728 2015-12-30 01:46:11Z ertl-honda $
 */


/*
 *  コア依存の特殊命令のインライン関数定義（ARM-M用）
 */

#ifndef CORE_INSN_H
#define CORE_INSN_H

#include <arm_m.h>

/*
 *  メモリが変更されることをコンパイラに伝えるためのマクロ
 */
#define ARM_MEMORY_CHANGED __memory_changed()

/*
 *  FAULTMASKのセット
 */
Inline void
set_faultmask(void){
	__disable_fiq();
}

/*
 *  FAULTMASKのクリア
 */
Inline void
clear_faultmask(void){
	__enable_fiq();
}

/*
 *  PRIMASKのセット
 */
Inline void
set_primask(void){
	__disable_irq();
}

/*
 *  PRIMASKのクリア
 */
Inline void
clear_primask(void){
	__enable_irq();
}

/*
 *  BASEPRIのセット
 */
Inline void
set_basepri(uint32_t val){
	register uint32_t basepri __asm("basepri");
	basepri = val;
}

/*
 *  BASEPRIの取得
 */
Inline uint32_t
get_basepri(void){
	register uint32_t basepri __asm("basepri");
	return basepri;
}

/*
 *  CONTROLのセット
 */
Inline void
set_control(uint32_t val){
	register uint32_t control __asm("control");
	control = val;
	/*
	 *  controlレジスタセット後にはisbが必須
	 *  [ARMv7-M Architecture Reference Manaual(DDI0403B) A3-37]
	 */
	__isb(0xf);
}

/*
 *  CONTROLの取得
 */
Inline uint32_t
get_control(void){
	register uint32_t control __asm("control");
	return(control);
}

/*
 *  ステータスレジスタ（IPSR）の現在値の読出し
 */
Inline uint32_t
get_ipsr(void)
{
	register uint32_t ipsr __asm("ipsr");
	return(ipsr);
}

#endif /* CORE_INSN_H */
