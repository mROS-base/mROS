/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2004-2012 by Embedded and Real-Time Systems Laboratory
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
 *  @(#) $Id: makeoffset.c 2728 2015-12-30 01:46:11Z ertl-honda $
 */

#include "kernel_impl.h"
#include "task.h"
#include "sil.h"

#define OFFSET_DEF(TYPE, FIELD)							\
	Asm("OFFSET_DEF " #TYPE "_" #FIELD " = %0"			\
	  : /* no output */									\
	  : "g"(offsetof(TYPE, FIELD)))

#define OFFSET_DEF2(TYPE, FIELD, FIELDNAME)				\
	Asm("OFFSET_DEF " #TYPE "_" #FIELDNAME " = %0"		\
	  : /* no output */									\
	  : "g"(offsetof(TYPE, FIELD)))

void
makeoffset(void)
{
	OFFSET_DEF(TCB, p_tinib);
	OFFSET_DEF(TCB, texptn);
	OFFSET_DEF2(TCB, tskctxb.msp, msp);
	OFFSET_DEF2(TCB, tskctxb.pc, pc);

	OFFSET_DEF(TINIB, exinf);
	OFFSET_DEF(TINIB, task);
}

void
sil_endian(void)
{
#ifdef SIL_ENDIAN_BIG			/* ビッグエンディアンプロセッサ */
	Asm("SIL_ENDIAN = BIG");
#else /* SIL_ENDIAN_BIG */
#ifdef SIL_ENDIAN_LITTLE		/* リトルエンディアンプロセッサ */
	Asm("SIL_ENDIAN = LITTLE");
#endif /* SIL_ENDIAN_LITTLE */
#endif /* SIL_ENDIAN_BIG */
}

uint32_t	BIT_REF_4 = 0x12345678;
uint16_t	BIT_REF_2 = 0x1234;
uint8_t		BIT_REF_1 = 0x12;

TCB	BIT_BB_TCB_enatex = {
	{ NULL, NULL },			/* task_queue */
	NULL,					/* p_tinib */
	0U,						/* tstat */
#ifdef TOPPERS_SUPPORT_MUTEX
	0U,						/* bpriority */
#endif /* TOPPERS_SUPPORT_MUTEX */
	0U,						/* priority */
	false,					/* acqeue */
	false,					/* wupque */
	true,					/* enatex */
	0U,						/* texptn */
	NULL,					/* p_winifo */
#ifdef TOPPERS_SUPPORT_MUTEX
	{ NULL, NULL },			/* mutex_queue */
#endif /* TOPPERS_SUPPORT_MUTEX */
#ifdef TOPPERS_SUPPORT_OVRHDR
	0U,						/* leftotm */
#endif /* TOPPERS_SUPPORT_OVRHDR */
	{ NULL, NULL }			/* tskctxb */
};
