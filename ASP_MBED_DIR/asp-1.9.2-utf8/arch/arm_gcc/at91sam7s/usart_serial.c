/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2009-2013 by Embedded and Real-Time Systems Laboratory
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

/*
 *		シリアルI/Oデバイス（SIO）ドライバ（mindstormsNXT用）
 */
#include "target_serial.h"
#include "at91sam7s.h"
#include <kernel.h>
#include <t_stddef.h>
#include <t_syslog.h>

/*
 *  シリアルI/Oポート初期化ブロックの定義
 */
typedef struct sio_port_initialization_block 
{
	uint32_t uart_base;
	INTNO    intno;
}SIOPINIB;

/*
 *  シリアルI/Oポート管理ブロックの定義
 */
struct sio_port_control_block 
{
	const SIOPINIB  *p_siopinib;  /* シリアルI/Oポート初期化ブロック */
	intptr_t  exinf;              /* 拡張情報 */ 
	bool_t    openflag;           /* オープン済みフラグ */
	bool_t    sendflag;           /* 送信割込みイネーブルフラグ */
	bool_t    getready;           /* 文字を受信した状態 */
	bool_t    putready;           /* 文字を送信できる状態 */
};

/*
 * シリアルI/Oポート初期化ブロック
 */
const SIOPINIB siopinib_table[TNUM_SIOP] = {
	{TADR_US_BASE, INTNO_US0},  /* US0 */
	{(TADR_US_BASE + US_WINDOW), INTNO_US0}   /* US1 */
};

/*
 *  シリアルI/Oポート管理ブロックのエリア
 */
SIOPCB	siopcb_table[TNUM_SIOP];

/*
 * 文字を受信したか?
 */ 
Inline bool_t
uart_getready(SIOPCB *p_siopcb)
{
	return ((sil_rew_mem((void*)(p_siopcb->p_siopinib->uart_base+TOFF_US_CSR)) & US_RXRDY) != 0);
}

/*
 *  送信可能か
 */
Inline bool_t
uart_putready(SIOPCB *p_siopcb)
{
	return ((sil_rew_mem((void*)(p_siopcb->p_siopinib->uart_base+TOFF_US_CSR)) & US_TXEMPTY) != 0);
}

/*
 *  受信した文字の取り出し
 */
Inline uint8_t
uart_getchar(SIOPCB *p_siopcb)
{
	uint8_t c;

	c = sil_rew_mem((void *)(p_siopcb->p_siopinib->uart_base+TOFF_US_RHR));

	return c;
}


/*
 *  送信する文字の書き込み
 */
Inline void
uart_putchar(SIOPCB *p_siopcb, uint8_t c)
{
	sil_wrw_mem((void*)(p_siopcb->p_siopinib->uart_base + TOFF_US_THR), c);
}

/*
 *  送信割込み許可
 *  送信バッファ空割込み
 */
Inline void
uart_enable_send(SIOPCB *p_siopcb)
{
	sil_wrw_mem((void*)(p_siopcb->p_siopinib->uart_base + TOFF_US_IER), US_TXEMPTY);
}

/*
 *  送信割込み禁止
 *  送信バッファ空割込み
 */
Inline void
uart_disable_send(SIOPCB *p_siopcb)
{
	sil_wrw_mem((void*)(p_siopcb->p_siopinib->uart_base+TOFF_US_IDR), US_TXEMPTY);
}


/*
 *  受信割込み許可
 */
Inline void
uart_enable_rcv(SIOPCB *p_siopcb)
{
	sil_wrw_mem((void*)(p_siopcb->p_siopinib->uart_base+TOFF_US_IER), US_RXRDY);
}

/*
 *  受信割込み禁止
 */
Inline void
uart_disable_rcv(SIOPCB *p_siopcb)
{
    sil_wrw_mem((void*)(p_siopcb->p_siopinib->uart_base+TOFF_US_IDR), US_RXRDY);
}

/*
 *  シリアルI/Oポートのオープン
 */
SIOPCB *
sio_opn_por(ID siopid, intptr_t exinf)
{
	SIOPCB	*p_siopcb = &siopcb_table[siopid - 1];
	const SIOPINIB  *p_siopinib;
	bool_t opnflg;
	ER      ercd;

	p_siopinib = p_siopcb->p_siopinib;
	opnflg = p_siopcb->openflag;

	if(!opnflg) {
		sil_wrw_mem((void*)(TADR_PMC_BASE+TOFF_PMC_PCER), 
					sil_rew_mem((void*)(TADR_PMC_BASE+TOFF_PMC_PCER)) | (1 << 6));
		sil_wrw_mem((void*)(TADR_PIO_BASE + TOFF_PIO_PDR), (1 <<  5)|(1 <<  6));
		sil_wrw_mem((void*)(p_siopcb->p_siopinib->uart_base + TOFF_US_CR),
					US_RSTRX |          /* Reset Receiver      */
					US_RSTTX |          /* Reset Transmitter   */
					US_RXDIS |          /* Receiver Disable    */
					US_TXDIS            /* Transmitter Disable */
					);
		sil_wrw_mem((void*)(p_siopcb->p_siopinib->uart_base + TOFF_US_MR),
					US_CHMODE_NORMAL |  /* Normal Mode */
					US_CLKS_MCK      |  /* Clock = MCK */
					US_CHRL_8        |  /* 8-bit Data  */
					US_PAR_NO        |  /* No Parity   */
					US_NBSTOP_1         /* 1 Stop Bit  */
					);
	sil_wrw_mem((void*)(p_siopcb->p_siopinib->uart_base + TOFF_US_BRGR), BRD);
		sil_wrw_mem((void*)(p_siopcb->p_siopinib->uart_base + TOFF_US_CR),
					US_RXEN  |          /* Receiver Enable     */
					US_TXEN             /* Transmitter Enable  */
					);

		ercd = ena_int(p_siopcb->p_siopinib->intno);
		assert(ercd == E_OK);
	}

	sil_rew_mem((void*)(p_siopinib->uart_base+TOFF_US_RHR));

	p_siopcb->exinf = exinf;
	p_siopcb->getready = p_siopcb->putready = false;
	p_siopcb->openflag = true;

	return p_siopcb;
}

/* 
 *  シリアルI/Oポートのクローズ
 */
void
sio_cls_por(SIOPCB *p_siopcb)
{
	sil_wrw_mem((void *)(p_siopcb->p_siopinib->uart_base + TOFF_US_CR), US_RSTRX|US_RSTTX|US_RXDIS|US_TXDIS);
	
	p_siopcb->openflag = false;

	dis_int(p_siopcb->p_siopinib->intno);
}

/*
 *  シリアルI/Oポートへの文字送信
 */
bool_t
sio_snd_chr(SIOPCB *siopcb, char c)
{
	if (uart_putready(siopcb)) {
		uart_putchar(siopcb, c);
		return(true);
	}
	return(false);
}

/*
 *  シリアルI/Oポートからの文字受信
 */
int_t
sio_rcv_chr(SIOPCB *siopcb)
{
	if (uart_getready(siopcb)) {
		return((int_t)(uint8_t) uart_getchar(siopcb));
	}
	return(false);
}

/*
 *  シリアルI/Oポートからのコールバックの許可
 */
void
sio_ena_cbr(SIOPCB *siopcb, uint_t cbrtn)
{
	switch (cbrtn) {
	  case SIO_RDY_SND:
		uart_enable_send(siopcb);
		break;
	  case SIO_RDY_RCV:
		uart_enable_rcv(siopcb);
		break;
	}
}

/*
 *  シリアルI/Oポートからのコールバックの禁止
 */
void
sio_dis_cbr(SIOPCB *siopcb, uint_t cbrtn)
{
	switch (cbrtn) {
	  case SIO_RDY_SND:
		uart_disable_send(siopcb);
		break;
	  case SIO_RDY_RCV:
		uart_disable_rcv(siopcb);
		break;
	}
}

/*
 *  SIOドライバの初期化
 */
void
sio_initialize(intptr_t exinf)
{
	SIOPCB  *p_siopcb;
	uint_t  i;

	/*
	 *  シリアルI/Oポート管理ブロックの初期化
	 */
	for (p_siopcb = siopcb_table, i = 0; i < TNUM_SIOP; p_siopcb++, i++) {
		p_siopcb->p_siopinib = &(siopinib_table[i]);
		p_siopcb->openflag = false;
		p_siopcb->sendflag = false;
	}
}


void
sio_isr(intptr_t exinf)
{
	SIOPCB *p_siopcb = &(siopcb_table[0]);

	if (uart_getready(p_siopcb)) {
		/*
		 *  受信通知コールバックルーチンを呼び出す．
		 */
		sio_irdy_rcv(p_siopcb->exinf);
	}
	if (uart_putready(p_siopcb)) {
		/*
         *  送信可能コールバックルーチンを呼び出す．
		 */
		sio_irdy_snd(p_siopcb->exinf);
	}
}
