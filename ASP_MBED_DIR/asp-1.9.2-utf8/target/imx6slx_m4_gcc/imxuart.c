/*
 *  TOPPERS/FMP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Flexible MultiProcessor Kernel    
 * 
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2007-2015 by Embedded and Real-Time Systems Laboratory
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
 *  @(#) $Id: ns16550.c 1019 2013-07-11 13:29:21Z ertl-honda $
 */

/*
 *     IMXUART 用 簡易SIOドライバ
 */
#include <kernel.h>
#include <t_syslog.h>
#include <sil.h>
#include "target_syssvc.h"
#include "imxuart.h"

#define UART_URXD  (0x00)
#define UART_UTXD  (0x40)
#define UART_UCR1  (0x80)
#define UART_UCR2  (0x84)
#define UART_UCR3  (0x88)
#define UART_UCR4  (0x8C)
#define UART_UFCR  (0x90)
#define UART_USR1  (0x94)
#define UART_USR2  (0x98)
#define UART_UESC  (0x9C)
#define UART_UTIM  (0xA0)
#define UART_UBIR  (0xA4)
#define UART_UBMR  (0xA8)
#define UART_UBRC  (0xAC)
#define UART_ONEMS (0xB0)
#define UART_UTS   (0xB4)
#define UART_UMCR  (0xB8)

#define UART_UTS_TXFULL (1<<4)
#define UART_UTS_RXEMPTY (1<<5)

#define UART_UCR1_TXMPTYEN (1<<6)
#define UART_UCR1_RRDYEN   (1<<9)
#define UART_UCR1_UARTEN   (1<<0)

#define UART_UCR2_IRTS   (1<<14)
#define UART_UCR2_PREN   (1<<8)
#define UART_UCR2_STPB   (1<<6)
#define UART_UCR2_WS     (1<<5)
#define UART_UCR2_TXEN   (1<<2)
#define UART_UCR2_RXEN   (1<<1)
#define UART_UCR2_SRST   (1<<0)

#define UART_USR2_TXFE   (1<<14)


/*
 *  コールバックルーチンの識別番号
 */
#define SIO_RDY_SND	1U		/* 送信可能コールバック */
#define SIO_RDY_RCV	2U		/* 受信通知コールバック */

/*
 *  シリアルI/Oポート初期化ブロックの定義
 */
typedef struct sio_port_initialization_block {
	uint32_t reg_base;    /* レジスタのベースアドレス */
	uint8_t intno;        /* 割込み番号 */
} SIOPINIB;

/*
 *  シリアルI/Oポート管理ブロックの定義
 */
struct sio_port_control_block {
	const SIOPINIB	*p_siopinib; /* シリアルI/Oポート初期化ブロック */
	intptr_t		exinf;      /* 拡張情報 */
	bool_t			openflag;   /* オープン済みフラグ */
	bool_t			sendflag;   /* 送信割込みイネーブルフラグ */
	bool_t			getready;   /* 文字を受信した状態 */
	bool_t			putready;   /* 文字を送信できる状態 */
};


/*
 * シリアルI/Oポート初期化ブロック
 */
const SIOPINIB siopinib_table[TNUM_SIOP] = {
#ifdef TARGET_USE_UART1
	{UART1_BASE, IRQNO_UART1},
#elif defined(TARGET_USE_UART2)
	{UART2_BASE, IRQNO_UART2},
#elif defined(TARGET_USE_UART3)
	{UART3_BASE, IRQNO_UART3},
#endif /* TARGET_USE_UART1 */
};

/*
 *  シリアルI/Oポート初期化ブロックの取出し
 */
#define INDEX_SIOPINIB(siopid)  ((uint_t)((siopid) - 1))
#define get_siopinib(siopid)  (&(siopinib_table[INDEX_SIOPINIB(siopid)]))

/*
 *  シリアルI/Oポート管理ブロックのエリア
 */
SIOPCB  siopcb_table[TNUM_SIOP];

/*
 *  シリアルI/OポートIDから管理ブロックを取り出すためのマクロ
 */
#define INDEX_SIOP(siopid)  ((uint_t)((siopid) - 1))
#define get_siopcb(siopid)  (&(siopcb_table[INDEX_SIOP(siopid)]))


/*
 * 文字を受信したか?
 */ 
Inline bool_t
imxuart_getready(SIOPCB *p_siopcb)
{
	uint16_t status;

	status = sil_rew_mem((void*)(p_siopcb->p_siopinib->reg_base + UART_UTS));
	return(!(status & UART_UTS_RXEMPTY));
}

/*
 * 文字を送信できるか?
 */
Inline bool_t
imxuart_putready(SIOPCB *p_siopcb)
{
	uint16_t status;

	status = sil_rew_mem((void*)(p_siopcb->p_siopinib->reg_base + UART_UTS));
	return(!(status & UART_UTS_TXFULL));
}

/*
 *  受信した文字の取り出し
 */
Inline uint8_t
imxuart_getchar(SIOPCB *p_siopcb)
{
	return(sil_rew_mem((void*)(p_siopcb->p_siopinib->reg_base + UART_URXD)));
}

/*
 *  送信する文字の書き込み
 */
Inline void
imxuart_putchar(SIOPCB *p_siopcb, uint8_t c)
{
	sil_wrw_mem((void*)(p_siopcb->p_siopinib->reg_base + UART_UTXD), c);
}

/*
 *  送信割込み許可
 */
Inline void
imxuart_enable_send(SIOPCB *p_siopcb)
{
	sil_wrw_mem((void*)(p_siopcb->p_siopinib->reg_base + UART_UCR1),
				sil_rew_mem((void*)(p_siopcb->p_siopinib->reg_base + UART_UCR1))|UART_UCR1_TXMPTYEN);
}

/*
 *  送信割込み禁止
 */
Inline void
imxuart_disable_send(SIOPCB *p_siopcb)
{
	sil_wrw_mem((void*)(p_siopcb->p_siopinib->reg_base + UART_UCR1),
				sil_rew_mem((void*)(p_siopcb->p_siopinib->reg_base + UART_UCR1))&~UART_UCR1_TXMPTYEN);
}

/*
 *  受信割込み許可
 */
Inline void
imxuart_enable_rcv(SIOPCB *p_siopcb)
{
	sil_wrw_mem((void*)(p_siopcb->p_siopinib->reg_base + UART_UCR1),
				sil_rew_mem((void*)(p_siopcb->p_siopinib->reg_base + UART_UCR1))|UART_UCR1_RRDYEN);
}

/*
 *  受信割込み禁止
 */
Inline void
imxuart_disable_rcv(SIOPCB *p_siopcb)
{
	sil_wrw_mem((void*)(p_siopcb->p_siopinib->reg_base + UART_UCR1),
				sil_rew_mem((void*)(p_siopcb->p_siopinib->reg_base + UART_UCR1))&~UART_UCR1_RRDYEN);
}

/*
 *  シリアルI/Oポートへのポーリングでの出力
 */
void
sio_pol_putc(char c, ID siopid)
{
	const SIOPINIB *p_siopinib;

	p_siopinib = get_siopinib(siopid);

	while((sil_rew_mem((void*)(p_siopinib->reg_base + UART_UTS)) & UART_UTS_TXFULL));
	sil_wrw_mem((void*)(p_siopinib->reg_base + UART_UTXD), c);
}

/*
 *  SIOドライバの初期化ルーチン
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

    
/*
 * SIOPINIBの初期化
 */
static void
imxuart_init_siopinib(const SIOPINIB *p_siopinib)
{
	/* Wait until Tx FIFO empty */
	if (sil_rew_mem((void*)(p_siopinib->reg_base + UART_UCR1)) & UART_UCR1_UARTEN) {
		while(!(sil_rew_mem((void*)(p_siopinib->reg_base + UART_USR2)) & UART_USR2_TXFE));
	}

	/* Disable */
	sil_wrw_mem((void*)(p_siopinib->reg_base + UART_UCR1), 0x00);

	/* Disable tranmitter/receiver, Software Reset */
	sil_wrw_mem((void*)(p_siopinib->reg_base + UART_UCR2),0x00);

	while((sil_rew_mem((void*)(p_siopinib->reg_base + UART_UCR2)) & UART_UCR2_SRST) != UART_UCR2_SRST);

	/* Set RXD Muxed Input Selected */
	sil_wrw_mem((void*)(p_siopinib->reg_base + UART_UCR3), 0x0784);

	/* CTS Trigger Level */
	sil_wrw_mem((void*)(p_siopinib->reg_base + UART_UCR4), 0x8000);

	/* Set Escape Character */
	sil_wrw_mem((void*)(p_siopinib->reg_base + UART_UESC), 0x002b);
	/* Set Escape Timer */
	sil_wrw_mem((void*)(p_siopinib->reg_base + UART_UTIM), 0x0000);

	/* Receive FIFO interrupt trigger level */
	sil_wrw_mem((void*)(p_siopinib->reg_base + UART_UFCR),
				(sil_rew_mem((void*)(p_siopinib->reg_base + UART_UFCR)) & ~0x3f) |0x01);

	/* Transmit FIFO interrupt trigger level */
	sil_wrw_mem((void*)(p_siopinib->reg_base + UART_UFCR),
				(sil_rew_mem((void*)(p_siopinib->reg_base + UART_UFCR)) & ~0xfc00) | (0x02<<10));

	/*
	 * Baud rate関連(u-bootでのPLLの設定が変化すると追従する必要あり)
	 */
	/* Rererence Frequency Divider by 1*/
	sil_wrw_mem((void*)(p_siopinib->reg_base + UART_UFCR),
				(sil_rew_mem((void*)(p_siopinib->reg_base + UART_UFCR)) & ~0x0380) | (0x04<<7));
	/* Set Baud rate */
	sil_wrw_mem((void*)(p_siopinib->reg_base + UART_UBIR), UART_UBIR_VAL);
	sil_wrw_mem((void*)(p_siopinib->reg_base + UART_UBMR), UART_UBMR_VAL);

	/* 8bit, 1stop, Noprity, Ignore RTS, Enable tranmitter/receiver */
	sil_wrw_mem((void*)(p_siopinib->reg_base + UART_UCR2), UART_UCR2_IRTS|UART_UCR2_WS|UART_UCR2_TXEN|UART_UCR2_RXEN|UART_UCR2_SRST);
	
	/* Enable */
	sil_wrw_mem((void*)(p_siopinib->reg_base + UART_UCR1), UART_UCR1_UARTEN);
}


/*
 *  起動時のローレベル出力用の初期化
 */
void
sio_low_init(ID siopid)
{
	const SIOPINIB *p_siopinib;

	p_siopinib = get_siopinib(siopid);
	imxuart_init_siopinib(p_siopinib);
}

/*
 * シリアルI/Oポートのオープン
 */
SIOPCB *
sio_opn_por(ID siopid, intptr_t exinf)
{
	SIOPCB  *p_siopcb;
	bool_t  opnflg;
	ER      ercd;

	p_siopcb = get_siopcb(siopid);

	/*
	 *  オープンしたポートがあるかをopnflgに読んでおく．
	 */
	opnflg = p_siopcb->openflag;
    
	/*
	 * 初期化
	 */
	imxuart_init_siopinib(p_siopcb->p_siopinib);
    
	/* 受信割込み禁止 */
	imxuart_enable_rcv(p_siopcb);
	/* 送信割込み禁止 */
	imxuart_disable_rcv(p_siopcb);
    
	p_siopcb->exinf = exinf;
	p_siopcb->getready = p_siopcb->putready = false;
	p_siopcb->openflag = true;

	/*
	 *  シリアルI/O割込みのマスクを解除する．
	 */
	if (!opnflg) {
		ercd = ena_int(p_siopcb->p_siopinib->intno);
		assert(ercd == E_OK);
	}

	return(p_siopcb);
}

/*
 *  シリアルI/Oポートのクローズ
 */
void
sio_cls_por(SIOPCB *p_siopcb)
{
	/* 割込み禁止 */
	imxuart_disable_send(p_siopcb);
	imxuart_disable_rcv(p_siopcb);
	p_siopcb->openflag = false;

	/*
	 *  シリアルI/O割込みをマスクする．
	 */
	if (!(p_siopcb->openflag)) {
		dis_int(p_siopcb->p_siopinib->intno);
	}
}

/*
 *  シリアルI/Oポートへの文字送信
 */
bool_t
sio_snd_chr(SIOPCB *p_siopcb, char c)
{
	if (imxuart_putready(p_siopcb)){
		imxuart_putchar(p_siopcb, c);
		return(true);
	}
	return(false);
}

/*
 *  シリアルI/Oポートからの文字受信
 */
int
sio_rcv_chr(SIOPCB *p_siopcb)
{
	if (imxuart_getready(p_siopcb)) {
		return((int)(uint8_t)imxuart_getchar(p_siopcb));
	}
	return(-1);
}

/*
 *  シリアルI/Oポートからのコールバックの許可
 */
void
sio_ena_cbr(SIOPCB *p_siopcb, uint_t cbrtn)
{
	switch (cbrtn) {
	  case SIO_RDY_SND:
		imxuart_enable_send(p_siopcb);
		break;
	  case SIO_RDY_RCV:
		imxuart_enable_rcv(p_siopcb);
		break;
	}
}

/*
 *  シリアルI/Oポートからのコールバックの禁止
 */
void
sio_dis_cbr(SIOPCB *p_siopcb, uint_t cbrtn)
{
	switch (cbrtn) {
	  case SIO_RDY_SND:
		imxuart_disable_send(p_siopcb);
		break;
	  case SIO_RDY_RCV:
		imxuart_disable_rcv(p_siopcb);
		break;
	}
}

/*
 *  シリアルI/Oポートに対する割込み処理
 */
static void
imxuart_isr_siop(SIOPCB *p_siopcb)
{
	/* 割込みのクリア */
	sil_wrw_mem((void*)(p_siopcb->p_siopinib->reg_base + UART_USR1),
				sil_rew_mem((void*)(p_siopcb->p_siopinib->reg_base + UART_USR1)));
	sil_wrw_mem((void*)(p_siopcb->p_siopinib->reg_base + UART_USR2),
				sil_rew_mem((void*)(p_siopcb->p_siopinib->reg_base + UART_USR2)));

	if (imxuart_getready(p_siopcb)) {
		/*
		 *  受信通知コールバックルーチンを呼び出す．
		 */
		sio_irdy_rcv(p_siopcb->exinf);
	}else if (imxuart_putready(p_siopcb)) {
		/*
		 *  送信可能コールバックルーチンを呼び出す．
		 */
		sio_irdy_snd(p_siopcb->exinf);
	}
}


/*
 *  SIOの割込みサービスルーチン
 */
void
sio_isr(intptr_t exinf)
{
	imxuart_isr_siop(get_siopcb(exinf));
}
