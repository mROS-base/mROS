/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 *
 *  Copyright (C) 2015 by Embedded and Real-Time Systems Laboratory
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
 *  @(#) $Id: target_serial.c 2728 2015-12-30 01:46:11Z ertl-honda $
 */

/*
 *  シリアルI/Oデバイス（SIO）ドライバ（ARDUINO_M0用）
 */
#include <kernel.h>
#include <t_syslog.h>
#include "arduino_m0.h"
#include "target_serial.h"
#include "variant.h"

/*
 *  シリアルI/Oポート初期化ブロックの定義
 */
typedef struct sio_port_initialization_block 
{
	Sercom*		p_sercom;		/* 使用するSERCOMへのポインタ */
	uint8_t		rx_pin;			/* RX用PIN */
	uint8_t		tx_pin;			/* TX用PIN */
	uint8_t		sample_rate;	/* サンプリングレート */
	uint8_t		port;			/* ポート */
	int8_t		pio;			/* PIO */
	uint8_t		gcm;			/* GCM */
	uint32_t	bps;			/* ボーレート */
	uint32_t	pm;				/* PowerManagement */
}
SIOPINIB;

/*
 *  シリアルI/Oポート管理ブロックの定義
 */
struct sio_port_control_block 
{
	const SIOPINIB*	p_siopinib;	/* 初期化ブロック */
	intptr_t		exinf;		/* 拡張情報 */
	bool_t			openflag;	/* オープン済みフラグ */
	bool_t			devinitd;	/* デバイス初期化済みフラグ */
	bool_t			sendflag;	/* 送信割込みイネーブルフラグ */
	bool_t			getready;	/* 文字を受信した状態 */
	bool_t			putready;	/* 文字を送信できる状態 */
};

/*
 * シリアルI/Oポート初期化ブロック
 */
const SIOPINIB siopinib_table[TNUM_SIOP] = {
	{SERCOM5, 23u, 22u, 16u, PORTB, PIO_SERCOM_ALT, GCM_SERCOM5_CORE, 115200, PM_APBCMASK_SERCOM5}
};

/*
 *  シリアルI/Oポート管理ブロックのエリア
 */
SIOPCB siopcb_table[TNUM_SIOP];

/*
 *  シリアルI/OポートIDから管理ブロックを取り出すためのマクロ
 */
#define INDEX_SIOP(siopid)  ((uint_t)((siopid) - 1))
#define get_siopcb(siopid)  (&(siopcb_table[INDEX_SIOP(siopid)]))


static void
sercom_uart_devinit(SIOPCB *p_siopcb){
	Sercom*	p_sercom = p_siopcb->p_siopinib->p_sercom;

	/* デバイスが初期化済みならリターン */
	if (p_siopcb->devinitd) {
	  return;
	}

	p_siopcb->devinitd = true;

	/* RX Pin Initialize */
	PORT->Group[p_siopcb->p_siopinib->port].PMUX[(p_siopcb->p_siopinib->rx_pin) >> 1].reg =
	  ((PORT->Group[p_siopcb->p_siopinib->port].PMUX[(p_siopcb->p_siopinib->rx_pin) >> 1].reg) & PORT_PMUX_PMUXE(0xF))
		|PORT_PMUX_PMUXO(p_siopcb->p_siopinib->pio);
	PORT->Group[p_siopcb->p_siopinib->port].PINCFG[(p_siopcb->p_siopinib->rx_pin)].reg |= PORT_PINCFG_PMUXEN;

	/* TX Pin Initialize */
	PORT->Group[p_siopcb->p_siopinib->port].PMUX[(p_siopcb->p_siopinib->tx_pin) >> 1].reg =
	  ((PORT->Group[p_siopcb->p_siopinib->port].PMUX[(p_siopcb->p_siopinib->tx_pin) >> 1].reg) & PORT_PMUX_PMUXO(0xF))
		|PORT_PMUX_PMUXE(p_siopcb->p_siopinib->pio);
	PORT->Group[p_siopcb->p_siopinib->port].PINCFG[(p_siopcb->p_siopinib->tx_pin)].reg |= PORT_PINCFG_PMUXEN;

	/* 電源の投入 */
	PM->APBCMASK.reg |= p_siopcb->p_siopinib->pm;

	/* Reset Uart */
	p_sercom->USART.CTRLA.bit.SWRST = 1 ;
	while(p_sercom->USART.CTRLA.bit.SWRST || p_sercom->USART.SYNCBUSY.bit.SWRST){};

	/* Init Clock */
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(p_siopcb->p_siopinib->gcm)
	  | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_CLKEN ;
	while(GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY) {}

	/* Set Baund Rate */
	p_sercom->USART.CTRLA.reg = SERCOM_USART_CTRLA_MODE(0x01) | SERCOM_USART_CTRLA_SAMPR(p_siopcb->p_siopinib->sample_rate);
	p_sercom->USART.BAUD.reg = 0xffff - ((0xffff * (((int)p_siopcb->p_siopinib->sample_rate * (int)p_siopcb->p_siopinib->bps)/(CORE_CLOCK_HZ/1000000))) / 1000000u);

	/* Disable All Interrupt */
	p_sercom->USART.INTENSET.reg = 0;

	/* Set Frame : 8bit, 1Stop, No parity, Lsb first*/
	p_sercom->USART.CTRLA.reg |= SERCOM_USART_CTRLA_FORM(0) |
								(1 << SERCOM_USART_CTRLA_DORD_Pos);
	p_sercom->USART.CTRLB.reg |= SERCOM_USART_CTRLB_CHSIZE(0) |
								(0 << SERCOM_USART_CTRLB_SBMODE_Pos) |
								(0 << SERCOM_USART_CTRLB_PMODE_Pos);

	/* Set Pad */
	p_sercom->USART.CTRLA.reg |= SERCOM_USART_CTRLA_TXPO(0x01) | SERCOM_USART_CTRLA_RXPO(0x03);

	/* Tx/Rx Enable */
	p_sercom->USART.CTRLB.reg |= SERCOM_USART_CTRLB_TXEN | SERCOM_USART_CTRLB_RXEN;

	/* Enable Uart */
	p_sercom->USART.CTRLA.bit.ENABLE = 0x01;
	while(p_sercom->USART.SYNCBUSY.bit.ENABLE){};
}

/*
 * カーネル起動時のログ出力用の初期化
 */
void
arduino_m0_init_uart(uint32_t siopid)
{
	SIOPCB  *p_siopcb;

	p_siopcb = get_siopcb(siopid);
	p_siopcb->p_siopinib = &siopinib_table[INDEX_SIOP(siopid)];
	sercom_uart_devinit(p_siopcb);
}

/*
 *  UARTからのポーリング出力
 */
void
arduino_m0_putc(uint32_t siopid, char c)
{
	Sercom* p_sercom = (get_siopcb(siopid))->p_siopinib->p_sercom;
	while(p_sercom->USART.INTFLAG.bit.DRE != SERCOM_USART_INTFLAG_DRE);
	p_sercom->USART.DATA.reg = (uint16_t)c;
}


/*
 * 文字を受信したか?
 */ 
Inline bool_t
uart_getready(SIOPCB *p_siopcb)
{
	Sercom*	p_sercom = p_siopcb->p_siopinib->p_sercom;
	return(p_sercom->USART.INTFLAG.bit.RXC);
}

/*
 * 文字を送信できるか?
 */
Inline bool_t
uart_putready(SIOPCB *p_siopcb)
{
	Sercom*	p_sercom = p_siopcb->p_siopinib->p_sercom;
	return(p_sercom->USART.INTFLAG.bit.DRE == SERCOM_USART_INTFLAG_DRE);
}

/*
 *  受信した文字の取り出し
 */
Inline uint8_t
uart_getchar(SIOPCB *p_siopcb)
{
	Sercom*	p_sercom = p_siopcb->p_siopinib->p_sercom;
	return p_sercom->USART.DATA.bit.DATA;
}

/*
 *  送信する文字の書き込み
 */
Inline void
uart_putchar(SIOPCB *p_siopcb, uint8_t c)
{
	Sercom*	p_sercom = p_siopcb->p_siopinib->p_sercom;
	p_sercom->USART.DATA.reg = (uint16_t)c;
}

/*
 *  送信割込み許可
 */
Inline void
uart_enable_send(SIOPCB *p_siopcb)
{
	Sercom*	p_sercom = p_siopcb->p_siopinib->p_sercom;
	p_sercom->USART.INTENSET.reg = SERCOM_USART_INTENSET_TXC;
}

/*
 *  送信割込み禁止
 */
Inline void
uart_disable_send(SIOPCB *p_siopcb)
{
	Sercom*	p_sercom = p_siopcb->p_siopinib->p_sercom;
	p_sercom->USART.INTENCLR.reg = SERCOM_USART_INTENCLR_TXC;
}


/*
 *  受信割込み許可
 */
Inline void
uart_enable_rcv(SIOPCB *p_siopcb)
{
	Sercom*	p_sercom = p_siopcb->p_siopinib->p_sercom;
	p_sercom->USART.INTENSET.reg = SERCOM_USART_INTENSET_RXC;
}

/*
 *  受信割込み禁止
 */
Inline void
uart_disable_rcv(SIOPCB *p_siopcb)
{
	Sercom*	p_sercom = p_siopcb->p_siopinib->p_sercom;
	p_sercom->USART.INTENCLR.reg = SERCOM_USART_INTENCLR_RXC;
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

/*
 * シリアルI/Oポートのオープン
 */
SIOPCB *
arduino_m0_uart_opn_por(SIOPCB *p_siopcb, intptr_t exinf)
{
	p_siopcb->exinf = exinf;
	p_siopcb->getready = p_siopcb->putready = false;
	p_siopcb->openflag = true;

	sercom_uart_devinit(p_siopcb);

	return(p_siopcb);
}


/*
 *  シリアルI/Oポートのオープン
 */
SIOPCB *
sio_opn_por(ID siopid, intptr_t exinf)
{
	SIOPCB  *p_siopcb = get_siopcb(siopid);
	bool_t    opnflg;
	ER      ercd;

	/*
	 *  オープンしたポートがあるかをopnflgに読んでおく．
	 */
	opnflg = p_siopcb->openflag;

	/*
	 *  デバイス依存のオープン処理．
	 */
	arduino_m0_uart_opn_por(p_siopcb, exinf);

	/*
	 *  シリアルI/O割込みのマスクを解除する．
	 */
	if (!opnflg) {
		ercd = ena_int(INTNO_SIO);
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
	/*
	 *  デバイス依存のクローズ処理．
	 */
	p_siopcb->openflag = false;
    
	/*
	 *  シリアルI/O割込みをマスクする．
	 */
	dis_int(INTNO_SIO);
}

/*
 *  SIOの割込みハンドラ
 */
void
sio_isr(intptr_t exinf)
{
	SIOPCB *p_siopcb = get_siopcb(exinf);

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


/*
 *  シリアルI/Oポートへの文字送信
 */
bool_t
sio_snd_chr(SIOPCB *siopcb, char c)
{	
	if (uart_putready(siopcb)){
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
	return(-1);
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

#define UART_SERCOM   SERCOM5

void
serial_ena(void){
	UART_SERCOM->USART.INTENSET.reg |= SERCOM_USART_INTENSET_RXC;
}
	
void
serial_putc(char c){
	while(UART_SERCOM->USART.INTFLAG.bit.DRE != SERCOM_USART_INTFLAG_DRE);
	UART_SERCOM->USART.DATA.reg = (uint16_t)c;
}

char
serial_getc(void){
	while(!UART_SERCOM->USART.INTFLAG.bit.RXC);
	return UART_SERCOM->USART.DATA.bit.DATA;
}
