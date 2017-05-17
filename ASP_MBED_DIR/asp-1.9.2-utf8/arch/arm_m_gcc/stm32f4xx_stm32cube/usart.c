/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2007,2011,2013,2015 by Embedded and Real-Time Systems Laboratory
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
 */

/*
 * シリアルドライバ（STM32F USART用）
 */

#include <kernel.h>
#include <sil.h>
#include "usart.h"
#include "target_syssvc.h"
#include "stm32f4xx_hal.h"

/*
 * レジスタ設定値
 */
#define PORT2SIOPID(x)	((x) + 1)
#define INDEX_PORT(x)	((x) - 1)
#define GET_SIOPCB(x)	(&siopcb_table[INDEX_PORT(x)])

/*
 * USARTレジスタ定義
 */
#define USART_SR(x)		(x)
#define USART_DR(x)		(x + 0x04)
#define USART_BRR(x)	(x + 0x08)
#define USART_CR1(x)	(x + 0x0C)
#define USART_CR2(x)	(x + 0x10)
#define USART_CR3(x)	(x + 0x14)
#define USART_GTPR(x)	(x + 0x18)

/*
 *  シリアルポートの管理ブロック
 */
struct sio_port_control_block {
	ID port;
	uint32_t reg;
	intptr_t exinf;
};

/*
 * シリアルI/Oポート管理ブロックエリア
 */
SIOPCB siopcb_table[TNUM_PORT];

static const uint32_t sioreg_table[TNUM_PORT] = {
	USART1_BASE,
#if (TNUM_PORT >= 2)
	USART2_BASE,
#endif
};

Inline bool_t
sio_putready(SIOPCB* siopcb)
{
	return (sil_rew_mem((void*)USART_SR(siopcb->reg)) & USART_SR_TXE) != 0;
}

Inline bool_t
sio_getready(SIOPCB* siopcb)
{
	return (sil_rew_mem((void*)USART_SR(siopcb->reg)) & USART_SR_RXNE) != 0;
}

/*
 *  ターゲットのシリアル初期化
 */
void
usart_init(ID siopid)
{
	uint32_t tmp, usartdiv, fraction;
	uint32_t reg = sioreg_table[INDEX_PORT(siopid)];
	uint32_t src_clock;

	/* USARTの無効化 */
	sil_andw((void*)USART_CR1(reg), ~USART_CR1_UE);

	/* 1STOP BIT */
	sil_wrw_mem((void*)USART_CR2(reg), 0);

	/* 1START BIT, 8DATA bits, Parityなし */
	sil_wrw_mem((void*)USART_CR1(reg), 0);

	/* CR3初期化 */
	sil_wrw_mem((void*)USART_CR3(reg), 0);

	/* 通信速度設定 */
	if (siopid == 1) {
		/* USART1のみPCLK2を使用する */
		src_clock = HAL_RCC_GetPCLK2Freq();
	} else {
		src_clock = HAL_RCC_GetPCLK1Freq();
	}
	tmp = (1000 * (src_clock / 100)) / ((BPS_SETTING / 100) * 16);
	usartdiv = (tmp / 1000) << 4;
	fraction = tmp - ((usartdiv >> 4) * 1000);
	fraction = ((16 * fraction) + 500) / 1000;
	usartdiv |= (fraction & 0x0F);
	sil_wrw_mem((void*)USART_BRR(reg), usartdiv);

	/* 送受信の有効化、エラー割込みの有効化 */
	sil_orw((void*)USART_CR1(reg), USART_CR1_RE | USART_CR1_TE);
	sil_orw((void*)USART_CR3(reg), USART_CR3_EIE);

	/* USARTの有効化 */
	sil_orw((void*)USART_CR1(reg), USART_CR1_UE);
}

/*
 *  ターゲットのシリアル終了
 */
static void
usart_term(ID siopid)
{
	uint32_t reg = sioreg_table[INDEX_PORT(siopid)];

	/* USARTの無効化 */
	sil_andw((void*)USART_CR1(reg),  ~USART_CR1_UE);
}

/*
 *  SIO初期化
 */
void
sio_initialize(intptr_t exinf)
{
	int i;

	for (i = 0; i < TNUM_PORT; i++) {
		siopcb_table[i].port = i;
		siopcb_table[i].reg = sioreg_table[i];
		siopcb_table[i].exinf = 0;
	}
}

/*
 *  シリアルオープン
 */
SIOPCB
*sio_opn_por(ID siopid, intptr_t exinf)
{
	SIOPCB* siopcb;

	if (siopid > TNUM_PORT) {
		return NULL;
	}

	siopcb = GET_SIOPCB(siopid);
	siopcb->exinf = exinf;

	usart_init(siopid);

	return siopcb;
}

/*
 *  シリアルクローズ
 */
void
sio_cls_por(SIOPCB *p_siopcb)
{
	usart_term(PORT2SIOPID(p_siopcb->port));
}

/*
 *  割込みサービスルーチン
 */
void
sio_isr(intptr_t exinf)
{
	SIOPCB* siopcb = GET_SIOPCB(exinf);

	if (sio_putready(siopcb)) {
		sio_irdy_snd(siopcb->exinf);
	}
	if (sio_getready(siopcb)) {
		sio_irdy_rcv(siopcb->exinf);
	}
}

/*
 *  1文字送信
 */
bool_t
sio_snd_chr(SIOPCB *siopcb, char c)
{
	if (sio_putready(siopcb)) {
		sil_wrw_mem((void*)USART_DR(siopcb->reg), c);

		return true;
	}

	return false;
}

/*
 *  1文字受信
 */
int_t
sio_rcv_chr(SIOPCB *siopcb)
{
	int_t c = -1;

	if (sio_getready(siopcb)) {
		c = sil_rew_mem((void*)USART_DR(siopcb->reg)) & 0xFF;
	}

	return c;
}

/*
 *  コールバックの許可
 */
void
sio_ena_cbr(SIOPCB *siopcb, uint_t cbrtn)
{
	switch (cbrtn) {
	case SIO_RDY_SND:
		sil_orw((void*)USART_CR1(siopcb->reg), USART_CR1_TXEIE);
		break;
	case SIO_RDY_RCV:
		sil_orw((void*)USART_CR1(siopcb->reg), USART_CR1_RXNEIE);
		break;
	default:
		break;
	}
}

/* 
 *  コールバックの禁止
 */
void
sio_dis_cbr(SIOPCB *siopcb, uint_t cbrtn)
{
	switch (cbrtn) {
	case SIO_RDY_SND:
		sil_andw((void*)USART_CR1(siopcb->reg), ~USART_CR1_TXEIE);
		break;
	case SIO_RDY_RCV:
		sil_andw((void*)USART_CR1(siopcb->reg), ~USART_CR1_RXNEIE);
		break;
	default:
		break;
	}
}

/*
 *  1文字出力（ポーリングでの出力）
 */
void
sio_pol_snd_chr(char c, ID siopid)
{
	uint32_t reg = sioreg_table[INDEX_PORT(siopid)];

	sil_wrw_mem((void*)USART_DR(reg), c);

	while ((sil_rew_mem((void*)USART_SR(reg)) & USART_SR_TXE) == 0) ;
}
