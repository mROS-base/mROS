/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 *
 *  Copyright (C) 2007-2016 by Embedded and Real-Time Systems Laboratory
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
 *  @(#) $Id: chip_serial.c 2758 2016-03-10 15:15:26Z ertl-honda $
 */

/*
 *  シリアルI/Oデバイス（SCIF）ドライバ（RZ/A1用）
 */

#include <kernel.h>
#include <t_syslog.h>
#include "chip_serial.h"

static INTNO const intno_sio_bri[TNUM_SIOP] = {
	INTNO_SCIF_BRI_1,
	INTNO_SCIF_BRI_2,
	INTNO_SCIF_BRI_3,
	INTNO_SCIF_BRI_4,
	INTNO_SCIF_BRI_5,
#if TNUM_SIOP > 5
	INTNO_SCIF_BRI_6,
	INTNO_SCIF_BRI_7,
	INTNO_SCIF_BRI_8,
#endif /* TNUM_SIOP > 5 */
};

static INTNO const intno_sio_eri[TNUM_SIOP] = {
	INTNO_SCIF_ERI_1,
	INTNO_SCIF_ERI_2,
	INTNO_SCIF_ERI_3,
	INTNO_SCIF_ERI_4,
	INTNO_SCIF_ERI_5,
#if TNUM_SIOP > 5
	INTNO_SCIF_ERI_6,
	INTNO_SCIF_ERI_7,
	INTNO_SCIF_ERI_8,
#endif /* TNUM_SIOP > 5 */
};

static INTNO const intno_sio_rxi[TNUM_SIOP] = {
	INTNO_SCIF_RXI_1,
	INTNO_SCIF_RXI_2,
	INTNO_SCIF_RXI_3,
	INTNO_SCIF_RXI_4,
	INTNO_SCIF_RXI_5,
#if TNUM_SIOP > 5
	INTNO_SCIF_RXI_6,
	INTNO_SCIF_RXI_7,
	INTNO_SCIF_RXI_8,
#endif /* TNUM_SIOP > 5 */
};

static INTNO const intno_sio_txi[TNUM_SIOP] = {
	INTNO_SCIF_TXI_1,
	INTNO_SCIF_TXI_2,
	INTNO_SCIF_TXI_3,
	INTNO_SCIF_TXI_4,
#if TNUM_SIOP > 5
	INTNO_SCIF_TXI_5,
	INTNO_SCIF_TXI_6,
	INTNO_SCIF_TXI_7,
	INTNO_SCIF_TXI_8,
#endif /* TNUM_SIOP > 5 */
};

/*
 *  シリアルI/OポートIDから割込み番号を取り出すためのマクロ
 */
#define INDEX_SIOP(siopid)           ((uint_t)((siopid) - 1))
#define get_intno_sio_bri(siopid)    (intno_sio_bri[INDEX_SIOP(siopid)])
#define get_intno_sio_eri(siopid)    (intno_sio_eri[INDEX_SIOP(siopid)])
#define get_intno_sio_rxi(siopid)    (intno_sio_rxi[INDEX_SIOP(siopid)])
#define get_intno_sio_txi(siopid)    (intno_sio_txi[INDEX_SIOP(siopid)])

/*
 *  SIOドライバの初期化
 */
void
sio_initialize(intptr_t exinf)
{
	scif_initialize();
}

/*
 *  シリアルI/Oポートのオープン
 */
SIOPCB *
sio_opn_por(ID siopid, intptr_t exinf)
{
	SIOPCB    *p_siopcb;
	ER        ercd;

	/*
	 *  シリアルI/O割込みをマスクする．
	 */
	ercd = dis_int(get_intno_sio_rxi(siopid));
	assert(ercd == E_OK);
	ercd = dis_int(get_intno_sio_txi(siopid));
	assert(ercd == E_OK);

	/*
	 *  デバイス依存のオープン処理．
	 */
	p_siopcb = scif_opn_por(siopid, exinf);

	/*
	 *  シリアルI/O割込みのマスクを解除する．
	 */
	ercd = ena_int(get_intno_sio_rxi(siopid));
	assert(ercd == E_OK);
	ercd = ena_int(get_intno_sio_txi(siopid));
	assert(ercd == E_OK);

	return(p_siopcb);
}

/*
 *  シリアルI/Oポートのクローズ
 */
void
sio_cls_por(SIOPCB *p_siopcb)
{
	ER ercd;
	ID siopid;

	/*
	 *  シリアルI/OポートIDの取得
	 */
	siopid = scif_get_siopid(p_siopcb);

	/*
	 *  デバイス依存のクローズ処理．
	 */
	scif_cls_por(p_siopcb);

	/*
	 *  シリアルI/O割込みをマスクする．
	 */
	ercd = dis_int(intno_sio_rxi[siopid]);
	assert(ercd == E_OK);
	ercd = dis_int(intno_sio_txi[siopid]);
	assert(ercd == E_OK);
}

/*
 *  SIOの割込みハンドラ
 */
void sio_isr_rxi(intptr_t exinf)
{
    scif_rx_isr(exinf);
}
void sio_isr_txi(intptr_t exinf)
{
    scif_tx_isr(exinf);
}

/*
 *  シリアルI/Oポートへの文字送信
 */
bool_t sio_snd_chr(SIOPCB *siopcb, char c)
{
	return(scif_snd_chr(siopcb, c));
}

/*
 *  シリアルI/Oポートからの文字受信
 */
int_t sio_rcv_chr(SIOPCB *siopcb)
{
	return(scif_rcv_chr(siopcb));
}

/*
 *  シリアルI/Oポートからのコールバックの許可
 */
void sio_ena_cbr(SIOPCB *siopcb, uint_t cbrtn)
{
	scif_ena_cbr(siopcb, cbrtn);
}

/*
 *  シリアルI/Oポートからのコールバックの禁止
 */
void sio_dis_cbr(SIOPCB *siopcb, uint_t cbrtn)
{
	scif_dis_cbr(siopcb, cbrtn);
}

/*
 *  シリアルI/Oポートからの送信可能コールバック
 */
void scif_irdy_snd(intptr_t exinf)
{
	sio_irdy_snd(exinf);
}

/*
 *  シリアルI/Oポートからの受信通知コールバック
 */
void scif_irdy_rcv(intptr_t exinf)
{
	sio_irdy_rcv(exinf);
}
