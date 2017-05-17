/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2005-2011 by Embedded and Real-Time Systems Laboratory
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
 *		シリアルI/Oデバイス（SIO）ドライバ（DVE68K/40用）
 */

#include <kernel.h>
#include <t_syslog.h>
#include "dve68k.h"
#include "pdic/upd72001/upd72001.h"
#include "target_serial.h"

/*
 *  SIOドライバの初期化
 */
void
sio_initialize(intptr_t exinf)
{
	upd72001_initialize();
}

/*
 *  シリアルI/Oポートのオープン
 */
SIOPCB *
sio_opn_por(ID siopid, intptr_t exinf)
{
	SIOPCB	*p_siopcb;
	bool_t	opnflg;
	ER		ercd;

	/*
	 *  オープンしたポートがあるかをopnflgに読んでおく．
	 */
	opnflg = upd72001_openflag();

	/*
	 *  デバイス依存のオープン処理．
	 */
	p_siopcb = upd72001_opn_por(siopid, exinf);

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
	ER		ercd;

	/*
	 *  デバイス依存のクローズ処理．
	 */
	upd72001_cls_por(p_siopcb);

	/*
	 *  シリアルI/O割込みをマスクする．
	 */
	if (!upd72001_openflag()) {
		ercd = dis_int(INTNO_SIO);
		assert(ercd == E_OK);
	}
}

/*
 *  SIOの割込みサービスルーチン
 */
void
sio_isr(intptr_t exinf)
{
	upd72001_isr();
}

/*
 *  シリアルI/Oポートへの文字送信
 */
bool_t
sio_snd_chr(SIOPCB *siopcb, char c)
{
	return(upd72001_snd_chr(siopcb, c));
}

/*
 *  シリアルI/Oポートからの文字受信
 */
int_t
sio_rcv_chr(SIOPCB *siopcb)
{
	return(upd72001_rcv_chr(siopcb));
}

/*
 *  シリアルI/Oポートからのコールバックの許可
 */
void
sio_ena_cbr(SIOPCB *siopcb, uint_t cbrtn)
{
	upd72001_ena_cbr(siopcb, cbrtn);
}

/*
 *  シリアルI/Oポートからのコールバックの禁止
 */
void
sio_dis_cbr(SIOPCB *siopcb, uint_t cbrtn)
{
	upd72001_dis_cbr(siopcb, cbrtn);
}

/*
 *  シリアルI/Oポートからの送信可能コールバック
 */
void
upd72001_irdy_snd(intptr_t exinf)
{
	sio_irdy_snd(exinf);
}

/*
 *  シリアルI/Oポートからの受信通知コールバック
 */
void
upd72001_irdy_rcv(intptr_t exinf)
{
	sio_irdy_rcv(exinf);
}
