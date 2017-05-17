/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 *
 *  Copyright (C) 2007-2016 by Embedded and Real-Time Systems Laboratory
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2001-2011 by Industrial Technology Institute,
 *                              Miyagi Prefectural Government, JAPAN
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
 *  @(#) $Id: scif.h 2758 2016-03-10 15:15:26Z ertl-honda $
 */


#ifndef TOPPERS_SCIF_H
#define TOPPERS_SCIF_H

#ifndef TOPPERS_MACRO_ONLY

#include <kernel.h>

/*
 *  シリアルI/Oポート管理ブロックの定義
 */
typedef struct sio_port_control_block	SIOPCB;

/*
 *  コールバックルーチンの識別番号
 */
#define SIO_RDY_SND    1U        /* 送信可能コールバック */
#define SIO_RDY_RCV    2U        /* 受信通知コールバック */

/*
 *  SIOドライバの初期化ルーチン
 */
extern void scif_initialize(void);

/*
 * カーネル起動時のバーナ出力用の初期化
 */
extern void scif_init(ID siopid);

/*
 *  シリアルI/Oポートへのポーリングでの出力
 */
extern void scif_pol_putc(char c, ID siopid);

/*
 *  オープンしているポートがあるか？
 */
extern bool_t scif_openflag_id(ID siopid);			/*  ポートID  */
extern bool_t scif_openflag_cb(SIOPCB *p_siopcb);	/*  管理ブロックの先頭番地  */

/*
 *  ポートIDの取得
 */
extern ID scif_get_siopid(SIOPCB *p_siopcb);

/*
 *  シリアルI/Oポートのオープン
 */
extern SIOPCB *scif_opn_por(ID siopid, intptr_t exinf);

/*
 *  シリアルI/Oポートのクローズ
 */
extern void scif_cls_por(SIOPCB *p_siopcb);

/*
 *  シリアルI/Oポートへの文字送信
 */
extern bool_t scif_snd_chr(SIOPCB *p_siopcb, char c);

/*
 *  シリアルI/Oポートからの文字受信
 */
extern int_t scif_rcv_chr(SIOPCB *p_siopcb);

/*
 *  シリアルI/Oポートからのコールバックの許可
 */
extern void  scif_ena_cbr(SIOPCB *p_siopcb, uint_t cbrtn);

/*
 *  シリアルI/Oポートからのコールバックの禁止
 */
extern void scif_dis_cbr(SIOPCB *p_siopcb, uint_t cbrtn);

/*
 *  SIOの割込みサービスルーチン
 */
extern void scif_tx_isr(ID siopid);
extern void scif_rx_isr(ID siopid);

/*
 *  シリアルI/Oポートからの送信可能コールバック
 */
extern void scif_irdy_snd(intptr_t exinf);

/*
 *  シリアルI/Oポートからの受信通知コールバック
 */
extern void scif_irdy_rcv(intptr_t exinf);

/*
 *  ポート番号から管理ブロックの先頭番地への変換
 */
extern SIOPCB *scif_get_siopcb(ID siopid);

#endif /* TOPPERS_MACRO_ONLY */
#endif /* TOPPERS_SCIF_H */
