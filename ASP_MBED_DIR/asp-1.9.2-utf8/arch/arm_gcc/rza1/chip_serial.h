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
 *  @(#) $Id: chip_serial.h 2758 2016-03-10 15:15:26Z ertl-honda $
 */

/*
 *  シリアルI/Oデバイス（SIO）ドライバ（RZ/A1用）
 */

#ifndef TOPPERS_CHIP_SERIAL_H
#define TOPPERS_CHIP_SERIAL_H

#include "scif.h"

/*
 *  SIOの割込みハンドラのベクタ番号
 */
#define INTNO_SCIF_BRI_1        TOPPERS_INTID_SCIF_BRI_1    /* 割込みハンドラ番号(BRI) */
#define INTNO_SCIF_ERI_1        TOPPERS_INTID_SCIF_ERI_1    /* 割込みハンドラ番号(ERI) */
#define INTNO_SCIF_RXI_1        TOPPERS_INTID_SCIF_RXI_1    /* 割込みハンドラ番号(RXI) */
#define INTNO_SCIF_TXI_1        TOPPERS_INTID_SCIF_TXI_1    /* 割込みハンドラ番号(TXI) */
#ifndef INTPRI_SIO_1
#define INTPRI_SIO_1            (-4)      /* 割込み優先度 */
#endif  /* INTPRI_SIO_1 */
#define INTATR_SIO_1            0U        /* 割込み属性 */

#define INTNO_SCIF_BRI_2        TOPPERS_INTID_SCIF_BRI_2    /* 割込みハンドラ番号(BRI) */
#define INTNO_SCIF_ERI_2        TOPPERS_INTID_SCIF_ERI_2    /* 割込みハンドラ番号(ERI) */
#define INTNO_SCIF_RXI_2        TOPPERS_INTID_SCIF_RXI_2    /* 割込みハンドラ番号(RXI) */
#define INTNO_SCIF_TXI_2        TOPPERS_INTID_SCIF_TXI_2    /* 割込みハンドラ番号(TXI) */
#ifndef INTPRI_SIO_2
#define INTPRI_SIO_2            (-4)      /* 割込み優先度 */
#endif  /* INTPRI_SIO_2 */
#define INTATR_SIO_2            0U        /* 割込み属性 */

#define INTNO_SCIF_BRI_3        TOPPERS_INTID_SCIF_BRI_3    /* 割込みハンドラ番号(BRI) */
#define INTNO_SCIF_ERI_3        TOPPERS_INTID_SCIF_ERI_3    /* 割込みハンドラ番号(ERI) */
#define INTNO_SCIF_RXI_3        TOPPERS_INTID_SCIF_RXI_3    /* 割込みハンドラ番号(RXI) */
#define INTNO_SCIF_TXI_3        TOPPERS_INTID_SCIF_TXI_3    /* 割込みハンドラ番号(TXI) */
#ifndef INTPRI_SIO_3
#define INTPRI_SIO_3            (-4)      /* 割込み優先度 */
#endif  /* INTPRI_SIO_3 */
#define INTATR_SIO_3            0U        /* 割込み属性 */

#define INTNO_SCIF_BRI_4        TOPPERS_INTID_SCIF_BRI_4    /* 割込みハンドラ番号(BRI) */
#define INTNO_SCIF_ERI_4        TOPPERS_INTID_SCIF_ERI_4    /* 割込みハンドラ番号(ERI) */
#define INTNO_SCIF_RXI_4        TOPPERS_INTID_SCIF_RXI_4    /* 割込みハンドラ番号(RXI) */
#define INTNO_SCIF_TXI_4        TOPPERS_INTID_SCIF_TXI_4    /* 割込みハンドラ番号(TXI) */
#ifndef INTPRI_SIO_4
#define INTPRI_SIO_4            (-4)      /* 割込み優先度 */
#endif  /* INTPRI_SIO_4 */
#define INTATR_SIO_4            0U        /* 割込み属性 */

#define INTNO_SCIF_BRI_5        TOPPERS_INTID_SCIF_BRI_5    /* 割込みハンドラ番号(BRI) */
#define INTNO_SCIF_ERI_5        TOPPERS_INTID_SCIF_ERI_5    /* 割込みハンドラ番号(ERI) */
#define INTNO_SCIF_RXI_5        TOPPERS_INTID_SCIF_RXI_5    /* 割込みハンドラ番号(RXI) */
#define INTNO_SCIF_TXI_5        TOPPERS_INTID_SCIF_TXI_5    /* 割込みハンドラ番号(TXI) */
#ifndef INTPRI_SIO_5
#define INTPRI_SIO_5            (-4)      /* 割込み優先度 */
#endif  /* INTPRI_SIO_5 */
#define INTATR_SIO_5            0U        /* 割込み属性 */

#define INTNO_SCIF_BRI_6        TOPPERS_INTID_SCIF_BRI_6    /* 割込みハンドラ番号(BRI) */
#define INTNO_SCIF_ERI_6        TOPPERS_INTID_SCIF_ERI_6    /* 割込みハンドラ番号(ERI) */
#define INTNO_SCIF_RXI_6        TOPPERS_INTID_SCIF_RXI_6    /* 割込みハンドラ番号(RXI) */
#define INTNO_SCIF_TXI_6        TOPPERS_INTID_SCIF_TXI_6    /* 割込みハンドラ番号(TXI) */
#ifndef INTPRI_SIO_6
#define INTPRI_SIO_6            (-4)      /* 割込み優先度 */
#endif  /* INTPRI_SIO_6 */
#define INTATR_SIO_6            0U        /* 割込み属性 */

#define INTNO_SCIF_BRI_7        TOPPERS_INTID_SCIF_BRI_7    /* 割込みハンドラ番号(BRI) */
#define INTNO_SCIF_ERI_7        TOPPERS_INTID_SCIF_ERI_7    /* 割込みハンドラ番号(ERI) */
#define INTNO_SCIF_RXI_7        TOPPERS_INTID_SCIF_RXI_7    /* 割込みハンドラ番号(RXI) */
#define INTNO_SCIF_TXI_7        TOPPERS_INTID_SCIF_TXI_7    /* 割込みハンドラ番号(TXI) */
#ifndef INTPRI_SIO_7
#define INTPRI_SIO_7            (-4)      /* 割込み優先度 */
#endif  /* INTPRI_SIO_7 */
#define INTATR_SIO_7            0U        /* 割込み属性 */

#define INTNO_SCIF_BRI_8        TOPPERS_INTID_SCIF_BRI_8    /* 割込みハンドラ番号(BRI) */
#define INTNO_SCIF_ERI_8        TOPPERS_INTID_SCIF_ERI_8    /* 割込みハンドラ番号(ERI) */
#define INTNO_SCIF_RXI_8        TOPPERS_INTID_SCIF_RXI_8    /* 割込みハンドラ番号(RXI) */
#define INTNO_SCIF_TXI_8        TOPPERS_INTID_SCIF_TXI_8    /* 割込みハンドラ番号(TXI) */
#ifndef INTPRI_SIO_8
#define INTPRI_SIO_8            (-4)      /* 割込み優先度 */
#endif  /* INTPRI_SIO_8 */
#define INTATR_SIO_8            0U        /* 割込み属性 */

#ifndef TOPPERS_MACRO_ONLY

/*
 *  SIOドライバの初期化
 */
extern void sio_initialize(intptr_t exinf);

/*
 *  シリアルI/Oポートのオープン
 */
extern SIOPCB *sio_opn_por(ID siopid, intptr_t exinf);

/*
 *  シリアルI/Oポートのクローズ
 */
extern void sio_cls_por(SIOPCB *p_siopcb);

/*
 *  SIOの割込みハンドラ
 */
extern void sio_isr_rxi(intptr_t exinf);
extern void sio_isr_txi(intptr_t exinf);

/*
 *  シリアルI/Oポートへの文字送信
 */
extern bool_t sio_snd_chr(SIOPCB *siopcb, char c);

/*
 *  シリアルI/Oポートからの文字受信
 */
extern int_t sio_rcv_chr(SIOPCB *siopcb);

/*
 *  シリアルI/Oポートからのコールバックの許可
 */
extern void sio_ena_cbr(SIOPCB *siopcb, uint_t cbrtn);

/*
 *  シリアルI/Oポートからのコールバックの禁止
 */
extern void sio_dis_cbr(SIOPCB *siopcb, uint_t cbrtn);

/*
 *  シリアルI/Oポートからの送信可能コールバック
 */
extern void sio_irdy_snd(intptr_t exinf);

/*
 *  シリアルI/Oポートからの受信通知コールバック
 */
extern void sio_irdy_rcv(intptr_t exinf);

#endif /* TOPPERS_MACRO_ONLY */
#endif /* TOPPERS_CHIP_SERIAL_H */
