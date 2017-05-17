/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2004-2011 by Embedded and Real-Time Systems Laboratory
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
 *  @(#) $Id: serial.h 2728 2015-12-30 01:46:11Z ertl-honda $
 */

/*
 *		シリアルインタフェースドライバ
 */

#ifndef TOPPERS_SERIAL_H
#define TOPPERS_SERIAL_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  シリアルインタフェースドライバの用いるパケット
 */
typedef struct {
	uint_t		reacnt;			/* 受信バッファ中の文字数 */
	uint_t		wricnt;			/* 送信バッファ中の文字数 */
} T_SERIAL_RPOR;

/*
 *  シリアルインタフェースドライバの初期化ルーチン
 */
extern void		serial_initialize(intptr_t exinf) throw();

/*
 *  シリアルインタフェースドライバからの未送信文字の取出し
 */
extern bool_t	serial_get_chr(ID portid, char *p_c) throw();

/*
 *  シリアルインタフェースドライバのサービスコール
 */
extern ER		serial_opn_por(ID portid) throw();
extern ER		serial_cls_por(ID portid) throw();
extern ER_UINT	serial_rea_dat(ID portid, char *buf, uint_t len) throw();
extern ER_UINT	serial_wri_dat(ID portid, const char *buf, uint_t len) throw();
extern ER		serial_ctl_por(ID portid, uint_t ioctl) throw();
extern ER		serial_ref_por(ID portid, T_SERIAL_RPOR *pk_rpor) throw();

/*
 *  シリアルインタフェースドライバの動作制御用のための定数
 *
 *  以下の定数は，ビット毎に論理和をとって用いる．
 */
#define	IOCTL_NULL	0U			/* 指定なし */
#define	IOCTL_ECHO	0x0001U		/* 受信した文字をエコーバック */
#define	IOCTL_CRLF	0x0010U		/* LFを送信する前にCRを付加 */
#define	IOCTL_FCSND	0x0100U		/* 送信に対してフロー制御を行う */
#define	IOCTL_FCANY	0x0200U		/* どのような文字でも送信再開 */
#define	IOCTL_FCRCV	0x0400U		/* 受信に対してフロー制御を行う */

#ifdef __cplusplus
}
#endif

#endif /* TOPPERS_SERIAL_H */
