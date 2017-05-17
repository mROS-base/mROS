/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2005-2013 by Embedded and Real-Time Systems Laboratory
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
 *  @(#) $Id: dataqueue.h 2728 2015-12-30 01:46:11Z ertl-honda $
 */

/*
 *		データキュー機能
 */

#ifndef TOPPERS_DATAQUEUE_H
#define TOPPERS_DATAQUEUE_H

#include "wait.h"

/*
 *  データ管理ブロック
 */
typedef struct data_management_block {
	intptr_t	data;			/* データ本体 */
} DTQMB;

/*
 *  データキュー初期化ブロック
 *
 *  この構造体は，同期・通信オブジェクトの初期化ブロックの共通部分
 *  （WOBJINIB）を拡張（オブジェクト指向言語の継承に相当）したもので，
 *  最初のフィールドが共通になっている．
 */
typedef struct dataqueue_initialization_block {
	ATR			dtqatr;			/* データキュー属性 */
	uint_t		dtqcnt;			/* データキューの容量 */
	DTQMB		*p_dtqmb;		/* データキュー管理領域の先頭番地 */
} DTQINIB;

/*
 *  データキュー管理ブロック
 *
 *  この構造体は，同期・通信オブジェクトの管理ブロックの共通部分（WOBJCB）
 *  を拡張（オブジェクト指向言語の継承に相当）したもので，最初の2つの
 *  フィールドが共通になっている．
 */
typedef struct dataqueue_control_block {
	QUEUE		swait_queue;	/* データキュー送信待ちキュー */
	const DTQINIB *p_dtqinib;	/* 初期化ブロックへのポインタ */
	QUEUE		rwait_queue;	/* データキュー受信待ちキュー */
	uint_t		count;			/* データキュー中のデータの数 */
	uint_t		head;			/* 最初のデータの格納場所 */
	uint_t		tail;			/* 最後のデータの格納場所の次 */
} DTQCB;

/*
 *  データキュー待ち情報ブロックの定義
 *
 *  この構造体は，同期・通信オブジェクトの待ち情報ブロックの共通部分
 *  （WINFO_WOBJ）を拡張（オブジェクト指向言語の継承に相当）したもので，
 *  最初の2つのフィールドが共通になっている．
 *  データキューへの送信待ちとデータキューからの受信待ちで，同じ待ち情
 *  報ブロックを使う．
 */
typedef struct dataqueue_waiting_information {
	WINFO		winfo;			/* 標準の待ち情報ブロック */
	DTQCB		*p_dtqcb;		/* 待っているデータキューの管理ブロック */
	intptr_t	data;			/* 送受信データ */
} WINFO_DTQ;

/*
 *  使用していないデータキュー管理ブロックのリスト
 */
extern QUEUE	free_dtqcb;

/*
 *  データキューIDの最大値（kernel_cfg.c）
 */
extern const ID	tmax_dtqid;
extern const ID	tmax_sdtqid;

/*
 *  データキュー初期化ブロックのエリア（kernel_cfg.c）
 */
extern const DTQINIB	dtqinib_table[];
extern DTQINIB			adtqinib_table[];

/*
 *  データキュー管理ブロックのエリア（kernel_cfg.c）
 */
extern DTQCB	dtqcb_table[];

/*
 *  データキュー管理ブロックからデータキューIDを取り出すためのマクロ
 */
#define	DTQID(p_dtqcb)	((ID)(((p_dtqcb) - dtqcb_table) + TMIN_DTQID))

/*
 *  データキュー機能の初期化
 */
extern void	initialize_dataqueue(void);

/*
 *  データキュー管理領域へのデータの格納
 */
extern void	enqueue_data(DTQCB *p_dtqcb, intptr_t data);

/*
 *  データキュー管理領域へのデータの強制格納
 */
extern void	force_enqueue_data(DTQCB *p_dtqcb, intptr_t data);

/*
 *  データキュー管理領域からのデータの取出し
 */
extern void	dequeue_data(DTQCB *p_dtqcb, intptr_t *p_data);

/*
 *  データキューへのデータ送信
 */
extern bool_t	send_data(DTQCB *p_dtqcb, intptr_t data, bool_t *p_dspreq);

/*
 *  データキューへのデータ強制送信
 */
extern bool_t	force_send_data(DTQCB *p_dtqcb, intptr_t data);

/*
 *  データキューからのデータ受信
 */
extern bool_t	receive_data(DTQCB *p_dtqcb, intptr_t *p_data,
													bool_t *p_dspreq);

#endif /* TOPPERS_DATAQUEUE_H */
