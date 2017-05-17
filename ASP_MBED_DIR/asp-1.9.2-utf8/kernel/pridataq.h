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
 *  @(#) $Id: pridataq.h 2728 2015-12-30 01:46:11Z ertl-honda $
 */

/*
 *		優先度データキュー機能
 */

#ifndef TOPPERS_PRIDATAQ_H
#define TOPPERS_PRIDATAQ_H

#include "wait.h"

/*
 *  優先度データ管理ブロック
 */
typedef struct pridata_management_block PDQMB;

struct pridata_management_block {
	PDQMB		*p_next;		/* 次のデータ */
	intptr_t	data;			/* データ本体 */
	PRI			datapri;		/* データ優先度 */
};

/*
 *  優先度データキュー初期化ブロック
 *
 *  この構造体は，同期・通信オブジェクトの初期化ブロックの共通部分
 *  （WOBJINIB）を拡張（オブジェクト指向言語の継承に相当）したもので，
 *  最初のフィールドが共通になっている．
 */
typedef struct pridataq_initialization_block {
	ATR			pdqatr;			/* 優先度データキュー属性 */
	uint_t		pdqcnt;			/* 優先度データキューの容量 */
	PRI			maxdpri;		/* データ優先度の最大値 */
	PDQMB		*p_pdqmb;		/* 優先度データキュー管理領域の先頭番地 */
} PDQINIB;

/*
 *  優先度データキュー管理ブロック
 *
 *  この構造体は，同期・通信オブジェクトの管理ブロックの共通部分（WOBJCB）
 *  を拡張（オブジェクト指向言語の継承に相当）したもので，最初の2つの
 *  フィールドが共通になっている．
 */
typedef struct pridataq_control_block {
	QUEUE		swait_queue;	/* 優先度データキュー送信待ちキュー */
	const PDQINIB *p_pdqinib;	/* 初期化ブロックへのポインタ */
	QUEUE		rwait_queue;	/* 優先度データキュー受信待ちキュー */
	uint_t		count;			/* 優先度データキュー中のデータの数 */
	PDQMB		*p_head;		/* 最初のデータ */
	uint_t		unused;			/* 未使用データ管理ブロックの先頭 */
	PDQMB		*p_freelist;	/* 未割当てデータ管理ブロックのリスト */
} PDQCB;

/*
 *  優先度データキュー待ち情報ブロックの定義
 *
 *  この構造体は，同期・通信オブジェクトの待ち情報ブロックの共通部分
 *  （WINFO_WOBJ）を拡張（オブジェクト指向言語の継承に相当）したもので，
 *  最初の2つのフィールドが共通になっている．
 *  優先度データキューへの送信待ちと優先度データキューからの受信待ちで，
 *  同じ待ち情報ブロックを使う．
 */
typedef struct pridataq_waiting_information {
	WINFO		winfo;			/* 標準の待ち情報ブロック */
	PDQCB		*p_pdqcb;		/* 待っている優先度データキューの管理ブロック*/
	intptr_t	data;			/* 送受信データ */
	PRI			datapri;		/* データ優先度 */
} WINFO_PDQ;

/*
 *  使用していない優先度データキュー管理ブロックのリスト
 */
extern QUEUE	free_pdqcb;

/*
 *  優先度データキューIDの最大値（kernel_cfg.c）
 */
extern const ID	tmax_pdqid;
extern const ID	tmax_spdqid;

/*
 *  優先度データキュー初期化ブロックのエリア（kernel_cfg.c）
 */
extern const PDQINIB	pdqinib_table[];
extern PDQINIB			apdqinib_table[];

/*
 *  優先度データキュー管理ブロックのエリア（kernel_cfg.c）
 */
extern PDQCB	pdqcb_table[];

/*
 *  優先度データキュー管理ブロックから優先度データキューIDを取り出すた
 *  めのマクロ
 */
#define	PDQID(p_pdqcb)	((ID)(((p_pdqcb) - pdqcb_table) + TMIN_PDQID))

/*
 *  優先度データキュー機能の初期化
 */
extern void	initialize_pridataq(void);

/*
 *  優先度データキュー管理領域へのデータの格納
 */
extern void	enqueue_pridata(PDQCB *p_pdqcb, intptr_t data, PRI datapri);

/*
 *  優先度データキュー管理領域からのデータの取出し
 */
extern void	dequeue_pridata(PDQCB *p_pdqcb, intptr_t *p_data, PRI *p_datapri);

/*
 *  優先度データキューへのデータ送信
 */
extern bool_t	send_pridata(PDQCB *p_pdqcb, intptr_t data,
											PRI datapri, bool_t *p_dspreq);

/*
 *  優先度データキューからのデータ受信
 */
extern bool_t	receive_pridata(PDQCB *p_pdqcb, intptr_t *p_data,
											PRI *p_datapri, bool_t *p_dspreq);

#endif /* TOPPERS_PRIDATAQ_H */
