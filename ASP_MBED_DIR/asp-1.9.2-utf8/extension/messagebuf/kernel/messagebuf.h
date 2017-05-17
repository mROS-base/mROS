/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2005-2014 by Embedded and Real-Time Systems Laboratory
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
 *  $Id: messagebuf.h 2728 2015-12-30 01:46:11Z ertl-honda $
 */

/*
 *		メッセージバッファ機能
 */

#ifndef TOPPERS_MESSAGEBUF_H
#define TOPPERS_MESSAGEBUF_H

#include "wait.h"

/*
 *  メッセージバッファ初期化ブロック
 *
 *  この構造体は，同期・通信オブジェクトの初期化ブロックの共通部分
 *  （WOBJINIB）を拡張（オブジェクト指向言語の継承に相当）したもので，
 *  最初のフィールドが共通になっている．
 */
typedef struct messagebuf_initialization_block {
	ATR			mbfatr;			/* メッセージバッファ属性 */
	uint_t		maxmsz;			/* メッセージの最大長 */
	SIZE		mbfsz;			/* メッセージバッファ管理領域のサイズ */
	void		*mbfmb;			/* メッセージバッファ管理領域の先頭番地 */
} MBFINIB;

/*
 *  メッセージバッファ管理ブロック
 *
 *  この構造体は，同期・通信オブジェクトの管理ブロックの共通部分（WOBJCB）
 *  を拡張（オブジェクト指向言語の継承に相当）したもので，最初の2つの
 *  フィールドが共通になっている．
 */
typedef struct messagebuf_control_block {
	QUEUE		swait_queue;	/* メッセージバッファ送信待ちキュー */
	const MBFINIB *p_mbfinib;	/* 初期化ブロックへのポインタ */
	QUEUE		rwait_queue;	/* メッセージバッファ受信待ちキュー */
	SIZE		fmbfsz;			/* 空き領域のサイズ */
	SIZE		head;			/* 最初のメッセージの格納場所 */
	SIZE		tail;			/* 最後のメッセージの格納場所の次 */
	uint_t		smbfcnt;		/* 管理領域に格納されているメッセージの数 */
} MBFCB;

/*
 *  メッセージバッファ待ち情報ブロックの定義
 *
 *  この構造体は，同期・通信オブジェクトの待ち情報ブロックの共通部分
 *  （WINFO_WOBJ）を拡張（オブジェクト指向言語の継承に相当）したもので，
 *  最初の2つのフィールドが共通になっている．
 *  メッセージバッファへの送信待ちとメッセージバッファからの受信待ちで，
 *  同じ待ち情報ブロックを使う．
 */
typedef struct messagebuf_waiting_information {
	WINFO		winfo;			/* 標準の待ち情報ブロック */
	MBFCB		*p_mbfcb;		/* 待っているメッセージバッファの管理ブロック */
	void		*msg;			/* 送受信メッセージ */
	uint_t		msgsz;			/* 送受信メッセージサイズ */
} WINFO_MBF;

/*
 *  メッセージバッファIDの最大値（kernel_cfg.c）
 */
extern const ID	tmax_mbfid;

/*
 *  メッセージバッファ初期化ブロックのエリア（kernel_cfg.c）
 */
extern const MBFINIB	mbfinib_table[];

/*
 *  メッセージバッファ管理ブロックのエリア（kernel_cfg.c）
 */
extern MBFCB	mbfcb_table[];

/*
 *  メッセージバッファ管理ブロックからメッセージバッファIDを取り出すた
 *  めのマクロ
 */
#define	MBFID(p_mbfcb)	((ID)(((p_mbfcb) - mbfcb_table) + TMIN_MBFID))

/*
 *  メッセージバッファ機能の初期化
 */
extern void	initialize_messagebuf(void);

/*
 *  メッセージバッファ管理領域へのメッセージの格納
 */
extern bool_t	enqueue_message(MBFCB *p_mbfcb, const void *msg, uint_t msgsz);

/*
 *  メッセージバッファ管理領域からのメッセージの取出し
 */
extern uint_t	dequeue_message(MBFCB *p_mbfcb, void *msg);

/*
 *  メッセージバッファへのメッセージ送信
 */
extern bool_t	send_message(MBFCB *p_mbfcb, const void *msg,
											uint_t msgsz, bool_t *p_dspreq);

/*
 *  メッセージバッファ送信待ちタスクのチェック
 */
extern bool_t	messagebuf_signal(MBFCB *p_mbfcb);

/*
 *  メッセージバッファからのメッセージ受信
 */
extern uint_t	receive_message(MBFCB *p_mbfcb, void *msg, bool_t *p_dspreq);

/*
 *  メッセージバッファ送信待ちタスクの待ち解除時処理
 */
extern bool_t	(*mbfhook_dequeue_wobj)(TCB *p_tcb);
extern bool_t	messagebuf_dequeue_wobj(TCB *p_tcb);

/*
 *  メッセージバッファ送信待ちタスクの優先度変更時処理
 */
extern bool_t	(*mbfhook_change_priority)(WOBJCB *p_wobjcb);
extern bool_t	messagebuf_change_priority(WOBJCB *p_wobjcb);

#endif /* TOPPERS_MESSAGEBUF_H */
