/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2005-2012 by Embedded and Real-Time Systems Laboratory
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
 *  @(#) $Id: mempfix.h 2728 2015-12-30 01:46:11Z ertl-honda $
 */

/*
 *		固定長メモリプール機能
 */

#ifndef TOPPERS_MEMPFIX_H
#define TOPPERS_MEMPFIX_H

#include "wait.h"

/*
 *  固定長メモリブロック管理ブロック
 *
 *  nextフィールドには，メモリブロックが割当て済みの場合はINDEX_ALLOCを，
 *  未割当ての場合は次の未割当てブロックのインデックス番号を格納する．
 *  最後の未割当てブロックの場合には，INDEX_NULLを格納する．
 */
typedef struct fixed_memoryblock_management_block {
	uint_t		next;			/* 次の未割当てブロック */
} MPFMB;

/*
 *  固定長メモリプール初期化ブロック
 *
 *  この構造体は，同期・通信オブジェクトの初期化ブロックの共通部分
 *  （WOBJINIB）を拡張（オブジェクト指向言語の継承に相当）したもので，
 *  最初のフィールドが共通になっている．
 */
typedef struct fixed_memorypool_initialization_block {
	ATR			mpfatr;			/* 固定長メモリプール属性 */
	uint_t		blkcnt;			/* メモリブロック数 */
	uint_t		blksz;			/* メモリブロックのサイズ（丸めた値） */
	void		*mpf;			/* 固定長メモリプール領域の先頭番地 */
	MPFMB		*p_mpfmb;		/* 固定長メモリプール管理領域の先頭番地 */
} MPFINIB;

/*
 *  固定長メモリプール管理ブロック
 *
 *  この構造体は，同期・通信オブジェクトの管理ブロックの共通部分（WOBJCB）
 *  を拡張（オブジェクト指向言語の継承に相当）したもので，最初の2つの
 *  フィールドが共通になっている．
 */
typedef struct fixed_memorypool_control_block {
	QUEUE		wait_queue;		/* 固定長メモリプール待ちキュー */
	const MPFINIB *p_mpfinib;	/* 初期化ブロックへのポインタ */
	uint_t		fblkcnt;		/* 未割当てブロック数 */
	uint_t		unused;			/* 未使用ブロックの先頭 */
	uint_t		freelist;		/* 未割当てブロックのリスト */
} MPFCB;

/*
 *  固定長メモリプール待ち情報ブロックの定義
 *
 *  この構造体は，同期・通信オブジェクトの待ち情報ブロックの共通部分
 *  （WINFO_WOBJ）を拡張（オブジェクト指向言語の継承に相当）したもので，
 *  最初の2つのフィールドが共通になっている．
 */
typedef struct fixed_memorypool_waiting_information {
	WINFO		winfo;			/* 標準の待ち情報ブロック */
	MPFCB		*p_mpfcb;		/* 待っている固定長メモリプールの管理ブロック*/
	void		*blk;			/* 獲得したメモリブロック */
} WINFO_MPF;

/*
 *  使用していない固定長メモリプール管理ブロックのリスト
 */
extern QUEUE	free_mpfcb;

/*
 *  固定長メモリプールIDの最大値（kernel_cfg.c）
 */
extern const ID	tmax_mpfid;
extern const ID	tmax_smpfid;

/*
 *  固定長メモリプール初期化ブロックのエリア（kernel_cfg.c）
 */
extern const MPFINIB	mpfinib_table[];
extern MPFINIB			ampfinib_table[];

/*
 *  固定長メモリプール管理ブロックのエリア（kernel_cfg.c）
 */
extern MPFCB	mpfcb_table[];

/*
 *  固定長メモリプール管理ブロックから固定長メモリプールIDを取り出すた
 *  めのマクロ
 */
#define	MPFID(p_mpfcb)	((ID)(((p_mpfcb) - mpfcb_table) + TMIN_MPFID))

/*
 *  固定長メモリプール機能の初期化
 */
extern void	initialize_mempfix(void);

/*
 *  固定長メモリプールからブロックを獲得
 */
extern void	get_mpf_block(MPFCB *p_mpfcb, void **p_blk);

#endif /* TOPPERS_MEMPFIX_H */
