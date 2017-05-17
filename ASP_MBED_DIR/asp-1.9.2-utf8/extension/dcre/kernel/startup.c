/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2005-2010 by Embedded and Real-Time Systems Laboratory
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
 *  @(#) $Id: startup.c 2728 2015-12-30 01:46:11Z ertl-honda $
 */

/*
 *		カーネルの初期化と終了処理
 */

#include "kernel_impl.h"
#include "time_event.h"
#include <sil.h>

/*
 *  トレースログマクロのデフォルト定義
 */
#ifndef LOG_KER_ENTER
#define LOG_KER_ENTER()
#endif /* LOG_KER_ENTER */

#ifndef LOG_KER_LEAVE
#define LOG_KER_LEAVE()
#endif /* LOG_KER_LEAVE */

#ifndef LOG_EXT_KER_ENTER
#define LOG_EXT_KER_ENTER()
#endif /* LOG_EXT_KER_ENTER */

#ifndef LOG_EXT_KER_LEAVE
#define LOG_EXT_KER_LEAVE(ercd)
#endif /* LOG_EXT_KER_LEAVE */

#ifdef TOPPERS_sta_ker

/*
 *  カーネル動作状態フラグ
 *
 *  スタートアップルーチンで，false（＝0）に初期化されることを期待して
 *  いる．
 */
bool_t	kerflg = false;

/*
 *  カーネルの起動
 */
void
sta_ker(void)
{
	/*
	 *  ターゲット依存の初期化
	 */
	target_initialize();

	/*
	 *  各モジュールの初期化
	 *
	 *  タイムイベント管理モジュールは他のモジュールより先に初期化
	 *  する必要がある．
	 */
	initialize_kmm();
	initialize_tmevt();
	initialize_object();

	/*
	 *  初期化ルーチンの実行
	 */ 
	call_inirtn();

	/*
	 *  カーネル動作の開始
	 */
	kerflg = true;
	LOG_KER_ENTER();
	start_dispatch();
	assert(0);
}

#endif /* TOPPERS_sta_ker */

/*
 *  カーネルの終了
 */
#ifdef TOPPERS_ext_ker

ER
ext_ker(void)
{
	SIL_PRE_LOC;

	LOG_EXT_KER_ENTER();

	/*
	 *  割込みロック状態に移行
	 */
	SIL_LOC_INT();

	/*
	 *  カーネル動作の終了
	 */
	LOG_KER_LEAVE();
	kerflg = false;

	/*
	 *  カーネルの終了処理の呼出し
	 *
	 *  非タスクコンテキストに切り換えて，exit_kernelを呼び出す．
	 */
	call_exit_kernel();

	/*
	 *  コンパイラの警告対策（ここへ来ることはないはず）
	 */
	SIL_UNL_INT();
	LOG_EXT_KER_LEAVE(E_SYS);
	return(E_SYS);
}

/*
 *  カーネルの終了処理
 */
void
exit_kernel(void)
{
	/*
	 *  終了処理ルーチンの実行
	 */
	call_terrtn();

	/*
	 *  ターゲット依存の終了処理
	 */
	target_exit();
	assert(0);
}

#endif /* TOPPERS_ext_ker */

/*
 *  カーネルの割り付けるメモリ領域の管理
 *
 *  メモリ領域を先頭から順に割り当て，解放されたメモリ領域を再利用しな
 *  いメモリ管理モジュール．
 */
#ifdef TOPPERS_kermem
#ifndef OMIT_KMM_ALLOCONLY

static void	*kmm_brk;

void
initialize_kmm(void)
{
	kmm_brk = ((char *) kmm) + kmmsz;
}

void *
kernel_malloc(SIZE size)
{
	if (((char *) kmm_brk) - ((char *) kmm) >= size) {
		kmm_brk = ((char *) kmm_brk) - size;
		return(kmm_brk);
	}
	else {
		return(NULL);
	}
}

void
kernel_free(void *ptr)
{
}

#endif /* OMIT_KMM_ALLOCONLY */
#endif /* TOPPERS_kermem */
