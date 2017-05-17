/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2013-2014 by Embedded and Real-Time Systems Laboratory
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
 *  $Id: target_config.c 2728 2015-12-30 01:46:11Z ertl-honda $
 */

/*
 *		ターゲット依存モジュール（ダミーターゲット用）
 */

#include "kernel_impl.h"
#include "task.h"

/*
 *  トレースログマクロのデフォルト定義
 */
#ifndef LOG_DSP_ENTER
#define LOG_DSP_ENTER(p_tcb)
#endif /* LOG_DSP_ENTER */

#ifndef LOG_DSP_LEAVE
#define LOG_DSP_LEAVE(p_tcb)
#endif /* LOG_DSP_LEAVE */

/*
 *  ディスパッチャ本体
 */
static void
dispatcher(void)
{
	LOG_DSP_ENTER(p_runtsk);
	while ((p_runtsk = p_schedtsk) == NULL) {
		/* 割込みを許可したらCPUロック解除状態になるよう準備する */
		/* 非タスクコンテキストに切り換える */
		do {
			/* 割込みをすべて許可し，割込み発生を待つ */
			/* 割込み許可前の状態に戻す（＝カーネル管理の割込みを禁止する）*/
		} while (!reqflg);
		reqflg = false;
		/* タスクコンテキストに戻す */
		/* CPUロック状態に戻す */
	}
	/* 自タスク（p_runtsk）のTCBからスタックポインタを復帰する */
	LOG_DSP_LEAVE(p_runtsk);
	/* 自タスクのTCBから実行再開番地を復帰し，そこへ分岐する */
}

/*
 *  最高優先順位タスクへのディスパッチ
 */
void
dispatch(void)
{
#ifdef TOPPERS_SUPPORT_OVRHDR
	ovrtimer_stop();					/* オーバランタイマの停止 */
#endif /* TOPPERS_SUPPORT_OVRHDR */
	/* スクラッチレジスタを除くすべてのレジスタをスタックに保存する */
	/* スタックポインタを自タスク（p_runtsk）のTCBに保存する */
	/* dispatch_rを，実行再開番地として自タスクのTCBに保存する */
	dispatcher();

/* dispatch_r: */
	/* スクラッチレジスタを除くすべてのレジスタをスタックから復帰する */
#ifdef TOPPERS_SUPPORT_OVRHDR
	ovrtimer_start();					/* オーバランタイマの動作開始 */
#endif /* TOPPERS_SUPPORT_OVRHDR */
	calltex();
}

/*
 *  ディスパッチャの動作開始
 */
void
start_dispatch(void)
{
	/* タスクコンテキストに切り換える */
	/* CPUロック状態・割込み優先度マスク全解除状態にする */
	/* dispatcher_0に分岐する */
}

/*
 *  現在のコンテキストを捨ててディスパッチ
 */
void
exit_and_dispatch(void)
{
	/* dispatcherに分岐する */
}

/*
 *  割込みハンドラ出口処理
 *
 *  割込みハンドラ中では，カーネル管理の割込みを禁止しているため，出口
 *  処理で割込みを禁止する必要はない．
 */
void
ret_int(void)
{
}

/*
 *  CPU例外ハンドラ出口処理
 */
void
ret_exc(void)
{
}

/*
 *  カーネルの終了処理の呼出し
 */
void
call_exit_kernel(void)
{
	/* 非タスクコンテキストに切り換える */
	/* exit_kernelに分岐する */
	while (true);
}

/*
 *  タスク開始時処理
 */
void
start_r(void)
{
#ifdef TOPPERS_SUPPORT_OVRHDR
	ovrtimer_start();					/* オーバランタイマの動作開始 */
#endif /* TOPPERS_SUPPORT_OVRHDR */
	/* CPUロック解除状態にする */
	/* 自タスク（p_runtsk）の起動番地を，拡張情報をパラメータとして呼び出す */
	/* ext_tskに分岐する */
}

/*
 *  ターゲット依存の初期化
 */
void
target_initialize(void)
{
}

/*
 *  ターゲット依存の終了処理
 */
void
target_exit(void)
{
	exit(0);
}

/*
 *  システムログの低レベル出力のための文字出力
 */
void
target_fput_log(char c)
{
	/* 文字cを表示/出力/保存する */
}

/*
 *  メイン関数
 */
int
main()
{
	sta_ker();
	return(0);
}

/*
 *  カーネルの割り付けるメモリ領域の管理
 *
 *  TLSF（オープンソースのメモリ管理ライブラリ）を用いて実現．
 */
#ifdef TOPPERS_SUPPORT_DYNAMIC_CRE

#include "tlsf.h"

static bool_t	tlsf_initialized = false;

void
initialize_kmm(void)
{
	if (init_memory_pool(kmmsz, kmm) >= 0) {
		tlsf_initialized = true;
	}
}

void *
kernel_malloc(SIZE size)
{
	if (tlsf_initialized) {
		return(malloc_ex(size, kmm));
	}
	else {
		return(NULL);
	}
}

void
kernel_free(void *ptr)
{
	if (tlsf_initialized) {
		free_ex(ptr, kmm);
	}
}

#endif /* TOPPERS_SUPPORT_DYNAMIC_CRE */
