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
 *  $Id: target_config.h 2728 2015-12-30 01:46:11Z ertl-honda $
 */

/*
 *		ターゲット依存モジュール（ダミーターゲット用）
 *
 *  カーネルのターゲット依存部のインクルードファイル．kernel_impl.hのター
 *  ゲット依存部の位置付けとなる．
 */

#ifndef TOPPERS_TARGET_CONFIG_H
#define TOPPERS_TARGET_CONFIG_H

#ifndef TOPPERS_MACRO_ONLY
#ifdef TOPPERS_SUPPORT_OVRHDR
#include "overrun.h"
#endif /* TOPPERS_SUPPORT_OVRHDR */
#endif /* TOPPERS_MACRO_ONLY */

/*
 *  ターゲットシステムのハードウェア資源の定義
 */
#include "dummy.h"

/*
 *  エラーチェック方法の指定
 */
#define CHECK_STKSZ_ALIGN	4	/* スタックサイズのアライン単位 */
#define CHECK_FUNC_ALIGN	4	/* 関数のアライン単位 */
#define CHECK_FUNC_NONNULL		/* 関数の非NULLチェック */
#define CHECK_STACK_ALIGN	4	/* スタック領域のアライン単位 */
#define CHECK_STACK_NONNULL		/* スタック領域の非NULLチェック */
#define CHECK_MPF_ALIGN		4	/* 固定長メモリプール領域のアライン単位 */
#define CHECK_MPF_NONNULL		/* 固定長メモリプール領域の非NULLチェック */
#define CHECK_MB_ALIGN		4	/* 管理領域のアライン単位 */

/*
 *  トレースログに関する設定
 */
#ifdef TOPPERS_ENABLE_TRACE
#include "logtrace/trace_config.h"
#endif /* TOPPERS_ENABLE_TRACE */

/*
 *  トレースログマクロのデフォルト定義
 */
#ifndef LOG_INH_ENTER
#define LOG_INH_ENTER(inhno)
#endif /* LOG_INH_ENTER */

#ifndef LOG_INH_LEAVE
#define LOG_INH_LEAVE(inhno)
#endif /* LOG_INH_LEAVE */

#ifndef LOG_EXC_ENTER
#define LOG_EXC_ENTER(excno)
#endif /* LOG_EXC_ENTER */

#ifndef LOG_EXC_LEAVE
#define LOG_EXC_LEAVE(excno)
#endif /* LOG_EXC_LEAVE */

/*
 *  非タスクコンテキスト用スタックのデフォルトのサイズ
 */
#define DEFAULT_ISTKSZ			4096

#ifndef TOPPERS_MACRO_ONLY

/*
 *  タスクコンテキストブロックの定義
 */
typedef struct task_context_block {
	void		*sp;			/* スタックポインタ */
	void		*pc;			/* 実行再開番地 */
} TSKCTXB;

/*
 *  コンテキストの参照
 */
Inline bool_t
sense_context(void)
{
	return(false);
}

/*
 *  CPUロック状態への移行
 */
Inline void
x_lock_cpu(void)
{
}

#define t_lock_cpu()	x_lock_cpu()
#define i_lock_cpu()	x_lock_cpu()

/*
 *  CPUロック状態の解除
 */
Inline void
x_unlock_cpu(void)
{
}

#define t_unlock_cpu()	x_unlock_cpu()
#define i_unlock_cpu()	x_unlock_cpu()

/*
 *  CPUロック状態の参照
 */
Inline bool_t
x_sense_lock(void)
{
	return(false);
}

#define t_sense_lock()	x_sense_lock()
#define i_sense_lock()	x_sense_lock()

/*
 *  割込み優先度マスクの設定
 */
Inline void
x_set_ipm(PRI intpri)
{
}

#define t_set_ipm(intpri)	x_set_ipm(intpri)
#define i_set_ipm(intpri)	x_set_ipm(intpri)

/*
 *  割込み優先度マスクの参照
 */
Inline PRI
x_get_ipm(void)
{
	return(TIPM_ENAALL);
}

#define t_get_ipm()	x_get_ipm()
#define i_get_ipm()	x_get_ipm()

/*
 *  割込み番号の範囲の判定
 */
#define	VALID_INTNO(intno)	(0 <= (intno) && (intno) <= 31)
#define	VALID_INTNO_CREISR(intno)	VALID_INTNO(intno)
#define	VALID_INTNO_DISINT(intno)	VALID_INTNO(intno)

/*
 *  割込み要求禁止フラグのセット
 *
 *  割込み属性が設定されていない割込み要求ラインに対して割込み要求禁止
 *  フラグをセットしようとした場合には，falseを返す．
 */
Inline bool_t
x_disable_int(INTNO intno)
{
	return(true);
}

#define t_disable_int(intno)	x_disable_int(intno)
#define i_disable_int(intno)	x_disable_int(intno)

/*
 *  割込み要求禁止フラグのクリア
 *
 *  割込み属性が設定されていない割込み要求ラインに対して割込み要求禁止
 *  フラグをクリアしようとした場合には，falseを返す．
 */
Inline bool_t
x_enable_int(INTNO intno)
{
	return(true);
}

#define t_enable_int(intno)		x_enable_int(intno)
#define i_enable_int(intno)		x_enable_int(intno)

/*
 *  割込み要求のクリア
 */
Inline void
x_clear_int(INTNO intno)
{
}

#define t_clear_int(intno)		x_clear_int(intno)
#define i_clear_int(intno)		x_clear_int(intno)

/*
 *  割込み要求のチェック
 */
Inline bool_t
x_probe_int(INTNO intno)
{
	return(true);
}

#define t_probe_int(intno)		x_probe_int(intno)
#define i_probe_int(intno)		x_probe_int(intno)

/*
 *  割込みハンドラの入口で必要なIRC操作
 */
Inline void
i_begin_int(INTNO intno)
{
}

/*
 *  割込みハンドラの出口で必要なIRC操作
 */
Inline void
i_end_int(INTNO intno)
{
}

/*
 *  最高優先順位タスクへのディスパッチ
 */
extern void	dispatch(void);

/*
 *  ディスパッチャの動作開始
 */
extern void start_dispatch(void);

/*
 *  現在のコンテキストを捨ててディスパッチ
 */
extern void	exit_and_dispatch(void);

/*
 *  割込みハンドラ出口処理
 */
extern void	ret_int(void);

/*
 *  CPU例外ハンドラ出口処理
 */
extern void	ret_exc(void);

/*
 *  カーネルの終了処理の呼出し
 */
extern void call_exit_kernel(void) NoReturn;

/*
 *  タスクコンテキストの初期化
 */
extern void	start_r(void);

#define activate_context(p_tcb)

/*
 *  割込みハンドラ番号とCPU例外ハンドラ番号の範囲の判定
 */
#define VALID_INHNO_DEFINH(inhno)		VALID_INTNO((INTNO)(inhno))
#define VALID_EXCNO_DEFEXC(excno)		VALID_INTNO((INTNO)(excno))

/*
 *  割込みハンドラの設定
 *
 *  ベクトル番号inhnoの割込みハンドラの出入口処理の番地をint_entryに
 *  設定する．
 */
Inline void
x_define_inh(INHNO inhno, FP int_entry)
{
}

/*
 *  割込み要求ライン属性の設定
 */
Inline void
x_config_int(INTNO intno, ATR intatr, PRI intpri)
{
}

/*
 *  CPU例外ハンドラの設定
 *
 *  ベクトル番号excnoのCPU例外ハンドラの出入口処理の番地をexc_entryに設
 *  定する．
 */
Inline void
x_define_exc(EXCNO excno, FP exc_entry)
{
}

/*
 *  オーバランハンドラ停止のためのマクロ
 */
#ifdef TOPPERS_SUPPORT_OVRHDR
#define OVRTIMER_STOP()	{				\
			i_lock_cpu();				\
			_kernel_ovrtimer_stop();	\
			i_unlock_cpu();				\
		}
#else /* TOPPERS_SUPPORT_OVRHDR */
#define OVRTIMER_STOP()
#endif /* TOPPERS_SUPPORT_OVRHDR */

/*
 *  割込みハンドラの入口処理の生成マクロ
 */
#define INT_ENTRY(inhno, inthdr)	inthdr
#define INTHDR_ENTRY(inhno, inthdr, intpri)

/*
 *  CPU例外ハンドラの入口処理の生成マクロ
 */
#define EXC_ENTRY(excno, exchdr)	exchdr
#define EXCHDR_ENTRY(excno, excno_num, exchdr)

/*
 *  CPU例外の発生した時のコンテキストの参照
 *
 *  CPU例外の発生した時のコンテキストが，タスクコンテキストの時にfalse，
 *  そうでない時にtrueを返す．
 */
Inline bool_t
exc_sense_context(void *p_excinf)
{
	return(true);
}

/*
 *  カーネル管理外のCPU例外の判別
 *
 *  カーネル管理外のCPU例外の時にtrue，そうでない時にfalseを返す．
 */
Inline bool_t
exc_sense_nonkernel(void *p_excinf)
{
	return(true);
}

/*
 *  CPU例外の発生した時のコンテキストと割込みのマスク状態の参照
 *
 *  CPU例外の発生した時のシステム状態が，カーネル実行中でなく，タスクコ
 *  ンテキストであり，全割込みロック状態でなく，CPUロック状態でなく，割
 *  込み優先度マスク全解除状態である時にtrue，そうでない時にfalseを返す
 *  （CPU例外がカーネル管理外の割込み処理中で発生した場合にもfalseを返
 *  す）．
 */
Inline bool_t
exc_sense_intmask(void *p_excinf)
{
	return(true);
}

/*
 *  ターゲットシステム依存の初期化
 */
extern void	target_initialize(void);

/*
 *  ターゲットシステムの終了
 *
 *  システムを終了する時に使う．
 */
extern void	target_exit(void) NoReturn;

#endif /* TOPPERS_MACRO_ONLY */

/*
 *  カーネルの割り付けるメモリ領域の管理
 *
 *  target_config.cに，TLSF（オープンソースのメモリ管理ライブラリ）を用
 *  いたメモリ管理ルーチンを含めている．
 */
#define OMIT_KMM_ALLOCONLY

#endif /* TOPPERS_TARGET_CONFIG_H */
