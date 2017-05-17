/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2006-2014 by Embedded and Real-Time Systems Laboratory
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
 *		ターゲット依存モジュール（Mac OS X用）
 *
 *  カーネルのターゲット依存部のインクルードファイル．kernel_impl.hのター
 *  ゲット依存部の位置付けとなる．
 */

#ifndef TOPPERS_TARGET_CONFIG_H
#define TOPPERS_TARGET_CONFIG_H

/*
 *  標準のインクルードファイル
 */
#ifndef TOPPERS_MACRO_ONLY
#include <sys/types.h>
#include <unistd.h>
#include <stdlib.h>
#include <setjmp.h>
#include <signal.h>
#include <stdio.h>

#ifdef TOPPERS_SUPPORT_OVRHDR
#include "overrun.h"
#endif /* TOPPERS_SUPPORT_OVRHDR */
#endif /* TOPPERS_MACRO_ONLY */

/*
 *  ターゲットシステムのOS依存の定義
 */
#include "macosx.h"

/*
 *  ターゲット定義のオブジェクト属性
 */
#define TARGET_INHATR	TA_NONKERNEL	/* カーネル管理外の割込み */

/*
 *  エラーチェック方法の指定
 */
#define CHECK_STKSZ_ALIGN	16	/* スタックサイズのアライン単位 */
#define CHECK_FUNC_ALIGN	4	/* 関数のアライン単位 */
#define CHECK_FUNC_NONNULL		/* 関数の非NULLチェック */
#define CHECK_STACK_ALIGN	16	/* スタック領域のアライン単位 */
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
 *  アーキテクチャ（プロセッサ）依存の定義
 */
#if defined(__ppc__)

#define JMPBUF_PC				21			/* jmp_buf中でのPCの位置 */
#define JMPBUF_SP				0			/* jmp_buf中でのSPの位置 */
#define TASK_STACK_MERGIN		4U
#define DEFAULT_ISTKSZ			SIGSTKSZ	/* シグナルスタックのサイズ */

#elif defined(__i386__)

#define JMPBUF_PC				12			/* jmp_buf中でのPCの位置 */
#define JMPBUF_SP				9			/* jmp_buf中でのSPの位置 */
#define TASK_STACK_MERGIN		4U 
#define DEFAULT_ISTKSZ			SIGSTKSZ	/* シグナルスタックのサイズ */

#elif defined(__x86_64__)

#error architecture not supported
#define JMPBUF_PC				7			/* jmp_buf中でのPCの位置 */
#define JMPBUF_SP				2			/* jmp_buf中でのSPの位置 */
#define TASK_STACK_MERGIN		8U 
#define DEFAULT_ISTKSZ			SIGSTKSZ	/* シグナルスタックのサイズ */

#else
#error architecture not supported
#endif

/* 
 *  標準の割込み管理機能の初期化を行わないための定義
 */
#define OMIT_INITIALIZE_INTERRUPT

#ifndef TOPPERS_MACRO_ONLY

/*
 *  タスクコンテキストブロックの定義
 */
typedef struct task_context_block {
	jmp_buf		env;			/* コンテキスト情報 */
} TSKCTXB;

/*
 *  割込みハンドラ初期化ブロック
 *
 *  標準の割込みハンドラ初期化ブロックに，割込み優先度を追加したもの．
 */
typedef struct interrupt_handler_initialization_block {
	INHNO		inhno;			/* 割込みハンドラ番号 */
	ATR			inhatr;			/* 割込みハンドラ属性 */
	FP			int_entry;		/* 割込みハンドラの出入口処理の番地 */
	PRI			intpri;			/* 割込み優先度 */
} INHINIB;

/*
 *  割込みハンドラ番号の数（kernel_cfg.c）
 */
extern const uint_t	tnum_inhno;

/*
 *  割込みハンドラ初期化ブロックのエリア（kernel_cfg.c）
 */
extern const INHINIB	inhinib_table[];

/*
 *  シグナルセット操作マクロ
 */
#define sigequalset(set1, set2)		(*(set1) == *(set2))
#define sigassignset(set1, set2)	(*(set1) = *(set2))
#define sigjoinset(set1, set2)		(*(set1) |= *(set2))

/*
 *  割込み優先度マスクによるシグナルマスク（kernel_cfg.c）
 *
 *  割込み優先度マスクによってマスクされている割込みと，割込み属性が設
 *  定されていない割込みに対応するシグナルをマスクするためのシグナルマ
 *  スクを保持する配列．配列のインデックスは，割込み優先度マスクの符号
 *  を反転したもの．
 *
 *  sigmask_table[0]：割込み属性が設定されていない割込みに対応するシグ
 *                    ナルのみをマスクするシグナルマスク
 *  sigmask_table[-TMIN_INTPRI]：カーネル管理の割込みすべてと，割込み属
 *                    性が設定されていない割込みに対応するシグナルをマ
 *                    スクするシグナルマスク
 *  sigmask_table[6]：NMIとSIGUSR2を除くすべての割込みと，割込み属性が設
 *                    定されていない割込みに対応するシグナルをマスクする
 *                    シグナルマスク
 *  sigmask_table[7]：sigmask_table[6]と同じ値
 */
extern const sigset_t sigmask_table[8];

/*
 *  割込み要求禁止フラグ実現のための変数の初期値（kernel_cfg.c）
 */
extern const sigset_t sigmask_disint_init;

/*
 *  割込みロック／CPUロックへの移行でマスクするシグナルを保持する変数
 */
extern sigset_t	sigmask_intlock;	/* 割込みロックでマスクするシグナル */
extern sigset_t	sigmask_cpulock;	/* CPUロックでマスクするシグナル */

/*
 *  コンテキストの参照
 */
Inline bool_t
sense_context(void)
{
	stack_t	ss;

	sigaltstack(NULL, &ss);
	return((ss.ss_flags & SA_ONSTACK) != 0);
}

/*
 *  CPUロックフラグ実現のための変数
 */
extern volatile bool_t		lock_flag;		/* CPUロックフラグを表す変数 */
extern volatile sigset_t	saved_sigmask;	/* シグナルマスクを保存する変数 */

/*
 *  割込み優先度マスク実現のための変数
 */
extern volatile PRI			ipm_value;		/* 割込み優先度マスクを表す変数 */

/*
 *  割込み要求禁止フラグ実現のための変数
 */
extern volatile sigset_t	sigmask_disint;	/* 個別にマスクしているシグナル */

/*
 *  シグナルマスクの設定
 *
 *  現在の状態（コンテキスト，CPUロックフラグ，割込み優先度マスク，割込
 *  み禁止フラグ）を参照して，現在のシグナルマスクとsaved_sigmaskを適切
 *  な値に設定する．
 */
Inline void
set_sigmask(void)
{
	sigset_t	sigmask;

	sigassignset(&sigmask, &(sigmask_table[-ipm_value]));
	sigjoinset(&sigmask, &sigmask_disint);
	if (sense_context()) {
		sigaddset(&sigmask, SIGUSR2);
	}
	if (lock_flag) {
		sigassignset(&saved_sigmask, &sigmask);
		sigjoinset(&sigmask, &sigmask_cpulock);
	}
	sigprocmask(SIG_SETMASK, &sigmask, NULL);
}

/*
 *  CPUロック状態への移行
 */
Inline void
x_lock_cpu(void)
{
	assert(!lock_flag);
	sigprocmask(SIG_BLOCK, &sigmask_cpulock, (sigset_t *) &saved_sigmask);
	lock_flag = true;
}

#define t_lock_cpu()	x_lock_cpu()
#define i_lock_cpu()	x_lock_cpu()

/*
 *  CPUロック状態の解除
 */
Inline void
x_unlock_cpu(void)
{
	assert(lock_flag);
	lock_flag = false;
	sigprocmask(SIG_SETMASK, (sigset_t *) &saved_sigmask, NULL);
}

#define t_unlock_cpu()	x_unlock_cpu()
#define i_unlock_cpu()	x_unlock_cpu()

/*
 *  CPUロック状態の参照
 */
Inline bool_t
x_sense_lock(void)
{
	return(lock_flag);
}

#define t_sense_lock()	x_sense_lock()
#define i_sense_lock()	x_sense_lock()

/*
 *  割込み優先度マスクの設定
 */
Inline void
x_set_ipm(PRI intpri)
{
	ipm_value = intpri;
	set_sigmask();
}

#define t_set_ipm(intpri)	x_set_ipm(intpri)
#define i_set_ipm(intpri)	x_set_ipm(intpri)

/*
 *  割込み優先度マスクの参照
 */
Inline PRI
x_get_ipm(void)
{
	return(ipm_value);
}

#define t_get_ipm()	x_get_ipm()
#define i_get_ipm()	x_get_ipm()

/*
 *  割込み番号の範囲の判定
 */
#define	VALID_INTNO(intno)	(1 <= (intno) && (intno) <= 30 \
								&& (intno) != SIGKILL && (intno) != SIGSTOP)
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
	if (sigismember(&(sigmask_table[0]), intno)
				|| !sigismember(&(sigmask_table[7]), intno)) {
		return(false);
	}
	sigaddset(&sigmask_disint, intno);
	set_sigmask();
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
	if (sigismember(&(sigmask_table[0]), intno)
				|| !sigismember(&(sigmask_table[7]), intno)) {
		return(false);
	}
	sigdelset(&sigmask_disint, intno);
	set_sigmask();
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
	sigset_t	sigmask;

	sigpending(&sigmask);
	return(sigismember(&sigmask, intno));
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
 *
 *  dispatchは，タスクコンテキストから呼び出されたサービスコール処理か
 *  ら呼び出すべきもので，タスクコンテキスト・CPUロック状態・ディスパッ
 *  チ許可状態・（モデル上の）割込み優先度マスク全解除状態で呼び出さな
 *  ければならない．
 */
extern void	dispatch(void);

/*
 *  ディスパッチャの動作開始
 *
 *  start_dispatchをreturnにマクロ定義することで，カーネルの初期化完了
 *  後にsta_kerからmainにリターンさせ，シグナルスタックから元のスタック
 *  に戻す．
 */
#define start_dispatch()	return

/*
 *  現在のコンテキストを捨ててディスパッチ
 *
 *  exit_and_dispatchは，ext_tskから呼び出すべきもので，タスクコンテキ
 *  スト・CPUロック状態・ディスパッチ許可状態・（モデル上の）割込み優先
 *  度マスク全解除状態で呼び出さなければならない．
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
 *
 *  call_exit_kernelは，カーネルの終了時に呼び出すべきもので，非タスク
 *  コンテキストに切り換えて，カーネルの終了処理（exit_kernel）を呼び出
 *  す．
 */
extern void call_exit_kernel(void) NoReturn;

/*
 *  タスクコンテキストの初期化
 *
 *  タスクが休止状態から実行できる状態に移行する時に呼ばれる．この時点
 *  でスタック領域を使ってはならない．
 *
 *  activate_contextを，インライン関数ではなくマクロ定義としているのは，
 *  この時点ではTCBが定義されていないためである．
 *
 *  Intelプロセッサでは，スタックは16ビット境界にアラインしていなければ
 *  ならない．
 */
extern void	start_r(void);

#define activate_context(p_tcb)											\
{																		\
	((intptr_t *) &((p_tcb)->tskctxb.env))[JMPBUF_PC]					\
											= (intptr_t) start_r;		\
	((intptr_t *) &((p_tcb)->tskctxb.env))[JMPBUF_SP]					\
						= ((((intptr_t)((char *)((p_tcb)->p_tinib->stk)	\
								+ (p_tcb)->p_tinib->stksz)) & ~0x0f)	\
								- TASK_STACK_MERGIN);					\
}

/*
 *  割込みハンドラ番号とCPU例外ハンドラ番号の範囲の判定
 */
#define VALID_INHNO_DEFINH(inhno)		VALID_INTNO((INTNO)(inhno))
#define VALID_EXCNO_DEFEXC(excno)		VALID_INTNO((INTNO)(excno))

/*
 *  割込みハンドラの設定
 *
 *  ベクトル番号inhnoの割込みハンドラの出入口処理の番地をint_entryに，
 *  割込み優先度をintpriに設定する．
 */
Inline void
x_define_inh(INHNO inhno, FP int_entry, PRI intpri)
{
	struct sigaction	sigact;

	assert(VALID_INHNO_DEFINH(inhno));
	sigact.sa_handler = (void (*)(int))(int_entry);
	sigact.sa_flags = SA_ONSTACK;
	sigassignset(&(sigact.sa_mask), &(sigmask_table[-intpri]));
	sigaddset(&(sigact.sa_mask), SIGUSR2);
	sigaction(inhno, &sigact, NULL);
}

/*
 *  CPU例外ハンドラの設定
 *
 *  ベクトル番号excnoのCPU例外ハンドラの出入口処理の番地をexc_entryに設
 *  定する．
 *
 *  SA_NODEFERにより，シグナルハンドラの起動時に，そのシグナルをマスク
 *  するのを抑止している．
 */
Inline void
x_define_exc(EXCNO excno, FP exc_entry)
{
	struct sigaction	sigact;

	assert(VALID_EXCNO_DEFEXC(excno));
	sigact.sa_sigaction =
				(void (*)(int, struct __siginfo *, void *))(exc_entry);
	sigact.sa_flags = (SA_ONSTACK | SA_SIGINFO | SA_NODEFER);
	sigemptyset(&(sigact.sa_mask));
	sigaddset(&(sigact.sa_mask), SIGUSR2);
	sigaction(excno, &sigact, NULL);
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
#define INT_ENTRY(inhno, inthdr)	_kernel_##inthdr##_##inhno

#define INTHDR_ENTRY(inhno, inthdr, intpri)						\
void _kernel_##inthdr##_##inhno(void)							\
{																\
	PRI		saved_ipm;											\
																\
	saved_ipm = _kernel_ipm_value;								\
	_kernel_ipm_value = intpri;									\
	OVRTIMER_STOP();											\
	LOG_INH_ENTER(inhno);										\
	inthdr();			/* 割込みハンドラを呼び出す */			\
	LOG_INH_LEAVE(inhno);										\
	_kernel_ret_int();	/* 割込みハンドラ出口処理を呼び出す */	\
	_kernel_ipm_value = saved_ipm;								\
	_kernel_lock_flag = false;									\
}

/*
 *  CPU例外ハンドラの入口処理の生成マクロ
 */
#define EXC_ENTRY(excno, exchdr)	_kernel_##exchdr##_##excno

#define EXCHDR_ENTRY(excno, excno_num, exchdr)							\
void _kernel_##exchdr##_##excno(int sig,								\
						struct __siginfo *p_info, void *p_ctx)			\
{																		\
	bool_t		saved_lock_flag;										\
																		\
	saved_lock_flag = _kernel_lock_flag;								\
	if (exc_sense_nonkernel(p_ctx)) {									\
		/* カーネル管理外のCPU例外ハンドラの場合 */						\
		exchdr(p_ctx);			/* CPU例外ハンドラを呼び出す */			\
	}																	\
	else {																\
		/* カーネル管理のCPU例外ハンドラの場合 */						\
		OVRTIMER_STOP();												\
		LOG_EXC_ENTER(excno);											\
		exchdr(p_ctx);			/* CPU例外ハンドラを呼び出す */			\
		LOG_EXC_LEAVE(excno);											\
		_kernel_ret_exc();		/* CPU例外ハンドラ出口処理を呼び出す */	\
	}																	\
	_kernel_lock_flag = saved_lock_flag;								\
}

/*
 *  CPU例外の発生した時のコンテキストの参照
 *
 *  CPU例外の発生した時のコンテキストが，タスクコンテキストの時にfalse，
 *  そうでない時にtrueを返す．
 */
Inline bool_t
exc_sense_context(void *p_excinf)
{
	return(((ucontext_t *) p_excinf)->uc_onstack != 0);
}

/*
 *  カーネル管理外のCPU例外の判別
 *
 *  カーネル管理外のCPU例外の時にtrue，そうでない時にfalseを返す．
 */
Inline bool_t
exc_sense_nonkernel(void *p_excinf)
{
	sigset_t	sigmask;

	sigassignset(&sigmask, &(((ucontext_t *) p_excinf)->uc_sigmask));
	return(sigismember(&sigmask, SIGUSR2));
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
	return(!exc_sense_context(p_excinf) && !exc_sense_nonkernel(p_excinf)
												&& ipm_value == TIPM_ENAALL);
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
