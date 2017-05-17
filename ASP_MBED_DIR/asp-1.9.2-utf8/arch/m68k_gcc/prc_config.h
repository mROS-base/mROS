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
 *  $Id: prc_config.h 2728 2015-12-30 01:46:11Z ertl-honda $
 */

/*
 *		プロセッサ依存モジュール（M68040用）
 *
 *  このインクルードファイルは，target_config.h（または，そこからインク
 *  ルードされるファイル）のみからインクルードされる．他のファイルから
 *  直接インクルードしてはならない．
 */

#ifndef TOPPERS_PRC_CONFIG_H
#define TOPPERS_PRC_CONFIG_H

#ifndef TOPPERS_MACRO_ONLY

/*
 *  プロセッサの特殊命令のインライン関数定義
 */
#include "prc_insn.h"

/*
 *  タスクコンテキストブロックの定義
 */
typedef struct task_context_block {
	void	*msp;		/* スタックポインタ */
	FP		pc;			/* プログラムカウンタ */
} TSKCTXB;

#endif /* TOPPERS_MACRO_ONLY */

/*
 *  割込み優先度マスク操作ライブラリ
 *
 *  M68040では，ステータスレジスタ（SR）の下から8～10ビットめの3ビット
 *  に割込み優先度マスク（ハードウェアの割込み優先度マスク，IPM）が置か
 *  れている．IPMを保存しておくために，割込み優先度の外部表現（-1から連
 *  続した負の値）を使うことも可能であるが，余計な左右ビットシフトと符
 *  号反転が必要になる．これを避けるために，IPMを保存する場合には，SRの
 *  8～10ビットめを取り出した値を使うことにする．この値を割込み優先度マ
 *  スクの内部表現と呼び，IIPMと書くことにする．
 */

/*
 *  割込み優先度マスクの外部表現と内部表現の変換
 */
#define EXT_IPM(iipm)	(-CAST(PRI, (iipm) >> 8))		/* 外部表現に変換 */
#define INT_IPM(ipm)	(CAST(uint16_t, -(ipm)) << 8)	/* 内部表現に変換 */

#ifndef TOPPERS_MACRO_ONLY

/*
 *  IPM（ハードウェアの割込み優先度マスク，内部表現）の現在値の読出し
 */
Inline uint16_t
current_iipm(void)
{
	return(current_sr() & 0x0700U);
}

/*
 *  IPM（ハードウェアの割込み優先度マスク，内部表現）の現在値の設定
 */
Inline void
set_iipm(uint16_t iipm)
{
	set_sr((current_sr() & ~0x0700U) | iipm);
}

/*
 *  TOPPERS標準割込み処理モデルの実現
 *
 *  M68040は，ステータスレジスタ（SR）中に割込み優先度マスク（ハードウェ
 *  アの割込み優先度マスク，IPM）を持っているが，CPUロックフラグに相当
 *  する機能を持たない．そのため，CPUロックフラグの機能を，IPMによって
 *  実現する．
 *
 *  まずCPUロックフラグの値（すなわち，CPUロック状態かCPUロック解除状態
 *  か）は，そのための変数（lock_flag）を用意して保持する．
 *
 *  CPUロックフラグがクリアされている間（すなわち，CPUロック解除状態の
 *  間）は，IPM（ハードウェアの割込み優先度マスク）を，モデル上の割込み
 *  優先度マスクの値に設定する．この間は，モデル上の割込み優先度マスク
 *  は，IPMを用いて保持する．
 *
 *  それに対してCPUロックフラグがセットされている間（すなわち，CPUロッ
 *  ク状態の間）は，IPM（ハードウェアの割込み優先度マスク）を，カーネル
 *  管理の割込みをすべてマスクする値（TIPM_LOCK）と，モデル上の割込み優
 *  先度マスクとの高い方に設定する．この間のモデル上の割込み優先度マス
 *  クは，そのための変数（saved_iipm，内部表現で保持）を用意して保持す
 *  る．
 */

/*
 *  コンテキストの参照
 *
 *  M68040では，タスクコンテキストをマスタモードで，非タスクコンテキス
 *  トを割込みモードで実行する．マスタモードか割込みモードかは，ステー
 *  タスレジスタ（SR）中の割込みモードビットにより判別できる．
 */
Inline bool_t
sense_context(void)
{
	return((current_sr() & 0x1000U) == 0U);
}

#endif /* TOPPERS_MACRO_ONLY */

/*
 *  CPUロック状態での割込み優先度マスク
 *
 *  TIPM_LOCKは，カーネル管理の割込みをすべてマスクする値に定義する．
 *
 *  TIPM_LOCKは，基本的にはTMIN_INTPRIに一致させればよいが，M68040では，
 *  IPM（ハードウェアの割込み優先度マスク）が-6の場合と-7の場合の振舞い
 *  が同じで，IPMに-7を設定することは効率的に行えることから，
 *  TMIN_INTPRIが-6の時はTIPM_LOCKを-7にしている．
 */
#if TMIN_INTPRI == -6		/* NMI以外にカーネル管理外の割込みを設けない */
#define TIPM_LOCK		(-7)
#else /* TMIN_INTPRI == -6 */
#if (-1 >= TMIN_INTPRI) && (TMIN_INTPRI > -6)
#define TIPM_LOCK		TMIN_INTPRI
#else /* (-1 >= TMIN_INTPRI) && (TMIN_INTPRI > -6) */
#error TMIN_INTPRI out of range.
#endif /* (-1 >= TMIN_INTPRI) && (TMIN_INTPRI > -6) */
#endif /* TMIN_INTPRI == -6 */

/*
 *  CPUロック状態での割込み優先度マスクの内部表現
 */
#define IIPM_LOCK		INT_IPM(TIPM_LOCK)

/*
 *  TIPM_ENAALL（割込み優先度マスク全解除）の内部表現
 */
#define IIPM_ENAALL		INT_IPM(TIPM_ENAALL)

#ifndef TOPPERS_MACRO_ONLY

/*
 *  CPUロックフラグ実現のための変数
 *
 *  これらの変数は，CPUロック状態の時のみ書き換えてよいものとする．
 */
extern volatile bool_t		lock_flag;	/* CPUロックフラグの値を保持する変数 */
extern volatile uint16_t	saved_iipm;	/* 割込み優先度マスクを保存する変数 */

/*
 *  CPUロック状態への移行
 *
 *  IPM（ハードウェアの割込み優先度マスク）を，saved_iipmに保存し，カー
 *  ネル管理の割込みをすべてマスクする値（TIPM_LOCK）に設定する．また，
 *  lock_flagをtrueにする．
 *
 *  IPMが，最初からTIPM_LOCKと同じかそれより高い場合には，それを
 *  saved_iipmに保存するのみで，TIPM_LOCKには設定しない．これは，モデル
 *  上の割込み優先度マスクが，TIPM_LOCKと同じかそれより高いレベルに設定
 *  されている状態にあたる．
 *
 *  この関数は，CPUロック状態（lock_flagがtrueの状態）で呼ばれることは
 *  ないものと想定している．
 */
Inline void
x_lock_cpu(void)
{
	uint16_t	iipm;

	/*
	 *  current_iipm()の返り値を直接saved_iipmに保存せず，一時変数iipm
	 *  を用いているのは，current_iipm()を呼んだ直後に割込みが発生し，
	 *  起動された割込み処理でsaved_iipmが変更される可能性があるためで
	 *  ある．
	 */
	iipm = current_iipm();
#if TIPM_LOCK == -7
	disint();
#else /* TIPM_LOCK == -7 */
	if (IIPM_LOCK > iipm) {
		set_iipm(IIPM_LOCK);
	}
#endif /* TIPM_LOCK == -7 */
	saved_iipm = iipm;
	lock_flag = true;
	Asm("":::"memory");
}

#define t_lock_cpu()	x_lock_cpu()
#define i_lock_cpu()	x_lock_cpu()

/*
 *  CPUロック状態の解除
 *
 *  lock_flagをfalseにし，IPM（ハードウェアの割込み優先度マスク）を，
 *  saved_iipmに保存した値に戻す．
 *
 *  この関数は，CPUロック状態（lock_flagがtrueの状態）でのみ呼ばれるも
 *  のと想定している．
 */
Inline void
x_unlock_cpu(void)
{
	Asm("":::"memory");
	lock_flag = false;
	set_iipm(saved_iipm);
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
 *  chg_ipmで有効な割込み優先度の範囲の判定
 *
 *  TMIN_INTPRIの値によらず，chg_ipmでは，-6～TIPM_ENAALL（＝0）の範囲
 *  に設定できることとする（ターゲット定義の拡張）．
 */
#define VALID_INTPRI_CHGIPM(intpri) \
				(-6 <= (intpri) && (intpri) <= TIPM_ENAALL)

/*
 * （モデル上の）割込み優先度マスクの設定
 *
 *  CPUロックフラグがクリアされている時は，ハードウェアの割込み優先度マ
 *  スクを設定する．CPUロックフラグがセットされている時は，saved_iipm
 *  を設定し，さらに，ハードウェアの割込み優先度マスクを，設定しようと
 *  した（モデル上の）割込み優先度マスクとTIPM_LOCKの高い方に設定する．
 */
Inline void
x_set_ipm(PRI intpri)
{
	uint16_t	iipm = INT_IPM(intpri);

	if (!lock_flag) {
		set_iipm(iipm);
	}
	else {
		saved_iipm = iipm;
#if TIPM_LOCK == -7
		/*
		 *  TIPM_LOCKが-7の場合には，この時点でハードウェアの割込み優先
		 *  度マスクが必ず7に設定されているため，設定しなおす必要がない．
		 */
#else /* TIPM_LOCK == -7 */
		set_iipm(iipm > IIPM_LOCK ? iipm : IIPM_LOCK);
#endif /* TIPM_LOCK == -7 */
	}
}

#define t_set_ipm(intpri)	x_set_ipm(intpri)
#define i_set_ipm(intpri)	x_set_ipm(intpri)

/*
 * （モデル上の）割込み優先度マスクの参照
 *
 *  CPUロックフラグがクリアされている時はハードウェアの割込み優先度マ
 *  スクを，セットされている時はsaved_iipmを参照する．
 */
Inline PRI
x_get_ipm(void)
{
	uint16_t	iipm;

	if (!lock_flag) {
		iipm = current_iipm();
	}
	else {
		iipm = saved_iipm;
	}
	return(EXT_IPM(iipm));
}

#define t_get_ipm()		x_get_ipm()
#define i_get_ipm()		x_get_ipm()

/*
 *  最高優先順位タスクへのディスパッチ（prc_support.S）
 *
 *  dispatchは，タスクコンテキストから呼び出されたサービスコール処理か
 *  ら呼び出すべきもので，タスクコンテキスト・CPUロック状態・ディスパッ
 *  チ許可状態・（モデル上の）割込み優先度マスク全解除状態で呼び出さな
 *  ければならない．
 */
extern void	dispatch(void);

/*
 *  ディスパッチャの動作開始（prc_support.S）
 *
 *  start_dispatchは，カーネル起動時に呼び出すべきもので，すべての割込
 *  みを禁止した状態（割込みロック状態と同等の状態）で呼び出さなければ
 *  ならない．
 */
extern void	start_dispatch(void) NoReturn;

/*
 *  現在のコンテキストを捨ててディスパッチ（prc_support.S）
 *
 *  exit_and_dispatchは，ext_tskから呼び出すべきもので，タスクコンテキ
 *  スト・CPUロック状態・ディスパッチ許可状態・（モデル上の）割込み優先
 *  度マスク全解除状態で呼び出さなければならない．
 */
extern void	exit_and_dispatch(void) NoReturn;

/*
 *  カーネルの終了処理の呼出し（prc_support.S）
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
 */
extern void	start_r(void);

#define activate_context(p_tcb)											\
{																		\
	(p_tcb)->tskctxb.msp = (void *)((char *)((p_tcb)->p_tinib->stk)		\
										+ (p_tcb)->p_tinib->stksz);		\
	(p_tcb)->tskctxb.pc = (void *) start_r;								\
}

/*
 *  calltexは使用しない
 */
#define OMIT_CALLTEX

/*
 *  例外ベクタテーブルの構造の定義
 */
typedef struct exc_vector_entry {
	FP		exc_handler;		/* 例外ハンドラの起動番地 */
} EXCVE;

/*
 *  割込みハンドラ番号とCPU例外ハンドラ番号の範囲の判定
 */
#define VALID_INHNO_DEFINH(inhno)	((0x10U <= (inhno) && (inhno) <= 0x1fU) \
									|| (0x40U <= (inhno) && (inhno) <= 0xffU))
#define VALID_EXCNO_DEFEXC(excno)	((0x02U <= (excno) && (excno) <= 0x0fU) \
									|| (0x20U <= (excno) && (excno) <= 0x3fU))

/*
 *  割込みハンドラの設定
 *
 *  ベクトル番号inhnoの割込みハンドラの出入口処理の番地をint_entryに設
 *  定する．
 */
Inline void
x_define_inh(INHNO inhno, FP int_entry)
{
	EXCVE	*excvt;

	assert(VALID_INHNO_DEFINH(inhno));

#ifdef EXCVT_KERNEL
	/*
	 *  EXCVT_KERNELが定義されている時は，初期化処理の中でVBRを
	 *  EXCVT_KERNELに設定するので，EXCVT_KERNELを使う．
	 */
	excvt = (EXCVE *) EXCVT_KERNEL;
#else /* EXCVT_KERNEL */
	excvt = (EXCVE *) current_vbr();
#endif /* EXCVT_KERNEL */
	excvt[inhno].exc_handler = int_entry;
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
	EXCVE	*excvt;

	assert(VALID_EXCNO_DEFEXC(excno));

#ifdef EXCVT_KERNEL
	/*
	 *  EXCVT_KERNELが定義されている時は，初期化処理の中でVBRを
	 *  EXCVT_KERNELに設定するので，EXCVT_KERNELを使う．
	 */
	excvt = (EXCVE *) EXCVT_KERNEL;
#else /* EXCVT_KERNEL */
	excvt = (EXCVE *) current_vbr();
#endif /* EXCVT_KERNEL */
	excvt[excno].exc_handler = exc_entry;
}

/*
 *  割込みハンドラの出入口処理の生成
 */

/*
 *  割込みハンドラの出入口処理のラベルを作るマクロ
 */
#define INT_ENTRY(inhno, inthdr)	_kernel_##inthdr##_##inhno

/*
 *  LOG_INH_ENTERがマクロ定義されている場合に，CALL_LOG_INH_ENTERを，
 *  inhno_numをパラメータとしてlog_inh_enterを呼び出すアセンブリ言語コー
 *  ドにマクロ定義する．
 */
#ifdef LOG_INH_ENTER

#define CALL_LOG_INH_ENTER(inhno_num) \
"	move.l #" #inhno_num ", -(%sp)	\n"  /* inhno_numをパラメータに */ \
"	jsr _kernel_log_inh_enter		\n"  /* log_inh_enterを呼び出す */ \
"	addq.l #4, %sp					\n"

#else /* LOG_INH_ENTER */
#define CALL_LOG_INH_ENTER(inhno_num)
#endif /* LOG_INH_ENTER */

#ifdef LOG_INH_LEAVE

/*
 *  CALL_LOG_INH_LEAVEを，inhno_numをパラメータとしてlog_inh_leaveを呼
 *  び出すアセンブリ言語コードにマクロ定義する．
 */
#define CALL_LOG_INH_LEAVE(inhno_num) \
"	move.l #" #inhno_num ", -(%sp)	\n"  /* inhno_numをパラメータに */ \
"	jsr _kernel_log_inh_leave		\n"  /* log_inh_leaveを呼び出す */ \
"	addq.l #4, %sp					\n"

/*
 *  LOG_INH_LEAVEがマクロ定義されている場合の割込みハンドラの出入口処理．
 *  割込みハンドラをサブルーチンコールし，戻ってきたら，トレースログの
 *  取得後，ret_intに分岐する．
 */
#define INTHDR_ENTRY(inhno, inhno_num, inthdr) \
extern void _kernel_##inthdr##_##inhno(void); \
asm(".text							\n" \
"_kernel_" #inthdr "_" #inhno ":	\n" \
"	movem.l %d0-%d1/%a0-%a1, -(%sp)	\n"  /* スクラッチレジスタを保存 */ \
	CALL_LOG_INH_ENTER(inhno_num) \
"	jsr " #inthdr "					\n"  /* 割込みハンドラを呼び出す */ \
	CALL_LOG_INH_LEAVE(inhno_num) \
"	jmp _kernel_ret_int				\n");/* ret_intへ分岐 */

#else /* LOG_INH_LEAVE */

/*
 *  LOG_INH_LEAVEがマクロ定義されていない場合の割込みハンドラの出入口処
 *  理．戻り番地としてret_intをスタックに積んだ後，割込みハンドラの起動
 *  番地に分岐する．割込みハンドラからのリターンにより，ret_intへ分岐す
 *  る．
 */
#define INTHDR_ENTRY(inhno, inhno_num, inthdr) \
extern void _kernel_##inthdr##_##inhno(void); \
asm(".text							\n" \
"_kernel_" #inthdr "_" #inhno ":	\n" \
"	movem.l %d0-%d1/%a0-%a1, -(%sp)	\n"  /* スクラッチレジスタを保存 */ \
	CALL_LOG_INH_ENTER(inhno_num) \
"	move.l #_kernel_ret_int, -(%sp)	\n"  /* 戻り番地をスタックに積む */ \
"	jmp " #inthdr "					\n");/* 割込みハンドラへ分岐 */

#endif /* LOG_INH_LEAVE */

/*
 *  CPU例外ハンドラの出入口処理の生成
 *
 *  CPU例外ハンドラの番地をA1に，CPU例外ハンドラ番号をD1に入れて，
 *  exchdr_entryに分岐する．割込みハンドラの出入口処理と同様に，CPU例外
 *  ハンドラ毎にCPU例外ハンドラを呼び出す処理を展開する方法もあるが，展
 *  開する処理内容が複雑であるため，採用していない．
 */

/*
 *  CPU例外ハンドラの出入口処理のラベルを作るマクロ
 */
#define EXC_ENTRY(excno, exchdr)	_kernel_##exchdr##_##excno

/*
 *  CPU例外ハンドラの出入口処理
 */
#define EXCHDR_ENTRY(excno, excno_num, exchdr) \
extern void _kernel_##exchdr##_##excno(void *sp); \
asm(".text							\n" \
"_kernel_" #exchdr "_" #excno ":	\n" \
"	movem.l %d0-%d1/%a0-%a1, -(%sp)	\n"  /* スクラッチレジスタを保存 */ \
"	lea.l " #exchdr ", %a1			\n"  /* CPU例外ハンドラの番地をA1に */ \
"	move.l #" #excno_num ", %d1		\n"  /* excno_numをD1に */ \
"	jmp _kernel_exchdr_entry		\n");/* exchdr_entryに分岐 */

/*
 *  CPU例外の発生した時のコンテキストの参照
 *
 *  CPU例外の発生した時のコンテキストが，タスクコンテキストの時にfalse，
 *  そうでない時にtrueを返す．
 */
Inline bool_t
exc_sense_context(void *p_excinf)
{
	return((*((uint16_t *) p_excinf) & 0x1000U) == 0U);
}

/*
 *  CPU例外の発生した時のIPM（ハードウェアの割込み優先度マスク，内部表
 *  現）の参照
 */
Inline uint16_t
exc_get_iipm(void *p_excinf)
{
	return(*((uint16_t *) p_excinf) & 0x0700U);
}

/*
 *  CPU例外の発生した時のコンテキストと割込みのマスク状態の参照
 *
 *  CPU例外の発生した時のシステム状態が，カーネル実行中でなく，タスクコ
 *  ンテキストであり，全割込みロック状態でなく，CPUロック状態でなく，割
 *  込み優先度マスク全解除状態である時にtrue，そうでない時にfalseを返す
 *  （CPU例外がカーネル管理外の割込み処理中で発生した場合にもfalseを返
 *  す）．
 *
 *  M68040では，CPU例外の発生した時のIPM（ハードウェアの割込み優先度マ
 *  スク）がすべての割込みを許可する状態であることをチェックすることで，
 *  カーネル実行中でないこと，全割込みロック状態でないこと，CPUロック状
 *  態でないこと，割込み優先度マスク全解除状態であることの4つの条件を
 *  チェックすることができる（CPU例外が発生した時のlock_flagを参照する
 *  必要はない）．
 */
Inline bool_t
exc_sense_intmask(void *p_excinf)
{
	return(!exc_sense_context(p_excinf)
					&& exc_get_iipm(p_excinf) == IIPM_ENAALL);
}

/*
 *  プロセッサ依存の初期化
 */
extern void	prc_initialize(void);

/*
 *  プロセッサ依存の終了時処理
 */
extern void	prc_terminate(void);

#endif /* TOPPERS_MACRO_ONLY */
#endif /* TOPPERS_PRC_CONFIG_H */
