/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2008-2011,2015,2016 by Embedded and Real-Time Systems Laboratory
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
 *  @(#) $Id: core_config.h 2744 2016-01-11 01:44:30Z ertl-honda $
 */

/*
 *		プロセッサ依存モジュール（ARM-M用）
 *
 *  このインクルードファイルは，target_config.h（または，そこからインク
 *  ルードされるファイル）のみからインクルードされる．他のファイルから
 *  直接インクルードしてはならない．
 */
#ifndef TOPPERS_CORE_CONFIG_H
#define TOPPERS_CORE_CONFIG_H

/*
 *  エラーチェック方法の指定
 */
#define CHECK_STKSZ_ALIGN	8	/* スタックサイズのアライン単位 */
#define CHECK_FUNC_ALIGN	1	/* 関数のアライン単位 */
#define CHECK_FUNC_NONNULL		/* 関数の非NULLチェック */
#define CHECK_STACK_ALIGN	8	/* スタック領域のアライン単位 */
#define CHECK_STACK_NONNULL		/* スタック領域の非NULLチェック */
#define CHECK_MPF_ALIGN		4	/* 固定長メモリプール領域のアライン単位 */
#define CHECK_MPF_NONNULL		/* 固定長メモリプール領域の非NULLチェック */
#define CHECK_MB_ALIGN		4	/* 管理領域のアライン単位 */

#ifndef TOPPERS_MACRO_ONLY

/*
 *  プロセッサの特殊命令のインライン関数定義
 */
#include <core_insn.h>

/*
 *  非タスクコンテキスト用のスタック初期値
 */
#define TOPPERS_ISTKPT(istk, istksz) ((STK_T *)((uint8_t *)(istk) + (istksz)))

/*
 *  タスクコンテキストブロックの定義
 */
typedef struct task_context_block {
	void	*sp;		/* スタックポインタ */
	FP		pc;			/* プログラムカウンタ */
} TSKCTXB;

/*
 *  コンテキストの参照
 *
 */
Inline bool_t
sense_context(void)
{
	/*
	 *  PSPが有効ならタスクコンテキスト，MSPが有効なら非タスクコンテキスト
	 *  とする． 
	 */
	if ((get_control() & CONTROL_PSP) == CONTROL_PSP){
		return false;
	}
	else {
		return true;
	}
}

/*
 *  スタートアップルーチン（start.S）
 */
extern void _start(void);

/*
 *  最高優先順位タスクへのディスパッチ（core_support.S）
 *
 *  dispatchは，タスクコンテキストから呼び出されたサービスコール処理か
 *  ら呼び出すべきもので，タスクコンテキスト・CPUロック状態・ディスパッ
 *  チ許可状態・（モデル上の）割込み優先度マスク全解除状態で呼び出さな
 *  ければならない．
 */
extern void dispatch(void);

/*
 *  ディスパッチャの動作開始（core_support.S）
 *
 *  start_dispatchは，カーネル起動時に呼び出すべきもので，すべての割込
 *  みを禁止した状態（割込みロック状態と同等の状態）で呼び出さなければ
 *  ならない．
 */
extern void start_dispatch(void) NoReturn;

/*
 *  現在のコンテキストを捨ててディスパッチ（core_support.S）
 *
 *  exit_and_dispatchは，ext_tskから呼び出すべきもので，タスクコンテキ
 *  スト・CPUロック状態・ディスパッチ許可状態・（モデル上の）割込み優先
 *  度マスク全解除状態で呼び出さなければならない．
 */
extern void exit_and_dispatch(void) NoReturn;

/*
 *  カーネルの終了処理の呼出し（core_support.S）
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
extern void start_r(void);

#define activate_context(p_tcb)											\
{																		\
	(p_tcb)->tskctxb.sp = (void *)((uint8_t *)((p_tcb)->p_tinib->stk)	\
								+ (p_tcb)->p_tinib->stksz);				\
	(p_tcb)->tskctxb.pc = (FP) start_r;									\
}

/*
 *  calltexは使用しない
 */
#define OMIT_CALLTEX

/*
 *  割込み番号・割込みハンドラ番号
 *
 *  割込みハンドラ番号(inhno)と割込み番号(intno)は，割り込み発生時に
 *  IPSRに設定される例外番号とする． 
 */

/*
 *  割込み番号の範囲の判定
 */
#define VALID_INTNO(intno)           ((TMIN_INTNO <= (intno)) && ((intno) <= TMAX_INTNO))
#define VALID_INTNO_CREISR(intno)    VALID_INTNO(intno)
#define VALID_INTNO_DISINT(intno)    VALID_INTNO(intno)
#define VALID_INTNO_CFGINT(intno)    VALID_INTNO(intno)

/*
 *  割込みハンドラの設定
 *
 *  ベクトル番号inhnoの割込みハンドラの起動番地int_entryに設定する．割込み
 *  ハンドラテーブル
 */
Inline void
x_define_inh(INHNO inhno, FP int_entry)
{

}

/*
 *  割込みハンドラの出入口処理の生成マクロ
 *
 */
#define INT_ENTRY(inhno, inthdr)    inthdr
#define INTHDR_ENTRY(inhno, inhno_num, inthdr) extern void inthdr(void);

/*
 *  割込み要求ラインの属性の設定
 */
extern void x_config_int(INTNO intno, ATR intatr, PRI intpri);

/*
 *  割込みハンドラ入口で必要なIRC操作
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
 *  CPU例外エントリ（core_support.S）
 */
extern void core_exc_entry(void);

/*
 *  割込みエントリ（core_support.S）
 */
extern void core_int_entry(void);

/*
 *  プロセッサ依存の初期化
 */
extern void core_initialize(void);

/*
 *  プロセッサ依存の終了時処理
 */
extern void core_terminate(void);

/*
 * 登録されていない例外が発生すると呼び出される
 */
extern void default_exc_handler(void *p_excinf);

/*
 * 未登録の割込みが発生した場合に呼び出される
 */
extern void default_int_handler(void *p_excinf);

#endif /* TOPPERS_MACRO_ONLY */

/*
 *  ARMv7-MとARMv6-Mで異なる処理
 *  ARMv6-Mの処理はcore_config_armv6m.hに記述する
 */
#if __TARGET_ARCH_THUMB == 4

/*
 *  ARMv7-Mに関する処理
 */
  
/*
 *  ターゲット依存のオブジェクト属性
 */
#define TARGET_INHATR  TA_NONKERNEL /* ターゲット定義の割込みハンドラ属性 */

/*
 *  TOPPERS標準割込み処理モデルの実現
 *
 *  割込み優先度マスクとしては，BASEPRIを用いる．全割込みを禁止する
 *  機能として，FAULTMASKやPRIMASKがあるが，カーネル管理外の割込みを
 *  サポートするため，これらはCPUロックのために用いない．
 *  そのため，BASEPRIを用いて擬似的にCPUロックフラグを実現する．
 *
 *  まず，CPUロック状態を管理すための変数(lock_flag)を用意する．
 *
 *  CPUロックフラグがクリアされている間は，BASEPRIをモデル上の割込み
 *  優先度マスクの値に設定する．この間は，モデル上の割込み優先度マス
 *  クは，BASEPRIを用いる．
 * 
 *  それに対してCPUロックフラグがセットされいる間は，BASEPRIを，カーネ
 *  ル管理外のものを除くすべての割込み要求をマスクする値(TIPM_LOCK)と，
 *  モデル上の割込み優先度マスクとの高い方に設定する．この間のモデル上
 *  の割込み優先度マスクは，そのための変数(saved_iipm, 内部表現で保持)
 *  を用意して保持する．
 */

/*
 *  割込み優先度マスクの外部表現と内部表現の変換
 *
 *  アセンブリ言語のソースファイルからインクルードする場合のために，
 *  CASTを使用
 *  割込み優先度のビット幅(TBITW_IPRI)が 8 の場合は，内部優先度 255
 *  は，外部優先度 -1 に対応する．
 */
#define EXT_IPM(iipm)   (CAST(PRI,((iipm >> (8 - TBITW_IPRI)) - (1 << TBITW_IPRI))))       /* 内部表現を外部表現に */
#define INT_IPM(ipm)    (((1 << TBITW_IPRI) - CAST(uint8_t, -(ipm)))  << (8 - TBITW_IPRI)) /* 外部表現を内部表現に */

/*
 *  割込み優先度マスクをNVICの優先度に変換
 */
#define INT_NVIC_PRI(ipm)    INT_IPM(ipm)

/*
 *  CPUロック状態での割込み優先度マスク
 */
#define TIPM_LOCK    TMIN_INTPRI

/*
 *  CPUロック状態での割込み優先度マスクの内部表現
 *
 *  TIPM_LOCKは，CPUロック状態でのBASEPRIの値．カーネル管理外のものを
 *  除くすべての割込みをマスクする値に定義する．  
 */
#define IIPM_LOCK    INT_IPM(TIPM_LOCK)

/*
 *  TIPM_ENAALL（割込み優先度マスク全解除）の内部表現
 *
 *  BASEPRIに '0' を設定することで，全割込みを許可する．
 */
#define IIPM_ENAALL  (0)

#ifndef TOPPERS_MACRO_ONLY

/*
 *  CPUロックフラグ実現のための変数
 * 
 *  これらの変数は，CPUロック状態の時のみ書き換えてもよいとする．
 *  インライン関数中で，アクセスの順序が変化しないよう，volatile を指定． 
 */
extern volatile bool_t  lock_flag;    /* CPUロックフラグの値を保持する変数 */
extern volatile uint32_t saved_iipm;  /* 割込み優先度をマスクする変数 */

/*
 *  CPUロック状態への移行
 *
 *  BASEPRI（ハードウェアの割込み優先度マスク）を，saved_iipmに保存し，
 *  カーネル管理外のものを除くすべての割込みをマスクする値（TIPM_LOCK）
 *  に設定する．また，lock_flagをtrueにする．
 *
 *  BASEPRIが，最初からTIPM_LOCKと同じかそれより高い場合には，それを
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
	uint32_t iipm;

	/*
	 *  get_basepri()の返り値を直接saved_iipmに保存せず，一時変数iipm
	 *  を用いているのは，get_baespri()を呼んだ直後に割込みが発生し，
	 *  起動された割込み処理でsaved_iipmが変更される可能性があるためで
	 *  ある．
	 */
	iipm = get_basepri();
	/*
	 *  BASEPRIレジスタは値が小さいほど優先度が高いが，IIPM_ENAALL が
	 *  '0'であるため，単純に優先度比較だけでは不十分である．
	 */
	if ((IIPM_LOCK < iipm) || (IIPM_ENAALL == iipm)) {
		set_basepri(IIPM_LOCK);
		SCS_SYNC;
	}
	saved_iipm = iipm;
	lock_flag = true;

	/* クリティカルセクションの前後でメモリが書き換わる可能性がある */
	ARM_MEMORY_CHANGED;    
}

#define t_lock_cpu()    x_lock_cpu()
#define i_lock_cpu()    x_lock_cpu()

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
	/* クリティカルセクションの前後でメモリが書き換わる可能性がある */
	ARM_MEMORY_CHANGED;
	lock_flag = false;
	set_basepri(saved_iipm);
}

#define t_unlock_cpu()    x_unlock_cpu()
#define i_unlock_cpu()    x_unlock_cpu()

/*
 *  CPUロック状態の参照
 */
Inline bool_t
x_sense_lock(void)
{
	return(lock_flag);
}

#define t_sense_lock()    x_sense_lock()
#define i_sense_lock()    x_sense_lock()

/*
 *  chg_ipmで有効な割込み優先度の範囲の判定
 *
 *  TMIN_INTPRIの値によらず，chg_ipmでは，-(1 << TBITW_IPRI)～TIPM_ENAALL（＝0）
 *  の範囲に設定できることとする（ターゲット定義の拡張）．
 *  割込み優先度のビット幅(TBITW_IPRI)が 8 の場合は，-256 ～ 0 が指定可能である．
 *   
 */
#define VALID_INTPRI_CHGIPM(intpri) \
				((-((1 << TBITW_IPRI) - 1) <= (intpri) && (intpri) <= TIPM_ENAALL))

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
	uint8_t   iipm = INT_IPM(intpri);

	if (intpri == TIPM_ENAALL){
		iipm = IIPM_ENAALL;
	}

	if (!lock_flag) {
		set_basepri(iipm);
	}
	else {
		saved_iipm = iipm;
		/*
		 *  BASEPRIレジスタは値が小さいほど優先度が高いが，IIPM_ENAALL が
		 *  '0'であるため，単純に優先度比較だけでは不十分である．
		 */
		if ((iipm < IIPM_LOCK ) && (IIPM_ENAALL != iipm)) {
			set_basepri(iipm);
		}
		else {
			set_basepri(IIPM_LOCK);
		}
	}
	SCS_SYNC;
}

#define t_set_ipm(intpri)    x_set_ipm(intpri)
#define i_set_ipm(intpri)    x_set_ipm(intpri)

/*
 * （モデル上の）割込み優先度マスクの参照
 *
 *  CPUロックフラグがクリアされている時はハードウェアの割込み優先度マ
 *  スクを，セットされている時はsaved_iipmを参照する．
 */
Inline PRI
x_get_ipm(void)
{
	uint8_t iipm;

	if (!lock_flag) {
		iipm = get_basepri();
	}
	else {
		iipm = saved_iipm;
	}

	if (iipm == IIPM_ENAALL) {
		return(TIPM_ENAALL);
	}
	else {
		return(EXT_IPM(iipm));
	}
}

#define t_get_ipm()    x_get_ipm()
#define i_get_ipm()    x_get_ipm()

/*
 *  割込み要求禁止フラグ
 */

/*
 *  割込み属性が設定されているかを判別するための変数（kernel_cfg.c）
 */
extern const uint32_t	bitpat_cfgint[];

/*
 *  割込み要求禁止フラグのセット
 *
 *  割込み属性が設定されていない割込み要求ラインに対して割込み要求禁止
 *  フラグをクリアしようとした場合には，falseを返す．  
 */
Inline bool_t
x_disable_int(INTNO intno)
{
	uint32_t tmp;

	/*
	 *  割込み属性が設定されていない場合
	 */
	if ((bitpat_cfgint[intno >> 5] & (1 << (intno & 0x1f))) == 0x00) {
		return(false);
	}

	if (intno == IRQNO_SYSTICK) {
		tmp = sil_rew_mem((void *)SYSTIC_CONTROL_STATUS);
		tmp &= ~SYSTIC_TICINT;
		sil_wrw_mem((void *)SYSTIC_CONTROL_STATUS, tmp);
	}else {
		tmp = intno - 16;
		sil_wrw_mem((void *)((uint32_t *)NVIC_CLRENA0 + (tmp >> 5)),
					(1 << (tmp & 0x1f)));
	}
	SCS_SYNC;

	return(true);
}

#define t_disable_int(intno) x_disable_int(intno)
#define i_disable_int(intno) x_disable_int(intno)

/*
 *  割込み要求禁止フラグの解除
 *
 *  割込み属性が設定されていない割込み要求ラインに対して割込み要求禁止
 *  フラグをクリアしようとした場合には，falseを返す．
 */
Inline bool_t
x_enable_int(INTNO intno)
{
	uint32_t tmp;

	/*
	 *  割込み属性が設定されていない場合
	 */
	if ((bitpat_cfgint[intno >> 5] & (1 << (intno & 0x1f))) == 0x00) {
		return(false);
	}

	if (intno == IRQNO_SYSTICK) {
		tmp = sil_rew_mem((void *)SYSTIC_CONTROL_STATUS);
		tmp |= SYSTIC_TICINT;
		sil_wrw_mem((void *)SYSTIC_CONTROL_STATUS, tmp);
	}else {
		tmp = intno - 16;
		sil_wrw_mem((void *)((uint32_t *)NVIC_SETENA0 + (tmp >> 5)),
					(1 << (tmp & 0x1f)));
	}
	return(true);
}

#define t_enable_int(intno) x_enable_int(intno)
#define i_enable_int(intno) x_enable_int(intno)

/*
 *  SVCハンドラ（core_support.S）
 */
extern void svc_handler(void);

#endif /* TOPPERS_MACRO_ONLY */

/*
 *  FPU関連の定義
 */

/*
 *  FPCCRの初期値
 */
#if defined(TOPPERS_FPU_NO_PRESERV)
#define FPCCR_INIT FPCCR_NO_PRESERV
#elif defined(TOPPERS_FPU_NO_LAZYSTACKING)
#define FPCCR_INIT FPCCR_NO_LAZYSTACKING
#elif defined(TOPPERS_FPU_LAZYSTACKING)
#define FPCCR_INIT FPCCR_LAZYSTACKING
#endif /* defined(TOPPERS_FPU_NO_PRESERV) */

#else /* __TARGET_ARCH_THUMB == 3 */

/*
 *  ARMv6-Mに関する処理
 */
#include "core_config_v6m.h"

#endif /* __TARGET_ARCH_THUMB == 4 */

#ifndef TOPPERS_MACRO_ONLY

/*
 *  CPU例外ハンドラ関係
 */ 

/*
 *  CPU例外ハンドラ番号
 */
#define VALID_EXCNO_DEFEXC(excno)    (TMIN_EXCNO <= (excno) && (excno) <= TMAX_EXCNO)

/*
 *  CPU例外ハンドラの許可
 */
extern void enable_exc(EXCNO excno);

/*
 *  CPU例外ハンドラの禁止
 */
extern void disable_exc(EXCNO excno);

/*
 *  CPU例外ハンドラの設定
 */
Inline void
x_define_exc(EXCNO excno, FP exc_entry)
{
	/*
	 *  一部の例外は許可を行う必要がある
	 */
	enable_exc(excno);
}

/*
 *  CPU例外ハンドラの入口処理の生成マクロ
 */
#define EXC_ENTRY(excno, exchdr)    exchdr
#define EXCHDR_ENTRY(excno, excno_num, exchdr) extern void exchdr(void *p_excinf);

/*
 *  CPU例外の発生した時のコンテキストの参照
 *
 *  CPU例外の発生した時のコンテキストが，タスクコンテキストの時にfalse，
 *  そうでない時にtrueを返す．
 */
Inline bool_t
exc_sense_context(void *p_excinf)
{
	uint32_t exc_return;

	exc_return = *((uint32_t *)p_excinf + P_EXCINF_OFFSET_EXC_RETURN);
	if ((exc_return & EXC_RETURN_PSP) == EXC_RETURN_PSP){
		return false;
	}
	else {
		return true;
	}
}

/*
 *  CPU例外の発生した時のIPM（ハードウェアの割込み優先度マスク，内部表
 *  現）の参照
 */
Inline uint32_t
exc_get_iipm(void *p_excinf)
{
	return(*((uint32_t *)p_excinf + P_EXCINF_OFFSET_IIPM));
}

/*
 *  CPU例外の発生した時のコンテキストと割込みのマスク状態の参照
 *
 *  CPU例外の発生した時のシステム状態が，カーネル実行中でなく，タスクコ
 *  ンテキストであり，割込みロック状態でなく，CPUロック状態でなく，（モ
 *  デル上の）割込み優先度マスク全解除状態である時にtrue，そうでない時
 *  にfalseを返す（CPU例外がカーネル管理外の割込み処理中で発生した場合
 *  にもfalseを返す）．
 *
 *  PU例外の発生した時のBASEPRI（ハードウェアの割込み優先度マスク）
 *  がすべての割込みを許可する状態であることをチェックすることで，カー
 *  ネル実行中でないこと，割込みロック状態でないこと，CPUロック状態でな
 *  いこと，（モデル上の）割込み優先度マスク全解除状態であることの4つの
 *  条件をチェックすることができる（CPU例外が発生した時のlock_flagを参
 *  照する必要はない）．
 */
Inline bool_t
exc_sense_intmask(void *p_excinf)
{
	return(!exc_sense_context(p_excinf)
		   && (exc_get_iipm(p_excinf) == IIPM_ENAALL) && !x_sense_lock());   
}
#endif /* TOPPERS_MACRO_ONLY */

#endif /* TOPPERS_CORE_CONFIG_H */
