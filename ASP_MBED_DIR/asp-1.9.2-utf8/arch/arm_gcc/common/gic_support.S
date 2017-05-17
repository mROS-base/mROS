/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2006-2012 by Embedded and Real-Time Systems Laboratory
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
 *  @(#) $Id: gic_support.S 2742 2016-01-09 04:25:18Z ertl-honda $
 */

/*
 *  GICドライバ  アセンブリ言語部
 */

#define  TOPPERS_MACRO_ONLY
#define UINT_C(val)		(val)		/* uint_t型の定数を作るマクロ */
#define ULONG_C(val)	(val)		/* ulong_t型の定数を作るマクロ */
#include "kernel_impl.h"
#include "target_asm.inc" 

#if  (__TARGET_ARCH_ARM < 6)
#error gic_suport.S supports ARMv6 or later.
#endif /*  (__TARGET_ARCH_ARM < 6) */

/*
 * 割込みハンドラ
 */
	ATEXT
	AALIGN(2)
	AGLOBAL(irq_handler)
ALABEL(irq_handler)	
	/* 
	 * IRQモードで実行される
	 */
	sub   lr, lr, #4 /* 戻り先アドレス  */
	
	/* 
	 * lr(戻り先) spsr(cpsr_svc)をスーパーバイザーモードのスタックへ
	 * 保存．
	 */
	srsdb #CPSR_SVC! 

	/*
	 *  割込み前のモード(スーパーバイザーモード)へ
	 *  移行し，コンテキストを保存する
	 */
	msr   cpsr_cxsf, #(CPSR_SVC AOR CPSR_CPULOCK AOR CPSR_ALWAYS_SET)
	stmfd sp!, {r0-r3,ip,lr}

	/*
	 *  割り込み要因の判定．
	 */
	ldr   r2, =(GICC_IAR)
	ldr   r12, [r2]
	mov   r1, r12, lsl #22  /* 下位10ビット以外をクリア */
	mov   r1, r1, lsr #22

	/*
	 *  spurious割込み(INT_ID=1023(0x03ff))の場合，割込み処理を終了 
	 */
	ldr   r2, =(0x03ff)
	cmp   r2, r1
	ldreq r2,=GICC_EOIR
	streq r12,[r2]                /* EOIレジスタへの書き込み */
	beq   ret_int_1

#ifdef TOPPERS_SAFEG_SECURE
	/*
	 *  割込みグループをサポートしている場合は(INT_ID=1022(0x03fe))もspurious割込み
	 */
	ldr   r2, =(0x03fe)
	cmp   r2, r1
	bne   irq_handler_0
	bl    check_spurious_1022
	ldr   r2, =(0x03fe)
	cmp   r2, r0
	beq   ret_int_1
	mov   ip, r0
	mov   r1, r0

irq_handler_0:
#endif /* TOPPERS_SAFEG_SECURE */
	mov   lr, sp       /* この時点のスタックを復帰のため取得 */

	/*
	 *  多重割込みか判定
	 */
	ldr   r2, =excpt_nest_count /* 例外・割込みネスト回数を取得 */
	ldr   r3, [r2]
	add   r0,r3,#1              /* 例外・割込みネスト回数を更新 */
	str   r0, [r2]
	cmp   r3, #0
	bne   irq_handler_1

	/* 
	 * 一段目の例外（割込み）ならスタックを変更する
	 */
	ldr  r0, =_kernel_istkpt
	ldr  sp, [r0]

ALABEL(irq_handler_1)
	stmfd sp!, {lr}  /* 復帰用のスタックポインタの保存 */

	/*
	 *  割込みハンドラのアドレスを取得
	 */
	ldr   r2, =inh_tbl           /* 割込みハンドラテーブルの読み込み */
	ldr   r0, [r2, r1, lsl #2]   /* r0<-割込みハンドラ */

	/*
	 *  割込み要因の割込み優先度を求め(モデル上の)割込み優先度マスクをセット
	 *  する．またその際，ハンドラ実行前の(モデル上の)割込み優先度マスクを
	 *  保存する．
	 */
	ldr   lr, =(GICC_RPR)
	ldr   r2, [lr]
	ldr   r3, =(GICC_PMR) /* 割込み発生前の割込み優先度マスクを取得 */
	ldr   lr, [r3]   
	str   r2, [r3]   /* 新しい割込み優先度マスクをセットする */
	stmfd sp!, {lr}  /* 割込み発生前の割込み優先度マスクをスタックに保存 */

	stmfd sp!,{r12}   /* GICC_IAR から取得した内容を保存  */
	stmfd sp!,{r1}   /* inhno を保存  */

#ifdef TOPPERS_SAFEG_LOAD
	stmfd sp!,{r0}
	bl safeg_load_int
	ldmfd sp!,{r0}
#endif /* TOPPERS_SAFEG_LOAD */

	/* 
	 * 割り込み許可 
	 */
	msr   cpsr_cxsf, #(CPSR_SVC AOR CPSR_ALWAYS_SET)
							
#ifdef LOG_INH_ENTER
	stmfd sp!,{r0}
	mov   r0, r1         /* inhno をパラメータに */
	bl    log_inh_enter  /* log_inh_enterを呼び出す */
	ldmfd sp!,{r0}
#endif /* LOG_INH_ENTER */

	/* 
	 * 割込みハンドラの呼び出し
	 */
	mov   lr,  pc
	bx    r0

	ldmfd sp!, {r0}     /* inhno を復帰  */
	ldmfd sp!, {r12}     /* GICC_IAR から取得した内容を保存  */

	ldr   r1, =(GICC_EOIR)  /* EOIレジスタへの書き込み */
	str   r12, [r1]

#ifdef LOG_INH_LEAVE
	bl    log_inh_leave  /* log_inh_leaveを呼び出す */
#endif /* LOG_INH_LEAVE */

	/*
	 * カーネル管理の割込みを禁止する
	 */
	msr   cpsr_cxsf, #(CPSR_SVC AOR CPSR_CPULOCK AOR CPSR_ALWAYS_SET)

	/*
	 * 割込み優先度マスクを元に戻す
	 */
	ldmfd sp!,{r1}                /* 元の割込み優先度マスクを取得 */
	ldr   r0, =(GICC_PMR)         /* GICにセット */
	str   r1, [r0]

	/*
	 * スタックポインタの復帰
	 */
	ldmfd sp!,{r2}        /* 元のスタックポインタを取得 */
	mov   sp, r2

	/*
	 * 後の処理はARM依存部で実行
	 */
	b     ret_int

/*
 * ターゲット依存の例外入口処理
 * ARMv6/7はARMv4と呼び出し時条件が異なるため注意すること．
 * スーパーバイザーモードで呼び出される．
 */
	ATEXT
	AGLOBAL(target_exc_handler)
ALABEL(target_exc_handler)
	mov   lr, sp       /* この時点のスタックを復帰のため取得 */

	ldr   r0, =(GICC_PMR) /* 例外発生前の割込み優先度マスクを取得 */
	ldr   r3, [r0]
	asr   r3, r3, #GICC_PMR_OFFSET /* 外部表現に変更 */
	sub   r3, r3, #(GICC_PMR_MASK)
	stmfd sp!, {r3}  /* 例外発生前の割込み優先度マスクをスタックに保存 */

	/* 
	 * コンテキスト判定のため，excpt_nest_count をスタックに保存．
	 * スタックに保存せず，現在のexcpt_nest_countを-1すると取得できるが，
	 * スタックに積んでおいた方がデバッグ等が行いやすいので，スタックに
	 * 保存する．
	 */
	ldr   r0, =excpt_nest_count
	ldr   r3, [r0]
	stmfd sp!, {r3}

	mov   r3,  sp   /* 例外フレーム番地を保存 */

	/*
	 * カーネル管理外のCPU例外か判定する
	 * 
	 * カーネル管理外のCPU例外は，カーネル実行中，全割込みロック状態，
	 * CPUロック状態，カーネル管理外の割込みハンドラ実行中に発生した
	 * CPU例外である．ARMの場合は，SPSRのI/Fフラグのいずれかが'1'なら，
	 * 該当する．
	 */
	ldr   r1, [sp, #(9*4)]              /* 例外フレームからspsrを取得 */
	tst   r1, #(CPSR_FIQ_BIT AOR CPSR_IRQ_BIT) 
	bne   target_kernel_unc_exc_handler /* カーネル管理外のCPU例外の処理へ */

	/*
	 *  例外発生時のコンテキストを判定
	 */
	ldr   r0, =excpt_nest_count
	ldr   r1, [r0]
	add   r1,r1,#1
	str   r1, [r0]
	cmp   r1, #1
	bne   target_exc_handler_1

	/* 
	 * タスクコンテキストでCPU例外が発生した場合
	 * スタックを非タスクコンテキストに切り替える．
	 */
	ldr  r0, =_kernel_istkpt
	ldr  sp, [r0]

ALABEL(target_exc_handler_1)
	stmfd sp!, {lr}  /* 復帰用のスタックポインタの保存 */

	/* 
	 * 割込み発生前の割込み優先度マスクをスタックに保存 
	 * 割込みハンドラと出口ルーチンを共有するために保存
	 */
	ldr   r0, =(GICC_PMR) /* 例外発生前の割込み優先度マスクを取得 */
	ldr   r1, [r0]   
	stmfd sp!, {r1}  /* 例外発生前の割込み優先度マスクをスタックに保存 */

	/*
	 *  CPU例外ハンドラの起動番地を取得
	 */
	ldr   r0, =exch_tbl        /* 割込みハンドラテーブルの読み出し */
	ldr   r1, [r0,r2,lsl #2]   /* r1<-例外ハンドラ */

	stmfd sp!,{r2}    /* excno を保存 */

	/* 
	 * 割り込み許可
	 * CPU管理外の例外ハンドラは別ルーチンで呼び出すため，単純にSVCモード
	 * にするだけでよい．
	 */
	msr cpsr_cxsf, #(CPSR_SVC AOR CPSR_ALWAYS_SET)

#ifdef LOG_EXC_ENTER
	stmfd sp!, {r1, r3}
	mov   r0, r2       /* excno をパラメータに */
	bl	log_exc_enter  /* log_exc_enterを呼び出す */
	ldmfd sp!, {r1, r3}
#endif /* LOG_EXC_ENTER */

	/* 
	 * CPU例外ハンドラの呼び出し
	 *
	 * 例外フレームの先頭を引数として渡す
	 */
	mov   r0,  r3
	mov   lr,  pc
	bx    r1

ALABEL(target_ret_exc)
	ldmfd sp!, {r0}    /* excno を復帰  */
#ifdef LOG_EXC_LEAVE
	bl	log_exc_leave  /* log_exc_leaveを呼び出す */
#endif /* LOG_EXC_LEAVE */

	/*
	 * カーネル管理の割込みを禁止する
	 */
	msr   cpsr_cxsf, #(CPSR_SVC AOR CPSR_CPULOCK AOR CPSR_ALWAYS_SET)

	/*
	 * 割込み優先度マスクを元に戻す
	 */
	ldmfd sp!,{r1}                     /* 元の割込み優先度マスクを取得 */
	ldr   r0, =(GICC_PMR)              /* GICにセット */
	str   r1, [r0]

	/*
	 * スタックポインタの復帰
	 */
	ldmfd sp!, {r2}    /* 元のスタックポインタを取得 */
	mov   sp, r2

	/*
	 * 後の処理はARM依存部で実行
	 */
	b     ret_exc

/*
 * カーネル管理外のCPU例外の出入口処理
 */
ALABEL(target_kernel_unc_exc_handler)
	/*
	 * 割込み発生時のコンテキストを判定
	 */
	ldr   r0, =excpt_nest_count /* 例外・割込みネスト回数を取得 */
	ldr   r1, [r0]
	add   r1, r1, #1
	str   r1, [r0]
	cmp   r1, #1
	bne   target_kernel_unc_exc_handler_1

	/* 
	 * タスクコンテキストでCPU例外が発生した場合
	 * スタックを非タスクコンテキストに切り替える．
	 */
	ldr  r0, =_kernel_istkpt
	ldr  sp, [r0]

ALABEL(target_kernel_unc_exc_handler_1)
	stmfd sp!, {lr}  /* 復帰用のスタックポインタの保存 */
	
	/*
	 *  CPU例外ハンドラの起動番地を取得
	 */
	ldr   r0, =exch_tbl        /* CPU例外ハンドラテーブルの読み出し */
	ldr   r1, [r0, r2, lsl #2] /* r1<-例外ハンドラ */

	/* 	
	 * システム状態（コンテキストは除く）を，CPU例外発生時の状態へ
 	 */
	ldr   r0, [r3,#(9*4)] /* CPU例外発生前のCPSRの取得 */
	and   r0, r0, #(CPSR_IRQ_BIT AOR CPSR_FIQ_BIT)
	orr   r0, r0, #(CPSR_SVC AOR CPSR_ALWAYS_SET)
	msr   cpsr_cxsf, r0

	/* 
	 * CPU例外ハンドラの呼び出し
	 * 例外フレームの先頭を引数として渡す
	 */
	mov   r0, r3
	mov   lr, pc
	bx    r1

	/*
	 *  例外・割込みのネストカウント（excpt_nest_count)のデクリメント
	 */
	ldr   r0, =excpt_nest_count /* 例外・割込みネスト回数を取得 */
	ldr   r1, [r0]
	sub   r2, r1, #1
	str   r2, [r0]

	/*
	 * スタックポインタの復帰
	 */
	ldmfd sp!, {r2}    /* 元のスタックポインタを取得 */
	mov   sp, r2

	/*
	 * CPU例外からの復帰
	 */
	ldmfd sp!,{r0-r3,ip,lr}
	rfeia sp!
