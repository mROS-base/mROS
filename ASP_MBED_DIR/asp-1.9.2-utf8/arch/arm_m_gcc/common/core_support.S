/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2008-2015 by Embedded and Real-Time Systems Laboratory
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
 *  @(#) $Id: core_support.S 2728 2015-12-30 01:46:11Z ertl-honda $
 */

/*
 *  プロセッサ依存モジュール アセンブリ言語部（ARM-M用）
 */

#define TOPPERS_MACRO_ONLY
#define UINT_C(val)		(val)		/* uint_t型の定数を作るマクロ */
#define ULONG_C(val)	(val)		/* ulong_t型の定数を作るマクロ */
#define CAST(type, val)	(val)		/* 型キャストを行うマクロ */

#include "kernel_impl.h"
#include "arm_m.h"
#include "offset.h"
#include "target_asm.inc"

/*
 *  タスクディスパッチャ
 */
	ATEXT
	ABALIGN(4)
	ATHUMB(dispatch)
	AGLOBAL(dispatch)
ALABEL(dispatch)
	/*
	 *
	 *  このルーチンは，タスクコンテキスト・CPUロック状態・ディパッチ許可状態
	 *  ・（モデル上の）割込み優先度マスク全開状態で呼び出される．
	 */
	stmfd sp!,{r4-r11,lr}         /* レジスタの保存 */
#ifdef TOPPERS_FPU_CONTEXT
	/* 
	 * ディスパッチ元のタスクFPUを使用した場合はFPUレジスタを保存
	 */ 
	mrs   r0, control             /* CONTROL.FPCA == 0(FPU未使用)ならばスキップ */ 
	tst   r0, #CONTROL_FPCA
	beq   dispatch_1
	vpush {s16-s31}               /* FPUレジスタの保存 */ 
ALABEL(dispatch_1)
	push  {r0}                    /* 復帰時の判定用にCONTROLを保存 */  
#endif /* TOPPERS_FPU_CONTEXT */
	ldr   r0, =p_runtsk           /* p_runtskを読み込む */
	ldr   r1, [r0]
	str   sp, [r1,#TCB_sp]        /* タスクスタックを保存 */
	ldr   lr, =dispatch_r         /* 実行再開番地を保存 */
	str   lr, [r1,#TCB_pc]        
	b     dispatcher

	ATHUMB(dispatch_r)
	AGLOBAL(dispatch_r)
ALABEL(dispatch_r)
#ifdef TOPPERS_FPU_CONTEXT 
	/* 
	 * ディスパッチ先のタスクでFPUを使用していた場合はFPUレジスタを復帰
	 * スタックに保存したCONTROL.FPCAをチェック 
	 */ 
	pop   {r3}
	tst   r3, #CONTROL_FPCA /* CONTROL.FPCA == 0(FPU未使用)ならばスキップ */ 
	beq   dispatch_r_0
	vpop {s16-s31}          /* FPUレジスタの復帰 */ 
ALABEL(dispatch_r_0)
#endif /* TOPPERS_FPU_CONTEXT */
	ldmfd sp!,{r4 - r11,lr}       /* レジスタの復帰 */
	/*
	 * タスク例外処理ルーチンの起動
	 * dispatcherから呼び出されるため，TCBのアドレスはr1に入っている
	 */
	ldrb  r0,[r1,#TCB_enatex]
	tst   r0,#TCB_enatex_mask
	beq   dispatch_r_1            /* enatex が false ならリターン */
	ldr   r0,[r1,#TCB_texptn]     /* texptn が 0 ならリターン     */
	tst   r0,r0
	beq   dispatch_r_1            
	ldr   r1, =ipmflg             /* ipmflgが false ならリターン  */
	ldr   r0, [r1]
	tst   r0,r0
	beq   dispatch_r_1
	ldr   r0, =call_texrtn        /* タスク例外ルーチンの呼び出し */
	bx    r0
ALABEL(dispatch_r_1)              /* タスクへのcall_textnから戻る */
	bx    lr

/*
 *  CPU例外エントリ
 *
 *  割込みエントリと処理の内容は同等だが，ログの種類が異なるため，
 *  分けている．
 */
	ABALIGN(4)
	ATEXT
	ATHUMB(core_exc_entry)
	AGLOBAL(core_exc_entry)
ALABEL(core_exc_entry)
	/*
	 *  例外/割込みが発生すると，発生時にアクティブなスタックにスクラ
	 *  ッチレジスタ等が保存される．
	 *  この内容に加えて，CPU例外ハンドラへの情報として，basepri の値と，
	 *  EXC_RETURNの情報を加えて保存する．basepriの値は，CPU例外からの
	 *  リターン時に割込み優先度マスクの値を元に戻すためにも用いられる．
	 *
	 *   -----------
	 *  |  basepri  |
	 *   -----------
	 *  | EXC_RETURN|
	 *   -----------
	 *  |    R0     |  
	 *   -----------
	 *  |    R1     |
	 *   -----------
	 *  |    R2     |
	 *   -----------
	 *  |    R3     |
	 *   -----------
	 *  |    R12    |
	 *   -----------
	 *  |    LR     |
	 *   -----------
	 *  |    PC     |
	 *   -----------
	 *  |   xPSR    |
	 *   -----------
	 *
	 */

	/*
	 *  カーネル管理外の例外かチェック
	 *  カーネル内のクリティカルセクションの実行中，全割込みロック状態，
	 *  CPUロック状態，カーネル管理外の割込みハンドラ実行中のいずれかで
	 *  発生したCPU例外を，カーネル管理外のCPU例外と呼ぶ
	 *  全割込みロック状態はFAULTMASKが'1'の場合
	 *  CPUロック状態はbasepriがIIPM_LOCKかで判断する．
	 */
	mrs   r2, FAULTMASK           /* 全割込みロック状態ならカーネル管理外例外処理へ */
	cbnz  r2, core_nonkernel_exc_entry

	mrs   r0, basepri             /* baepriの値を取得 */
	cmp   r0, #IIPM_LOCK          /* CPUロック状態ならカーネル管理外例外処理へ */
	beq   core_nonkernel_exc_entry

	/*
	 * スタックを変更する必要があるかチェック
	 * EXC_RETURN（割込み時にLRに設定される値）をチェックして，例外発生時に
	 * アクティブなスタックを特定することで多重割込みか判定する．
	 */
	tst   lr, #EXC_RETURN_PSP    /* 割込み元がMSPなら多重割込み */
	beq   core_exc_entry_1       /* 多重割込みならcore_exc_entry_1へ */
	mrs   r3, psp                /* 一段目の割込みの場合はPSP上に */
	stmfd r3!, {r0, lr}          /* 割込み発生時の割込み優先度マスク(r0)，EXC_RETURN(lr)の順にPSP上に積む */
	msr   psp, r3                
	push  {r0,lr}                /* 割込み発生時の割込み優先度マスク(r0)，EXC_RETURN(lr)の順にMSP上に積む */
	mov   r0, r3                 /* CPU例外ハンドラへの引数となる */
	b     core_exc_entry_2
ALABEL(core_exc_entry_1)         /* 多重割込みの場合 */
	push  {r0,lr}                /* 割込み発生時の割込み優先度マスク(r0)，EXC_RETURN(lr)の順にPSP上に積む */
	mov   r0, sp                 /* CPU例外ハンドラへの引数となる */

	/*
	 *  共通処理
	 */
ALABEL(core_exc_entry_2)
	mrs   r3, ipsr               /* ハンドラアドレスを取得 */
	ldr   r1, =_kernel_exc_tbl
	ldr   r2, [r1, r3, lsl #2]

#ifdef LOG_EXC_ENTER
	push  {r0,r2,r3}
	mov   r0, r3                 /* 例外番号をパラメータに  */
	bl    log_exc_enter          /* log_exc_enterを呼び出す */
	pop   {r0,r2,r3}
	push  {r3}                   /* 例外番号をスタックへ    */
#endif /* LOG_EXC_ENTER */

	/*
	 *  CPU例外ハンドラの呼び出し
	 */
	blx   r2

#ifdef LOG_EXC_ENTER
	pop   {r0}                   /* 例外番号を引数に        */
	bl    log_exc_leave          /* log_exc_leaveを呼び出す */
#endif /* LOG_EXC_ENTER */

	b     ret_exc

/*
 *  カーネル管理外のCPU例外の出入口処理
 */
ALABEL(core_nonkernel_exc_entry)
	tst   lr, #EXC_RETURN_PSP    /* 割込み元がMSPなら多重割込み */
	beq   core_nonkernel_exc_entry_1  /* 多重割込みなら */
	mrs   r3, psp                /* 一段目の割込みの場合はPSP上に */
	stmfd r3!, {r0, lr}          /* 割込み発生時の割込み優先度マスク(r0)，EXC_RETURN(lr)の順にPSP上に積む */
	msr   psp, r3                
	push  {r0,lr}                /* 割込み発生時の割込み優先度マスク(r0)，EXC_RETURN(lr)の順にMSP上に積む */
	mov   r0, r3                 /* CPU例外ハンドラへの引数となる */
	b     core_nonkernel_exc_entry_2
ALABEL(core_nonkernel_exc_entry_1) /* 多重割込みの場合 */
	push  {r0,lr}                /* 割込み発生時の割込み優先度マスク(r0)，EXC_RETURN(lr)の順にPSP上に積む */
	mov   r0, sp                 /* CPU例外ハンドラへの引数となる */

ALABEL(core_nonkernel_exc_entry_2)
	mrs   r3, ipsr               /* CPU例外ハンドラのアドレスを取得 */
	ldr   r1, =_kernel_exc_tbl
	ldr   r2, [r1, r3, lsl #2]

	/*
	 *  CPU例外ハンドラの呼び出し
	 */
	blx   r2

	/*
	 *  割込みロック状態とする．
	 */
	cpsid f

	/*
	 *  戻り先のコンテキストの判定
	 * 
	 *  割込みハンドラ実行にLRにセットされるEXC_RETURNをチェックして，戻り
	 *  先でMSPが使われていれば，割込み先が非タスクコンテキストと判定する．
	 */
	pop   {r1,r3}                  /* 元の割込み優先度マスク(basepri)をr1へ，EXC_RETURNをr3へ */
	tst   r3, #EXC_RETURN_PSP      /* 戻り先がPSPなら              */
	bne   core_nonkernel_ret_exc_1
	b     core_nonkernel_ret_exc_2 /* の値をMSPから取得 */

ALABEL(core_nonkernel_ret_exc_1)
	/*
	 *  PSP上から，EXC_RETURN(r0)と元の割込み優先度マスク(basepri)分を削除
	 */
	mrs   r2, psp
	add   r2, r2, #(0x04*2)
	msr   psp, r2

ALABEL(core_nonkernel_ret_exc_2)
	msr   basepri, r1             /* 割込み優先度マスクを割込み前に状態へ */
	bx    r3                      /* リターン */

/*
 *  割込みエントリ
 */
	ATHUMB(core_int_entry)
	AGLOBAL(core_int_entry)
ALABEL(core_int_entry)
	/*
	 *  割込み発生時の割込み優先度マスクをスタックに保存するため取得
	 */
	mrs   r0, basepri             /* baepriの値を取得 */

	/*
	 * 多重割込みかチェック
	 * EXC_RETURN（割込み時にLRに設定される値）をチェックして，例外発生時に
	 * アクティブなスタックを特定することで多重割込みか判定する．
	 */
	tst   lr, #EXC_RETURN_PSP    /* 割込み元がMSPなら多重割込み */
	beq   core_int_entry_1       /* 多重割込みならcore_int_entry_1へ */
	mrs   r3, psp                /* 一段目の割込みの場合はPSP上に */
	stmfd r3!, {r0, lr}          /* 割込み発生時の割込み優先度マスク(r0)，EXC_RETURN(lr)の順にPSP上に積む */
	msr   psp, r3                
	push  {r0,lr}                /* 割込み発生時の割込み優先度マスク(r0)，EXC_RETURN(lr)の順にMSP上に積む */
	mov   r0, r3                 /* 未定義の割込みが発生した場合の情報とする */
	b     core_int_entry_2
ALABEL(core_int_entry_1)         /* 多重割込みの場合 */
	push  {r0,lr}                /* 割込み発生時の割込み優先度マスク(r0)，EXC_RETURN(lr)の順にPSP上に積む */
	mov   r0, sp                 /* 未定義の割込みが発生した場合の情報とする */

	/*
	 *  共通処理
	 */
ALABEL(core_int_entry_2)
	mrs   r3, ipsr               /* ハンドラアドレスを取得 */
	ldr   r1, =_kernel_exc_tbl
	ldr   r2, [r1, r3, lsl #2]

	/*
	 *  basepriの設定
	 *  NVIC優先度マスクが自動的に設定されるため優先度マスクの点では必要な
	 *  いが，x_get_ipm()がbasepriを参照するため，basepriも更新する．
	 */
	ldr   r1, =_kernel_int_iipm_tbl
	ldr   lr, [r1, r3, lsl #2]
	msr   basepri, lr

#ifdef LOG_INH_ENTER
	push  {r0,r2,r3}
	mov   r0, r3                 /* 例外番号をパラメータに  */
	bl    log_inh_enter          /* log_exc_enterを呼び出す */
	pop   {r0,r2,r3}
#endif /* LOG_EXC_ENTER */

#ifdef LOG_INH_LEAVE
	push  { r3 }                   /* 例外番号をスタックへ    */
#endif /* LOG_INT_LEAVE */

	/*
	 *  割込みハンドラの呼び出し
	 */
	blx   r2

#ifdef LOG_INH_LEAVE
	pop   {r0}                   /* 例外番号を引数に        */
	bl    log_exc_leave          /* log_exc_leaveを呼び出す */
#endif /* LOG_INH_LEAVE */

/*
 *  割込み/例外出口
 *
 *  ret_exc/ret_intは，CPU例外/割込みハンドラから戻った直後に実行する
 *  ルーチンである．
 */
ALABEL(ret_exc)
ALABEL(ret_int)
	/*
	 *  割込みロック状態とする．この時点では，CPUロック状態にはならない
	 * （basepriとlock_flagとsaved_iipmは更新しない）．
	 *
	 *  割込みロック状態とするのは，戻り先のコンテキストのチェックと，
	 *  戻り先が非タスクコンテキストであった場合のリターンをアトミック
	 *  に行うためである．bsepriをCPUロックの値にすることでもアトミッ
	 *  クなチェックと復帰は可能であるが，割込みからリターンしても，
	 *  basepri の設定内容は元に戻らないため，使用することができない． 
	 *  一方，FAULTMASKは，割込みからのリターン処理によって，'0'にクリ
	 *  アされる．
	 */
	cpsid f

	/*
	 *  戻り先のコンテキストの判定
	 * 
	 *  割込みハンドラ実行にLRにセットされるEXC_RETURNをチェックして，戻り
	 *  先でMSPが使われていれば，割込み先が非タスクコンテキストと判定する．
	 */
	pop   {r1,r3}                  /* 元の割込み優先度マスク(basepri)をr1へ，EXC_RETURNをr3へ */
	tst   r3, #EXC_RETURN_PSP      /* 戻り先がPSPなら ret_int_1 へ */
	bne   ret_int_1
	b     ret_int_2                /* の値をMSPから取得 */

	/*
	 *  一段目の割込みの出口処理
	 */
ALABEL(ret_int_1)
	/*
	 *  reqflgをチェックする
	 * 
	 *  カーネル管理内の割込みは禁止した状態で実行する必要があるため，
	 *  FAULTMASKを'1'にした状態で実行する．
	 *  reqflgをチェックする前に割込みを禁止するのは，reqflgをチェック
	 *  した直後に割込みハンドラが起動され，その中でディスパッチが要求
	 *  された場合に，すぐにディスパッチされないという問題が生じるため
	 *  である．
	 */
	ldr   r0, =reqflg             /* reqflgがfalseならそのまま戻る */
	ldr   r2, [r0]
	cbnz  r2, ret_int_3           /* trueならret_int_3へ           */

	/*
	 *  PSP上から，EXC_RETURN(r0)と元の割込み優先度マスク(basepri)分を削除
	 */
	mrs   r2, psp
	add   r2, r2, #(0x04*2)
	msr   psp, r2

ALABEL(ret_int_2)
	/*
	 *  ここには割込みロック状態（FAULTMASKがセット）された状態で来る．
	 *  Threadモードからのリターンにより自動的に割込みロック解除状態になる．
	 *  割込み優先度マスクは割込み前に状態に戻す．
	 */ 
	msr   basepri, r1             /* 割込み優先度マスクを割込み前の状態へ */
	bx    r3                      /* リターン */

ALABEL(ret_int_3)
	/*
	 *  ここでは，戻り先がタスクであり，PSP上にスクラッチレジスタと割
	 *  込み優先度マスク(basepri)が保存された状態になっている．また，
	 *  プロセッサは，Handlerモード・割込みロック状態となっている．
	 *  また，r0には，reqflgのアドレス，r3には割込み受付時のlrの値が保
	 *  持されている．
	 */
	/*
	 *  タスク例外ハンドラやディスパッチをする際にThreadモードへ遷移する
	 *  ダミーのスタックフレームを作成して，bx命令でHandlerモードからリ
	 *  ターンする．また，遅延ディスパッチする場合も，再び割り込んだタス
	 *  クに戻る際には，svc命令で，svc_handlerを呼び出す．
	 *  スタックフレームは，Configureation and Control Register(CCR)の
	 *  STKALIGNが'1'の場合は，8byte境界にアラインされる．
	 *  参考 : DDI0403B_arm_architecture_v7m_reference_manual(P.220)
	 *  そのため，この時点のスタックは割込みや例外発生時に作成された
	 *  スタックフレームから，8byte境界のサイズにしておくと，svc_handler
	 *  等でスタックフレームのアライメントの有無の確認を省略できる．
	 *  ただし，システム起動後は，動的にCCRのSTKALIGNの設定を変更するのは
	 *  禁止とする．
	 *  この時点は標準のスタックフレームは，割込み・例外発生時と同等であ
	 *  るため，タスクスタック(PSP)は8byte境界になっている．
	 */
	mov   r1, #0                   /* reqflgをfalseに */
	str   r1, [r0]

	/*
	 *  CPUロック状態に移行する．
	 *
	 *  カーネルの管理内の割込みを禁止するようにbasepriを設定し，
	 *  lock_flag と saved_iipm を更新する．saved_iipmは，戻り先の割込み
	 *  優先度マスク（の内部表現）に設定する．
	 *  この時点でCPUロック状態とするのは，dispatcherへ分岐する時と，
	 *  call_texrtnを呼び出す時に，CPUロック状態になっている必要がある
	 *  ためである．
	 *  なお，この処理の後，Threadモードへの移行処理を行なうため，割込み
	 *  ロック状態(FAULTMASKを"1")は保持する．
	 */
	ldr   r1, =IIPM_LOCK          /* CPUロック状態 */ 
	msr   basepri, r1          
	mov   r1, #0x01               /* lock_flag を trueに */
	ldr   r0, =lock_flag          
	str   r1, [r0]

	/*
	 *  割込み優先度マスクを，全解除状態（TIPM_ENAALL）に設定する
	 *  すでにCPUロック状態なので，saved_iipmをIIPM_ENAALLとする．
	 */
	ldr   r1, =IIPM_ENAALL
	ldr   r0, =saved_iipm
	str   r1, [r0]

	/*
	 *  Threadモードへ移行する．
	 *
	 *  dispatcherやcall_texrnを呼び出す場合は，Threadモードである必
	 *  要があるため，PSPスタック上にダミーの例外フレームを置いて，
	 *  擬似的に割込みハンドラからリターンする．
	 *  リターンと同時にFAULTMASKが自動的にクリアされ，カーネル管理外の
	 *  割込みが許可される．
	 */
	ldr   r0, =ret_int_4          /* PC   */
	ldr   r1, =EPSR_T             /* xPSR(Tビットが'1'である必要がある) */
	mrs   r2, psp
	stmfd r2!, {r0-r1}            /* ダミーフレームをスタック上に積む   */
	subs  r2, #(EXC_FRAME_SIZE - (4*2)) /* r0-r3,r12,lrの内容は設定する必要がない */
	msr   psp,r2       
	/* Basicフレーム，THREADモード，PSPのための EXC_RETURN */
	mov   r0, #(EXC_RETURN_TREAD_PSP|EXC_RETURN_OTHER|EXC_RETURN_FP_NONUSED)
	bx    r0                      /* Threadモードへ移行 */
    
ALABEL(ret_int_4)
	/*
	 *  上記の処理により，Threadモードで実行される．
	 *  dspflgがfalseである場合と，p_runtskとp_schedtskが同じ場合には，
	 *  ディスパッチを行わない．このチェックが必要なのは，タスク例外処
	 *  理ルーチンの呼出しが必要な場合に，ディスパッチが必要なくても，
	 *  reqflgをtrueにするためである．
	 */
	ldr   r0, =p_runtsk       /* ディスパッチを行わない場合でも，r1にp_runtsk の値(TCB) */
	ldr   r1, [r0]            /* が入っている必要があるので，先に読み込む */
	ldr   r0, =dspflg         
	ldr   r2, [r0]
	cbz   r2, ret_int_r_1     /* dspflgがfalseならret_int_r_1へ */
	ldr   r0, =p_schedtsk     
	ldr   r2, [r0]
	cmp   r1, r2              /* p_runtskとp_schedtskが同じなら */
	beq   ret_int_r_1         /*                  ret_int_r_1へ */
#ifdef TOPPERS_FPU_CONTEXT
	ldr  r3, [sp,4]          /*  ディスパッチ元のEXC_RETURNを取得 */ 
	/* 
	 * ディスパッチ元のタスクFPUを使用した場合はFPUレジスタを保存
	 */ 
	tst   r3, #EXC_RETURN_FP
	bne   ret_int_5         /* EXC_RETURN[4] == 1(FPU未使用)ならばスキップ */ 
	vpush {s16-s31}         /* FPUレジスタの保存 */ 
ALABEL(ret_int_5) 
	push {r3}               /*  戻り先のEXC_RETURNを保存 */ 
#endif /* TOPPERS_FPU_CONTEXT */ 
	stmfd sp!, {r4-r11}       /* 残りのレジスタを保存 */
	str   sp, [r1,#TCB_sp]    /* タスクスタックを保存 */
	ldr   lr, =ret_int_r      /* 実行再開番地を保存   */
	str   lr, [r1,#TCB_pc]
	b     dispatcher          /* ディスパッチャへ */

/*
 * 割込みによりプリエンプトされたタスクへのリターン処理
 *
 * Threadモードで，ディスパッチャや割込みの出口処理から呼び出される．
 * 割込みによりプリエンプトされたタスクへリターンするには，いったん
 * Handlerモードに移行し，PCに0xfffffffdを代入してリターンする必要
 * がある．そのため，SVCにより，SVCハンドラを呼び出し，Handlerモー
 * ドへ移行する．
 */
	ATHUMB(ret_int_r)
	AGLOBAL(ret_int_r)
ALABEL(ret_int_r)
	pop {r4-r11}                  /* レジスタの復帰 */
#ifdef TOPPERS_FPU_CONTEXT 
	pop {r3}     /*  戻り先のEXC_RETURNを取得 */ 
	/* 
	 * ディスパッチ元のタスクFPUを使用した場合はFPUレジスタを復帰
	 */ 
	tst r3, #EXC_RETURN_FP
	bne ret_int_r_1      /* EXC_RETURN[4] == 1(FPU未使用)ならばスキップ */ 
	vpop {s16-s31}       /* FPUレジスタの復帰 */ 
#endif /* TOPPERS_FPU_CONTEXT */ 
ALABEL(ret_int_r_1)
	/*
	 *  enatexがtrueで，texptnが0でなければ，タスク例外処理ルーチンを
	 *  呼び出す．
	 *  dispatcherから呼び出されるため，TCBのアドレスはr1に入っている
	 */
	ldrb  r0, [r1,#TCB_enatex]
	tst   r0, #TCB_enatex_mask
	beq   ret_int_r_2           /* enatex が false なら ret_int_r_2へ */
	ldr   r0, [r1,#TCB_texptn]  /* texptn が 0 ならリターン     */
	cbz   r0, ret_int_r_2
	ldr   r1, =ipmflg           /* ipmflgが false ならリターン  */
	ldr   r0, [r1]
	cbz   r0, ret_int_r_2
#ifdef TOPPERS_FPU_LAZYSTACKING
	/*  
	 *  Lazy Stacking かつ 割込みの出口処理からの分岐で来た かつ
	 *  割込み元のタスクでFPUを使用 かつ 割込みハンドラでFPU未使用の場合,
	 *  (FPCCR.LSPACT == 1 の場合)
	 *  例外フレームにFPUのコンテキストが書き戻されていない.
	 *  ここでFPU命令を実行して例外フレームにFPUのコンテキストを書き戻す必要がある
	 */
	ldr r1, =(FPCCR)
	ldr r0, [r1] 
	tst r0, #FPCCR_LSPACT
	beq ret_int_r_1_1     /* FPCCR.LSPACT == 0ならばスキップ */ 
	vmov r3, s15          /* ダミー命令で例外フレームに書き戻し */ 
ALABEL(ret_int_r_1_1)
#endif /* TOPPERS_FPU_LAZYSTACKING */ 
	bl    call_texrtn           /* タスク例外ルーチンの呼び出し */
ALABEL(ret_int_r_2)
#ifdef TOPPERS_FPU_CONTEXT
	/*
	 *  SVCの実行でFPUコンテキストをスタックに積まないように
	 *  CONTROL.FPCAをクリア
	 */
	mrs   r3, control       
	bic   r3, r3, #CONTROL_FPCA
	msr   control, r3
#endif /* TOPPERS_FPU_CONTEXT */
	svc   0                     /* SVCの呼び出し */

/*
 *  SVCハンドラ
 */
	ATHUMB(svc_handler)
	AGLOBAL(svc_handler)
ALABEL(svc_handler)
	/*
	 *  割込み処理からのリターンにより，CPUロック解除状態に移行するよ
	 *  う準備する．
	 */
	cpsid f                       /* 割込みロック状態へ */
	mrs   r0, psp
	adds  r0, #EXC_FRAME_SIZE     /* スタックを捨てる   */
	ldmfd r0!,{r1,lr}             /* EXC_RETURNは，FPU使用時・未使用時で異なるためスタックから取得 */
	msr   psp, r0
	mov   r0, #0
	ldr   r1, =lock_flag          /* CPUロック解除状態へ */
	str   r0, [r1]
	ldr   r1, =IIPM_ENAALL        /* 割込み優先度マスクを全解除状態に設定 */
	msr   basepri, r1             
	bx    lr                      /* リターン     */

/*
 *  ディスパッチャの動作開始
 */
	ATHUMB(start_dispatch)
	AGLOBAL(start_dispatch)
ALABEL(start_dispatch)
	/*
	 *  このルーチンは，カーネル起動時に，すべての割込みを禁止した状態
	 * （割込みロック状態と同等）で呼び出される．また，割込みモード（非
	 *  タスクコンテキストと同等）で呼び出されることを想定している．
	 *
	 *  core_initializeで，lock_flagをtrueに，saved_iipmをIIPM_ENAALLに
	 *  初期化しているため，カーネル管理外の割込みを許可することで，
	 *  CPUロック状態・（モデル上の）割込み優先度マスク全解除状態になる．
	 *  また，task_initializeでdisdspをfalseに初期化しているため，ディ
	 *  スパッチ許可状態になっている．
	 */
	ldr   r0,=istkpt              /* MSPを初期化   */
	ldr   r1,[r0]                 /* start_dispatch呼び出し時に呼び出し用に */
	msr   msp, r1                 /* 使用しているため初期化する             */
	ldr   r1, =IIPM_LOCK          /* カーネル管理内の割込みを禁止 */
	msr   basepri, r1 
	cpsie f                       /* カーネル管理外の割込みを許可 */
	mov   r0, #CONTROL_PSP        /* PSPを有効に  */
	msr   control, r0
	isb                           /* control の操作後に必要 */

/*
 *  現在のコンテキストを捨ててディスパッチ
 */
	ATHUMB(exit_and_dispatch)
	AGLOBAL(exit_and_dispatch)
ALABEL(exit_and_dispatch)
	/* ディスパッチャ本体（dispatcher）へ */ 

/*
 *  ディスパッチャ本体
 */
ALABEL(dispatcher)
	/*
	 *  このルーチンは，タスクコンテキスト・CPUロック状態・ディスパッチ
	 *  許可状態・（モデル上の）割込み優先度マスク全解除状態で呼び出さ
	 *  れる．
	 *
	 *  すなわち，Threadモード・lock_flagがtrue・disdspがfalse・dspflg
	 *  がtrue・saved_iipmがIIPM_ENAALLとなっている．実行再開番地へもこ
	 *  の状態のまま分岐する．
	 */
#ifdef TOPPERS_FPU_CONTEXT
	mrs   r3, control       /* ディスパッチャ実行時はFPCAを一律クリアする */
	bic   r3, r3, #CONTROL_FPCA
	msr   control, r3
#endif /* TOPPERS_FPU_CONTEXT */
#ifdef LOG_DSP_ENTER
	ldr   r1, =p_runtsk     /* p_runtskをパラメータに */
	ldr   r0, [r1]        
	bl    log_dsp_enter
#endif /* LOG_DSP_ENTER */
ALABEL(dispatcher_0)
	ldr   r0, =p_schedtsk   /* p_schedtskをp_runtskに */
	ldr   r1, [r0]
	ldr   r2, =p_runtsk   
	str   r1, [r2]        
	cbz   r1, dispatcher_1  /* p_runtskがNULLならdispatcher_1へ */           
	ldr   sp, [r1,#TCB_sp]  /* タスクスタックを復帰 */
#ifdef LOG_DSP_LEAVE
	mov   r0, r1            /* p_runtskをパラメータに */
	mov   r4, r1            /* r1はスクラッチレジスタなので保存 */
	bl    log_dsp_leave
	mov   r1, r4
#endif /* LOG_DSP_LEAVE */
	ldr   pc, [r1,#TCB_pc]  /* 実行再開番地を復帰   */    
ALABEL(dispatcher_1)
	/*
	 * CPUロック状態の解除と，非タスクコンテキスト実行状態への
	 * 準備をする
	 */
	mov   r0, #CONTROL_MSP  /* MSPを有効に  */
	msr   control, r0       
	isb                     /* control の操作後に必要 */
	mov   r4, #0            /* r4 <- '0' */
	ldr   r5, =IIPM_LOCK    /* r5 <- 割込みロック状態の割込み優先度マスクの値 */
	ldr   r6, =reqflg       /* r6 <- reqflg */
	ldr   r7, =lock_flag    /* r7 <- lock_flg */
	str   r4, [r7]          /* CPUロック解除状態へ */
ALABEL(dispatcher_2)
	/*
	 *  割込みを許可し，非タスクコンテキスト実行状態とし割込みを待つ．
	 *
	 *  ここで非タスクコンテキスト実行状態に切り換えるのは，ここで発生
	 *  する割込み処理にどのスタックを使うかという問題の解決と，割込み
	 *  ハンドラ内でのタスクディスパッチの防止という2つの意味がある．
	 *
	 *  プロセッサを割込み待ちに移行させる処理と，割込み許可とは，不可
	 *  分に行なう必要がある．
	 *  これを不可分に行なわない場合，割込みを許可した直後に割込
	 *  みが入り，その中でタスクが実行可能状態になると，実行すべきタス
	 *  クがあるにもかかわらずプロセッサが割込み待ちになってしまう．
	 *  ARM-Mでは，PRIMASKをセットした状態でWFIを呼び出すことで実現できる．
	 *  この状態で割込みが入ると，割込みは実行されず，WFIからリターンす
	 *  ることになるので，一旦割込みを許可して割込みハンドラを実行する．
	 *
	 *  割込み待ちの間は，p_runtskをNULL（＝0）に設定しなければならな
	 *  い．このように設定しないと，割込みハンドラからiget_tidを呼び出
	 *  した際の動作が仕様に合致しなくなる．
	 *
	 *  ターゲットによっては，省電力モード等に移行するため，標準の方法と
	 *  異なる手順が必要な場合がある．
	 *  そのようなターゲットでは，ターゲット依存において，TOPPERS_CUSTOM_IDLE
	 *  を定義し，アセンブラマクロとして，toppers_asm_custom_idle を用意
	 *  すればよい．
	 *
	 *  なお，toppers_asm_custom_idle の記述にあたっては，次のレジスタは
	 *  toppers_asm_custom_idleの前後で使用するため，
	 *  toppers_asm_custom_idle 内で使用する場合は，前後で保存復帰すること．
	 *  これらのレジスタは Calee saved レジスタであるため， 
	 *  toppers_asm_custom_idle として関数呼び出しをした場合は，呼び出した
	 *  関数で自動的に保存復帰されるため，アセンブラレベルでの保存復帰は必
	 *  要ない．
	 *
	 *  レジスタ : 内容
	 *   r4      : '0'
	 *   r5      : 'IIPM_LOCK'
	 *   r6      : reqflgのアドレス
	 *   r7      : lock_flgのアドレス
	 *   sp      : 非タスクコンテキスト用のスタックの先頭アドレス(msp)
	 */
#ifdef TOPPERS_CUSTOM_IDLE
	toppers_asm_custom_idle
#else
	cpsid i               /* PRIMASK をセット */
	msr   basepri, r4     /* 全割込み許可 */ 
	wfi
	cpsie i               /* PRIMASK をクリア（割込みを受け付ける） */
	msr   basepri, r5     /* CPUロック状態へ */ 
#endif /* TOPPERS_CUSTOM_IDLE */

	ldr   r0, [r6]        /* reqflgがfalseならdispatcher_2へ */
	cmp   r0, #0
	beq   dispatcher_2
	str   r4, [r6]        /* reqflgをfalseに */

	/*
	 *  CPUロック状態に戻す．割込み待ちの間に実行した割込みハンドラによ
	 *  り，saved_iipmが書き換えられる可能性があるため，元の値に戻す必
	 *  要がある．dispatcherが実行される時は，saved_iipmがIIPM_ENAALL
	 *  となっているため，ここではsaved_iipmをIIPM_ENAALL（＝0）に戻せ
	 *  ばよい．
	 */
	mov   r0, #CONTROL_PSP  /* PSPを有効に  */ 
	msr   control, r0      
	isb                     /* control の操作後に必要 */
	mov   r2, #1            /* lock_flagをtrueへ */
	str   r2, [r7]
	ldr   r0, =saved_iipm   /* saved_iipm を0に */
	str   r4, [r0]
	b     dispatcher_0


/*
 *  カーネルの終了処理の呼出し
 *
 *  スタックを非タスクコンテキスト用に切り替え．
 *  
 */
	ATHUMB(call_exit_kernel)
	AGLOBAL(call_exit_kernel)
ALABEL(call_exit_kernel)
	mov   r0, #CONTROL_MSP
	msr   control, r0       /* MSPを有効に  */
	isb                     /* control の操作後に必要 */
	ldr   r0, =exit_kernel  /* カーネルの終了処理を呼ぶ */
	bx    r0


/*
 *  タスク起動処理
 *
 *  dispatcherから呼び出されるため，TCBのアドレスはr1に入っている
 *
 */ 
	ATHUMB(start_r)
	AGLOBAL(start_r)
ALABEL(start_r)
	mov   r0, #0
	ldr   r4, =lock_flag                    /* CPUロック解除状態へ */
	str   r0, [r4]
	msr   basepri, r0                       /* 割込み許可   */
	ldr   lr, =ext_tsk                      /* 戻り番地設定 */
	ldr   r2, [r1, #TCB_p_tinib]            /* p_runtsk->p_tinibをr2に  */
	ldr   r0, [r2, #TINIB_exinf]            /* exinfを引数レジスタr0に  */
	ldr   r1, [r2, #TINIB_task]             /* タスク起動番地にジャンプ */
	bx    r1

/*
 *  微少時間待ち
 */
	ABALIGN(4)
	ATEXT
	ATHUMB(sil_dly_nse)
	AGLOBAL(sil_dly_nse)
ALABEL(sil_dly_nse)
	subs  r0, r0, #SIL_DLY_TIM1
	cmp   r0, #0
	bgt   sil_dly_nse1
	bx    lr
ALABEL(sil_dly_nse1)
	subs  r0, r0, #SIL_DLY_TIM2
	cmp   r0, #0
	bgt   sil_dly_nse1
	bx    lr
