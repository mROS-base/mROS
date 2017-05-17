/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2006-2013 by Embedded and Real-Time Systems Laboratory
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
 *  @(#) $Id: chip_support.S 2742 2016-01-09 04:25:18Z ertl-honda $
 */

/*
 *  チップ依存モジュール アセンブリ言語部（AT91SAM7S用）
 */

#define  TOPPERS_MACRO_ONLY
#define UINT_C(val)		(val)		/* uint_t型の定数を作るマクロ */
#define ULONG_C(val)	(val)		/* ulong_t型の定数を作るマクロ */
#include "kernel_impl.h"
#include "target_asm.inc"

/*
 *  低レベルのターゲットシステム依存の初期化
 *
 *  スタートアップモジュールの中で，メモリ初期化の前に呼び出される．          
 */
        ATEXT
        AALIGN(2)
        AGLOBAL(hardware_init_hook)
ALABEL(hardware_init_hook)
        ldr     r10, =TADR_BASE_RSTC
        ldr     r0, =0xA5000c01        /* NRST enable */
        str     r0, [r10, #TOFF_RSTC_MR]
                                    /* Set up FLASH wait state */
        ldr     r10, =TADR_MC_BASE
        ldr     r0, =(50<<MC_FMR_FMCN_SHIFT) AOR MC_FMR_FWS_1FWS
        str     r0, [r10, #TOFF_MC_FMR]
                                    /* Disable Watchdog */
        ldr     r10, =TADR_WDT_BASE
        ldr     r0, =WDT_MR_WDDIS
        str     r0, [r10, #TOFF_WDT_MR]
                                    /* Enable the main oscillator */
        ldr     r10, =TADR_PMC_BASE
        ldr     r0, =(6<<CKGR_MOR_OSCOUNT_SHIFT) AOR CKGR_MOR_MOSCEN
        str     r0, [r10, #TOFF_CKGR_MOR]
                                    /* Wait for main oscillator to stabilize */
ALABEL(oscillator_wait)
        ldr     r0, [r10, #TOFF_PMC_SR]
        tst     r0, #PMC_SR_MOSCS
        beq     oscillator_wait
                                    /* Set up the PLL */
                                    /* MCK=18.432[MHz]/(DIV=14)*((MUL=72)+1)/2=48054857[Hz] */
        ldr     r0, =(14<<CKGR_PLLR_DIV_SHIFT) AOR (28<<CKGR_PLLR_PLLCOUNT_SHIFT) AOR (72<<CKGR_PLLR_MUL_SHIFT)
        str     r0, [r10, #TOFF_CKGR_PLLR]
                                    /* Wait for PLL to lock */
ALABEL(PLLlock_wait)
        ldr     r0, [r10, #TOFF_PMC_SR]
        tst     r0, #PMC_SR_LOCK
        beq     PLLlock_wait
                                    /* Select PLL as clock source */
        ldr     r0, =(PMC_MCKR_CSS_PLL_CLOCK AOR PMC_MCKR_PRES_CLK_2)
        str     r0, [r10, #TOFF_PMC_MCKR]
                                    /* Setup the stack for each mode */
        mov     r0, sp
        msr     cpsr_c, #(CPSR_FIQ AOR CPSR_IRQ_BIT AOR CPSR_FIQ_BIT)    /* Set up Fast Interrupt Mode and set FIQ Mode Stack */
        ldr     r8, =TADR_AIC_BASE                      /* Init the FIQ register */

        msr     cpsr_c, #(CPSR_IRQ AOR CPSR_IRQ_BIT AOR CPSR_FIQ_BIT)    /* Set up Interrupt Mode and set IRQ Mode Stack */
        mov     r13, r0                                 /* Init stack IRQ */

        msr     cpsr_c, #(CPSR_SVC AOR CPSR_IRQ_BIT AOR CPSR_FIQ_BIT)   /* Return Supervisor Mode and set Supervisor Mode */

ALABEL(init_done)
        bx      lr

        AALIGN(2)
        AGLOBAL(software_init_hook)
ALABEL(software_init_hook)
        bx      lr

        AALIGN(2)
        AGLOBAL(software_term_hook)
ALABEL(software_term_hook)
        bx      lr


/*
 * 割込みハンドラ
 *
 * IRQ 例外ベクタから呼び出される
 */
    AALIGN(2)
    AGLOBAL(irq_handler)
ALABEL(irq_handler)
    /* 
     * IRQモードで実行される
     */
    /* 
     *  割込み前のモード(スーパーバイザーモード)へ
     *  移行し，コンテキストを保存する
     */
    msr   cpsr_cxsf, #(CPSR_SVC AOR CPSR_IRQ_BIT) 
    stmfd sp!, {r0-r3, ip, lr, pc} /* pcはダミー */

    /*
     * spsrと戻り番地を取得するためにIRQモードへ
     */
    msr   cpsr_cxsf, #(CPSR_IRQ AOR CPSR_IRQ_BIT)
    sub   r0, lr, #4
    mrs   r1, spsr

    /*
     *  割込みハンドラ実行時のモード（スーパーバイザーモード）に
     */
    msr   cpsr_cxsf, #(CPSR_SVC AOR CPSR_IRQ_BIT) 
    str   r0, [sp, #0x18] /* 戻り番地をスタックに */
    stmfd sp!, {r1}       /* spsrをスタックに保存 */
    mov   lr, sp          /* この時点のスタックを復帰のため取得 */

    /*
     *  多重割込みか判定
     */
    ldr   r2, =excpt_nest_count /* 例外・割込みネスト回数を取得 */
    ldr   r3, [r2]
    add   r0, r3, #1            /* 例外・割込みネスト回数を更新 */
    str   r0, [r2]
    cmp   r3, #0    
    bne   irq_handler_1

    /* 
     * 一段目の例外（割込み）ならスタックを変更する
     */
    ldr   r0, =_kernel_istkpt
    ldr   sp, [r0]

ALABEL(irq_handler_1)
    stmfd sp!, {lr}     /* 復帰用のスタックポインタの保存 */

    /*
     *  割り込み要因の判定
     */
    ldr   r1, =TADR_AIC_BASE
    ldr   r3, [r1, #TOFF_AIC_IVR]

    /* suprious interrupt ならリターン */
    cmp   r3, #0
    beq   ret_suprious

    /*
     *  割込み要因の割込み優先度を求め(モデル上の)割込み優先度マスクをセット
     *  する．またその際，ハンドラ実行前の(モデル上の)割込み優先度マスクを
     *  保存する．
     */
    ldr   r0, =inh_ipm_tbl      /* 割込み優先度を取得 */
    ldr   r1, [r0, r3, lsl #2]  /* r1<-割込み優先度   */
    ldr   r0, =ipm              /* 割込み発生前の割込み優先度マスクをスタックに保存 */
    ldr   r2, [r0]              
    stmfd sp!,{r2}
    str   r1, [r0]              /* (モデル上の)割込み優先度マスクをセット */

    /*
     * ハードウェア(IRC)は割込みを受け付けた後，割込みに応じて優先度マスク
     * をセットする．そのためソフトウェア的には特に処理は行う必要はない．
     * ソフトウェア的に(モデル上の)割込み優先度マスクを上げる際には，
     * 割込み優先度マスクに応じて割込み要求禁止フラグにより割込みを禁止する．
     */

    /* 
     * 割り込み許可 
     */         
    msr   cpsr_cxsf, #(CPSR_SVC)

#ifdef LOG_INH_ENTER
    stmfd sp!,{r3}       /* ログ出力のためにinhnoをスタックに保存 */
    mov   r0, r3         /* inhno をパラメータにする */
    bl    log_inh_enter  /* log_inh_enterを呼び出す */
    ldmfd sp!, {r3}      /* 割込み番号を戻す */
#endif /* LOG_INH_ENTER */

    /*
     *  割込みハンドラの起動番地を取得
     */
    ldr   r0, =inh_tbl         /* 割込みハンドラテーブルの読み出し  */
    ldr   r0, [r0, r3, lsl #2] /* r0<-割込みハンドラ            */    

    /* 
     * 割込みハンドラの呼び出し
     */
    mov   lr, pc
    bx    r0

#ifdef LOG_INH_LEAVE
    ldr   r1, =TADR_AIC_BASE
    ldr   r0, [r1, #TOFF_AIC_ISR]  /* 割込み番号の取得 */
    bl    log_inh_leave            /* log_inh_leaveを呼び出す */
#endif /* LOG_INH_LEAVE */

    /*
     * カーネル管理の割込みを禁止する
     */
    msr   cpsr_cxsf, #(CPSR_SVC AOR CPSR_IRQ_BIT)

ALABEL(irq_handler_2)
    /*
     * 割込みクリア
     */
    ldr   r3, =TADR_AIC_BASE
    mov   r0, #0
    str   r0, [r3, #TOFF_AIC_EOICR]
    
    b     target_ret_int

    /*
     *  Suprious 割り込み時のリターン処理
     */
ALABEL(ret_suprious)
    /*
     * 割込みクリア
     */
    ldr   r3, =TADR_AIC_BASE
    mov   r0, #0
    str   r0, [r3, #TOFF_AIC_EOICR]

    b     target_ret_int_1

/*
 * ターゲット依存の例外入口処理
 */
    AGLOBAL(target_exc_handler)
ALABEL(target_exc_handler)
    /*
     *  例外実行のモード（スーパーバイザーモード）に
     *  割込みロック状態，CPUロック状態はCPU例外発生時の
     *  状態を継承する．
     *  この時点のレジスタの内容は以下の通り．
     *    r0 : lr(戻り先)
     *    r1 : spsr
     *    r2 : 例外番号
     */
    msr   cpsr_cxsf, #(CPSR_SVC AOR CPSR_FIQ_BIT AOR CPSR_IRQ_BIT) 
    str   r0, [sp, #0x18] /* 戻り番地をスタックに */
    stmfd sp!, {r1}       /* spsrをスタックに保存 */    
    mov   lr, sp          /* この時点のスタックを復帰のため取得 */

    ldr   r0, =ipm        /* 割込み発生前の割込み優先度マスクをスタックに保存 */
    ldr   r3, [r0]              
    stmfd sp!, {r3}

    /* 
     * コンテキスト判定のため，excpt_nest_count をスタックに保存．
     * スタックに保存せず，現在のexcpt_nest_countを-1すると取得できるが，
     * スタックに積んでおいた方がデバッグ等が行いやすいので，スタックに
     * 保存する．
     */
    ldr   r0, =excpt_nest_count
    ldr   r3, [r0]
    stmfd sp!, {r3}

    mov   r3,  sp         /* 例外フレーム番地を保存 */

    /*
     * カーネル管理外のCPU例外か判定する
     * 
     * カーネル管理外のCPU例外は，カーネル実行中，全割込みロック状態，
     * CPUロック状態，カーネル管理外の割込みハンドラ実行中に発生した
     * CPU例外である．ARMの場合は，SPSRのI/Fフラグのいずれかが'1'なら，
     * 該当する．
     */
    tst   r1, #(CPSR_FIQ_BIT AOR CPSR_IRQ_BIT) 
    bne   target_kernel_unc_exc_handler /* カーネル管理外のCPU例外の処理へ */

    /*
     * 割込み発生時のコンテキストを判定
     */
    ldr   r0, =excpt_nest_count
    ldr   r1, [r0]
    add   r1, r1, #1
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
    stmfd sp!, {lr}     /* 復帰用のスタックポインタの保存 */

    /* 
     * 割込み発生前の割込み優先度マスクをスタックに保存 
     * 割込みハンドラと出口ルーチンを共有するために保存
     */
    ldr   r0, =ipm       
    ldr   r1, [r0]              
    stmfd sp!, {r1}

    /*
     *  CPU例外ハンドラの起動番地を取得
     */
    ldr   r0, =exch_tbl        /* 割込みハンドラテーブルの読み出し */
    ldr   r1, [r0, r2, lsl #2] /* r1<-例外ハンドラ                 */

    stmfd sp!,{r2}      /* excno を保存 */

    /* 
     * 割り込み許可
     * CPU管理外の例外ハンドラは別ルーチンで呼び出すため，単純にSVCモード
     * にするだけでよい．
     */
    msr   cpsr_cxsf, #(CPSR_SVC)

#ifdef LOG_EXC_ENTER
    stmfd sp!,{r1,r3}
    mov   r0, r2         /* excno をパラメータに    */
    bl    log_exc_enter  /* log_exc_enterを呼び出す */
    ldmfd sp!,{r1,r3}
#endif /* LOG_EXC_ENTER */

    /* 
     * CPU例外ハンドラの呼び出し
     * 例外フレームの先頭を引数として渡す
     */        
    mov   r0, r3
    mov   lr, pc
    bx    r1

    ldmfd sp!,{r0}       /* excno を復帰  */    
#ifdef LOG_EXC_LEAVE
    bl    log_exc_leave  /* log_exc_leaveを呼び出す */
#endif /* LOG_EXC_LEAVE */

/*
 * ターゲット依存の例外・割込みの出口処理
 *
 */

    AGLOBAL(target_ret_exc)
ALABEL(target_ret_exc)
    /*
     * カーネル管理の割込みを禁止する
     */
    msr   cpsr_cxsf, #(CPSR_SVC AOR CPSR_IRQ_BIT)

    AGLOBAL(target_ret_int)
ALABEL(target_ret_int)
    /*
     * 割込み優先度マスクを元に戻す
     */
    ldmfd sp!, {r1}             /* 元の割込み優先度マスクを取得 */
    ldr   r0, =ipm              /* 割込み優先度マスクを復帰     */
    str   r1, [r0]
    rsb   r1, r1, #0            /* インデックスとするため反転   */              
    ldr   r0, =ipm_mask_tbl     /* (モデル上)の割込み優先度のを実現するための */
    ldr   r2, [r0, r1, lsl #2]  /* 割込み要求禁止フラグを取得 */    
    ldr   r0, =idf              /* 各割込みの割込み要求禁止フラグの状態を取得 */
    ldr   r1, [r0]

    /*
     * AICの割込み優先度マスク(AIC_IMR)を設定する
     *
     * 各割込みの割込み要求禁止フラグの状態と(モデル上)の割込み優先度を実現する
     * ための割込み要求禁止フラグの状態のORをとり，それの否定を求めることにより，
     * AICの割込み許可レジスタへの設定値を生成し設定する．
     *
     * 割込みの場合，割込みを受け付けた際に，AICで内部優先度を自動的に変更する
     * ため，割込みの入口では，AIC_IMRを変更していない．irq_handler_2 で，
     * AIC_EOICRを書き込むことで，内部優先度は割込み前に戻る．
     * 一方，割込みハンドラ内で，set_ipm()を実施すると，AIC_IMRの値が書き換えら
     * れるため，ここで再設定する．
     */
    ldr   r0, =TADR_AIC_BASE    /* いったん全割込みの割込み要求をマスクする */
    mvn   lr, #0                
    str   lr, [r0, #TOFF_AIC_IDCR]                                
    orr   r1, r1, r2            /* マスク指定されていない割込みの許可 */
    mvn   r1, r1                /* 設定値を生成 */    
    str   r1, [r0, #TOFF_AIC_IECR]

ALABEL(target_ret_int_1)
    /*
     * スタックポインタの復帰
     */
    ldmfd sp!, {r2}             /* 元のスタックポインタを取得 */
    mov   sp, r2
    
    /*
     * 後の処理はARM依存部で実行
     */            
    b     ret_int

/*
 *
 * カーネル管理外のCPU例外の出入口処理
 *
 */
ALABEL(target_kernel_unc_exc_handler)
    /*
     * 割込み発生時のコンテキストを判定
     */
    ldr   r0, =excpt_nest_count
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
    stmfd sp!, {lr}     /* 復帰用のスタックポインタの保存 */
    
    /*
     *  CPU例外ハンドラの起動番地を取得
     */
    ldr   r0, =exch_tbl        /* 割込みハンドラテーブルの読み出し */
    ldr   r1, [r0, r2, lsl #2] /* r1<-例外ハンドラ                 */

   /*
    * システム状態（コンテキストは除く）を，CPU例外発生時の状態へ
    */
    ldr   r0, [lr]             /* CPU例外発生前のCPSRの取得 */
    and   r0, r0, #(CPSR_IRQ_BIT AOR CPSR_FIQ_BIT)
    orr   r0, r0, #(CPSR_SVC)
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
    ldr   r0, =excpt_nest_count   /* r0 <-excpt_nest_count */
    ldr   r1, [r0]
    sub   r2, r1, #1
    str   r2, [r0]

    /*
     * スタックポインタの復帰
     */
    ldmfd sp!, {r2}             /* 元のスタックポインタを取得 */
    mov   sp, r2

    /*
     * CPU例外からの復帰
     */
    ldmfd sp!,{r1}              /* CPSRの復帰処理 */
    msr   spsr_cxsf, r1         /* 戻り先のcpsrをspsrに設定 */
    ldmfd sp!,{r0-r3,ip,lr,pc}^ /* コンテキストの復帰，^付きなので、cpsr <- spsr */
