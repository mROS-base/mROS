/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2006 by GJ Business Division RICOH COMPANY,LTD. JAPAN
 *  Copyright (C) 2007-2013 by Embedded and Real-Time Systems Laboratory
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
 *  @(#) $Id: chip_config.c 2742 2016-01-09 04:25:18Z ertl-honda $
 */

/*
 * チップ依存モジュール（AT91SAM7S用）
 */

#include "kernel_impl.h"
#include <sil.h>

/*
 * 各割込みの割込み要求禁止フラグの状態
 */
uint32_t idf;

/*
 * 現在の割込み優先度マスクの値
 */
PRI ipm;

/*
 *  割込み属性が設定されているかを判別するための変数
 */
uint32_t	bitpat_cfgint;

/*
 *  ターゲット依存の初期化
 */
void
target_initialize(void)
{
    int i;
    
    /*
     *  ARM依存の初期化
     */
    core_initialize();


    /*
     * 割込み入力時に割込み要因を判定するためSVRに割込み番号をセットする
     */
    /*
     * 全割込み禁止
     */
    sil_wrw_mem((void*)(TADR_AIC_BASE+TOFF_AIC_ICCR), ~0U);

    /*
     * EOICRをセット
     */
    sil_wrw_mem((void*)(TADR_AIC_BASE+TOFF_AIC_EOICR), 0U);
    
    
    for(i = 0; i < TNUM_INT; i++){
        sil_wrw_mem((void*)(TADR_AIC_BASE+TOFF_AIC_SVR+(i*4)), (uint32_t)i);
    }
    
    /*
     * 各割込みの割込み要求禁止フラグ全禁止
     */
    idf = ~0U;

    /*
     * 割込み優先度マスクは0
     */ 
    ipm = 0U;

    /*
     * 全ての割込みをマスク
     */ 
    at91sam7s_disable_int(~0U);

    /*
     *  割込み属性が設定されているかを判別するための変数を初期化する．
     */
    bitpat_cfgint = 0U;
    
    /*
     *  target_fput_log が可能になるようにUARTを初期化
     */
    at91sam7s_init_uart();
}

/*
 *  ターゲット依存の終了処理
 */
void
target_exit(void)
{
    extern void    software_term_hook(void);
    void (*volatile fp)(void) = software_term_hook;

   /*
     *  software_term_hookへのポインタを，一旦volatile指定のあるfpに代
     *  入してから使うのは，0との比較が最適化で削除されないようにするた
     *  めである．
     */
    if (fp != 0) {
        (*fp)();
    }

    /*
     *  ARM依存の終了処理
     */
    core_terminate();

    /*
     *  すべての割込みをマスクする．
     */
    at91sam7s_disable_int(~0U);

    /*
     *  開発環境依存の終了処理
     */
    at91sam7s_exit();

    while(1);
}

/*
 *  ターゲット依存の文字出力
 */
void
target_fput_log(char c)
{
    if (c == '\n') {
        at91sam7s_putc('\r');
    }
    at91sam7s_putc(c);
}

/*
 *  割込み要求ラインの属性の設定
 *
 *  ASPカーネルでの利用を想定して，パラメータエラーはアサーションでチェッ
 *  クしている．FI4カーネルに利用する場合には，エラーを返すようにすべき
 *  であろう．
 *
 */
void
x_config_int(INTNO intno, ATR intatr, PRI intpri)
{
    assert(VALID_INTNO(intno));
    assert(TMIN_INTPRI <= intpri && intpri <= TMAX_INTPRI);

	/*
	 *  割込み属性が設定されているかを判別するための変数の設定
	 */
	bitpat_cfgint |= INTNO_BITPAT(intno);
    
    /* 
     * いったん割込みを禁止する
     */    
    x_disable_int(intno);

    /*
     * レベルトリガ/エッジトリガの設定
     * IRQのみサポートする
     */
    uint32_t smr_val;

    if((intatr & TA_POSEDGE) != 0U) {
        /*
         * ポジティブエッジ
         */
        smr_val = AIC_SRCTYPE_EXT_POSITIVE_EDGE;
    }else if((intatr & TA_HIGHLEVEL) != 0U) {
        /*
         * ハイレベルトリガ
         */
        smr_val = AIC_SRCTYPE_EXT_HIGH_LEVEL ;
    }else if((intatr & TA_EDGE) != 0U) {
        /*
         * エッジトリガ
         */
        smr_val = AIC_SRCTYPE_INT_EDGE_TRIGGERED;
    }else{
        /*
         * レベルトリガ
         */
        smr_val = AIC_SRCTYPE_INT_LEVEL_SENSITIVE;
    }
    
    sil_wrw_mem((void*)(TADR_AIC_BASE+TOFF_AIC_SMR+intno*4U), smr_val|INT_IPM(intpri));
    
    if ((intatr & TA_ENAINT) != 0U){
        (void)x_enable_int(intno);
    }    
}

#ifndef OMIT_DEFAULT_INT_HANDLER
/*
 * 未定義の割込みが入った場合の処理
 */
void
default_int_handler(void){
    syslog_0(LOG_EMERG, "Unregistered Interrupt occurs.");
    target_exit();
}
#endif /* OMIT_DEFAULT_INT_HANDLER */
