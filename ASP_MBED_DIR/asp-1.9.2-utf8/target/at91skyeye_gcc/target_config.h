/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2006 by GJ Business Division RICOH COMPANY,LTD. JAPAN
 *  Copyright (C) 2007-2016 by Embedded and Real-Time Systems Laboratory
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
 *  @(#) $Id: target_config.h 2743 2016-01-09 05:47:54Z ertl-honda $
 */

/*
 *  チップ依存モジュール（AT91SKYEYE用）
 *
 *  カーネルのチップ依存部のインクルードファイル．kernel_impl.hのター
 *  ゲット依存部の位置付けとなる．
 */

#ifndef TOPPERS_TARGET_CONFIG_H
#define TOPPERS_TARGET_CONFIG_H

/*
 *  チップシステムのハードウェア資源の定義
 */
#include "at91skyeye.h"
 
/*
 *  トレースログに関する設定
 */
#ifdef TOPPERS_ENABLE_TRACE
#include "logtrace/trace_config.h"
#endif /* TOPPERS_ENABLE_TRACE */

/*
 *  サポートしているSKYEYのバージョン
 */
#define SUPPORT_SKYEYE_VER  0x1024
#define SUPPORT_DEVM_VER    0x2020

/*
 *  ベクタールーチンをカーネルで持つかの定義
 */
#define VECTOR_KERNEL

/*
 *  ベクタールーチンを持たない場合のベクターアドレスの先頭番地
 */ 
#define VECTOR_START  0x00

/*
 *  データセクションの初期化をしない
 */
#define TOPPERS_OMIT_DATA_INIT

/*
 * 割込み待ち命令
 */
#define ASM_TARGET_WAIT_INTERRUPT nop

/*
 *  デフォルトの非タスクコンテキスト用のスタック領域の定義
 */
#define DEFAULT_ISTKSZ      0x1000U   /* 4KB */
                                                   
/*
 *  微少時間待ちのための定義（本来はSILのターゲット依存部）
 */
#define SIL_DLY_TIM1    20
#define SIL_DLY_TIM2    10

/*
 *  割込みハンドラ番号に関する定義
 */ 
#define TMIN_INHNO 0U
#define TMAX_INHNO 31U
#define TNUM_INH   32U

/*
 *  割込み番号に関する定義
 */ 
#define TMIN_INTNO 0U
#define TMAX_INTNO 31U
#define TNUM_INT   32U

#ifndef TOPPERS_MACRO_ONLY

/*
 *  割込み番号の範囲の判定
 *
 *  ビットパターンを求めるのを容易にするために，8は欠番になっている．
 *  intnoは符号無し変数なので，TMIN_INTNOが0の場合は下限チェックはしない．
 */
#if TMIN_INTNO == 0
#define VALID_INTNO(intno) ((intno) <= TMAX_INTNO)
#else /* !TMIN_INTNO == 0 */
#define VALID_INTNO(intno) (TMIN_INTNO <= (intno) && (intno) <= TMAX_INTNO)
#endif /* TMIN_INTNO == 0 */

#define VALID_INTNO_DISINT(intno)	VALID_INTNO(intno)
#define VALID_INTNO_CFGINT(intno)	VALID_INTNO(intno)
#define VALID_INTNO_ATTISR(intno)   VALID_INTNO(intno)

/*
 *  割込みハンドラの登録用テーブル
 *   実態はコンフィギュレータで生成する 
 */
extern const FP inh_tbl[TNUM_INH];

/*
 *  割込みハンドラの設定
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
 *  TOPPERS標準割込み処理モデルの実現
 */

/*
 * 各割込みの割込み要求禁止フラグの状態
 */
extern uint32_t tsim_idf;

/*
 *  割込み優先度マスク操作ライブラリ
 *
 *  AT91SAM7Sは割込み優先度マスクをIRC内でハードウェア的に持つが，
 *  ソフトウェア側から値を読み書きできないため，割込み要求禁止フラ
 *  グにより割込み優先度マスクを実現する
 */

/*
 *  現在の割込み優先度マスクの値
 */
extern PRI tsim_ipm;

/*
 *  割込み優先度マスク毎にセットする，割込み要求禁止フラグの値
 *  のテーブル
 */
extern const uint32_t ipm_mask_tbl[8];

#endif /* TOPPERS_MACRO_ONLY */  

/*
 *  IPMをimp_mask_tblのインデックスに変換するマクロ
 */
#define INDEX_IPM(ipm)  (-(ipm))

#ifndef TOPPERS_MACRO_ONLY

/*
 *  (モデル上の)割込み優先度マスクの設定
 * 
 *  指定された優先度に設定された割込み要求禁止フラグのテーブルの値と（モデ
 *  ル上の）各割込みの割込み要求禁止フラグの状態を保持した変数の値との
 *  ORをIRCの割込み要求禁止フラグにセットし，設定した優先度を内部変数
 *  tsim_ipmに保存する．
 */
Inline void
x_set_ipm(PRI intpri)
{
    uint32_t ipm_mask = ipm_mask_tbl[INDEX_IPM(intpri)];

    /*
     *  AT91SAM7Sの割込みコントローラはイネーブルレジスタと
     *  クリアーレジスタがあるため，一旦全ての割込みを禁止してから，
     *  特定の割込みのみ許可する必要がある
     */
    /* 全割込み禁止 */
    at91skyeye_disable_int(~0U);

    /* マスク指定されていない割込みのみ許可 */
    at91skyeye_enable_int(~(ipm_mask|tsim_idf));

    tsim_ipm = intpri;
}

#define t_set_ipm(intpri) x_set_ipm(intpri)
#define i_set_ipm(intpri) x_set_ipm(intpri)

/*
 *  (モデル上の)割込み優先度マスクの参照
 *
 *  ipmの値を返す
 */
Inline PRI
x_get_ipm(void)
{
    return(tsim_ipm);
}

#define t_get_ipm() x_get_ipm()
#define i_get_ipm() x_get_ipm()

/*
 *  割込み属性が設定されているかを判別するための変数
 */
extern uint32_t	bitpat_cfgint;

/*
 * （モデル上の）割込み要求禁止フラグのセット
 *
 *  指定された，割込み番号の割込み要求禁止フラグのセットして，割込みを
 *  禁止する．また，（モデル上の）割込み要求禁止フラグを管理するtsim_idfの対
 *  応するビットををセットする．
 *  割込み要求をマスクする機能をサポートしていない場合には，falseを返す
 */


Inline bool_t
x_disable_int(INTNO intno)
{
    if ((bitpat_cfgint & INTNO_BITPAT(intno)) == 0U) {
        return(false);
    }
    at91skyeye_disable_int(INTNO_BITPAT(intno));
    tsim_idf |= INTNO_BITPAT(intno);
    return(true);
}

#define t_disable_int(intno)  x_disable_int(intno)
#define i_disable_int(intno)  t_disable_int(intno)

/* 
 * (モデル上の)割り要求禁止フラグの解除
 *
 * 指定された，割込み番号の割込み要求禁止フラグのクリアして，割込みを
 * 許可する．また，（モデル上の）割込み要求禁止フラグを管理するtsim_idfの対
 * 応するビットををクリアする．
 * 割込み要求をマスクする機能をサポートしていない場合には，falseを返す
 */

Inline bool_t
x_enable_int(INTNO intno)
{
    uint32_t ipm_mask = ipm_mask_tbl[INDEX_IPM(tsim_ipm)];

    if ((bitpat_cfgint & INTNO_BITPAT(intno)) == 0U) {
        return(false);
    }
    tsim_idf &= ~INTNO_BITPAT(intno);

    /* 全割込み禁止 */
    at91skyeye_disable_int(~0U);    
    /* マスク指定されていない割込みのみ許可 */
    at91skyeye_enable_int(~(ipm_mask|tsim_idf));

    return(true);
}

#define t_enable_int(intno) x_enable_int(intno)
#define i_enable_int(intno) x_enable_int(intno)

/*
 * 割込み要求のクリア
 */
Inline void
x_clear_int(INTNO intno)
{
    at91skyeye_clear_int(INTNO_BITPAT(intno));
}

#define t_clear_int(intno) x_clear_int(intno) 
#define i_clear_int(intno) x_clear_int(intno) 


/*
 *  割込み要求のチェック
 */
Inline bool_t
x_probe_int(INTNO intno)
{
    return(at91skyeye_probe_int(INTNO_BITPAT(intno)));
}

#define t_probe_int(intno) x_probe_int(intno)
#define i_probe_int(intno) x_probe_int(intno)

/*
 *  割込み要求ラインの属性の設定
 *
 */
extern void    x_config_int(INTNO intno, ATR intatr, PRI intpri);

/*
 *  割込みハンドラの入り口で必要なIRC操作
 *
 *  AT91SAM7Sでは，必要な処理はない
 */
Inline void
i_begin_int(INTNO intno)
{
    
}

/*
 *  割込みハンドラの出口で必要なIRC操作
 *
 *  AT91SAM7Sでは，必要な処理はない
 */
Inline void
i_end_int(INTNO intno)
{
    
}

/*
 *  ターゲットシステム依存の初期化
 */
extern void target_initialize(void);

/*
 *  ターゲットシステムの終了
 *
 *  システムを終了する時に使う．
 */
extern void target_exit(void) NoReturn;

/*
 *  割込みハンドラ（chip_support.S）
 */
extern void interrupt_handler(void);

/*
 *  未定義の割込みが入った場合の処理
 */
extern void default_int_handler(void);

#ifdef TOPPERS_ENABLE_GCOV_PART
/*
 *  GCOV一部取得用ライブラリ
 */
/*
 *  GCOV初期化関数
 */
extern void gcov_init(void);

/*
 *  GCOV中断関数
 */
extern void gcov_pause(void);

/*
 * GCOV再開関数
 */
extern void gcov_resume(void);

/*
 *  GCOV出力関数
 */
extern void gcov_dump(void);
#endif /* TOPPERS_ENABLE_GCOV_PART */

#endif /* TOPPERS_MACRO_ONLY */

/*
 *  コア依存モジュール（ARM用）
 */
#include "core_config.h"

#endif /* TOPPERS_TARGET_CONFIG_H */
