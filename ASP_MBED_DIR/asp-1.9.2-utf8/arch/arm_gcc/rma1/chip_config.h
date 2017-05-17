/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2006-2011 by Embedded and Real-Time Systems Laboratory
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
 *  @(#) $Id: chip_config.h 2742 2016-01-09 04:25:18Z ertl-honda $
 */

/*
 *  チップ依存モジュール（RMA1用）
 *
 *  カーネルのチップ依存部のインクルードファイル．kernel_impl.hのター
 *  ゲット依存部の位置付けとなる．
 */

#ifndef TOPPERS_CHIP_CONFIG_H
#define TOPPERS_CHIP_CONFIG_H

#include "rma1.h"
#include "gic.h"

/*
 *  割込みハンドラ番号と割込み番号に関する定義
 */
#define TMIN_INTNO  GIC_TMIN_INTNO 
#define TMAX_INTNO  GIC_TMAX_INTNO
#define TNUM_INT    GIC_TNUM_INT  

#define TMIN_INHNO  GIC_TMIN_INTNO 
#define TMAX_INHNO  GIC_TMAX_INTNO
#define TNUM_INH    GIC_TNUM_INT

#ifndef TOPPERS_MACRO_ONLY

/*
 *  割込み番号の範囲の判定
 */
#define VALID_INTNO(intno) (TMIN_INTNO <= (intno) && (intno) <= TMAX_INTNO)
#define VALID_INTNO_DISINT(intno)    VALID_INTNO(intno)
#define VALID_INTNO_CFGINT(intno)    VALID_INTNO(intno)
#define VALID_INTNO_ATTISR(intno)    VALID_INTNO(intno)

/*
 *  割込みハンドラの登録用テーブル
 */
extern const FP inh_tbl[TNUM_INH];

/*
 *  割込みハンドラの設定
 * 
 *  割込みハンドラ番号inhnoの割込みハンドラの起動番地をinthdrに設定する
 */
Inline void
x_define_inh(INHNO inhno, FP int_entry)
{
	assert(TMIN_INHNO <= inhno && inhno <= TMAX_INHNO);
}

/*
 *  割込みハンドラの出入口処理の生成マクロ
 *
 */
#define INT_ENTRY(inhno, inthdr)    inthdr
#define INTHDR_ENTRY(inhno, inhno_num, inthdr) extern void inthdr(void);

#endif /* TOPPERS_MACRO_ONLY */  

/*
 *  割込み優先度マスク操作ライブラリ（PL390（GIC）用）
 *
 *  GICでは，GIC内の優先度レジスタにCPUの割込み優先度マスク（IPM）を設定する．  
 *  割込み優先度マスクは値が大きいほど低優先度である．
 *  割込み優先度マスクの段階はGICの実装時のパラメータにより，256,64,32,16 から
 *  選択可能である． 
 * 
 *  IPMを保存しておくために，割込み優先度の外部表現（-1から連続した負の値）
 *  を使うことも可能であるが，余計な計算が必要になる．これを避けるため，IPMを
 *  保存する場合には，GICで扱う優先度の形式とする．この値を割込み優先度マスク
 *  の内部表現と呼び，IIPMと書くことにする． 
 */

/*
 *  割込み優先度マスクの外部表現と内部表現の変換
 *
 *  アセンブリ言語のソースファイルからインクルードする場合のために，型
 *  キャストしない定義も用意している．
 */
#ifndef TOPPERS_MACRO_ONLY
#define EXT_IPM(iipm)    ((PRI)(iipm - GICC_PMR_MASK))    /* 内部表現を外部表現に */
#define INT_IPM(ipm)    ((uint8_t)(ipm + GICC_PMR_MASK))  /* 外部表現を内部表現に */
#else /* TOPPERS_MACRO_ONLY */
#define EXT_IPM(iipm)    (iipm - GICC_PMR_MASK)           /* 内部表現を外部表現に */
#define INT_IPM(ipm)    (ipm + GICC_PMR_MASK)             /* 外部表現を内部表現に */
#endif /* TOPPERS_MACRO_ONLY */

/*
 *  TIPM_ENAALL（割込み優先度マスク全解除）の内部表現
 */
#define IIPM_ENAALL        (INT_IPM(TIPM_ENAALL))

#ifndef TOPPERS_MACRO_ONLY

/*
 *  IPM（ハードウェアの割込み優先度マスク，内部表現）の現在値の読出し
 */
Inline uint8_t
current_iipm(void)
{
	return(gicc_current_priority());
}

/*
 *  IPM（ハードウェアの割込み優先度マスク，内部表現）の現在値の設定
 */
Inline void
set_iipm(uint8_t iipm)
{
	gicc_set_priority(iipm);
}

/*
 * (モデル上の)割込み優先度マスクの設定
 */
Inline void
x_set_ipm(PRI intpri)
{
	set_iipm(INT_IPM(intpri));
}

#define t_set_ipm(intpri) x_set_ipm(intpri)
#define i_set_ipm(intpri) x_set_ipm(intpri)

/*
 *  (モデル上の)割込み優先度マスクの参照
 */
Inline PRI
x_get_ipm(void)
{
	return(EXT_IPM(current_iipm()));
}

#define t_get_ipm() x_get_ipm()
#define i_get_ipm() x_get_ipm()

/*
 *  割込み属性テーブル
 * 
 *  割込み属性が設定されていれば"1"，設定されていなければ"0"となる
 */
extern const uint8_t cfgint_tbl[TNUM_INT];

/*
 * （モデル上の）割込み要求禁止フラグのセット
 *
 *  指定された，割込み番号の割込み要求禁止フラグのセットして，割込みを
 *  禁止する．
 *    
 *  割込み属性が設定されていない割込み要求ラインに対して割込み要求禁止
 *  フラグをクリアしようとした場合には，falseを返す．  
 */
Inline bool_t
x_disable_int(INTNO intno)
{
	if (cfgint_tbl[intno] == 0){
		return(false);
	}
    
	gicd_disable_int(intno);
	return(true);
}

#define t_disable_int(intno)  x_disable_int(intno)
#define i_disable_int(intno)  t_disable_int(intno)

/* 
 *  (モデル上の)割り要求禁止フラグの解除
 *
 *  指定された，割込み番号の割込み要求禁止フラグのクリアして，割込みを
 *  許可する．
 *
 *  割込み属性が設定されていない割込み要求ラインに対して割込み要求禁止
 *  フラグをクリアしようとした場合には，falseを返す．
 */
Inline bool_t
x_enable_int(INTNO intno)
{
	if (cfgint_tbl[intno] == 0){
		return(false);
	}
	gicd_enable_int(intno);
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
	gicd_clear_pending(intno);
}

#define t_clear_int(intno) x_clear_int(intno) 
#define i_clear_int(intno) x_clear_int(intno) 

/*
 *  割込み要求のチェック
 */
Inline bool_t
x_probe_int(INTNO intno)
{
	return(gicd_probe_int(intno));
}

#define t_probe_int(intno) x_probe_int(intno)
#define i_probe_int(intno) x_probe_int(intno)

/*
 *  割込み要求ラインの属性の設定
 *
 */
extern void x_config_int(INTNO intno, ATR intatr, PRI intpri);

/*
 * 割込みハンドラの入り口で必要なIRC操作
 *
 */
Inline void
i_begin_int(INTNO intno)
{    
}

/*
 * 割込みハンドラの出口で必要なIRC操作
 */
Inline void
i_end_int(INTNO intno)
{
}

/*
 *  チップ依存の初期化
 */
extern void chip_initialize(void);

/*
 *  チップ依存部の終了
 *
 *  システムを終了する時に使う．
 */
extern void chip_exit(void);

/*
 *  割込みハンドラ（chip_support.S）
 */
extern void interrupt_handler(void);

/*
 *  未定義の割込みが入った場合の処理
 */
extern void default_int_handler(void);


#endif /* TOPPERS_MACRO_ONLY */

/*
 *  ARM依存モジュール
 */
#include "arm_gcc/common/core_config.h"

#endif /* TOPPERS_CHIP_CONFIG_H */
