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
 *  @(#) $Id: target_config.h 2728 2015-12-30 01:46:11Z ertl-honda $
 */

/*
 *		ターゲット依存モジュール（DVE-68K/40用）
 *
 *  カーネルのターゲット依存部のインクルードファイル．kernel_impl.hのター
 *  ゲット依存部の位置付けとなる．
 */

#ifndef TOPPERS_TARGET_CONFIG_H
#define TOPPERS_TARGET_CONFIG_H

/*
 *  ターゲットシステムのハードウェア資源の定義
 */
#include "dve68k.h"

/*
 *  ターゲット定義のオブジェクト属性
 */
#define TARGET_INHATR	TA_NONKERNEL	/* カーネル管理外の割込み */

/*
 *  エラーチェック方法の指定
 */
#define CHECK_STKSZ_ALIGN	4	/* スタックサイズのアライン単位 */
#define CHECK_FUNC_ALIGN	2	/* 関数のアライン単位 */
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
 *  ASPカーネル動作時のメモリマップと関連する定義
 *
 *	00000000 - 0000ffff		ROMモニタ／スタブワークエリア（64KB）
 *	00010000 - 000fffff		コード領域（約1MB）
 *	00100000 -				データ領域（約3MB）
 *			 - 003fffff		デフォルトの非タスクコンテキスト用のスタック領域
 */

/*
 *  デフォルトの非タスクコンテキスト用のスタック領域の定義
 */
#define DEFAULT_ISTKSZ		0x2000U
#define DEFAULT_ISTK		(void *)(0x00400000U - DEFAULT_ISTKSZ)

#ifndef TOPPERS_MACRO_ONLY

/*
 *  プロセッサ識別のための変数（マルチプロセッサ対応）
 */
extern uint_t	board_id;		/* ボードID */
extern void		*board_addr;	/* ローカルメモリの先頭アドレス */

/*
 *  割込み番号の範囲の判定
 *
 *  ビットパターンを求めるのを容易にするために，8は欠番になっている．
 */
#define VALID_INTNO(intno)	(1U <= (intno) && (intno) <= 31U && (intno) != 8U)
#define	VALID_INTNO_CREISR(intno)	VALID_INTNO(intno)
#define VALID_INTNO_DISINT(intno)	VALID_INTNO(intno)
#define VALID_INTNO_CFGINT(intno)	VALID_INTNO(intno)

/*
 *  割込み要求を操作するためのビットパターンを求めるマクロ
 *
 *  シフト演算の遅いプロセッサで，メモリに余裕がある場合には，表を引く
 *  形で実装した方がよいだろう．表を引くなら，割込み番号8を欠番にする
 *  必要はない．
 */
#define DGA_INT_BITPAT(intno)	(1U << (32U - (intno)))

/*
 *  レベルトリガ／エッジトリガが設定できるかの判定用定数
 */
#define DGA_INT_TRG_CONF	(0xe0000000U)	/* いずれにも設定できる */
#define DGA_INT_TRG_LEVEL	(0x108f00feU)	/* レベルトリガに固定 */
#define DGA_INT_TRG_EDGE	(0x0e70ff00U)	/* エッジトリガに固定 */

/*
 *  割込み属性が設定されているかを判別するための変数（kernel_cfg.c）
 */
extern const uint32_t	bitpat_cfgint;

/*
 *  割込み要求禁止フラグのセット
 *
 *  割込み属性が設定されていない割込み要求ラインに対して割込み要求禁止
 *  フラグをセットしようとした場合には，falseを返す．
 */
Inline bool_t
x_disable_int(INTNO intno)
{
	uint32_t	bitpat = DGA_INT_BITPAT(intno);

	if ((bitpat_cfgint & bitpat) == 0U) {
		return(false);
	}
	dga_bit_and((void *) TADR_DGA_CSR21, ~bitpat);
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
	uint32_t	bitpat = DGA_INT_BITPAT(intno);

	if ((bitpat_cfgint & bitpat) == 0U) {
		return(false);
	}
	dga_bit_or((void *) TADR_DGA_CSR21, bitpat);
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
	dga_write((void *) TADR_DGA_CSR23, DGA_INT_BITPAT(intno));
}

#define t_clear_int(intno)		x_clear_int(intno)
#define i_clear_int(intno)		x_clear_int(intno)

/*
 *  割込み要求のチェック
 */
Inline bool_t
x_probe_int(INTNO intno)
{
	return((dga_read((void *) TADR_DGA_CSR20) & DGA_INT_BITPAT(intno)) != 0U);
}

#define t_probe_int(intno)		x_probe_int(intno)
#define i_probe_int(intno)		x_probe_int(intno)

/*
 *  割込み要求ラインの属性の設定
 *
 *  DVE-68K/40のIRCでは，割込み要求ラインの割込み優先度にNMIを指定する
 *  ことができため，-7を与えるとNMIに設定されることとする（ターゲット定
 *  義の拡張）．
 */
extern void	x_config_int(INTNO intno, ATR intatr, PRI intpri);

/*
 *  割込みハンドラの入口で必要なIRC操作
 *
 *  エッジトリガの場合には，割込み要求をクリアする．エッジトリガかレベ
 *  ルトリガかが設定できる場合も，現在の設定値を見ずにクリアしている．
 *  レベルトリガの場合もクリア処理をして差し支えないが，最適化で消える
 *  ことを期待して，クリアしないこととしている．
 */
Inline void
i_begin_int(INTNO intno)
{
	if ((DGA_INT_BITPAT(intno) & (DGA_INT_TRG_CONF | DGA_INT_TRG_EDGE))
																!= 0U) {
		i_clear_int(intno);
	}
}

/*
 *  割込みハンドラの出口で必要なIRC操作
 *
 *  DVE-68K/40では必要なIRC操作はない．
 */
Inline void
i_end_int(INTNO intno)
{
}

#endif /* TOPPERS_MACRO_ONLY */

#ifndef TOPPERS_MACRO_ONLY

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
 *  微少時間待ちのための定義（本来はSILのターゲット依存部）
 */
#define SIL_DLY_TIM1	420
#define SIL_DLY_TIM2	90

/*
 *  プロセッサ依存モジュール（M68040用）
 */
#include "m68k_gcc/prc_config.h"

#endif /* TOPPERS_TARGET_CONFIG_H */
