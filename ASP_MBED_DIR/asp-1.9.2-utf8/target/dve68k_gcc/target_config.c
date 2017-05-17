/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2005-2011 by Embedded and Real-Time Systems Laboratory
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
 *  @(#) $Id: target_config.c 2728 2015-12-30 01:46:11Z ertl-honda $
 */

/*
 *		ターゲット依存モジュール（DVE68K/40用）
 */

#include "kernel_impl.h"
#include <sil.h>

/*
 *  プロセッサ識別のための変数（マルチプロセッサ対応）
 */
uint_t	board_id;			/* ボードID */
void	*board_addr;		/* ローカルメモリの先頭アドレス */

/*
 *  ターゲット依存の初期化
 */
void
target_initialize(void)
{
	/*
	 *  プロセッサ依存の初期化
	 */
	prc_initialize();

	/*
	 *  プロセッサ識別のための変数の初期化
	 */
	board_id = ((uint_t)(sil_rew_mem((void *) TADR_BOARD_REG0)) & 0x1fU);
	board_addr = (void *)(board_id << 24);

	/*
	 *  割込み関連の初期化
	 *
	 *  割込み属性が設定されているかを判別するための変数を初期化する．
	 *  また，すべての割込みをマスク・クリアし，割込みベクトルを設定す
	 *  る．
	 */
	dga_write((void *) TADR_DGA_CSR21, 0U);
	dga_write((void *) TADR_DGA_CSR23, ~0U);
	dga_write((void *) TADR_DGA_CSR19, (TVEC_G0I << 24) | (TVEC_G1I << 16)
										| (TVEC_SWI << 8) | TVEC_SPRI);

	/*
	 *  アボート割込みの設定（NMI）
	 *
	 *  アボート割込みをエッジトリガ，割込みレベルをNMIに設定し，マスク
	 *  を解除する．
	 */
	x_config_int(TINTNO_ABT, TA_EDGE, TIRQ_NMI);

	/*
	 *  メモリ領域の設定
	 *
	 *  ローカルメモリのVMEバス上での先頭アドレスとサイズ（16MB）し，ア
	 *  クセスを受け付けるように設定する．また，VMEバスから拡張アドレス
	 *  アクセスを受け付けるようにに設定する．
	 */
	dga_write((void *) TADR_DGA_CSR4, (uint32_t) board_addr | 0x00ffU);
	dga_write((void *) TADR_DGA_CSR5, 0x0000012fU);

	/*
	 *  インタフェースレジスタ（IFR）の設定
	 *
	 *  インタフェースレジスタのベースアドレスを設定する．また，インタ
	 *  フェースレジスタ0のサービスリクエストフラグをクリア．インタフェー
	 *  スレジスタ3にボードのID番号を設定．
	 */
	dga_write((void *) TADR_DGA_CSR3, (board_id << 4) | 0x3U);
	dga_write((void *) TADR_DGA_IFR0, 0x80000000U);
	dga_write((void *) TADR_DGA_IFR3, board_id);

	/*
	 *  ラウンドロビンモードに設定（マルチプロセッサ対応）
	 */
	dga_write((void *) TADR_DGA_CSR1,
						(dga_read((void *) TADR_DGA_CSR1) & 0xffeffcffU)
									| (1U << 20) | ((board_id % 4) << 8));
}

/*
 *  ターゲット依存の終了処理
 */
void
target_exit(void)
{
	/*
	 *  プロセッサ依存の終了処理
	 */
	prc_terminate();

	/*
	 *  すべての割込みをマスク・クリアする．
	 */
	dga_write((void *) TADR_DGA_CSR21, 0U);
	dga_write((void *) TADR_DGA_CSR23, ~0U);

	/*
	 *  開発環境依存の終了処理
	 */
	dve68k_exit();
}

/*
 *  システムログの低レベル出力のための文字出力
 */
void
target_fput_log(char c)
{
	if (c == '\n') {
		dve68k_putc('\r');
	}
	dve68k_putc(c);
}

/*
 *  割込み要求ラインの属性の設定
 *
 *  ASPカーネルでの利用を想定して，パラメータエラーはアサーションでチェッ
 *  クしている．cfg_intサービスコールを設ける場合には，エラーを返すよう
 *  にすべきであろう．
 */
void
x_config_int(INTNO intno, ATR intatr, PRI intpri)
{
	uint32_t	bitpat = DGA_INT_BITPAT(intno);

	assert(VALID_INTNO_CFGINT(intno));
	assert(TIRQ_NMI <= intpri && intpri <= TIRQ_LEVEL1);

	/*
	 *  割込みのマスク
	 *
	 *  割込みを受け付けたまま，レベルトリガ／エッジトリガの設定や，割
	 *  込み優先度の設定を行うのは危険なため，割込み属性にかかわらず，
	 *  一旦マスクする．
	 */
	(void) x_disable_int(intno);

	/*
	 *  レベルトリガ／エッジトリガの設定
	 */
	if ((bitpat & DGA_INT_TRG_CONF) != 0U) {
		/*
		 *  いずれにも設定できる場合
		 */
		if ((intatr & TA_EDGE) != 0U) {
			dga_bit_or((void *) TADR_DGA_CSR18, (1U << (24 - (intno))));
			x_clear_int(intno);
		}
		else {
			dga_bit_and((void *) TADR_DGA_CSR18, ~(1U << (24 - (intno))));
		}
	}
	else if ((bitpat & DGA_INT_TRG_EDGE) != 0U) {
		/*
		 *  エッジトリガに固定されている場合
		 */
		assert((intatr & TA_EDGE) != 0U);
		x_clear_int(intno);
	}
	else {
		/*
		 *  レベルトリガに固定されている場合
		 */
		assert((intatr & TA_EDGE) == 0U);
	}

	/*
	 *  割込み優先度の設定
	 */
	dga_set_ilv((void *)(TADR_DGA_CSR24 + (intno - 1) / 8 * 4),
				(uint_t)(((32 - intno) % 8) * 4), (uint_t)(7 + intpri));

	/*
	 *  割込みのマスク解除（必要な場合）
 	 */
	if ((intatr & TA_ENAINT) != 0U) {
		(void) x_enable_int(intno);
	}
}
