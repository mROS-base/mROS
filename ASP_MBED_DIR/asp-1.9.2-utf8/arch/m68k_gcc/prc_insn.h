/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2005-2007 by Embedded and Real-Time Systems Laboratory
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
 *  @(#) $Id: prc_insn.h 2728 2015-12-30 01:46:11Z ertl-honda $
 */

/*
 *		プロセッサの特殊命令のインライン関数定義（M68040用）
 */

#ifndef	TOPPERS_PRC_INSN_H
#define	TOPPERS_PRC_INSN_H

/*
 *  ステータスレジスタ（SR）の現在値の読出し
 */
Inline uint16_t
current_sr(void)
{
	uint16_t	sr;

	Asm("move.w %%sr, %0" : "=g"(sr));
	return(sr);
}

/*
 *  ステータスレジスタ（SR）の現在値の設定
 */
Inline void
set_sr(uint16_t sr)
{
	Asm("move.w %0, %%sr" : : "g"(sr) : "cc");
}

/*
 *  NMIを除くすべての割込みの禁止
 *
 *  set_srを用いて実現することもできるが，別の命令を用いて実現した方が
 *  効率が良いために用意している．
 */
Inline void
disint(void)
{
	Asm("or.w #0x0700, %sr");
}

/*
 *  すべての割込みの許可
 *
 *  set_srを用いて実現することもできるが，別の命令を用いて実現した方が
 *  効率が良いために用意している．
 */
Inline void
enaint(void)
{
	Asm("and.w #~0x0700, %sr");
}

/*
 *  ベクタベースレジスタ（VBR）の現在値の読出し
 */
Inline void
*current_vbr(void)
{
	void	*vbr;

	Asm("movec.l %%vbr, %0" : "=r"(vbr));
	return(vbr);
}

/*
 *  ベクタベースレジスタ（VBR）の現在値の設定
 */
Inline void
set_vbr(void *vbr)
{
	Asm("movec.l %0, %%vbr" : : "r"(vbr));
}

/*
 *  レディキューサーチのためのビットマップサーチ関数
 *
 *  bfffo命令は最上位ビットからサーチするため，最上位ビットを最高優先度
 *  に対応させる．
 */
#define	OMIT_BITMAP_SEARCH
#define	PRIMAP_BIT(pri)		(0x8000U >> (pri))

Inline uint_t
bitmap_search(uint16_t bitmap)
{
	uint32_t	offset;

	Asm("bfffo %1{16,16}, %0" : "=d"(offset) : "d"((uint32_t) bitmap));
	return((uint_t)(offset - 16));
}

#endif /* TOPPERS_PRC_INSN_H */
