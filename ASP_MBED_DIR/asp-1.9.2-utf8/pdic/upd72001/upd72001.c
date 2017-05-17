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
 *  @(#) $Id: upd72001.c 2728 2015-12-30 01:46:11Z ertl-honda $
 */

/*
 *		μPD72001用 簡易SIOドライバ
 */

#include <sil.h>
#include "target_syssvc.h"
#include "upd72001.h"

/*
 *  デバイスレジスタのアクセス間隔時間（nsec単位）
 *
 *  200という値にあまり根拠はない．
 */
#define	UPD72001_DELAY	200U

/*
 *  μPD72001のレジスタの番号
 */
#define	UPD72001_CR0	0x00U		/* コントロールレジスタ */
#define	UPD72001_CR1	0x01U
#define	UPD72001_CR2	0x02U
#define	UPD72001_CR3	0x03U
#define	UPD72001_CR4	0x04U
#define	UPD72001_CR5	0x05U
#define	UPD72001_CR10	0x0aU
#define	UPD72001_CR12	0x0cU
#define	UPD72001_CR14	0x0eU
#define	UPD72001_CR15	0x0fU

#define	UPD72001_SR0	0x00U		/* ステータスレジスタ */

/*
 *  コントロールレジスタの設定値
 */
#define CR_RESET	0x18U		/* ポートリセットコマンド */

#define CR0_EOI		0x38U		/* EOI（End of Interrupt）*/

#define CR1_DOWN	0x00U		/* 全割込みを禁止 */
#define CR1_RECV	0x10U		/* 受信割込み許可ビット */
#define CR1_SEND	0x02U		/* 送信割込み許可ビット */

#define CR3_DEF		0xc1U		/* データ 8bit，受信イネーブル */
#define CR4_DEF		0x44U		/* ストップビット 1bit，パリティなし */
#define CR5_DEF		0xeaU		/* データ 8bit，送信イネーブル */

#define CR10_DEF	0x00U		/* NRZ */
#define CR14_DEF	0x07U		/* ボーレートジェネレータイネーブル */
#define CR15_DEF	0x56U		/* ボーレートジェネレータ使用 */

#define SR0_RECV	0x01U		/* 受信通知ビット */
#define SR0_SEND	0x04U		/* 送信可能ビット */

/*
 *  シリアルI/Oポート初期化ブロックの定義
 */
typedef struct sio_port_initialization_block {
	void		*data;			/* データレジスタの番地 */
	void		*ctrl;			/* コントロールレジスタの番地 */

	uint8_t		cr3_def;		/* CR3の設定値（受信ビット数）*/
	uint8_t		cr4_def;		/* CR4の設定値（ストップビット，パリティ）*/
	uint8_t		cr5_def;		/* CR5の設定値（送信ビット数）*/
	uint8_t		brg1_def;		/* ボーレート上位の設定値 */
	uint8_t		brg2_def;		/* ボーレート下位の設定値 */
} SIOPINIB;

/*
 *  シリアルI/Oポート管理ブロックの定義
 */
struct sio_port_control_block {
	const SIOPINIB *p_siopinib;	/* シリアルI/Oポート初期化ブロック */
	intptr_t	exinf;			/* 拡張情報 */
	bool_t		openflag;		/* オープン済みフラグ */
	uint8_t		cr1;			/* CR1の設定値（割込み許可）*/
	bool_t		getready;		/* 文字を受信した状態 */
	bool_t		putready;		/* 文字を送信できる状態 */
};

/*
 *  シリアルI/Oポート初期化ブロック
 */
const SIOPINIB siopinib_table[TNUM_SIOP] = {
	{ (void *) TADR_UPD72001_DATAA, (void *) TADR_UPD72001_CTRLA,
					CR3_DEF, CR4_DEF, CR5_DEF, BRG1_DEF, BRG2_DEF },
	{ (void *) TADR_UPD72001_DATAB, (void *) TADR_UPD72001_CTRLB,
					CR3_DEF, CR4_DEF, CR5_DEF, BRG1_DEF, BRG2_DEF }
};

/*
 *  シリアルI/Oポート管理ブロックのエリア
 */
SIOPCB	siopcb_table[TNUM_SIOP];

/*
 *  シリアルI/OポートIDから管理ブロックを取り出すためのマクロ
 */
#define INDEX_SIOP(siopid)	((uint_t)((siopid) - 1))
#define get_siopcb(siopid)	(&(siopcb_table[INDEX_SIOP(siopid)]))

/*
 *  デバイスレジスタへのアクセス関数
 */
Inline uint8_t
upd72001_read_reg(void *addr)
{
	uint8_t	val;

	val = upd72001_reb_reg(addr);
	sil_dly_nse(UPD72001_DELAY);
	return(val);
}

Inline void
upd72001_write_reg(void *addr, uint8_t val)
{
	upd72001_wrb_reg(addr, val);
	sil_dly_nse(UPD72001_DELAY);
}

Inline uint8_t
upd72001_read_ctrl(void *addr, uint8_t reg)
{
	upd72001_write_reg(addr, reg);
	return(upd72001_read_reg(addr));
}

Inline void
upd72001_write_ctrl(void *addr, uint8_t reg, uint8_t val)
{
	upd72001_write_reg(addr, reg);
	upd72001_write_reg(addr, val);
}

Inline void
upd72001_write_brg(void *addr, uint8_t reg, uint8_t val,
									uint8_t brg2, uint8_t brg1)
{
	upd72001_write_reg(addr, reg);
	upd72001_write_reg(addr, val);
	upd72001_write_reg(addr, brg2);
	upd72001_write_reg(addr, brg1);
	(void) upd72001_read_reg(addr);		/* ダミーリード */
}

/*
 *  状態の読出し（SR0の読出し）
 *
 *  μPD72001は，状態（SR0）を一度読むと受信通知ビットが落ちてしまうた
 *  め，状態を読み出す関数を設け，シリアルI/Oポート管理ブロック中の
 *  getreadyに受信通知状態，putreadyに送信可能状態を保存している（送信
 *  可能状態の保存は不要かもしれない）．
 *  状態レジスタを読んでも受信通知ビットが落ちないデバイス（こちらが普
 *  通と思われる）では，この関数は必要ない．
 */
static void
upd72001_get_stat(SIOPCB *p_siopcb)
{
	uint8_t	sr0;

	sr0 = upd72001_read_ctrl(p_siopcb->p_siopinib->ctrl, UPD72001_SR0);
	if ((sr0 & SR0_RECV) != 0) {
		p_siopcb->getready = true;
	}
	if ((sr0 & SR0_SEND) != 0) {
		p_siopcb->putready = true;
	}
}

/*
 *  文字を受信できるか？
 */
Inline bool_t
upd72001_getready(SIOPCB *p_siopcb)
{
	upd72001_get_stat(p_siopcb);
	return(p_siopcb->getready);
}

/*
 *  文字を送信できるか？
 */
Inline bool_t
upd72001_putready(SIOPCB *p_siopcb)
{
	upd72001_get_stat(p_siopcb);
	return(p_siopcb->putready);
}

/*
 *  受信した文字の取出し
 */
Inline char
upd72001_getchar(SIOPCB *p_siopcb)
{
	p_siopcb->getready = false;
	return((char) upd72001_read_reg(p_siopcb->p_siopinib->data));
}

/*
 *  送信する文字の書込み
 */
Inline void
upd72001_putchar(SIOPCB *p_siopcb, char c)
{
	p_siopcb->putready = false;
	upd72001_write_reg(p_siopcb->p_siopinib->data, (uint8_t) c);
}

/*
 *  EOI（End Of Interrupt）発行
 */
Inline void
upd72001_eoi(void)
{
	upd72001_write_ctrl((void *) TADR_UPD72001_CTRLA, UPD72001_CR0, CR0_EOI);
}

/*
 *  SIOドライバの初期化
 */
void
upd72001_initialize(void)
{
	SIOPCB	*p_siopcb;
	uint_t	i;

	/*
	 *  シリアルI/Oポート管理ブロックの初期化
	 */
	for (i = 0; i < TNUM_SIOP; i++) {
		p_siopcb = &(siopcb_table[i]);
		p_siopcb->p_siopinib = &(siopinib_table[i]);
		p_siopcb->openflag = false;
	}
}

/*
 *  オープンしているポートがあるか？
 */
bool_t
upd72001_openflag(void)
{
	return(siopcb_table[0].openflag || siopcb_table[1].openflag);
}

/*
 *  シリアルI/Oポートのオープン
 */
SIOPCB *
upd72001_opn_por(ID siopid, intptr_t exinf)
{
	SIOPCB		*p_siopcb;
	const SIOPINIB	*p_siopinib;

	p_siopcb = get_siopcb(siopid);
	p_siopinib = p_siopcb->p_siopinib;

	upd72001_write_reg(p_siopinib->ctrl, CR_RESET);
	if (!upd72001_openflag()) {
		upd72001_write_ctrl((void *) TADR_UPD72001_CTRLA, UPD72001_CR2, 0x18);
		upd72001_write_ctrl((void *) TADR_UPD72001_CTRLB, UPD72001_CR2, 0x00);
	}
	p_siopcb->cr1 = CR1_DOWN;
	upd72001_write_ctrl(p_siopinib->ctrl, UPD72001_CR1, p_siopcb->cr1);
	upd72001_write_ctrl(p_siopinib->ctrl, UPD72001_CR4, p_siopinib->cr4_def);
	upd72001_write_brg(p_siopinib->ctrl, UPD72001_CR12, 0x01U,
							p_siopinib->brg2_def, p_siopinib->brg1_def);
	upd72001_write_brg(p_siopinib->ctrl, UPD72001_CR12, 0x02U,
							p_siopinib->brg2_def, p_siopinib->brg1_def);
	upd72001_write_ctrl(p_siopinib->ctrl, UPD72001_CR15, CR15_DEF);
	upd72001_write_ctrl(p_siopinib->ctrl, UPD72001_CR14, CR14_DEF);
	upd72001_write_ctrl(p_siopinib->ctrl, UPD72001_CR10, CR10_DEF);
	upd72001_write_ctrl(p_siopinib->ctrl, UPD72001_CR3, p_siopinib->cr3_def);
	upd72001_write_ctrl(p_siopinib->ctrl, UPD72001_CR5, p_siopinib->cr5_def);
	p_siopcb->exinf = exinf;
	p_siopcb->getready = false;
	p_siopcb->putready = false;
	p_siopcb->openflag = true;
	return(p_siopcb);
}

/*
 *  シリアルI/Oポートのクローズ
 */
void
upd72001_cls_por(SIOPCB *p_siopcb)
{
	upd72001_write_ctrl(p_siopcb->p_siopinib->ctrl, UPD72001_CR1, CR1_DOWN);
	p_siopcb->openflag = false;
}

/*
 *  シリアルI/Oポートへの文字送信
 */
bool_t
upd72001_snd_chr(SIOPCB *p_siopcb, char c)
{
	if (upd72001_putready(p_siopcb)) {
		upd72001_putchar(p_siopcb, c);
		return(true);
	}
	return(false);
}

/*
 *  シリアルI/Oポートからの文字受信
 */
int_t
upd72001_rcv_chr(SIOPCB *p_siopcb)
{
	if (upd72001_getready(p_siopcb)) {
		return((int_t)(uint8_t) upd72001_getchar(p_siopcb));
	}
	return(-1);
}

/*
 *  シリアルI/Oポートからのコールバックの許可
 */
void
upd72001_ena_cbr(SIOPCB *p_siopcb, uint_t cbrtn)
{
	uint8_t	cr1_bit;

	switch (cbrtn) {
	case SIO_RDY_SND:
		cr1_bit = CR1_SEND;
		break;
	case SIO_RDY_RCV:
		cr1_bit = CR1_RECV;
		break;
	default:
		cr1_bit = 0U;
		break;
	}
	p_siopcb->cr1 |= cr1_bit;
	upd72001_write_ctrl(p_siopcb->p_siopinib->ctrl,
									UPD72001_CR1, p_siopcb->cr1);
}

/*
 *  シリアルI/Oポートからのコールバックの禁止
 */
void
upd72001_dis_cbr(SIOPCB *p_siopcb, uint_t cbrtn)
{
	uint8_t	cr1_bit;

	switch (cbrtn) {
	case SIO_RDY_SND:
		cr1_bit = CR1_SEND;
		break;
	case SIO_RDY_RCV:
		cr1_bit = CR1_RECV;
		break;
	default:
		cr1_bit = 0U;
		break;
	}
	p_siopcb->cr1 &= ~cr1_bit;
	upd72001_write_ctrl(p_siopcb->p_siopinib->ctrl,
									UPD72001_CR1, p_siopcb->cr1);
}

/*
 *  シリアルI/Oポートに対する割込み処理
 */
static void
upd72001_isr_siop(SIOPCB *p_siopcb)
{
	if ((p_siopcb->cr1 & CR1_RECV) != 0U) {
		if (upd72001_getready(p_siopcb)) {
			/*
			 *  受信通知コールバックルーチンを呼び出す．
			 */
			upd72001_irdy_rcv(p_siopcb->exinf);
		}
	}
	if ((p_siopcb->cr1 & CR1_SEND) != 0U) {
		if (upd72001_putready(p_siopcb)) {
			/*
			 *  送信可能コールバックルーチンを呼び出す．
			 */
			upd72001_irdy_snd(p_siopcb->exinf);
		}
	}
}

/*
 *  SIOの割込みサービスルーチン
 */
void
upd72001_isr(void)
{
	if (siopcb_table[0].openflag) {
		upd72001_isr_siop(&(siopcb_table[0]));
	}
	if (siopcb_table[1].openflag) {
		upd72001_isr_siop(&(siopcb_table[1]));
	}
	upd72001_eoi();
}
