/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2013-2014 by Embedded and Real-Time Systems Laboratory
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
 *  $Id: target_serial.c 2728 2015-12-30 01:46:11Z ertl-honda $
 */

/*
 *		シリアルI/Oデバイス（SIO）ドライバ（ダミーターゲット用）
 */

#include "target_serial.h"

/*
 *  シリアルI/Oポート初期化ブロックの定義
 */
typedef struct sio_port_initialization_block {
	int_t		dummy;			/* ダミーフィールド */
	/* SIOのレジスタの番地など */
} SIOPINIB;

/*
 *  シリアルI/Oポート管理ブロックの定義
 */
struct sio_port_control_block {
	const SIOPINIB *p_siopinib;	/* シリアルI/Oポート初期化ブロック */
	intptr_t	exinf;			/* 拡張情報 */
	bool_t		openflag;		/* オープン済みフラグ */
	/* SIOの状態など */
};

/*
 *  シリアルI/Oポート初期化ブロック
 */
const SIOPINIB siopinib_table[TNUM_SIOP] = {
	{ 0 }
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
 *  SIOドライバの初期化
 */
void
sio_initialize(intptr_t exinf)
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
		/* その他のフィールドの初期化 */
	}
}

/*
 *  SIOドライバの終了処理
 */
void
sio_terminate(intptr_t exinf)
{
	SIOPCB	*p_siopcb;
	uint_t	i;

	/*
	 *  オープンされているシリアルI/Oポートのクローズ
	 */
	for (i = 0; i < TNUM_SIOP; i++) {
		p_siopcb = &(siopcb_table[i]);
		if (p_siopcb->openflag) {
			/* SIOのクローズ処理 */
		}
	}
}

/*
 *  シリアルI/Oポートのオープン
 */
SIOPCB *
sio_opn_por(ID siopid, intptr_t exinf)
{
	SIOPCB			*p_siopcb;
	const SIOPINIB	*p_siopinib;

	p_siopcb = get_siopcb(siopid);
	p_siopinib = p_siopcb->p_siopinib;

	/* SIOのオープン処理 */

	p_siopcb->exinf = exinf;
	p_siopcb->openflag = true;
	return(p_siopcb);
}

/*
 *  シリアルI/Oポートのクローズ
 */
void
sio_cls_por(SIOPCB *p_siopcb)
{
	/* SIOのクローズ処理 */

	p_siopcb->openflag = false;
}

/*
 *  SIOの割込みサービスルーチン
 */
void
sio_isr(intptr_t exinf)
{
	SIOPCB	*p_siopcb = &(siopcb_table[(int_t) exinf]);
	
	if (1/* 送信完了割込みの場合 */) {
		sio_irdy_snd(p_siopcb->exinf);
	}
	if (1/* 受信完了割込みの場合 */) {
		sio_irdy_rcv(p_siopcb->exinf);
	}
}

/*
 *  シリアルI/Oポートへの文字送信
 */
bool_t
sio_snd_chr(SIOPCB *p_siopcb, char c)
{
	if (1/* 送信レジスタに空きがある場合 */) {
		/* 送信レジスタに文字cを入れる */
		return(true);
	}
	else {
		return(false);
	}
}

/*
 *  シリアルI/Oポートからの文字受信
 */
int_t
sio_rcv_chr(SIOPCB *p_siopcb)
{
	char	c;

	if (1/* 受信レジスタに文字がある場合 */) {
		/* 受信レジスタの文字をcに入れる */
		return((int_t) c);
	}
	else {
		return(-1);
	}
}

/*
 *  シリアルI/Oポートからのコールバックの許可
 */
void
sio_ena_cbr(SIOPCB *p_siopcb, uint_t cbrtn)
{
	switch (cbrtn) {
	case SIO_RDY_SND:
		/* 送信完了割込みを許可 */
		break;
	case SIO_RDY_RCV:
		/* 受信完了割込みを許可 */
		break;
	}
}

/*
 *  シリアルI/Oポートからのコールバックの禁止
 */
void
sio_dis_cbr(SIOPCB *p_siopcb, uint_t cbrtn)
{
	switch (cbrtn) {
	case SIO_RDY_SND:
		/* 送信完了割込みを禁止 */
		break;
	case SIO_RDY_RCV:
		/* 受信完了割込みを禁止 */
		break;
	}
}
