/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 *
 *  Copyright (C) 2006-2016 by Embedded and Real-Time Systems Laboratory
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2001-2011 by Industrial Technology Institute,
 *                              Miyagi Prefectural Government, JAPAN
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
 *  @(#) $Id: scif.c 2758 2016-03-10 15:15:26Z ertl-honda $
 */

/*
 *   RZ/A1用 簡易SIOドライバ
 */

#include <sil.h>
#include "target_syssvc.h"
#include "scif.h"


/*
 * 各レジスタのオフセット
 */
#define REG_SCSMR       0x00    /* シリアルモードレジスタ, 16bit */
#define REG_SCBRR       0x04    /* ビットレートレジスタ, 8bit */
#define REG_SCSCR       0x08    /* シリアルコントロールレジスタ, 16bit */
#define REG_SCFTDR      0x0C    /* 送信FIFOデータレジスタ, 8bit */
#define REG_SCFSR       0x10    /* シリアルステータスレジスタ, 16bit */
#define REG_SCFRDR      0x14    /* 受信FIFOデータレジスタ, 8bit */
#define REG_SCFCR       0x18    /* FIFOコントロールレジスタ, 16bit */
#define REG_SCFDR       0x1C    /* FIFOデータカウントレジスタ, 16bit */
#define REG_SCSPTR      0x20    /* シリアルポートレジスタ, 16bit */
#define REG_SCLSR       0x24    /* ラインステータスレジスタ, 16bit */
#define REG_SCEMR       0x28    /* シリアル拡張モードレジスタ, 16bit */

/*
 * 各レジスタの設定値
 */
#define SCSMR_SYNC      0x0080          /* 1:クロック同期式モード */
#define SCSMR_7BIT      0x0040          /* 1:７ビットデータ */
#define SCSMR_PARITY    0x0020          /* 1:パリティ付加 */
#define SCSMR_ODD       0x0010          /* 1:奇数パリティ */
#define SCSMR_2STOP     0x0008          /* 1:２ストッピビット */
#define SCSMR_CKS1      0x0000          /* 00:P1clock / 1 */
#define SCSMR_CKS4      0x0001          /* 01:P1clock / 4 */
#define SCSMR_CKS16     0x0002          /* 10:P1clock / 16 */
#define SCSMR_CKS64     0x0003          /* 11:P1clock / 64 */

#define SCSCR_TIE       0x0080          /* 1:送信割り込み許可 */
#define SCSCR_RIE       0x0040          /* 1:受信割り込み,受信エラー割り込み,ブレーク割り込み許可 */
#define SCSCR_TE        0x0020          /* 1:送信許可 */
#define SCSCR_RE        0x0010          /* 1:受信許可 */
#define SCSCR_REIE      0x0008          /* 1:受信エラー割り込み,ブレーク割り込み許可 */
#define SCSCR_INTCLK    0x0000          /* 00:内部クロック,CKS端子は無視（調歩同期式の場合） */

#define SCFSR_PER_MASK  0xF000          /* パリティエラー数抽出マスク */
#define SCFSR_PER_SHIFT 12              /* パリティエラー数抽出右シフト数 */
#define SCFSR_FER_MASK  0x0F00          /* フレーミングエラー数抽出マスク */
#define SCFSR_FER_SHIFT 8               /* フレーミングエラー数抽出右シフト数 */
#define SCFSR_ER        0x0080          /* 1:受信エラー */
#define SCFSR_TEND      0x0040          /* 1:送信完了 */
#define SCFSR_TDFE      0x0020          /* 1:送信FIFOデータエンプティ */
#define SCFSR_BRK       0x0010          /* 1:ブレーク検出 */
#define SCFSR_FER       0x0008          /* 1:フレーミングエラー検出 */
#define SCFSR_PER       0x0004          /* 1:パリティエラー検出 */
#define SCFSR_RDF       0x0002          /* 1:受信FIFOデータフル */
#define SCFSR_DR        0x0001          /* 1:受信データレディ */

#define SCFCR_RSTRG_15  0x0000          /* RTS#出力アクティブトリガ:15 */
#define SCFCR_RSTRG_1   0x0100          /* RTS#出力アクティブトリガ:1 */
#define SCFCR_RSTRG_4   0x0200          /* RTS#出力アクティブトリガ:4 */
#define SCFCR_RSTRG_6   0x0300          /* RTS#出力アクティブトリガ:6 */
#define SCFCR_RSTRG_8   0x0400          /* RTS#出力アクティブトリガ:8 */
#define SCFCR_RSTRG_10  0x0500          /* RTS#出力アクティブトリガ:10 */
#define SCFCR_RSTRG_12  0x0600          /* RTS#出力アクティブトリガ:12 */
#define SCFCR_RSTRG_14  0x0700          /* RTS#出力アクティブトリガ:14 */
#define SCFCR_RTRG_1    0x0000          /* 受信FIFOデータ数トリガ:1（調歩同期式の場合） */
#define SCFCR_RTRG_4    0x0040          /* 受信FIFOデータ数トリガ:4（調歩同期式の場合） */
#define SCFCR_RTRG_8    0x0080          /* 受信FIFOデータ数トリガ:8（調歩同期式の場合） */
#define SCFCR_RTRG_14   0x00C0          /* 受信FIFOデータ数トリガ:14（調歩同期式の場合） */
#define SCFCR_TTRG_8    0x0000          /* 送信FIFOデータ数トリガ:8（調歩同期式の場合） */
#define SCFCR_TTRG_4    0x0010          /* 送信FIFOデータ数トリガ:4（調歩同期式の場合） */
#define SCFCR_TTRG_2    0x0020          /* 送信FIFOデータ数トリガ:2（調歩同期式の場合） */
#define SCFCR_TTRG_0    0x0030          /* 送信FIFOデータ数トリガ:0（調歩同期式の場合） */
#define SCFCR_MCE       0x0008          /* CTS#,RTS#許可 */
#define SCFCR_TFRST     0x0004          /* 1:送信FIFOデータレジスタリセット */
#define SCFCR_RFRST     0x0002          /* 1:受信FIFOデータレジスタリセット */
#define SCFCR_LOOP      0x0001          /* 1:ループバックテスト */

#define SCFDR_T_MASK    0x1F00          /* 未送信データ数抽出マスク */
#define SCFDR_T_SHIFT   8               /* 未送信データ数抽出右シフト数 */
#define SCFDR_R_MASK    0x001F          /* 受信データ数抽出マスク */
#define SCFDR_R_SHIFT   0               /* 受信データ数抽出右シフト数 */

#define SCLSR_ORER      0x0001          /* 1:オーバーランエラー */

#define SCEMR_BGDM      0x0080          /* 1:ボーレートジェネレータ倍速モード */
#define SCEMR_ABCS16    0x0000          /* ビットレートの16倍の基本クロックで動作 */
#define SCEMR_ABCS8     0x0001          /* ビットレートの8倍の基本クロックで動作 */

/*
 *  シリアルI/Oポート初期化ブロックの定義
 */
typedef struct sio_port_initialization_block {
	uint16_t *port;   			/* シリアルポートのベースレジスタ */
	uint32_t bps_setting; 		/* ボーレートの設定値 */
} SIOPINIB;

/*
 *  シリアルI/Oポート管理ブロックの定義
 */
struct sio_port_control_block {
	const SIOPINIB  *p_siopinib;  /* シリアルI/Oポート初期化ブロック */
	intptr_t  exinf;              /* 拡張情報 */
	bool_t    openflag;           /* オープン済みフラグ */
	bool_t    sendflag;           /* 送信割込みイネーブルフラグ */
	bool_t    getready;           /* 文字を受信した状態 */
	bool_t    putready;           /* 文字を送信できる状態 */
	bool_t    is_initialized;     /* デバイス初期化済みフラグ */
};

/*
 *  シリアルI/Oポート初期化ブロック
 */
static const SIOPINIB siopinib_table[TNUM_SIOP] = {
	{
		(uint16_t *)(UART1_BASE),
		UART1_BPS_SETTING,
	},
#if TNUM_SIOP > 1
	{
		(uint16_t *)(UART2_BASE),
		UART2_BPS_SETTING,
	},
#endif /* TNUM_SIOP > 1 */
#if TNUM_SIOP > 2
	{
		(uint16_t *)(UART3_BASE),
		UART3_BPS_SETTING,
	},
#endif /* TNUM_SIOP > 2 */
#if TNUM_SIOP > 3
	{
		(uint16_t *)(UART4_BASE),
		UART4_BPS_SETTING,
	},
#endif /* TNUM_SIOP >  3*/
#if TNUM_SIOP > 4
	{
		(uint16_t *)(UART5_BASE),
		UART5_BPS_SETTING,
	},
#endif /* TNUM_SIOP >  4*/
#if TNUM_SIOP > 5
	{
		(uint16_t *)(UART6_BASE),
		UART6_BPS_SETTING,
	},
#endif /* TNUM_SIOP >  5*/
#if TNUM_SIOP > 6
	{
		(uint16_t *)(UART7_BASE),
		UART7_BPS_SETTING,
	},
#endif /* TNUM_SIOP >  6*/
#if TNUM_SIOP > 7
	{
		(uint16_t *)(UART8_BASE),
		UART8_BPS_SETTING,
	},
#endif /* TNUM_SIOP >  7*/
};

/*
 *  シリアルI/Oポート管理ブロックのエリア
 */
static SIOPCB	siopcb_table[TNUM_SIOP];

/*
 *  シリアルI/OポートIDから管理ブロックを取り出すためのマクロ
 */
#define INDEX_SIOP(siopid)	 ((uint_t)((siopid) - 1))
#define get_siopcb(siopid)	 (&(siopcb_table[INDEX_SIOP(siopid)]))
#define get_siopinib(siopid) (&(siopinib_table[INDEX_SIOP(siopid)]))

/*
 *  管理ブロックへのポインタからシリアルI/OポートIDを取り出すためのマクロ
 */
#define SIOPID(p_siopcb)    ((ID)((p_siopcb) - siopcb_table))

/*
 *  文字を受信したか？
 */
Inline bool_t scif_getready(SIOPCB *p_siopcb)
{
	uint16_t fsrval;
	uint16_t lsrval;

	fsrval = sil_reh_mem((uint16_t *)((uint8_t *)p_siopcb->p_siopinib->port + REG_SCFSR));
	lsrval = sil_reh_mem((uint16_t *)((uint8_t *)p_siopcb->p_siopinib->port + REG_SCLSR));
	if (fsrval & (SCFSR_ER | SCFSR_BRK)) {
		fsrval = fsrval & ~(SCFSR_ER | SCFSR_BRK);
		sil_wrh_mem((uint16_t *)((uint8_t *)p_siopcb->p_siopinib->port + REG_SCFSR), fsrval);
	}
	if (lsrval & SCLSR_ORER) {
		lsrval = lsrval & ~SCLSR_ORER;
		sil_wrh_mem((uint16_t *)((uint8_t *)p_siopcb->p_siopinib->port + REG_SCLSR), lsrval);
	}
	if (fsrval & SCFSR_RDF) {
		return true;
	}
	return false;
}

/*
 *  文字を送信できるか？
 */
Inline bool_t scif_putready(SIOPCB *p_siopcb)
{
	uint16_t fsrval;

	fsrval = sil_reh_mem((uint16_t *)((uint8_t *)p_siopcb->p_siopinib->port + REG_SCFSR));
	if (fsrval & SCFSR_TDFE) {
		return true;
	}
	return false;
}

/*
 *  受信した文字の取出し
 */
Inline bool_t scif_getchar(SIOPCB *p_siopcb, char *rxdata)
{
	uint16_t fsrval;
	uint16_t lsrval;
	uint8_t read_data;

	fsrval = sil_reh_mem((uint16_t *)((uint8_t *)p_siopcb->p_siopinib->port + REG_SCFSR));
	lsrval = sil_reh_mem((uint16_t *)((uint8_t *)p_siopcb->p_siopinib->port + REG_SCLSR));
	if (fsrval & (SCFSR_ER | SCFSR_BRK)) {
		fsrval = fsrval & ~(SCFSR_ER | SCFSR_BRK);
		sil_wrh_mem((uint16_t *)((uint8_t *)p_siopcb->p_siopinib->port + REG_SCFSR), fsrval);
	}
	if (lsrval & SCLSR_ORER) {
		lsrval = lsrval & ~SCLSR_ORER;
		sil_wrh_mem((uint16_t *)((uint8_t *)p_siopcb->p_siopinib->port + REG_SCLSR), lsrval);
	}
	if (fsrval & SCFSR_RDF) {
		read_data = sil_reb_mem((uint8_t *)p_siopcb->p_siopinib->port + REG_SCFRDR);
		fsrval = fsrval & ~SCFSR_RDF;
		sil_wrh_mem((uint16_t *)((uint8_t *)p_siopcb->p_siopinib->port + REG_SCFSR), fsrval);
		*rxdata = (char)read_data;
		return true;
	}
	return false;
}

/*
 *  送信する文字の書込み
 */
Inline void scif_putchar(SIOPCB *p_siopcb, char c)
{
	sil_wrb_mem((uint8_t *)p_siopcb->p_siopinib->port + REG_SCFTDR, c);
	sil_wrh_mem((uint16_t *)((uint8_t *)p_siopcb->p_siopinib->port + REG_SCFSR), ~(SCFSR_TEND | SCFSR_TDFE));
}

/*
 *  送信割込み許可
 */
Inline void scif_enable_send(SIOPCB *p_siopcb)
{
	uint16_t scrval;

	scrval = sil_reh_mem((uint16_t *)((uint8_t *)p_siopcb->p_siopinib->port + REG_SCSCR));
	scrval |= SCSCR_TIE;
	sil_wrh_mem((uint16_t *)((uint8_t *)p_siopcb->p_siopinib->port + REG_SCSCR), scrval);
}

/*
 *  送信割込み禁止
 */
Inline void scif_disable_send(SIOPCB *p_siopcb)
{
	uint16_t scrval;

	scrval = sil_reh_mem((uint16_t *)((uint8_t *)p_siopcb->p_siopinib->port + REG_SCSCR));
	scrval = scrval & ~SCSCR_TIE;
	sil_wrh_mem((uint16_t *)((uint8_t *)p_siopcb->p_siopinib->port + REG_SCSCR), scrval);
}


/*
 *  受信割込み許可
 */
Inline void scif_enable_rcv(SIOPCB *p_siopcb)
{
	uint16_t scrval;

	scrval = sil_reh_mem((uint16_t *)((uint8_t *)p_siopcb->p_siopinib->port + REG_SCSCR));
	scrval |= SCSCR_RIE;
	sil_wrh_mem((uint16_t *)((uint8_t *)p_siopcb->p_siopinib->port + REG_SCSCR), scrval);
}

/*
 *  受信割込み禁止
 */
Inline void
scif_disable_rcv(SIOPCB *p_siopcb)
{
	uint16_t scrval;

	scrval = sil_reh_mem((uint16_t *)((uint8_t *)p_siopcb->p_siopinib->port + REG_SCSCR));
	scrval = scrval & ~SCSCR_RIE;
	sil_wrh_mem((uint16_t *)((uint8_t *)p_siopcb->p_siopinib->port + REG_SCSCR), scrval);
}

/*
 *  SIOドライバの初期化
 */
void scif_initialize(void)
{
	SIOPCB	*p_siopcb;
	uint_t	i;

	/*
	 *  シリアルI/Oポート管理ブロックの初期化
	 */
	for (p_siopcb = siopcb_table, i = 0; i < TNUM_SIOP; p_siopcb++, i++) {
		p_siopcb->p_siopinib = &(siopinib_table[i]);
		p_siopcb->openflag = false;
		p_siopcb->sendflag = false;
	}
}


/*
 * ハードウェアの初期化処理
 */
static int scif_init_siopinib(const SIOPINIB *p_siopinib)
{
	uint16_t fsrval;
	uint8_t brrval;

	fsrval = UART_CLK / (32 * p_siopinib->bps_setting) - 1;
	if (fsrval > 255) return 1;
	brrval = (uint8_t)fsrval;

	sil_wrh_mem((uint16_t *)((uint8_t *)p_siopinib->port + REG_SCSCR), 0);
	sil_wrh_mem((uint16_t *)((uint8_t *)p_siopinib->port + REG_SCFCR), SCFCR_TFRST | SCFCR_RFRST);
	(void)sil_reh_mem((uint16_t *)((uint8_t *)p_siopinib->port + REG_SCFSR));
	(void)sil_reh_mem((uint16_t *)((uint8_t *)p_siopinib->port + REG_SCLSR));
	sil_wrh_mem((uint16_t *)((uint8_t *)p_siopinib->port + REG_SCFSR), 0);
	sil_wrh_mem((uint16_t *)((uint8_t *)p_siopinib->port + REG_SCLSR), 0);
	sil_wrh_mem((uint16_t *)((uint8_t *)p_siopinib->port + REG_SCSCR), SCSCR_INTCLK);
	sil_wrh_mem((uint16_t *)((uint8_t *)p_siopinib->port + REG_SCSMR), SCSMR_CKS1);	/* 8N1, P1clock/1 */
	sil_wrh_mem((uint16_t *)((uint8_t *)p_siopinib->port + REG_SCEMR), 0);
	sil_wrb_mem((uint8_t *)p_siopinib->port + REG_SCBRR, brrval);
	sil_wrh_mem((uint16_t *)((uint8_t *)p_siopinib->port + REG_SCFCR), SCFCR_RSTRG_15 | SCFCR_RTRG_1 | SCFCR_TTRG_8);
	sil_wrh_mem((uint16_t *)((uint8_t *)p_siopinib->port + REG_SCSCR), SCSCR_TE | SCSCR_RE | SCSCR_INTCLK);
	while (sil_reh_mem((uint16_t *)((uint8_t *)p_siopinib->port + REG_SCFSR)) & SCFSR_RDF) {
		(void)sil_reb_mem((uint8_t *)p_siopinib->port + REG_SCFRDR);
		sil_wrh_mem((uint16_t *)((uint8_t *)p_siopinib->port + REG_SCFSR), ~SCFSR_RDF);
	}
	sil_wrh_mem((uint16_t *)((uint8_t *)p_siopinib->port + REG_SCFSR), 0);
	return 0;
}


/*
 * カーネル起動時のバーナ出力用の初期化
 */
void
scif_init(ID siopid)
{
	SIOPCB		  *p_siopcb   = get_siopcb(siopid);
	const SIOPINIB  *p_siopinib = get_siopinib(siopid);
	/*  この時点では、p_siopcb->p_siopinibは初期化されていない  */

	if (!(p_siopcb->is_initialized)) {
		scif_init_siopinib(p_siopinib);
		p_siopcb->is_initialized = true;
	}
}

/*
 *  ポートnがオープン済みか？（ポートIDが引数）
 */
bool_t
scif_openflag_id(ID siopid)
{
	return(get_siopcb(siopid)->openflag);
}

/*
 *  ポートnがオープン済みか？
 *  （「シリアルI/Oポート管理ブロック」の先頭番地が引数）
 */
bool_t
scif_openflag_cb(SIOPCB *p_siopcb)
{
	return(p_siopcb->openflag);
}

/*
 *  ポートIDの取得
 */
ID
scif_get_siopid(SIOPCB *p_siopcb)
{
	return(SIOPID(p_siopcb));
}

/*
 *  シリアルI/Oポートのオープン
 */
SIOPCB *
scif_opn_por(ID siopid, intptr_t exinf)
{
	SIOPCB		  *p_siopcb;
	const SIOPINIB  *p_siopinib;

	p_siopcb = get_siopcb(siopid);
	p_siopinib = p_siopcb->p_siopinib;

	/*
	 * ハードウェアの初期化
	 * 　既に初期化している場合は、二重に初期化しない。
	 */
	if (!(p_siopcb->is_initialized)) {
		scif_init_siopinib(p_siopinib);
		p_siopcb->is_initialized = true;
	}

	p_siopcb->exinf = exinf;
	p_siopcb->getready = p_siopcb->putready = false;
	p_siopcb->openflag = true;

	return(p_siopcb);
}

/*
 *  シリアルI/Oポートのクローズ
 */
void
scif_cls_por(SIOPCB *p_siopcb)
{
	sil_wrh_mem(p_siopcb->p_siopinib->port + REG_SCSCR, 0);
}


/*
 *  シリアルI/Oポートへのポーリングでの出力
 */
void
scif_pol_putc(char c, ID siopid)
{
	const SIOPINIB *p_siopinib;
	uint16_t fsrval;
	SIL_PRE_LOC;

	p_siopinib = get_siopinib(siopid);

	while(1) {
		/*
		 *  リエントラントにするため、全割込みロック状態にする。
		 */
		SIL_LOC_INT();

		fsrval = sil_reh_mem((uint16_t *)((uint8_t *)p_siopinib->port + REG_SCFSR));
		if (fsrval & SCFSR_TDFE) {
			sil_wrb_mem((uint8_t *)p_siopinib->port + REG_SCFTDR, c);
			sil_wrh_mem((uint16_t *)((uint8_t *)p_siopinib->port + REG_SCFSR), ~(SCFSR_TEND | SCFSR_TDFE));
			/*
			 *  リターンする前に全割込みロックフラグを元の状態に戻す。
			 */
			SIL_UNL_INT();
			return;
		} else {
			/*
			 *  ここで全割込みロックを解除して、割込みを受け付ける。
			 */
			SIL_UNL_INT();
		}
	}
}


/*
 *  シリアルI/Oポートへの文字送信
 */
bool_t
scif_snd_chr(SIOPCB *p_siopcb, char c)
{
	if (scif_putready(p_siopcb)){
		scif_putchar(p_siopcb, c);
		return(true);
	}
	return(false);
}

/*
 *  シリアルI/Oポートからの文字受信
 */
int_t
scif_rcv_chr(SIOPCB *p_siopcb)
{
	char rxdata;
	if (scif_getready(p_siopcb)) {
		if (scif_getchar(p_siopcb, &rxdata)) {
			return((int_t)rxdata);
		}
	}
	return(-1);
}

/*
 *  シリアルI/Oポートからのコールバックの許可
 */
void
scif_ena_cbr(SIOPCB *p_siopcb, uint_t cbrtn)
{
	switch (cbrtn) {
	  case SIO_RDY_SND:
		scif_enable_send(p_siopcb);
		break;
	  case SIO_RDY_RCV:
		scif_enable_rcv(p_siopcb);
		break;
	}
}

/*
 *  シリアルI/Oポートからのコールバックの禁止
 */
void
scif_dis_cbr(SIOPCB *p_siopcb, uint_t cbrtn)
{
	switch (cbrtn) {
	  case SIO_RDY_SND:
		scif_disable_send(p_siopcb);
		break;
	  case SIO_RDY_RCV:
		scif_disable_rcv(p_siopcb);
		break;
	}
}

/*
 *  SIOの割込みサービスルーチン
 */
void
scif_tx_isr(ID siopid)
{
	SIOPCB *p_siopcb = get_siopcb(siopid);

	if (scif_putready(p_siopcb)) {
		scif_irdy_snd(p_siopcb->exinf);
	}
}


/*
 *  SIOの割込みサービスルーチン
 */
void
scif_rx_isr(ID siopid)
{
	SIOPCB *p_siopcb = get_siopcb(siopid);
	while (scif_getready(p_siopcb)) {
		scif_irdy_rcv(p_siopcb->exinf);
	}
}
