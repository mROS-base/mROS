/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2006-2011 by Embedded and Real-Time Systems Laboratory
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
 *  @(#) $Id: scif.c 2742 2016-01-09 04:25:18Z ertl-honda $
 */

/*
 *   SH SCIF用 簡易SIOドライバ
 */

#include <sil.h>
#include "target_syssvc.h"
#include "scif.h"

/*
 * SH SCIFのレジスタ設定
 */

/*
 * 各レジスタのオフセット
 */
#define SCIFA_SCSMR   0x0000U  /* H */
#define SCIFA_SCBRR   0x0004U  /* B */
#define SCIFA_SCSCR   0x0008U  /* H */
#define SCIFA_SCFSR   0x0014U  /* H */
#define SCIFA_SCFCR   0x0018U  /* H */
#define SCIFA_SCFDR   0x001cU  /* H */
#define SCIFA_SCFTDR  0x0020U  /* B */
#define SCIFA_SCFRDR  0x0024U  /* B */


#define SCIFB_SCSMR   0x0000U  /* H */
#define SCIFB_SCBRR   0x0004U  /* B */
#define SCIFB_SCSCR   0x0008U  /* H */
#define SCIFB_SCFSR   0x0014U  /* H */
#define SCIFB_SCFCR   0x0018U  /* H */
#define SCIFB_SCFDR   0x003cU  /* H */
#define SCIFB_SCFTDR  0x0040U  /* B */
#define SCIFB_SCFRDR  0x0060U  /* B */

/*
 * 各レジスタの設定値
 */
#define SCSMR_CHR    0x0040U
#define SCSMR_PE     0x0020U
#define SCSMR_OE     0x0010U
#define SCSMR_STOP   0x0008U
#define SCSMR_CKS1   0x0002U
#define SCSMR_CKS0   0x0001U

#define SCSCR_TIE    0x0080U
#define SCSCR_RIE    0x0040U
#define SCSCR_TE     0x0020U
#define SCSCR_RE     0x0010U
#define SCSCR_CKE1   0x0002U
#define SCSCR_CKE0   0x0001U

#define SCFSR_ER     0x0080U
#define SCFSR_TEND   0x0040U
#define SCFSR_TDFE   0x0020U
#define SCFSR_BRK    0x0010U
#define SCFSR_FER    0x0008U
#define SCFSR_PER    0x0004U
#define SCFSR_RDF    0x0002U
#define SCFSR_DR     0x0001U

#define SCFCR_RTRG1  0x0080U
#define SCFCR_RTRG0  0x0040U
#define SCFCR_TTRG1  0x0020U
#define SCFCR_TTRG0  0x0010U
#define SCFCR_MCE    0x0008U
#define SCFCR_TFRST  0x0004U
#define SCFCR_RFRST  0x0002U
#define SCFCR_LOOP   0x0001U

#define SIO_INIT_DLY    5000U /* 1ビット分がよく分からないので暫定値 */

/*
 *  シリアルI/Oポート初期化ブロックの定義
 */
typedef struct sio_port_initialization_block {
	uint16_t *scsmr_h;   /* シリアルモードレジスタ */
	uint8_t  *scbrr_b;   /* ビットレートレジスタ */
	uint16_t *scscr_h;   /* シリアルコントロールレジスタ */
	uint16_t *scfsr_h;   /* シリアルステータスレジスタ */
	uint16_t *scfcr_h;   /* FIFOコントロールレジスタ */
	uint16_t *scfdr_h;   /* レシーブFIFOデータ数レジスタ */
	uint8_t  *scftdr_b;  /* トランスミットFIFOデータレジスタ */
	uint8_t  *scfrdr_b;  /* レシーブFIFOデータレジスタ */
	uint32_t bps_setting; /* ボーレートの設定値 */
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
#ifdef USE_SCIFA0
	{
		(uint16_t *)(SCIFA0_BASE + SCIFA_SCSMR),
		(uint8_t  *)(SCIFA0_BASE + SCIFA_SCBRR),
		(uint16_t *)(SCIFA0_BASE + SCIFA_SCSCR),
		(uint16_t *)(SCIFA0_BASE + SCIFA_SCFSR),
		(uint16_t *)(SCIFA0_BASE + SCIFA_SCFCR),
		(uint16_t *)(SCIFA0_BASE + SCIFA_SCFDR),
		(uint8_t  *)(SCIFA0_BASE + SCIFA_SCFTDR),
		(uint8_t  *)(SCIFA0_BASE + SCIFA_SCFRDR),
		SCIFA0_BPS_SETTING,
	},
#endif /*  USE_SCIFA0 */
#ifdef USE_SCIFA1
	{        
		(uint16_t *)(SCIFA1_BASE + SCIFA_SCSMR),
		(uint8_t  *)(SCIFA1_BASE + SCIFA_SCBRR),
		(uint16_t *)(SCIFA1_BASE + SCIFA_SCSCR),
		(uint16_t *)(SCIFA1_BASE + SCIFA_SCFSR),
		(uint16_t *)(SCIFA1_BASE + SCIFA_SCFCR),
		(uint16_t *)(SCIFA1_BASE + SCIFA_SCFDR),
		(uint8_t  *)(SCIFA1_BASE + SCIFA_SCFTDR),
		(uint8_t  *)(SCIFA1_BASE + SCIFA_SCFRDR),
		SCIFA1_BPS_SETTING,
	},
#endif /* USE_SCIFA1 */
#ifdef USE_SCIFA2
	{
		(uint16_t *)(SCIFA2_BASE + SCIFA_SCSMR),
		(uint8_t  *)(SCIFA2_BASE + SCIFA_SCBRR),
		(uint16_t *)(SCIFA2_BASE + SCIFA_SCSCR),
		(uint16_t *)(SCIFA2_BASE + SCIFA_SCFSR),
		(uint16_t *)(SCIFA2_BASE + SCIFA_SCFCR),
		(uint16_t *)(SCIFA2_BASE + SCIFA_SCFDR),
		(uint8_t  *)(SCIFA2_BASE + SCIFA_SCFTDR),
		(uint8_t  *)(SCIFA2_BASE + SCIFA_SCFRDR),
		SCIFA2_BPS_SETTING,
	},
#endif /* USE_SCIFA2 */
#ifdef USE_SCIFA3
	{
		(uint16_t *)(SCIFA3_BASE + SCIFA_SCSMR),
		(uint8_t  *)(SCIFA3_BASE + SCIFA_SCBRR),
		(uint16_t *)(SCIFA3_BASE + SCIFA_SCSCR),
		(uint16_t *)(SCIFA3_BASE + SCIFA_SCFSR),
		(uint16_t *)(SCIFA3_BASE + SCIFA_SCFCR),
		(uint16_t *)(SCIFA3_BASE + SCIFA_SCFDR),
		(uint8_t  *)(SCIFA3_BASE + SCIFA_SCFTDR),
		(uint8_t  *)(SCIFA3_BASE + SCIFA_SCFRDR),
		SCIFA3_BPS_SETTING,
	},
#endif /* USE_SCIFA3 */
#ifdef USE_SCIFA4
	{
		(uint16_t *)(SCIFA4_BASE + SCIFA_SCSMR),
		(uint8_t  *)(SCIFA4_BASE + SCIFA_SCBRR),
		(uint16_t *)(SCIFA4_BASE + SCIFA_SCSCR),
		(uint16_t *)(SCIFA4_BASE + SCIFA_SCFSR),
		(uint16_t *)(SCIFA4_BASE + SCIFA_SCFCR),
		(uint16_t *)(SCIFA4_BASE + SCIFA_SCFDR),
		(uint8_t  *)(SCIFA4_BASE + SCIFA_SCFTDR),
		(uint8_t  *)(SCIFA4_BASE + SCIFA_SCFRDR),
		SCIFA4_BPS_SETTING,
	},
#endif /* USE_SCIFA4 */
#ifdef USE_SCIFA5
	{
		(uint16_t *)(SCIFA5_BASE + SCIFA_SCSMR),
		(uint8_t  *)(SCIFA5_BASE + SCIFA_SCBRR),
		(uint16_t *)(SCIFA5_BASE + SCIFA_SCSCR),
		(uint16_t *)(SCIFA5_BASE + SCIFA_SCFSR),
		(uint16_t *)(SCIFA5_BASE + SCIFA_SCFCR),
		(uint16_t *)(SCIFA5_BASE + SCIFA_SCFDR),
		(uint8_t  *)(SCIFA5_BASE + SCIFA_SCFTDR),
		(uint8_t  *)(SCIFA5_BASE + SCIFA_SCFRDR),
		SCIFA5_BPS_SETTING,
	},
#endif /* USE_SCIFA5 */
#ifdef USE_SCIFA6
	{
		(uint16_t *)(SCIFA6_BASE + SCIFA_SCSMR),
		(uint8_t  *)(SCIFA6_BASE + SCIFA_SCBRR),
		(uint16_t *)(SCIFA6_BASE + SCIFA_SCSCR),
		(uint16_t *)(SCIFA6_BASE + SCIFA_SCFSR),
		(uint16_t *)(SCIFA6_BASE + SCIFA_SCFCR),
		(uint16_t *)(SCIFA6_BASE + SCIFA_SCFDR),
		(uint8_t  *)(SCIFA6_BASE + SCIFA_SCFTDR),
		(uint8_t  *)(SCIFA6_BASE + SCIFA_SCFRDR),
		SCIFA6_BPS_SETTING,
	},
#endif /* USE_SCIFA6 */
#ifdef USE_SCIFA7
	{
		(uint16_t *)(SCIFA7_BASE + SCIFA_SCSMR),
		(uint8_t  *)(SCIFA7_BASE + SCIFA_SCBRR),
		(uint16_t *)(SCIFA7_BASE + SCIFA_SCSCR),
		(uint16_t *)(SCIFA7_BASE + SCIFA_SCFSR),
		(uint16_t *)(SCIFA7_BASE + SCIFA_SCFCR),
		(uint16_t *)(SCIFA7_BASE + SCIFA_SCFDR),
		(uint8_t  *)(SCIFA7_BASE + SCIFA_SCFTDR),
		(uint8_t  *)(SCIFA7_BASE + SCIFA_SCFRDR),
		SCIFA7_BPS_SETTING,
	},
#endif /* USE_SCIFA7 */
#ifdef USE_SCIFB
	{
		(uint16_t *)(SCIFB_BASE + SCIFB_SCSMR),
		(uint8_t  *)(SCIFB_BASE + SCIFB_SCBRR),
		(uint16_t *)(SCIFB_BASE + SCIFB_SCSCR),
		(uint16_t *)(SCIFB_BASE + SCIFB_SCFSR),
		(uint16_t *)(SCIFB_BASE + SCIFB_SCFCR),
		(uint16_t *)(SCIFB_BASE + SCIFB_SCFDR),
		(uint8_t  *)(SCIFB_BASE + SCIFB_SCFTDR),
		(uint8_t  *)(SCIFB_BASE + SCIFB_SCFRDR),
		SCIFB_BPS_SETTING,
	},
#endif /* USE_SCIFB */
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
Inline bool_t
scif_getready(SIOPCB *p_siopcb)
{
    return((sil_reh_mem(p_siopcb->p_siopinib->scfsr_h) & SCFSR_RDF) != 0);
}

/*
 *  文字を送信できるか？
 */
Inline bool_t
scif_putready(SIOPCB *p_siopcb)
{
    return((sil_reh_mem(p_siopcb->p_siopinib->scfsr_h) & SCFSR_TDFE) != 0);
}

/*
 *  受信した文字の取出し
 */
Inline char
scif_getchar(SIOPCB *p_siopcb)
{
    uint8_t c;
    c = sil_reb_mem(p_siopcb->p_siopinib->scfrdr_b);
    sil_wrh_mem(p_siopcb->p_siopinib->scfsr_h,
                (sil_reh_mem(p_siopcb->p_siopinib->scfsr_h) & ~SCFSR_RDF));
    return(c);
}

/*
 *  送信する文字の書込み
 */
Inline void
scif_putchar(SIOPCB *p_siopcb, char c)
{
    sil_wrb_mem(p_siopcb->p_siopinib->scftdr_b, (uint8_t)c);
    sil_wrh_mem(p_siopcb->p_siopinib->scfsr_h,
                (sil_reh_mem(p_siopcb->p_siopinib->scfsr_h) & ~(SCFSR_TDFE)));
}

/*
 *  送信割込み許可
 */
Inline void
scif_enable_send(SIOPCB *p_siopcb)
{
    sil_wrh_mem(p_siopcb->p_siopinib->scscr_h,
                (sil_reh_mem(p_siopcb->p_siopinib->scscr_h) | SCSCR_TIE));
}

/*
 *  送信割込み禁止
 */
Inline void
scif_disable_send(SIOPCB *p_siopcb)
{
    sil_wrh_mem(p_siopcb->p_siopinib->scscr_h,
                (sil_reh_mem(p_siopcb->p_siopinib->scscr_h) & ~SCSCR_TIE));
}


/*
 *  受信割込み許可
 */
Inline void
scif_enable_rcv(SIOPCB *p_siopcb)
{
    sil_wrh_mem(p_siopcb->p_siopinib->scscr_h,
                (sil_reh_mem(p_siopcb->p_siopinib->scscr_h) | SCSCR_RIE));    
}

/*
 *  受信割込み禁止
 */
Inline void
scif_disable_rcv(SIOPCB *p_siopcb)
{
    sil_wrh_mem(p_siopcb->p_siopinib->scscr_h,
                (sil_reh_mem(p_siopcb->p_siopinib->scscr_h) & ~SCSCR_RIE));
}

/*
 *  SIOドライバの初期化
 */
void
scif_initialize(void)
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
static void
scif_init_siopinib(const SIOPINIB *p_siopinib)
{
    /* UART停止 */
	sil_wrh_mem(p_siopinib->scscr_h, 0x00U);    

    /* 受信送信FIFOをクリア */   
	sil_wrh_mem(p_siopinib->scfcr_h, (SCFCR_TFRST|SCFCR_RFRST));

	/*  8データ,1ストップビット,no parity */
	sil_wrh_mem(p_siopinib->scsmr_h, 0x0000U);

	/*  ^RTS2 に出力 */
	//sil_wrh_mem(p_siopinib->scsptr_h, SCSPTR_RTSIO);

#ifndef USE_SCIF_CLK_EXT
	/* ボーレートを設定 */
	//sil_wrb_mem(p_siopinib->scbrr_b, p_siopinib->bps_setting);
	//sil_wrb_mem(p_siopinib->scbrr_b, 0x1a);	
	sil_wrb_mem(p_siopinib->scbrr_b, (uint16_t)((50 * 1000000 / (16 * 1 * (p_siopinib->bps_setting))) - 1));
	/* 1ビット分待つ必要がある */
	sil_dly_nse(SIO_INIT_DLY);
#else /* USE_SCIF_CLK_EXT */
	sil_wrh_mem(p_siopinib->cks_h, CKS_EXTERNAL);
	sil_wrh_mem(p_siopinib->dl_h, SCIF_CLK / p_siopinib->bps_setting / 16);
	sil_dly_nse((1000000 * 2 * 16 / SCIF_CLK) * 1000 + 1);
#endif /* USE_SCIF_CLK_EXT */

	/* 受信バッファのクリア */
	while((sil_reh_mem(p_siopinib->scfsr_h) & SCFSR_RDF) == SCFSR_RDF) {
		sil_reb_mem(p_siopinib->scfrdr_b);
		sil_wrh_mem(p_siopinib->scfsr_h,
					(sil_reh_mem(p_siopinib->scfsr_h) & ~SCFSR_RDF));
	}

	/* ステータスのクリア */
	sil_wrh_mem(p_siopinib->scfsr_h,
				sil_reh_mem(p_siopinib->scfsr_h) & ~(SCFSR_TEND|SCFSR_TDFE));
    
	/* トリガの設定 R-FIFO=1,T-FIFO=1 */
	sil_wrh_mem(p_siopinib->scfcr_h, (SCFCR_TTRG1|SCFCR_TTRG0));

	/* UART開始 */
#ifndef USE_SCIF_CLK_EXT
	sil_wrh_mem(p_siopinib->scscr_h, (SCSCR_TE|SCSCR_RE));
#else /* USE_SCIF_CLK_EXT */
	sil_wrh_mem(p_siopinib->scscr_h, (SCSCR_TE|SCSCR_RE|SCSCR_CKE1));
#endif /* USE_SCIF_CLK_EXT */
}


/*
 * カーネル起動時のバーナ出力用の初期化
 */
void
scif_init(ID siopid)
{
	SIOPCB          *p_siopcb   = get_siopcb(siopid);
	const SIOPINIB  *p_siopinib = get_siopinib(siopid);
	/*  この時点では、p_siopcb->p_siopinibは初期化されていない  */
    
	/*  二重初期化の防止  */
	p_siopcb->is_initialized = true;

	/*  ハードウェアの初期化処理  */
	scif_init_siopinib(p_siopinib);
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
	SIOPCB          *p_siopcb;
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
	/*
	 * UART停止
	 */
	sil_wrh_mem(p_siopcb->p_siopinib->scscr_h, 0x00);
	p_siopcb->openflag = false;
	p_siopcb->is_initialized = false;
}


/*
 *  シリアルI/Oポートへのポーリングでの出力
 */
void
scif_pol_putc(char c, ID siopid)
{
	const SIOPINIB *p_siopinib;
	uint32_t scfsr;
	void *p_scfsr;
	SIL_PRE_LOC;
    
	p_siopinib = get_siopinib(siopid);
	p_scfsr = p_siopinib->scfsr_h;
    
	while(1) {
		/*
		 *  リエントラントにするため、全割込みロック状態にする。
		 */
		SIL_LOC_INT();
		scfsr = sil_reh_mem(p_scfsr);
		
		if ((scfsr & SCFSR_TDFE) == 0) {
			/*
			 *  ここで全割込みロックを解除して、割込みを受け付ける。
			 */
			SIL_UNL_INT();
		} else {
			sil_wrb_mem(p_siopinib->scftdr_b, (uint8_t)c);
			scfsr = sil_reh_mem(p_scfsr);
			sil_wrh_mem(p_scfsr, scfsr & ~SCFSR_TDFE);
			/*
			 *  リターンする前に全割込みロックフラグを元の状態に戻す。
			 */
			SIL_UNL_INT();
			return;
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
	if (scif_getready(p_siopcb)) {
		return((int_t)scif_getchar(p_siopcb));
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
	SIOPCB	*p_siopcb = get_siopcb(siopid);

	if (scif_putready(p_siopcb)) {
		/*
		 *  送信可能コールバックルーチンを呼び出す．
		 */
		scif_irdy_snd(p_siopcb->exinf);
	}
}


/*
 *  SIOの割込みサービスルーチン
 */
void
scif_rx_isr(ID siopid)
{
	SIOPCB	*p_siopcb = get_siopcb(siopid);
    
	if (scif_getready(p_siopcb)) {
		/*
		 *  受信通知コールバックルーチンを呼び出す．
		 */
		scif_irdy_rcv(p_siopcb->exinf);
	}
}
