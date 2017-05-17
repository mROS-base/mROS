/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 *
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
 *  @(#) $Id: target_config.c 2767 2016-03-14 15:29:03Z ertl-honda $
 */

/*
 * ターゲット依存モジュール（GR-PEACH用）
 */
#include "kernel_impl.h"
#include <sil.h>
#include "arm.h"
#include "scif.h"
#include "chip_timer.h"
//#include "pl310.h"

void
target_mmu_init(void){
	MEMORY_ATTRIBUTE m_attribute;

	/*
	 * Basic initialization of the whole virtual memory space to
	 * non-cachable, non-bufferable, strongly-ordered.
	 */
	m_attribute.pa   = 0x00000000;
	m_attribute.va   = m_attribute.pa;
	m_attribute.size = 0x80000000; /* 2GB */
	m_attribute.ns   = 0;          /* 0=Secure */
	m_attribute.s    = 1;          /* 1=Shared */
	m_attribute.ap   = 3;          /* Full access */
	m_attribute.tex  = 0;          /* Strongly Ordered */
	m_attribute.c    = 0;
	m_attribute.b    = 0;
	mmu_map_memory (&m_attribute);

	m_attribute.pa   = 0x80000000; /* 2GB */
	m_attribute.va   = m_attribute.pa;
	mmu_map_memory (&m_attribute);

	/*
	 *  シリアルフラッシュメモリの設定
	 */
	m_attribute.pa   = ASP_SPI_BASE;
	m_attribute.va   = m_attribute.pa;
	m_attribute.size = ASP_SPI_SIZE;
	m_attribute.ns   = 0;          /* Secure       */
	m_attribute.s    = 0;          /* 非共有       */
	m_attribute.ap   = 3;          /* フルアクセス */
	m_attribute.tex  = 1;          /* Outer and Inner */
	m_attribute.c    = 1;          /* Inner Write-Back, Write Allocate */
	m_attribute.b    = 1;
	mmu_map_memory(&m_attribute);

	/*
	 *  内蔵RAMの設定
	 */
	m_attribute.pa   = ASP_SRAM_BASE;
	m_attribute.va   = m_attribute.pa;
	m_attribute.size = ASP_SRAM_SIZE;
	m_attribute.ns   = 0;          /* Secure       */
	m_attribute.s    = 0;          /* 非共有       */
	m_attribute.ap   = 3;          /* フルアクセス */
	m_attribute.tex  = 1;          /* Outer and Inner */
	m_attribute.c    = 1;          /* Inner Write-Back, Write Allocate */
	m_attribute.b    = 1;
	mmu_map_memory(&m_attribute);

	/*
	 *  I/Oの設定
	 */
	m_attribute.pa   = ASP_IO1_BASE;
	m_attribute.va   = m_attribute.pa;
	m_attribute.size = ASP_IO1_SIZE;
	m_attribute.ns   = 0;          /* Secure       */
	m_attribute.s    = 0;          /* 非共有       */
	m_attribute.ap   = 3;          /* フルアクセス */
	m_attribute.tex  = 0;          /* Strongly-ordered */
	m_attribute.c    = 0;
	m_attribute.b    = 0;
	mmu_map_memory(&m_attribute);

	/*
	 *  I/Oの設定
	 */
	m_attribute.pa   = ASP_IO2_BASE;
	m_attribute.va   = m_attribute.pa;
	m_attribute.size = ASP_IO2_SIZE;
	m_attribute.ns   = 0;          /* Secure       */
	m_attribute.s    = 0;          /* 非共有       */
	m_attribute.ap   = 3;          /* フルアクセス */
	m_attribute.tex  = 0;          /* Strongly-ordered */
	m_attribute.c    = 0;
	m_attribute.b    = 0;
	mmu_map_memory(&m_attribute);

}


/*
 * 低消費電力モードの初期化
 */
static void
lowpower_initialize(void)
{
	sil_wrb_mem((void*)ASP_STBCR2,  0x6A);   /* スタンバイモード時に端子状態を維持する。CoreSight動作 */
	(void)sil_reb_mem((void*)ASP_STBCR2);	 /* dummy read */

	sil_wrb_mem((void*)ASP_STBCR3,  0x00);   /* IEBus, irDA, LIN0, LIN1, MTU2, RSCAN2, ASC, PWM動作 */
	(void)sil_reb_mem((void*)ASP_STBCR3);	 /* dummy read */

	sil_wrb_mem((void*)ASP_STBCR4,  0x00);   /* SCIF0, SCIF1, SCIF2, SCIF3, SCIF4, SCIF5, SCIF6, SCIF7動作 */
	(void)sil_reb_mem((void*)ASP_STBCR4);	 /* dummy read */

	sil_wrb_mem((void*)ASP_STBCR5,  0x00);   /* SCIM0, SCIM1, SDG0, SDG1, SDG2, SDG3, OSTM0, OSTM1動作 */
	(void)sil_reb_mem((void*)ASP_STBCR5);	 /* dummy read */

	sil_wrb_mem((void*)ASP_STBCR6,  0x00);   /* A/D, CEU, DISCOM0, DISCOM1, DRC0, DRC1, JCU, RTClock動作 */
	(void)sil_reb_mem((void*)ASP_STBCR6);	 /* dummy read */

	sil_wrb_mem((void*)ASP_STBCR7,  0x24);   /* DVDEC0, DVDEC1, ETHER, FLCTL, USB0, USB1動作 */
	(void)sil_reb_mem((void*)ASP_STBCR7);	 /* dummy read */

	sil_wrb_mem((void*)ASP_STBCR8,  0x05);   /* IMR-LS20, IMR-LS21, IMR-LSD, MMCIF, MOST50, SCUX動作 */
	(void)sil_reb_mem((void*)ASP_STBCR8);	 /* dummy read */

	sil_wrb_mem((void*)ASP_STBCR9,  0x00);   /* I2C0, I2C1, I2C2, I2C3, SPIBSC0, SPIBSC1, VDC50, VDC51動作 */
	(void)sil_reb_mem((void*)ASP_STBCR9);	 /* dummy read */

	sil_wrb_mem((void*)ASP_STBCR10,  0x00);  /* RSPI0, RSPI1, RSPI2, RSPI3, RSPI4, CD-ROMDEC, RSPDIF, RGPVG動作 */
	(void)sil_reb_mem((void*)ASP_STBCR10);	/* dummy read */

	sil_wrb_mem((void*)ASP_STBCR11,  0xC0);  /* SSIF0, SSIF1, SSIF2, SSIF3, SSIF4, SSIF5動作 */
	(void)sil_reb_mem((void*)ASP_STBCR11);	/* dummy read */

	sil_wrb_mem((void*)ASP_STBCR12,  0xF0);  /* SDHI00, SDHI01, SDHI10, SDHI11動作 */
	(void)sil_reb_mem((void*)ASP_STBCR12);	/* dummy read */
}

/*
 * Set half word register bit 
 *  port : port address
 *  bit  : bit position, 0-15
 *  set  : 0 - reset specified bit / 1 - set specified bit
 */ 
Inline void
SetHPortBit(void *port, int bit, int set)
{
	uint16_t val;
	uint16_t mask;

	if ((bit < 0) || (bit > 15)) {
		return;
	}
	mask = 0x1 << bit;
	val = sil_reh_mem(port);
	val &= ~mask;
	if (set) val |= mask;
	sil_wrh_mem(port, val);
}

/*
 * 汎用入出力ポートの初期化（ポート／ペリフェラル兼用ピンのアサインの設定）
 */
static void
port_initialize(void)
{
	// Configure port P6:bit3(TxD2) - output/Fnction No.7
	SetHPortBit((void *)ASP_PORT_PIBC(6),  3, 0);
	SetHPortBit((void *)ASP_PORT_PBDC(6),  3, 0);
	SetHPortBit((void *)ASP_PORT_PIPC(6),  3, 1);
	// Set Function
	SetHPortBit((void *)ASP_PORT_PMC(6),   3, 1);
	SetHPortBit((void *)ASP_PORT_PFCAE(6), 3, 1);
	SetHPortBit((void *)ASP_PORT_PFCE(6),  3, 1);
	SetHPortBit((void *)ASP_PORT_PFC(6),   3, 0);
	// Set Direction
	SetHPortBit((void *)ASP_PORT_PM(6),    3, 0);

	// Configure port P6:bit2(RxD2) - input/Fnction No.7
	SetHPortBit((void *)ASP_PORT_PIBC(6),  2, 0);
	SetHPortBit((void *)ASP_PORT_PBDC(6),  2, 0);
	SetHPortBit((void *)ASP_PORT_PIPC(6),  2, 1);
	// Set Function
	SetHPortBit((void *)ASP_PORT_PMC(6),   2, 1);
	SetHPortBit((void *)ASP_PORT_PFCAE(6), 2, 1);
	SetHPortBit((void *)ASP_PORT_PFCE(6),  2, 1);
	SetHPortBit((void *)ASP_PORT_PFC(6),   2, 0);
	// Set Direction
	SetHPortBit((void *)ASP_PORT_PM(6),    2, 1);

	// Configure port P6:bit13(LED1/RED) - output
	SetHPortBit((void *)ASP_PORT_PIBC(6),  13, 0);
	SetHPortBit((void *)ASP_PORT_PBDC(6),  13, 0);
	// Set Port Mode
	SetHPortBit((void *)ASP_PORT_PMC(6),   13, 0);
	// Set Direction
	SetHPortBit((void *)ASP_PORT_PM(6),    13, 0);

	// Configure port P6:bit14(LED2/GREEN) - output
	SetHPortBit((void *)ASP_PORT_PIBC(6),  14, 0);
	SetHPortBit((void *)ASP_PORT_PBDC(6),  14, 0);
	// Set Port Mode
	SetHPortBit((void *)ASP_PORT_PMC(6),   14, 0);
	// Set Direction
	SetHPortBit((void *)ASP_PORT_PM(6),    14, 0);

	// Configure port P6:bit15(LED3/BLUE) - output
	SetHPortBit((void *)ASP_PORT_PIBC(6),  15, 0);
	SetHPortBit((void *)ASP_PORT_PBDC(6),  15, 0);
	// Set Port Mode
	SetHPortBit((void *)ASP_PORT_PMC(6),   15, 0);
	// Set Direction
	SetHPortBit((void *)ASP_PORT_PM(6),    15, 0);

	// Configure port P6:bit12(LED4/USER) - output
	SetHPortBit((void *)ASP_PORT_PIBC(6),  12, 0);
	SetHPortBit((void *)ASP_PORT_PBDC(6),  12, 0);
	// Set Port Mode
	SetHPortBit((void *)ASP_PORT_PMC(6),   12, 0);
	// Set Direction
	SetHPortBit((void *)ASP_PORT_PM(6),    12, 0);

	// Configure port P6:bit0(USER BUTTON) - input
	SetHPortBit((void *)ASP_PORT_PIBC(6),  0, 1);
	SetHPortBit((void *)ASP_PORT_PBDC(6),  0, 0);
	// Set Function No.6 input IRQ5
	SetHPortBit((void *)ASP_PORT_PMC(6),   0, 1);
	SetHPortBit((void *)ASP_PORT_PFCAE(6), 0, 1);
	SetHPortBit((void *)ASP_PORT_PFCE(6),  0, 0);
	SetHPortBit((void *)ASP_PORT_PFC(6),   0, 1);
	// Set Port Mode
	SetHPortBit((void *)ASP_PORT_PMC(6),   0, 1);
	// Set Direction
	SetHPortBit((void *)ASP_PORT_PM(6),    0, 1);
}

/*
 *  ターゲット依存の初期化
 */
void
target_initialize(void)
{
	/*
	 *  High exception vector は使用しない．
	 */
	//	set_high_vector(false);

	/*
	 * チップ依存の初期化
	 */
	//	chip_initialize();

	/* 低消費電力モードの初期化 */
	lowpower_initialize();

	/* 汎用入出力ポートの初期化（ポート／ペリフェラル兼用ピンのアサインの設定） */
	port_initialize();
	
	/*
	 * 低レベル出力用にUARTを初期化
	 */
	scif_init(USE_SIO_PORTID);

	/*
	 * 割込みベクタテーブルを VECTOR_TABLE_BASE レジスタに設定する
	 */
	/*
	extern void *vector_table;
	CP15_SET_VBAR((uint32_t) &vector_table);
	pl310_init(0, ~0x0U);
	*/
}

void
set_led(uint8_t led, bool_t on) {
	uint8_t bit_tbl[4] = {13,14,15,12};
	SetHPortBit((void *)ASP_PORT_P(6),  bit_tbl[led], on);
}

bool_t
get_button(void) {
	return(sil_reh_mem((void *)ASP_PORT_PPR(6)) & 0x01);
}

/*
 *  ターゲット依存の終了処理
 */
void
target_exit(void)
{
	/*
	 *  チップ依存の終了処理
	 */
	chip_exit();

	while(1);
}

/*
 *  システムログの低レベル出力のための文字出力
 */
void
target_fput_log(char c)
{
	if (c == '\n') {
		scif_pol_putc('\r', USE_SIO_PORTID);
	}
	scif_pol_putc(c, USE_SIO_PORTID);
}
