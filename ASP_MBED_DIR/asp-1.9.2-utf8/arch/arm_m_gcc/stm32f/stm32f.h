/*
 *  TOPPERS/JSP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Just Standard Profile Kernel
 *
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2005-2011 by Embedded and Real-Time Systems Laboratory
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 * 
 *  上記著作権者は，Free Software Foundation によって公表されている 
 *  GNU General Public License の Version 2 に記述されている条件か，以
 *  下の(1)～(4)の条件を満たす場合に限り，本ソフトウェア（本ソフトウェ
 *  アを改変したものを含む．以下同じ）を使用・複製・改変・再配布（以下，
 *  利用と呼ぶ）することを無償で許諾する．
 *  (1) 本ソフトウェアをソースコードの形で利用する場合には，上記の著作
 *      権表示，この利用条件および下記の無保証規定が，そのままの形でソー
 *      スコード中に含まれていること．
 *  (2) 本ソフトウェアを再利用可能なバイナリコード（リロケータブルオブ
 *      ジェクトファイルやライブラリなど）の形で利用する場合には，利用
 *      に伴うドキュメント（利用者マニュアルなど）に，上記の著作権表示，
 *      この利用条件および下記の無保証規定を掲載すること．
 *  (3) 本ソフトウェアを再利用不可能なバイナリコードの形または機器に組
 *      み込んだ形で利用する場合には，次のいずれかの条件を満たすこと．
 *    (a) 利用に伴うドキュメント（利用者マニュアルなど）に，上記の著作
 *        権表示，この利用条件および下記の無保証規定を掲載すること．
 *    (b) 利用の形態を，別に定める方法によって，上記著作権者に報告する
 *        こと．
 *  (4) 本ソフトウェアの利用により直接的または間接的に生じるいかなる損
 *      害からも，上記著作権者を免責すること．
 * 
 *  本ソフトウェアは，無保証で提供されているものである．上記著作権者は，
 *  本ソフトウェアに関して，その適用可能性も含めて，いかなる保証も行わ
 *  ない．また，本ソフトウェアの利用により直接的または間接的に生じたい
 *  かなる損害に関しても，その責任を負わない．
 * 
 */
#ifndef TOPPERS_ST32F_H
#define TOPPERS_ST32F_H

#include <sil.h>

/*
 * ARM-M依存部のインクルード
 */
#include "arm_m_gcc/common/arm_m.h"

/*
 *  割込み番号の最大値
 */
#define TMAX_INTNO   (16 + 42)

/*
 *  割込み優先度ビット幅中のサブ優先度のビット幅
 */
#define TBITW_SUBIPRI  0

/*
 *  割込みベクタ番号定義
 */
#define IRQ_VECTOR_USART1		(16 + 37)
#define IRQ_VECTOR_USART2		(16 + 38)


/*
 * ペリフェラルレジスタ定義
 */
#define PERIPH_REG_BASE			(0x40000000UL)
#define SRAM_BASE				(0x20000000UL)

#define APB1_PERIPH				(PERIPH_REG_BASE)
#define APB2_PERIPH				(PERIPH_REG_BASE + 0x10000)
#define AHB_PERIPH				(PERIPH_REG_BASE + 0x20000)

/* BUS:APB1 */
#define TIM2_BASE				(APB1_PERIPH)
#define TIM3_BASE				(APB1_PERIPH + 0x400)
#define TIM4_BASE				(APB1_PERIPH + 0x800)
#define RTC_BASE				(APB1_PERIPH + 0x2800)
#define WWDG_BASE				(APB1_PERIPH + 0x2C00)
#define IWDG_BASE				(APB1_PERIPH + 0x3000)
#define SPI2_BASE				(APB1_PERIPH + 0x3800)
#define USART2_BASE				(APB1_PERIPH + 0x4400)
#define USART3_BASE				(APB1_PERIPH + 0x4800)
#define I2C1_BASE				(APB1_PERIPH + 0x5400)
#define I2C2_BASE				(APB1_PERIPH + 0x5800)
#define USB_BASE				(APB1_PERIPH + 0x5C00)
#define CAN_BASE				(APB1_PERIPH + 0x6400)
#define BKP_BASE				(APB1_PERIPH + 0x6C00)
#define PWR_BASE				(APB1_PERIPH + 0x7000)

/* BUS:APB2 */
#define AFIO_BASE				(APB2_PERIPH)
#define EXTI_BASE				(APB2_PERIPH + 0x400)
#define GPIOA_BASE				(APB2_PERIPH + 0x800)
#define GPIOB_BASE				(APB2_PERIPH + 0xC00)
#define GPIOC_BASE				(APB2_PERIPH + 0x1000)
#define GPIOD_BASE				(APB2_PERIPH + 0x1400)
#define GPIOE_BASE				(APB2_PERIPH + 0x1800)
#define ADC1_BASE				(APB2_PERIPH + 0x2400)
#define ADC2_BASE				(APB2_PERIPH + 0x2800)
#define TIM1_BASE				(APB2_PERIPH + 0x2C00)
#define SPI1_BASE				(APB2_PERIPH + 0x3000)
#define USART1_BASE				(APB2_PERIPH + 0x3800)

/* BUS:AHB */
#define DMA_BASE				(AHB_PERIPH)
#define RCC_BASE				(AHB_PERIPH + 0x1000)
#define FLASH_BASE				(AHB_PERIPH + 0x2000)

/* System Control space */
#define SCS_BASE				(0xE000E000)
#define SYSTM_BASE				(SCS_BASE + 0x0010)
#define NVIC_BASE				(SCS_BASE + 0x0100)
#define SYSCB_BASE				(SCS_BASE + 0x0D00)

/* RCC */
#define RCC_CR					(RCC_BASE)
#define RCC_CFGR				(RCC_BASE + 0x04)
#define RCC_CIR					(RCC_BASE + 0x08)
#define RCC_APB2RSTR			(RCC_BASE + 0x0C)
#define RCC_APB1RSTR			(RCC_BASE + 0x10)
#define RCC_AHBENR				(RCC_BASE + 0x14)
#define RCC_APB2ENR				(RCC_BASE + 0x18)
#define RCC_APB1ENR				(RCC_BASE + 0x1C)
#define RCC_BDCR				(RCC_BASE + 0x20)
#define RCC_CSR					(RCC_BASE + 0x24)


/* NVIC */
#define NVIC_ENAVLE_REG(ch)		(NVIC_BASE + ((ch) >> 5))
#define NVIC_DISABLE_REG(ch)	(NVIC_BASE + 0x80 + ((ch) >> 5))
#define NVIC_SET_PEND_REG(ch)	(NVIC_BASE + 0x100 + ((ch) >> 5))
#define NVIC_CLEAR_PEND_REG(ch)	(NVIC_BASE + 0x180 + ((ch) >> 5))
#define NVIC_ACTIVE_REG(ch)		(NVIC_BASE + 0x200 + ((ch) >> 5))
#define NVIC_PRIO_REG(ch)		(NVIC_BASE + 0x300 + ((ch) >> 2))

/* GPIOx */
#define GPIO_CRL(x)				(x)
#define GPIO_CRH(x)				((x) + 0x04)
#define GPIO_IDR(x)				((x) + 0x08)
#define GPIO_ODR(x)				((x) + 0x0C)
#define GPIO_BSRR(x)			((x) + 0x10)
#define GPIO_BRR(x)				((x) + 0x14)
#define GPIO_LCKR(x)			((x) + 0x18)

/* AFIO */
#define AFIO_EVCR				(AFIO_BASE)
#define AFIO_MAPR				(AFIO_BASE + 0x04)
#define AFIO_EXTICR1			(AFIO_BASE + 0x08)
#define AFIO_EXTICR2			(AFIO_BASE + 0x0C)
#define AFIO_EXTICR3			(AFIO_BASE + 0x10)
#define AFIO_EXTICR4			(AFIO_BASE + 0x14)

/* FLASH */
#define FLASH_ACR				(FLASH_BASE)

/* RCCレジスタ定義 */
#define CR_PLL_RDY				(0x02000000)
#define CR_PLL_ON				(0x01000000)
#define CR_HSE_RDY				(0x00020000)
#define CR_HSE_ON				(0x00010000)
#define CR_HSI_RDY				(0x00000002)
#define CR_HSI_ON				(0x00000001)
#define CFGR_PLLMUL_MASK		(0x003C0000)
#define CFGR_PLL_XTPRE			(0x00020000)
#define CFGR_PLL_SRC			(0x00010000)
#define CFGR_HPRE_MASK			(0x000000F0)
#define CFGR_PPRE2_MASK			(0x00003800)
#define CFGR_PPRE1_MASK			(0x00000700)
#define CFGR_SWS_MASK			(0x0000000C)
#define CFGR_SW_MASK			(0x00000003)
#define CFGR_SW_PLL				(0x02)
#define APB1ENR_USART2_EN		(0x00020000)
#define APB2ENR_ADC3_EN			(0x8000)
#define APB2ENR_USART1_EN		(0x4000)
#define APB2ENR_TIM8_EN			(0x2000)
#define APB2ENR_SPI1_EN			(0x1000)
#define APB2ENR_TIM1_EN			(0x0800)
#define APB2ENR_ADC2_EN			(0x0400)
#define APB2ENR_ADC1_EN			(0x0200)
#define APB2ENR_IOPG_EN			(0x0100)
#define APB2ENR_IOPF_EN			(0x0080)
#define APB2ENR_IOPE_EN			(0x0040)
#define APB2ENR_IOPD_EN			(0x0020)
#define APB2ENR_IOPC_EN			(0x0010)
#define APB2ENR_IOPB_EN			(0x0008)
#define APB2ENR_IOPA_EN			(0x0004)
#define APB2ENR_AFIO_EN			(0x0001)
#define AHBENR_SDIO_EN			(0x0400)
#define AHBENR_FSMC_EN			(0x0100)
#define AHBENR_CRCE_EN			(0x0040)
#define AHBENR_FLITF_EN			(0x0010)
#define AHBENR_SRAM_EN			(0x0004)
#define AHBENR_DMA_EN			(0x0001)

/* FLASHレジスタ定義 */
#define ACR_PRFTBE				(0x10)
#define ACR_LATENCY_MASK		(0x07)
#define ACR_LATENCY_ZERO		(0x00)
#define ACR_LATENCY_ONE			(0x01)
#define ACR_LATENCY_TWO			(0x02)

/* GPIOxレジスタ定義 */
#define CNF_IN_ANALOG			(0x00)
#define CNF_IN_FLOATING			(0x01)
#define CNF_IN_PULL				(0x02)
#define CNF_OUT_GP_PP			(0x00)
#define CNF_OUT_GP_OD			(0x01)
#define CNF_OUT_AF_PP			(0x02)
#define CNF_OUT_AF_OD			(0x03)
#define MODE_INPUT				(0x00)
#define MODE_OUTPUT_10MHZ		(0x01)
#define MODE_OUTPUT_2MHZ		(0x02)
#define MODE_OUTPUT_50MHZ		(0x03)

#define CR_MODE_MASK(x)			(0x03 << ((x) << 2))
#define CR_CNF_MASK(x)			(0x0C << ((x) << 2))
#define CR_MODE(x,v)			(((v) & 0x03) << ((x) << 2))
#define CR_CNF(x,v)				((((v) << 2) & 0x0C) << ((x) << 2))

/* AFIOレジスタ定義 */
#define MAPR_USART2_REMAP		(0x0008)

#ifndef TOPPERS_MACRO_ONLY
/*
 * GPIOレジスタ操作関数
 */
Inline void
set_cr_mode(uint32_t reg, uint_t p, int_t v)
{
	if (p < 8) {
		sil_andw((void*)GPIO_CRL(reg), ~CR_MODE_MASK(p));
		sil_orw((void*)GPIO_CRL(reg), CR_MODE(p, v));
	} else if (8 <= p && p < 16) {
		sil_andw((void*)GPIO_CRH(reg), ~CR_MODE_MASK(p - 8));
		sil_orw((void*)GPIO_CRH(reg), CR_MODE(p - 8, v));
	}
}

Inline void
set_cr_cnf(uint32_t reg, uint_t p, int_t v)
{
	if (p < 8) {
		sil_andw((void*)GPIO_CRL(reg), ~CR_CNF_MASK(p));
		sil_orw((void*)GPIO_CRL(reg), CR_CNF(p, v));
	} else if (8 <= p && p < 16) {
		sil_andw((void*)GPIO_CRH(reg), ~CR_CNF_MASK(p - 8));
		sil_orw((void*)GPIO_CRH(reg), CR_CNF(p - 8, v));
	}
}

Inline void
set_port_pull(uint32_t reg, uint_t p, bool_t up)
{
	if (up) {
		sil_wrw_mem((void*)GPIO_BSRR(reg), 0x01 << p);
	} else {
		sil_wrw_mem((void*)GPIO_BRR(reg), 0x01 << p);
	}
}

#endif /* TOPPERS_MACRO_ONLY */
#endif /* TOPPERS_ST32F_H */
