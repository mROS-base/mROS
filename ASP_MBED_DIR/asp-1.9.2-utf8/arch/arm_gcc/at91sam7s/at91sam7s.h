/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 *
 *  Copyright (C) 2006 by GJ Business Division RICOH COMPANY,LTD. JAPAN
 *  Copyright (C) 2007-2013 by Embedded and Real-Time Systems Laboratory
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
 *  @(#) $Id: at91sam7s.h 2742 2016-01-09 04:25:18Z ertl-honda $
 */

#ifndef TOPPERS_AT91SAM7S_H
#define TOPPERS_AT91SAM7S_H

#include <sil.h>

/*
 * 割込みハンドラ番号から，IRC操作のためのビットパターンを求めるマクロ
 */
#define INTNO_BITPAT(intno) (1U << intno)

/*
 *  ADVANCED INTERRUPT CONTROLLER
 */
#define TADR_AIC_BASE   0xFFFFF000	/* AIC base address */
#define TOFF_AIC_SMR    0x0000		/* Source Mode Register0-31 (R/W) */
  #define AIC_PRIOR                        (7<<0)	/* Priority Level */
  #define AIC_PRIOR_LOWEST                 (0)		/* Lowest priority level */
  #define AIC_PRIOR_HIGHEST                (7)		/* Highest priority level */
  #define AIC_SRCTYPE                      (3<<5)	/* Interrupt Source Type */
  #define AIC_SRCTYPE_INT_LEVEL_SENSITIVE  (0<<5)	/* Internal Sources Code Label Level Sensitive */
  #define AIC_SRCTYPE_INT_EDGE_TRIGGERED   (1<<5)	/* Internal Sources Code Label Edge triggered */
  #define AIC_SRCTYPE_EXT_HIGH_LEVEL       (2<<5)	/* External Sources Code Label High-level Sensitive */
  #define AIC_SRCTYPE_EXT_POSITIVE_EDGE    (3<<5)	/* External Sources Code Label Positive Edge triggered */
#define TOFF_AIC_SVR    0x0080		/* Source Vector Register0-31 (R/W) */
#define TOFF_AIC_IVR    0x0100		/* Interrupt Vector Register (R) */
#define TOFF_AIC_FVR    0x0104		/* Fast Interrupt Vector Register (R) */
#define TOFF_AIC_ISR    0x0108		/* Interrupt Status Register (R) */
#define TOFF_AIC_IPR    0x010C		/* Interrupt Pending Register (R) */
#define TOFF_AIC_IMR    0x0110		/* Interrupt Mask Register (R) */
#define TOFF_AIC_CISR   0x0114		/* Core Interrupt Status Register (R) */
#define TOFF_AIC_IECR   0x0120		/* Interrupt Enable Command Register (W) */
#define TOFF_AIC_IDCR   0x0124		/* Interruot Disable Command Register (W) */
#define TOFF_AIC_ICCR   0x0128		/* Interrupt Clear Command Register (W) */
#define TOFF_AIC_ISCR   0x012C		/* Interrupt Set Command Register (W) */
#define TOFF_AIC_EOICR  0x0130		/* End of Interrupt Command Register (W) */
#define TOFF_AIC_SPU    0x0134		/* Spurios Interrupt Vector Register (R/W */
#define TOFF_AIC_DCR    0x0138		/* Debug Control Register (R/W) */
#define TOFF_AIC_FFER   0x0140		/* Fast Forcing Enable Register (W) */
#define TOFF_AIC_FFDR   0x0144		/* Fast Forcing Disable Register (W) */
#define TOFF_AIC_FFSR   0x0148		/* Fast Forcing Status Register (R) */

#define INTNO_FIQ_PID     0
#define INTNO_SYSIRQ_PID  1
#define INTNO_PIOA_PID    2
#define INTNO_ADC_PID     4
#define INTNO_SPI_PID     5
#define INTNO_US0_PID     6
#define INTNO_US1_PID     7
#define INTNO_SSC_PID     8
#define INTNO_TWI_PID     9
#define INTNO_PWM_PID     10
#define INTNO_UDP_PID     11
#if defined (__AT91SAM7A3__)
#define INTNO_TC0_PID     15
#define INTNO_TC1_PID     16
#define INTNO_TC2_PID     17
#else
#define INTNO_TC0_PID     12
#define INTNO_TC1_PID     13
#define INTNO_TC2_PID     14
#endif /* __AT91SAM7A3__ */
#define INTNO_IRQ0_PID    30
#define INTNO_IRQ1_PID    31

/*
 *  DEBUG UNIT Debug Unit
 */
#define TADR_DBGU_BASE  0xFFFFF200		/* Debug Unit BASE address */
#define TOFF_DBGU_CR    0x0000			/* Control Register (W):TOFF_US_CR */
#define TOFF_DBGU_MR    0x0004			/* Mode Register (R/W):TOFF_US_MR */
#define TOFF_DBGU_IER   0x0008			/* Interrupt Enable Register (W):TOFF_US_IER */
#define TOFF_DBGU_IDR   0x000C			/* Interrupt Disable Register (W):TOFF_US_IDR */
#define TOFF_DBGU_IMR   0x0010			/* Interrupt Mask Regiser (R):TOFF_US_IMR */
#define TOFF_DBGU_SR    0x0014			/* Status Register (R):TOFF_US_CSR */
#define TOFF_DBGU_RHR   0x0018			/* Receive Holding Register (R):TOFF_US_RHR */
#define TOFF_DBGU_THR   0x001C			/* Transmit Holding Register (W):TOFF_US_THR */
#define TOFF_DBGU_BRGR  0x0020			/* Baud Rate Generator Register (R/W):TOFF_US_BRGR */
#define TOFF_DBGU_CIDR  0x0040			/* Chip ID Register (R) */
#define TOFF_DBGU_EXID  0x0044			/* Chip ID Extension Register (R) */
#define TOFF_DBGU_FNR   0x0048			/* Force NTRST Register (R/W) */


/*
 *  PARALLEL INPUT/OUTPUT CONTROLLER
 */
#define TADR_PIO_BASE	0xFFFFF400		/* PIO BASE ADDRESS */
#define TOFF_PIO_PER    0x0000			/* PIO Enable Register (W) */
#define TOFF_PIO_PDR    0x0004			/* PIO Disable Register (W) */
#define TOFF_PIO_PSR    0x0008			/* PIO Status Register (R) */
#define TOFF_PIO_OER    0x0010			/* Output Enable Register (W) */
#define TOFF_PIO_ODR    0x0014			/* Output Disable Register (W) */
#define TOFF_PIO_OSR    0x0018			/* Output Status Register (R) */
#define TOFF_PIO_IFER   0x0020			/* Glitch Input Filter Enable Register (W) */
#define TOFF_PIO_IFDR   0x0024			/* Glitch Input Filter Disable Register (W) */
#define TOFF_PIO_IFSR   0x0028			/* Glitch Input Filter Status Register (R) */
#define TOFF_PIO_SODR   0x0030			/* Set Output Data Register (W) */
#define TOFF_PIO_CODR   0x0034			/* Clear Output Data Register (W) */
#define TOFF_PIO_ODSR   0x0038			/* Output Data Status Register (R) */
#define TOFF_PIO_PDSR   0x003C			/* Pin Data Status Register (R) */
#define TOFF_PIO_IER    0x0040			/* Interrupt Enable Register (W) */
#define TOFF_PIO_IDR    0x0044			/* Interrupt Disable Register (W) */
#define TOFF_PIO_IMR    0x0048			/* Interrupt Mask Register (R) */
#define TOFF_PIO_ISR    0x004C			/* Interrupt Status Register (R) */
#define TOFF_PIO_MDER   0x0050			/* Multi-driver Enable Register (W) */
#define TOFF_PIO_MDDR   0x0054			/* Multi-driver Disable Register (W) */
#define TOFF_PIO_MDSR   0x0058			/* Multi-driver Status Register (R) */
#define TOFF_PIO_PUDR   0x0060			/* Pull-up Disable Register (W) */
#define TOFF_PIO_PUER   0x0064			/* Pull-up Enable Register (W) */
#define TOFF_PIO_PUSR   0x0068			/* Pad Pull-up Statuse Register (R) */
#define TOFF_PIO_ASR    0x0070			/* Peripheral A Select Register (W) */
#define TOFF_PIO_BSR    0x0074			/* Peripheral B Select Register (W) */
#define TOFF_PIO_ABSR   0x0078			/* AB Status Register (R) */
#define TOFF_PIO_OWER   0x00A0			/* Output Write Enable (W) */
#define TOFF_PIO_OWDR   0x00A4			/* Output Write Disable (W) */
#define TOFF_PIO_OWSR   0x00A8			/* Output Write Status Register (R) */

/*
 *  POWER MANAGMENT CONTROLLER
 */
#define TADR_PMC_BASE   0xFFFFFC00		/* PMC BASE ADDRESS */
#define TOFF_PMC_SCER   0x0000			/* System Clock Enable Register (W) */
#define TOFF_PMC_SCDR   0x0004			/* System Clock Disable Register (W) */
#define TOFF_PMC_SCSR   0x0008			/* System Clock Status Register (R) */
#define TOFF_PMC_PCER   0x0010			/* Peripheral Clock Enable Register (W) */
#define TOFF_PMC_PCDR   0x0014			/* Peripheral Clock Disable Register (W) */
#define TOFF_PMC_PCSR   0x0018			/* Peripheral Clock Status Register (W) */
#define TOFF_CKGR_MOR   0x0020			/* Main Oscillator Register (W) */
  #define CKGR_MOR_MOSCEN         (1<<0)
  #define CKGR_MOR_OSCBYPASS      (1<<1)
  #define CKGR_MOR_OSCOUNT_SHIFT  8
#define TOFF_CKGR_MCFR  0x0024			/* Main Clock Frequency Register (R) */
#define TOFF_CKGR_PLLR  0x002C			/* PLL Register (R/W) */
  #define CKGR_PLLR_DIV_SHIFT      0
  #define CKGR_PLLR_PLLCOUNT_SHIFT 8
  #define CKGR_PLLR_MUL_SHIFT      16
#define TOFF_PMC_MCKR   0x0030			/* Master Clock Register (R/W) */
  #define PMC_MCKR_CSS_PLL_CLOCK  (3<<0)
  #define PMC_MCKR_PRES_CLK_2     (1<<2)
#define TOFF_PMC_PCK0   0x0040			/* Programmable Clock 0 Register (R/W) */
#define TOFF_PMC_PCK1   0x0044			/* Programmable Clock 1 Register (R/W) */
#define TOFF_PMC_IER    0x0060			/* Interrupt Enable Register (W) */
#define TOFF_PMC_IDR    0x0064			/* Interrupt Disable Register (W) */
#define TOFF_PMC_SR     0x0068			/* Status Register (R) */
  #define PMC_SR_MOSCS            (1<<0)
  #define PMC_SR_LOCK             (1<<2)
#define TOFF_PMC_IMR    0x006C			/* Interrupt Mask Register (R) */

/*
 *  RESET CONTROLLER (RSTC)
 */
#define TADR_BASE_RSTC  0xFFFFFD00		/* RSTC BASE Address */
#define TOFF_RSTC_CR    0x0000			/* Reset Controller Control Register (W) */
#define TOFF_RSTC_SR    0x0004			/* Reset Controller Status Register (R) */
#define TOFF_RSTC_MR    0x0008			/* Reset Controller Mode Register (R/W) */

/*
 *  REAL-TIME TIMER (RTT)
 */
#define TADR_RTT_BASE   0xFFFFFD20		/* Real-time Timer BASE address */
#define TOFF_RTT_MR     0x0000			/* Mode Register (R/W) */
#define TOFF_RTT_AR     0x0004			/* Alarm Register (R/W) */
#define TOFF_RTT_VR     0x0008			/* Value Register (R) */
#define TOFF_RTT_SR     0x000C			/* Status Register (R) */

/*
 *  PERIODIC INTERVAL TIMER (PIT)
 */
#define TADR_PIT_BASE    0xFFFFFD30		/* Periodic Interval Timer BASE Address */
#define TOFF_PIT_MR      0x0000			/* Mode Register (R/W) */
#define TOFF_PIT_SR      0x0004			/* Status Register (R) */
#define TOFF_PIT_PIVR    0x0008			/* Periodic Interval Value Register (R) */
#define TOFF_PIT_PIIR    0x000C			/* Periodic Interval Image Register (R) */

/*
 *  WATCHDOG TIMER (WDT)
 */
#define TADR_WDT_BASE    0xFFFFFD40		/* Watchdog Timer BASE address */
#define TOFF_WDT_CR      0x0000			/* Watchdog Timer Control Register (W) */
#define TOFF_WDT_MR      0x0004			/* Watchdog Timer Mode Register (R/W) */
  #define WDT_MR_WDDIS           (1<<15)
#define TOFF_WDT_SR      0x0008			/* Watchdog Timer Status Register (R) */

/*
 *  VOLTAGE REGULATOR POWER CONTROLLER (VREG)
 */
#define TADR_VREG_BASE   0xFFFFFD60		/* VREG BASE address */
#define TOFF_VREG_MR     0x0000			/* Voltage Regulator Mode Register (R/W) */
/*
 *  MEMORY CONTROLLER (MC)
 */
#define TADR_MC_BASE     0xFFFFFF00		/* Memory Controller BASE address */
#define TOFF_MC_RCR      0x0000			/* MC Remap Control Register (W) */
#define TOFF_MC_ASR      0x0004			/* MC Abort Status Register (R) */
#define TOFF_MC_AASR     0x0008			/* MC Abort Address Status Register (R) */
#define TOFF_MC_FMR      0x0060			/* MC Flash Mode Register(R/W) */
  #define MC_FMR_FWS_0FWS        (0<<8)
  #define MC_FMR_FWS_1FWS        (1<<8)
  #define MC_FMR_FWS_2FWS        (2<<8)
  #define MC_FMR_FWS_3FWS        (3<<8)
  #define MC_FMR_FMCN_SHIFT      16
#define TOFF_MC_FCR      0x0064			/* MC Flash Command Register (W) */
#define TOFF_MC_FSR      0x0068			/* MC Flash Status Register (R) */

/*
 *  TIMER COUNTER
 */
#define TADR_TC_BASE	0xFFFA0000		/* Timer Counter BASE ADDRESS */
#define TC_WINDOW       0x0040			/* Timer Counter window size */
#define TOFF_TC_CCR     0x0000			/* Channel Control Register (W) */
  #define TC_CLKEN              (1<<0)	/* (TC) Counter Clock Enable Command */
  #define TC_CLKDIS             (1<<1)	/* (TC) Counter Clock Disable Command */
  #define TC_SWTRG              (1<<2)	/* (TC) Software Trigger Command */
#define TOFF_TC_CMR     0x0004			/* Channel Mode Register (R/W) */
  #define TC_CLKS               0x7
  #define TC_CLKS_MCK2          0x0
  #define TC_CLKS_MCK8          0x1
  #define TC_CLKS_MCK32         0x2
  #define TC_CLKS_MCK128        0x3
  #define TC_CLKS_MCK1024       0x4
  #define TC_WAVESEL00          (0<<13)	/* (TC) UP mode without atomatic trigger on RC Compare */
  #define TC_WAVESEL01          (1<<13)	/* (TC) UPDOWN mode without automatic trigger on RC Compare */
  #define TC_WAVESEL10          (2<<13)	/* (TC) UP mode with automatic trigger on RC Compare */
  #define TC_WAVESEL11          (3<<13)	/* (TC) UPDOWN mode with automatic trigger on RC Compare */
#define TOFF_TC_CV      0x0010			/* Counter Value (R) */
#define TOFF_TC_RA      0x0014			/* Register A (R/W) */
#define TOFF_TC_RB      0x0018			/* Register B (R/W) */
#define TOFF_TC_RC      0x001C			/* Register C (R/W) */
#define TOFF_TC_SR      0x0020			/* Statis Register (R) */
  #define TC_COVFS              (1<<0)	/* (TC) Counter Overflow */
  #define TC_LOVRS              (1<<1)	/* (TC) Load Overrun */
  #define TC_CPAS               (1<<2)	/* (TC) RA Compare */
  #define TC_CPBS               (1<<3)	/* (TC) RB Compare */
  #define TC_CPCS               (1<<4)	/* (TC) RC Compare */
  #define TC_LDRAS              (1<<5)	/* (TC) RA Loading */
  #define TC_LDRBS              (1<<6)	/* (TC) RB Loading */
  #define TC_ETRGS              (1<<7)	/* (TC) External Trigger */
  #define TC_CLKSTA             (1<<16)	/* (TC) Clock Enabling */
  #define TC_MTIOA              (1<<17)	/* (TC) TIOA Mirror */
  #define TC_MTIOB              (1<<18)	/* (TC) TIOA Mirror */
#define TOFF_TC_IER     0x0024			/* Interrupt Enable Register (W) */
#define TOFF_TC_IDR     0x0028			/* Interrupt Disable Register (W) */
#define TOFF_TC_IMR     0x002C			/* Interrupt Mask Register (R) */
#define TOFF_TC_BCR     0x00C0			/* TC Block Control Register (W) */
#define TOFF_TC_BMR     0x00C4			/* TC Block Mode Register (R/W) */

/*
 *  USB DEVICE PORT (UDP)
 */
#define TADR_UDP_BASE    0xFFFB0000		/* USB Device Port BASE Address */
#define TOFF_UDP_FRM_NUM  0x0000		/* Frame Number Register (R) */
  #define UDP_FRM_NUM     (0x7FF)		/* Frame Number as Defined in the Packet Field Formats */
  #define UDP_FRM_ERR     (1<< 16)		/* Frame Error */
  #define UDP_FRM_OK      (1<< 17)		/* Frame OK */
#define TOFF_UDP_GLB_STAT 0x0004		/* Global State Register (R/W) */
  #define UDP_FADDEN      (1<<0)		/* Function Address Enable */
  #define UDP_CONFG       (1<<1)		/* Configured */
  #define UDP_ESR         (1<<2)		/* Enable Send Resume */
  #define UDP_RSMINPR     (1<<3)		/* A Resume Has Been Sent to the Host */
  #define UDP_RMWUPE      (1<<4)		/* Remote Wake Up Enable */
#define TOFF_UDP_FADDR    0x0008		/* Function Address Register (R/W) */
  #define UDP_FADD        (0xFF<<0)		/* Function Address Value */
  #define UDP_FEN         (   1<<8)		/* Function Enable */
#define TOFF_UDP_IER      0x0010		/* Interrupt Enable Register (W) */
  #define UDP_IEPINT0     (1<<0)		/* Endpoint 0 Interrupt */
  #define UDP_IEPINT1     (1<<1)		/* Endpoint 0 Interrupt */
  #define UDP_IEPINT2     (1<<2)		/* Endpoint 2 Interrupt */
  #define UDP_IEPINT3     (1<<3)		/* Endpoint 3 Interrupt */
  #define UDP_IEPINT4     (1<<4)		/* Endpoint 4 Interrupt */
  #define UDP_IEPINT5     (1<<5)		/* Endpoint 5 Interrupt */
  #define UDP_IEPINT6     (1<<6)		/* Endpoint 6 Interrupt */
  #define UDP_IEPINT7     (1<<7)		/* Endpoint 7 Interrupt */
  #define UDP_IRXSUSP     (1<<8)		/* USB Suspend Interrupt */
  #define UDP_IRXRSM      (1<<9)		/* USB Resume Interrupt */
  #define UDP_IEXTRSM     (1<<10)		/* USB External Resume Interrupt */
  #define UDP_ISOFINT     (1<<11)		/* USB Start Of frame Interrupt */
  #define UDP_IWAKEUP     (1<<13)		/* USB Walkup Interrupt */
#define TOFF_UDP_IDR      0x0014		/* Interrupt Disable Register (W) */
#define TOFF_UDP_IMR      0x0018		/* Interrupt Mask Register (R) */
#define TOFF_UDP_ISR      0x001C		/* Interrupt Status Register (R) */
  #define UDP_ENDBUSRES   (1<<12)		/* USB End Of Bus Reset Interrupt */
#define TOFF_UDP_ICR      0x0020		/* Interrupt Clear Register (W) */
#define TOFF_UDP_RST_EP   0x0028		/* Reset Endpoint Register (R/W) */
  #define UDP_EP0         (1<<0)		/* Reset Endpoint 0 */
  #define UDP_EP1         (1<<1)		/* Reset Endpoint 1 */
  #define UDP_EP2         (1<<2)		/* Reset Endpoint 2 */
  #define UDP_EP3         (1<<3)		/* Reset Endpoint 3 */
  #define UDP_EP4         (1<<4)		/* Reset Endpoint 4 */
  #define UDP_EP5         (1<<5)		/* Reset Endpoint 5 */
  #define UDP_EP6         (1<<6)		/* Reset Endpoint 6 */
  #define UDP_EP7         (1<<7)		/* Reset Endpoint 7 */
#define TOFF_UDP_CSR      0x0030		/* Endpoint Control Status Register (R/W) */
#define TOFF_UDP_CSR0     0x0030		/* Endpoint0 Control Status Register (R/W) */
#define TOFF_UDP_CSR1     0x0034		/* Endpoint1 Control Status Register (R/W) */
#define TOFF_UDP_CSR2     0x0038		/* Endpoint2 Control Status Register (R/W) */
#define TOFF_UDP_CSR3     0x003C		/* Endpoint3 Control Status Register (R/W) */
  #define UDP_TXCOMP      (1<<0)		/* Generates an IN packet with data previously written in the DPR */
  #define UDP_RX_DATA_BK0 (1<<1)		/* Receive Data Bank 0 */
  #define UDP_RXSETUP     (1<<2)		/* Sends STALL to the Host (Control endpoints) */
  #define UDP_ISOERROR    (1<<3)		/* Isochronous error (Isochronous endpoints) */
  #define UDP_TXPKTRDY    (1<<4)		/* Transmit Packet Ready */
  #define UDP_FORCESTALL  (1<<5)		/* Force Stall (used by Control, Bulk and Isochronous endpoints). */
  #define UDP_RX_DATA_BK1 (1<<6)		/* Receive Data Bank 1 (only used by endpoints with ping-pong attributes). */
  #define UDP_DIR         (1<<7)		/* Transfer Direction */
  #define UDP_EPTYPE      (7<<8)		/* Endpoint type */
  #define UDP_EPTYPE_CTRL     (0<<8)	/* Control */
  #define UDP_EPTYPE_ISO_OUT  (1<<8)	/* Isochronous OUT */
  #define UDP_EPTYPE_BULK_OUT (2<<8)	/* Bulk OUT */
  #define UDP_EPTYPE_INT_OUT  (3<<8)	/* Interrupt OUT */
  #define UDP_EPTYPE_ISO_IN   (5<<8)	/* Isochronous IN */
  #define UDP_EPTYPE_BULK_IN  (6<<8)	/* Bulk IN */
  #define UDP_EPTYPE_INT_IN   (7<<8)	/* Interrupt IN */
  #define UDP_DTGLE       (1<<11)		/* Data Toggle */
  #define UDP_EPEDS       (1<<15)		/* Endpoint Enable Disable */
  #define UDP_RXBYTECNT   (0x7FF<<16)	/* Number Of Bytes Available in the FIFO */
#define TOFF_UDP_FDR      0x0050		/* Endpoint FIFO Data Register (R/W) */
#define TOFF_UDP_FDR0     0x0050		/* Endpoint0 FIFO Data Register (R/W) */
#define TOFF_UDP_FDR1     0x0054		/* Endpoint1 FIFO Data Register (R/W) */
#define TOFF_UDP_FDR2     0x0058		/* Endpoint2 FIFO Data Register (R/W) */
#define TOFF_UDP_FDR3     0x005C		/* Endpoint3 FIFO Data Register (R/W) */
#define TOFF_UDP_TXVC     0x0074		/* Transmitter Control Register (R/W) */
  #define UDP_TXVDIS      (1<<8)		/* */
  #define UDP_PUON        (1<<9)		/* Pull-up ON */

/*
 * TWO-WIRE INTERFACE (TWI)
 */
#define TADR_TWI_BASE    0xFFFB8000		/* Two-wire Interface BASE address */
#define TOFF_TWI_CR      0x0000			/* Control Register (W) */
#define TOFF_TWI_MMR     0x0004			/* Master Mode Register (R/W) */
#define TOFF_TWI_IADR    0x000C			/* Internal Address Register (R/W) */
#define TOFF_TWI_CWGR    0x0010			/* Clock Wavefrom Generator Register (R/W) */
#define TOFF_TWI_SR      0x0020			/* Status Register (R) */
#define TOFF_TWI_IER     0x0024			/* Interrupt Enable Register (W) */
#define TOFF_TWI_IDR     0x0028			/* Interrupt Disable Register (W) */
#define TOFF_TWI_IMR     0x002C			/* Interrupt Mask Register (R) */
#define TOFF_TWI_RHR     0x0030			/* Receive Holding Register (R) */
#define TOFF_TWI_THR     0x0034			/* Transmit Holding Register (R/W) */

/*
 *  UNIVERSAL SYNCHRONOUS ASYNCHRONOUS RECEIVER TRANSMITTER(USART)
 */
#define TADR_US_BASE    0xFFFC0000		/* USART BASE address */
#define US_WINDOW       0x4000			/* USART Window size */
#define TOFF_US_CR      0x0000			/* Control Register (W) */
  #define US_RSTRX      0x0004			/* Reset Receiver */
  #define US_RSTTX      0x0008			/* Reset Transmitter */
  #define US_RXEN       0x0010			/* Receiver Enable */
  #define US_RXDIS      0x0020			/* Receiver Disable */
  #define US_TXEN       0x0040			/* Transmitter Enable */
  #define US_TXDIS      0x0080			/* Transmitter Disable */
  #define US_RSTSTA     0x0100			/* Reset Status Bits */
  #define US_STTBRK     0x0200			/* Start Break */
  #define US_STPBRK     0x0400			/* Stop Break */
  #define US_STTTO      0x0800			/* Start Time-out */
  #define US_SENDA      0x1000			/* Send Address */
#define TOFF_US_MR      0x0004			/* Mode Register (R/W) */
  #define US_CLKS       0x0030			/* Clock Selection */
  #define US_CLKS_MCK        0x0000		/* Master Clock */
  #define US_CLKS_MCK8       0x0010		/* Master Clock divided by 8 */
  #define US_CLKS_SCK        0x0020		/* External Clock */
  #define US_CLKS_SLCK       0x0030		/* Slow Clock */
  #define US_CHRL       0x00C0			/* Byte Length */
  #define US_CHRL_5          0x0000		/* 5 bits */
  #define US_CHRL_6          0x0040		/* 6 bits */
  #define US_CHRL_7          0x0080		/* 7 bits */
  #define US_CHRL_8          0x00C0		/* 8 bits */
  #define US_PAR        0x0E00			/* Parity Mode */
  #define US_PAR_EVEN         0x0000	/* Even Parity */
  #define US_PAR_ODD          0x0200	/* Odd Parity */
  #define US_PAR_SPACE        0x0400	/* Space Parity to 0 */
  #define US_PAR_MARK         0x0600	/* Marked Parity to 1 */
  #define US_PAR_NO           0x0800	/* No Parity */
  #define US_PAR_MULTIDROP    0x0C00	/* Multi-drop Mode */
  #define US_NBSTOP     0x3000			/* Stop Bit Number */
  #define US_NBSTOP_1         0x0000	/* 1 Stop Bit */
  #define US_NBSTOP_1_5       0x1000	/* 1.5 Stop Bits */
  #define US_NBSTOP_2         0x2000	/* 2 Stop Bits */
  #define US_CHMODE     0xC000			/* Channel Mode */
  #define US_CHMODE_NORMAL          0x0000	/* Normal Mode */
  #define US_CHMODE_AUTOMATIC_ECHO  0x4000  /* Automatic Echo */
  #define US_CHMODE_LOCAL_LOOPBACK  0x8000  /* Local Loopback */
  #define US_CHMODE_REMOTE_LOOPBACK 0xC000  /* Remote Loopback */
#define TOFF_US_IER     0x0008			/* Interrupt Enable Register (W) */
#define TOFF_US_IDR     0x000C			/* Interrupt Disable Register (W) */
#define TOFF_US_IMR     0x0010			/* Interrupt Mask Register (R) */
#define TOFF_US_CSR     0x0014			/* Channel Staus Register (R) */
  #define US_RXRDY      0x0001			/* Receiver Ready */
  #define US_TXRDY      0x0002			/* Transmitter Ready */
  #define US_RXBRK      0x0004			/* Receiver Break */
  #define US_ENDRX      0x0008			/* End of Receiver PDC Transfer */
  #define US_ENDTX      0x0010			/* End of Transmitter PDC Transfer */
  #define US_OVRE       0x0020			/* Overrun Error */
  #define US_FRAME      0x0040			/* Framing Error */
  #define US_PARE       0x0080			/* Parity Error */
  #define US_TIMEOUT    0x0100			/* Receiver Timeout */
  #define US_TXEMPTY    0x0200			/* Transmitter Empty */
#define TOFF_US_RHR     0x0018			/* Receiver Holding Register (R) */
#define TOFF_US_THR     0x001C			/* Transmitter Holding Register (W) */
#define TOFF_US_BRGR    0x0020			/* Baud Rate Generator Register (R/W) */

#define TOFF_US_RTOR    0x0024			/* Receiver Time-out Register (R/W) */
#define TOFF_US_TTGR    0x0028			/* Transmitter Timeguard Register (R/W) */
#define TOFF_US_FIDI    0x0040			/* FIDI Ratio Register (R/W) */
#define TOFF_US_NER     0x0044			/* Number of Errors Register (R) */
#define TOFF_US_IF      0x004C			/* IrDA Filter Register (R/W) */
#define TOFF_US_MAN     0x0050			/* Manchester Encoder Decoder Register (R/W) */

/*
 *  PUSE WIDTH MODULATION CONTROLLER
 */
#define TADR_PWM_BASE   0xFFFCC000		/* Pluse Widh Modulation Controller BASE address */
#define TOFF_PWM_MR     0x0000			/* PWM Mode Register (R/W) */
#define TOFF_PWM_ENA    0x0004			/* PWM Enable Register (W) */
#define TOFF_PWM_DIS    0x0008			/* PWM Disable Register (W) */
#define TOFF_PWM_SR     0x000C			/* PWM Status Register (R) */
#define TOFF_PWM_IER    0x0010			/* PWM Interrupt Enable Register (W) */
#define TOFF_PWM_IDR    0x0014			/* PWM Interrupt Disable Register (W) */
#define TOFF_PWM_IMR    0x0018			/* PWM Interrupt Mask Register (R) */
#define TOFF_PWM_ISR    0x001C			/* PWM Interrupt Status Register (R) */
#define TADR_PWMC_BASE  0xFFFCC200		/* PWM Channel0 BASE address */
#define CN_WINDOW       0x0020			/* PWM Channel Window size */
#define TADR_PWMC0_BASE (TADR_PWMC_BASE)
#define TADR_PWMC1_BASE (TADR_PWMC_BASE+CN_WINDOW)
#define TOFF_PWM_CMR    0x0000			/* PWM Channel Mode Register (R/W) */
#define TOFF_PWM_CDTY   0x0004			/* PWM Channel Duty Cycle Register (R/W) */
#define TOFF_PWM_CPRD   0x0008			/* PWM Channel Period Register (R/W) */
#define TOFF_PWM_CCNT   0x000C			/* PWM Channel Counter Register (R) */
#define TOFF_PWM_CUPD   0x0010			/* PWM Channel Update Register (W) */


/*
 *  SYNCHRONOUS SERIAL CONTROLLER (SSC)
 */
#define TADR_SSC_BASE    0xFFFD4000		/* Synchronous Serial Controller BASE Address */
#define TOFF_SSC_CR      0x0000			/* Control Register (W) */
#define TOFF_SSC_CMR     0x0004			/* Clock Mode Register (R/W) */
#define TOFF_SSC_RCMR    0x0010			/* Receive Clock Mode Register (R/W) */
#define TOFF_SSC_RFMR    0x0014			/* Receive Frame Mode Register (R/W) */
#define TOFF_SSC_TCMR    0x0018			/* Transmit Clock Mode Register (R/W) */
#define TOFF_SSC_TFMR    0x001C			/* Transmit Frame Mode Register (R/W) */
#define TOFF_SSC_RHR     0x0020			/* Receive Holding Register (R) */
#define TOFF_SSC_THR     0x0024			/* Transmit Holding Register (R) */
#define TOFF_SSC_RSHR    0x0030			/* Receive Sync. Holding Register (R) */
#define TOFF_SSC_TSHR    0x0034			/* Transmit Sync. Holding Register (R/W) */
#define TOFF_SSC_RC0R    0x0038			/* Receive Compare 0 Register (R/W) */
#define TOFF_SSC_RC1R    0x003C			/* Receive Compare 1 Register (R/W) */
#define TOFF_SSC_SR      0x0040			/* Status Register (R) */
#define TOFF_SSC_IER     0x0044			/* Interrupt Enable Register (W) */
#define TOFF_SSC_IDR     0x0048			/* Interrupt Disable Register (W) */
#define TOFF_SSC_IMR     0x004C			/* Interrupt Mask Register (R) */


/*
 *  ANALOG-TO-DIGITAL CONVERTER (ADC)
 */
#define TADR_ADC_BASE    0xFFFD8000		/* Analog-to-digital Converter BASE address */
#define TOFF_ADC_CR      0x0000			/* Control Register (W) */
#define TOFF_ADC_MR      0x0004			/* Mode Register (R/W) */
#define TOFF_ADC_CHER    0x0010			/* Channel Enable Register (W) */
#define TOFF_ADC_CHDR    0x0014			/* Channel Disable Register (W) */
#define TOFF_ADC_CHSR    0x0018			/* Channel Status Register (R) */
#define TOFF_ADC_SR      0x001C			/* Status Register (R) */
#define TOFF_ADC_LCDR    0x0020			/* Last Converted Data Register (R) */
#define TOFF_ADC_IER     0x0024			/* Interrupt Enable Register (W) */
#define TOFF_ADC_IDR     0x0028			/* Interrupt Disable Register (W) */
#define TOFF_ADC_IMR     0x002C			/* Interrupt Mask Register (R) */
#define TOFF_ADC_CDR     0x0030			/* Channel Data Register (R) */
#define TOFF_ADC_CDR0    0x0030			/* Channel Data Register0 (R) */
#define TOFF_ADC_CDR1    0x0034			/* Channel Data Register1 (R) */
#define TOFF_ADC_CDR2    0x0038			/* Channel Data Register2 (R) */
#define TOFF_ADC_CDR3    0x003C			/* Channel Data Register3 (R) */
#define TOFF_ADC_CDR4    0x0040			/* Channel Data Register4 (R) */
#define TOFF_ADC_CDR5    0x0044			/* Channel Data Register5 (R) */
#define TOFF_ADC_CDR6    0x0048			/* Channel Data Register6 (R) */
#define TOFF_ADC_CDR7    0x004C			/* Channel Data Register7 (R) */


/*
 *  SERIAL PERIPHERAL INTERFACE (SPI)
 */
#define TADR_SPI_BASE    0xFFFE0000		/* Serial Peripheral Interfcae BASE Address */
#define TOFF_SPI_CR      0x0000			/* Control Register (W) */
#define TOFF_SPI_MR      0x0004			/* Mode Register (R/W) */
#define TOFF_SPI_RDR     0x0008			/* Receive Data Register (R) */
#define TOFF_SPI_TDR     0x000C			/* Transmit Data Register (W) */
#define TOFF_SPI_SR      0x0010			/* Status Register (R) */
#define TOFF_SPI_IER     0x0014			/* Interrupt Enable Register (W) */
#define TOFF_SPI_IDR     0x0018			/* Interrupt Disable Register (W) */
#define TOFF_SPI_IMR     0x001C			/* Interrupt Mask Register (R) */
#define TOFF_SPI_CSR0    0x0030			/* Chip Select Register0 (R/W) */
#define TOFF_SPI_CSR1    0x0034			/* Chip Select Register1 (R/W) */
#define TOFF_SPI_CSR2    0x0038			/* Chip Select Register2 (R/W) */
#define TOFF_SPI_CSR3    0x003C			/* Chip Select Register3 (R/W) */

/*
 *  PERIPHERAL DMA CONTROLLER (PDC)
 */
#define TOFF_PDC_RPR    0x0100			/* Receive Pointer Register (R/W) */
#define TOFF_PDC_RCR    0x0104			/* Receive Counter Register (R/W) */
#define TOFF_PDC_TPR    0x0108			/* Transmit Pointer Register (R/W) */
#define TOFF_PDC_TCR    0x010C			/* Transmit Counter Register (R/W) */
#define TOFF_PDC_RNPR   0x0110			/* Receive Next Pointer Register (R/W) */
#define TOFF_PDC_RNCR   0x0114			/* Receive Next Counter Register (R/W) */
#define TOFF_PDC_TNPR   0x0118			/* Transmit Next Pointer Register (R/W) */
#define TOFF_PDC_TNCR   0x011C			/* Transmit Next Counter Register (R/W) */
#define TOFF_PDC_PTCR   0x0120			/* PDC Transfar Control Register (W) */
#define TOFF_PDC_PTSR   0x0124			/* PDC Transfar Status Register (R) */


#if defined (__AT91SAM7S128__)
// 128kbytes,512pages of 256bytes
#define FLASH_PAGE_NB           512
#define FLASH_PAGE_LOCK         64
#define FLASH_PAGE_SIZE         256
#define FLASH_PAGE_SIZE_BYTE    256
#define FLASH_PAGE_SIZE_LONG    64
// 8lockbits, protecting 8sectors of 64pages
#define FLASH_LOCK_BITS_SECTOR  8
#define FLASH_SECTOR_PAGE       64
#define FLASH_LOCK_BITS         8

#define SRAM_SIZE               (32U*1024U)

#elif defined (__AT91SAM7S256__)
// 256kbytes,1024pages of 256bytes
#define  FLASH_PAGE_NB          1024
#define  FLASH_PAGE_LOCK        65
#define  FLASH_PAGE_SIZE        256
#define  FLASH_PAGE_SIZE_BYTE   256
#define  FLASH_PAGE_SIZE_LONG   64
// 16lockbits, protecting 16sectors of 64pages
#define  FLASH_LOCK_BITS_SECTOR 16
#define  FLASH_SECTOR_PAGE      64
#define  FLASH_LOCK_BITS        16

#define SRAM_SIZE               (64U*1024U)

#endif
#define FLASH_BASE_ADDRESS      0x00100000
#define SRAM_BASE_ADDRESSS      0x00200000

#define MCK	                    48054857

/*
 * ボーレート
 */
#define BAUDRATE   38400

#if defined (__AT91SAM7A3__)
#define TOFF_PIO_PDR_VAL   ((1<<30U)|(1<<31U))
#else
#define TOFF_PIO_PDR_VAL   ((1<<9U)|(1<<10U))
#endif /* AT91SAM7A3 */

#ifndef TOPPERS_MACRO_ONLY
        
/*
 * IRC操作関数
 */

/*
 * 割込み要求のマスク
 */
Inline void
at91sam7s_disable_int(uint32_t mask)
{
    sil_wrw_mem((void *)(TADR_AIC_BASE + TOFF_AIC_IDCR), mask);	
}

/*
 * 割込み要求のマスクの解除
 */
Inline void
at91sam7s_enable_int(uint32_t mask)
{
    sil_wrw_mem((void *)(TADR_AIC_BASE + TOFF_AIC_IECR), mask);	
}

/*
 * 割込み要求のクリア
 */
Inline void
at91sam7s_clear_int(uint32_t mask)
{
    sil_wrw_mem((void *)(TADR_AIC_BASE + TOFF_AIC_ICCR), mask);	
}

/*
 * 割込み要求のチェック
 */
Inline bool_t
at91sam7s_probe_int(uint32_t mask)
{
    return((sil_rew_mem((void *)(TADR_AIC_BASE + TOFF_AIC_IPR)) & mask) == mask);
}

#define BR    115200                        /* Baud Rate */
#define BRD  (MCK/16/BR)                    /* Baud Rate Divisor */

/*
 * カーネル起動時のログ出力用の初期化
 */
Inline void
at91sam7s_init_uart(void)
{
#ifdef USE_US0
	sil_wrw_mem((void*)(TADR_PMC_BASE+TOFF_PMC_PCER), 
				sil_rew_mem((void*)(TADR_PMC_BASE+TOFF_PMC_PCER)) | (1 << 6));
	sil_wrw_mem((void*)(TADR_PIO_BASE + TOFF_PIO_PDR), (1 <<  5)|(1 <<  6));
	sil_wrw_mem((void*)(TADR_US_BASE + TOFF_US_CR),
				US_RSTRX |          /* Reset Receiver      */
				US_RSTTX |          /* Reset Transmitter   */
				US_RXDIS |          /* Receiver Disable    */
				US_TXDIS            /* Transmitter Disable */
			);
	sil_wrw_mem((void*)(TADR_US_BASE + TOFF_US_MR),
				US_CHMODE_NORMAL |  /* Normal Mode */
				US_CLKS_MCK      |  /* Clock = MCK */
				US_CHRL_8        |  /* 8-bit Data  */
				US_PAR_NO        |  /* No Parity   */
				US_NBSTOP_1         /* 1 Stop Bit  */
				);

	sil_wrw_mem((void*)(TADR_US_BASE + TOFF_US_BRGR), BRD);
	sil_wrw_mem((void*)(TADR_US_BASE + TOFF_US_CR),
				US_RXEN  |          /* Receiver Enable     */
				US_TXEN             /* Transmitter Enable  */
				);
#else /* !USE_US0 */
    uint32_t baud = ((MCK * 10) / (BAUDRATE * 16));
    uint32_t brgr;

    /*
     *  at91sam7s_putc が可能になるようにUARTを初期化
     */
    brgr = baud / 10U;
    if((baud % 10U) >= 5U){
        brgr = (baud / 10U) + 1;
    }
    sil_wrw_mem((void*)(TADR_PIO_BASE+TOFF_PIO_PDR), TOFF_PIO_PDR_VAL);
    sil_wrw_mem((void*)(TADR_DBGU_BASE+TOFF_US_IDR), 0xFFFFFFFFU);
    sil_wrw_mem((void*)(TADR_DBGU_BASE+TOFF_US_CR), US_RSTRX|US_RSTTX|US_RXDIS|US_TXDIS);
    sil_wrw_mem((void*)(TADR_DBGU_BASE+TOFF_US_BRGR), brgr);
    sil_wrw_mem((void*)(TADR_DBGU_BASE+TOFF_US_MR), 4U<<9U);
    sil_wrw_mem((void*)(TADR_DBGU_BASE+TOFF_US_CR), US_TXEN|US_RXEN);
#endif /* USE_US0 */
}

/*
 *  UARTからのポーリング出力
 */
Inline void
at91sam7s_putc(char c)
{
#ifdef USE_US0
	while (!(sil_rew_mem((void*)(TADR_US_BASE + TOFF_US_CSR)) & US_TXRDY));
	sil_wrw_mem((void*)(TADR_US_BASE + TOFF_US_THR), c);
#else /* !USE_US0 */
	while (!(sil_rew_mem((void*)(TADR_DBGU_BASE+TOFF_US_CSR)) & US_TXEMPTY));
	sil_wrw_mem((void*)(TADR_DBGU_BASE+TOFF_US_THR), c);
#endif /* USE_US0 */
}

/*
 * 終了処理
 */
Inline void
at91sam7s_exit(void)
{    
}
     
#endif /* TOPPPERS_MACRO_ONLY */


#endif /* TOPPERS_AT91SAM7S_H */
