/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2005-2015 by Embedded and Real-Time Systems Laboratory
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
 */

/*
 * ターゲット依存モジュール（IMX6SOLOX_M4用）
 */
#include "kernel_impl.h"
#include <sil.h>
#include "imxuart.h"

/*
 *  前方参照
 */
static void init_uart_iomuxc(void);

static uint32_t tmp;

/*
 *  起動時のハードウェア初期化処理
 */
void
hardware_init_hook(void) {
	/*
	 *  -fdata-sectionsを使用するとistkが削除され，
	 *  cfgのパス3のチェックがエラーとなるため，
	 *  削除されないようにする 
	 */
	tmp = (uint32_t)istk;
}

/*
 * ターゲット依存部 初期化処理
 */
void
target_initialize(void)
{
	/*
	 *  コア依存部の初期化
	 */
	core_initialize();

	init_uart_iomuxc();
	sio_low_init(1);
}

/*
 * ターゲット依存部 終了処理
 */
void
target_exit(void)
{
	/* チップ依存部の終了処理 */
	core_terminate();
	while(1);
}

void
target_fput_log(char c)
{
	if (c == '\n') {
		sio_pol_putc('\r', 1);
	}
		sio_pol_putc(c, 1);
}

static void
set_iomuxc(uint32_t mux_ctl_offset, uint32_t mux_mode_val,
			uint32_t sel_input_offset, uint32_t sel_input_val,
			uint32_t pad_ctl_offset, uint32_t pad_ctl_val)
{
	if (mux_ctl_offset)
	  sil_wrw_mem((uint32_t*)(IOMUXC_BASE + mux_ctl_offset), mux_mode_val);

	if (sel_input_offset)
	  sil_wrw_mem((uint32_t*)(IOMUXC_BASE + sel_input_offset), sel_input_val);

	if (pad_ctl_offset)
	  sil_wrw_mem((uint32_t*)(IOMUXC_BASE + pad_ctl_offset), pad_ctl_val);
}


#define UART_PAD_CTL_VAL (IOMUXC_PAD_CTL_PUS_100K_UP | IOMUXC_PAD_CTL_SPEED_MED | IOMUXC_PAD_CTL_DSE_40ohm | IOMUXC_PAD_CTL_SRE_FAST | IOMUXC_PAD_CTL_HYS)

static void
init_uart_iomuxc(void){
#ifdef TARGET_USE_UART1
		set_iomuxc(UART1_TX_IOMUX);
		set_iomuxc(UART1_RX_IOMUX);
#elif defined(TARGET_USE_UART2)
		set_iomuxc(UART2_TX_IOMUX);
		set_iomuxc(UART2_RX_IOMUX);
#endif /* TARGET_USE_UART1 */
}
