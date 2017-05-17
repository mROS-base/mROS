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
 * ターゲット依存モジュール（NUCLEO_F401RE用）
 */
#include "kernel_impl.h"
#include <sil.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_nucleo.h"

/*
 *  起動直後の初期化(system_stm32f4xx.c)
 */
extern void SystemInit(void);

/*
 *  クロックの初期化(systemclock_config.c)
 */
extern void SystemClock_Config(void);

/*
 *  バーナ出力用のUARTの初期化
 */
static void usart_early_init(void);

/*
 *  エラー時の処理
 */
extern void Error_Handler(void);

/*
 *  起動時のハードウェア初期化処理
 */
void
hardware_init_hook(void) {
	SystemInit();

	/*
	 *  -fdata-sectionsを使用するとistkが削除され，
	 *  cfgのパス3のチェックがエラーとなるため，
	 *  削除されないようにする 
	 */
	SystemCoreClock = (uint32_t)istk;
}

/*
 * ターゲット依存部 初期化処理
 */
void
target_initialize(void)
{
	/*
	 *  HALによる初期化
	 *  HAL_Init() : stm32f4xx_hal.c の内容から必要な初期化のみ呼び出す．
	 */
	__HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
	__HAL_FLASH_DATA_CACHE_ENABLE();
	__HAL_FLASH_PREFETCH_BUFFER_ENABLE();

	/*
	 *  クロックの初期化
	 */
	SystemClock_Config();

	/*
	 *  コア依存部の初期化
	 */
	core_initialize();

	/*
	 *  使用するペリフェラルにクロックを供給
	 */

	/*
	 *  UserLEDの初期化
	 */
	BSP_LED_Init(LED2);

	/*
	 *  バーナー出力用のシリアル初期化
	 */
	usart_early_init();
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

static UART_HandleTypeDef UartHandle;

void
usart_early_init()
{
	GPIO_InitTypeDef  GPIO_InitStruct;

	/* Enable Clock */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_USART2_CLK_ENABLE();
  
	/* UART TX GPIO pin configuration  */
	GPIO_InitStruct.Pin       = GPIO_PIN_2;
	GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull      = GPIO_PULLUP;
	GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART2;

	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
	/* UART RX GPIO pin configuration  */
	GPIO_InitStruct.Pin = GPIO_PIN_3;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	UartHandle.Instance          = USART2;
  
	UartHandle.Init.BaudRate     = 115200;
	UartHandle.Init.WordLength   = UART_WORDLENGTH_9B;
	UartHandle.Init.StopBits     = UART_STOPBITS_1;
	UartHandle.Init.Parity       = UART_PARITY_ODD;
	UartHandle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
	UartHandle.Init.Mode         = UART_MODE_TX_RX;
	UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
    
	if(HAL_UART_Init(&UartHandle) != HAL_OK) {
		Error_Handler();
	}
};

/*
 * システムログの低レベル出力のための文字出力
 */
void
target_fput_log(char c)
{
	char cr = '\r';
	if (c == '\n') {
		HAL_UART_Transmit(&UartHandle, (uint8_t *)&cr, 1, 0xFFFF); 
	}
	HAL_UART_Transmit(&UartHandle, (uint8_t *)&c, 1, 0xFFFF); 
}

/*
 * 初期化時のエラー発生時の処理
 */
void
Error_Handler(void){
	volatile int loop;
	BSP_LED_Init(LED2);
	while(1){
		for(loop = 0; loop < 0x100000; loop++);
		BSP_LED_Toggle(LED2);
	}
}

#include "time_event.h"

/*
 *  HAL実行用の関数
 */
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
  return HAL_OK;
}

uint32_t HAL_GetTick(void)
{
  return current_time;
}