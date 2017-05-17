#include <kernel.h>
#include <t_syslog.h>
#include <t_stdlib.h>

#include "common.h"

extern "C" void *__dso_handle = 0;

/*
 *  初期化タスク
 *
 *  C++の静的グローバルクラスのコンストラクタを呼び出す．
 */
typedef void (*func_ptr)(void);
void init_main_task(intptr_t exinf) {
	extern func_ptr __preinit_array_start[0], __preinit_array_end[0];
	for (func_ptr* func = __preinit_array_start; func != __preinit_array_end; func++) {
		(*func)();
	}	
	
	extern func_ptr __init_array_start[0], __init_array_end[0];
	for (func_ptr* func = __init_array_start; func != __init_array_end; func++) {
		(*func)();
	}	
}
