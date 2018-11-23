//ROSのライブラリインクルードだけでmbedのライブラリとかもインクルードしたい？
//#include "../mros/ros.h" //rosのプログラムでは #include "ros/ros.h" 合わせたほうがいい？
#ifndef _APP_H_
#define _APP_H_


#ifdef __cplusplus
extern "C" {
#endif
void usr_task1();
void usr_task2();
void tcp_tx();
void tcp_rx();
#ifdef __cplusplus
}
#endif

#endif
