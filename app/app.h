//ROSのライブラリインクルードだけでmbedのライブラリとかもインクルードしたい？
//#include "../mros/ros.h" //rosのプログラムでは #include "ros/ros.h" 合わせたほうがいい？

#ifdef __cplusplus
extern "C" {
#endif
void usr_task1();
//void usr_task2();
#ifdef __cplusplus
}
#endif
