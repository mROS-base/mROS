#ifndef _TCP_ROS_H_
#define _TCP_ROS_H_

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

//TCPROSヘッダデータサイズを付与する関数
char* addtcproshead(char *buf);

//TCPROSコネクションヘッダを作る関数
int genPubTcpRosH(char *buf);

//TCPROSのボディを作る関数
int genMessage(char *buf);

/* 
* TCPROSのヘッダに必要なfieldの要素をどうするか考える．
* structでpub/sub,server/clientで分けるかどうか
* md5sumとかtopicのname,typeとかの参照をどうするか
*/




#endif
