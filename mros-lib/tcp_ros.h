#ifndef _TCP_ROS_H_
#define _TCP_ROS_H_

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <malloc.h>
#include "mbed.h"
#include "EthernetInterface.h"

//TCPROSヘッダデータサイズを付与する関数
char* addtcproshead(char *buf);

//TCPROSコネクションヘッダを作る関数
int genPubTcpRosH(char *buf);
int genSubTcpRosH(char *buf);

//TCPROSのボディを作る関数
int genMessage(char *buf,char *msg);
char* addtcproshead2(char *buf);
/* 
* TCPROSのヘッダに必要なfieldの要素をどうするか考える．
* structでpub/sub,server/clientで分けるかどうか
* md5sumとかtopicのname,typeとかの参照をどうするか
*/

bool check_head(char *buf);



#endif
