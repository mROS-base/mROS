#ifndef _MROS_PROTOCOL_SUBSCRIBE_CIMPL_H_
#define _MROS_PROTOCOL_SUBSCRIBE_CIMPL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "mros_types.h"

typedef enum {
	MROS_PROTOCOL_SUBSCRIBE_STATE_WAITING = 0,
	MROS_PROTOCOL_SUBSCRIBE_STATE_PUB_CONNECTING,
	MROS_PROTOCOL_SUBSCRIBE_STATE_PUB_REQUESTING,
} mRosProtocolSubscribeStateEnumType;

extern mRosReturnType mros_protocol_subscribe_init(void);
extern void mros_protocol_subscribe_run(void);

#ifdef __cplusplus
}
#endif
#endif /* _MROS_PROTOCOL_SUBSCRIBE_CIMPL_H_ */
