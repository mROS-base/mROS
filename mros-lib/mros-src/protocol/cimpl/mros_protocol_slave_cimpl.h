#ifndef _MROS_PROTOCOL_SLAVE_CIMPL_H_
#define _MROS_PROTOCOL_SLAVE_CIMPL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "mros_types.h"

typedef enum {
	MROS_PROTOCOL_SLAVE_STATE_WAITING = 0,
	MROS_PROTOCOL_SLAVE_STATE_REPLYING_REQUEST_TOPIC,
} mRosProtocolSlaveStateEnumType;


extern mRosReturnType mros_protocol_slave_init(void);
extern void mros_protocol_slave_run(void);

#ifdef __cplusplus
}
#endif
#endif /* _MROS_PROTOCOL_SLAVE_CIMPL_H_ */
