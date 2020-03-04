#ifndef _MROS_PROTOCOL_PUBLISH_CIMPL_H_
#define _MROS_PROTOCOL_PUBLISH_CIMPL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "mros_types.h"

typedef enum {
	MROS_PROTOCOL_PUBLISH_STATE_WAITING = 0,
	MROS_PROTOCOL_PUBLISH_STATE_STARTING_PUBLISH_TOPIC,
} mRosProtocolPublishStateEnumType;


extern mRosReturnType mros_protocol_publish_init(void);
extern void mros_protocol_publish_run(void);

#ifdef __cplusplus
}
#endif
#endif /* _MROS_PROTOCOL_PUBLISH_CIMPL_H_ */
