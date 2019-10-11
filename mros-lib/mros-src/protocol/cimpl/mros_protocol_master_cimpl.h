#ifndef _MROS_PROTOCOL_MASTER_CIMPL_H_
#define _MROS_PROTOCOL_MASTER_CIMPL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "mros_types.h"

typedef enum {
	MROS_PROTOCOL_MASTER_STATE_WAITING = 0,
	MROS_PROTOCOL_MASTER_STATE_REGISTER_PUBLISHER,
	MROS_PROTOCOL_MASTER_STATE_REGISTER_SUBSCRIBER,
	MROS_PROTOCOL_MASTER_STATE_REQUESTING_TOPIC,
} mRosProtocolMasterStateEnumType;

typedef enum {
	MROS_PROTOCOL_MASTER_REQ_REGISTER_PUBLISHER = 0,
	MROS_PROTOCOL_MASTER_REQ_REGISTER_SUBSCRIBER,
} mRosProtocolMasterRequestEnumType;

typedef struct {
	mRosProtocolMasterRequestEnumType 	req_type;
	mRosContainerObjType 				connector_obj;
} mRosProtocolMasterRequestType;

extern mRosReturnType mros_protocol_master_init(void);
extern void mros_protocol_master_run(void);

#ifdef __cplusplus
}
#endif
#endif /* _MROS_PROTOCOL_MASTER_CIMPL_H_ */
