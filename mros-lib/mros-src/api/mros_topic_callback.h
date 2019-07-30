#ifndef _MROS_INTEGRATION_H_
#define _MROS_INTEGRATION_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "mros_types.h"

extern void mros_topic_callback(mros_uint32 type_id, mRosFuncIdType func_id, const char *data, int len);


#ifdef __cplusplus
}
#endif


#endif /* _MROS_INTEGRATION_H_ */
