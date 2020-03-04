#ifndef _MROS_NODE_CIMPL_H_
#define _MROS_NODE_CIMPL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "mros_types.h"

typedef enum {
	MROS_NODE_TYPE_INNER = 0,
	MROS_NODE_TYPE_OUTER,
	MROS_NODE_TYPE_NUM,
} mRosNodeEnumType;


extern mRosReturnType mros_node_init(void);
extern mRosReturnType mros_node_get_byname(const char *node_name, mRosNodeIdType *id);
extern mRosReturnType mros_node_get_bytid(mRosNodeIdType *id);
extern mRosNodeEnumType mros_node_type(mRosNodeIdType id);
extern const char* mros_node_name(mRosNodeIdType id);

extern mRosReturnType mros_node_create_inner(const char *node_name, mRosNodeIdType *id);
extern mRosReturnType mros_node_create_outer(mRosNodeIdType *id);

extern mRosReturnType mros_node_remove(mRosNodeIdType id);

#ifdef __cplusplus
}
#endif
#endif /* _MROS_NODE_CIMPL_H_ */
