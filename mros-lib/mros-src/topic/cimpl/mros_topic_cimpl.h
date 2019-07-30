#ifndef _MROS_TOPIC_CIMPL_H_
#define _MROS_TOPIC_CIMPL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "mros_memory.h"

extern mRosReturnType mros_topic_init(void);
extern mRosContainerObjType mros_topic_get_first(void);
extern mRosContainerObjType mros_topic_get_next(mRosContainerObjType obj);
extern const char *mros_topic_get_topic_name(mRosTopicIdType id);
extern const char *mros_topic_get_topic_typename(mRosTopicIdType id);
extern mRosTopicIdType mros_topic_get_id(mRosContainerObjType obj);
extern mRosReturnType mros_topic_get(const char *topic_name, mRosTopicIdType *id);
extern mRosReturnType mros_topic_create(const char *topic_name, const char *topic_typename, mRosTopicIdType *id);
extern mRosReturnType mros_topic_set_quesize_byname(const char *topic_name, mRosSizeType size);
extern mRosReturnType mros_topic_set_quesize_byid(mRosTopicIdType id, mRosSizeType size);
extern mRosReturnType mros_topic_remove_byname(const char *topic_name);
extern mRosReturnType mros_topic_remove_byid(mRosTopicIdType id);
extern mRosReturnType mros_topic_add_data(mRosTopicIdType id, mRosMemoryListEntryType *data);
extern mRosReturnType mros_topic_get_data(mRosTopicIdType id, mRosMemoryListEntryType **data);


extern mRosReturnType mros_topic_set_typeid(mRosTopicIdType topic_id, mros_uint32 type_id);
extern mRosReturnType mros_topic_get_typeid(mRosTopicIdType topic_id, mros_uint32 *type_id);

extern mRosReturnType mros_topic_set_definition(mRosTopicIdType topic_id, const char* definition);
extern mRosReturnType mros_topic_get_definition(mRosTopicIdType topic_id, const char **definition);

extern mRosReturnType mros_topic_set_md5sum(mRosTopicIdType topic_id, const char* md5sum);
extern mRosReturnType mros_topic_get_md5sum(mRosTopicIdType topic_id, const char **md5sum);

#ifdef __cplusplus
}
#endif
#endif /* _MROS_TOPIC_CIMPL_H_ */
