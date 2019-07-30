#ifndef _MROS_NAME_H_
#define _MROS_NAME_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "mros_types.h"

extern void mros_name_formalize(const char *src_name, mros_uint32 src_namelen, char *dst_name, mros_uint32 *dst_namelen);


#ifdef __cplusplus
}
#endif

#endif /* _MROS_NAME_H_ */
