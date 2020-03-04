#include "mros_name.h"
#include <string.h>

void mros_name_formalize(const char *src_name, mros_uint32 src_namelen, char *dst_name, mros_uint32 *dst_namelen)
{
	mros_uint32 i;
	mros_uint32 slash_count = 0;
	const char* src_copy_head = MROS_NULL;
	mros_uint32 src_copy_len;

	for (i = 0; i < src_namelen; i++) {
		if (src_name[i] != '/') {
			src_copy_head = &src_name[i];
			break;
		}
		else {
			slash_count++;
		}
	}
	src_copy_len = src_namelen - slash_count;
	dst_name[0] = '/';
	memcpy(&dst_name[1], src_copy_head, src_copy_len);
	dst_name[src_copy_len + 1] = '\0';
	*dst_namelen = src_copy_len + 1;
	return;
}
