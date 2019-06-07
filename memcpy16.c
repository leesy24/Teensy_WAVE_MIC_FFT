#include "memcpy16.h"

void memcpy16(int16_t *dst, const int16_t *src, int cnt)
{
	for (int i = 0; i < cnt; i ++) {
		*dst++ = *src++;
	}
}
