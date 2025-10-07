#include <assert.h>
#include <err.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "heap.h"

struct Entry {
	uint64_t	i;
};

HEAP_HEAD(Heap, Entry);

static int
cmp(struct Entry const *a_l, struct Entry const *a_r)
{
	return a_l->i < a_r->i ? 1 : 0;
}

int
main()
{
	struct Heap heap;
	uint64_t i;

#define N 1000000
	HEAP_INIT(heap, cmp, 2 * N, fail);
	for (i = 0; i < N; ++i) {
		struct Entry e;

		e.i = 1000 * (i + 1);
		HEAP_INSERT(heap, e, fail);
		e.i = 1000 * (N - i);
		HEAP_INSERT(heap, e, fail);
	}
	for (i = 1000; heap.num; i += 1000) {
		struct Entry e;
		uint64_t j;

		for (j = 0; j < 2; ++j) {
			HEAP_EXTRACT(heap, e, fail);
			if (i != e.i) {
				printf("Out of order "
				    "(heap=%08x%08x!=expect=%08x%08x)!\n",
				    (uint32_t)(e.i >> 32),
				    (uint32_t)e.i,
				    (uint32_t)(i >> 32),
				    (uint32_t)i);
			}
		}
	}
	HEAP_FREE(heap);
	return 0;
fail:
	abort();
}
