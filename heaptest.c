#include <assert.h>
#include <err.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "heap.h"

struct Entry {
	unsigned	i;
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
	unsigned i;

#define N 1000000
	HEAP_INIT(heap, cmp, N, fail);
	for (i = 0; i < N; ++i) {
		struct Entry e;

		e.i = i + 1;
		HEAP_INSERT(heap, e, fail);
	}
	i = 0;
	while (heap.num) {
		struct Entry e;

		HEAP_EXTRACT(heap, e, fail);
		if (i >= e.i) {
			printf("Out of order (%u>=%u)!\n", i, e.i);
		}
		i = e.i;
	}
	HEAP_FREE(heap);
	return 0;
fail:
	abort();
}
