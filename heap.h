#ifndef HEAP_H
#define HEAP_H

#define HEAP_HEAD(Name, Type) \
	struct Name { \
		/* ret>0 -> move left towards the root. */ \
		int	(*cmp)(struct Type const *, struct Type const *); \
		size_t	capacity; \
		size_t	num; \
		struct	Type *arr; \
	}
#define HEAP_INIT(h, cmp_, size, label) do { \
	(h).cmp = cmp_; \
	(h).capacity = size; \
	(h).num = 0; \
	(h).arr = calloc(size, sizeof *(h).arr); \
	if (!(h).arr) { \
		warn("calloc"); \
		goto label; \
	} \
} while (0)

#define HEAP_SWAP(h, l, r) do { \
	char tmp_[sizeof *(h).arr]; \
	memcpy(tmp_, &(h).arr[l], sizeof tmp_); \
	memcpy(&(h).arr[l], &(h).arr[r], sizeof tmp_); \
	memcpy(&(h).arr[r], tmp_, sizeof tmp_); \
} while (0)
#define HEAP_EXTRACT(h, v, label) do { \
	if (!(h).num) { \
		warnx("Heap underflow"); \
		goto label; \
	} \
	assert(sizeof v == sizeof *(h).arr); \
	memcpy(&(v), &(h).arr[0], sizeof v); \
	memcpy(&(h).arr[0], &(h).arr[--(h).num], sizeof v); \
	HEAP_HEAPIFY(h); \
} while (0)
#define HEAP_FREE(h) do { \
	(h).capacity = 0; \
	(h).num = 0; \
	free((h).arr); \
} while (0)
#define HEAP_HEAPIFY(h) do { \
	size_t i_ = 0; \
	for (;;) { \
		size_t extr_ = i_; \
		size_t l_ = 2 * i_ + 1; \
		size_t r_ = 2 * i_ + 2; \
		if (l_ < (h).num && \
		    (h).cmp(&(h).arr[l_], &(h).arr[extr_])) { \
			extr_ = l_; \
		} \
		if (r_ < (h).num && \
		    (h).cmp(&(h).arr[r_], &(h).arr[extr_])) { \
			extr_ = r_; \
		} \
		if (extr_ == i_) { \
			break; \
		} \
		HEAP_SWAP(h, i_, extr_); \
		i_ = extr_; \
	} \
} while (0)
#define HEAP_INSERT(h, v, label) do { \
	size_t i_; \
	if ((h).num >= (h).capacity) { \
		warnx("Heap overflow"); \
		goto label; \
	} \
	i_ = (h).num++; \
	assert(sizeof v == sizeof *(h).arr); \
	memcpy(&(h).arr[i_], &(v), sizeof v); \
	while (i_) { \
		size_t parent_ = (i_ - 1) / 2; \
		if ((h).cmp(&(h).arr[i_], &(h).arr[parent_])) { \
			HEAP_SWAP(h, i_, parent_); \
			i_ = parent_; \
		} else { \
			break; \
		} \
	} \
} while (0)

#endif
