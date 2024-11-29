#include <arpa/inet.h>
#include <assert.h>
#include <err.h>
#include <fcntl.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include "ungray_table.h"

#define T2NS (5.24288 / 0.233045)

#define ROI_LEFT_US -1.0
#define ROI_RIGHT_US +1.0
#define MS_WINDOW_NS 100.0
#define MS_TIMEOUT_S 3.0

#define ROI_LEFT (1e3 * ROI_LEFT_US / T2NS)
#define ROI_RIGHT (1e3 * ROI_RIGHT_US / T2NS)

#define LENGTH(x) (sizeof x / sizeof *x)
#define SUBMOD(l, r, range) \
    ((((int64_t)(l - r) + (range) + (range)/2) & ((range) - 1)) - (range)/2)

#define PF_TS(ts) \
	(uint32_t)((ts) >> 32), \
	(uint32_t)((ts) & 0xffffffff)

/* Circular buffer code. */

/* wr == rd -> empty, wr+1 == rd -> full. */
#define CIRC_DECL(Type, N) struct { \
	size_t	rd_i; \
	size_t	wr_i; \
	struct	Type arr[N]; \
}
#define CIRC_GETNUM(circ) SUBMOD(circ.wr_i, circ.rd_i, LENGTH(circ.arr))
#define CIRC_STEP(circ, idx, ofs) ((circ.idx + ofs) & (LENGTH(circ.arr) - 1))
#define CIRC_PEEK(ptr, circ, ofs) do { \
	size_t i_; \
	assert(CIRC_GETNUM(circ) > 0); \
	i_ = CIRC_STEP(circ, rd_i, ofs); \
	ptr = &circ.arr[i_]; \
} while (0)
#define CIRC_PEEK_REV(ptr, circ, ofs) do { \
	size_t i_; \
	assert(CIRC_GETNUM(circ) > 0); \
	i_ = CIRC_STEP(circ, wr_i, LENGTH(circ.arr) - ofs - 1); \
	ptr = &circ.arr[i_]; \
} while (0)
#define CIRC_DROP(circ) do { \
	assert(CIRC_GETNUM(circ) > 0); \
	circ.rd_i = CIRC_STEP(circ, rd_i, 1); \
} while (0)
#define CIRC_PUSH(ptr, name, circ, do_complain_about_full) do { \
	size_t wr_n = CIRC_STEP(circ, wr_i, 1); \
	int is_full = wr_n == circ.rd_i; \
	ptr = &circ.arr[circ.wr_i]; \
	circ.wr_i = wr_n; \
	if (is_full) { \
		if (do_complain_about_full) { \
			fprintf(stderr, \
			    "Too many " name "s, dropping data!\n"); \
		} \
		circ.rd_i = CIRC_STEP(circ, rd_i, 1); \
	} \
} while (0)
#define CIRC_FOREACH(idx, circ) \
    for (idx = circ.rd_i; idx != circ.wr_i; \
	idx = (idx + 1) & (LENGTH(circ.arr) - 1))

enum HTState {
	HT_STATE_PERIODIC,
	HT_STATE_00,
	HT_STATE_01,
	HT_STATE_10,
	HT_STATE_11,
};

struct TsPair {
	uint64_t	ht;
	uint64_t	vmm_ts;
};
struct Hit {
	unsigned	ch;
	uint64_t	vmm_ts;
};
struct Vmm {
	uint64_t	ts_marker;
	struct {
		enum	HTState state;
		unsigned	bits;
		uint32_t	mask;
		uint64_t	ht;
		uint64_t	vmm_ts_prev;
		uint64_t	vmm_ts;
	} ht_build;
	/* 16 * 0.671 ms ~= 10 s. */
	CIRC_DECL(TsPair, 16) ht;
	/* 1<<20 / 10 s ~= 100 kHz. */
	CIRC_DECL(TsPair, 1 << 20) ms;
	CIRC_DECL(Hit, 1 << 20) hit;
	/* Not necessarily the most recent, just rather recent. */
	uint64_t	ts_recent;
};

static struct Vmm *g_vmm_arr;
static size_t g_vmm_num;

double
time_get(void)
{
	struct timespec tp;

	clock_gettime(CLOCK_MONOTONIC, &tp);
	return tp.tv_sec + 1e-9 * tp.tv_nsec;
}

uint32_t
ungray(uint32_t a_u)
{
	/*
	a_u ^= a_u >> 16;
	a_u ^= a_u >>  8;
	a_u ^= a_u >>  4;
	a_u ^= a_u >>  2;
	a_u ^= a_u >>  1;
	return a_u;
	*/
	assert(a_u < 0x1000);
	return g_ungray[a_u];
}

/*
 * pl -> ecl1: 0,20 -- 1,1 -- 0,43 -- 0,62
 * ht -> ecl2: 0,63 -- 1,0 -- 1,22 -- 1,41
 * ht -> ecl3: 2,63 -- 3,0 -- 3,22 -- 3,41
 * pl -> ecl4: 2,20 -- 3,1 -- 2,43 -- 2,62
 */

int
is_ht_ch(unsigned a_vmm_i, unsigned a_ch_i)
{
	return
	    0 == a_vmm_i && 63 == a_ch_i ||
	    1 == a_vmm_i &&  0 == a_ch_i ||
	    2 == a_vmm_i && 63 == a_ch_i ||
	    3 == a_vmm_i &&  0 == a_ch_i;
}

int
is_ms_ch(unsigned a_vmm_i, unsigned a_ch_i)
{
	return
	    0 == a_vmm_i && 20 == a_ch_i ||
	    1 == a_vmm_i &&  1 == a_ch_i ||
	    2 == a_vmm_i && 20 == a_ch_i ||
	    3 == a_vmm_i &&  1 == a_ch_i;
}

struct Vmm *
vmm_get(unsigned a_i)
{
	if (a_i >= g_vmm_num) {
		struct Vmm *arr;
		unsigned num;

		num = a_i + 1;
		arr = calloc(num, sizeof *arr);
		if (!arr) {
			err(EXIT_FAILURE, "calloc");
		}
		memcpy(arr, g_vmm_arr, g_vmm_num * sizeof *arr);
		free(g_vmm_arr);
		g_vmm_arr = arr;
		g_vmm_num = num;
	}
	return &g_vmm_arr[a_i];
}

void
process_ht(unsigned a_vmm_i, unsigned a_ch_i, uint64_t a_ts_curr)
{
	struct Vmm *vmm = vmm_get(a_vmm_i);
	double dt = T2NS * (a_ts_curr - vmm->ht_build.vmm_ts_prev);
	int do_incr = 0;

#define HT_PERIOD 5.24288
#define HT_ZERO 0.16384
#define HT_ONE 0.65536
#define APPROX(ms) fabs(dt - 1e6 * (ms)) < 1e4
#define HT_LOST(msg) do { \
		fprintf(stderr, "vmm=%u: Heimtime sync lost, expected " msg \
		    "!\n", a_vmm_i); \
		vmm->ht_build.state = HT_STATE_PERIODIC; \
	} while (0)
	switch (vmm->ht_build.state) {
	case HT_STATE_PERIODIC:
		if (APPROX(HT_PERIOD)) {
			do_incr = 1;
			vmm->ht_build.mask = 0;
			vmm->ht_build.bits = 0;
		} else if (APPROX(HT_ZERO)) {
			vmm->ht_build.state = HT_STATE_00;
		} else if (APPROX(HT_ONE)) {
			vmm->ht_build.state = HT_STATE_10;
		} else {
			HT_LOST("periodic");
		}
		break;
	case HT_STATE_00:
		if (APPROX(HT_ZERO)) {
			vmm->ht_build.state = HT_STATE_01;
		} else {
			HT_LOST("2nd zero");
		}
		break;
	case HT_STATE_01:
		if (APPROX(HT_PERIOD - 2 * HT_ZERO)) {
			++vmm->ht_build.bits;
			vmm->ht_build.state = HT_STATE_PERIODIC;
			do_incr = 1;
		} else {
			HT_LOST("period after 2nd zero");
		}
		break;
	case HT_STATE_10:
		if (APPROX(HT_ONE)) {
			vmm->ht_build.state = HT_STATE_11;
		} else {
			HT_LOST("2nd one");
		}
		break;
	case HT_STATE_11:
		if (APPROX(HT_PERIOD - 2 * HT_ONE)) {
			vmm->ht_build.mask |= 1 << vmm->ht_build.bits;
			++vmm->ht_build.bits;
			vmm->ht_build.state = HT_STATE_PERIODIC;
			do_incr = 1;
		} else {
			HT_LOST("period after 2nd one");
		}
		break;
	}
	if (do_incr && vmm->ht_build.ht > 0) {
		/* Advance Heimtime on every 2^19-pulse. */
		vmm->ht_build.ht += 1 << 19;
		vmm->ht_build.vmm_ts = a_ts_curr;
		if (0)
			fprintf(stderr, "vmm=%u: TS1 = %08x%08x  HT1 = %08x%08x\n",
			    a_vmm_i,
			    PF_TS(vmm->ht_build.vmm_ts),
			    PF_TS(vmm->ht_build.ht));
	}
	if (32 == vmm->ht_build.bits) {
		/* Full Heimtime decoded! */
		struct TsPair *pair;
		uint64_t ht = (uint64_t)vmm->ht_build.mask << 24;

		if (vmm->ht_build.ht > 0 && vmm->ht_build.ht != ht) {
			fprintf(stderr, "vmm=%u: Heimtime got=%08x%08x but "
			    "expected=%08x%08x!\n",
			    a_vmm_i,
			    PF_TS(ht),
			    PF_TS(vmm->ht_build.ht));
		}
		vmm->ht_build.ht = ht;
		vmm->ht_build.vmm_ts = a_ts_curr;
		vmm->ht_build.state = HT_STATE_PERIODIC;
		vmm->ht_build.mask = 0;
		vmm->ht_build.bits = 0;
		if (0)
			fprintf(stderr, "vmm=%u: Lock TS1 = %08x%08x  HT1 = %08x%08x\n",
			    a_vmm_i,
			    PF_TS(vmm->ht_build.vmm_ts),
			    PF_TS(vmm->ht_build.ht));
		/*
		 * Don't complain about lost HT's, will happen when no MS
		 * coming in. Not great, not terrible.
		 */
		CIRC_PUSH(pair, "Heimtime", vmm->ht, 0);
		pair->ht = ht;
		pair->vmm_ts = a_ts_curr;
	}
	vmm->ht_build.vmm_ts_prev = a_ts_curr;
}

void
process_hit(uint32_t a_u32, uint16_t a_u16)
{
	uint32_t ofs = 0x1f & (a_u32 >> 27);
	uint32_t vmm_i = 0x1f & (a_u32 >> 22);
	uint32_t adc = 0x3ff & (a_u32 >> 12);
	uint32_t bcid = ungray(0xfff & a_u32);
	uint32_t over = 0x1 & (a_u16 >> 14);
	uint32_t ch_i = 0x3f & (a_u16 >> 8);
	uint32_t tdc = 0xff & a_u16;

	struct Vmm *vmm;
	uint64_t ts;

	if (0)
		printf("Vmm = %2u  "
		    "Ch = %2u  "
		    "A/T = %4u/%4u  "
		    "Ofs/BCID = %2u/%4u\n",
		    vmm_i,
		    ch_i,
		    adc, tdc,
		    ofs, bcid);

	vmm = vmm_get(vmm_i);
	ts = vmm->ts_marker | ofs << 12 | bcid;
	if (0)
		printf("%08x%08x\n", PF_TS(ts));

	if (is_ht_ch(vmm_i, ch_i)) {
		/* Stop! Heimtime! */
		process_ht(vmm_i, ch_i, ts);
	} else if (is_ms_ch(vmm_i, ch_i)) {
		struct TsPair *ms;

		/* Master start. */
		CIRC_PUSH(ms, "master-start", vmm->ms, 1);
		ms->vmm_ts = ts;
		ms->ht = 0;
	} else {
		/* Just some hit. */
		struct Hit *hit;

		CIRC_PUSH(hit, "hit", vmm->hit, 0);
		hit->ch = ch_i;
		hit->vmm_ts = ts;
	}
	vmm->ts_recent = ts;
}

void
process_marker(uint32_t a_u32, uint16_t a_u16)
{
	uint32_t vmm_i = 0x1f & (a_u16 >> 10);
	uint32_t ts_lo = 0x3ff & a_u16;
	uint32_t ts_hi = a_u32;
	uint64_t ts = ts_hi << 10 | ts_lo;

	struct Vmm *vmm;

	if (0)
		printf("VMM = %2u  "
		    "TS = 0x%08x%08x\n",
		    vmm_i,
		    PF_TS(ts));

	vmm = vmm_get(vmm_i);
	/* TODO: Report bug? */
	vmm->ts_marker = ts & ~0xffff;
}

/* Drops hits until 1st MS left edge, or if span > RoI. */
void
clean_hits()
{
	size_t i;

	for (i = 0; i < g_vmm_num; ++i) {
		struct Vmm *vmm = &g_vmm_arr[i];

		if (CIRC_GETNUM(vmm->hit) == 0) {
			continue;
		}

		if (CIRC_GETNUM(vmm->ms) == 0) {
			/* Drop hits until last-1st < ~RoI. */
			struct Hit const *hit_last;

			CIRC_PEEK_REV(hit_last, vmm->hit, 0);
			while (CIRC_GETNUM(vmm->hit) > 0) {
				struct Hit const *hit;
				int64_t roi;

				CIRC_PEEK(hit, vmm->hit, 0);
				roi = (ROI_RIGHT_US - ROI_LEFT_US) * 1e3 /
				    T2NS * 2;
				if (SUBMOD(hit_last->vmm_ts, hit->vmm_ts,
				    1 << 30) < roi) {
					break;
				}
				CIRC_DROP(vmm->hit);
			}
		} else {
			/* Drop hits until 1st MS left edge. */
			struct TsPair const *ms;
			uint64_t roi_left, roi_right;

			CIRC_PEEK(ms, vmm->ms, 0);
			roi_left = ms->vmm_ts + ROI_LEFT;
			roi_right = ms->vmm_ts + ROI_RIGHT;

			while (CIRC_GETNUM(vmm->hit) > 0) {
				struct Hit const *hit;

				CIRC_PEEK(hit, vmm->hit, 0);
				if (SUBMOD(roi_left, hit->vmm_ts, 1 << 30) >
				    0) {
					CIRC_DROP(vmm->hit);
				} else {
					break;
				}
			}
		}
	}
}

/* Converts 1st MS VMM times to Heimtime. */
int
time_ms()
{
	size_t i;
	int ok = 1;

	for (i = 0; i < g_vmm_num; ++i) {
		struct Vmm *vmm = &g_vmm_arr[i];
		struct TsPair *ms;
		struct TsPair const *ht0;
		struct TsPair const *ht1;

find_ms:
		if (CIRC_GETNUM(vmm->ms) == 0) {
			ok = 0;
			continue;
		}
		CIRC_PEEK(ms, vmm->ms, 0);
		if (0 != ms->ht) {
			/* Already converted, skip. */
			continue;
		}

find_ht_span:
		if (CIRC_GETNUM(vmm->ht) < 2) {
			/* We need at least 2 HT's to interpolate. */
			ok = 0;
			continue;
		}
		CIRC_PEEK(ht0, vmm->ht, 0);
		if (SUBMOD(ht0->vmm_ts, ms->vmm_ts, 1 << 30) > 0) {
			/* 1st HT is after 1st MS, problem! */
if(0)			fprintf(stderr, "1st MS before 1st HT, dropping!\n");
			CIRC_DROP(vmm->ms);
			goto find_ms;
		}
		CIRC_PEEK(ht1, vmm->ht, 1);
		if (SUBMOD(ms->vmm_ts, ht1->vmm_ts, 1 << 30) >= 0) {
			/* 1st MS after 2nd HT, drop 1st HT and retry. */
			CIRC_DROP(vmm->ht);
			goto find_ht_span;
		}

		/* ht0 <= ms < ht1. */

		ms->ht = (ms->vmm_ts - ht0->vmm_ts)
		    * (ht1->ht - ht0->ht)
		    / (ht1->vmm_ts - ht0->vmm_ts)
		    + ht0->ht;
		if (0) printf("%u: MS HT = %08x%08x\n",
		    (unsigned)i,
		    PF_TS(ms->ht));
	}
	return ok;
}

int
ms_coinc(void)
{
	uint64_t ht;
	size_t i;

	ht = 0;
	for (i = 0; i < g_vmm_num; ++i) {
		struct Vmm *vmm = &g_vmm_arr[i];
		struct TsPair const *ms;

		if (CIRC_GETNUM(vmm->ms) == 0) {
			/* VMM has no MS, cannot emit. */
			return 0;
		}
		CIRC_PEEK(ms, vmm->ms, 0);
		if (0 == ms->ht) {
			/* VMM has no time for MS, cannot emit. */
			return 0;
		}
		if (0 == ht) {
			ht = ms->ht;
			continue;
		}
		if (fabs((double)(int64_t)(ms->ht - ht)) > MS_WINDOW_NS) {
			/* MS's are not coincident, cannot emit. */
			return 0;
		}
	}
	if (0) printf("MS coinc!\n");
	return 1;
}

int
ms_timeout(void)
{
	size_t i;

	for (i = 0; i < g_vmm_num; ++i) {
		struct Vmm *vmm = &g_vmm_arr[i];
		struct TsPair const *ms;
		int64_t dt;

		if (CIRC_GETNUM(vmm->ms) == 0) {
			continue;
		}
		CIRC_PEEK(ms, vmm->ms, 0);
		dt = SUBMOD(vmm->ts_recent, ms->vmm_ts, 1 << 30);
		if (dt > MS_TIMEOUT_S / (T2NS * 1e-9)) {
			/* This VMM is timing out, emit. */
			if (0) printf("MS timeout!\n");
			return 1;
		}
	}
	return 0;
}

struct TsPair const *
earliest()
{
	struct TsPair const *earliest;
	size_t i;

	earliest = NULL;
	for (i = 0; i < g_vmm_num; ++i) {
		struct Vmm *vmm = &g_vmm_arr[i];
		struct TsPair const *ms;

		if (CIRC_GETNUM(vmm->ms) == 0) {
			continue;
		}
		CIRC_PEEK(ms, vmm->ms, 0);
		if (0 == ms->ht) {
			continue;
		}
		if (!earliest || SUBMOD(earliest->ht, ms->ht, 1 << 30) > 0) {
			earliest = ms;
		}
	}
	return earliest;
}

void
emit()
{
	struct TsPair const *ms_earliest;
	uint64_t ht, ts;
	size_t i;

	/* Find earliest MS. */
	ms_earliest = earliest();
	if (!ms_earliest) {
		return;
	}
	ht = ms_earliest->ht;
	ts = ms_earliest->vmm_ts;

	if (0)
		printf("0x%08x%08x 0x%08x%08x\n", PF_TS(ts), PF_TS(ht));

	/* Emit coincident MS's. */
	for (i = 0; i < g_vmm_num; ++i) {
		struct Vmm *vmm = &g_vmm_arr[i];
		struct TsPair const *ms;

		if (CIRC_GETNUM(vmm->ms) == 0) {
			continue;
		}
		CIRC_PEEK(ms, vmm->ms, 0);
		if (0 == ms->ht) {
			continue;
		}
		if (fabs((double)(int64_t)(ms->ht - ht)) < MS_WINDOW_NS) {
			uint64_t roi_left, roi_right;

			roi_left = ms->vmm_ts + ROI_LEFT;
			roi_right = ms->vmm_ts + ROI_RIGHT;

			/* Emit hits! */
			while (CIRC_GETNUM(vmm->hit) > 0) {
				struct Hit const *hit;

				CIRC_PEEK(hit, vmm->hit, 0);
				if (SUBMOD(roi_left, hit->vmm_ts, 1 << 30) >
				    0) {
					/* Hit earlier than ROI, drop. */
					CIRC_DROP(vmm->hit);
				} else if (SUBMOD(hit->vmm_ts, roi_right, 1 <<
				    30) > 0) {
					/* Hit later than ROI, done. */
					break;
				} else {
					/* Emit hit! */
					if (0) printf(" vmm=%u ch=%u ts=%08x%08x\n",
					    (unsigned)i,
					    hit->ch,
					    PF_TS(hit->vmm_ts));
					CIRC_DROP(vmm->hit);
				}
			}
			CIRC_DROP(vmm->ms);
			if (0) printf(" %u 0x%08x%08x 0x%08x%08x\n",
			    (unsigned)i,
			    PF_TS(ms->vmm_ts),
			    PF_TS(ms->ht));
		}
	}

	{
		static unsigned msn = 0;
		static double t_last = -1.0;
		double t;

		++msn;
		t = time_get();
		if (t_last < 0.0) {
			t_last = t;
		} else if (t_last + 0.1 < t) {
			if (0) printf("MS = %u  /s = %f\n",
			    msn, msn / (t - t_last));
			msn = 0;
			t_last = t;
		}
	}
}

void
roi()
{
	/* Find as many ROI's as possible in current data. */
	for (;;) {
		int do_emit;

		/* Clean hits. */
		clean_hits();

		/* Transform 1st MS times to HT, and drop old HT+MS. */
		if (!time_ms()) {
			break;
		}

		/* On a good day we have coinc MS's in all VMM's. */
		if (!ms_coinc()) {
			/*
			 * A few things could be up:
			 * - Too little data to say if any VMM missed MS.
			 * - Any VMM missed MS, emit the others.
			 */
if (0) {
	size_t i;

	for (i = 0; i < g_vmm_num; ++i) {
		struct Vmm *vmm = &g_vmm_arr[i];
		struct TsPair const *ms;

		if (CIRC_GETNUM(vmm->ms) > 0) {
			CIRC_PEEK(ms, vmm->ms, 0);
			printf("%u %08x%08x %08x%08x",
					(unsigned)i,
					PF_TS(ms->vmm_ts), PF_TS(ms->ht));
		}
		if (CIRC_GETNUM(vmm->ms) > 1) {
			CIRC_PEEK(ms, vmm->ms, 1);
			printf(" - %08x%08x %08x%08x",
					PF_TS(ms->vmm_ts), PF_TS(ms->ht));
		}
		puts("");
	}
}
			if (!ms_timeout()) {
				/* Let's get more data. */
				break;
			}
		}

		emit();
	}
}

void
mon(void)
{
	static double t_prev = -1.0;
	double t = time_get();
	size_t i;

	if (t_prev < 0.0) {
		t_prev = t;
	}
	if (t_prev + 1 > t) {
		return;
	}
	t_prev = t;

	for (i = 0; i < g_vmm_num; ++i) {
		struct Vmm *vmm = &g_vmm_arr[i];

		printf("%u: ht=%u ms=%u hit=%u\n",
			(unsigned)i,
			(unsigned)((vmm->ht.wr_i - vmm->ht.rd_i) %
			LENGTH(vmm->ht.arr)),
			(unsigned)((vmm->ms.wr_i - vmm->ms.rd_i) %
			LENGTH(vmm->ms.arr)),
			(unsigned)((vmm->hit.wr_i - vmm->hit.rd_i) %
			LENGTH(vmm->hit.arr)));
	}
}

int
main()
{
	struct sockaddr_in addr;
	int fd;
	size_t buf_ofs = 16;
	uint32_t frame_prev = 0xffffffff;
	int has_header = 0;

#define FROM_FILE 1
#if FROM_FILE
	fd = open("/home/hydra-tpc/new/v010/srs-install/bin/tjohej.bin",
	    O_RDONLY);
	if (-1 == fd) {
		err(EXIT_FAILURE, "open");
	}
#else
	{
		fd = socket(AF_INET, SOCK_DGRAM, 0);
		if (-1 == fd) {
			err(EXIT_FAILURE, "socket");
		}
		memset(&addr, 0, sizeof addr);
		addr.sin_family = AF_INET;
		addr.sin_addr.s_addr = INADDR_ANY;
		addr.sin_port = htons(9000);
		if (bind(fd, (void *)&addr, sizeof addr) < 0) {
			err(EXIT_FAILURE, "bind");
		}
	}
#endif

	for (;;) {
		/* 16 potential data from last read + new buf. */
#define BUF_SIZE (1 << 16)
		uint8_t buf[16 + BUF_SIZE];
		ssize_t bytes, end, i, len;

		/* Read block. */
#if FROM_FILE
		bytes = read(fd, buf + 16, sizeof buf - 16);
#else
		{
			socklen_t socklen;

			socklen = sizeof addr;
			bytes = recvfrom(fd, buf + 16, sizeof buf - 16,
			    MSG_WAITALL, (void *)&addr, &socklen);
		}
#endif
		if (-1 == bytes) {
			err(EXIT_FAILURE, "read");
		} else if (0 == bytes) {
			break;
		}

		/* Parse block. */
#define BUF_R32(ofs) ntohl(*(uint32_t const *)(buf + i + ofs))
#define BUF_R16(ofs) ntohs(*(uint16_t const *)(buf + i + ofs))
		end = 16 + bytes;
		for (i = buf_ofs; i < end;) {
			uint32_t id;

			if (i + 4 * sizeof(uint32_t) > end) {
				break;
			}

			id = BUF_R32(4);
			if ((0xffffff00 & id) == 0x564d3300) {
				uint32_t frame;

				/* Looks like a header. */
				frame = BUF_R32(0);
				if (0xffffffff != frame_prev &&
				    frame_prev + 1 != frame) {
					fprintf(stderr, "Frame counter "
					    "mismatch (0x%08x -> 0x%08x)!\n",
					    frame_prev, frame);
				}
				frame_prev = frame;

				i += 4 * sizeof(uint32_t);
				has_header = 1;
			} else if (has_header) {
				uint32_t u32;
				uint16_t u16;
				unsigned is_hit;

				/* Hit or marker. */
				u32 = BUF_R32(0);
				i += sizeof(uint32_t);
				u16 = BUF_R16(0);
				i += sizeof(uint16_t);

				is_hit = 0x8000 & u16;
				if (is_hit) {
					process_hit(u32, u16);
				} else {
					process_marker(u32, u16);
				}
			}
		}

		mon();

		/* Look for ROI's. */
		roi();

		/* Move any remaining data to the pre-buf space. */
		len = end - i;
		buf_ofs = 16 - len;
		memcpy(buf + buf_ofs, buf + i, len);
	}
	return 0;
}
