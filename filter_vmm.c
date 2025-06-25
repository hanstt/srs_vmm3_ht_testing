#include <assert.h>
#include <err.h>
#include <math.h>

#include <lwroc_filter.h>
#include <lwroc_message.h>
#include <lwroc_mon_block.h>
#include <lwroc_filter_loop.h>
#include <lwroc_parse_util.h>
#include <lmd/lwroc_lmd_util.h>
#include <lmd/lwroc_lmd_white_rabbit_stamp.h>

#include "heap.h"
#include "ungray_table.h"

#define WR_ID 0x100

#define ROI_LEFT_US -1.0
#define ROI_RIGHT_US +1.0

#define MS_WINDOW_NS 100.0

/* Wait this long until we start building events. */
#define HT_DT_SAFE 1.0

static uint32_t g_evtcnt;

struct HtMs {
	uint16_t	vmm_i;
	uint64_t	ht;
};
HEAP_HEAD(HeapMs, HtMs);
static struct HeapMs g_ms_heap;

struct HtHit {
	uint16_t	ch;
	uint64_t	ht;
	uint16_t	adc;
	uint16_t	tdc;
};
HEAP_HEAD(HeapHit, HtHit);

static void process_hit(uint32_t, uint16_t);
static void process_marker(uint32_t, uint16_t);
static void raw2heap(void);
static void roi(lwroc_pipe_buffer_consumer *, lwroc_data_pipe_handle *);

/*
 * VMM stuff.
 * There are two kinds of timestamps:
 *  ht = Heimtime
 *  ts = VMM
 */

#define TS2NS 22.5 /* (5.24288 / 0.233045) */

#define ROI_LEFT_TS (1e3 * ROI_LEFT_US / TS2NS)
#define ROI_RIGHT_TS (1e3 * ROI_RIGHT_US / TS2NS)

#define LENGTH(x) (sizeof x / sizeof *x)
#define MAX(a, b) ((a) > (b) ? (a) : (b))
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
			printf("Too many " name "s, dropping data!\n"); \
		} \
		circ.rd_i = CIRC_STEP(circ, rd_i, 1); \
	} \
} while (0)
#define CIRC_FOREACH(idx, circ) \
    for (idx = circ.rd_i; idx != circ.wr_i; \
	idx = (idx + 1) & (LENGTH(circ.arr) - 1))

struct HtPair {
	uint64_t	ht;
	/*
	 * VMM timestamps are 42 bits, but 32 bits cover (1<<32)*22.5 ns,
	 * which is way more than needed for decoding HT and building ROI's.
	 */
	uint32_t	ts;
};
struct VmmChannel {
	uint16_t	ch;
	uint32_t	ts;
	uint16_t	adc;
	uint16_t	tdc;
};
struct Vmm {
	uint64_t	ts_marker;
	struct {
		/* Last three pulse diffs to survive lost pulses. */
		uint32_t	history[4];
		/* Last known 1<<19 pulse. */
		struct {
			int	has;
			uint32_t	ts_prev;
		} carry;
		/* HT under construction. */
		unsigned	bit_i;
		uint32_t	mask;
		/*
		 * Last known Heimtime, shifted up 24 bits, incremented on
		 * periodic pulse.
		 */
		struct {
			int	has;
			uint64_t	ht;
		} ht;
	} ht_build;
	/* 256 * 5.24 ms ~= 1.3 s. */
	CIRC_DECL(HtPair, 256) ht_buf;
	/* 1<<17 / 1.3 s ~= 100 kHz. */
	CIRC_DECL(VmmChannel, 1 << 17) ch_buf;
	struct	HeapHit hit_heap;
};

static uint64_t	g_ht_latest;
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

int
is_ht_ch(unsigned a_vmm_i, unsigned a_ch_i)
{
	return
	    (0 == a_vmm_i &&  3 == a_ch_i) /*||
	    (1 == a_vmm_i && 37 == a_ch_i)*/;
}

int
is_ms_ch(unsigned a_vmm_i, unsigned a_ch_i)
{
	return
	    (0 == a_vmm_i &&  1 == a_ch_i) ||
	    (1 == a_vmm_i && 53 == a_ch_i);
}

int
cmp_ms(struct HtMs const *a_l, struct HtMs const *a_r)
{
	return a_l->ht < a_r->ht;
}

int
cmp_hit(struct HtHit const *a_l, struct HtHit const *a_r)
{
	return a_l->ht < a_r->ht;
}

struct Vmm *
vmm_get(unsigned a_i)
{
	if (a_i >= g_vmm_num) {
		struct Vmm *arr;
		unsigned i, num;

		num = a_i + 1;
		arr = calloc(num, sizeof *arr);
		if (!arr) {
			err(EXIT_FAILURE, "calloc");
		}
		memcpy(arr, g_vmm_arr, g_vmm_num * sizeof *arr);
		free(g_vmm_arr);
		for (i = g_vmm_num; i < num; ++i) {
			HEAP_INIT(arr[i].hit_heap, cmp_hit, 1 << 17, fail);
		}
		g_vmm_arr = arr;
		g_vmm_num = num;
	}
	return &g_vmm_arr[a_i];
fail:
	err(EXIT_FAILURE, "Hit heap init failed");
}

void
process_ht(unsigned a_vmm_i, unsigned a_ch_i, uint32_t a_ts_curr)
{
	uint32_t dt_arr[3];
	struct Vmm *vmm = vmm_get(a_vmm_i);
	struct HtPair *pair;
	size_t i;
	unsigned bit_ts;
	int do_bit, do_clear = 0;

	/* Keep a list of pulse distances. */
	memmove(
	    &vmm->ht_build.history[0],
	    &vmm->ht_build.history[1],
	    3 * sizeof vmm->ht_build.history[0]);
	vmm->ht_build.history[3] = a_ts_curr;
	for (i = 0; i + 1 < LENGTH(vmm->ht_build.history); ++i) {
		dt_arr[i] =
		    vmm->ht_build.history[i + 1] - vmm->ht_build.history[i];
	}
if(0)printf("%u %2u  %6.3f,%6.3f,%6.3f\n",
	a_vmm_i, a_ch_i,
	1e-6 * TS2NS * dt_arr[0],
	1e-6 * TS2NS * dt_arr[1],
	1e-6 * TS2NS * dt_arr[2]);

#define HT_P (uint32_t)(5.24288 * 1e6 / TS2NS)
#define HT_0 (uint32_t)(0.16384 * 1e6 / TS2NS)
#define HT_1 (uint32_t)(0.65536 * 1e6 / TS2NS)
#define HT_APPROX(l, r) (fabs((l) - (r)) < 2e3)

	if (!vmm->ht_build.carry.has) {
		/* Find 1<<19 pulses, then we start decoding. */
		if (!(HT_APPROX(dt_arr[1], HT_P) &&
		      HT_APPROX(dt_arr[2], HT_P))) {
			return;
		}
		printf("Found HT carry.\n");
		vmm->ht_build.carry.has = 1;
		vmm->ht_build.carry.ts_prev = a_ts_curr;
		do_clear = 1;
	}

	/* Create a timestamp for every 1<<19 pulse. */
	{
		uint32_t dts;
		int do_inc;

		dts = a_ts_curr - vmm->ht_build.carry.ts_prev;
		do_inc = 0;
		if (HT_APPROX(dts, HT_P)) {
			do_inc = 1;
		} else if (HT_APPROX(dts, HT_P)) {
			/* Lost periodic pulse, increment twice. */
			do_inc = 2;
		} else if (dts > HT_P) {
			fprintf(stderr, "Lost HT carry signal.\n");
			vmm->ht_build.carry.has = 0;
			return;
		}
		if (do_inc) {
			vmm->ht_build.carry.ts_prev = a_ts_curr;
			vmm->ht_build.ht.ht += do_inc << 19;
			/*
			 * Don't complain about lost HT's, will happen when no
			 * MS coming in.
			 */
			CIRC_PUSH(pair, "Heimtime", vmm->ht_buf, 0);
			pair->ht = vmm->ht_build.ht.ht;
			pair->ts = a_ts_curr;
		}
	}

	/*
	 * Ways to detect bit (w=0.16/0.66, h=5.24-2w):
	 * |...|...| h |
	 * |...|...|h+w|
	 * | w | w |...|
	 * |...|2*w|...|
	 * The latter two miss a definite timestamp for the ending 1<<19
	 * pulse, we guesstimate it.
	 */
	do_bit = 0;
#define TEST_BIT(bit) do { \
	if (HT_APPROX(dt_arr[2], HT_P - 2*HT_##bit) || \
	    HT_APPROX(dt_arr[2], HT_P - 1*HT_##bit)) { \
		bit_ts = vmm->ht_build.history[3]; \
		do_bit = 1 + bit; \
	} else if ((HT_APPROX(dt_arr[0], 1*HT_##bit) && \
	            HT_APPROX(dt_arr[1], 1*HT_##bit)) || \
		   HT_APPROX(dt_arr[1], 2*HT_##bit)) { \
		bit_ts = vmm->ht_build.history[2] + \
		    (uint32_t)((1e6 * (HT_P - 2*HT_##bit)) / TS2NS); \
		do_bit = 1 + bit; \
	} \
	} while (0)
	TEST_BIT(0);
	TEST_BIT(1);
	if (do_bit) {
		vmm->ht_build.mask |= (do_bit - 1) << vmm->ht_build.bit_i;
		++vmm->ht_build.bit_i;
	}

	if (32 == vmm->ht_build.bit_i) {
		/* Full Heimtime decoded! */
		uint64_t ht = (uint64_t)vmm->ht_build.mask << 24;

		if (vmm->ht_build.ht.has) {
			if (vmm->ht_build.ht.ht != ht) {
				printf("vmm=%u: Heimtime expected=%08x but "
				    "got=%08x!\n",
				    a_vmm_i,
				    vmm->ht_build.ht.ht >> 24,
				    vmm->ht_build.mask);
			} else if (vmm->ht_build.ht.ht > ht) {
				printf("vmm=%u: Heimtime reversed %08x -> "
				    "%08x!\n",
				    a_vmm_i,
				    vmm->ht_build.ht.ht >> 24,
				    vmm->ht_build.mask);
			}
		} else {
			printf("Found HT=0x%08x\n", vmm->ht_build.mask);
		}
		do_clear = 1;
		vmm->ht_build.ht.has = 1;
		vmm->ht_build.ht.ht = ht;
		if (0)
			printf("vmm=%u: HT = %08x\n",
			    a_vmm_i,
			    vmm->ht_build.ht);
	}
	if (do_clear) {
		vmm->ht_build.bit_i = 0;
		vmm->ht_build.mask = 0;
	}
}

void
process_hit(uint32_t a_u32, uint16_t a_u16)
{
	int32_t ofs = 0x1f & (a_u32 >> 27);
	uint32_t vmm_i = 0x1f & (a_u32 >> 22);
	uint32_t adc = 0x3ff & (a_u32 >> 12);
	uint32_t bcid = ungray(0xfff & a_u32);
	uint32_t over = 0x1 & (a_u16 >> 14);
	uint32_t ch_i = 0x3f & (a_u16 >> 8);
	uint32_t tdc = 0xff & a_u16;

	struct Vmm *vmm;
	uint64_t ts;

	/* Wireshark hax! */
	/*
	if (16 == ofs) {
		ofs = 5;
	}
	*/
	if (31 == ofs) {
		ofs = -1;
	}

	if (0)
		printf("vmm=%2u  "
		    "ch=%2u  "
		    "a/t=%4u/%4u  "
		    "ofs/bcid=%3d/%4u\n",
		    vmm_i,
		    ch_i,
		    adc, tdc,
		    ofs, bcid);

	vmm = vmm_get(vmm_i);
	ts = vmm->ts_marker + (ofs * 4096) + bcid;
	if (0)
		printf("%08x%08x\n", PF_TS(ts));

	if (is_ht_ch(vmm_i, ch_i)) {
		/* Stop! Heimtime! */
		process_ht(vmm_i, ch_i, ts);
	} else {
		/* MS or hit. */
		struct VmmChannel *ch;

		CIRC_PUSH(ch, "channel", vmm->ch_buf, 0);
		ch->ch = ch_i;
		ch->ts = ts;
		ch->adc = adc;
		ch->tdc = tdc;
	}
}

void
process_marker(uint32_t a_u32, uint16_t a_u16)
{
	uint32_t vmm_i = 0x1f & (a_u16 >> 10);
	uint32_t ts_lo = 0x3ff & a_u16;
	uint64_t ts_hi = a_u32;
	uint64_t ts = ts_hi << 10 | ts_lo;

	struct Vmm *vmm;

	if (0)
		printf("Marker vmm=%2u  "
		    "%08x:%04x  "
		    "ts=0x%08x%08x\n",
		    vmm_i,
		    a_u32, a_u16,
		    PF_TS(ts));

	vmm = vmm_get(vmm_i);
	vmm->ts_marker = ts;
}

#if 0
/* Drops hits until 1st MS left edge, or if span > RoI. */
void
clean_heap()
{
	size_t i;

	for (i = 0; i < g_vmm_num; ++i) {
		struct Vmm *vmm = &g_vmm_arr[i];
		uint64_t limit_ts;

		if (CIRC_GETNUM(vmm->hit_buf) == 0) {
			continue;
		}

		if (CIRC_GETNUM(vmm->ms_buf) == 0) {
			/* Drop hits until last - RoI. */
			struct Hit const *hit_last;
			int64_t span;

			CIRC_PEEK_REV(hit_last, vmm->hit_buf, 0);
			span = ROI_RIGHT_TS - ROI_LEFT_TS;
			limit_ts = hit_last->vmm_ts - span;
		} else {
			/* Drop hits until MS left edge. */
			struct TsPair const *ms;

			CIRC_PEEK(ms, vmm->ms_buf, 0);
			limit_ts = ms->vmm_ts + ROI_LEFT_TS;
		}
		while (CIRC_GETNUM(vmm->hit_buf) > 0) {
			struct Hit const *hit;

			CIRC_PEEK(hit, vmm->hit_buf, 0);
			if (SUBMOD(limit_ts, hit->vmm_ts, 1 << 30) <= 0) {
				break;
			}
			CIRC_DROP(vmm->hit_buf);
		}
	}
}

/* Converts 1st MS VMM times to Heimtime. */
void
time_ms()
{
	size_t i;

	for (i = 0; i < g_vmm_num; ++i) {
		struct Vmm *vmm = &g_vmm_arr[i];
		struct TsPair *ms;
		struct TsPair const *ht0;
		struct TsPair const *ht1;

find_ms:
		if (CIRC_GETNUM(vmm->ms_buf) == 0) {
			continue;
		}
		CIRC_PEEK(ms, vmm->ms_buf, 0);
		if (0 != ms->ht) {
			/* Already converted, skip. */
			continue;
		}

find_ht_span:
		if (CIRC_GETNUM(vmm->ht_buf) < 2) {
			/* We need at least 2 HT's to interpolate. */
			continue;
		}
		CIRC_PEEK(ht0, vmm->ht_buf, 0);
		if (SUBMOD(ht0->vmm_ts, ms->vmm_ts, 1 << 30) > 0) {
			/* 1st HT is after 1st MS, problem! */
			printf("1st MS before 1st HT, dropping MS!\n");
			CIRC_DROP(vmm->ms_buf);
			goto find_ms;
		}
		CIRC_PEEK(ht1, vmm->ht_buf, 1);
		if (SUBMOD(ms->vmm_ts, ht1->vmm_ts, 1 << 30) >= 0) {
			/* 1st MS after 2nd HT, drop 1st HT and retry. */
			CIRC_DROP(vmm->ht_buf);
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

		if (CIRC_GETNUM(vmm->ms_buf) == 0) {
			/* VMM has no MS, cannot emit. */
			return 0;
		}
		CIRC_PEEK(ms, vmm->ms_buf, 0);
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

		if (CIRC_GETNUM(vmm->ms_buf) == 0) {
			continue;
		}
		CIRC_PEEK(ms, vmm->ms_buf, 0);
		dt = SUBMOD(g_ts_latest, ms->vmm_ts, 1 << 30);
		if (dt > MS_TIMEOUT_S / (TS2NS * 1e-9)) {
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

		if (CIRC_GETNUM(vmm->ms_buf) == 0) {
			continue;
		}
		CIRC_PEEK(ms, vmm->ms_buf, 0);
		if (0 == ms->ht) {
			continue;
		}
		if (!earliest || SUBMOD(earliest->ht, ms->ht, 1 << 30) > 0) {
			earliest = ms;
		}
	}
	return earliest;
}

/* Write event. */
void
emit(lwroc_pipe_buffer_consumer *pipe_buf, lwroc_data_pipe_handle
    *data_handle)
{
	struct TsPair const *ms_earliest;
	uint32_t *p32;
	size_t i;
	uint64_t ht, ts;
	uint32_t header_size;
	uint32_t wr_size;
	uint32_t payload_size;
	uint32_t write_size;
	uint32_t sync_check;

	/* Find earliest MS. */
	ms_earliest = earliest();
	if (!ms_earliest) {
		return;
	}
	ht = ms_earliest->ht;
	ts = ms_earliest->vmm_ts;

	header_size = (sizeof (lmd_subevent_10_1_host) +
	    sizeof (lmd_event_10_1_host));
	wr_size = (5 + 1) * sizeof (uint32_t);
	payload_size = 4;

	write_size = header_size + wr_size + payload_size;

	p32 = (void *)lwroc_request_event_space(data_handle, write_size);

	ts = 123456;

	p32[0] = (WR_ID << WHITE_RABBIT_STAMP_EBID_BRANCH_ID_SHIFT) &
	    WHITE_RABBIT_STAMP_EBID_BRANCH_ID_MASK;
	p32[1] = WHITE_RABBIT_STAMP_LL16_ID |
	    (uint32_t)((ts >>  0) & 0xffff);
	p32[2] = WHITE_RABBIT_STAMP_LH16_ID |
	    (uint32_t)((ts >> 16) & 0xffff);
	p32[3] = WHITE_RABBIT_STAMP_HL16_ID |
	    (uint32_t)((ts >> 32) & 0xffff);
	p32[4] = WHITE_RABBIT_STAMP_HH16_ID |
	    (uint32_t)((ts >> 48) & 0xffff);

	sync_check = 1000;

	p32[5] = SYNC_CHECK_MAGIC | SYNC_CHECK_RECV |
	    (sync_check & 0xffff);

	p32 += wr_size / 4;


	if (0)
		printf("0x%08x%08x 0x%08x%08x\n", PF_TS(ts), PF_TS(ht));

	/* Emit coincident MS's. */
	for (i = 0; i < g_vmm_num; ++i) {
		struct Vmm *vmm = &g_vmm_arr[i];
		struct TsPair const *ms;

		if (CIRC_GETNUM(vmm->ms_buf) == 0) {
			continue;
		}
		CIRC_PEEK(ms, vmm->ms_buf, 0);
		if (0 == ms->ht) {
			continue;
		}
		if (fabs((double)(int64_t)(ms->ht - ht)) < MS_WINDOW_NS) {
			uint64_t roi_left, roi_right;

			roi_left = ms->vmm_ts + ROI_LEFT_TS;
			roi_right = ms->vmm_ts + ROI_RIGHT_TS;

			/* Emit hits! */
			while (CIRC_GETNUM(vmm->hit_buf) > 0) {
				struct Hit const *hit;

				CIRC_PEEK(hit, vmm->hit_buf, 0);
				if (SUBMOD(roi_left, hit->vmm_ts, 1 << 30) >
				    0) {
					/* Hit earlier than ROI, drop. */
					CIRC_DROP(vmm->hit_buf);
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
					CIRC_DROP(vmm->hit_buf);
				}
			}
			CIRC_DROP(vmm->ms_buf);
			if (0) printf(" %u 0x%08x%08x 0x%08x%08x\n",
			    (unsigned)i,
			    PF_TS(ms->vmm_ts),
			    PF_TS(ms->ht));
		}
	}

	lwroc_used_event_space(data_handle, write_size, 0);

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
#endif

void
roi(lwroc_pipe_buffer_consumer *pipe_buf, lwroc_data_pipe_handle *data_handle)
{
	/* Find as many ROI's as possible in current data. */
	while (g_ms_heap.num) {
		struct HtMs const *msp;
		uint64_t dht, ht0;

		void *p0;
		lmd_event_10_1_host *ev;
		lmd_subevent_10_1_host *sev;
		uint32_t *p32;
		uint8_t *p8;
		uint32_t header_size;
		uint32_t wr_size;
		uint32_t payload_size;
		uint32_t write_size;
		uint32_t sync_check;

		/*
		 * Peek at 1st MS and see if we have enough data to try to
		 * build events.
		 */
		msp = &g_ms_heap.arr[0];
		ht0 = msp->ht;
		dht = g_ht_latest - ht0;
		if (dht < HT_DT_SAFE) {
			break;
		}

		/* Build LMD event. */

		header_size = sizeof ev + sizeof sev;
		wr_size = (1 + 4 + 1) * sizeof (uint32_t);
		payload_size = 0x10000;

		write_size = header_size + wr_size + payload_size;

		p0 = lwroc_request_event_space(data_handle, write_size);

		ev = p0;
		sev = (void *)(ev + 1);
		p32 = (void *)(sev + 1);

		*p32++ = (WR_ID << WHITE_RABBIT_STAMP_EBID_BRANCH_ID_SHIFT) &
		    WHITE_RABBIT_STAMP_EBID_BRANCH_ID_MASK;
		*p32++ = WHITE_RABBIT_STAMP_LL16_ID |
		    (uint32_t)((ht0 >>  0) & 0xffff);
		*p32++ = WHITE_RABBIT_STAMP_LH16_ID |
		    (uint32_t)((ht0 >> 16) & 0xffff);
		*p32++ = WHITE_RABBIT_STAMP_HL16_ID |
		    (uint32_t)((ht0 >> 32) & 0xffff);
		*p32++ = WHITE_RABBIT_STAMP_HH16_ID |
		    (uint32_t)((ht0 >> 48) & 0xffff);

		sync_check = 1000;

		*p32++ = SYNC_CHECK_MAGIC | SYNC_CHECK_RECV |
		    (sync_check & 0xffff);

		/* For each VMM with the MS, extract hits in ROI. */
		while (g_ms_heap.num) {
			struct HtMs ms;
			struct Vmm *vmm;

			msp = &g_ms_heap.arr[0];
			dht = msp->ht - ht0;
			if (dht > MS_WINDOW_NS) {
				break;
			}
			HEAP_EXTRACT(g_ms_heap, ms, fail);

			/* Write MS. */
			*p32++ = 1;

			vmm = vmm_get(ms.vmm_i);
			while (vmm->hit_heap.num) {
				struct HtHit const *hp;
				struct HtHit hit;

				hp = &vmm->hit_heap.arr[0];
				if (ms.ht + ROI_RIGHT_US < hp->ht) {
					break;
				}
				if (hp->ht + ROI_LEFT_US >= ms.ht) {
					/* Write hit. */
					*p32++ = 2;
				}
				HEAP_EXTRACT(vmm->hit_heap, hit, fail);
			}
		}

		ev->_header.l_dlen =
		    ((uintptr_t)p32 - (uintptr_t)(&ev->_header + 1)) / 2;
		ev->_header.i_type = 10;
		ev->_header.i_subtype = 1;
		ev->_info.i_dummy = 0;
		ev->_info.i_trigger = 1;
		ev->_info.l_count = ++g_evtcnt;
		sev->_header.l_dlen =
		    ((uintptr_t)p32 - (uintptr_t)(&sev->_header + 1)) / 2;
		sev->_header.i_type = 10;
		sev->_header.i_subtype = 1;
		sev->i_procid = 0;
		sev->h_subcrate = 0;
		sev->h_control = 0;

		lwroc_used_event_space(data_handle,
		    (uintptr_t)p32 - (uintptr_t)p0, 0);
	}
	return;
fail:
	errx(EXIT_FAILURE, "Shouldn't happen");
}

void
raw2heap(void)
{
	size_t vmm_i;

	for (vmm_i = 0; vmm_i < g_vmm_num; ++vmm_i) {
		struct Vmm *vmm = &g_vmm_arr[vmm_i];
		struct HtPair const *ht0;
		struct HtPair const *ht1;

find_ht_span:
		if (CIRC_GETNUM(vmm->ht_buf) < 2) {
			/* We need at least 2 HT's to interpolate. */
			continue;
		}
		CIRC_PEEK(ht0, vmm->ht_buf, 0);
		CIRC_PEEK(ht1, vmm->ht_buf, 1);

		while (CIRC_GETNUM(vmm->ch_buf)) {
			struct VmmChannel *ch;
			uint64_t ht;

			CIRC_PEEK(ch, vmm->ch_buf, 0);
			if (SUBMOD(ht0->ts, ch->ts, 1 << 30) > 0) {
				/* Hit before 1st HT, drop hit. */
				CIRC_DROP(vmm->ch_buf);
				continue;
			}
			if (SUBMOD(ch->ts, ht1->ts, 1 << 30) > 0) {
				/* Hit after 2nd HT, drop HT. */
				CIRC_DROP(vmm->ht_buf);
				goto find_ht_span;
			}
			ht = (ch->ts - ht0->ts)
			    * (ht1->ht - ht0->ht)
			    / (ht1->ts - ht0->ts)
			    + ht0->ht;
			g_ht_latest = MAX(g_ht_latest, ht);
			if (is_ms_ch(vmm_i, ch->ch)) {
				struct HtMs ms;

				ms.vmm_i = vmm_i;
				ms.ht = ht;
				HEAP_INSERT(g_ms_heap, ms, fail);
			} else {
				struct HtHit hit;

				hit.ch = ch->ch;
				hit.ht = ht;
				hit.adc = ch->adc;
				hit.tdc = ch->tdc;
				HEAP_INSERT(vmm->hit_heap, hit, fail);
			}
			CIRC_DROP(vmm->ch_buf);
			continue;
fail:
			fprintf(stderr, "Heap overflow, Heimtime missing?\n");
			CIRC_DROP(vmm->ch_buf);
		}
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

		printf("%u: #ht=%u #ch=%u\n",
			(unsigned)i,
			(unsigned)((vmm->ht_buf.wr_i - vmm->ht_buf.rd_i) %
			LENGTH(vmm->ht_buf.arr)),
			(unsigned)((vmm->ch_buf.wr_i - vmm->ch_buf.rd_i) %
			LENGTH(vmm->ch_buf.arr)));
	}
}

/* Drasi stuff. */

void loop(lwroc_pipe_buffer_consumer *, const lwroc_thread_block *,
    lwroc_data_pipe_handle *, volatile int *);
void max_ev_len(uint32_t *, uint32_t *);

void
lwroc_user_filter_pre_setup_functions(void)
{
	_lwroc_user_filter_functions->loop = loop;
	_lwroc_user_filter_functions->max_ev_len = max_ev_len;
	_lwroc_user_filter_functions->name = "filter_vmm";
	HEAP_INIT(g_ms_heap, cmp_ms, 1 << 17, fail);
	return;
fail:
	err(EXIT_FAILURE, "MS heap init failed");
}

void
loop(lwroc_pipe_buffer_consumer *pipe_buf, const lwroc_thread_block
    *thread_block, lwroc_data_pipe_handle *data_handle, volatile int
    *terminate)
{
	char buf[1 << 16];
	size_t buf_bytes = 0;
	int frame_had = 0;
	uint32_t frame_prev;
	int is_sync = 0;

	for (;;) {
		lwroc_iterate_event iter_data;
		const lmd_event_10_1_host *header;
		size_t size, i;
		unsigned chunk_bytes;
		int ret;

		lmd_subevent_10_1_host const *subev_header;
		lwroc_iterate_event subev_data;
		uint8_t const *srcp;

		/* Get sub-event. */

		ret = lwroc_lmd_pipe_get_event(pipe_buf, thread_block,
		    &header, &size, &iter_data);

		if (ret == LWROC_LMD_PIPE_GET_EVENT_FAILURE) {
			LWROC_BUG("Filter failed to get event.");
		}
		if (ret == LWROC_LMD_PIPE_GET_EVENT_WOULD_BLOCK) {
			struct timeval timeout;
			if (*terminate) {
				break;
			}
			timeout.tv_sec = 0;
			timeout.tv_usec = 100000;
			lwroc_thread_block_get_token_timeout(thread_block,
			    &timeout);
			goto update_monitor;
		}

		if (ret == LWROC_LMD_PIPE_GET_EVENT_STICKY ||
		    ret == LWROC_LMD_PIPE_GET_EVENT_INTERNAL) {
			LWROC_FATAL("Sticky event not handled.");
		}

		if (!lwroc_lmd_get_subevent(&iter_data, &subev_header,
		    &subev_data)) {
			LWROC_FATAL("No subevent found.");
		}

		/*
		 * Put payload on top of current buffer contents.
		 * Latest format, big endian:
		 *  32-bit block size, excludes itself.
		 *  32-bit frame counter.
		 *  32-bit header.
		 *  32+16 bit markers/hits.
		 */

		srcp = subev_data._p;
		if (subev_data._size < 4) {
			LWROC_FATAL("Not enough payload for chunk-size.");
		}

		chunk_bytes =
		    srcp[0] << 24 | srcp[1] << 16 |
		    srcp[2] <<  8 | srcp[3] <<  0;
		if (4 + chunk_bytes != subev_data._size) {
			char tmp[256];

			snprintf(tmp, sizeof tmp,
			    "Chunk bytes (4+%u) != sub-event bytes (%u).",
			    chunk_bytes, subev_data._size);
			LWROC_FATAL(tmp);
		}
		srcp += 4;
		memcpy(buf + buf_bytes, srcp, chunk_bytes);
		buf_bytes += chunk_bytes;

		lwroc_pipe_buffer_did_read(pipe_buf, size);

		/* Extract from buf. */

#define BUF_R32(ofs) ntohl(*(uint32_t const *)(buf + i + (ofs)))
#define BUF_R16(ofs) ntohs(*(uint16_t const *)(buf + i + (ofs)))
		for (i = 0; i < buf_bytes;) {
			uint32_t id;

			id = BUF_R32(1 * sizeof(uint32_t));
			if ((0xffffff00 & id) == 0x564d3300) {
				uint32_t frame;

				/*
				 * Probably a header:
				 *  32-bit frame-counter.
				 *  32-bit id + fec-id.
				 *  32-bit udp timestamp.
				 *  32-bit offset overflow.
				 */
				frame = BUF_R32(0 * sizeof(uint32_t));
				if (frame_had && frame_prev + 1 != frame) {
					printf("Frame counter "
					    "mismatch (prev=0x%08x, "
					    "curr=0x%08x)!\n",
					    frame_prev, frame);
				}
				/* ts = BUF_R32(2 * sizeof(uint32_t)). */
				/* of = BUF_R32(3 * sizeof(uint32_t)). */
				frame_had = 1;
				frame_prev = frame;

				if (!is_sync) {
					printf("Found data parsing sync.\n");
					is_sync = 1;
				}

				i += 4 * sizeof(uint32_t);
			} else if (i + 6 <= buf_bytes) {
				uint32_t u32;
				uint16_t u16;
				unsigned is_hit;

				/* Hit or marker. */
				u32 = BUF_R32(0);
				i += sizeof(uint32_t);
				u16 = BUF_R16(0);
				i += sizeof(uint16_t);

				if (0x8000 & u16) {
					process_hit(u32, u16);
				} else {
					process_marker(u32, u16);
				}
			} else {
				if (is_sync) {
					printf("Lost data parsing sync.\n");
					is_sync = 0;
				}
				++i;
			}
		}

		/* Transform ms/hits to HT and populate heaps. */
		raw2heap();

		/* Look for ROI's. */
		roi(pipe_buf, data_handle);

		/* Move any remaining data to the pre-buf space. */
		buf_bytes -= i;
		memmove(buf, buf + i, buf_bytes);

update_monitor:
		LWROC_FILTER_MON_CHECK(LWROC_FILTER_USER, 0);
	}
}

void
max_ev_len(uint32_t *max_len, uint32_t *add_len)
{
	(void)add_len;
	*max_len = 0x10000;
}
