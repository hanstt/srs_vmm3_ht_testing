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

/* From 0x1 to 0x3f. */
#define WR_ID 0x10

#define ROI_LEFT_US -10.0
#define ROI_RIGHT_US +10.0

#define MS_WINDOW_NS 100.0

/*
 * Heimtime pulse time differences multiplied by this, i.e.:
 * slew_counter_add ~= 0x1000000 -> set to 0x1.
 *                  ~= 0xa000000 -> set to 0xa.
 */
#define HT_SCALE 0xa

/* If it takes 5 s to collect data from VMM, force-build events. */
#define HT_DT_TOO_LITTLE_S 0.1
#define HT_DT_TOO_MUCH_S 5.0

/*
 * VMM stuff.
 * There are two kinds of timestamps:
 *  ht = Heimtime
 *  ts = VMM
 */

static uint32_t g_evtcnt;

struct HtMs {
	uint16_t	fec_i;
	uint16_t	vmm_i;
	uint64_t	ht;
	uint16_t	adc;
};
HEAP_HEAD(HeapMs, HtMs);
static struct HeapMs g_ms_heap;
/* "Global", total. */
static size_t	g_ms_heap_maxg;
/* "Local", since last time. */
static size_t	g_ms_heap_maxl;

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

#define TS2NS 22.5 /* (5.24288 / 0.233045) */

#define HT2NS (HT_SCALE / 10.)

#define ROI_LEFT_HT (HT2NS * 1e3 * ROI_LEFT_US)
#define ROI_RIGHT_HT (HT2NS * 1e3 * ROI_RIGHT_US)

#define MS_WINDOW_HT (HT2NS * 1e0 * MS_WINDOW_NS)

#define HT_DT_TOO_LITTLE_HT (HT2NS * 1e9 * HT_DT_TOO_LITTLE_S)
#define HT_DT_TOO_MUCH_HT (HT2NS * 1e9 * HT_DT_TOO_MUCH_S)

#define LENGTH(x) (sizeof x / sizeof *x)
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))

#define PF_TS(ts) \
	(uint32_t)((ts) >> 32), \
	(uint32_t)((ts) & 0xffffffff)

/* Circular buffer code. */

/* wr == rd -> empty, wr+1 == rd -> full. */
#define CIRC_DECL(Type) struct { \
	size_t	capacity; \
	size_t	rd_i; \
	size_t	wr_i; \
	struct	Type *arr; \
}
#define CIRC_GETNUM(circ) ((circ.wr_i - circ.rd_i) % circ.capacity)
#define CIRC_INIT(circ, N) do { \
	circ.capacity = N; \
	circ.rd_i = 0; \
	circ.wr_i = 0; \
	circ.arr = calloc(N, sizeof *circ.arr); \
} while (0)
#define CIRC_STEP(circ, idx, ofs) ((circ.idx + ofs) & (circ.capacity - 1))
#define CIRC_PEEK(ptr, circ, ofs) do { \
	size_t i_; \
	assert(CIRC_GETNUM(circ) > 0); \
	i_ = CIRC_STEP(circ, rd_i, ofs); \
	ptr = &circ.arr[i_]; \
} while (0)
#define CIRC_PEEK_REV(ptr, circ, ofs) do { \
	size_t i_; \
	assert(CIRC_GETNUM(circ) > 0); \
	i_ = CIRC_STEP(circ, wr_i, circ.capacity - ofs - 1); \
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
			LWROC_ERROR("Too many " name "s, dropping data!"); \
		} \
		circ.rd_i = CIRC_STEP(circ, rd_i, 1); \
	} \
} while (0)
#define CIRC_FOREACH(idx, circ) \
    for (idx = circ.rd_i; idx != circ.wr_i; \
	idx = (idx + 1) & (circ.capacity - 1))

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
		/* Locked 1<<19 pulses. */
		int	has_carry;
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
	uint64_t ht_latest;
	/* 256 * 5.24 ms ~= 1.3 s. */
	CIRC_DECL(HtPair) ht_buf;
	/* 1<<17 / 1.3 s ~= 100 kHz. */
	CIRC_DECL(VmmChannel) ch_buf;
	struct	HeapHit hit_heap;
	struct {
		size_t	ht_maxg;
		size_t	ht_maxl;
		size_t	ch_maxg;
		size_t	ch_maxl;
		size_t	heap_maxg;
		size_t	heap_maxl;
	} stats;
};
struct Fec {
	size_t	vmm_num;
	struct	Vmm *vmm_arr;
};

static struct Fec *g_fec_arr;
static size_t g_fec_num;
static uint32_t g_fec_i;

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
	    (0 == a_vmm_i && 57 == a_ch_i) ||
	    (1 == a_vmm_i &&  7 == a_ch_i);
}

int
is_ms_ch(unsigned a_vmm_i, unsigned a_ch_i)
{
	return
	    (0 == a_vmm_i && 63 == a_ch_i) ||
	    (1 == a_vmm_i &&  1 == a_ch_i);
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

struct Fec *
fec_get(unsigned a_i)
{
	if (a_i >= g_fec_num) {
		struct Fec *arr;
		unsigned i, num;

		num = a_i + 1;
		arr = calloc(num, sizeof *arr);
		if (!arr) {
			err(EXIT_FAILURE, "calloc");
		}
		memcpy(arr, g_fec_arr, g_fec_num * sizeof *arr);
		free(g_fec_arr);
		g_fec_arr = arr;
		g_fec_num = num;
	}
	return &g_fec_arr[a_i];
fail:
	err(EXIT_FAILURE, "Hit heap init failed");
}

struct Vmm *
vmm_get(struct Fec *a_fec, unsigned a_i)
{
	struct Vmm *vmm;

	if (a_i >= a_fec->vmm_num) {
		struct Vmm *arr;
		unsigned i, num;

		num = a_i + 1;
		arr = calloc(num, sizeof *arr);
		if (!arr) {
			err(EXIT_FAILURE, "calloc");
		}
		memcpy(arr, a_fec->vmm_arr, a_fec->vmm_num * sizeof *arr);
		free(a_fec->vmm_arr);
		for (i = a_fec->vmm_num; i < num; ++i) {
			HEAP_INIT(arr[i].hit_heap, cmp_hit, 1 << 17, fail);
		}
		a_fec->vmm_arr = arr;
		a_fec->vmm_num = num;
	}
	vmm = &a_fec->vmm_arr[a_i];
	if (!vmm->ht_buf.arr) {
		CIRC_INIT(vmm->ht_buf, 256);
		CIRC_INIT(vmm->ch_buf, 1 << 17);
	}
	return vmm;
fail:
	err(EXIT_FAILURE, "Hit heap init failed");
}

void
process_ht(unsigned a_vmm_i, unsigned a_ch_i, uint32_t a_ts_curr)
{
	uint32_t dt_arr[3];
	struct Fec *fec = fec_get(g_fec_i);
	struct Vmm *vmm = vmm_get(fec, a_vmm_i);
	struct HtPair *pair;
	size_t i;
	unsigned bit_ts;
	int do_bit = 0, do_clear = 0, do_inc = 0;

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
if(0)if(0==a_vmm_i)printf("%2u %2u %2u  %08x,%08x,%08x  %10.3f,%10.3f,%10.3f.\n",
	g_fec_i, a_vmm_i, a_ch_i,
	dt_arr[0],
	dt_arr[1],
	dt_arr[2],
	1e-6 * TS2NS * HT_SCALE * dt_arr[0],
	1e-6 * TS2NS * HT_SCALE * dt_arr[1],
	1e-6 * TS2NS * HT_SCALE * dt_arr[2]);

#define HT_P (uint32_t)(5.24288 * 1e6 / (TS2NS * HT_SCALE))
#define HT_0 (uint32_t)(0.16384 * 1e6 / (TS2NS * HT_SCALE))
#define HT_1 (uint32_t)(0.65536 * 1e6 / (TS2NS * HT_SCALE))
#define HT_APPROX(l, r) (fabs((l) - (r)) < 1e2)

	if (!vmm->ht_build.has_carry) {
		/* Find 1<<19 pulses, then we start decoding. */
		if (!(HT_APPROX(dt_arr[1], HT_P) &&
		      HT_APPROX(dt_arr[2], HT_P))) {
			return;
		}
		LWROC_INFO_FMT("%2u:%2u: Found HT carry.", g_fec_i, a_vmm_i);
		vmm->ht_build.has_carry = 1;
		do_clear = 1;
	}

	if (HT_APPROX(dt_arr[0], HT_P)) {
		do_inc = 1;
		do_clear = 1;
	} else if (HT_APPROX(dt_arr[0], 2*HT_P)) {
		/* Lost periodic pulse, increment twice. */
		do_inc = 2;
		do_clear = 1;
	} else if (dt_arr[0] > 2.5*HT_P) {
		LWROC_ERROR_FMT("%2u:%2u: Lost HT carry signal.",
		    g_fec_i, a_vmm_i);
		vmm->ht_build.has_carry = 0;
		return;
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
#define TEST_BIT(bit) do { \
	if (HT_APPROX(dt_arr[2], HT_P - 2*HT_##bit) || \
	    HT_APPROX(dt_arr[2], HT_P - 1*HT_##bit)) { \
		bit_ts = vmm->ht_build.history[3]; \
		do_bit = 1 + bit; \
	} else if ((HT_APPROX(dt_arr[0], 1*HT_##bit) && \
	            HT_APPROX(dt_arr[1], 1*HT_##bit)) || \
		   HT_APPROX(dt_arr[1], 2*HT_##bit)) { \
		bit_ts = vmm->ht_build.history[2] + \
		    (uint32_t)((1e6 * (HT_P - 2*HT_##bit)) / \
		    (TS2NS * HT_SCALE)); \
		do_bit = 1 + bit; \
	} \
	} while (0)
	TEST_BIT(0);
	TEST_BIT(1);
	if (do_bit) {
		vmm->ht_build.mask |= (do_bit - 1) << vmm->ht_build.bit_i;
		++vmm->ht_build.bit_i;
		do_inc = 1;
	}

	/* Create a timestamp for every 1<<19 pulse. */
	if (0) if (vmm->ht_build.ht.has && do_inc) {
		vmm->ht_build.ht.ht += do_inc << 19;
		/*
		 * Don't complain about lost HT's, will happen when no
		 * MS coming in.
		 */
		CIRC_PUSH(pair, "Heimtime", vmm->ht_buf, 0);
		pair->ht = vmm->ht_build.ht.ht;
		pair->ts = a_ts_curr;
	}

	if (32 == vmm->ht_build.bit_i) {
		/* Full Heimtime decoded! */
		uint64_t ht = (uint64_t)vmm->ht_build.mask << 24;

		if (vmm->ht_build.ht.has) {
			if (vmm->ht_build.ht.ht + (4 << 24) != ht) {
				LWROC_ERROR_FMT(
				    "vmm=%u: Heimtime expected=%08x "
				    "but got=%08x!",
				    a_vmm_i,
				    (uint32_t)(vmm->ht_build.ht.ht >> 24),
				    vmm->ht_build.mask);
			} else if (vmm->ht_build.ht.ht > ht) {
				LWROC_ERROR_FMT(
				    "vmm=%u: Heimtime reversed %08x -> %08x!",
				    a_vmm_i,
				    (uint32_t)(vmm->ht_build.ht.ht >> 24),
				    vmm->ht_build.mask);
			}
		} else {
			LWROC_INFO_FMT("%2u:%2u: Found HT=0x%08x.",
			    g_fec_i, a_vmm_i,
			    vmm->ht_build.mask);
		}
		do_clear = 1;
		vmm->ht_build.ht.has = 1;
		vmm->ht_build.ht.ht = ht;
		if (0)
			LWROC_INFO_FMT("%2u:%2u: HT = %08x.",
			    g_fec_i, a_vmm_i,
			    (uint32_t)vmm->ht_build.mask);
		if (1) {
			CIRC_PUSH(pair, "Heimtime", vmm->ht_buf, 0);
			pair->ht = vmm->ht_build.ht.ht;
			pair->ts = a_ts_curr;
		}
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

	struct Fec *fec;
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
		LWROC_INFO_FMT("Hit fec=%2u  "
		    "vmm=%2u  "
		    "ch=%2u  "
		    "a/t=%4u/%4u  "
		    "ofs/bcid=%3d/%4u.",
		    g_fec_i,
		    vmm_i,
		    ch_i,
		    adc, tdc,
		    ofs, bcid);

	fec = fec_get(g_fec_i);
	vmm = vmm_get(fec, vmm_i);
	ts = vmm->ts_marker + (ofs * 4096) + bcid;
	if (0)
		LWROC_INFO_FMT("%08x%08x.", PF_TS(ts));

	if (is_ht_ch(vmm_i, ch_i)) {
		/* Stop! Heimtime! */
		process_ht(vmm_i, ch_i, ts);
	} else {
		/* MS or hit. */
		struct VmmChannel *ch;

		CIRC_PUSH(ch, "channel", vmm->ch_buf, 0);
		ch->ch = ch_i;
		ch->ts = ts;
		ch->adc = over << 15 | adc;
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

	struct Fec *fec;
	struct Vmm *vmm;

	if (0)
		LWROC_INFO_FMT("Marker fec=%2u vmm=%2u  "
		    "%08x:%04x  "
		    "ts=0x%08x%08x.",
		    g_fec_i,
		    vmm_i,
		    a_u32, a_u16,
		    PF_TS(ts));

	fec = fec_get(g_fec_i);
	vmm = vmm_get(fec, vmm_i);
	vmm->ts_marker = ts;
}

uint64_t g_ht_prev = 0;

void
roi(lwroc_pipe_buffer_consumer *pipe_buf, lwroc_data_pipe_handle *data_handle)
{
	size_t fec_i, vmm_i;
	uint64_t ht_last_max = 0;
	uint64_t ht_last_min = UINT64_MAX;

	/*
	 * Find max/min latest hits in any VMM.
	 *  - If (max - first_ms_ht) is large, build so buffers are emptied.
	 *  - If (min - first_ms_ht) is small, events may fragment.
	 */
	for (fec_i = 0; fec_i < g_fec_num; ++fec_i) {
		struct Fec *fec;

		fec = &g_fec_arr[fec_i];
		for (vmm_i = 0; vmm_i < fec->vmm_num; ++vmm_i) {
			struct Vmm *vmm;

			vmm = &fec->vmm_arr[vmm_i];
			ht_last_max = MAX(ht_last_max, vmm->ht_latest);
			ht_last_min = MIN(ht_last_min, vmm->ht_latest);
		}
	}

	/* Find as many ROI's as possible in current data. */
	while (g_ms_heap.num) {
		struct HtMs const *msp;
		uint64_t ht0;

		void *p0;
		lmd_event_10_1_host *ev;
		lmd_subevent_10_1_host *sev;
		uint32_t *p32, *p_vmm_n;
		uint8_t *p8;
		uint32_t header_size;
		uint32_t wr_size;
		uint32_t payload_size;
		uint32_t write_size;
		uint32_t sync_check;
		uint32_t vmm_n;

		/* Peek at 1st MS and see what to do. */
		msp = &g_ms_heap.arr[0];
		ht0 = msp->ht;
		if (ht_last_max - ht0 < HT_DT_TOO_MUCH_HT) {
			/* We don't have a crapload of data. */
			if (ht_last_min - ht0 < HT_DT_TOO_LITTLE_HT) {
				/* We actually have too little, bail. */
				break;
			}
		}

		/* Build LMD event. */

		header_size = sizeof ev + sizeof sev;
		wr_size = (1 + 4 + 1) * sizeof (uint32_t);
		payload_size = 0x100000;

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

		sync_check = msp->adc;

		*p32++ = SYNC_CHECK_MAGIC | SYNC_CHECK_RECV |
		    (sync_check & 0xffff);

		p_vmm_n = p32++;
		vmm_n = 0;

		/* For each VMM with the MS, extract hits in ROI. */
		while (g_ms_heap.num) {
			struct HtMs ms;
			struct Fec *fec;
			struct Vmm *vmm;
			uint64_t dht;
			uint32_t *p_hit_n;
			uint32_t hit_n;

			msp = &g_ms_heap.arr[0];
			dht = msp->ht - ht0;
			if (dht > MS_WINDOW_HT) {
				break;
			}
			HEAP_EXTRACT(g_ms_heap, ms, fail);

			if (msp->ht < g_ht_prev) {
				LWROC_ERROR_FMT("%2u:%2u: Master starts "
				    "out of order! (%08x%08x -> %08x%08x).",
				    msp->fec_i, msp->vmm_i,
				    PF_TS(g_ht_prev),
				    PF_TS(msp->ht));
			}
			g_ht_prev = msp->ht;

			/* Write MS. */
			*p32++ = ms.adc << 16 | ms.fec_i << 4 | ms.vmm_i;
			*p32++ = (uint32_t)(ms.ht >> 32);
			*p32++ = (uint32_t)ms.ht;
			++vmm_n;

			fec = fec_get(ms.fec_i);
			vmm = vmm_get(fec, ms.vmm_i);

			p_hit_n = p32++;
			hit_n = 0;
			while (vmm->hit_heap.num) {
				struct HtHit const *hp;
				struct HtHit hit;

				hp = &vmm->hit_heap.arr[0];
				if (ms.ht + ROI_RIGHT_HT < hp->ht) {
					/* Next hit outside ROI. */
					break;
				}
				HEAP_EXTRACT(vmm->hit_heap, hit, fail);
				if (ms.ht + ROI_LEFT_HT < hit.ht) {
					uint32_t dht;

					/* Write hit. */
					dht = 0x00ffffff &
					    (hit.ht - ms.ht + (1 << 23));
					*p32++ = hit.ch << 24 | dht;
					*p32++ = hit.adc << 16 | hit.tdc;
					++hit_n;
				}
			}
			*p_hit_n = hit_n;
		}
		*p_vmm_n = vmm_n;

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
	unsigned fec_i, vmm_i;

	for (fec_i = 0; fec_i < g_fec_num; ++fec_i) {
		struct Fec *fec = fec_get(fec_i);

		for (vmm_i = 0; vmm_i < fec->vmm_num; ++vmm_i) {
			struct Vmm *vmm = vmm_get(fec, vmm_i);
			struct HtPair const *ht0;
			struct HtPair const *ht1;
			size_t n;

			n = CIRC_GETNUM(vmm->ht_buf);
			vmm->stats.ht_maxg = MAX(vmm->stats.ht_maxg, n);
			vmm->stats.ht_maxl = MAX(vmm->stats.ht_maxl, n);
			n = CIRC_GETNUM(vmm->ch_buf);
			vmm->stats.ch_maxg = MAX(vmm->stats.ch_maxg, n);
			vmm->stats.ch_maxl = MAX(vmm->stats.ch_maxl, n);

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
				uint32_t dt0, dt1, ts;

				CIRC_PEEK(ch, vmm->ch_buf, 0);
				ts = ch->ts;
				dt0 = ts - ht0->ts;
				dt1 = ht1->ts - ts;
				if (dt0 > 0x80000000) {
					/* Hit before 1st HT, drop hit. */
					CIRC_DROP(vmm->ch_buf);
					continue;
				}
				if (dt1 > 0x80000000) {
					/* Hit after 2nd HT, drop HT. */
					CIRC_DROP(vmm->ht_buf);
					goto find_ht_span;
				}
				ht = (ts - ht0->ts)
				    * (ht1->ht - ht0->ht)
				    / (ht1->ts - ht0->ts)
				    + ht0->ht;
				vmm->ht_latest = MAX(vmm->ht_latest, ht);
				if (is_ms_ch(vmm_i, ch->ch)) {
					struct HtMs ms;

					ms.fec_i = fec_i;
					ms.vmm_i = vmm_i;
					ms.ht = ht;
					ms.adc = ch->adc;
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
				LWROC_ERROR_FMT(
				    "%2u:%2u: Heap overflow, "
				    "Heimtime missing?", fec_i, vmm_i);
				CIRC_DROP(vmm->ch_buf);
			}
			g_ms_heap_maxg = MAX(g_ms_heap_maxg, g_ms_heap.num);
			g_ms_heap_maxl = MAX(g_ms_heap_maxl, g_ms_heap.num);
			vmm->stats.heap_maxg = MAX(vmm->stats.heap_maxg,
			    vmm->hit_heap.num);
			vmm->stats.heap_maxl = MAX(vmm->stats.heap_maxl,
			    vmm->hit_heap.num);
		}
	}
}

void
mon(void)
{
	static double t_prev = -1.0;
	double t = time_get();
	size_t fec_i, vmm_i;

	if (t_prev < 0.0) {
		t_prev = t;
	}
	if (t_prev + 1 > t) {
		return;
	}
	t_prev = t;

	printf("MS-heap = %u/%u/%u\n",
	    (unsigned)g_ms_heap_maxl,
	    (unsigned)g_ms_heap_maxg,
	    (unsigned)g_ms_heap.capacity);
	g_ms_heap_maxl = 0;
	for (fec_i = 0; fec_i < g_fec_num; ++fec_i) {
		struct Fec *fec = fec_get(fec_i);

		for (vmm_i = 0; vmm_i < fec->vmm_num; ++vmm_i) {
			struct Vmm *vmm = vmm_get(fec, vmm_i);

			printf("%2u:%2u: "
			    "#HT=%3u/%3u/%3u "
			    "#MS+hit=%6u/%6u/%6u "
			    "#hit-heap=%6u/%6u/%6u\n",
			    (unsigned)fec_i,
			    (unsigned)vmm_i,
			    (unsigned)vmm->stats.ht_maxl,
			    (unsigned)vmm->stats.ht_maxg,
			    (unsigned)vmm->ht_buf.capacity,
			    (unsigned)vmm->stats.ch_maxl,
			    (unsigned)vmm->stats.ch_maxg,
			    (unsigned)vmm->ch_buf.capacity,
			    (unsigned)vmm->stats.heap_maxl,
			    (unsigned)vmm->stats.heap_maxg,
			    (unsigned)vmm->hit_heap.capacity
			    );
			vmm->stats.ht_maxl = 0;
			vmm->stats.ch_maxl = 0;
			vmm->stats.heap_maxl = 0;
		}
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
	uint32_t frame_prev[16];
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
			    chunk_bytes, (unsigned)subev_data._size);
			LWROC_FATAL_FMT("%s", tmp);
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
				g_fec_i = (id & 0xff) >> 4;
				frame = BUF_R32(0 * sizeof(uint32_t));
				if (frame_had && frame_prev[g_fec_i] + 1 !=
				    frame) {
					LWROC_ERROR_FMT("FEC=%u "
					    "frame counter mismatch "
					    "(prev=0x%08x, curr=0x%08x)!",
					    g_fec_i, frame_prev[g_fec_i],
					    frame);
				}
				/* ts = BUF_R32(2 * sizeof(uint32_t)). */
				/* of = BUF_R32(3 * sizeof(uint32_t)). */
				frame_had = 1;
				frame_prev[g_fec_i] = frame;

				if (!is_sync) {
					LWROC_INFO(
					    "Found data parsing sync.");
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
					LWROC_ERROR(
					    "Lost data parsing sync.");
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

		mon();

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
