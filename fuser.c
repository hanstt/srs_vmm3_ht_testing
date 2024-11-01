#include <lwroc_message.h>
#include <lwroc_mon_block.h>
#include <lwroc_net_conn_monitor.h>
#include <lwroc_readout.h>
#include <lwroc_parse_util.h>
#include <lwroc_data_pipe.h>

#include <lmd/lwroc_lmd_event.h>
#include <lmd/lwroc_lmd_white_rabbit_stamp.h>
#include <lmd/lwroc_lmd_ev_sev.h>

#define LENGTH(x) (sizeof x / sizeof *x)

static double time_get(void);

void cmdline_usage(void);
void init(void);
int parse_cmdline_arg(const char *);
void untriggered_loop(int *);

extern struct lwroc_readout_functions _lwroc_readout_functions;
extern volatile int _lwroc_quit;
extern lwroc_thread_block *_main_thread_block;
extern lwroc_pipe_buffer_control *_lwroc_main_data;
extern lwroc_monitor_main_block  _lwroc_mon_main;
extern lwroc_mon_block          *_lwroc_mon_main_handle;
extern lwroc_net_conn_monitor   *_lwroc_mon_main_system_handle;

static struct {
	unsigned	ctrl;
	unsigned	crate;
	unsigned	proc;
	unsigned	wr_id;
	unsigned	rbuf;
	unsigned	sbuf;
} g_config = {
	0, 0, 0, 0,
	1, 100
};
static lmd_stream_handle *g_lmd_stream;
static uint32_t g_event_size;

static struct {
	/* read == write -> empty, read == write + 1 -> full. */
	size_t	read_i;
	size_t	write_i;
	uint8_t	*arr;
} g_buf;

void
cmdline_usage()
{
	printf("  --crate=N  Mark subevents with crate N.\n");
	printf("  --ctrl=N   Mark subevents with ctrl N.\n");
	printf("  --proc=N   Mark subevents with proc N.\n");
	printf("  --wr=N     Mark WR stamp with ID N.\n");
	printf("  --rbuf=N   N MiB for reading (default = 1).\n");
	printf("  --sbuf=N   Allocate N MiB for stating (default = 100).\n");
	printf("\n");
}

void
data_evict_old()
{

}

void
init()
{
	g_buf.arr = malloc(g_config.gbuf);

	g_lmd_stream = lwroc_get_lmd_stream("READOUT_PIPE");
	g_event_size = lwroc_lmd_stream_get_max_ev_len(g_lmd_stream);
	lwroc_init_timestamp_track();
}

void
lwroc_readout__setup_functions()
{
}

void
lwroc_readout_pre_setup_functions()
{
	memset(&g_config, 0, sizeof g_config);

	_lwroc_readout_functions.init = init;
	_lwroc_readout_functions.untriggered_loop = untriggered_loop;
	_lwroc_readout_functions.cmdline_usage = cmdline_usage;
	_lwroc_readout_functions.parse_cmdline_arg = parse_cmdline_arg;
}

int
parse_cmdline_arg(char const *request)
{
	char const *post;

	if (LWROC_MATCH_C_PREFIX("--crate=", post)) {
		g_config.crate = atol(post);
	} else if (LWROC_MATCH_C_PREFIX("--ctrl=", post)) {
		g_config.ctrl = atol(post);
	} else if (LWROC_MATCH_C_PREFIX("--proc=", post)) {
		g_config.proc = atol(post);
	} else if (LWROC_MATCH_C_PREFIX("--wr=", post)) {
		g_config.wr_id = atol(post);
	} else if (LWROC_MATCH_C_PREFIX("--rbuf=", post)) {
		g_config.rbuf = atol(post);
	} else if (LWROC_MATCH_C_PREFIX("--sbuf=", post)) {
		g_config.sbuf = atol(post);
	} else {
		return 0;
	}
	return 1;
}

void
read_event(uint64_t cycle, uint16_t trig)
{
	uint64_t tstamp;
	uint32_t *p32, *wr_p32;
	uint32_t buf_bytes;
	uint32_t tstamp_status;

	lwroc_reserve_event_buffer(g_lmd_stream, cycle, g_event_size, 0, 0);

	lwroc_new_event(g_lmd_stream, &ev, trig);

	/* WR sub-event. */
	info.type = 10;
	info.subtype = 1;
	info.control = g_config.ctrl;
	info.subcrate = g_config.crate;
	info.procid = g_config.proc;
	p32 = (uint32_t *)lwroc_new_subevent(g_lmd_stream,
			LWROC_LMD_SEV_NORMAL, &sev, &info);
	// Skip payload until we've parsed the FOOT payload.
	wr_p32 = p32;
	p32 += 5;
	lwroc_finalise_subevent(g_lmd_stream, LWROC_LMD_SEV_NORMAL, p32);

	/* FOOT sub-event. */
	info.type = 83;
	info.subtype = 8300;
	info.control = g_config.ctrl;
	info.subcrate = g_config.crate;
	info.procid = g_config.proc;
	p32 = (uint32_t *)lwroc_new_subevent(g_lmd_stream,
			LWROC_LMD_SEV_NORMAL, &sev, &info);
	lwroc_finalise_subevent(g_lmd_stream, LWROC_LMD_SEV_NORMAL, p32);

	{
		// We have FOOT rataclock TS, fill in the WR subev.
		lwroc_report_event_timestamp(tstamp, 0);

		*wr_p32++ = g_config.wr_id << 8 |
			(tstamp_status ? WHITE_RABBIT_STAMP_EBID_ERROR : 0);
		*wr_p32++ = WHITE_RABBIT_STAMP_LL16_ID |
			(uint32_t)((tstamp >>  0) & 0xffff);
		*wr_p32++ = WHITE_RABBIT_STAMP_LH16_ID |
			(uint32_t)((tstamp >> 16) & 0xffff);
		*wr_p32++ = WHITE_RABBIT_STAMP_HL16_ID |
			(uint32_t)((tstamp >> 32) & 0xffff);
		*wr_p32++ = WHITE_RABBIT_STAMP_HH16_ID |
			(uint32_t)((tstamp >> 48) & 0xffff);
	}

	lwroc_finalise_event_buffer(g_lmd_stream);
}

void
untriggered_loop(int *start_no_stop)
{
	uint64_t cycle;

	*start_no_stop = 0;
	for (cycle = 0; !_lwroc_main_thread->_terminate; ++cycle) {
		struct lwroc_lmd_subevent_info info;
		lmd_event_10_1_host *ev;
		lmd_subevent_10_1_host *sev;

		/* Potentially evict old data. */
		data_evict_old();

		if (!data_make_event()) {
			/* Need more Heimtime and MS, farm input. */
			bytes = read(STDIN_FILENO, buf, sizeof buf);
			if (-1 == bytes) {
				err(EXIT_FAILURE, "read");
			}
			data_process(buf);
		}

	/*
	 * Farm blocks until we have Heimtime + MS + enough hits so we can
	 * make at least one ROI.
	 */
	for (;;) {
		ssize_t bytes;

		/* Look for Heimtime lock and MS on all VMM's. */
		{
			int is_bad = 0;

			for (i = 0; i < vmm_n; ++i) {
				struct Vmm *vmm;

				vmm = &g_vmm[i];
				is_bad |= !vmm->ht.is_locked;
				is_bad |= 0 == vmm->ms.num;
			}
			if (!is_bad) {
				break;
			}
		}

		if (g_buf.bytes + sizeof buf > LENGTH(g_buf.arr)) {
			break;
		}


	}

		LWROC_MON_CHECK_COPY_BLOCK(_lwroc_mon_main_handle,
		    &_lwroc_mon_main, 0);
		LWROC_MON_CHECK_COPY_CONN_MON_BLOCK(
		    _lwroc_mon_main_system_handle, 0);
	}
}

double
time_get(void)
{
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	return ts.tv_sec + 1e-9 * ts.tv_nsec;
}
