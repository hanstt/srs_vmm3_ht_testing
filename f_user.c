#include <err.h>
#include <fcntl.h>

#include <lwroc_mon_block.h>
#include <lwroc_net_conn_monitor.h>
#include <lwroc_readout.h>
#include <lwroc_parse_util.h>
#include <lwroc_data_pipe.h>
#include <lwroc_leap_seconds.h>

#include <lmd/lwroc_lmd_event.h>
#include <lmd/lwroc_lmd_white_rabbit_stamp.h>
#include <lmd/lwroc_lmd_ev_sev.h>
#include <lmd/lwroc_lmd_util.h>

void init(void);
void read_event(uint64_t cycle, uint16_t trig);
void readout_loop(int *start_no_stop);
void cmdline_usage(void);
int parse_cmdline_arg(const char *request);

extern struct lwroc_readout_functions _lwroc_readout_functions;
extern volatile int _lwroc_quit;

extern lwroc_thread_block *_main_thread_block;
extern lwroc_pipe_buffer_control *_lwroc_main_data;
extern lwroc_monitor_main_block  _lwroc_mon_main;
extern lwroc_mon_block          *_lwroc_mon_main_handle;
extern lwroc_net_conn_monitor   *_lwroc_mon_main_system_handle;

static lmd_stream_handle *g_lmd_stream;

/*****************************************************************************/

struct Config {
	int ctrl;
	int crate;
	int procid;
	char const *fifo_path;
} g_config;

/*****************************************************************************/

void
lwroc_readout_pre_parse_functions(void)
{
	_lwroc_readout_functions.init = init;
	_lwroc_readout_functions.read_event = read_event;
	_lwroc_readout_functions.untriggered_loop = readout_loop;
	_lwroc_readout_functions.cmdline_fcns.usage = cmdline_usage;
	_lwroc_readout_functions.cmdline_fcns.parse_arg = parse_cmdline_arg;
	_lwroc_readout_functions.fmt = &_lwroc_lmd_format_functions;
}

static int g_fifo;
static uint32_t g_event_size;

void
init(void)
{
	g_fifo = open(g_config.fifo_path, O_RDONLY);
	if (-1 == g_fifo) {
		err(EXIT_FAILURE, "open");
	}

	g_lmd_stream = lwroc_get_lmd_stream("READOUT_PIPE");
	g_event_size = lwroc_lmd_stream_get_max_ev_len(g_lmd_stream);
}

void
cmdline_usage(void)
{
	printf("  --crate=N               Mark subevents with crate N.\n");
	printf("  --ctrl=N                Mark subevents with ctrl N.\n");
	printf("  --procid=N              Mark subevents with procID N.\n");
	printf("  --fifo=path             Path to fifo to read from.\n");
}

int
parse_cmdline_arg(const char *request)
{
	const char *post;

	if (LWROC_MATCH_C_PREFIX("--crate=", post)) {
		g_config.crate = strtol(post, NULL, 0);
	} else if (LWROC_MATCH_C_PREFIX("--ctrl=", post)) {
		g_config.ctrl = strtol(post, NULL, 0);
	} else if (LWROC_MATCH_C_PREFIX("--procid=", post)) {
		g_config.procid = strtol(post, NULL, 0);
	} else if (LWROC_MATCH_C_PREFIX("--fifo=", post)) {
		g_config.fifo_path = post;
	} else {
		return 0;
	}
	return 1;
}

void lwroc_readout_setup_functions(void) {}

static char g_buf[1 << 16];
static size_t g_buf_bytes;

void
read_event(uint64_t cycle, uint16_t trig)
{
	struct lwroc_lmd_subevent_info info;
	lmd_event_10_1_host *event;
	lmd_subevent_10_1_host *sev;
	uint32_t *p32;
	unsigned chunk_bytes;
	int do_read = 0;

	for (;;) {
		ssize_t bytes;

		if (g_buf_bytes >= 12) {
			if (0x56 == g_buf[ 8] &&
			    0x4d == g_buf[ 9] &&
			    0x33 == g_buf[10] &&
			    0x20 == g_buf[11]) {
				chunk_bytes =
				    g_buf[0] << 24 |
				    g_buf[1] << 16 |
				    g_buf[2] <<  8 |
				    g_buf[3] <<  0;
				if (g_buf_bytes < chunk_bytes) {
					do_read = 1;
				} else {
					/* Enough data to cover a chunk. */
					chunk_bytes += 4;
					break;
				}
			} else {
				/* Haven't found header, shift. */
				--g_buf_bytes;
				memmove(g_buf, g_buf + 1, g_buf_bytes);
			}
		} else {
			do_read = 1;
		}
		if (do_read) {
			bytes = read(g_fifo,
			    g_buf + g_buf_bytes, sizeof g_buf - g_buf_bytes);
			if (-1 == bytes) {
				err(EXIT_FAILURE, "read");
			}
			g_buf_bytes += bytes;
		}
	}

	info.type = 10;
	info.subtype = 1;
	info.control = g_config.ctrl;
	info.subcrate = g_config.crate;
	info.procid = g_config.procid;

	lwroc_reserve_event_buffer(g_lmd_stream, (uint32_t)cycle,
	    g_event_size, 0, 0);

	lwroc_new_event(g_lmd_stream, &event, trig);
	event->_info.i_trigger = 1;

	p32 = (void *)lwroc_new_subevent(g_lmd_stream, LWROC_LMD_SEV_NORMAL,
	    &sev, &info);

	memcpy(p32, g_buf, chunk_bytes);
	p32 += chunk_bytes / 4;

	lwroc_finalise_subevent(g_lmd_stream, LWROC_LMD_SEV_NORMAL, p32);

	lwroc_finalise_event_buffer(g_lmd_stream);

	/* Move remaining data back. */
	g_buf_bytes -= chunk_bytes;
	memmove(g_buf, g_buf + chunk_bytes, g_buf_bytes);
}

void
readout_loop(int *start_no_stop)
{
	uint64_t cycle;

	*start_no_stop = 0;
	for (cycle = 1; !_lwroc_main_thread->_terminate;) {
		read_event(cycle++, 1);

		LWROC_MON_CHECK_COPY_BLOCK(
		    _lwroc_mon_main_handle, &_lwroc_mon_main, 0);
		LWROC_MON_CHECK_COPY_CONN_MON_BLOCK(
		    _lwroc_mon_main_system_handle, 0);
	}
}
