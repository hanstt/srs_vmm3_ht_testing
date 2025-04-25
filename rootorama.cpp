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
#include <TFile.h>
#include <TTree.h>
#include "ungray_table.h"

#define LENGTH(x) (sizeof x / sizeof *x)

#define PF_TS(ts) \
    (uint32_t)((ts) >> 32), \
    (uint32_t)((ts) & 0xffffffff)

struct Vmm {
  uint64_t	ts_marker;
};

static struct Vmm *g_vmm_arr;
static size_t g_vmm_num;

static uint32_t g_marker;
static uint32_t g_marker_vmm;
static uint64_t g_marker_ts;
static uint32_t g_hit;
static uint32_t g_hit_ofs;
static int g_hit_ofs_signed;
static uint32_t g_hit_vmm;
static uint32_t g_hit_adc;
static uint32_t g_hit_bcid;
static uint32_t g_hit_over;
static uint32_t g_hit_ch;
static uint32_t g_hit_tdc;
static uint64_t g_hit_ts;

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

struct Vmm *
vmm_get(unsigned a_i)
{
  if (a_i >= g_vmm_num) {
    struct Vmm *arr;
    unsigned num;

    num = a_i + 1;
    arr = (struct Vmm *)calloc(num, sizeof *arr);
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
process_hit(uint32_t a_u32, uint16_t a_u16)
{
  int32_t ofs = 0x1f & (a_u32 >> 27);
  if (ofs == 31) ofs = -1;
  int32_t vmm_i = 0x1f & (a_u32 >> 22);
  int32_t adc = 0x3ff & (a_u32 >> 12);
  int32_t bcid = ungray(0xfff & a_u32);
  int32_t over = 0x1 & (a_u16 >> 14);
  int32_t ch_i = 0x3f & (a_u16 >> 8);
  int32_t tdc = 0xff & a_u16;

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
  ts = (int64_t)vmm->ts_marker + ofs * 4096 + bcid;

  g_hit = 1;
  g_hit_ofs  = ofs;
  g_hit_ofs_signed = ofs;
  g_hit_vmm  = vmm_i;
  g_hit_adc  = adc;
  g_hit_bcid = bcid;
  g_hit_over = over;
  g_hit_ch   = ch_i;
  g_hit_tdc  = tdc;
  g_hit_ts   = ts;
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

  g_marker = 1;
  g_marker_vmm = vmm_i;
  g_marker_ts = ts;
}

int
main(int argc, char **argv)
{
  /* 16 potential data from last read + new buf. */
#define BUF_SIZE (1 << 16)
  uint8_t buf[16 + BUF_SIZE];
  size_t buf_ofs = 0;
  int has_header = 0;

  char *opath;
  int fd;
  uint32_t frame_prev = 0xffffffff;

  unsigned tot_byte = 0;
  unsigned tot_marker = 0;
  unsigned tot_hit = 0;
  unsigned tot = 0;

  if (2 != argc) {
    errx(EXIT_FAILURE, "Usage: %s in-file", argv[0]);
  }

  fd = open(argv[1], O_RDONLY);
  if (-1 == fd) {
    err(EXIT_FAILURE, "open(%s)", argv[1]);
  }

  opath = (char *)malloc(strlen(argv[1]) + 5 + 1);
  strcpy(opath, argv[1]);
  strcat(opath, ".root");

  TFile ofile(opath, "RECREATE");
  auto tree = new TTree("tree", "SRS VMM3 data");
  tree->Branch("marker",     &g_marker,     "marker/i");
  tree->Branch("marker_vmm", &g_marker_vmm, "marker_vmm[marker]/i");
  tree->Branch("marker_ts",  &g_marker_ts,  "marker_ts[marker]/l");
  tree->Branch("hit",        &g_hit,        "hit/i");
  tree->Branch("hit_ofs",    &g_hit_ofs_signed,    "hit_ofs[hit]/I");
  tree->Branch("hit_vmm",    &g_hit_vmm,    "hit_vmm[hit]/i");
  tree->Branch("hit_adc",    &g_hit_adc,    "hit_adc[hit]/i");
  tree->Branch("hit_bcid",   &g_hit_bcid,   "hit_bcid[hit]/i");
  tree->Branch("hit_over",   &g_hit_over,   "hit_over[hit]/i");
  tree->Branch("hit_ch",     &g_hit_ch,     "hit_ch[hit]/i");
  tree->Branch("hit_tdc",    &g_hit_tdc,    "hit_tdc[hit]/i");
  tree->Branch("hit_ts",     &g_hit_ts,     "hit_ts[hit]/l");

  for (;;) {
    ssize_t bytes, end, i;

    /* Read block. */
    bytes = read(fd, buf + buf_ofs, BUF_SIZE);
    if (-1 == bytes) {
      err(EXIT_FAILURE, "read");
    } else if (0 == bytes) {
      break;
    }

    tot_byte += bytes;
    tot += bytes;

    /* Parse block. */
#define BUF_R32(ofs) ntohl(*(uint32_t const *)(buf + i + (ofs)))
#define BUF_R16(ofs) ntohs(*(uint16_t const *)(buf + i + (ofs)))
    end = buf_ofs + bytes;
    for (i = 0; i < end;) {
      uint32_t id;
      int is_header;
      int has_size;

      if (i + 6 * sizeof(uint32_t) > end) {
        break;
      }

      g_marker = 0;
      g_hit = 0;

      is_header = 0;
      id = BUF_R32(1 * sizeof(uint32_t));
      is_header = (0xffffff00 & id) == 0x564d3300;
      has_size = 0;
      if (!is_header) {
        id = BUF_R32(2 * sizeof(uint32_t));
        is_header = (0xffffff00 & id) == 0x564d3300;
        has_size = 1;
      }
      if (is_header) {
        uint32_t frame;

        /* Looks like a header. */
        frame = BUF_R32(has_size * sizeof(uint32_t));
        if (0xffffffff != frame_prev &&
            frame_prev + 1 != frame) {
          fprintf(stderr, "Frame counter "
              "mismatch (0x%08x -> 0x%08x)!\n",
              frame_prev, frame);
        }
        frame_prev = frame;

        has_header = 1;

        i += (has_size + 4) * sizeof(uint32_t);
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
          ++tot_hit;
        } else {
          process_marker(u32, u16);
          ++tot_marker;
        }
        if (g_marker || g_hit) {
          tree->Fill();
        }
      } else {
        i += sizeof(uint32_t);
      }
    }

    {
      static double t0 = -1.0;
      double t = time_get();
      if (t0 + 0.1 < t) {
        double dt = t - t0;

        printf("  %.1f MB/s  %.1f markers/s  %.1f hits/s         \r",
            1e-6 * tot_byte / dt, tot_marker / dt, tot_hit / dt);
        fflush(stdout);
        tot_byte = 0;
        tot_marker = 0;
        tot_hit = 0;
        t0 = t;
      }
    }

    /* Move any remaining data to the pre-buf space. */
    buf_ofs = end - i;
    memcpy(buf, buf + i, buf_ofs);
  }
  printf("\nTot=%.1f MB\n", 1e-6 * tot);
  tree->Write();
  return 0;
}
