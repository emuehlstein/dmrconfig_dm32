/*
 * Interface to Baofeng DM-32UV (experimental).
 *
 * The read implementation follows the protocol and memory layout documented in
 * dm32_reference/read_connection.md and dm32_reference/codeplug_format.md. It
 * performs the OEM CPS handshake, downloads the mapped regions, and presents a
 * consolidated view of the strings and channel slots we currently understand.
 *
 * Write support is intentionally omitted until the erase/program semantics are
 * verified on hardware.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>

#include "radio.h"
#include "util.h"

#define DM32_BAUD               115200

#define MEMSZ                   0x200000u
#define DM32_PAGE_SIZE          0x1000u
#define DM32_PAGE_MASK          (~(DM32_PAGE_SIZE - 1u))

#define OFFSET_FW_VERSION       0x00000030u
#define LEN_FW_VERSION          32u

#define OFFSET_BOOT_LINE1       0x00005001u
#define OFFSET_BOOT_LINE2       0x0000500Fu
#define LEN_BOOT_LINE           16u

/*
 * Block metadata types. The DM-32UV stores a 1-byte type marker at
 * (block_base + 0xFFF) for every 4 KiB block in the main config region.
 * Discovery reads that byte per block to locate data dynamically.
 * Values per infamy/DM32-Protocol-Spec (MIT).
 */
#define DM32_META_EMPTY         0x00u
#define DM32_META_INVALID       0xFFu
#define DM32_META_MSG           0x0Au   /* Quick text messages            */
#define DM32_META_RXGROUP       0x0Fu   /* RX group list (109-byte entries)*/
#define DM32_META_SCANLIST      0x11u   /* Scan lists (57-byte entries)   */
#define DM32_META_CHAN_FIRST    0x12u   /* Channel block 0 (has header)   */
#define DM32_META_CHAN_LAST     0x41u   /* Channel block 47               */
#define DM32_META_TXCONTACT_LO  0x42u   /* TX contact map, channels 1-2048*/
#define DM32_META_TXCONTACT_HI  0x43u   /* TX contact map, channels 2049+ */
#define DM32_META_TG_FIRST      0x44u   /* Talkgroup block 0              */
#define DM32_META_TG_LAST       0x48u   /* Talkgroup block 4              */
#define DM32_META_ZONE          0x5Cu   /* Zones (145-byte entries)       */
#define DM32_META_RADIOID       0x67u   /* Radio's own DMR ID list        */

/* Main config block bounds (fallback if V-frame 0x0A is unavailable). */
#define DM32_MAIN_START         0x001000u
#define DM32_MAIN_END           0x0C8FFFu
#define DM32_META_OFFSET        0x0FFFu  /* type byte within a 4 KiB block */
#define DM32_MAX_CHAN_BLOCKS    48u

/* Channel record (48 bytes, little-endian; spec 05-DATA-STRUCTURES). */
#define DM32_CHANNEL_SIZE       0x30u
#define DM32_CHANNEL_HDR        0x10u    /* header size in first block      */
#define LEN_CHANNEL_NAME        16u
#define MAX_CHANNELS            4000u

/* Zone record (145 bytes, metadata 0x5C). */
#define DM32_ZONE_SIZE          145u
#define DM32_ZONE_HDR           16u
#define LEN_ZONE_NAME           11u

/* Scan list record (57 bytes, metadata 0x11). */
#define DM32_SCANLIST_SIZE      57u
#define LEN_SCANLIST_NAME       11u

/* Radio ID record (16 bytes, metadata 0x67). */
#define DM32_RADIOID_SIZE       16u
#define LEN_RADIOID_NAME        12u

typedef struct __attribute__((packed)) {
    uint8_t  name[16];       /* 0x00 ASCII, null-term, 0xFF pad           */
    uint8_t  rx_bcd[4];      /* 0x10 RX freq, BCD little-endian           */
    uint8_t  tx_bcd[4];      /* 0x14 TX freq, BCD little-endian           */
    uint8_t  mode_flags;     /* 0x18 mode / forbid-TX / power / lone      */
    uint8_t  scan_bw;        /* 0x19 bandwidth / scan-add / scanlist id   */
    uint8_t  talkaround;     /* 0x1A talkaround / aprs / reverse          */
    uint8_t  emergency;      /* 0x1B emergency                            */
    uint8_t  squelch_aprs;   /* 0x1C squelch (7-4) / aprs mode            */
    uint8_t  analog;         /* 0x1D vox / scramble / compander / talkback*/
    uint8_t  reserved_1e;    /* 0x1E                                      */
    uint8_t  ptt_id;         /* 0x1F ptt id display / value               */
    uint8_t  color_code;     /* 0x20 DMR color code 0-15                  */
    uint8_t  rx_tone[2];     /* 0x21 RX CTCSS/DCS                         */
    uint8_t  tx_tone[2];     /* 0x23 TX CTCSS/DCS                         */
    uint8_t  extra[6];       /* 0x25..0x2A misc flags                     */
    uint8_t  radio_id_index; /* 0x2B index into radio-ID list, 0xFF none  */
    uint8_t  reserved_2c[4]; /* 0x2C padding                              */
} channel_raw_t;

/* Channel mode_flags (byte 0x18). */
#define DM32_MODE_MASK          0xF0u
#define DM32_MODE_SHIFT         4
#define DM32_MODE_ANALOG        0u
#define DM32_MODE_DIGITAL       1u
#define DM32_FORBID_TX_BIT      0x08u
#define DM32_POWER_MASK         0x06u
#define DM32_POWER_SHIFT        1
#define DM32_LONE_BIT           0x01u
/* Channel scan_bw (byte 0x19). */
#define DM32_BW_WIDE_BIT        0x80u
#define DM32_SCAN_ADD_BIT       0x40u
#define DM32_SCANLIST_MASK      0x3Cu
#define DM32_SCANLIST_SHIFT     2
/* Channel byte 0x1D on a digital channel: bits 3-0 color code, bit 4 time slot. */
#define DM32_DIG_CC_MASK        0x0Fu
#define DM32_DIG_TS_BIT         0x10u

typedef struct {
    uint8_t  id;
    uint32_t start;      /* first byte of segment (little-endian u32) */
    uint32_t end;        /* last byte of segment  (little-endian u32) */
} dm32_vseg_t;

static char dm32_board_id[32];
static char dm32_fw_version_cached[32];
static char dm32_build_date[32];
static dm32_vseg_t dm32_vsegs[16];
static unsigned dm32_nvsegs;

/* Discovered main-config bounds and per-type block addresses. */
static uint32_t dm32_main_start = DM32_MAIN_START;
static uint32_t dm32_main_end   = DM32_MAIN_END;
static uint32_t dm32_chan_blocks[DM32_MAX_CHAN_BLOCKS];
static unsigned dm32_chan_nblocks;
static uint32_t dm32_zone_block;      /* 0 = not found */
static uint32_t dm32_scan_block;
static uint32_t dm32_rxgroup_block;
static uint32_t dm32_radioid_block;
static uint32_t dm32_msg_block;
static uint32_t dm32_tg_block;        /* talkgroups (metadata 0x44)        */
static uint32_t dm32_txc_lo_block;    /* TX contact, channels 1-2048 (0x42)*/
static uint32_t dm32_txc_hi_block;    /* TX contact, channels 2049+ (0x43) */

#define DM32_MAX_TG 256
typedef struct {
    char     name[17];
    uint32_t number;
    uint8_t  call_type;
} dm32_tg_t;
static dm32_tg_t dm32_tgs[DM32_MAX_TG];
static unsigned  dm32_ntgs;
static int       dm32_tgs_parsed;

static uint32_t dm32_read_addrs[256];
static unsigned dm32_read_addrs_n;
static unsigned dm32_written_max;

static inline uint32_t dm32_page_base(uint32_t addr)
{
    return addr & DM32_PAGE_MASK;
}

static int dm32_is_ascii(unsigned char c)
{
    return c >= 32 && c <= 126;
}

static void dm32_mark_read(uint32_t addr)
{
    for (unsigned i = 0; i < dm32_read_addrs_n; ++i) {
        if (dm32_read_addrs[i] == addr) {
            return;
        }
    }
    if (dm32_read_addrs_n < (sizeof(dm32_read_addrs) / sizeof(dm32_read_addrs[0]))) {
        dm32_read_addrs[dm32_read_addrs_n++] = addr;
    }
}

static void dm32_trim_right(char *s)
{
    size_t n = strlen(s);
    while (n > 0 && (s[n - 1] == ' ' || s[n - 1] == '\t')) {
        s[--n] = '\0';
    }
}

static size_t dm32_copy_ascii(char *dst, size_t dstlen, const unsigned char *src, size_t maxlen)
{
    size_t out = 0;
    if (!dst || dstlen == 0) {
        return 0;
    }
    while (out + 1 < dstlen && out < maxlen) {
        unsigned char c = src[out];
        if (c == 0x00 || c == 0xFF) {
            break;
        }
        if (!dm32_is_ascii(c)) {
            break;
        }
        dst[out++] = (char)c;
    }
    dst[out] = '\0';
    dm32_trim_right(dst);
    return out;
}

static double dm32_bcd_mhz(const uint8_t *p)
{
    /*
     * Frequencies are 4-byte BCD stored little-endian. Reverse the bytes
     * to big-endian, then read the eight BCD nibbles most-significant
     * first. e.g. stored 00 50 53 14 -> 14 53 50 00 -> 14535000 -> 145.35.
     */
    unsigned long long digits = 0;
    for (int i = 3; i >= 0; --i) {
        digits = digits * 10 + ((p[i] >> 4) & 0xF);
        digits = digits * 10 + (p[i] & 0xF);
    }
    return digits / 100000.0;
}

static const unsigned char *dm32_mem_ptr(uint32_t offset, size_t len)
{
    if (offset + len > MEMSZ) {
        return NULL;
    }
    return &radio_mem[offset];
}

static void dm32_send_ascii(const char *s)
{
    if (trace_flag) {
        fprintf(stderr, "DM32: send '%s'\n", s);
    }
    serial_write((const unsigned char *)s, (int)strlen(s));
}

static int dm32_read_exact(unsigned char *buf, int n, int timeout_msec)
{
    int got = 0;
    while (got < n) {
        int r = serial_read(buf + got, n - got, timeout_msec);
        if (r <= 0) {
            break;
        }
        got += r;
    }
    return got;
}

static int dm32_collect_reads(unsigned char *out, int maxlen, int msec)
{
    int total = 0;
    int iters = msec / 50;
    if (iters < 1) {
        iters = 1;
    }
    while (iters-- > 0) {
        unsigned char buf[256];
        int n = serial_read(buf, sizeof(buf), 50);
        if (n > 0) {
            if (out && total < maxlen) {
                int cp = n;
                if (total + cp > maxlen) {
                    cp = maxlen - total;
                }
                memcpy(out + total, buf, cp);
            }
            total += n;
        }
    }
    return total;
}

static void dm32_extract_board_id(const unsigned char *buf, int n)
{
    int best_start = -1;
    int best_len = 0;
    for (int i = 0; i < n;) {
        if (!dm32_is_ascii(buf[i])) {
            ++i;
            continue;
        }
        int j = i;
        while (j < n && dm32_is_ascii(buf[j])) {
            ++j;
        }
        int len = j - i;
        if (len >= 5 && len > best_len) {
            best_start = i;
            best_len = len;
        }
        i = j;
    }
    if (best_start >= 0) {
        int copy = best_len;
        if (copy > (int)sizeof(dm32_board_id) - 1) {
            copy = (int)sizeof(dm32_board_id) - 1;
        }
        memcpy(dm32_board_id, buf + best_start, copy);
        dm32_board_id[copy] = '\0';
    }
}

static int dm32_read_header_sync(unsigned char hdr[6], int timeout_msec)
{
    unsigned char b;
    int waited = 0;
    while (waited < timeout_msec) {
        int r = dm32_read_exact(&b, 1, 150);
        if (r <= 0) {
            waited += 200;
            continue;
        }
        if (b != 0x57) {
            continue;
        }
        hdr[0] = b;
        if (dm32_read_exact(hdr + 1, 5, 2000) != 5) {
            return -1;
        }
        return 0;
    }
    return -1;
}

static int dm32_read_block(uint32_t addr24, uint16_t len)
{
    unsigned char cmd[6];
    unsigned char hdr[6];

    int priming = (addr24 >= 0xFF0000u);
    if (!priming && (uint64_t)addr24 + len > MEMSZ) {
        return -1;
    }

    cmd[0] = 0x52;
    cmd[1] = addr24 & 0xFF;          /* address is 24-bit little-endian */
    cmd[2] = (addr24 >> 8) & 0xFF;
    cmd[3] = (addr24 >> 16) & 0xFF;
    cmd[4] = len & 0xFF;
    cmd[5] = (len >> 8) & 0xFF;

    if (trace_flag) {
        fprintf(stderr, "DM32: R %02X %02X %02X %02X %02X\n",
                cmd[1], cmd[2], cmd[3], cmd[4], cmd[5]);
    }
    if (serial_write(cmd, 6) < 0) {
        return -1;
    }

    if (dm32_read_header_sync(hdr, 4000) != 0) {
        return -1;
    }
    if (hdr[0] != 0x57 || hdr[1] != cmd[1] || hdr[2] != cmd[2] ||
        hdr[3] != cmd[3] || hdr[4] != cmd[4] || hdr[5] != cmd[5]) {
        if (trace_flag) {
            fprintf(stderr, "DM32: unexpected header\n");
            print_hex(hdr, 6);
        }
        return -1;
    }

    unsigned toread = len;
    unsigned offset = 0;
    while (toread > 0) {
        unsigned char buf[512];
        int chunk = (toread > sizeof(buf)) ? (int)sizeof(buf) : (int)toread;
        int r = dm32_read_exact(buf, chunk, 2000);
        if (r <= 0) {
            return -1;
        }
        if (!priming) {
            memcpy(radio_mem + addr24 + offset, buf, r);
            offset += r;
            toread -= r;
            if (addr24 + offset > dm32_written_max) {
                dm32_written_max = addr24 + offset;
            }
        } else {
            offset += r;
            toread -= r;
        }
    }

    if (!priming) {
        dm32_mark_read(dm32_page_base(addr24));
        double pct = (dm32_written_max / (double)MEMSZ) * 100.0;
        if (pct > 100.0) {
            pct = 100.0;
        }
        radio_progress = (int)pct;
    }
    return 0;
}

static int dm32_read_block_retry(uint32_t addr24, uint16_t len, int attempts)
{
    for (int i = 0; i < attempts; ++i) {
        if (dm32_read_block(addr24, len) == 0) {
            return 0;
        }
        usleep(50000);
    }
    return -1;
}

static int dm32_read_v_frame(uint8_t *type, uint8_t *length, unsigned char *payload, int maxlen, int timeout_msec)
{
    unsigned char b;
    int waited = 0;
    while (waited < timeout_msec) {
        int r = serial_read(&b, 1, 50);
        if (r <= 0) {
            waited += 50;
            continue;
        }
        if (b != 0x56) {
            continue;
        }
        unsigned char hdr[2];
        if (dm32_read_exact(hdr, 2, 200) != 2) {
            return -1;
        }
        uint8_t t = hdr[0];
        uint8_t len = hdr[1];
        if (len > maxlen) {
            unsigned char tmp[256];
            int remain = len;
            while (remain > 0) {
                int chunk = (remain > (int)sizeof(tmp)) ? (int)sizeof(tmp) : remain;
                int got = dm32_read_exact(tmp, chunk, 200);
                if (got <= 0) {
                    break;
                }
                remain -= got;
            }
            return -1;
        }
        if (dm32_read_exact(payload, len, 500) != len) {
            return -1;
        }
        if (type) {
            *type = t;
        }
        if (length) {
            *length = len;
        }
        return 0;
    }
    return -1;
}

static void dm32_parse_v_reply(uint8_t type, uint8_t len, const unsigned char *pl)
{
    if (type == 0x01) {
        size_t copy = len < sizeof(dm32_fw_version_cached) - 1 ? len : sizeof(dm32_fw_version_cached) - 1;
        memcpy(dm32_fw_version_cached, pl, copy);
        dm32_fw_version_cached[copy] = '\0';
    } else if (type == 0x03) {
        size_t copy = len < sizeof(dm32_build_date) - 1 ? len : sizeof(dm32_build_date) - 1;
        memcpy(dm32_build_date, pl, copy);
        dm32_build_date[copy] = '\0';
    } else if (type == 0x06 || type == 0x07 || type == 0x08 ||
               type == 0x09 || type == 0x0A || type == 0x0E || type == 0x0F) {
        if (len == 8) {
            /* 8-byte pointer = start(u32 LE) + end(u32 LE). */
            uint32_t start = (uint32_t)pl[0] | ((uint32_t)pl[1] << 8) |
                             ((uint32_t)pl[2] << 16) | ((uint32_t)pl[3] << 24);
            uint32_t end   = (uint32_t)pl[4] | ((uint32_t)pl[5] << 8) |
                             ((uint32_t)pl[6] << 16) | ((uint32_t)pl[7] << 24);
            if (dm32_nvsegs < (sizeof(dm32_vsegs) / sizeof(dm32_vsegs[0]))) {
                dm32_vsegs[dm32_nvsegs].id = type;
                dm32_vsegs[dm32_nvsegs].start = start;
                dm32_vsegs[dm32_nvsegs].end = end;
                ++dm32_nvsegs;
            }
            if (type == 0x0A && end > start) {
                dm32_main_start = start;
                dm32_main_end = end;
            }
        }
    }
}

static void dm32_collect_handshake_info(void)
{
    unsigned char buf[512];

    dm32_send_ascii("PSEARCH");
    int n = dm32_collect_reads(buf, sizeof(buf), 200);
    dm32_extract_board_id(buf, n);

    dm32_send_ascii("PASSSTA");
    n = dm32_collect_reads(buf, sizeof(buf), 200);
    if (!dm32_board_id[0]) {
        dm32_extract_board_id(buf, n);
    }

    dm32_send_ascii("SYSINFO");
    n = dm32_collect_reads(buf, sizeof(buf), 250);
    if (!dm32_board_id[0]) {
        dm32_extract_board_id(buf, n);
    }

    unsigned char request[5] = { 0x56, 0x00, 0x00, 0x40, 0x0D };
    serial_write(request, 5);
    uint8_t t, l;
    unsigned char payload[256];
    if (dm32_read_v_frame(&t, &l, payload, sizeof(payload), 800) == 0) {
        dm32_parse_v_reply(t, l, payload);
    }

    for (int i = 1; i <= 16; ++i) {
        if (i == 12) {
            continue;
        }
        unsigned char req[5] = { 0x56, 0x00, 0x00, 0x00, (unsigned char)i };
        serial_write(req, 5);
        if (dm32_read_v_frame(&t, &l, payload, sizeof(payload), 800) == 0) {
            dm32_parse_v_reply(t, l, payload);
        }
    }
}

static void dm32_enter_program_mode(void)
{
    static const unsigned char preamble[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0x0C,
                                              'P',  'R',  'O',  'G',  'R',  'A',  'M' };
    serial_write(preamble, sizeof(preamble));
    usleep(30000);

    static const unsigned char b02[] = { 0x02 };
    static const unsigned char b06[] = { 0x06 };
    serial_write(b02, sizeof(b02));
    dm32_collect_reads(NULL, 0, 80);
    serial_write(b06, sizeof(b06));
    dm32_collect_reads(NULL, 0, 120);
}

/* Read the 1-byte type marker at (block_addr + 0xFFF); returns -1 on error. */
static int dm32_read_meta(uint32_t block_addr)
{
    uint32_t a = block_addr + DM32_META_OFFSET;
    if (a + 1 > MEMSZ) {
        return -1;
    }
    if (dm32_read_block_retry(a, 1, 2) != 0) {
        return -1;
    }
    return radio_mem[a];
}

/*
 * Classify every 4 KiB block in the main config region by the type marker
 * stored at (block + 0xFFF) in radio_mem, building the dynamic layout map.
 * Channel blocks are ordered by their metadata value (0x12..0x41), since
 * metadata order is the logical channel order, independent of address order.
 * Works both after a live download and on an image already loaded into memory.
 */
static void dm32_classify(void)
{
    dm32_chan_nblocks = 0;
    dm32_zone_block = 0;
    dm32_scan_block = 0;
    dm32_rxgroup_block = 0;
    dm32_radioid_block = 0;
    dm32_msg_block = 0;
    dm32_tg_block = 0;
    dm32_txc_lo_block = 0;
    dm32_txc_hi_block = 0;
    dm32_ntgs = 0;
    dm32_tgs_parsed = 0;

    uint32_t start = dm32_page_base(dm32_main_start);
    uint32_t end = dm32_main_end;

    for (uint32_t addr = start;
         addr <= end && addr + DM32_PAGE_SIZE <= MEMSZ;
         addr += DM32_PAGE_SIZE) {
        uint8_t type = radio_mem[addr + DM32_META_OFFSET];
        if (type == DM32_META_EMPTY || type == DM32_META_INVALID) {
            continue;
        }
        if (trace_flag) {
            fprintf(stderr, "DM32: block 0x%06X type 0x%02X\n", addr, type);
        }

        if (type >= DM32_META_CHAN_FIRST && type <= DM32_META_CHAN_LAST) {
            if (dm32_chan_nblocks < DM32_MAX_CHAN_BLOCKS) {
                /* insertion sort by stored metadata byte */
                unsigned pos = dm32_chan_nblocks;
                while (pos > 0 &&
                       radio_mem[dm32_chan_blocks[pos - 1] + DM32_META_OFFSET] > type) {
                    dm32_chan_blocks[pos] = dm32_chan_blocks[pos - 1];
                    --pos;
                }
                dm32_chan_blocks[pos] = addr;
                ++dm32_chan_nblocks;
            }
        } else if (type == DM32_META_ZONE && !dm32_zone_block) {
            dm32_zone_block = addr;
        } else if (type == DM32_META_SCANLIST && !dm32_scan_block) {
            dm32_scan_block = addr;
        } else if (type == DM32_META_RXGROUP && !dm32_rxgroup_block) {
            dm32_rxgroup_block = addr;
        } else if (type == DM32_META_RADIOID && !dm32_radioid_block) {
            dm32_radioid_block = addr;
        } else if (type == DM32_META_MSG && !dm32_msg_block) {
            dm32_msg_block = addr;
        } else if (type == DM32_META_TG_FIRST && !dm32_tg_block) {
            dm32_tg_block = addr;
        } else if (type == DM32_META_TXCONTACT_LO && !dm32_txc_lo_block) {
            dm32_txc_lo_block = addr;
        } else if (type == DM32_META_TXCONTACT_HI && !dm32_txc_hi_block) {
            dm32_txc_hi_block = addr;
        }
    }
}

/*
 * Live discovery: probe the type marker of every block in the main config
 * region (bounds from V-frame 0x0A) over the serial link, then classify.
 */
static void dm32_discover(void)
{
    uint32_t start = dm32_page_base(dm32_main_start);
    uint32_t end = dm32_main_end;

    for (uint32_t addr = start;
         addr <= end && addr + DM32_PAGE_SIZE <= MEMSZ;
         addr += DM32_PAGE_SIZE) {
        dm32_read_meta(addr);
    }
    dm32_classify();
}

/* Fetch full 4 KiB payloads for every block located during discovery. */
static void dm32_read_discovered(void)
{
    for (unsigned i = 0; i < dm32_chan_nblocks; ++i) {
        dm32_read_block_retry(dm32_chan_blocks[i], DM32_PAGE_SIZE, 2);
    }
    if (dm32_zone_block) {
        dm32_read_block_retry(dm32_zone_block, DM32_PAGE_SIZE, 2);
    }
    if (dm32_scan_block) {
        dm32_read_block_retry(dm32_scan_block, DM32_PAGE_SIZE, 2);
    }
    if (dm32_rxgroup_block) {
        dm32_read_block_retry(dm32_rxgroup_block, DM32_PAGE_SIZE, 2);
    }
    if (dm32_radioid_block) {
        dm32_read_block_retry(dm32_radioid_block, DM32_PAGE_SIZE, 2);
    }
    if (dm32_msg_block) {
        dm32_read_block_retry(dm32_msg_block, DM32_PAGE_SIZE, 2);
    }
    if (dm32_tg_block) {
        dm32_read_block_retry(dm32_tg_block, DM32_PAGE_SIZE, 2);
    }
    if (dm32_txc_lo_block) {
        dm32_read_block_retry(dm32_txc_lo_block, DM32_PAGE_SIZE, 2);
    }
    if (dm32_txc_hi_block) {
        dm32_read_block_retry(dm32_txc_hi_block, DM32_PAGE_SIZE, 2);
    }
}

static int dm32_channel_blank(const channel_raw_t *ch)
{
    const unsigned char *raw = (const unsigned char *)ch;
    unsigned ff = 0;
    unsigned zz = 0;
    for (unsigned i = 0; i < sizeof(*ch); ++i) {
        if (raw[i] == 0xFF) {
            ++ff;
        }
        if (raw[i] == 0x00) {
            ++zz;
        }
    }
    if (ff == sizeof(*ch) || zz == sizeof(*ch)) {
        return 1;
    }
    return raw[0] == 0x00 || raw[0] == 0xFF;
}

/*
 * Parse the talkgroup list (metadata block 0x44). Entry 1 begins with a
 * 1-byte header (0x00); every entry is then flag(1) + name(16) + null(1) +
 * number(3 LE) + call_type(1) + pad(2) = 24 bytes.
 */
static void dm32_parse_talkgroups(void)
{
    dm32_ntgs = 0;
    dm32_tgs_parsed = 1;

    const unsigned char *b = dm32_tg_block
        ? dm32_mem_ptr(dm32_tg_block, DM32_PAGE_SIZE) : NULL;
    if (!b) {
        return;
    }

    unsigned off = 1;   /* skip entry-1 header byte */
    while (dm32_ntgs < DM32_MAX_TG) {
        unsigned name_off = off + 1;    /* skip flag byte */
        if (name_off + 23 > DM32_PAGE_SIZE) {
            break;
        }
        if (b[name_off] == 0x00 || b[name_off] == 0xFF) {
            break;      /* end sentinel */
        }
        dm32_tg_t *tg = &dm32_tgs[dm32_ntgs];
        dm32_copy_ascii(tg->name, sizeof(tg->name), b + name_off, 16);
        const unsigned char *c = b + name_off + 16 + 1;   /* after name+null */
        tg->number = c[0] | ((uint32_t)c[1] << 8) | ((uint32_t)c[2] << 16);
        tg->call_type = c[3];
        ++dm32_ntgs;
        off += 24;
    }
}

/* Resolve a channel's TX talkgroup name via blocks 0x42/0x43; NULL if none. */
static const char *dm32_channel_contact(unsigned ch_index)
{
    const unsigned char *blk;
    unsigned off;

    if (ch_index >= 1 && ch_index <= 2048 && dm32_txc_lo_block) {
        blk = dm32_mem_ptr(dm32_txc_lo_block, DM32_PAGE_SIZE);
        off = (ch_index - 1) * 2;
    } else if (ch_index > 2048 && dm32_txc_hi_block) {
        blk = dm32_mem_ptr(dm32_txc_hi_block, DM32_PAGE_SIZE);
        off = (ch_index & 0x7FF) * 2;
    } else {
        return NULL;
    }
    if (!blk || off + 1 >= DM32_PAGE_SIZE) {
        return NULL;
    }
    /* 12-bit talkgroup index: (byte0>>4)<<8 | byte1; 0 = none. */
    unsigned tg = (((blk[off] >> 4) & 0x0F) << 8) | blk[off + 1];
    if (tg == 0) {
        return NULL;
    }
    if (!dm32_tgs_parsed) {
        dm32_parse_talkgroups();
    }
    if (tg >= 1 && tg <= dm32_ntgs) {
        return dm32_tgs[tg - 1].name;
    }
    return NULL;
}

static void dm32_print_talkgroups(FILE *out)
{
    if (!dm32_tgs_parsed) {
        dm32_parse_talkgroups();
    }
    fprintf(out, "\nTalkgroups\n");
    fprintf(out, "Idx  %-16s %-10s Type\n", "Name", "DMR ID");
    if (dm32_ntgs == 0) {
        fprintf(out, "(none)\n");
        return;
    }
    for (unsigned i = 0; i < dm32_ntgs; ++i) {
        const char *ct = dm32_tgs[i].call_type == 0x03 ? "Private" :
                         dm32_tgs[i].call_type == 0x04 ? "Group" :
                         dm32_tgs[i].call_type == 0x05 ? "All" : "?";
        fprintf(out, "%4u  %-16s %-10u %s\n",
                i + 1, dm32_tgs[i].name, dm32_tgs[i].number, ct);
    }
}

static void dm32_format_tone(const uint8_t *t, char *buf, size_t n)
{
    uint8_t lo = t[0], hi = t[1];
    if ((lo == 0xFF && hi == 0xFF) || (lo == 0x00 && hi == 0x00)) {
        snprintf(buf, n, "-");
    } else if (hi >= 0x80) {
        /* DCS: BCD digits. hundreds = hi low nibble; tens/ones = lo nibbles.
         * high byte 0x80-0xBF = normal (N), >=0xC0 = inverted (P). */
        unsigned code = (hi & 0x0F) * 100 + ((lo >> 4) & 0x0F) * 10 + (lo & 0x0F);
        snprintf(buf, n, "D%03u%c", code, (hi >= 0xC0) ? 'P' : 'N');
    } else {
        /* CTCSS: BCD. hundreds = hi[7-4], tens = hi[3-0],
         * ones = lo[7-4], tenths = lo[3-0]. */
        unsigned whole = ((hi >> 4) & 0x0F) * 100 + (hi & 0x0F) * 10 +
                         ((lo >> 4) & 0x0F);
        snprintf(buf, n, "%u.%u", whole, lo & 0x0F);
    }
}

static void dm32_print_channels(FILE *out)
{
    static const char *power_name[] = { "Low", "Mid", "High", "High" };

    fprintf(out, "\nChannels\n");
    fprintf(out, "Idx  %-16s %-10s %-10s Md Pwr  TS CC Tone   %-14s Scan\n",
            "Name", "RX (MHz)", "TX (MHz)", "Contact");

    if (dm32_chan_nblocks == 0) {
        fprintf(out, "(no channel blocks discovered)\n");
        return;
    }

    /* Total channel count is a little-endian u32 in the first block header. */
    uint32_t b0 = dm32_chan_blocks[0];
    uint32_t total = radio_mem[b0] | ((uint32_t)radio_mem[b0 + 1] << 8) |
                     ((uint32_t)radio_mem[b0 + 2] << 16) |
                     ((uint32_t)radio_mem[b0 + 3] << 24);
    if (total == 0 || total > MAX_CHANNELS) {
        total = MAX_CHANNELS;
    }

    unsigned shown = 0;
    unsigned index = 0;
    for (unsigned bi = 0; bi < dm32_chan_nblocks && index < total; ++bi) {
        uint32_t base = dm32_chan_blocks[bi];
        uint32_t off = (bi == 0) ? DM32_CHANNEL_HDR : 0;
        for (; off + DM32_CHANNEL_SIZE <= DM32_PAGE_SIZE && index < total;
             off += DM32_CHANNEL_SIZE) {
            const channel_raw_t *ch =
                (const channel_raw_t *)dm32_mem_ptr(base + off, sizeof(channel_raw_t));
            if (!ch) {
                break;
            }
            ++index;
            if (dm32_channel_blank(ch)) {
                continue;
            }

            char name[LEN_CHANNEL_NAME + 1];
            dm32_copy_ascii(name, sizeof(name), ch->name, sizeof(ch->name));
            if (!name[0]) {
                continue;
            }

            double rx = dm32_bcd_mhz(ch->rx_bcd);
            double tx = dm32_bcd_mhz(ch->tx_bcd);
            unsigned mode = (ch->mode_flags & DM32_MODE_MASK) >> DM32_MODE_SHIFT;
            unsigned power = (ch->mode_flags & DM32_POWER_MASK) >> DM32_POWER_SHIFT;
            int digital = (mode & 1);   /* 0=Analog,1=Digital,2=FixAna,3=FixDig */
            int scan = (ch->scan_bw & DM32_SCAN_ADD_BIT) != 0;

            char ts[3], cc[4], tone[8];
            if (digital) {
                /* byte 0x1D: bit4 time slot, bits3-0 color code */
                snprintf(ts, sizeof(ts), "%d",
                         (ch->analog & DM32_DIG_TS_BIT) ? 2 : 1);
                snprintf(cc, sizeof(cc), "%u", ch->analog & DM32_DIG_CC_MASK);
                snprintf(tone, sizeof(tone), "-");
            } else {
                snprintf(ts, sizeof(ts), "-");
                snprintf(cc, sizeof(cc), "-");
                dm32_format_tone(ch->rx_tone, tone, sizeof(tone));
            }

            const char *contact = digital ? dm32_channel_contact(index) : NULL;
            fprintf(out, "%4u  %-16s %10.5f %10.5f  %c  %-4s %-2s %-2s %-6s %-14s %s\n",
                    index, name, rx, tx,
                    digital ? 'D' : 'A',
                    power_name[power & 3], ts, cc, tone,
                    contact ? contact : "-",
                    scan ? "yes" : "-");
            ++shown;
        }
    }

    if (shown == 0) {
        fprintf(out, "(no populated channels)\n");
    }
}

static void dm32_print_zones(FILE *out)
{
    fprintf(out, "\nZones\n");
    fprintf(out, "Idx  %-11s Channels\n", "Name");

    if (!dm32_zone_block) {
        fprintf(out, "(none)\n");
        return;
    }

    unsigned shown = 0;
    for (unsigned n = 1; ; ++n) {
        uint32_t off = DM32_ZONE_HDR + (n - 1) * DM32_ZONE_SIZE;
        if (off + DM32_ZONE_SIZE > DM32_PAGE_SIZE) {
            break;
        }
        const unsigned char *z = dm32_mem_ptr(dm32_zone_block + off, DM32_ZONE_SIZE);
        if (!z) {
            break;
        }
        if (z[0] == 0xFF || z[0] == 0x00) {
            break;      /* first empty slot terminates the list */
        }

        char name[LEN_ZONE_NAME + 1];
        dm32_copy_ascii(name, sizeof(name), z, LEN_ZONE_NAME);
        if (!name[0]) {
            continue;
        }

        unsigned cnt = z[0x10];
        if (cnt > 64) {
            cnt = 64;
        }
        fprintf(out, "%4u  %-11s ", n, name);
        for (unsigned i = 0; i < cnt; ++i) {
            unsigned ch = z[0x11 + i * 2] | ((unsigned)z[0x12 + i * 2] << 8);
            if (ch == 0) {
                break;
            }
            fprintf(out, "%s%u", i ? "," : "", ch);
        }
        fprintf(out, "\n");
        ++shown;
    }

    if (shown == 0) {
        fprintf(out, "(none)\n");
    }
}

static void dm32_print_scanlists(FILE *out)
{
    fprintf(out, "\nScan Lists\n");
    fprintf(out, "Idx  Name\n");

    if (!dm32_scan_block) {
        fprintf(out, "(none)\n");
        return;
    }

    unsigned count = radio_mem[dm32_scan_block];   /* count byte at +0x00 */
    unsigned shown = 0;
    for (unsigned n = 1; n <= count && n <= 71; ++n) {
        uint32_t off = (DM32_SCANLIST_SIZE * n) - 56;   /* entry N: (57*N)-56 */
        if (off + DM32_SCANLIST_SIZE > DM32_PAGE_SIZE) {
            break;
        }
        const unsigned char *s = dm32_mem_ptr(dm32_scan_block + off, DM32_SCANLIST_SIZE);
        if (!s) {
            break;
        }
        char name[LEN_SCANLIST_NAME + 1];
        dm32_copy_ascii(name, sizeof(name), s, LEN_SCANLIST_NAME);
        if (!name[0]) {
            continue;
        }
        fprintf(out, "%4u  %s\n", n, name);
        ++shown;
    }

    if (shown == 0) {
        fprintf(out, "(none)\n");
    }
}

static void dm32_print_radioids(FILE *out)
{
    fprintf(out, "\nRadio IDs\n");
    fprintf(out, "Idx  %-12s DMR ID\n", "Name");

    if (!dm32_radioid_block) {
        fprintf(out, "(none)\n");
        return;
    }

    uint32_t count = radio_mem[dm32_radioid_block] |
                     ((uint32_t)radio_mem[dm32_radioid_block + 1] << 8) |
                     ((uint32_t)radio_mem[dm32_radioid_block + 2] << 16) |
                     ((uint32_t)radio_mem[dm32_radioid_block + 3] << 24);
    if (count == 0 || count > 256) {
        count = 256;
    }

    unsigned shown = 0;
    for (unsigned n = 0; n < count; ++n) {
        /* 16-byte header holds the count; entry N starts at 0x10 + N*0x10. */
        uint32_t off = (n + 1) * DM32_RADIOID_SIZE;
        if (off + DM32_RADIOID_SIZE > DM32_PAGE_SIZE) {
            break;
        }
        const unsigned char *r = dm32_mem_ptr(dm32_radioid_block + off, DM32_RADIOID_SIZE);
        if (!r) {
            break;
        }
        char name[LEN_RADIOID_NAME + 1];
        dm32_copy_ascii(name, sizeof(name), r + 3, LEN_RADIOID_NAME);
        uint32_t id = r[0] | ((uint32_t)r[1] << 8) | ((uint32_t)r[2] << 16);
        if (id == 0 && !name[0]) {
            continue;
        }
        fprintf(out, "%4u  %-12s %u\n", n + 1, name[0] ? name : "-", id);
        ++shown;
    }

    if (shown == 0) {
        fprintf(out, "(none)\n");
    }
}

static void dm32_print_boot_message(FILE *out)
{
    char line1[LEN_BOOT_LINE + 1];
    char line2[LEN_BOOT_LINE + 1];
    const unsigned char *l1 = dm32_mem_ptr(OFFSET_BOOT_LINE1, LEN_BOOT_LINE);
    const unsigned char *l2 = dm32_mem_ptr(OFFSET_BOOT_LINE2, LEN_BOOT_LINE);
    if (l1) {
        dm32_copy_ascii(line1, sizeof(line1), l1, LEN_BOOT_LINE);
    } else {
        line1[0] = '\0';
    }
    if (l2) {
        dm32_copy_ascii(line2, sizeof(line2), l2, LEN_BOOT_LINE);
    } else {
        line2[0] = '\0';
    }

    if (line1[0] || line2[0]) {
        fprintf(out, "Boot Message: %s / %s\n",
                line1[0] ? line1 : "-",
                line2[0] ? line2 : "-");
    }
}

static void dm32_print_v_segments(FILE *out)
{
    if (dm32_nvsegs == 0) {
        return;
    }
    fprintf(out, "\nV-frame Segments (id, start, end, size)\n");
    for (unsigned i = 0; i < dm32_nvsegs; ++i) {
        uint32_t sz = (dm32_vsegs[i].end >= dm32_vsegs[i].start)
                      ? (dm32_vsegs[i].end - dm32_vsegs[i].start + 1) : 0;
        fprintf(out, " 0x%02X  0x%06X - 0x%06X  %8u\n",
                dm32_vsegs[i].id,
                dm32_vsegs[i].start,
                dm32_vsegs[i].end,
                sz);
    }
}

static void dm32_print_pages_table(FILE *out)
{
    fprintf(out, "\n# Pages read (base, non-FF, non-00)\n");
    for (unsigned i = 0; i < dm32_read_addrs_n; ++i) {
        uint32_t base = dm32_read_addrs[i];
        const unsigned char *pg = dm32_mem_ptr(base, DM32_PAGE_SIZE);
        if (!pg) {
            continue;
        }
        unsigned nonff = 0;
        unsigned non00 = 0;
        for (unsigned j = 0; j < DM32_PAGE_SIZE; ++j) {
            if (pg[j] != 0xFF) {
                ++nonff;
            }
            if (pg[j] != 0x00) {
                ++non00;
            }
        }
        fprintf(out, "0x%06X  %5u  %5u\n", base, nonff, non00);
    }
}

static void dm32_print_version(radio_device_t *radio, FILE *out)
{
    (void)radio;
    fprintf(out, "Baofeng DM-32UV (experimental)\n");
    const unsigned char *fw_ptr = dm32_mem_ptr(OFFSET_FW_VERSION, LEN_FW_VERSION);
    char fw_buf[LEN_FW_VERSION + 1] = {0};
    if (fw_ptr) {
        dm32_copy_ascii(fw_buf, sizeof(fw_buf), fw_ptr, LEN_FW_VERSION);
    }

    if (dm32_board_id[0]) {
        fprintf(out, "Board: %s\n", dm32_board_id);
    }
    if (dm32_fw_version_cached[0]) {
        fprintf(out, "Firmware (via V-frame): %s\n", dm32_fw_version_cached);
    } else if (fw_buf[0]) {
        fprintf(out, "Firmware: %s\n", fw_buf);
    }
    if (dm32_build_date[0]) {
        fprintf(out, "Build Date: %s\n", dm32_build_date);
    }
    dm32_print_boot_message(out);
}

static void dm32_print_config(radio_device_t *radio, FILE *out, int verbose)
{
    fprintf(out, "Radio: %s\n", radio->name);
    if (verbose) {
        dm32_print_version(radio, out);
    }

    /* When displaying a loaded image, classify the layout from memory first. */
    if (dm32_chan_nblocks == 0 && dm32_zone_block == 0) {
        dm32_classify();
    }

    dm32_print_radioids(out);
    dm32_print_talkgroups(out);
    dm32_print_zones(out);
    dm32_print_scanlists(out);
    dm32_print_channels(out);

    if (verbose) {
        dm32_print_v_segments(out);
        dm32_print_pages_table(out);
    }
}

static void dm32_download(radio_device_t *radio)
{
    (void)radio;
    if (serial_open_found(DM32_BAUD) < 0) {
        fprintf(stderr, "DM32: unable to open serial port at %d\n", DM32_BAUD);
        return;
    }

    dm32_board_id[0] = '\0';
    dm32_fw_version_cached[0] = '\0';
    dm32_build_date[0] = '\0';
    dm32_nvsegs = 0;
    dm32_read_addrs_n = 0;
    dm32_written_max = 0;
    radio_progress = 0;

    (void)serial_pulse_rts_dtr();
    usleep(150000);

    fprintf(stderr, "DM32: performing handshake...\n");
    dm32_collect_handshake_info();

    fprintf(stderr, "DM32: requesting resource banner...\n");
    static const unsigned char resource_req[] = { 0x47, 0x00, 0x00, 0x00, 0x00, 0x01 };
    serial_write(resource_req, sizeof(resource_req));
    dm32_collect_reads(NULL, 0, 200);

    fprintf(stderr, "DM32: entering PROGRAM mode...\n");
    dm32_enter_program_mode();

    fprintf(stderr, "DM32: discovering memory layout...\n");
    dm32_discover();

    fprintf(stderr, "DM32: reading data blocks...\n");
    dm32_read_discovered();
    radio_progress = 100;
}

static void dm32_upload(radio_device_t *radio, int cont_flag)
{
    (void)radio;
    (void)cont_flag;
    fprintf(stderr, "DM32: writing is not implemented yet.\n");
    exit(-1);
}

static int dm32_is_compatible(radio_device_t *radio)
{
    (void)radio;
    const unsigned char *fw_ptr = dm32_mem_ptr(OFFSET_FW_VERSION, 4);
    if (!fw_ptr) {
        return 0;
    }
    return fw_ptr[0] == 'D' && fw_ptr[1] == 'M' && fw_ptr[2] == '3' && fw_ptr[3] == '2';
}

static void dm32_read_image(radio_device_t *radio, FILE *img)
{
    (void)radio;
    size_t total = fread(radio_mem, 1, MEMSZ, img);
    if (total < MEMSZ) {
        memset(radio_mem + total, 0xFF, MEMSZ - total);
    } else if (total > MEMSZ) {
        fprintf(stderr, "DM32 image larger than expected (%zu bytes).\n", total);
        exit(-1);
    }
}

static void dm32_save_image(radio_device_t *radio, FILE *img)
{
    (void)radio;
    fwrite(radio_mem, 1, MEMSZ, img);
}

static int dm32_verify_config(radio_device_t *radio)
{
    (void)radio;
    return 1;
}

static void dm32_parse_parameter(radio_device_t *radio, char *param, char *value)
{
    (void)radio;
    (void)param;
    (void)value;
}

static int dm32_parse_header(radio_device_t *radio, char *line)
{
    (void)radio;
    (void)line;
    return 0;
}

static int dm32_parse_row(radio_device_t *radio, int table_id, int first_row, char *line)
{
    (void)radio;
    (void)table_id;
    (void)first_row;
    (void)line;
    return 0;
}

static void dm32_update_timestamp(radio_device_t *radio)
{
    (void)radio;
}

radio_device_t radio_dm32 = {
    "Baofeng DM-32UV",
    dm32_download,
    dm32_upload,
    dm32_is_compatible,
    dm32_read_image,
    dm32_save_image,
    dm32_print_version,
    dm32_print_config,
    dm32_verify_config,
    dm32_parse_parameter,
    dm32_parse_header,
    dm32_parse_row,
    dm32_update_timestamp,
    NULL,
};
