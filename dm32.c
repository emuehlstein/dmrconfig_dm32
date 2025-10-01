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

#define OFFSET_RADIO_ID_TABLE   0x00006013u
#define STRIDE_RADIO_ID         0x10u
#define MAX_RADIO_IDS           16u

#define OFFSET_MESSAGE_TABLE    0x00007011u
#define STRIDE_MESSAGE          0x81u
#define MAX_MESSAGES            20u
#define LEN_MESSAGE             0x80u

#define OFFSET_SCANLIST_TABLE   0x00008001u
#define STRIDE_SCANLIST         0x39u
#define MAX_SCANLISTS           32u
#define LEN_SCANLIST_NAME       24u

#define OFFSET_ROAM_TABLE       0x00009010u
#define STRIDE_ROAM             0x91u
#define MAX_ROAM_ZONES          32u
#define LEN_ROAM_NAME           24u

#define OFFSET_ZONE_TABLE       0x00011010u
#define STRIDE_ZONE             0x91u
#define MAX_ZONES               250u
#define LEN_ZONE_NAME           24u

#define OFFSET_CHANNEL_TABLE    0x00021010u
#define STRIDE_CHANNEL          0x30u
#define MAX_CHANNELS            4000u
#define LEN_CHANNEL_NAME        16u
#define LEN_CHANNEL_PARAM       24u

#define DM32_POWER_HIGH_BIT     0x04
#define DM32_TS2_BIT            0x10
#define DM32_CC_MASK            0x0F

typedef struct {
    uint32_t address;
    uint32_t length;
    uint32_t offset;
} fragment_t;

static const fragment_t dm32_static_map[] __attribute__((unused)) = {
#include "dm32-map.h"
};

typedef struct __attribute__((packed)) {
    uint8_t label[LEN_CHANNEL_NAME];
    uint8_t rx_bcd[4];
    uint8_t tx_bcd[4];
    uint8_t params[LEN_CHANNEL_PARAM];
} channel_raw_t;

static inline uint32_t channel_offset(uint32_t index)
{
    return OFFSET_CHANNEL_TABLE + (index * STRIDE_CHANNEL);
}

typedef struct {
    uint8_t  id;
    uint32_t base;
    uint16_t mask_le;
    uint16_t stride_le;
} dm32_vseg_t;

static char dm32_board_id[32];
static char dm32_fw_version_cached[32];
static char dm32_build_date[32];
static dm32_vseg_t dm32_vsegs[16];
static unsigned dm32_nvsegs;

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
    unsigned v = p[0] | (p[1] << 8) | (p[2] << 16) | (p[3] << 24);
    unsigned long long bcd = 0;
    for (int i = 0; i < 8; ++i) {
        bcd = bcd * 10 + ((v >> (i * 4)) & 0xF);
    }
    return bcd / 100000.0;
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
    cmd[1] = (addr24 >> 16) & 0xFF;
    cmd[2] = (addr24 >> 8) & 0xFF;
    cmd[3] = addr24 & 0xFF;
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
        if (len == 8 && dm32_nvsegs < (sizeof(dm32_vsegs) / sizeof(dm32_vsegs[0]))) {
            dm32_vsegs[dm32_nvsegs].id = type;
            dm32_vsegs[dm32_nvsegs].base = ((uint32_t)pl[0] << 16) | ((uint32_t)pl[1] << 8) | (uint32_t)pl[2];
            dm32_vsegs[dm32_nvsegs].mask_le = (uint16_t)(pl[4] | (pl[5] << 8));
            dm32_vsegs[dm32_nvsegs].stride_le = (uint16_t)(pl[6] | (pl[7] << 8));
            ++dm32_nvsegs;
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

static void dm32_prime_reads(void)
{
    dm32_read_block_retry(0xFF7F0C, 1, 1);
    dm32_read_block_retry(0xFF8F0C, 1, 1);
    dm32_read_block_retry(0xFF7F01, 1, 1);
    dm32_read_block_retry(0xFF8F01, 1, 1);
    dm32_read_block_retry(0x01F0FF, DM32_PAGE_SIZE, 1);
}

static void dm32_download_plan(void)
{
    static const uint32_t static_pages[] = {
        0x000000u, 0x00001000u, 0x00002000u, 0x00003000u,
        0x00004000u, 0x00005000u, 0x00006000u, 0x00007000u,
        0x00008000u, 0x00009000u, 0x0000A000u, 0x0000B000u,
        0x0000C000u, 0x0000D000u, 0x0000F000u, 0x00011000u,
        0x00012000u, 0x0001F000u
    };

    for (unsigned i = 0; i < sizeof(static_pages) / sizeof(static_pages[0]); ++i) {
        dm32_read_block_retry(static_pages[i], DM32_PAGE_SIZE, 2);
    }

    uint32_t channel_start = dm32_page_base(OFFSET_CHANNEL_TABLE);
    uint32_t channel_end = dm32_page_base(channel_offset(MAX_CHANNELS));
    if (channel_end > MEMSZ - DM32_PAGE_SIZE) {
        channel_end = MEMSZ - DM32_PAGE_SIZE;
    }

    unsigned blank_run = 0;
    for (uint32_t addr = channel_start; addr <= channel_end; addr += DM32_PAGE_SIZE) {
        if (dm32_read_block_retry(addr, DM32_PAGE_SIZE, 2) != 0) {
            break;
        }
        const unsigned char *page = dm32_mem_ptr(addr, DM32_PAGE_SIZE);
        if (!page) {
            break;
        }
        int blank = 1;
        for (unsigned i = 0; i < DM32_PAGE_SIZE; ++i) {
            if (page[i] != 0xFF) {
                blank = 0;
                break;
            }
        }
        if (blank) {
            ++blank_run;
            if (blank_run >= 8 && addr > channel_start + 8 * DM32_PAGE_SIZE) {
                break;
            }
        } else {
            blank_run = 0;
        }
    }
}

static int dm32_slot_is_blank(const channel_raw_t *slot)
{
    const unsigned char *raw = (const unsigned char *)slot;
    unsigned ff = 0;
    unsigned zz = 0;
    for (unsigned i = 0; i < sizeof(*slot); ++i) {
        if (raw[i] == 0xFF) {
            ++ff;
        }
        if (raw[i] == 0x00) {
            ++zz;
        }
    }
    if (ff == sizeof(*slot) || zz == sizeof(*slot)) {
        return 1;
    }
    char name[LEN_CHANNEL_NAME + 1];
    dm32_copy_ascii(name, sizeof(name), slot->label, sizeof(slot->label));
    return name[0] == '\0';
}

static int dm32_slot_is_digital(const channel_raw_t *slot)
{
    return slot->params[0] == 0x14 &&
           slot->params[1] == 0x00 &&
           slot->params[2] == 0x00 &&
           slot->params[3] == 0x00;
}

static void dm32_print_channels(FILE *out)
{
    fprintf(out, "\nChannels\n");
    fprintf(out, "Idx  %-16s %-9s %-9s Mode Slot CC Power\n", "Name", "RX (MHz)", "TX (MHz)");

    unsigned count = 0;
    for (unsigned idx = 0; idx < MAX_CHANNELS; ++idx) {
        uint32_t offset = channel_offset(idx);
        const channel_raw_t *slot = (const channel_raw_t *)dm32_mem_ptr(offset, sizeof(channel_raw_t));
        if (!slot) {
            break;
        }
        if (dm32_slot_is_blank(slot)) {
            continue;
        }

    char name[LEN_CHANNEL_NAME + 1];
        dm32_copy_ascii(name, sizeof(name), slot->label, sizeof(slot->label));
        if (!name[0]) {
            continue;
        }

        double rx = dm32_bcd_mhz(slot->rx_bcd);
        double tx = dm32_bcd_mhz(slot->tx_bcd);
        int digital = dm32_slot_is_digital(slot);
        int high = (slot->params[0] & DM32_POWER_HIGH_BIT) != 0;
        int slot2 = (slot->params[5] & DM32_TS2_BIT) != 0;
        int cc = slot->params[5] & DM32_CC_MASK;

        char slot_buf[2];
        char cc_buf[4];
        const char *slot_out = digital ? (slot_buf[0] = slot2 ? '2' : '1', slot_buf[1] = '\0', slot_buf) : "-";
        const char *cc_out = digital ? (snprintf(cc_buf, sizeof(cc_buf), "%d", cc), cc_buf) : "-";

        fprintf(out, "%4u  %-16s %9.5f %9.5f  %c    %s   %2s  %s\n",
                idx + 1,
                name,
                rx,
                tx,
                digital ? 'D' : 'A',
                slot_out,
                cc_out,
                high ? "High" : "Low");
        ++count;
    }

    if (count == 0) {
        fprintf(out, "(no populated channel slots in downloaded image)\n");
    }
}

static void dm32_print_string_table(FILE *out, const char *title,
                                    uint32_t base, uint32_t stride,
                                    unsigned max_items, unsigned name_len)
{
    fprintf(out, "\n%s\n", title);
    fprintf(out, "Idx  Name\n");

    unsigned printed = 0;
    for (unsigned idx = 0; idx < max_items; ++idx) {
        uint32_t offset = base + stride * idx;
        const unsigned char *ptr = dm32_mem_ptr(offset, name_len);
        if (!ptr) {
            break;
        }
        char name[64];
        dm32_copy_ascii(name, sizeof(name), ptr, name_len);
        if (!name[0]) {
            continue;
        }
        fprintf(out, "%4u  %s\n", idx + 1, name);
        ++printed;
    }
    if (printed == 0) {
        fprintf(out, "(none)\n");
    }
}

static void dm32_print_messages(FILE *out)
{
    fprintf(out, "\nCanned Messages\n");
    fprintf(out, "Idx  Text\n");

    unsigned printed = 0;
    for (unsigned idx = 0; idx < MAX_MESSAGES; ++idx) {
        uint32_t offset = OFFSET_MESSAGE_TABLE + idx * STRIDE_MESSAGE;
        const unsigned char *ptr = dm32_mem_ptr(offset, LEN_MESSAGE);
        if (!ptr) {
            break;
        }
        char buf[LEN_MESSAGE + 1];
        dm32_copy_ascii(buf, sizeof(buf), ptr, LEN_MESSAGE);
        if (!buf[0]) {
            continue;
        }
        fprintf(out, "%4u  %s\n", idx + 1, buf);
        ++printed;
    }
    if (printed == 0) {
        fprintf(out, "(none)\n");
    }
}

static void dm32_print_ids(FILE *out)
{
    fprintf(out, "\nRadio IDs\n");
    fprintf(out, "Idx  String\n");

    unsigned printed = 0;
    for (unsigned idx = 0; idx < MAX_RADIO_IDS; ++idx) {
        uint32_t offset = OFFSET_RADIO_ID_TABLE + idx * STRIDE_RADIO_ID;
        const unsigned char *ptr = dm32_mem_ptr(offset, STRIDE_RADIO_ID);
        if (!ptr) {
            break;
        }
        char buf[STRIDE_RADIO_ID + 1];
        dm32_copy_ascii(buf, sizeof(buf), ptr, STRIDE_RADIO_ID);
        if (!buf[0]) {
            continue;
        }
        fprintf(out, "%4u  %s\n", idx + 1, buf);
        ++printed;
    }
    if (printed == 0) {
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
    fprintf(out, "\nV-frame Segments (id, base, span, stride)\n");
    for (unsigned i = 0; i < dm32_nvsegs; ++i) {
        uint32_t span = (uint32_t)dm32_vsegs[i].mask_le + 1u;
        fprintf(out, " 0x%02X  base=0x%06X  span=%5u  stride=%3u\n",
                dm32_vsegs[i].id,
                dm32_vsegs[i].base,
                span,
                dm32_vsegs[i].stride_le);
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

    dm32_print_ids(out);
    dm32_print_messages(out);
    dm32_print_string_table(out, "Zones", OFFSET_ZONE_TABLE, STRIDE_ZONE,
                            MAX_ZONES, LEN_ZONE_NAME);
    dm32_print_string_table(out, "Scan Lists", OFFSET_SCANLIST_TABLE, STRIDE_SCANLIST,
                            MAX_SCANLISTS, LEN_SCANLIST_NAME);
    dm32_print_string_table(out, "Roam Zones", OFFSET_ROAM_TABLE, STRIDE_ROAM,
                            MAX_ROAM_ZONES, LEN_ROAM_NAME);
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
    dm32_prime_reads();

    fprintf(stderr, "DM32: reading memory pages...\n");
    dm32_download_plan();
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
