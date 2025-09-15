/*
 * Experimental interface to Baofeng DM-32 over CH340 serial.
 *
 * Minimal implementation to enter program mode and read a block,
 * based on captured CPS protocol (PSEARCH/PASSSTA/SYSINFO, V/G, PROGRAM, R/W).
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include "radio.h"
#include "util.h"

// -----------------------------------------------------------------------------
// DM-32 constants (advertised/spec + mapped offsets)
// -----------------------------------------------------------------------------
// Advertised capacities (dm32_reference/dm32_specs.md)
#define DM32_NCHAN          4000       // Memory Channels (advertised)
#define DM32_NCONTACTS      50000      // Digital contacts (advertised max)
#define DM32_NZONES         250        // Zones (advertised)
#define DM32_NGLISTS        32         // RX Group Lists (OEM CPS)
#define DM32_NSCANLISTS     32         // Scan lists (advertised)
#define DM32_NMESSAGES      20         // Canned messages (OEM CPS)

// Image/memory characteristics
#define DM32_MEMSZ          0x200000   // 2 MiB safe bound used by reader

// Channel slot layout window (observed)
#define DM32_CHAN_BASE      0x00601C   // First slot label address
#define DM32_CHAN_STRIDE    0x30       // 48 bytes per slot
#define DM32_CHAN_WINDOW    240        // Extend parsing to ~11.5 KiB window (fits reads at 0x00600C,0x007001,0x008000)

// Label → pad → signature seeking
#define DM32_LABEL_PAD_MAX  16         // Skip up to 16 pad bytes (0xFF/0x00) after label
#define DM32_SIG_SCAN_MAX   32         // Scan ahead up to 32 bytes for signature

// Parameter block relative to signature start (s)
#define DM32_PARAMS_OFS     8          // Params start at s+8
#define DM32_PARAMS_LEN     16         // 16 bytes of parameters
#define DM32_PARAM_IDX_POWER  0        // params[0]
#define DM32_PARAM_IDX_TSCC   5        // params[5]
#define DM32_PARAM_IDX_MON    7        // params[7]

// Bit masks within parameters
#define DM32_POWER_HIGH_BIT 0x04       // params[0] bit for High power
#define DM32_TS2_BIT        0x10       // params[5] bit for Timeslot 2
#define DM32_CC_MASK        0x0F       // params[5] low nibble for Color Code

// Serial characteristics
#define DM32_BAUD           115200

// Simple helpers and minimal protocol implementation
static unsigned dm32_written_max = 0;

typedef struct { uint32_t addr; uint16_t len; } dm32_block_t;
// Common entry types used across collectors
typedef struct { uint32_t off; char name[80]; } dm32_chan_t;
typedef struct { uint32_t off; char name[32]; } dm32_zone_t;

// -----------------------------------------------------------------------------
// DM-32 channel slot structures (patterned after other drivers)
// -----------------------------------------------------------------------------
// Data block starting at the post-label signature 's'.
// Offsets below are relative to 's'.
typedef struct {
    // Bytes 0-3: RX Frequency (8-digit BCD, little-endian)
    uint32_t rx_bcd;
    // Bytes 4-7: TX Frequency (8-digit BCD, little-endian)
    uint32_t tx_bcd;
    // Bytes 8-23: Parameters blob (16 bytes)
    uint8_t  params[DM32_PARAMS_LEN];
} dm32_sig_block_t;

// Parsed channel view for convenient consumption.
typedef struct {
    uint32_t offset;                 // Slot base address (label start)
    uint32_t sig_offset;             // Signature start address 's'
    char     name[32];               // NUL-terminated label
    double   rx_mhz;                 // Decoded MHz from BCD
    double   tx_mhz;                 // Decoded MHz from BCD
    uint8_t  timeslot;               // 1 or 2
    uint8_t  color_code;             // 0..15
    uint8_t  power_high;             // 1=High, 0=Low
    uint8_t  monitor_flag;           // 1 if monitor/special (from params[7] bit0), else 0
    uint8_t  params[DM32_PARAMS_LEN];// Raw params for diagnostics
} dm32_channel_t;

// Forward decls
static void dm32_write_slots_debug_csv(void);
static void dm32_write_channels_fields_csv(void);
static int dm32_parse_slot(uint32_t base, dm32_channel_t *out);
// Helpers used by parsers (defined later in file)
static double bcd_mhz(const unsigned char *p);
static double f32_mhz(const unsigned char *p);
static double bcd_mhz_alt(const unsigned char *p);
static double decode_freq_mhz(const unsigned char *p, double rx_hint);

// Addresses observed in CPS capture; used to guide reading and mapping.
// Kept in a separate header to ease collaborative reverse-engineering.
static const dm32_block_t dm32_blocks[] = {
#include "dm32-map.h"
};
static const unsigned dm32_nblocks = sizeof(dm32_blocks)/sizeof(dm32_blocks[0]);

static void dm32_dump_reads(int msec)
{
    unsigned char buf[512];
    int total = 0;
    int iters = msec / 50;
    if (iters < 1) iters = 1;
    for (int i = 0; i < iters; ++i) {
        int n = serial_read(buf, sizeof(buf), 50);
        if (n > 0) {
            total += n;
            if (trace_flag) {
                fprintf(stderr, "DM32: recv %d bytes\n", n);
                print_hex(buf, n);
            }
        }
    }
    if (trace_flag && total == 0) {
        fprintf(stderr, "DM32: idle (%d ms)\n", msec);
    }
}

// Drain and return total bytes observed over the next msec, without logging.
#if defined(__GNUC__)
__attribute__((unused))
#endif
static int dm32_drain_collect(int msec)
{
    unsigned char buf[256];
    int total = 0;
    int iters = msec / 50;
    if (iters < 1) iters = 1;
    for (int i = 0; i < iters; ++i) {
        int n = serial_read(buf, sizeof(buf), 50);
        if (n > 0) total += n;
    }
    return total;
}

// Return number of bytes observed over the next msec, draining input.
// (no longer used) activity helper

static void dm32_send_ascii(const char *s)
{
    if (trace_flag)
        fprintf(stderr, "DM32: send '%s'\n", s);
    (void) serial_write((const unsigned char*)s, (int)strlen(s));
}

// Read exactly n bytes (or 0 on timeout). Returns bytes read.
static int dm32_read_exact(unsigned char *buf, int n, int timeout_msec)
{
    int got = 0;
    while (got < n) {
        int r = serial_read(buf + got, n - got, timeout_msec);
        if (r <= 0) break;
        got += r;
    }
    return got;
}

// Read and synchronize to a DM32 reply header.
// Expects header starting with 0x57 ('W'), followed by 3-byte addr and 2-byte len.
// Discards spurious bytes like 0xFF fill or 0x06 ACK before the header.
// Returns 0 on success, -1 on timeout/error.
static int dm32_read_header_sync(unsigned char hdr[6], int timeout_msec)
{
    unsigned char b;
    int waited = 0;
    unsigned skip = 0;
    // Try to find 0x57 start within the timeout window.
    while (waited < timeout_msec) {
        int r = dm32_read_exact(&b, 1, 150);
        if (r <= 0) {
            waited += 200;
            continue;
        }
        if (b == 0x57) {
            hdr[0] = b;
            // Read the rest of the header bytes.
            if (dm32_read_exact(hdr + 1, 5, 5000) != 5)
                return -1;
            return 0;
        }
        // Ignore all non-header bytes; cap very high to tolerate long SYSINFO/0x56 bursts.
        if (++skip > 100000) return -1;
    }
    return -1;
}

// DM-32/Anytone-like block read: 0x52 + 24-bit addr (big-endian) + 16-bit len (little-endian)
static int dm32_read_block(uint32_t addr24, uint16_t len)
{
    unsigned char cmd[6];
    unsigned char hdr[6];

    // Bounds check to avoid overruns
    if ((uint64_t)addr24 + (uint64_t)len > (uint64_t)(1024*1024*2)) {
        if (trace_flag) fprintf(stderr, "DM32: skip out-of-range read %06X len %u\n", addr24, (unsigned)len);
        return -1;
    }

    // Build request
    cmd[0] = 0x52; // 'R'
    cmd[1] = (addr24 >> 16) & 0xFF;
    cmd[2] = (addr24 >> 8) & 0xFF;
    cmd[3] = addr24 & 0xFF;
    cmd[4] = len & 0xFF;        // little-endian length
    cmd[5] = (len >> 8) & 0xFF;
    if (trace_flag) {
        fprintf(stderr, "DM32: R %02X %02X %02X %02X %02X\n", cmd[1], cmd[2], cmd[3], cmd[4], cmd[5]);
    }
    if (serial_write(cmd, 6) < 0)
        return -1;

    // Read response header: 0x57 'W' + same addr (3) + len (2)
    if (dm32_read_header_sync(hdr, 4000) != 0)
        return -1;
    if (hdr[0] != 0x57 || hdr[1] != cmd[1] || hdr[2] != cmd[2] || hdr[3] != cmd[3] || hdr[4] != cmd[4] || hdr[5] != cmd[5]) {
        if (trace_flag) {
            fprintf(stderr, "DM32: Unexpected W header\n");
            print_hex(hdr, 6);
        }
        return -1;
    }

    // Read payload
    unsigned toread = len;
    unsigned off = 0;
    while (toread > 0) {
        unsigned char buf[512];
        int chunk = (toread > sizeof(buf)) ? sizeof(buf) : toread;
        int r = dm32_read_exact(buf, chunk, 2000);
        if (r <= 0) {
            if (trace_flag) fprintf(stderr, "DM32: payload timeout after %u bytes\n", off);
            return -1;
        }
        memcpy(radio_mem + addr24 + off, buf, r);
        off += r;
        toread -= r;
        if ((addr24 + off) > dm32_written_max)
            dm32_written_max = addr24 + off;
        // Update progress (best-effort, capped to 100%).
        if (radio_progress < 100)
            radio_progress = (int)((dm32_written_max / (double)(1024*1024)) * 100.0);
    }
    if (trace_flag) {
        fprintf(stderr, "DM32: read %u bytes at %06X\n", (unsigned)len, addr24);
    }
    return 0;
}

static int dm32_read_block_retry(uint32_t addr24, uint16_t len, int attempts)
{
    for (int i = 0; i < attempts; ++i) {
        if (dm32_read_block(addr24, len) == 0)
            return 0;
        usleep(50000);
    }
    return -1;
}
static void dm32_print_version(radio_device_t *radio, FILE *out)
{
    fprintf(out, "Baofeng DM-32 (experimental)\n");
}

static void dm32_download(radio_device_t *radio)
{
    // Ensure port is open at 115200 without triggering generic identify.
    if (serial_open_found(DM32_BAUD) < 0) {
        fprintf(stderr, "DM32: failed to open serial port at 115200\n");
        return;
    }
    // 0) Nudge the cable/radio lines
    (void)serial_pulse_rts_dtr();
    usleep(150000);

    // 1) Initial ASCII handshake (observe but ignore content)
    dm32_send_ascii("PSEARCH");
    dm32_dump_reads(150);

    dm32_send_ascii("PASSSTA");
    dm32_dump_reads(150);

    dm32_send_ascii("SYSINFO");
    dm32_dump_reads(150);

    // 2) Version/info probes (CPS-like)
    unsigned char v0[5] = {0x56, 0x00, 0x00, 0x40, 0x0D};
    (void)serial_write(v0, 5);
    dm32_dump_reads(100);
    for (int i = 1; i <= 16; ++i) {
        if (i == 12) continue; // 0x0C not observed
        unsigned char vv[5] = {0x56, 0x00, 0x00, 0x00, (unsigned char)i};
        (void)serial_write(vv, 5);
        dm32_dump_reads(90);
    }

    // 3) Resource fetch (ignored)
    unsigned char g[6] = {0x47, 0x00, 0x00, 0x00, 0x00, 0x01};
    (void)serial_write(g, 6);
    dm32_dump_reads(200);

    // 4) Enter PROGRAM mode
    static const unsigned char prog_preamble[] = {0xFF,0xFF,0xFF,0xFF,0x0C,'P','R','O','G','R','A','M'};
    (void)serial_write(prog_preamble, sizeof(prog_preamble));
    usleep(30000);
    static const unsigned char b02[] = {0x02};
    static const unsigned char b06[] = {0x06};
    (void)serial_write(b02, sizeof(b02));
    dm32_dump_reads(80);
    (void)serial_write(b06, sizeof(b06));
    dm32_dump_reads(120);

    // 5) Reads: small probe then mapped blocks
    (void) dm32_read_block_retry(0x008027, 4, 2);
    dm32_dump_reads(50);

    for (unsigned i = 0; i < dm32_nblocks; ++i) {
        if (trace_flag) fprintf(stderr, "DM32: read block %u/%u at %06X len %u\n", i+1, dm32_nblocks, dm32_blocks[i].addr, dm32_blocks[i].len);
        if (dm32_read_block_retry(dm32_blocks[i].addr, dm32_blocks[i].len, 2) != 0) {
            fprintf(stderr, "DM32: failed to read block at %06X len %u\n", dm32_blocks[i].addr, dm32_blocks[i].len);
        }
    }

    // Emit slot-level debug CSV for reverse-engineering
    dm32_write_slots_debug_csv();
    // Emit parsed fields CSV (rx/tx + timeslot)
    dm32_write_channels_fields_csv();
}

static void dm32_upload(radio_device_t *radio, int cont_flag)
{
    fprintf(stderr, "DM32 upload not implemented.\n");
}

static int dm32_is_compatible(radio_device_t *radio)
{
    // No image handling yet; always allow configuration-only.
    return 1;
}

static void dm32_read_image(radio_device_t *radio, FILE *img)
{
    fprintf(stderr, "DM32 image read not implemented.\n");
}

// Local ASCII classification helpers used by heuristics below
// (helpers defined above)

static void dm32_save_image(radio_device_t *radio, FILE *img)
{
    unsigned n = dm32_written_max;
    if (n == 0) n = 1; // write at least one byte to create the file
    fwrite(radio_mem, 1, n, img);
}

// Local ASCII classification helpers
static int is_ascii_print(unsigned char c){ return c >= 32 && c <= 126; }
static int is_upper(char c){ return c>='A'&&c<='Z'; }
static int is_lower(char c){ return c>='a'&&c<='z'; }
static int is_digit(char c){ return c>='0'&&c<='9'; }
static int is_space(char c){ return c==' '||c=='-'; }
// Allow additional punctuation commonly seen in channel labels
// removed: is_chan_punct (heuristic)
// removed: is_alnum, istarts_with, rstrip_spaces (heuristics)
// removed: trim_trailing_artifacts (heuristic)
// removed: is_disallowed_label (heuristic)

// Helper to register a channel label if it passes heuristics and isn't a duplicate.
// ... keep helper after looks_like_channel ...

/* duplicate block removed */

static int looks_like_zone(const char *s, int len)
{
    if (len < 3 || len > 24) return 0;
    if (!is_upper(s[0])) return 0;
    int lowers = 0, uppers = 0;
    for (int i=0;i<len;i++) {
        char c=s[i];
        if (!(is_upper(c)||is_lower(c)||is_digit(c)||is_space(c))) return 0;
        if (is_lower(c)) lowers++;
        if (is_upper(c)) uppers++;
    }
    // Prefer proper nouns: at least one lowercase, not shouting (few uppers)
    if (lowers == 0) return 0;
    if (uppers > len/2+1) return 0;
    return 1;
}

// Heuristic: channel names in DM-32 images (observed) tend to live near 0x006000..0x006FFF
// and look like human-readable labels with spaces, mixed case, length 5..26.
// removed: looks_like_channel (heuristic)

// Register a channel if it appears valid under heuristics.
// removed: add_channel_if_valid (heuristic)

// Slot-friendly insertion: accept simpler labels (no strict looks_like_channel),
// still filter disallowed and zone-name collisions.
// removed: add_channel_from_slot (heuristic)

// Check a known post-label slot signature at position s
static int is_slot_signature(uint32_t s, uint32_t end)
{
    if (s+8 >= end) return 0;
    const unsigned char *m = &radio_mem[s];
    // Pattern A: 50 87 ?? 44 50 87 ?? 44 (observed on some dumps)
    if (m[0]==0x50 && m[3]==0x44 && m[4]==0x50 && m[7]==0x44) return 1;
    // Pattern B: 25 ?? 44 [00]? 25 ?? 44 (alt framing)
    if (m[0]==0x25 && m[2]==0x44) {
        unsigned idx = 3; if (m[3]==0x00) idx = 4; if (s+idx+2 < end && m[idx]==0x25 && m[idx+2]==0x44) return 1;
    }
    // Generic: two 4-byte words decode to plausible frequencies (BCD or float)
    double rx_b = bcd_mhz(&radio_mem[s + 0]);
    double tx_b = bcd_mhz(&radio_mem[s + 4]);
    if (rx_b > 30.0 && rx_b < 1000.0 && tx_b >= 0.0 && tx_b < 1000.0) return 1;
    return 0;
}

// Structured channel slot parser: base 0x00601C, stride 0x30, read ASCII until NUL,
// allow 0-2 0xFF pad, then require known signature bytes. No hardcoded prefixes.
static unsigned dm32_extract_channels(dm32_chan_t *chans, unsigned maxc)
{
    const uint32_t base = DM32_CHAN_BASE;
    const uint32_t stride = DM32_CHAN_STRIDE;
    const uint32_t limit = (dm32_written_max ? dm32_written_max : 0x008000);
    unsigned count = 0;
    for (uint32_t p = base; p + 1 < limit && p < 0x010000; p += stride) {
        dm32_channel_t ch;
        if (dm32_parse_slot(p, &ch) != 0) continue;
        if (count < maxc) {
            chans[count].off = p;
            strncpy(chans[count].name, ch.name, sizeof(chans[count].name)-1);
            chans[count].name[sizeof(chans[count].name)-1]=0;
            count++;
        }
    }
    return count;
}

// Locate signature start 's' after the label for a slot base address.
static int dm32_locate_signature(uint32_t p, uint32_t *sig_out, char *name_buf, size_t name_sz)
{
    const uint32_t limit = (dm32_written_max ? dm32_written_max : 0x008000);
    if (p + 1 >= limit) return -1;
    // Extract label
    uint32_t q = p; unsigned k=0;
    while (q < limit && is_ascii_print(radio_mem[q]) && k < name_sz-1) { name_buf[k++] = radio_mem[q++]; }
    name_buf[k] = 0;
    if (k == 0) return -1;
    if (!(q < limit && radio_mem[q] == 0x00)) return -1;
    q++;
    // Skip padding (both 0xFF and 0x00) after label terminator.
    // Stock codeplug often has multiple 0x00 bytes here.
    for (int pad=0; pad<DM32_LABEL_PAD_MAX && q<limit; ++pad) {
        if (radio_mem[q] == 0xFF || radio_mem[q] == 0x00) { q++; continue; }
        break;
    }
    // Scan forward and choose the best-aligned signature candidate by score
    int best_score = -1; uint32_t best_sig = 0; int best_param_ok = 0;
    for (int scan=0; scan<DM32_SIG_SCAN_MAX; ++scan) {
        uint32_t base_sig = q + (uint32_t)scan;
        if (base_sig + 12 >= limit) break;
        for (int k = 0; k <= 3; ++k) {
            uint32_t sig = base_sig + (uint32_t)k;
            if (sig + 12 >= limit) break;
            int score = 0;
            int param_ok = 0;
            // Strong match if byte-pattern signatures agree
            if (is_slot_signature(sig, limit)) score += 3;
            // Evaluate BCD plausibility
            double rx_b = bcd_mhz(&radio_mem[sig + 0]);
            double tx_b = bcd_mhz(&radio_mem[sig + 4]);
            int bcd_ok = (rx_b > 30.0 && rx_b < 1000.0 && tx_b >= 0.0 && tx_b < 1000.0);
            if (bcd_ok) {
                score += 5;
                double diff = tx_b - rx_b; if (diff < 0) diff = -diff;
                if (diff < 0.001) score += 2;                    // equal RX/TX
                if ((diff > 4.999 && diff < 5.001) || (diff > 0.599 && diff < 0.601)) score += 2; // common offsets
                // Ham band bonus: closer to 144/430 MHz (not strict, just bias)
                double band_bonus = 0.0;
                double d144 = rx_b > 144.0 ? rx_b - 144.0 : 144.0 - rx_b;
                double d430 = rx_b > 430.0 ? rx_b - 430.0 : 430.0 - rx_b;
                if (d144 < 20.0 || d430 < 20.0) band_bonus = 1.0;
                score += (int)band_bonus;
            }
            // Parameter pattern plausibility at sig+8
            uint32_t pb = sig + DM32_PARAMS_OFS;
            if (pb + 12 < limit) {
                uint8_t p0 = radio_mem[pb + 0], p1 = radio_mem[pb + 1], p2 = radio_mem[pb + 2], p3 = radio_mem[pb + 3];
                uint8_t p4 = radio_mem[pb + 4], p5 = radio_mem[pb + 5];
                // Observed patterns:
                // Digital-like starts with 14 00 00 00 and p4 in {0x30,0x34}, p5==0x01
                if (p0 == 0x14 && p1 == 0x00 && p2 == 0x00 && p3 == 0x00 && (p4 == 0x30 || p4 == 0x34) && p5 == 0x01) {
                    score += 6; param_ok = 1;
                }
                // Analog-like starts with 04 80 00 00 and p4==0x30, p5==0x01
                if (p0 == 0x04 && p1 == 0x80 && p2 == 0x00 && p3 == 0x00 && p4 == 0x30 && p5 == 0x01) {
                    score += 5; param_ok = 1;
                }
                // 0xFF padding later in params is common
                if (radio_mem[pb + 10] == 0xFF && radio_mem[pb + 11] == 0xFF && radio_mem[pb + 12] == 0xFF && radio_mem[pb + 13] == 0xFF)
                    score += 2;
            }
            // Evaluate float plausibility (lower weight)
            double rx_f = f32_mhz(&radio_mem[sig + 0]);
            double tx_f = f32_mhz(&radio_mem[sig + 4]);
            if (rx_f > 30.0 && rx_f < 1000.0 && tx_f >= 0.0 && tx_f < 1000.0) score += 1;

            // Also evaluate an alternative alignment at sig+4 (some images show 4-byte pad)
            int score2 = 0; int param_ok2 = 0;
            uint32_t sig2 = sig + 4;
            if (sig2 + 12 < limit) {
                if (is_slot_signature(sig2, limit)) score2 += 2; // lighter weight
                double rx_b2 = bcd_mhz(&radio_mem[sig2 + 0]);
                double tx_b2 = bcd_mhz(&radio_mem[sig2 + 4]);
                int bcd_ok2 = (rx_b2 > 30.0 && rx_b2 < 1000.0 && tx_b2 >= 0.0 && tx_b2 < 1000.0);
                if (bcd_ok2) {
                    score2 += 6;
                    double diff2 = tx_b2 - rx_b2; if (diff2 < 0) diff2 = -diff2;
                    if (diff2 < 0.001) score2 += 2;
                    if ((diff2 > 4.999 && diff2 < 5.001) || (diff2 > 0.599 && diff2 < 0.601)) score2 += 2;
                }
                uint32_t pb2 = sig2 + DM32_PARAMS_OFS;
                if (pb2 + 12 < limit) {
                    uint8_t q0 = radio_mem[pb2 + 0], q1 = radio_mem[pb2 + 1], q2 = radio_mem[pb2 + 2], q3 = radio_mem[pb2 + 3];
                    uint8_t q4 = radio_mem[pb2 + 4], q5 = radio_mem[pb2 + 5];
                    if (q0 == 0x14 && q1 == 0x00 && q2 == 0x00 && q3 == 0x00 && (q4 == 0x30 || q4 == 0x34) && q5 == 0x01) {
                        score2 += 7; param_ok2 = 1; // prefer when params align cleanly at +8
                    }
                    if (q0 == 0x04 && q1 == 0x80 && q2 == 0x00 && q3 == 0x00 && q4 == 0x30 && q5 == 0x01) {
                        score2 += 5; param_ok2 = 1;
                    }
                }
            }

            if (score2 > score) { score = score2; sig = sig2; param_ok = param_ok2; }

            if (score > best_score) { best_score = score; best_sig = sig; best_param_ok = param_ok; }
        }
    }
    if (best_score >= 9 || (best_score >= 6 && best_param_ok)) { *sig_out = best_sig; return 0; }
    return -1;
}

// Parse a slot into a normalized dm32_channel_t. Returns 0 on success, -1 if invalid.
static int dm32_parse_slot(uint32_t base, dm32_channel_t *out)
{
    uint32_t s = 0;
    char name[32];
    if (dm32_locate_signature(base, &s, name, sizeof(name)) != 0) return -1;
    if (s + DM32_PARAMS_OFS + DM32_PARAMS_LEN > dm32_written_max) return -1;
    out->offset = base;
    out->sig_offset = s;
    strncpy(out->name, name, sizeof(out->name)-1);
    out->name[sizeof(out->name)-1] = 0;
    out->rx_mhz = decode_freq_mhz(&radio_mem[s + 0], 0.0);
    // Determine slot layout: if the 4 bytes at s+4 look like a params header, treat TX at s+8 and params at s+4.
    int tx_ofs = 4;
    int params_ofs = 8;
    if (s + 12 <= dm32_written_max) {
        uint8_t h0 = radio_mem[s + 4], h1 = radio_mem[s + 5], h2 = radio_mem[s + 6], h3 = radio_mem[s + 7];
        if ((h0 == 0x14 && h1 == 0x00 && h2 == 0x00 && h3 == 0x00) ||
            (h0 == 0x04 && h1 == 0x80 && h2 == 0x00 && h3 == 0x00)) {
            tx_ofs = 8;
            params_ofs = 4;
        }
    }
    out->tx_mhz = decode_freq_mhz(&radio_mem[s + tx_ofs], out->rx_mhz);
    // If TX looks implausible relative to RX (e.g., 300.100 on UHF/VHF), prefer simplex assumption
    if (out->rx_mhz >= 100.0 && out->rx_mhz <= 1000.0) {
        double diff = out->tx_mhz - out->rx_mhz; if (diff < 0) diff = -diff;
        if (out->tx_mhz < 100.0 || out->tx_mhz > 1000.0 || diff > 10.0) {
            out->tx_mhz = out->rx_mhz;
        }
    }
    uint8_t p0 = radio_mem[s + params_ofs + 0];
    uint8_t p1 = radio_mem[s + params_ofs + 1];
    uint8_t p2 = radio_mem[s + params_ofs + 2];
    uint8_t p3 = radio_mem[s + params_ofs + 3];
    uint8_t p5 = radio_mem[s + params_ofs + 5];
    uint8_t p7 = radio_mem[s + params_ofs + 7];
    // Infer analog vs digital by leading param pattern
    int is_digital = (p0 == 0x14 && p1 == 0x00 && p2 == 0x00 && p3 == 0x00 && p5 == 0x01);
    int is_analog  = (p0 == 0x04 && p1 == 0x80 && p2 == 0x00 && p3 == 0x00);
    // Power high bit appears in p0 bit2 for both modes
    out->power_high = (p0 & DM32_POWER_HIGH_BIT) ? 1 : 0;
    if (is_digital) {
        out->timeslot = (p5 & DM32_TS2_BIT) ? 2 : 1;
        out->color_code = (p5 & DM32_CC_MASK);
    } else if (is_analog) {
        // Analog: no meaningful timeslot/cc
        out->timeslot = 1;
        out->color_code = 0;
    } else {
        // Fallback to old indices if pattern unknown
    uint8_t pwr = radio_mem[s + params_ofs + DM32_PARAM_IDX_POWER];
    uint8_t tscc = radio_mem[s + params_ofs + DM32_PARAM_IDX_TSCC];
    uint8_t mon = radio_mem[s + params_ofs + DM32_PARAM_IDX_MON];
        out->power_high = (pwr & DM32_POWER_HIGH_BIT) ? 1 : 0;
        out->timeslot = (tscc & DM32_TS2_BIT) ? 2 : 1;
        out->color_code = (tscc & DM32_CC_MASK);
        out->monitor_flag = (mon & 0x01) ? 1 : 0;
    }
    out->monitor_flag = (p7 & 0x01) ? 1 : 0;
    // Copy params
    memcpy(out->params, &radio_mem[s + params_ofs], DM32_PARAMS_LEN);
    return 0;
}

// Write a debug CSV of raw 0x30-byte channel slots to help reverse engineering
// Helper: decode 4-byte little-endian BCD frequency where nibbles (from MSB to LSB)
// represent the decimal digits of frequency in 10 Hz units, e.g. 0x44358750 -> 443.58750 MHz
static double bcd_mhz(const unsigned char *p)
{
    int digits[8];
    unsigned idx = 0;
    for (int bi = 3; bi >= 0; --bi) {
        unsigned char byte = p[bi];
        digits[idx++] = (byte >> 4) & 0xF;
        digits[idx++] = byte & 0xF;
    }
    for (int i = 0; i < 8; ++i) if (digits[i] > 9) return 0.0;
    uint32_t val = 0;
    for (int i = 0; i < 8; ++i) {
        val = val * 10 + (uint32_t)digits[i];
    }
    return ((double)val) / 100000.0;
}

// Helper: read little-endian float32 as MHz
static double f32_mhz(const unsigned char *p)
{
    union { uint32_t u; float f; } u;
    u.u = (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
    double v = (double)u.f;
    if (v < 0.0 || v > 2000.0) return 0.0;
    return v;
}

// Alternative BCD decode: read bytes in forward order (little->big)
static double bcd_mhz_alt(const unsigned char *p)
{
    int digits[8];
    unsigned idx = 0;
    for (int bi = 0; bi < 4; ++bi) {
        unsigned char byte = p[bi];
        digits[idx++] = (byte >> 4) & 0xF;
        digits[idx++] = byte & 0xF;
    }
    for (int i = 0; i < 8; ++i) if (digits[i] > 9) return 0.0;
    uint32_t val = 0;
    for (int i = 0; i < 8; ++i) val = val * 10 + (uint32_t)digits[i];
    double v = ((double)val) / 100000.0;
    if (v < 0.0 || v > 2000.0) return 0.0;
    return v;
}

// Robust frequency decode with fallbacks and sanity using rx_hint when decoding TX
static double dm32_band_score(double v)
{
    double bands[] = {144.0, 145.0, 146.0, 430.0, 433.0, 435.0, 438.0, 439.0, 440.0};
    double best = 0.0;
    for (unsigned i=0;i<sizeof(bands)/sizeof(bands[0]);++i) {
        double d = v - bands[i]; if (d<0) d = -d;
        double s = (d < 2.0) ? (2.0 - d) : 0.0; // within 2 MHz
        if (s > best) best = s;
    }
    // Step alignment: multiples of 0.0125 MHz
    double steps = v / 0.0125;
    double nearest = (double)((long long)(steps + (steps>=0?0.5:-0.5)));
    double frac = steps - nearest; if (frac < 0) frac = -frac;
    if (frac < 0.02) best += 0.5;
    return best;
}

static double decode_freq_mhz(const unsigned char *p, double rx_hint)
{
    double v1 = bcd_mhz(p);
    double v2 = bcd_mhz_alt(p);
    int ok1 = (v1 >= 30.0 && v1 <= 1000.0);
    int ok2 = (v2 >= 30.0 && v2 <= 1000.0);
    if (ok1 && !ok2) return v1;
    if (!ok1 && ok2) return v2;
    if (ok1 && ok2) {
        double s1 = dm32_band_score(v1), s2 = dm32_band_score(v2);
        return (s2 > s1 ? v2 : v1);
    }
    // As a last resort for TX, if rx_hint is plausible, assume simplex
    if (rx_hint >= 30.0 && rx_hint <= 1000.0) return rx_hint;
    return 0.0;
}

static void dm32_write_slots_debug_csv(void)
{
    const uint32_t base = DM32_CHAN_BASE;
    const uint32_t stride = DM32_CHAN_STRIDE;
    const uint32_t end = DM32_CHAN_BASE + DM32_CHAN_STRIDE * DM32_CHAN_WINDOW; // window
    FILE *f = fopen("dm32_slots_debug.csv", "w");
    if (!f) return;
    fprintf(f, "slot,offset_hex,label,rx_bcd_mhz,tx_bcd_mhz,rx_f32_mhz,tx_f32_mhz,bytes_hex,params_hex16,sig_hex32\n");
    for (unsigned i=0;i<128;i++) {
        uint32_t p = base + i*stride;
        if (p+1 >= dm32_written_max || p >= end) break;
        // Extract label
        char lbl[64]; unsigned k=0; uint32_t q=p;
        while (q < dm32_written_max && is_ascii_print(radio_mem[q]) && k < sizeof(lbl)-1) { lbl[k++] = radio_mem[q++]; }
        lbl[k]=0;
        if (!(q < dm32_written_max && radio_mem[q]==0x00)) continue; // no label/invalid slot
    uint32_t s = q+1;
    // Skip up to DM32_LABEL_PAD_MAX FF padding
    for (int pad=0; pad<DM32_LABEL_PAD_MAX && s<dm32_written_max && radio_mem[s]==0xFF; ++pad) s++;
        // Some slots contain extra ASCII metadata between label and signature.
        // Seek forward up to 16 bytes to the first matching signature start.
        uint32_t sig = s;
    for (int scan=0; scan<DM32_SIG_SCAN_MAX && sig < dm32_written_max; ++scan) {
            if (is_slot_signature(sig, dm32_written_max)) { s = sig; break; }
            sig++;
        }
        // Gather the remaining bytes in this 0x30 window starting from label start
        unsigned slot_len = stride;
        if (p + slot_len > dm32_written_max) slot_len = dm32_written_max - p;
        // Decode RX/TX using BCD at signature start (s) and fallback to float32
        double rx_bcd = 0.0, tx_bcd = 0.0, rx_f32 = 0.0, tx_f32 = 0.0;
        if (s + 8 <= dm32_written_max) {
            rx_bcd = bcd_mhz(&radio_mem[s + 0]);
            tx_bcd = bcd_mhz(&radio_mem[s + 4]);
            rx_f32 = f32_mhz(&radio_mem[s + 0]);
            tx_f32 = f32_mhz(&radio_mem[s + 4]);
        }

        // bytes hex dump
        fprintf(f, "%u,%06X,%s,%.5f,%.5f,%.5f,%.5f,", i, p, lbl, rx_bcd, tx_bcd, rx_f32, tx_f32);
        for (unsigned b=0; b<slot_len; ++b) {
            fprintf(f, "%02X", radio_mem[p+b]);
            if (b+1<slot_len) fputc(' ', f);
        }
        // Also dump params after the two frequency words (to aid mapping)
        fputc(',', f);
        if (s + DM32_PARAMS_OFS < dm32_written_max) {
            unsigned plen = DM32_PARAMS_LEN;
            if (s + DM32_PARAMS_OFS + plen > dm32_written_max) plen = dm32_written_max - (s + DM32_PARAMS_OFS);
            for (unsigned b = 0; b < plen; ++b) {
                fprintf(f, "%02X", radio_mem[s + DM32_PARAMS_OFS + b]);
                if (b + 1 < plen) fputc(' ', f);
            }
        }
        // Dump 32 bytes starting at signature start
        fputc(',', f);
        if (s < dm32_written_max) {
            unsigned sl = 32;
            if (s + sl > dm32_written_max) sl = dm32_written_max - s;
            for (unsigned b = 0; b < sl; ++b) {
                fprintf(f, "%02X", radio_mem[s + b]);
                if (b + 1 < sl) fputc(' ', f);
            }
        }
        fputc('\n', f);
    }
    fclose(f);
}

// Write a CSV with parsed fields per channel slot: label, RX/TX MHz (BCD), timeslot (1/2), and the 16-byte params blob for diffing
static void dm32_write_channels_fields_csv(void)
{
    const uint32_t base = DM32_CHAN_BASE;
    const uint32_t stride = DM32_CHAN_STRIDE;
    const uint32_t end = DM32_CHAN_BASE + DM32_CHAN_STRIDE * DM32_CHAN_WINDOW; // window
    FILE *f = fopen("dm32_channels_fields.csv", "w");
    if (!f) return;
    fprintf(f, "slot,offset_hex,label,rx_mhz,tx_mhz,timeslot,power,color_code,params_hex16\n");
    for (unsigned i = 0; i < DM32_CHAN_WINDOW; ++i) {
        uint32_t p = base + i * stride;
        if (p + 1 >= dm32_written_max || p >= end) break;
        dm32_channel_t ch;
        if (dm32_parse_slot(p, &ch) != 0) continue;
        const char *power = ch.power_high ? "High" : "Low";
        fprintf(f, "%u,%06X,%s,%.5f,%.5f,%u,%s,%u,", i, p, ch.name, ch.rx_mhz, ch.tx_mhz, (unsigned)ch.timeslot, power, (unsigned)ch.color_code);
        // Dump 16 bytes of params after the two frequency words
        unsigned plen = DM32_PARAMS_LEN;
        for (unsigned b = 0; b < plen; ++b) {
            fprintf(f, "%02X", ch.params[b]);
            if (b + 1 < plen) fputc(' ', f);
        }
        fputc('\n', f);
    }
    fclose(f);
}

// Detect channel labels by a post-label signature observed in DM-32 images:
// label '\0' [optional 0xFF padding] then bytes 50 87 ?? 44 50 87 ?? 44
// removed: scan_channels_signature (heuristic)

static void dm32_print_config(radio_device_t *radio, FILE *out, int verbose)
{
    if (! verbose) return;

    // Match examples convention: print radio name header first.
    fprintf(out, "Radio: %s\n", radio->name);

    fprintf(out, "# DM-32: region map (experimental)\n");
    for (unsigned i = 0; i < dm32_nblocks; ++i) {
        uint32_t a = dm32_blocks[i].addr;
        uint32_t e = a + dm32_blocks[i].len;
        unsigned nonff = 0, non00 = 0;
        unsigned strings = 0;
        char sample1[40] = {0}, sample2[40] = {0};
        // Scan bytes within safe bounds of radio_mem
        if (e > dm32_written_max) e = dm32_written_max;
        uint32_t p = a;
        while (p < e) {
            uint8_t b = radio_mem[p++];
            if (b != 0xff) nonff++;
            if (b != 0x00) non00++;
        }
        // Find up to two sample printable strings
        p = a;
        while (p < e) {
            if (is_ascii_print(radio_mem[p])) {
                char buf[64];
                unsigned k = 0;
                while (p < e && is_ascii_print(radio_mem[p]) && k < sizeof(buf)-1) buf[k++] = radio_mem[p++];
                buf[k] = 0;
                if (k >= 4) {
                    strings++;
                    if (!sample1[0]) strncpy(sample1, buf, sizeof(sample1)-1);
                    else if (!sample2[0]) { strncpy(sample2, buf, sizeof(sample2)-1); }
                }
                // Skip non-printables between strings
                continue;
            }
            p++;
        }
        const char *hint = "";
        if (strstr(sample1, "Contacts") || strstr(sample2, "Contacts")) hint = " (contacts?)";
        else if (strstr(sample1, "Roam") || strstr(sample2, "Roam")) hint = " (roam?)";
        else if (strings > 10 && a >= 0x006000 && a < 0x007000) hint = " (channel/zone labels?)";
        fprintf(out, "0x%06X..0x%06X size=%u nonFF=%u non00=%u strings=%u%s\n",
                a, a + dm32_blocks[i].len - 1, dm32_blocks[i].len, nonff, non00, strings, hint);
        if (sample1[0]) fprintf(out, "  e.g. '%s'\n", sample1);
        if (sample2[0]) fprintf(out, "       '%s'\n", sample2);
    }

    // Before Zones, emit channel tables in examples format using what we know.
    // Parse digital-like slots from the first window.
    const uint32_t ch_base = DM32_CHAN_BASE;
    const uint32_t ch_end  = DM32_CHAN_BASE + DM32_CHAN_STRIDE * DM32_CHAN_WINDOW;
    unsigned printed = 0;
    for (uint32_t p = ch_base; p < ch_end && p < dm32_written_max; p += DM32_CHAN_STRIDE) {
        dm32_channel_t tmp;
        if (dm32_parse_slot(p, &tmp) == 0) { printed = 1; break; }
    }
    if (printed) {
        // Digital channels table.
        fprintf(out, "\n");
        fprintf(out, "# Table of digital channels.\n");
        fprintf(out, "# 1) Channel number: 1-%d\n", DM32_NCHAN);
        fprintf(out, "# 2) Name: up to 16 characters, use '_' instead of space\n");
        fprintf(out, "# 3) Receive frequency in MHz\n");
        fprintf(out, "# 4) Transmit frequency or +/- offset in MHz\n");
        fprintf(out, "# 5) Transmit power: High, Low\n");
        fprintf(out, "# 6) Scan list: - or index in Scanlist table\n");
        fprintf(out, "# 7) Transmit timeout timer in seconds: 0, 15, 30, 45... 555\n");
        fprintf(out, "# 8) Receive only: -, +\n");
        fprintf(out, "# 9) Admit criteria: -, Free, Color\n");
        fprintf(out, "# 10) Color code: 0, 1, 2, 3... 15\n");
        fprintf(out, "# 11) Time slot: 1 or 2\n");
        fprintf(out, "# 12) Receive group list: - or index in Grouplist table\n");
        fprintf(out, "# 13) Contact for transmit: - or index in Contacts table\n");
        fprintf(out, "#\n");
        fprintf(out, "Digital Name             Receive   Transmit Power Scan TOT RO Admit  Color Slot RxGL TxContact\n");

    unsigned idx = 1;
    for (uint32_t p = ch_base; p < ch_end && p < dm32_written_max; p += DM32_CHAN_STRIDE) {
        dm32_channel_t ch;
        if (dm32_parse_slot(p, &ch) != 0) continue;
        // Name to 16 chars, spaces->'_' like examples
        char name16[17]; memset(name16, 0, sizeof(name16));
        for (unsigned k=0; k<16 && ch.name[k]; ++k) name16[k] = (ch.name[k]==' ')?'_':ch.name[k];
        const char *power = ch.power_high ? "High" : "Low";
        double rx = ch.rx_mhz, tx = ch.tx_mhz;
        double diff = tx - rx;
        char txcol[24];
        if (diff > 4.999 && diff < 5.001) strcpy(txcol, "+5");
        else if (diff < -4.999 && diff > -5.001) strcpy(txcol, "-5");
        else if (diff > 0.599 && diff < 0.601) strcpy(txcol, "+0.6");
        else if (diff < -0.599 && diff > -0.601) strcpy(txcol, "-0.6");
        else snprintf(txcol, sizeof(txcol), "%.5f", tx);

        fprintf(out, "%5u   %-16.16s %-8.6g %-8s %-5s %-4s %-3s %-2s %-5u %-4u %-4s %-8s\n",
            idx++, name16, rx, txcol, power, "-", "-", "-",
            (unsigned)ch.color_code, (unsigned)ch.timeslot, "-", "-");
    }

        // Analog channels table (unknown mapping yet) with just header for now.
        fprintf(out, "\n");
        fprintf(out, "# Table of analog channels.\n");
        fprintf(out, "# 1) Channel number: 1-%d\n", DM32_NCHAN);
        fprintf(out, "# 2) Name: up to 16 characters, use '_' instead of space\n");
        fprintf(out, "# 3) Receive frequency in MHz\n");
        fprintf(out, "# 4) Transmit frequency or +/- offset in MHz\n");
        fprintf(out, "# 5) Transmit power: High, Low\n");
        fprintf(out, "# 6) Scan list: - or index\n");
        fprintf(out, "# 7) Transmit timeout timer in seconds: 0, 15, 30, 45... 555\n");
        fprintf(out, "# 8) Receive only: -, +\n");
        fprintf(out, "# 9) Admit criteria: -, Free, Tone\n");
        fprintf(out, "# 10) Squelch level: 0, 1, 2, 3, 4, 5, 6, 7, 8, 9\n");
        fprintf(out, "# 11) Guard tone for receive, or '-' to disable\n");
        fprintf(out, "# 12) Guard tone for transmit, or '-' to disable\n");
        fprintf(out, "# 13) Bandwidth in kHz: 12.5, 20, 25\n");
        fprintf(out, "#\n");
        fprintf(out, "Analog  Name             Receive   Transmit Power Scan TOT RO Admit  Squelch RxTone TxTone Width\n");
        // Rows intentionally omitted until analog mapping is confirmed.
    }

    // Zones: emit examples-style table with sequential numbering and unknown members as '-'.
    const uint32_t ZONES_MAX_ADDR = 0x010000;
    dm32_zone_t zones[128];
    unsigned nz = 0;
    for (unsigned i = 0; i < dm32_nblocks; ++i) {
        uint32_t a = dm32_blocks[i].addr;
        uint32_t e = a + dm32_blocks[i].len;
        if (a >= ZONES_MAX_ADDR) continue;
        if (e > dm32_written_max) e = dm32_written_max;
        uint32_t p = a;
        while (p < e) {
            if (is_ascii_print(radio_mem[p])) {
                char buf[64]; unsigned k=0; uint32_t q=p;
                while (q < e && is_ascii_print(radio_mem[q]) && k < sizeof(buf)-1) buf[k++] = radio_mem[q++];
                buf[k]=0;
                if (looks_like_zone(buf, k)) {
                    // Dedup
                    int dup=0; for (unsigned t=0;t<nz;t++){ if (strncmp(zones[t].name, buf, sizeof(zones[t].name))==0) { dup=1; break; } }
                    if (!dup && nz < 128) { zones[nz].off = p; strncpy(zones[nz].name, buf, sizeof(zones[nz].name)-1); zones[nz].name[sizeof(zones[nz].name)-1]=0; nz++; }
                }
                p = q; continue;
            }
            p++;
        }
    }
    // Filter to the clean, short zone-name table observed at low addresses (<0x002000).
    // This yields concise names like 'Richmond', 'Goochland', etc.
    dm32_zone_t clean[128];
    unsigned nclean = 0;
    for (unsigned i=0; i<nz; ++i) {
        size_t len = strlen(zones[i].name);
        if (zones[i].off < 0x002000 && len > 0 && len <= 16) {
            // Dedup into clean list preserving discovery order
            int dup = 0; for (unsigned t=0; t<nclean; ++t) {
                if (strncmp(clean[t].name, zones[i].name, sizeof(clean[t].name)) == 0) { dup = 1; break; }
            }
            if (!dup && nclean < (sizeof(clean)/sizeof(clean[0]))) {
                clean[nclean] = zones[i];
                nclean++;
            }
        }
    }

    if (nclean) {
        fprintf(out, "\n");
        if (verbose) {
            fprintf(out, "# Table of channel zones.\n");
            fprintf(out, "# 1) Zone number: 1-%d\n", DM32_NZONES);
            fprintf(out, "# 2) Name: up to 16 characters, use '_' instead of space\n");
            fprintf(out, "# 3) List of channels: numbers and ranges (N-M) separated by comma\n");
            fprintf(out, "#\n");
        }
        fprintf(out, "Zone    Name             Channels\n");
        for (unsigned i=0; i<nclean; ++i) {
            // Zone numbers start at 1, names padded to 16 characters max like examples
            fprintf(out, "%4u    %-16.16s -\n", i+1, clean[i].name);
        }
    }

    // Also write a CSV for offline mapping
    FILE *csv = fopen("dm32_zones.csv", "w");
    if (csv) {
        // Focus on the short zone-name table observed at low addresses (<0x002000).
        // This yields clean zone names like 'Richmond', 'Goochland', etc.
        fprintf(csv, "offset_hex,name\n");
        for (unsigned i=0; i<nclean; ++i) fprintf(csv, "%06X,%s\n", clean[i].off, clean[i].name);
        fclose(csv);
    }

    // Structured channel extraction from fixed slots
    dm32_chan_t chans[128];
    unsigned nc = dm32_extract_channels(chans, (unsigned)(sizeof(chans)/sizeof(chans[0])));

    FILE *ccsv = fopen("dm32_channels.csv", "w");
    if (ccsv) {
        fprintf(ccsv, "offset_hex,name\n");
        for (unsigned i=0;i<nc;i++) fprintf(ccsv, "%06X,%s\n", chans[i].off, chans[i].name);
        fclose(ccsv);
    }
}

static int dm32_verify_config(radio_device_t *radio)
{
    // Nothing to verify yet.
    return 1;
}

static void dm32_parse_parameter(radio_device_t *radio, char *param, char *value)
{
    // No-op for now.
}

static int dm32_parse_header(radio_device_t *radio, char *line)
{
    // No tables yet.
    return 0;
}

static int dm32_parse_row(radio_device_t *radio, int table_id, int first_row, char *line)
{
    return 0;
}

static void dm32_update_timestamp(radio_device_t *radio)
{
}

static void dm32_write_csv(radio_device_t *radio, FILE *csv)
{
    // Read latest state from radio (safe, read-only) so we can validate against it.
    // We reuse the download path which is already cautious for DM-32.
    dm32_download(radio);

    // Collect heuristic zones and channels from the image.
    dm32_zone_t zones[256];
    unsigned nz = 0;
    const uint32_t ZONES_MAX_ADDR = 0x010000;
    for (unsigned i = 0; i < dm32_nblocks; ++i) {
        uint32_t a = dm32_blocks[i].addr;
        uint32_t e = a + dm32_blocks[i].len;
        if (a >= ZONES_MAX_ADDR) continue;
        if (e > dm32_written_max) e = dm32_written_max;
        uint32_t p = a;
        while (p < e) {
            if (is_ascii_print(radio_mem[p])) {
                char buf[64]; unsigned k=0; uint32_t q=p;
                while (q < e && is_ascii_print(radio_mem[q]) && k < sizeof(buf)-1) buf[k++] = radio_mem[q++];
                buf[k]=0;
                if (looks_like_zone(buf, k)) {
                    int dup=0; for (unsigned t=0;t<nz;t++){ if (strncmp(zones[t].name, buf, sizeof(zones[t].name))==0) { dup=1; break; } }
                    if (!dup && nz < (sizeof(zones)/sizeof(zones[0]))) { zones[nz].off = p; strncpy(zones[nz].name, buf, sizeof(zones[nz].name)-1); zones[nz].name[sizeof(zones[nz].name)-1]=0; nz++; }
                }
                p = q; continue;
            }
            p++;
        }
    }

    dm32_chan_t chans[128];
    unsigned nc = dm32_extract_channels((dm32_chan_t*)chans, 128);

    // Peek header to decide CSV type.
    char header[1024];
    if (!fgets(header, sizeof(header), csv)) {
        fprintf(stderr, "Empty CSV input.\n");
        return;
    }
    // Normalize header line.
    for (char *p=header; *p; ++p) if (*p=='\r' || *p=='\n') *p='\0';

    int is_zone_csv = (strstr(header, "Zone Name") && strstr(header, "Channel Members")) ? 1 : 0;
    int is_chan_csv = (!is_zone_csv && strstr(header, "Channel Name")) ? 1 : 0;
    if (!is_zone_csv && !is_chan_csv) {
        fprintf(stderr, "Unsupported CSV format for DM-32 validation.\n");
        fprintf(stderr, "Header: %s\n", header);
        return;
    }

    unsigned missing = 0, checked = 0;
    char line[2048];
    if (is_zone_csv) {
        fprintf(stderr, "Validating zones CSV against radio...\n");
        while (fgets(line, sizeof(line), csv)) {
            // Trim EOL
            char *e=line; while (*e) { if (*e=='\r'||*e=='\n') { *e=0; break; } e++; }
            if (!*line) continue;
            // Split first two commas: No.,Zone Name,Channel Members
            char *p1 = strchr(line, ','); if (!p1) continue; *p1++ = 0;
            char *p2 = strchr(p1, ','); if (!p2) continue; *p2++ = 0;
            const char *zone_name = p1;
            checked++;
            int found_zone = 0; for (unsigned zi=0; zi<nz; ++zi) { if (strcmp(zones[zi].name, zone_name)==0) { found_zone=1; break; } }
            if (!found_zone) {
                fprintf(stderr, "Missing zone: %s\n", zone_name);
                missing++;
            }
            // Validate each channel member, pipe-separated in p2
            char *m = p2;
            while (m && *m) {
                char *next = strchr(m, '|');
                if (next) *next++ = 0;
                // Trim spaces around name (rare in these exports)
                while (*m==' '||*m=='\t') m++;
                char *t = m + strlen(m);
                while (t>m && (t[-1]==' '||t[-1]=='\t')) *--t=0;
                if (*m) {
                    int found_ch = 0; for (unsigned ci=0; ci<nc; ++ci) { if (strcmp(chans[ci].name, m)==0) { found_ch=1; break; } }
                    if (!found_ch) {
                        fprintf(stderr, "Missing channel from radio: %s (zone %s)\n", m, zone_name);
                        missing++;
                    }
                }
                m = next;
            }
        }
        fprintf(stderr, "Checked %u zones; radio has %u; channels seen %u.\n", checked, nz, nc);
    } else if (is_chan_csv) {
        fprintf(stderr, "Validating channels CSV against radio...\n");
        while (fgets(line, sizeof(line), csv)) {
            // Trim EOL
            char *e=line; while (*e) { if (*e=='\r'||*e=='\n') { *e=0; break; } e++; }
            if (!*line) continue;
            // Columns: No.,Channel Name,Channel Type,...
            char *p1 = strchr(line, ','); if (!p1) continue; *p1++ = 0;
            char *p2 = strchr(p1, ','); if (!p2) continue; *p2++ = 0;
            const char *chan_name = p1;
            checked++;
            int found_ch = 0; for (unsigned ci=0; ci<nc; ++ci) { if (strcmp(chans[ci].name, chan_name)==0) { found_ch=1; break; } }
            if (!found_ch) {
                fprintf(stderr, "Missing channel: %s\n", chan_name);
                missing++;
            }
        }
        fprintf(stderr, "Checked %u channels from CSV; radio has ~%u channel-like labels.\n", checked, nc);
    }

    if (missing == 0)
        fprintf(stderr, "Validation PASSED.\n");
    else
        fprintf(stderr, "Validation FAILED: %u missing items.\n", missing);
}

radio_device_t radio_dm32 = {
    .name = "Baofeng DM-32 (experimental)",
    .download = dm32_download,
    .upload = dm32_upload,
    .is_compatible = dm32_is_compatible,
    .read_image = dm32_read_image,
    .save_image = dm32_save_image,
    .print_version = dm32_print_version,
    .print_config = dm32_print_config,
    .verify_config = dm32_verify_config,
    .parse_parameter = dm32_parse_parameter,
    .parse_header = dm32_parse_header,
    .parse_row = dm32_parse_row,
    .update_timestamp = dm32_update_timestamp,
    .write_csv = dm32_write_csv,
    .channel_count = 0,
};
