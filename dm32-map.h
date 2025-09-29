/*
 * LEGACY: Static address map fragments for Baofeng DM-32 (experimental)
 *
 * This legacy table mirrored OEM CPS page reads and was used to guide
 * initial reverse-engineering. The project now prefers the dynamic
 * V-frame partition map (ids 0x06,0x07,0x08,0x09,0x0A,0x0E,0x0F), which
 * encodes base, mask (segment size), and stride (record size).
 *
 * This header is kept for reference only and is no longer included
 * by the active code paths.
 */
{ 0x00600C, 0x1000 },            // Channel section (this first page): 0x00601C–0x00700B (85 slots), Continues in next 4 KiB page: 0x005001–0x005FF0 (another 85 slots) [DM32_OFFSET_CHAN_PAGE_A]
{ 0x008000, 0x1000 },            // extend channel/label window safely (fills 0x008000..) [DM32_OFFSET_CHAN_PAGE_D]
{ 0x009000, 0x1000 },            // extend further for possible analog labels
{ 0x005001, 0x1000 },            // 85 slots of channel data continued from 0x00600C page [DM32_OFFSET_CHAN_PAGE_B]
{ 0x007001, 0x1000 },            // channel data continuation [DM32_OFFSET_CHAN_PAGE_C]
{ 0x003007, 0x1000 },
{ 0x002007, 0x1000 },
{ 0x00A002, 0x1000 },
{ 0x00D00A, 0x1000 },            // contains printable strings; likely UI/labels [DM32_OFFSET_UI_LABELS_DA]
{ 0x000002, 0x1000 },
{ 0x000020, 0x1000 },
{ 0x000104, 0x1000 },
{ 0x00020A, 0x1000 },
{ 0x00D000, 0x1000 },            // contains "Roam CH 1" string; roam-related? [DM32_OFFSET_UI_LABELS_D0]
// -- Expanded coverage around 0x00D0xx (RX Group / Scan List vicinity) --
{ 0x00D004, 0x1000 },            // adjacent to 0x00D000; additional labels
{ 0x00D006, 0x1000 },            // observed neighbors in CPS-style paging
{ 0x00D00C, 0x1000 },            // conservative extension within same 4K window
{ 0x000904, 0x1000 },
{ 0x000F03, 0x1000 },
{ 0x000300, 0x1000 },
{ 0x000103, 0x1000 },
{ 0x000802, 0x1000 },
{ 0x000408, 0x1000 },
{ 0x005007, 0x1000 },
{ 0x000800, 0x1000 },
{ 0x000A00, 0x1000 },
{ 0x000B00, 0x1000 },
// -- Additional label windows corroborated by OEM export --
{ 0x00A000, 0x1000 },            // adjacent to existing 0x00A002; safer page base
{ 0x00B000, 0x1000 },            // talkgroups
{ 0x00A004, 0x1000 },
{ 0x00A008, 0x1000 },
{ 0x00A00C, 0x1000 },
{ 0x00B004, 0x1000 },
{ 0x00B008, 0x1000 },
{ 0x00B00C, 0x1000 },
{ 0x000001, 0x1000 },           // zones
{ 0x000301, 0x1000 },
{ 0x00E007, 0x1000 },
// -- Expanded coverage around 0x00Exxx (neighboring UI/label pages) --
{ 0x00E000, 0x1000 },            // base page near 0x00E007
{ 0x00E004, 0x1000 },            // additional small page to capture nearby tables
{ 0x00E00A, 0x1000 },            // mirrors 0x00D00A-style offset on 0x00E0xx
{ 0x00E00C, 0x1000 },
{ 0x00E00E, 0x1000 },
{ 0x000202, 0x1000 },
{ 0x000208, 0x1000 },
{ 0x000906, 0x1000 },
{ 0x000A06, 0x1000 },
{ 0x000101, 0x1000 },
{ 0x000401, 0x1000 },
{ 0x000700, 0x1000 },
{ 0x000500, 0x1000 },
{ 0x00D008, 0x1000 },
{ 0x00D00E, 0x1000 },
{ 0x000F00, 0x1000 },
{ 0x000C00, 0x1000 },
{ 0x000B06, 0x1000 },
{ 0x000801, 0x1000 },
{ 0x00D001, 0x1000 },
{ 0x000900, 0x1000 },

// -- Additional CPS-captured 4 KiB pages (align to observed reads) --
{ 0x001001, 0x1000 },
{ 0x001003, 0x1000 },
{ 0x001004, 0x1000 },
{ 0x002000, 0x1000 },
{ 0x002002, 0x1000 },
{ 0x002008, 0x1000 },
{ 0x00200A, 0x1000 },
{ 0x003000, 0x1000 },
{ 0x003001, 0x1000 },
{ 0x004001, 0x1000 },
{ 0x004008, 0x1000 },
{ 0x005000, 0x1000 },           // some canned messages here [DM32_OFFSET_MESSAGES]
{ 0x007000, 0x1000 },           // contacts at 0x007200 [DM32_OFFSET_CONTACTS_PAGE]
{ 0x008001, 0x1000 },
{ 0x008002, 0x1000 },
{ 0x009004, 0x1000 },           // DMR IDs for this radio [DM32_OFFSET_RADIO_ID_A]
{ 0x009006, 0x1000 },           // [DM32_OFFSET_RADIO_ID_B]
{ 0x00A006, 0x1000 },
{ 0x00B006, 0x1000 },           // Scan lists [DM32_OFFSET_SCANLISTS]
{ 0x00C000, 0x1000 },           // Some emergency channel settings here ex (_emergency.csv), encryption at 00C300 [DM32_OFFSET_EMERGENCY]
// Observed CPS 4 KiB read at 0x0C0000 (emergency settings). Note this expands the image high-water mark.
{ 0x0C0000, 0x1000 },
{ 0x00F000, 0x1000 },           // RX group lits [DM32_OFFSET_RXGROUPS]
{ 0x00F003, 0x1000 },
{ 0x008027, 0x1000 },            // contains "Contacts 1" string; contacts index/table? [DM32_OFFSET_CONTACTS_STR]
{ 0x01F0FF, 0x1000 },
{ 0x008027, 0x0004 },            // short probe [DM32_OFFSET_CONTACTS_STR]
// Explicit zone base page observed in CPS sequence
{ 0x000000, 0x1000 },            // [DM32_OFFSET_ZONE_BASE]
