# FloppyController

Dual-core RP2040 firmware that turns a Raspberry Pi Pico into a USB floppy disk controller. Interfaces directly with a 3.5" IBM PC floppy drive over GPIO and presents either a USB Mass Storage device or a GreaseWeasel-compatible raw flux interface.

## Hardware

| Signal | GPIO | Direction |
|--------|------|-----------|
| INDEX  | 0    | IN        |
| TRACK0  | 1    | IN        |
| WP      | 2    | IN        |
| RDATA   | 3    | IN        |
| DSKCHG  | 4    | IN        |
| DRVSB   | 5    | OUT       |
| MOTEA   | 6    | OUT       |
| DIR     | 7    | OUT       |
| STEP    | 8    | OUT       |
| WDATA   | 9    | OUT       |
| WGATE   | 10   | OUT       |
| SIDE1   | 11   | OUT       |
| DENSITY | 12   | OUT       |
| DIP 1   | 27   | IN (debug) |
| DIP 2   | 26   | IN (write enable) |
| DIP 3   | 15   | IN (GW mode) |
| DIP 4   | 14   | IN (reserved) |
| LED     | 16   | OUT (WS2812) |

## DIP Switch Configuration

| DIP | Function | OFF | ON |
|-----|----------|-----|-----|
| 1 | Debug serial | Disabled | CDC ACM debug output |
| 2 | Write enable | Read-only | Writes allowed |
| 3 | Operation mode | MSC (mass storage) | GreaseWeasel |
| 4 | Reserved | — | — |

## Build

Requires Raspberry Pi Pico SDK 2.2.0 and `arm-none-eabi-gcc` 14.2.

```
mkdir build && cd build
cmake ..
make -j4
```

Flash `FloppyController.uf2` to the Pico in BOOTSEL mode.

## Architecture

- **Core 0**: TinyUSB stack (MSC + CDC ACM), SCSI command handling, LED driver, GreaseWeasel protocol
- **Core 1**: Bare-metal floppy I/O — seek, motor control, read/write via PIO state machines, MFM decode/encode

### PIO State Machines

| PIO | SM | Program | Clock | Purpose |
|-----|-----|---------|-------|---------|
| pio0 | flux_sm | `flux_reader.pio` | sysclk/1 (200 MHz) | Capture raw flux transition timings from RDATA |
| pio1 | write_sm | `flux_writer.pio` | sysclk/10 (20 MHz) | Generate WDATA from half-cell MFM bitstream |
| pio1 | gw_write_sm | `gw_writer.pio` | sysclk/4 (50 MHz) | Raw flux write for GreaseWeasel mode |
| pio0 | — | `ws2812.pio` | — | WS2812 RGB LED |

### Write Path

MSC writes update individual sectors on standard IBM 1.44 MB media. The
existing ID field and other sectors are preserved. A sector's write stream
contains 532 encoded bytes (8.512 ms), including the sync, data address mark,
payload, CRC, and two gap bytes. Seeking, rotational waits and verification
add to the host-visible write latency.

```
Host USB SCSI WRITE(10)
  → tud_msc_write10_cb()           [Core 0]
    → shared_sector_buffer          [cross-core]
    → core1_floppy_worker()         [Core 1]
      → encode target sector (MSB-first words, A1 sync = 0x4489)
      → preload DMA → flux_writer PIO
      → locate target C/H/S with valid ID CRC and size code N=2
      → wait through the standard 22-byte GAP2 (352 us)
      → assert WGATE and emit sync/DAM/data/CRC/gap via DMA + PIO
      → wait for PIO completion IRQ, then disable WGATE
      → read back target sector and compare all 512 bytes
```

At 500 kbit/s, each data cell lasts 2 us and each MFM half-cell lasts 1 us.
Both PIO branches take 20 cycles at 20 MHz; a transition pulses WDATA LOW
for 200 ns. Normal flux transition intervals are 2, 3 or 4 us. The reader's
nominal 200-tick PLL value represents the shortest **two-half-cell** interval,
because its counter decrements once per two system-clock cycles.

USB callbacks submit one sector, then return zero while core 1 is busy.
TinyUSB keeps the endpoint buffer and retries, servicing CDC, reset and
control events between callbacks. Writes commit and verify before USB
acknowledges their bytes. SYNCHRONIZE CACHE and START/STOP UNIT reject requests
while physical I/O remains outstanding. Missing/bad IDs, write protection,
DMA underruns, timeouts and readback failures cause an error. Core 0 cannot
reuse an outstanding request's buffer after a timeout or USB reset.

Debug CDC output never waits for buffer space. Excess text is discarded if
the serial host stops reading, so a full debug FIFO cannot stall disk I/O.
MSC uses a fixed 512-byte shared sector buffer and a 1,064-byte encoded write
stream regardless of file size. The 210 KB raw-flux buffer is allocated only
in GreaseWeasel mode; a track buffer is allocated and freed for each FORMAT
track. The Release build's static BSS is 10,508 bytes (previously 246,112).

FORMAT UNIT uses the same DMA/PIO writer for whole tracks, with C2/A1 address
marks and 0xE5 fill. Track length comes from the measured index period, with a
1 ms guard before the next index; unsupported spindle speeds are rejected.
Formatting stops on a write or verification failure.

### Write regression tests

Host tests require a C++17 compiler, Python 3, and `pioasm` from the Pico SDK:

```
cmake -S tests -B build/host-tests -DPIOASM_EXECUTABLE=/path/to/pioasm
cmake --build build/host-tests
ctest --test-dir build/host-tests --output-on-failure
```

The tests check packing order, MFM clock rules, A1/C2 marks, independent CRC
vectors, format bounds, and sector splices through the real decoder, including
neighboring sectors. Writer tests exercise protection, missed alignment,
DMA/PIO failures, readback errors and cleanup. USB tests keep the debug FIFO
full through simulated 150 KiB and 1 MiB transfers, checking callback retries,
buffer ownership, resets, timeouts and protection. A cycle-level model
executes the assembled PIO opcodes to check both branches, word boundaries,
pulse width, underruns and final-bit completion. These checks do not replace
physical-drive validation: copy a file of at least 150 KiB with debug enabled,
capture WDATA/WGATE and confirm 2/3/4 us transition intervals, then write/read/compare
sectors at both ends of the disk and check the surrounding sectors.

### Read Path

```
Host USB SCSI READ(10)
  → tud_msc_read10_cb()             [Core 0]
    → core1_floppy_worker()         [Core 1]
      → flux_reader PIO captures RDATA flux timings
      → Software PLL tracks half-cell period
      → Hunt for 0x4489 sync marks
      → Decode ID and data fields
      → CRC-16/CCITT validation
      → Single-sector read cache
```

## Modes

### MSC Mode (DIP3 OFF)

Appears as a 1.44 MB USB mass storage device (2880 blocks × 512 bytes). Supports:
- Read (10), Write (10)
- Format Unit (full 80-track format)
- Mode Sense, Read Format Capacities
- SYNCHRONIZE CACHE, START/STOP UNIT

### GreaseWeasel Mode (DIP3 ON)

CDC ACM virtual serial port implementing the Adafruit GreaseWeasel protocol. 22 commands: seek, read flux (ticks/RPM/revs modes), write flux, erase flux, parameter get/set, pin control. VID 0x1209 PID 0x4d69. Compatible with FluxEngine and HxC.

## License

MIT
