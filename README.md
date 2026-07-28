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
| DIP 4   | 14   | IN (track write) |
| LED     | 16   | OUT (WS2812) |

## DIP Switch Configuration

| DIP | Function | OFF | ON |
|-----|----------|-----|-----|
| 1 | Debug serial | Disabled | CDC ACM debug output |
| 2 | Write enable | Read-only | Writes allowed |
| 3 | Operation mode | MSC (mass storage) | GreaseWeasel |
| 4 | Track write | — | Read-modify-write entire track |

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

**⚠ Currently broken — writing via MSC corrupts the diskette.**

The `flux_writer` PIO output is not producing valid MFM-encoded flux. Track writes result in noise/unreadable data on disk. Root cause under investigation.

```
Host USB SCSI WRITE(10)
  → tud_msc_write10_cb()           [Core 0]
    → shared_sector_buffer          [cross-core]
    → core1_floppy_worker()         [Core 1]
      → read all 18 sectors of target track into tw_buf[18][512]
      → update target sector in buffer
      → format_track_with_data_encode() → MFM half-cell bitstream
      → write_track_raw()
        → DMA → flux_writer PIO → WDATA pin → drive write head
      → write_track_raw() returns → WGATE off
```

Writes are committed immediately (no deferred caching). SYNC CACHE and START/STOP UNIT SCSI commands trigger a dirty-track flush.

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
