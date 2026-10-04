Debug Serial Output Reference
============================

Enable: DIP switch 1 ON (GPIO27 to GND). Connect terminal to CDC COM port.

Output is best-effort. If the terminal stops reading or the USB debug FIFO
fills, excess text is dropped so disk transfers can continue. Partial or
missing log lines do not indicate a failed disk transfer.

Tiers:
  [I] = Info    — normal operations
  [W] = Warning — recoverable issue, read retried
  [E] = Error   — command failed

Lines:

  [I] TUR: no disk
      Drive empty. Host will retry.

  [I] TUR: media changed
      Disk just inserted/removed. Host re-enumerates.

  [I] READ lba=N n=M
      Host wants M blocks starting at LBA N.
      LBA 0 = boot sector, 1-18 = FAT, 19-32 = root dir.

  [W] READ fail lba=N pr=P pll=PPP hdr=C/H/S
      Sector read failed after the decoder timeout and worker retries. Fields:
        pr    Decoder progress (how far it got):
              0 = no progress at all
              1 = sync mark (0x4489) found
              2 = valid ID CRC passed on some sector
              3 = target sector matched (correct C:H:S)
              4 = data address mark (0xFB) found
              5 = complete success (not shown on fail)
        pll   Final PLL value. Nominal ~200 ticks (2us at 500kbps).
              Values far from 200 suggest drive speed issues.
        hdr   Last decoded sector header: cylinder/head/sector.
              "0/0/0" means no valid header was decoded.

  [E] sector I/O timeout
      Core 1 did not respond within 5 seconds.
      USB reports an error while retaining the sector buffer until core 1 exits.

  [W] SCSI unk op=HH
      Unrecognised SCSI command (hex). Some hosts probe for features.
