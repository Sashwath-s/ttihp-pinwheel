<!---
This file is used to generate your project datasheet. Please fill in the information below and delete any unused
sections.

You can also include images in this folder and reference them in the markdown. Each image must be less than
512 kb in size, and the combined size of all images must be less than 1 MB.
-->

## How it works

Pinwheel is a programmable protocol emulator. Instead of hard-wired UART/SPI/I2C blocks it has:

* **A 4-thread barrel processor.** One instruction per thread every 4 clocks in fixed round-robin, so each thread runs at a deterministic 12.5 MIPS at 50 MHz with no hazards. All four threads share one single-port memory (instructions and data). A load or store uses the issuing thread's own next slot, so it never slows the other threads. The ISA is 16-bit with 7 registers per thread, plus instructions for pins, timing (`waitt`), events and port I/O.
* **Four waveform lanes.** Each lane runs a FIFO of commands: shift a data word through a programmable *per-bit phase template* (up to 4 phases; each sets clock and data levels, optionally samples, and ends after a tick or on a clock edge), drive pins, wait, wait for a pin, change the word length, or drop a marker. Timing is exact to one clock cycle, whatever the software is doing. UART, SPI (all modes, master/slave), I2C (open-drain, clock stretching), Manchester, WS2812 and PS/2 are all just templates.
* **A host SPI port** (mode 0, ≤ clk/8) for loading programs, starting/stopping/single-stepping threads, and exchanging data through per-thread mailboxes.

Full architecture: `docs/ARCHITECTURE.md` in the repository.

## How to test

1. Assemble a program: `python sw/pwasm.py sw/fw/demo4.pws -o demo4.hex`.
2. From the RP2040 (or any SPI master) write the words to memory: `CS low, 0x40, 0x00, 0x00, w0_hi, w0_lo, ... CS high`.
3. Start the threads: write `0x000F` to register `0x8002` (`CS low, 0x00, 0x80, 0x02, 0x00, 0x0F, CS high`).
4. With the demo firmware:
   * write a byte to mailbox 0 (`0x8010`) → it comes out as 1 Mbaud UART on `uo[0]`;
   * UART bytes sent into `ui[0]` appear in mailbox 1 (read `0x8011`);
   * write `n, b0..bn-1` to mailbox 2 → SPI mode 0 transfer on `uo[1..3]`/`ui[1]`, received bytes in mailbox 2;
   * write I2C commands to mailbox 3 (`0x01xx` START, `0x02bb` write, `0x03aa` read, `0x04xx` STOP) → I2C on `uio[0]` (SCL) / `uio[1]` (SDA). These need external pull-ups.

Read transactions are `CS low, 0xC0|0x80, addr_hi, addr_lo, N, then 2N bytes`. During the command byte, MISO returns a status byte `{mailbox-has-data[3:0], attention[3:0]}`.

## External hardware

The RP2040 on the TT demo board (or any SPI master) acts as the host. For I2C, add pull-ups on `uio[0]/uio[1]`. For the protocol pins, use any PMOD device or a logic analyzer.
