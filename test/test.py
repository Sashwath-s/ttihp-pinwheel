# SPDX-License-Identifier: Apache-2.0
# Tiny Tapeout cocotb smoke test for Pinwheel (RTL and gate level).
# Loads sw/fw/demo4.pws through the host SPI port, starts the threads and checks
# UART TX (T0) and an SPI loopback-free transfer (T2, MISO tied high).
# The full regression is test/tb_demo.sv + sw/cosim.py (Vivado xsim).

import os
import sys

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, FallingEdge, Timer

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "sw"))
import pwasm  # noqa: E402

CLK_NS = 20
HOST_HALF_NS = 100
UART_BIT_NS = 1000


def _ns(v):
    """Timer/Clock kwargs for cocotb 1.x (units) and 2.x (unit)."""
    try:
        Timer(1, unit="ns")
        return {"unit": "ns"}
    except TypeError:
        return {"units": "ns"}


class Pins:
    """Shadow of ui_in so individual bits can be driven by several helpers."""

    def __init__(self, dut):
        self.dut = dut
        self.ui = 0b0001_0001          # HOST_CS_N=1, UART RX idle high
        dut.ui_in.value = self.ui

    def set(self, bit, val):
        self.ui = (self.ui & ~(1 << bit)) | (int(val) << bit)
        self.dut.ui_in.value = self.ui

    def uo(self, bit):
        return (int(self.dut.uo_out.value) >> bit) & 1


async def spi_byte(p, tx, kw):
    rx = 0
    for i in range(7, -1, -1):
        p.set(6, (tx >> i) & 1)
        await Timer(HOST_HALF_NS, **kw)
        p.set(5, 1)
        rx = (rx << 1) | p.uo(7)
        await Timer(HOST_HALF_NS, **kw)
        p.set(5, 0)
    return rx


async def host_xfer(p, tx_bytes, kw):
    p.set(4, 0)
    await Timer(HOST_HALF_NS, **kw)
    rx = [await spi_byte(p, b, kw) for b in tx_bytes]
    await Timer(HOST_HALF_NS, **kw)
    p.set(4, 1)
    await Timer(4 * HOST_HALF_NS, **kw)
    return rx


async def host_write(p, addr, words, kw, inc=True):
    body = [0x40 if inc else 0x00, addr >> 8, addr & 0xFF]
    for w in words:
        body += [w >> 8, w & 0xFF]
    await host_xfer(p, body, kw)


async def host_read(p, addr, n, kw, inc=True):
    rx = await host_xfer(p, [0xC0 if inc else 0x80, addr >> 8, addr & 0xFF, n] + [0] * (2 * n), kw)
    d = rx[4:]
    return [(d[2 * i] << 8) | d[2 * i + 1] for i in range(n)]


async def mbox_get(p, k, kw, tries=2000):
    for _ in range(tries):
        st = (await host_xfer(p, [0x80], kw))[0]
        if (st >> (4 + k)) & 1:
            return (await host_read(p, 0x8010 + k, 1, kw, inc=False))[0]
    raise AssertionError(f"mailbox {k} stayed empty")


async def uart_recv(p, kw):
    while p.uo(0) == 1:
        await FallingEdge(p.dut.clk)
    await Timer(UART_BIT_NS // 2, **kw)
    assert p.uo(0) == 0, "UART start bit"
    b = 0
    for i in range(8):
        await Timer(UART_BIT_NS, **kw)
        b |= p.uo(0) << i
    await Timer(UART_BIT_NS, **kw)
    assert p.uo(0) == 1, "UART stop bit"
    return b


@cocotb.test()
async def test_pinwheel(dut):
    kw = _ns(1)
    cocotb.start_soon(Clock(dut.clk, CLK_NS, **kw).start())
    p = Pins(dut)
    p.set(1, 1)                        # SPI MISO tied high
    dut.ena.value = 1
    dut.uio_in.value = 0xFF            # pull-ups
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 10)

    ident = (await host_read(p, 0x8000, 1, kw, inc=False))[0]
    assert ident == 0x5057, f"ID {ident:04x}"

    with open(os.path.join(os.path.dirname(__file__), "..", "sw", "fw", "demo4.pws")) as f:
        prog, _, _ = pwasm.assemble(f.read(), "demo4.pws")
    await host_write(p, 0x0000, prog, kw)
    back = await host_read(p, 0x0000, 16, kw)
    assert back == prog[:16], "program readback"

    await host_write(p, 0x8002, [0x000F], kw, inc=False)      # run T0-T3
    await Timer(20_000, **kw)

    # UART TX through thread 0
    rx = cocotb.start_soon(uart_recv(p, kw))
    await host_write(p, 0x8010, [0x4B], kw, inc=False)
    assert await rx == 0x4B, "UART TX byte"

    # SPI transfer through thread 2 (MISO high -> 0xFF back)
    await host_write(p, 0x8012, [1], kw, inc=False)
    await host_write(p, 0x8012, [0x3C], kw, inc=False)
    assert await mbox_get(p, 2, kw) == 0xFF, "SPI MISO byte"

    st = (await host_read(p, 0x8002, 1, kw, inc=False))[0]
    assert (st >> 12) & 0xF == 0, f"illegal instruction flags {st:04x}"
