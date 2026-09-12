"""
ethernet_mac_regs.py
---------------------
Behavioral model of the "Ethernet Register Interface" / "Control Register
Block" / "Status Monitor" modules -- the memory-mapped register file the
VEGA processor uses to configure and monitor the Gigabit Ethernet MAC IP
(which, per the brief, is developed by another team -- you only model the
register-level interaction here).
"""

from enum import IntFlag


class MACStatus(IntFlag):
    LINK_UP = 1 << 0
    TX_BUSY = 1 << 1
    RX_READY = 1 << 2
    TX_COMPLETE = 1 << 3


class EthernetMACRegisters:
    """
    A tiny memory-mapped register file:
        0x00 CTRL    - control register (bit0: enable TX)
        0x04 STATUS  - status register (read-only, see MACStatus)
        0x08 TX_LEN  - length of packet loaded for transmission
        0x0C IRQ_EN  - interrupt enable mask

    Timing model
    ------------
    A real Gigabit MAC does NOT transmit a frame in zero time. Every
    frame on the wire (GMII, 1 byte per clock at 125 MHz = 1 Gbps) costs:

        7  bytes  Preamble
        1  byte   SFD (Start Frame Delimiter)
        N  bytes  Payload (your TX_LEN)
        4  bytes  FCS (CRC-32)
        12 bytes  IFG (Inter-Frame Gap, mandatory idle before next frame)

    TX_BUSY stays asserted for that many byte-cycles instead of clearing
    instantly, and no new frame can start until the IFG has elapsed --
    this is what actually caps Ethernet's *effective* throughput below
    the raw 1000 Mbps line rate, and is worth mentioning to your guide.
    """

    CTRL, STATUS, TX_LEN, IRQ_EN = 0x00, 0x04, 0x08, 0x0C

    PREAMBLE_BYTES = 7
    SFD_BYTES = 1
    FCS_BYTES = 4
    IFG_BYTES = 12  # 96 bit-times at the given link rate

    def __init__(self, link_speed_mbps: int = 1000):
        self.regs = {self.CTRL: 0, self.STATUS: MACStatus.LINK_UP, 0x08: 0, 0x0C: 0}
        self.link_speed_mbps = link_speed_mbps
        self.tx_log = []          # completed frame lengths (payload bytes)
        self.tx_cycles_remaining = 0
        self.frames_transmitted = 0
        self.total_wire_bytes = 0  # preamble+SFD+payload+FCS+IFG, all frames

    def write(self, offset: int, value: int):
        self.regs[offset] = value
        if offset == self.CTRL and value & 0x1 and not self.busy():
            self._start_transmit()

    def read(self, offset: int) -> int:
        return int(self.regs.get(offset, 0))

    def busy(self) -> bool:
        return bool(self.regs[self.STATUS] & MACStatus.TX_BUSY)

    def _start_transmit(self):
        payload_len = self.regs.get(self.TX_LEN, 0)
        frame_bytes = (self.PREAMBLE_BYTES + self.SFD_BYTES +
                       payload_len + self.FCS_BYTES + self.IFG_BYTES)
        # At 1 byte transferred per MAC clock cycle (GMII @ 125MHz = 1Gbps
        # data path), transmission of the whole frame+IFG takes this many
        # MAC-domain cycles.
        self.tx_cycles_remaining = frame_bytes
        self._pending_payload_len = payload_len
        self.regs[self.STATUS] |= MACStatus.TX_BUSY
        self.regs[self.STATUS] &= ~MACStatus.TX_COMPLETE

    def tick(self):
        """Advance the MAC by one clock cycle. Call this every cycle
        from the top-level simulation loop, same as toggling a real
        clock in a testbench."""
        if self.tx_cycles_remaining > 0:
            self.tx_cycles_remaining -= 1
            if self.tx_cycles_remaining == 0:
                self.regs[self.STATUS] &= ~MACStatus.TX_BUSY
                self.regs[self.STATUS] |= MACStatus.TX_COMPLETE
                self.tx_log.append(self._pending_payload_len)
                self.frames_transmitted += 1
                self.total_wire_bytes += (
                    self.PREAMBLE_BYTES + self.SFD_BYTES +
                    self._pending_payload_len + self.FCS_BYTES + self.IFG_BYTES
                )