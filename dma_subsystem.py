"""
dma_subsystem.py
-----------------
Behavioral model of the DMA subsystem algorithms:
    1. DMADescriptor / DescriptorManager -> linked-list descriptor chain
    2. DMAStateMachine                   -> the FSM controlling a transfer
    3. PacketBufferManager               -> circular (ring) buffer for
                                             packets moving to the MAC
    4. InterruptController               -> priority-based IRQ handling

Maps directly to objectives #6, #7, #8 in the project brief.
"""

from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Optional

from axi_interconnect import AXI4Interconnect, AXITransaction, BurstType


# ---------------------------------------------------------------------
# 1. DMA DESCRIPTOR + DESCRIPTOR MANAGER
# ---------------------------------------------------------------------
@dataclass
class DMADescriptor:
    """One entry of a descriptor ring, similar to real Ethernet DMA IP."""
    desc_id: int
    src_addr: int
    dst_addr: int
    length: int          # bytes to transfer
    next_desc: Optional[int] = None   # index of next descriptor, or None
    owned_by_dma: bool = True         # ownership bit (like real MAC DMA)


class DescriptorManager:
    """
    Manages a linked list (ring) of descriptors, exactly like the
    "DMA Descriptor Manager" block in the project's Verilog module list.
    """

    def __init__(self):
        self.descriptors: dict[int, DMADescriptor] = {}
        self.head: Optional[int] = None
        self.current: Optional[int] = None

    def add_descriptor(self, desc: DMADescriptor):
        self.descriptors[desc.desc_id] = desc
        if self.head is None:
            self.head = desc.desc_id
            self.current = desc.desc_id

    def fetch_next(self) -> Optional[DMADescriptor]:
        if self.current is None:
            return None
        desc = self.descriptors[self.current]
        self.current = desc.next_desc
        return desc

    def has_more(self) -> bool:
        return self.current is not None


# ---------------------------------------------------------------------
# 2. DMA STATE MACHINE
# ---------------------------------------------------------------------
class DMAState(Enum):
    IDLE = auto()
    FETCH_DESCRIPTOR = auto()
    READ_MEMORY = auto()
    WRITE_TO_MAC = auto()
    RAISE_INTERRUPT = auto()
    DONE = auto()


class DMAStateMachine:
    """
    Implements the "DMA State Machine" module. Each call to step()
    advances the FSM by one clock cycle -- run it in a loop to see the
    full transfer sequence, exactly what you'd see on a waveform.

    Reads now go through the real AXI4Interconnect as a single BURST
    transaction (one address phase + burst_len data beats) instead of
    one word per arbitration round -- this matches how a real DMA
    engine uses the AXI bus and shows up clearly in the AXI trace.
    """

    def __init__(self, descriptor_mgr: DescriptorManager,
                 interconnect: AXI4Interconnect, master_id: int,
                 packet_buffer: "PacketBufferManager",
                 irq_ctrl: "InterruptController", burst_beat_bytes: int = 4):
        self.state = DMAState.IDLE
        self.desc_mgr = descriptor_mgr
        self.interconnect = interconnect
        self.master_id = master_id
        self.packet_buffer = packet_buffer
        self.irq_ctrl = irq_ctrl
        self.burst_beat_bytes = burst_beat_bytes
        self.active_desc: Optional[DMADescriptor] = None
        self.read_data: list[int] = []
        self.write_index = 0
        self.cycle = 0
        self.trace: list[tuple[int, str]] = []

    def _log(self, msg: str):
        self.trace.append((self.cycle, msg))

    def step(self):
        self.cycle += 1

        if self.state == DMAState.IDLE:
            if self.desc_mgr.has_more():
                self.state = DMAState.FETCH_DESCRIPTOR
                self._log("IDLE -> FETCH_DESCRIPTOR (new descriptor pending)")
            else:
                self._log("IDLE (no descriptors pending)")

        elif self.state == DMAState.FETCH_DESCRIPTOR:
            self.active_desc = self.desc_mgr.fetch_next()
            self.state = DMAState.READ_MEMORY
            self._log(f"FETCH_DESCRIPTOR -> desc#{self.active_desc.desc_id} "
                       f"({self.active_desc.length} bytes)")

        elif self.state == DMAState.READ_MEMORY:
            burst_len = max(1, self.active_desc.length // self.burst_beat_bytes)
            read_txn = AXITransaction(
                master_id=self.master_id, txn_type="READ",
                address=self.active_desc.src_addr,
                burst_len=burst_len, burst_size=self.burst_beat_bytes,
                burst_type=BurstType.INCR,
            )
            self.interconnect.submit([read_txn])
            self.read_data = list(read_txn.read_data)
            self.write_index = 0
            self._log(f"READ_MEMORY  <- AXI INCR burst @0x{self.active_desc.src_addr:08X} "
                       f"len={burst_len} beats ({self.active_desc.length}B) "
                       f"resp={read_txn.resp}")
            self.state = DMAState.WRITE_TO_MAC

        elif self.state == DMAState.WRITE_TO_MAC:
            if self.write_index < len(self.read_data):
                ok = self.packet_buffer.push(self.read_data[self.write_index])
                if ok:
                    self._log(f"WRITE_TO_MAC -> buffer beat "
                               f"{self.write_index + 1}/{len(self.read_data)} OK")
                    self.write_index += 1
                else:
                    self._log(f"WRITE_TO_MAC -> buffer FULL, stalling beat "
                               f"{self.write_index + 1}/{len(self.read_data)}")
            if self.write_index >= len(self.read_data):
                self.state = DMAState.RAISE_INTERRUPT

        elif self.state == DMAState.RAISE_INTERRUPT:
            self.irq_ctrl.raise_irq("DMA_TRANSFER_COMPLETE", priority=2)
            self._log(f"RAISE_INTERRUPT -> desc#{self.active_desc.desc_id} complete")
            self.state = DMAState.DONE

        elif self.state == DMAState.DONE:
            if self.desc_mgr.has_more():
                self.state = DMAState.FETCH_DESCRIPTOR
                self._log("DONE -> FETCH_DESCRIPTOR (next descriptor)")
            else:
                self.state = DMAState.IDLE
                self._log("DONE -> IDLE (all descriptors processed)")

    def run_until_idle(self, max_cycles: int = 2000):
        while not (self.state == DMAState.IDLE and not self.desc_mgr.has_more()):
            self.step()
            if self.cycle > max_cycles:
                raise RuntimeError("DMA state machine did not terminate")
        self.step()  # final IDLE log


# ---------------------------------------------------------------------
# 3. PACKET BUFFER MANAGER (circular / ring buffer)
# ---------------------------------------------------------------------
class PacketBufferManager:
    """
    Circular buffer between the DMA engine and the Ethernet MAC,
    mirroring the "Packet Buffer Manager" + "Packet Buffer Interface"
    modules. Detects overflow (backpressure) exactly like a real FIFO
    with almost-full flags.
    """

    def __init__(self, depth: int):
        self.depth = depth
        self.buffer: list[int] = []
        self.overflow_count = 0
        self.drained_count = 0

    def push(self, item) -> bool:
        if len(self.buffer) >= self.depth:
            self.overflow_count += 1
            return False
        self.buffer.append(item)
        return True

    def pop(self):
        if not self.buffer:
            return None
        self.drained_count += 1
        return self.buffer.pop(0)

    def occupancy(self) -> int:
        return len(self.buffer)

    def almost_full(self, threshold_pct: float = 0.8) -> bool:
        return len(self.buffer) >= self.depth * threshold_pct


# ---------------------------------------------------------------------
# 4. INTERRUPT CONTROLLER (priority-based)
# ---------------------------------------------------------------------
@dataclass(order=True)
class IRQEvent:
    priority: int
    name: str = field(compare=False)
    cycle: int = field(compare=False, default=0)


class InterruptController:
    """
    Priority interrupt controller: multiple sources (DMA complete,
    buffer overflow, AXI error) can request service; the controller
    resolves which one the VEGA processor sees first, using a min-heap
    priority encoder (lower number = higher priority), the standard
    algorithm for hardware priority interrupt controllers.
    """

    def __init__(self):
        import heapq
        self._heapq = heapq
        self.pending: list[IRQEvent] = []
        self.serviced: list[IRQEvent] = []

    def raise_irq(self, name: str, priority: int, cycle: int = 0):
        self._heapq.heappush(self.pending, IRQEvent(priority, name, cycle))

    def service_next(self) -> Optional[IRQEvent]:
        if not self.pending:
            return None
        ev = self._heapq.heappop(self.pending)
        self.serviced.append(ev)
        return ev

    def has_pending(self) -> bool:
        return bool(self.pending)


if __name__ == "__main__":
    # Self-test: transfer two descriptors' worth of "packets" through a
    # real AXI4Interconnect using burst reads.
    addr_map = {
        "ETH_REGS": (0x4000_0000, 0x1000),
        "DMA_REGS": (0x4001_0000, 0x1000),
        "MEMORY":   (0x8000_0000, 0x1000_0000),
    }
    ic = AXI4Interconnect(addr_map, num_masters=2)
    ic.slave_memory["MEMORY"][0x8000_0000] = 0xDEADBEEF
    ic.slave_memory["MEMORY"][0x8000_0004] = 0xC0FFEE00

    dm = DescriptorManager()
    dm.add_descriptor(DMADescriptor(0, 0x8000_0000, 0x9000_0000, length=4, next_desc=1))
    dm.add_descriptor(DMADescriptor(1, 0x8000_0004, 0x9000_0004, length=4, next_desc=None))

    pbuf = PacketBufferManager(depth=8)
    irq = InterruptController()
    fsm = DMAStateMachine(dm, ic, master_id=1, packet_buffer=pbuf, irq_ctrl=irq)
    fsm.run_until_idle()

    for cyc, msg in fsm.trace:
        print(f"[cycle {cyc:>3}] {msg}")

    print("\nPending interrupts serviced by VEGA CPU:")
    while irq.has_pending():
        ev = irq.service_next()
        print(f"  -> {ev.name} (priority {ev.priority})")

    print(f"\nPacket buffer occupancy at end: {pbuf.occupancy()}")
    print(f"Overflow events: {pbuf.overflow_count}")