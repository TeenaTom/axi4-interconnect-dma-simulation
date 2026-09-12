"""
axi_interconnect.py
--------------------
Behavioral (cycle-accurate-ish) Python model of the AXI4 interconnect
algorithms used in the "VEGA -> AXI4 Interconnect -> DMA -> Gigabit
Ethernet MAC" project.

This is NOT RTL. It is a functional model of the *algorithms* so you can
demonstrate correct behavior (address decode, arbitration, transaction
routing, handshake) in simulation before writing/verifying the Verilog.

Modules implemented here:
    1. AddressDecoder   -> maps an address to a target slave
    2. RoundRobinArbiter-> arbitrates between multiple AXI masters
    3. AXI4Interconnect -> ties decode + arbitration + read/write routing
                            together and logs every transaction (like a
                            waveform trace you can show your guide)
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import Optional


class BurstType(Enum):
    """AXI4 AxBURST encoding (the part relevant to this model)."""
    FIXED = "FIXED"   # address stays the same every beat (e.g. FIFO regs)
    INCR = "INCR"     # address increments by burst_size each beat (memory)


# ---------------------------------------------------------------------
# 1. AXI4 ADDRESS DECODER
# ---------------------------------------------------------------------
class AddressDecoder:
    """
    Maps a 32-bit AXI address to one of the memory-mapped slaves:
        - Configuration Bus / Ethernet MAC registers
        - DMA control registers
        - Main memory (via AXI Memory Interface)

    This mirrors objective #4 in the project brief: "Develop an AXI
    address decoder and transaction routing logic."
    """

    def __init__(self, address_map: dict):
        # address_map: {slave_name: (base_addr, size_in_bytes)}
        self.address_map = address_map

    def decode(self, address: int) -> Optional[str]:
        for slave, (base, size) in self.address_map.items():
            if base <= address < base + size:
                return slave
        return None  # DECERR condition in real AXI


# ---------------------------------------------------------------------
# 2. ROUND-ROBIN ARBITER
# ---------------------------------------------------------------------
class RoundRobinArbiter:
    """
    Arbitrates AXI bus access between multiple masters (VEGA processor,
    DMA engine, and any future master). Implements objective #5:
    "Design arbitration logic for multiple AXI masters."

    Algorithm: classic round-robin with a rotating priority pointer,
    same technique used in real AXI crossbar RTL (e.g. Xilinx AXI
    Interconnect IP).
    """

    def __init__(self, num_masters: int):
        self.num_masters = num_masters
        self.last_granted = -1

    def arbitrate(self, request_vector: list[bool]) -> Optional[int]:
        """
        request_vector[i] = True if master i is requesting the bus.
        Returns the index of the granted master, or None if no requests.
        """
        if not any(request_vector):
            return None

        n = self.num_masters
        for offset in range(1, n + 1):
            candidate = (self.last_granted + offset) % n
            if request_vector[candidate]:
                self.last_granted = candidate
                return candidate
        return None


# ---------------------------------------------------------------------
# 3. AXI4 BURST TRANSACTION MODEL + INTERCONNECT
# ---------------------------------------------------------------------
@dataclass
class AXITransaction:
    """
    Models one AXI4 burst request: a SINGLE address-phase handshake
    (AWVALID/AWREADY or ARVALID/ARREADY) followed by burst_len DATA
    beats -- this is the real AXI4 burst mechanism, not one arbitration
    per word. A DMA moving a 64-byte packet in 16-byte (4-word) beats
    issues ONE burst transaction with burst_len=16, not 16 separate
    transactions.
    """
    master_id: int
    txn_type: str                    # "READ" or "WRITE"
    address: int                     # start address (of the burst)
    burst_len: int = 1               # number of beats (AXI4: 1-256 for INCR)
    burst_size: int = 4              # bytes per beat (e.g. 4 = 32-bit bus)
    burst_type: BurstType = BurstType.INCR
    write_data: Optional[list[int]] = None   # one value per beat, for WRITE
    read_data: list = field(default_factory=list)  # filled in for READ
    slave: Optional[str] = None
    resp: str = "OKAY"               # AXI4 response: OKAY/EXOKAY/SLVERR/DECERR
    cycle_granted: Optional[int] = None   # cycle of the address-phase grant
    cycles_taken: int = 0                 # address phase + burst_len data beats


class AXI4Interconnect:
    """
    Top level: combines address decoding + arbitration + read/write
    channel handshake (VALID/READY abstraction) and produces a
    transaction log you can print/plot as evidence of correct
    behavior -- this is your "simulation to show sir".
    """

    def __init__(self, address_map: dict, num_masters: int):
        self.decoder = AddressDecoder(address_map)
        self.arbiter = RoundRobinArbiter(num_masters)
        self.cycle = 0
        self.log: list[AXITransaction] = []
        # simple memory-mapped register/memory backing store per slave
        self.slave_memory = {name: {} for name in address_map}

    def submit(self, pending: list[AXITransaction]) -> list[AXITransaction]:
        """
        Drain a queue of pending transactions using round-robin
        arbitration. Each master may have several queued transactions;
        one transaction per granted master is serviced per arbitration
        round, then the arbiter re-evaluates (this models real AXI
        behavior where a master holds a request line until served,
        then the next request on that channel goes through the same
        process).
        Returns the transactions in the order they were serviced.
        """
        # group transactions into a FIFO queue per master
        queues: dict[int, list[AXITransaction]] = {}
        for t in pending:
            queues.setdefault(t.master_id, []).append(t)

        serviced = []
        while any(queues.values()):
            request_vector = [
                bool(queues.get(i)) for i in range(self.arbiter.num_masters)
            ]
            granted = self.arbiter.arbitrate(request_vector)
            if granted is None:
                break
            txn = queues[granted].pop(0)
            self._service(txn)
            serviced.append(txn)
        return serviced

    def _service(self, txn: AXITransaction):
        """
        Services one full AXI4 burst: one address-phase grant, then
        burst_len data beats. Each beat is logged individually (like a
        waveform: ARVALID/RVALID pulses back-to-back) so the trace shows
        real burst behavior instead of one word per arbitration.
        """
        slave = self.decoder.decode(txn.address)
        txn.slave = slave
        txn.cycle_granted = self.cycle
        self.cycle += 1  # address phase costs 1 cycle

        if slave is None:
            txn.resp = "DECERR"
            txn.cycles_taken = 1
            self.log.append(("ADDR", txn, 0, txn.address, None))
            return

        mem = self.slave_memory[slave]
        beat_addr = txn.address

        for beat in range(txn.burst_len):
            if txn.txn_type == "WRITE":
                value = txn.write_data[beat] if txn.write_data else 0
                mem[beat_addr] = value
                self.log.append(("BEAT", txn, beat, beat_addr, value))
            else:  # READ
                value = mem.get(beat_addr, 0)
                txn.read_data.append(value)
                self.log.append(("BEAT", txn, beat, beat_addr, value))
            self.cycle += 1  # one data beat per cycle (back-to-back burst)
            if txn.burst_type == BurstType.INCR:
                beat_addr += txn.burst_size

        txn.resp = "OKAY"
        txn.cycles_taken = 1 + txn.burst_len
        self.log.append(("DONE", txn, txn.burst_len - 1, beat_addr, None))

    def print_trace(self):
        print(f"{'CYCLE':>5} {'MASTER':>6} {'TYPE':>5} {'BEAT':>5} "
              f"{'ADDR':>10} {'DATA':>10} {'SLAVE':>10} {'RESP':>7}")
        for phase, t, beat, addr, value in self.log:
            if phase != "BEAT":
                continue
            print(f"{t.cycle_granted:>5} {t.master_id:>6} {t.txn_type:>5} "
                  f"{beat:>3}/{t.burst_len:<2} 0x{addr:08X} {str(value):>10} "
                  f"{str(t.slave):>10} {t.resp:>7}")


if __name__ == "__main__":
    # Quick self-test: two masters (0=VEGA CPU, 1=DMA engine), now with
    # a real 4-beat burst read from memory.
    addr_map = {
        "ETH_REGS": (0x4000_0000, 0x1000),
        "DMA_REGS": (0x4001_0000, 0x1000),
        "MEMORY":   (0x8000_0000, 0x1000_0000),
    }
    ic = AXI4Interconnect(addr_map, num_masters=2)

    txns = [
        AXITransaction(master_id=0, txn_type="WRITE", address=0x4000_0004,
                        burst_len=1, write_data=[0x1]),
        AXITransaction(master_id=1, txn_type="READ", address=0x8000_0100,
                        burst_len=4, burst_size=4),  # 4-beat burst = 16 bytes
        AXITransaction(master_id=0, txn_type="READ", address=0x4001_0000,
                        burst_len=1),
    ]
    ic.submit(txns)
    ic.print_trace()