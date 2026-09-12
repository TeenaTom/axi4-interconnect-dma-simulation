"""
top_simulation.py
-------------------
End-to-end behavioral simulation of the whole subsystem described in the
project brief:

    VEGA Processor -> AXI4 Master Interface -> AXI4 Interconnect
        -> Configuration Bus -> Ethernet MAC Registers
        -> DMA Engine -> AXI Memory Interface -> Gigabit Ethernet MAC

v2 additions over the first version:
    - DMA reads now use real AXI4 INCR BURST transactions (one address
      phase + N back-to-back data beats), not one word per arbitration.
    - The Ethernet MAC has a realistic timing model: transmitting a
      frame costs Preamble+SFD+Payload+FCS+IFG byte-cycles, not zero
      time, and a new frame cannot start while TX_BUSY is set. This is
      what caps *effective* throughput below the 1000 Mbps line rate --
      a good point to make to your guide.

Run:  python3 top_simulation.py
"""

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from axi_interconnect import AXI4Interconnect, AXITransaction, BurstType
from dma_subsystem import (
    DescriptorManager, DMADescriptor, DMAStateMachine,
    PacketBufferManager, InterruptController,
)
from ethernet_mac_regs import EthernetMACRegisters


def build_memory_image(interconnect: AXI4Interconnect, num_packets: int,
                        packet_size_bytes: int):
    """Fill the MEMORY slave with dummy packet payloads at known addresses."""
    mem = interconnect.slave_memory["MEMORY"]
    base = 0x8000_0000
    addr = base
    packet_addrs = []
    words_per_packet = packet_size_bytes // 4
    for p in range(num_packets):
        packet_addrs.append(addr)
        for w in range(words_per_packet):
            mem[addr + w * 4] = 0xA000_0000 + p * 0x100 + w
        addr += packet_size_bytes + 0x100  # gap between packets
    return packet_addrs


def main():
    NUM_PACKETS = 6
    PACKET_SIZE_BYTES = 64      # realistic small Ethernet frame payload
    BURST_BEAT_BYTES = 4        # 32-bit AXI data bus -> 16 beats/packet
    # Buffer must hold at least one full frame's beats (64/4=16) before the
    # MAC can drain it; making it slightly bigger than one frame (here 1.25x)
    # still lets you see real backpressure without deadlocking.
    BUFFER_DEPTH = 20           # packet-buffer depth, in words (beats)
    LINK_SPEED_MBPS = 1000      # Gigabit Ethernet

    # ---- 1. Build the platform ----
    addr_map = {
        "ETH_REGS": (0x4000_0000, 0x1000),
        "DMA_REGS": (0x4001_0000, 0x1000),
        "MEMORY":   (0x8000_0000, 0x1000_0000),
    }
    interconnect = AXI4Interconnect(addr_map, num_masters=2)  # 0=VEGA CPU, 1=DMA
    mac_regs = EthernetMACRegisters(link_speed_mbps=LINK_SPEED_MBPS)

    packet_addrs = build_memory_image(interconnect, NUM_PACKETS, PACKET_SIZE_BYTES)

    desc_mgr = DescriptorManager()
    for i, addr in enumerate(packet_addrs):
        desc_mgr.add_descriptor(
            DMADescriptor(
                desc_id=i,
                src_addr=addr,
                dst_addr=0x4000_0100,          # Ethernet MAC TX FIFO address
                length=PACKET_SIZE_BYTES,
                next_desc=(i + 1 if i + 1 < NUM_PACKETS else None),
            )
        )

    packet_buffer = PacketBufferManager(depth=BUFFER_DEPTH)
    irq_ctrl = InterruptController()
    dma_fsm = DMAStateMachine(desc_mgr, interconnect, master_id=1,
                               packet_buffer=packet_buffer, irq_ctrl=irq_ctrl,
                               burst_beat_bytes=BURST_BEAT_BYTES)

    # ---- 2. VEGA CPU configures the Ethernet MAC over AXI (Configuration Bus) ----
    cfg_txn = AXITransaction(
        master_id=0, txn_type="WRITE",
        address=addr_map["ETH_REGS"][0] + EthernetMACRegisters.IRQ_EN,
        burst_len=1, write_data=[0xF],
    )
    interconnect.submit([cfg_txn])

    # ---- 3. Run the DMA state machine + MAC timing model together ----
    # Each "cycle" here advances BOTH the DMA FSM and the MAC clock, so
    # TX_BUSY / IFG timing genuinely gates when the buffer can drain.
    occupancy_over_time = []
    mac_busy_over_time = []
    cycle_marker = []
    beats_per_frame = PACKET_SIZE_BYTES // BURST_BEAT_BYTES

    while not (dma_fsm.state.name == "IDLE" and not desc_mgr.has_more()):
        dma_fsm.step()
        mac_regs.tick()

        # MAC pulls a full frame's worth of beats out of the buffer only
        # when it is free (not busy transmitting the previous frame) --
        # real backpressure between the DMA-side FIFO and the MAC.
        if not mac_regs.busy() and packet_buffer.occupancy() >= beats_per_frame:
            for _ in range(beats_per_frame):
                packet_buffer.pop()
            mac_regs.write(EthernetMACRegisters.TX_LEN, PACKET_SIZE_BYTES)
            mac_regs.write(EthernetMACRegisters.CTRL, 0x1)

        occupancy_over_time.append(packet_buffer.occupancy())
        mac_busy_over_time.append(1 if mac_regs.busy() else 0)
        cycle_marker.append(dma_fsm.cycle)

        if dma_fsm.cycle > 2000:
            break

    # drain the MAC clock a bit further so the last frame(s) finish transmitting
    tail_cycles = 0
    while (mac_regs.busy() or packet_buffer.occupancy() >= beats_per_frame) and tail_cycles < 200:
        if not mac_regs.busy() and packet_buffer.occupancy() >= beats_per_frame:
            for _ in range(beats_per_frame):
                packet_buffer.pop()
            mac_regs.write(EthernetMACRegisters.TX_LEN, PACKET_SIZE_BYTES)
            mac_regs.write(EthernetMACRegisters.CTRL, 0x1)
        mac_regs.tick()
        dma_fsm.cycle += 1
        occupancy_over_time.append(packet_buffer.occupancy())
        mac_busy_over_time.append(1 if mac_regs.busy() else 0)
        cycle_marker.append(dma_fsm.cycle)
        tail_cycles += 1

    # ---- 4. Print the full transaction / FSM trace ----
    print("=" * 72)
    print("AXI4 CONFIGURATION TRANSACTION")
    print("=" * 72)
    interconnect.print_trace()

    print("\n" + "=" * 72)
    print("DMA STATE MACHINE TRACE (AXI burst reads -> packet buffer)")
    print("=" * 72)
    for cyc, msg in dma_fsm.trace:
        print(f"[cycle {cyc:>4}] {msg}")

    print("\n" + "=" * 72)
    print("INTERRUPTS SERVICED BY VEGA CPU (priority order)")
    print("=" * 72)
    while irq_ctrl.has_pending():
        ev = irq_ctrl.service_next()
        print(f"  -> {ev.name} (priority {ev.priority})")

    # ---- 5. Throughput / latency metrics: raw line rate vs. effective ----
    total_payload_bytes = NUM_PACKETS * PACKET_SIZE_BYTES
    total_cycles = dma_fsm.cycle
    AXI_CLOCK_HZ = 200_000_000     # typical AXI fabric clock

    total_time_s = total_cycles / AXI_CLOCK_HZ
    dma_side_throughput_mbps = (
        (total_payload_bytes * 8 / total_time_s) / 1e6 if total_time_s > 0 else 0
    )

    frames_sent = mac_regs.frames_transmitted
    wire_bytes = mac_regs.total_wire_bytes
    mac_time_s = wire_bytes / (LINK_SPEED_MBPS * 1e6 / 8) if wire_bytes else 0
    effective_throughput_mbps = (
        (sum(mac_regs.tx_log) * 8 / mac_time_s) / 1e6 if mac_time_s > 0 else 0
    )
    overhead_pct = (
        100 * (wire_bytes - sum(mac_regs.tx_log)) / wire_bytes if wire_bytes else 0
    )

    print("\n" + "=" * 72)
    print("PERFORMANCE METRICS")
    print("=" * 72)
    print(f"Packets transferred (DMA)     : {NUM_PACKETS}")
    print(f"Frames transmitted (MAC)      : {frames_sent}")
    print(f"Payload bytes                 : {total_payload_bytes}")
    print(f"DMA/AXI-side cycles           : {total_cycles} @ {AXI_CLOCK_HZ/1e6:.0f} MHz")
    print(f"DMA-side estimated throughput : {dma_side_throughput_mbps:.2f} Mbps "
          f"(raw movement, ignores wire framing)")
    print(f"MAC wire bytes (incl. framing): {wire_bytes} "
          f"(preamble+SFD+FCS+IFG overhead = {overhead_pct:.1f}%)")
    print(f"Effective Ethernet throughput : {effective_throughput_mbps:.2f} Mbps "
          f"out of {LINK_SPEED_MBPS} Mbps line rate")
    print(f"Packet buffer overflow events : {packet_buffer.overflow_count}")

    # ---- 6. Plot buffer occupancy AND MAC busy state over time ----
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 6), sharex=True)

    ax1.step(cycle_marker, occupancy_over_time, where="post", color="#1f6feb")
    ax1.axhline(BUFFER_DEPTH, color="red", linestyle="--", label="Buffer depth (FULL)")
    ax1.set_ylabel("Packet buffer\noccupancy (beats)")
    ax1.set_title("Packet Buffer Occupancy vs. Time (DMA -> Ethernet MAC path)")
    ax1.legend(loc="upper right")

    ax2.step(cycle_marker, mac_busy_over_time, where="post", color="#d1242f")
    ax2.set_ylabel("MAC TX_BUSY")
    ax2.set_xlabel("Simulation cycle")
    ax2.set_yticks([0, 1])
    ax2.set_title("Ethernet MAC Busy (transmitting frame + IFG)")

    plt.tight_layout()
    plt.savefig("buffer_occupancy.png", dpi=150)
    print("\nSaved plot: buffer_occupancy.png")


if __name__ == "__main__":
    main()