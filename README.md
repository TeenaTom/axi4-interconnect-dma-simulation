# AXI4 Interconnect + DMA + Gigabit Ethernet Subsystem

## Phase-2 Assignment: Detailed Design, Algorithm Development & High-Level Simulation

**Team:** Basil Lesly (MDL23EVLSI017), Catherene Tessa (MDL23EVLSI018),  
Christo Augustine (MDL23EVLSI019), Teena Tom (MDL23EVLSI061)

**Model Engineering College, Dept. of Electronics Engineering**

---

## What's in this folder

| File | Purpose |
|---|---|
| `axi_interconnect.py` | AXI4 address decoder, round-robin arbiter, and INCR burst transaction model |
| `dma_subsystem.py` | DMA descriptor manager, DMA state machine with DECERR fault handling, packet buffer FIFO, and priority interrupt controller |
| `ethernet_mac_regs.py` | Ethernet MAC register model with framing and timing including Preamble, SFD, FCS, and IFG |
| `top_simulation.py` | Top-level integration that connects the modules, runs the simulation scenario, prints the transaction trace, calculates performance metrics, and generates the plot |
| `simulation_log.txt` | Captured console output from the simulation, including transaction traces and performance metrics |
| `buffer_occupancy.png` | Plot showing packet-buffer occupancy and MAC busy state over simulated time |

---

## Software Required

- Python 3.9+
- Python 3.12.3 was used for development and testing
- `matplotlib`

Install the required dependency using:

```bash
pip install matplotlib
```

---

## How to Run

Open a terminal inside this project folder and run:

```bash
python top_simulation.py
```

The simulation will:

1. Execute the AXI4 configuration transactions.
2. Run the DMA state machine.
3. Perform burst reads and packet-buffer writes.
4. Inject a deliberate AXI `DECERR` fault.
5. Generate and service interrupts.
6. Calculate performance metrics.
7. Generate `buffer_occupancy.png`.

Individual module self-tests can also be run using:

```bash
python axi_interconnect.py
python dma_subsystem.py
```

---

## Input Format

There is no external input file required.

The `top_simulation.py` script internally creates:

- A simulated DDR memory image.
- A descriptor ring containing valid packet descriptors.
- A deliberately faulty descriptor pointing to an unmapped address.
- Test data for the DMA and Ethernet MAC.

The faulty descriptor is inserted after the 3rd valid packet to demonstrate AXI `DECERR` fault handling.

---

## Expected Output

The simulation produces a transaction trace followed by a metrics section similar to:

```text
Packets transferred (DMA)     : 6
Frames transmitted (MAC)      : 6
DMA/AXI-side cycles           : 547 @ 200 MHz
DMA-side estimated throughput : 1123.22 Mbps
MAC wire bytes (incl. framing): 528
Effective Ethernet throughput : 727.27 Mbps out of 1000 Mbps line rate
Packet buffer overflow events : 260
Descriptors aborted (DECERR)  : 1
```

The exact values may depend on the current simulation parameters.

The simulation also generates:

```text
buffer_occupancy.png
```

which shows the packet-buffer occupancy and MAC `TX_BUSY` state over simulated time.

The complete console output from the simulation is provided in:

```text
simulation_log.txt
```

---

## Important Parameters

The main simulation parameters are defined in `top_simulation.py`.

| Parameter | Default | Meaning |
|---|---:|---|
| `NUM_PACKETS` | 6 | Number of valid packets transferred |
| `PACKET_SIZE_BYTES` | 64 | Ethernet frame payload size |
| `BURST_BEAT_BYTES` | 4 | AXI4 data width per beat (32-bit bus) |
| `BUFFER_DEPTH` | 20 | Packet-buffer depth in beats |
| `LINK_SPEED_MBPS` | 1000 | Ethernet line rate |
| `FAULT_AFTER_PACKET` | 3 | Position at which the faulty descriptor is inserted |
| `BAD_ADDRESS` | `0xF000_0000` | Unmapped address used to trigger `DECERR` |

---

## Golden-Model Note

This Python implementation is a **behavioral/functional reference model**, not RTL.

It is intended to serve as a golden reference for the eventual RTL implementation of the following blocks:

- AXI4 address decoder
- AXI4 arbiter
- DMA controller/FSM
- Packet buffer
- Interrupt controller
- Ethernet MAC register interface

The reference model can be used to compare and verify the behavior of the RTL implementation in the next phase of the project.

---

## Technologies Used

- Python
- AXI4
- DMA
- RISC-V / VEGA Processor
- Gigabit Ethernet MAC
- DDR Memory
- Behavioral / Functional Simulation
- High-Level Reference Modeling

---

## Project Structure

```text
axi4-interconnect-dma-simulation/
│
├── axi_interconnect.py
├── dma_subsystem.py
├── ethernet_mac_regs.py
├── top_simulation.py
├── simulation_log.txt
├── buffer_occupancy.png
├── README.md
└── .gitignore
```

---

## Project Purpose

The objective of this Phase-2 project is to develop a high-level functional model of an **AXI4 Interconnect and DMA subsystem for Gigabit Ethernet data transfer**.

The model demonstrates address decoding, arbitration, DMA descriptor processing, burst-based data movement, packet buffering, interrupt handling, error handling, and Ethernet transmission timing.

This high-level model provides the functional foundation for the subsequent RTL design and verification phase.
