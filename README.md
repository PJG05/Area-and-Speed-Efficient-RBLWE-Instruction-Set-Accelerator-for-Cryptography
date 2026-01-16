# Area and Speed Efficient RBLWE Instruction-Set Accelerator
This repository contains the RTL design and verification of an area- and speed-efficient
instruction-set accelerator for Ring Binary Learning With Errors (RBLWE)-based
post-quantum cryptographic operations.

## Project Overview
The accelerator targets low-latency and low-power hardware implementation for
resource-constrained systems. It supports multiple cryptographic primitives through
an opcode-controlled instruction-set architecture and is designed with constant-time
execution to improve resistance against timing side-channel attacks.

## Supported Operations
| Opcode  | Operation  | Description |
|--------:|------------|------------|
| 00001 | POLYMUL | Negacyclic polynomial multiplication |
| 00010 | POLYADD | Integer-domain addition mod 7 |
| 00011 | BINADD  | Binary polynomial addition (GF(2)) |
| 00100 | SAMPLE  | LFSR-based error polynomial generation |
| 00110 | ADDE    | Binary polynomial addition with error |

## Architecture Highlights
- Instruction-set based RBLWE accelerator
- Bipartite NLFSR for error sampling
- Constant-time negacyclic polynomial multiplier
- FSM-controlled datapath
- Optimized for Area–Delay–Power (ADP)

## Verification
A self-checking SystemVerilog testbench validates all supported operations using
opcode-driven stimulus. The SAMPLE operation generates and stores the error polynomial
for reuse across simulations via file-based loading.

## Tools & Technologies
- Verilog / SystemVerilog (RTL + Testbench)
- Xilinx Verilog
- Cadence Genus (analysis)
- Git & GitHub


