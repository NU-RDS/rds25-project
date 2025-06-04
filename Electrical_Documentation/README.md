# Electrical Engineering Project Documentation Wiki

Welcome to the final documentation portal for RDS 2025 Electrical Team. This wiki provides a comprehensive view of the system architecture, development process, challenges faced, and recommendations for future iterations.

---

## Table of Contents

| Section                             | Description                                                 |
|-------------------------------------|-------------------------------------------------------------|
| [1. Original Architecture](#1-original-architecture)     | Initial ebedded system design, the primary driver of electrical hardware decisions |
| [2. Existing Tooling](#2-existing-tooling)               | The motor drivers, position sensors, MCU, and power domain tools alreafy in place as of 1 April 2025 |
| [3. Project Goals](#3-project-goals)                     | The Design Parameters, goals, plans, and backup plans we agreed upon for Spring quarter '25    |
| [4. Project Plans and Backup Plans](#4-project-plans-and-backup-plans) 
| [5. Project Outcome](#5-project-outcome)                 | Summary of results, deliverables, and performance           |
| [6. Issues and Painpoints](#6-issues-and-painpoints)     | Technical and procedural blockers encountered, and how we overcame them |
| [7. Future Work](#7-future-work)                         | Suggestions for extending and improving the project         |
| [8. Recommendations for Future Teams](#8-recommendations-for-future-teams) | Practical advice from this iteration, and how to pick up where we left off |

---

## 1. Original Architecture

| Component         | Details                              |
|------------------|--------------------------------------|
| High-Level Control             | Laptop running ROS                      |
| Mid-Level Control              | Teensy 4.0             |
| Low-Level Control              | Teensy 4.0        |
| Joint Encoders                 | AS5047P to agggregating sensor board (palm board)     |
| Motor Encoders                 | AS5047P straight to mid-level teensy    |
| Power Domain                   | Power Distribution Board |
| Palm Board                     | Sensor Aggregation |

---
<details>
  <summary>📷 Click to view Original System Diagram</summary>

  ![Original System Diagram](./figures/original-system.png)

</details>

---

## 2. Existing Tooling

| Tool              | Purpose                              |
|------------------|--------------------------------------|
| Motor Driver & Logic/Control| Firmware development               |
| Motor Driver | Analog simulation and modeling       |
| Power Distribution  | PCB schematic and layout             |
| Logic / Control| Data logging and visualization       |
| Position Sensing | Pre-fabbed encoder boards from Mouser |

---
<details>
  <summary>📷 Click to view ODrive</summary>

  ![ODrive](./figures/ODrive.png)

</details>
<details>
  <summary>📷 Click to view Pre-fabbed Encoder</summary>

  ![Pre-fabbed Encoder](./figures/encoder-prefab.png)

</details>
<details>
  <summary>📷 Click to view RDS '24 Teensy 4.1 Breakout</summary>

  ![RDS '24 Teensy 4.1 Breakout](./figures/teensy41.png)

</details>
<details>
  <summary>📷 Click to view RDS '24 Power Board</summary>

  ![RDS '24 Power Board](./figures/2024power.png)

</details>
<details>
  <summary>📷 Click to view Ivor's Motor Driver</summary>

  ![Ivor's Motor Driver](./figures/ivor-motordriver.png)

</details>

---

## 3. 🚀 Project Goals

Our primary objective for Spring 2025 was to **consolidate disparate electrical subsystems** into a **modular, unified PCB stack** that handled:

| Domain                    | Responsibilities                                       |
|---------------------------|--------------------------------------------------------|
| 🟧 **Power Domain**        | 24V input → 5V regulation and stable power delivery    |
| 🟦 **Switching Domain**    | 40kHz gate driving and isolation switching logic       |
| 🟩 **Logic & Communication** | SPI, CAN, and MCU interfacing for control & telemetry |

The design is structured for **stackable, role-specific PCBs**, easing debugging, replacement, and scalability. The final layout aggregates all functional blocks into a clear topological arrangement, as shown below.

<details>
  <summary>📷 Click to view Modular Control Goal Diagram</summary>

  ![Modular Control Goal](./figures/modular-control-goal1.png)  
  **Figure 3.** Modular PCB stack: domain separation by function with visual overlays and control pathways highlighted.

</details>

We defined success as:
- A **working modular board system** with power, logic, and switching blocks  
- Compatibility with **existing Teensy-based controllers**  
- Future-ready for **multi-channel motor drivers**

---

## 4. Project Plans and Backup Plans

| Plan Order       | Status / Result                      |
|------------------|--------------------------------------|
| A                |  |
| B                |  |
| C                |  |
| D                |  |

---
## 5. Project Outcome

| Deliverable       | Status / Result                      |
|------------------|--------------------------------------|
| Hardware Rev 1    |                                     |
| Hardware Rev 2    |                                     |
| Firmware v1.2.3   |                                     |
| Deployment        |                                     |

---

## 6. Issues and Painpoints

| Category          | Problem                              | Resolution / Notes                          |
|------------------|--------------------------------------|---------------------------------------------|
| MCU Sleep Modes   | Unreliable LPM3 wakeup               | RTC interrupt logic revised                 |
| SPI Communication | Glitch at low VCAP                   | Capacitance buffer improved                 |
| Cold Start        | Unstable VOUT under low light        | Increased input capacitance + watchdog delay|

---

## 7. Future Work

| Area              | Next Steps                           |
|------------------|--------------------------------------|
| Hardware          | Migrate to Apollo3 for ultra-low power |
| Sensors           | Solid-state pH + moisture hybrid     |
| Networking        | Evaluate LoRa P2P for redundancy     |
| Storage           | Add FRAM buffering during outages    |

---

## 8. Recommendations for Future Teams

| Tip                                      | Why It Matters                               |
|------------------------------------------|-----------------------------------------------|
| Start with power-first design            | Energy availability drives all decisions      |
| Use precise current measurement early    | Debugging low-power bugs is easier upfront    |
| Version-control firmware + hardware docs | Prevent loss of institutional knowledge       |
| Modularize and test incrementally        | Complex systems fail at integration           |

---

