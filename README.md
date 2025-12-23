# Arduino Milk/Syrup Dispenser System

Multi-board setup for portioned dispensing with weigh-scale feedback, a steam wand cleaning module, and an optional Modbus master CLI.

## Components
- Dispenser controller (Mega 2560, Modbus RTU slave ID 1)
- Steam wand cleaner (Nano Every, Modbus RTU slave ID 2)
- Scale module (Nano Every + HX711, I2C address 0x2A or 0x2B)
- Reference Modbus master / USB CLI (Nano Every, drives both slaves on one RS-485 bus)

## Bus and Protocol
- RS-485, 19200 8N1, Modbus RTU function 0x17 (Read/Write Multiple Registers)
- Shared register model: command block at 0x0000, result block at 0x0100
- Motor IDs: Milk 11–18, Sauce 21–35
- Turnaround guards: both master and slaves hold DE low whenever idle and keep a character-time settle after replies to avoid header corruption.

## Quick Start
1) Flash boards: dispenser → cleaner → scale(s) → master (optional).  
2) Power scale modules first so they auto-tare.  
3) Calibrate each scale: long-press (~3 s) to enter CAL, place known weight (default 500 g), short-press to store.  
4) Wire RS-485 with termination/bias; set slave IDs as above; master/slaves default to 19200 8N1.  
5) From the master USB CLI (`115200`): run `d.status` and `c.status` to verify Modbus replies.  
6) Dry tests: `d.trigger Sauce2 1` and `c.init 5`.  
7) Wet test example: `d.dispense Milk1 120 50 20 45`.

## Fault Model
- Dispenser: motor, leak, scale, timeout, aborted (latched until `CLEAR`)
- Cleaner: timeout, aborted, off status (off is not a fault)
- ABORT is always available and latches `ABORTED_FAULT`

## Command Cheat Sheet (master CLI)
- Dispenser: `d.status`, `d.clear`, `d.abort`, `d.rinse <s>`, `d.trigger <motor|name> <s> [s|speed]`, `d.dispense <motor|name> <target_g> <slowPct0-100> <softCutPct0-100> <timeout_s>`
- Cleaner: `c.status`, `c.clear`, `c.abort`, `c.off`, `c.init <s>`, `c.froth <targetC> <timeout_s>`, `c.clean <tValve_s> <tSteam_s> <tStandby_s>`

## User Guide (reliability tips)
- Keep only one master on the RS-485 bus; slaves release DE/RE to RX whenever idle.
- If you see “bad header” or timeouts, verify termination/bias and ensure both sides have a full char-time settle after replies; current firmware does this by default.
- During long triggers or dispenses, Modbus should stay responsive (STATUS/ABORT). If a fault latches, use `clear` to resume.

## Directory Guide
- `dispenser_I2C_master_modbusRTU_slave/` — Dispenser firmware (ID 1). See its README for wiring and API details.
- `steam_wand_modbusRTU_slave/` — Steam wand cleaner firmware (ID 2).
- `dispenser_slave_scale_feedback/` — Scale module firmware and I2C protocol.
- `rtu_master_tcp_slave_translator/` — Reference Modbus master + USB CLI.
- `EasyEDA PCB/` — Hardware design assets.

Detailed behavior, pinouts, and register layouts are documented in each subfolder README.
