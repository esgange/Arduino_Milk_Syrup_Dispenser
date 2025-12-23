# Modbus Master + USB CLI (Reference)

Nano Every firmware that drives both slaves on one RS-485 bus and provides a USB CLI for testing/operation. Uses Modbus RTU function 0x17 with background polling and graceful salvage when start replies are lost.

## Bus
- RS-485 on Serial1, 19200 8N1
- MAX485 wiring: `RO->D0(RX1)`, `DI->D1(TX1)`, `RE+DE->D2` (HIGH=TX, LOW=RX)

## Targets
- Dispenser: slave ID 1 (see dispenser README)
- Cleaner: slave ID 2 (see steam wand README)

## Behavior
- Non-blocking: start commands log once, then STATUS is polled ~150 ms until completion.
- If a start reply CRC/timeout occurs but the slave shows `ACTIVE` on STATUS, the master treats the command as accepted (salvage path).
- `abort` requests can be issued while an op is pending.
- RS-485 timing: pre/post TX guards around DE/RE to avoid header corruption; RX is purged before each transaction.

## USB CLI (115200)
- Dispenser (`d.*`):  
  `d.status`, `d.clear`, `d.abort`, `d.rinse <seconds>`,  
  `d.trigger <motorId|name> <seconds> [s|speed]`,  
  `d.dispense <motorId|name> <target_g> <slowPct0-100> <viscousPct0-100> <timeout_s>`
- Cleaner (`c.*`):  
  `c.status`, `c.clear`, `c.abort`, `c.off`,  
  `c.init <seconds>`, `c.froth <targetC> <timeout_s>`,  
  `c.clean <tValve_s> <tSteam_s> <tStandby_s>`
- Motor tokens: `Milk1..Milk8` (`m1..m8`) map to IDs 11–18; `Sauce1..Sauce15` (`s1..s15`) map to IDs 21–35; numeric IDs accepted.

## Notes
- RESULT block from slaves is printed briefly on replies (`result=..., error=..., status=...`).
- Pending operations print a single completion line: `[cmd] done.` or `[cmd] ended: <STATUS>`.
