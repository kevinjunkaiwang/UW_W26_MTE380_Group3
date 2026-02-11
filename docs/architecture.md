# Architecture Snapshot

## Data flow
- Pi (webcam @ 20–60 Hz): capture frame → isolate line in bottom ROI → compute `Lk_norm` (lookahead) and validity flag → send ASCII `L,<lk>,<valid>\n` over USB serial (115200).
- Arduino Mega (control @ 200–1000 Hz): read IR QTR-8A → compute lateral error `e` → use latest `Lk_norm` to schedule base speed + PID gains → PID → mix to left/right commands → TB6612FNG PWM/dir (or print when motors absent).
- Staleness guard: if no fresh vision packet for 200 ms, drop to IR-only safe mode (reduced base speed, conservative gains).

## Two scheduling options
- **Default (Lookahead scheduler):** `Lk_norm` directly maps to speed bands and PID gain sets (simple, light).
- **Paper fuzzy scheduler (reference):** Inputs `X1` (current speed %), `X2` (line length %), 6 fuzzy rules (Table I) → output `X*` (1–100) → selects speed cap and PID triplet. Implemented in `pi/src/paper_fuzzy.py` for experimentation.

## Timing
- Vision loop target: 30 Hz (USB webcam).
- Control loop target: 200–500 Hz (set `CONTROL_DT_US` in firmware).

## Serial protocol
- ASCII line, newline-terminated: `L,<lk_norm>,<valid>`
  - `lk_norm`: float in `[0,1]`
  - `valid`: `1` if line detected, else `0`

## Hardware mapping (default pins)
- QTR-8A analog: `A0..A7`
- TB6612FNG: `AIN1=7, AIN2=6, PWMA=5, BIN1=4, BIN2=3, PWMB=9, STBY=8`
- Adjust in `arduino/firmware/firmware.ino` as needed.
