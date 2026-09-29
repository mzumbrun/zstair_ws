# harp_cart_drive — Step 2 (ROS 2 drive node, no IMU)

ROS 2 Jazzy drive node for the HARP cart on `rpi4u`: `cmd_vel` in, `odom` out, RoboClaw
2×15A over USB. Built in rungs; each rung passes before the next adds complexity.

| Rung | Scope | Status |
|---|---|---|
| 2a | Read-only: encoders, battery → `/odom`, TF, diagnostics | **PASSED 2026-09-28** |
| 2b | `cmd_vel` → cmd 40, timeout, saturation, fault latch, controlled stop (**on blocks**) | **PASSED 2026-09-28** (T1–T8; see carry-in for as-run) |
| 2c | Floor acceptance: ±0.2% straight run through ROS 2 | **Next** (procedure below) |

## Conventions

REP-103: +x forward, CCW yaw positive. **M1 = left (driver side), M2 = right.** Motor
reversal and encoder-2 inversion live in the RoboClaw; +counts = forward on both wheels.

| Topic | Type | Content |
|---|---|---|
| `/cmd_vel` (sub) | geometry_msgs/Twist | Only when `motion_enabled: true` |
| `/odom` | nav_msgs/Odometry | Pose from count differences; twist from Δcounts/Δt |
| TF `odom→base_link` | | `publish_tf: false` in Step 3 (EKF owns it) |
| `/cart/wheel_counts` | std_msgs/Int64MultiArray | [M1, M2 accumulated, **monotonic ns at read**] |
| `/cart/battery` | sensor_msgs/BatteryState | RoboClaw reading + 0.4 V |
| `/diagnostics` | diagnostic_msgs/DiagnosticArray | Link counters, latency, target, cmd age, timeouts, saturations |

## Rung 2a result (as-run, 2026-09-28)

| # | Check | Result |
|---|---|---|
| 1 | Mapping and signs | PASS: left→M1; more left travel → −13.56° yaw (CW) as predicted |
| 2 | Identity vs independent snapshot | PASS: 81,128 / 80,646 counts, **0-count difference** over 17.08 m |
| 3 | Link | PASS: ~25,000 transactions, 0 retries/failures/CRC/short/implausible |
| 4 | Latency | PASS: p50 0.36–0.38, p99 0.89–0.94, max 3.39 ms (estimate was 1–3 ms) |
| 5 | Loop period | PASS: mean 50.00 ms, max 51.74 ms |

## Rung 2b design

Each cycle (20 Hz): **write** cmd 40 (every cycle, zeros included) → battery (1 Hz) → **read** encoders.

| Behavior | Design | Number |
|---|---|---|
| Command | SpeedAccelM1M2 (cmd 40), accel 2,360 c/s² | ramp 0→1,000 c/s in 0.42 s, 212 counts |
| Saturation | Scale both wheels by one factor at 2,000 c/s | (0.42 m/s, 1.0 rad/s) → M1 646, M2 2000 |
| `cmd_vel` timeout | 0.25 s, then target 0 (ramped) | fires 0.25–0.30 s after last msg (cycle phase) |
| Write fault | 3 consecutive failed cmd 40 writes → latch | counted separately: reads succeeding can't mask it |
| Link fault | 3 consecutive failed transactions of any kind → latch | |
| On latch | one stop attempt, then **zero serial traffic** | RoboClaw 0.2 s timeout stops motors |
| Worst stale command (write-only failure) | 3 × ~81 ms + 80 ms stop attempt + 0.2 s | ≈ 0.53 s ≈ 0.22 m at 0.42 m/s |
| Ctrl-C / SIGTERM | zero command, keep feeding watchdog until still (≤ 2 s) | ramp, not coast |
| SIGKILL / hang | RoboClaw 0.2 s timeout | ≈ 84 mm at 0.42 m/s |
| Motion gate | `motion_enabled` (code default false) | YAML sets true for 2b |

Node owns SIGINT/SIGTERM (`SignalHandlerOptions.NO`), which also removes the rung 2a
"publisher's context is invalid" rosout messages.

## Build and offline tests

```bash
cd ~/zstair_ws && git pull
colcon build --packages-up-to harp_cart_drive && source install/setup.bash
python3 -m pytest -q src/harp_cart_drive/test      # 29 tests, no hardware
```

## Rung 2b procedure — CART ON BLOCKS, wheels clear

Terminal A: `ros2 launch harp_cart_drive cart_drive.launch.py` (expect `MOTION ENABLED (rung 2b)`).
Terminal B: `speed_check` (run on rpi4u; it uses the node's monotonic stamps).

| T | Command (terminal B) | Prediction | Pass |
|---|---|---|---|
| 1 | `ros2 run harp_cart_drive speed_check --v 0.2112` | M1 +1000, M2 +1000 c/s; stop 212–312 counts | each ±0.3%; stop in band |
| 2 | `... --v -0.2112` | −1000 / −1000 | each ±0.3% |
| 3 | `... --w 0.5` | M1 −509, M2 +509; **left wheel turns backward** | each ±0.5%; direction by eye |
| 4 | `... --v 0.42 --w 1.0` | M1 +646, M2 +2000; ratio 0.323 | each ±0.5%; ratio ±0.5% |
| 5 | `... --v 0.2112 --end abandon` | node logs timeout; stationary 0.67–0.82 s after last msg; 462–562 counts | in band; exactly one timeout logged |
| 6 | Kill test (below) | post-kill counts ≈ 200 + 0–50 + coast | < 400 counts; wheels stop. ≥ 500 would mean the 0.2 s timeout isn't set |
| 7 | `... --v 0.2112 --duration 30`, then Ctrl-C terminal A mid-run | `Controlled stop: ~0.4–0.5 s, counts ~212` | 180–320 counts each; FINAL printed; no "context is invalid" lines |
| 8 | Stats line during T1–T4 | as 2a | 0 retries/fails; p99 < 10 ms |

**T6 kill test:**
```bash
# node stopped
ros2 run harp_cart_drive enc_snapshot --save /tmp/kill.json
# terminal A: launch node.  Terminal B:
ros2 run harp_cart_drive speed_check --v 0.2112 --end kill
# terminal C, after a few seconds at speed:
pkill -9 -f lib/harp_cart_drive/drive_node
# speed_check prints last counts seen, then:
ros2 run harp_cart_drive enc_snapshot --compare /tmp/kill.json
# post-kill counts = snapshot delta - last counts seen (per wheel)
```

Rung 2b passes when T1–T8 all pass. Then rung 2c (floor).

## Rung 2b result (as-run, 2026-09-28, on blocks)

| T | Test | Result | Verdict |
|---|---|---|---|
| 1 | +0.2112 m/s | −0.091% / −0.091% | PASS |
| 2 | −0.2112 m/s | −0.063% / −0.054% | PASS |
| 3 | ω +0.5 → −509 / +509 | −0.082% / −0.064%; left wheel backward | PASS |
| 4 | (0.42, 1.0) → 646 / 2000 | −0.062% / −0.094%; ratio 0.3231 vs 0.3230 | PASS |
| 5 | Abandoned `cmd_vel` | 512 / 538 counts, still at 0.80 s (pred. 462–562, 0.67–0.82 s) | PASS |
| 6 | kill -9 at 1,000 c/s | post-kill 225 / 230 counts (≈ 48 mm): 0.2 s timeout confirmed | PASS |
| 7 | Ctrl-C controlled stop | 0.57 s, 206 / 232 counts, clean FINAL | PASS |
| 8 | Link under motion | ~13,940 transactions, 0 retries/failures, p99 1.28 ms, max 3.15 ms | PASS (fault rate < 1/4,400 at 95%) |

**Stop baseline (replaces the idealized model; T1–T4 stop bands were a recorded prediction
miss):** measured from the node's stop command, decel M1 ≈ 2,430 c/s² and M2 ≈ 2,160 c/s²
against 2,360 commanded. `speed_check` stop counts include about 0.1 s of message and
cycle delay (T1: 310/336 = 100 + 210/236). Timeout stop from the 2,000 c/s cap ≈ 370–390 mm.

**Finding:** cmd 40 ramps each motor at the same rate, so ramps don't preserve curvature
(the slower wheel finishes first). Per-wheel accel is deferred until after 2c.

## Rung 2c procedure — FLOOR acceptance

**Goal:** reproduce the bring-up ±0.2% straight run through ROS 2. Code is unchanged from
2b; this rung tests the whole chain against physical truth.

**Setup**
- Hard floor, straight lane ≥ 6 m, plus ≥ 0.3 m clear beyond the end (timeout stop at
  test speed ≈ 0.10–0.11 m).
- Tape line along the lane with a sharp **start cross-mark**.
- A **fixed pointer** on the chassis reaching down to just above the tape: a stiff card or
  rod, ideally on the drive-axle centerline. A pointer off the axle adds < 1 mm along-line
  at the expected ~2.7° end heading (L_p·θ²/2 = 0.3 mm for L_p = 0.3 m).
- Tape measure hooked at the start mark. Read to ±1 mm; that is 0.02% at 5 m.

**Each run (do 3 runs, forward)**
1. Node stopped: `ros2 run harp_cart_drive enc_snapshot --save /tmp/floor.json`
2. Terminal A: `ros2 launch harp_cart_drive cart_drive.launch.py`
3. Align the pointer on the start mark. Don't touch the cart after this.
4. Terminal B: `ros2 run harp_cart_drive speed_check --v 0.2112 --duration 23.5`
   It records steady-state wheel speeds (check 1) and the floor stop counts (baseline).
5. After the cart stops, measure (a) the **along-tape distance** from the start mark to the
   pointer, and (b) the **lateral offset** of the pointer from the tape line (right +).
6. Ctrl-C terminal A and record the FINAL block: accumulated counts, path, x, y, yaw.
7. `ros2 run harp_cart_drive enc_snapshot --compare /tmp/floor.json` (check 2).
8. Check 3: error % = (odom x − tape) / tape × 100.

**Predictions**

| Quantity | Prediction | Basis |
|---|---|---|
| Distance per run | ≈ 4.9–5.0 m (about 23,300–23,700 counts) | 0.2112 m/s × 23.5 s including ramps |
| Wheel speeds | 1,000 c/s each, within the 2b spread (~−0.05 to −0.09%) | 2b; floor load may add small droop |
| Lateral offset | about **116 mm right** (±50 mm) | M1 larger wheel (96.0 vs 95.6 mm), symmetric model |
| odom y, yaw | y ≈ 0; yaw within about ±1° (start/stop kicks ~0.7°) | Encoders see equal counts; the curve is invisible to odometry |
| odom x vs tape | odom reads **≈ +0.036% long** (≈ +1.8 mm at 5 m) | Curve: along-line distance = path × (1 − θ²/6), θ ≈ 2.7° |
| Floor stop counts | at or below on-blocks 210 / 236 (+ ~100 delay) | Rolling friction helps decel |

**Pass criteria (all three runs)**

| # | Check | Pass |
|---|---|---|
| 1 | Command path on floor | 1,000 c/s ±0.2% each wheel |
| 2 | Plumbing identity | snapshot delta = node accumulated counts, within 1 count |
| 3 | System (truth) | odom x within ±0.2% of tape (±10 mm at 5 m) |

If check 3 fails while 1 and 2 pass, the error is in the truth measurement or in
D_eff (95.8 mm), not in the node. A consistent-sign error across all three runs means
D_eff needs refining; that is a constants update, not a code change.

**Also record** (not pass criteria): the lateral offset, since it is the first direct
measurement of the M1/M2 wheel ratio and sets up the per-wheel multiplier rung; and the
floor stop counts, as the loaded stop baseline.

## Rung 2c result (as-run)

| Run | M1 / M2 speed error | Identity Δ (counts) | Tape (mm) | odom x (mm) | Error % | Lateral (mm, right +) | odom yaw | Stop counts M1 / M2 |
|---|---|---|---|---|---|---|---|---|
| 1 | | | | | | | | |
| 2 | | | | | | | | |
| 3 | | | | | | | | |
| **Mean** | | | | | | | | |

