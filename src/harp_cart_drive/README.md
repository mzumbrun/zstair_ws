# harp_cart_drive — Step 2 (ROS 2 drive node, no IMU)

ROS 2 Jazzy drive node for the HARP cart on `rpi4u`: `cmd_vel` in, `odom` out, RoboClaw
2×15A over USB. Built in rungs; each rung passes before the next adds complexity.

| Rung | Scope | Status |
|---|---|---|
| 2a | Read-only: encoders, battery → `/odom`, TF, diagnostics | **PASSED 2026-09-28** |
| 2b | `cmd_vel` → cmd 40, timeout, saturation, fault latch, controlled stop (**on blocks**) | This build (0.2.2) |
| 2c | Floor acceptance: ±0.2% straight run through ROS 2 | Next |

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
