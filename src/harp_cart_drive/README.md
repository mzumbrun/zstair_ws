# harp_cart_drive — Step 2, Rung 2a (read-only)

ROS 2 Jazzy drive node for the HARP cart on `rpi4u`. Rung 2a reads the RoboClaw
(encoders cmd 78, battery cmd 24) over USB and publishes odometry. **It sends no
motion commands and subscribes to nothing.** Motion (`cmd_vel`) arrives in rung 2b
only after this rung passes.

## Conventions

REP-103: +x forward, CCW yaw positive. **M1 = left (driver side), M2 = right.**
Motor reversal and encoder-2 inversion live in the RoboClaw, so +counts = forward on
both wheels and the node applies no sign flips. The bring-up spin rows (CCW: M1 −,
M2 +) are encoded as unit tests.

| Topic | Type | Content |
|---|---|---|
| `/odom` | nav_msgs/Odometry | Pose from count differences (midpoint integration); twist from Δcounts/Δt |
| TF `odom→base_link` | | `publish_tf: false` in Step 3 when the EKF owns it |
| `/cart/wheel_counts` | std_msgs/Int64MultiArray | [M1, M2] accumulated since node start |
| `/cart/battery` | sensor_msgs/BatteryState | RoboClaw reading + 0.4 V |
| `/diagnostics` | diagnostic_msgs/DiagnosticArray | Link counters, latency p99, battery |

The node never resets the encoders. The first read is the baseline, which is what makes
the independent before/after identity check possible.

## Build and offline tests

```bash
cd ~/zstair_ws
rosdep install --from-paths src/harp_cart_drive -y --ignore-src
colcon build --packages-up-to harp_cart_drive
source install/setup.bash
python3 -m pytest -q src/harp_cart_drive/test     # 21 tests, no hardware
```

## Rung 2a procedure

Cart on blocks for steps 1–2, on the floor for step 3. RoboClaw powered, no other
process holding the USB port.

1. **Signs (on blocks).** `ros2 launch harp_cart_drive cart_drive.launch.py`, then
   `ros2 topic echo /cart/wheel_counts`. Turn the **left** wheel forward by hand:
   only element 0 (M1) rises. Right wheel forward: element 1 rises. Then turn left
   back / right forward: `/odom` yaw goes **positive** (CCW).
2. **Loop and link.** Let it run 10 min. Read the periodic stats line: latency
   p50/p95/p99/max, period mean/max, counters.
3. **Identity (floor, push by hand ~3–5 m, any path).**
   ```bash
   ros2 run harp_cart_drive enc_snapshot --save /tmp/before.json   # node stopped
   ros2 launch harp_cart_drive cart_drive.launch.py                # push cart, then Ctrl-C
   ros2 run harp_cart_drive enc_snapshot --compare /tmp/before.json
   ```
   Compare the snapshot deltas with the node's FINAL "accumulated counts".
   Note: pushing back-drives the gearmotors; brief counts between the snapshot and the
   node's baseline read are the only way the two can disagree, so keep the cart still
   during start/stop.

## Pass criteria

| # | Check | Pass |
|---|---|---|
| 1 | Wheel mapping and signs | Left→M1, right→M2, both + forward; left-back/right-forward gives +yaw |
| 2 | Identity (truth-free) | Snapshot Δcounts = node accumulated counts, within 1 count per wheel |
| 3 | Link | 0 failures, 0 implausible jumps over ≥10 min; retries logged if any |
| 4 | Latency (replaces the 1–3 ms estimate) | Record p50/p99/max. p99 < 10 ms keeps a 2-transaction 20 Hz cycle under 40% busy |
| 5 | Loop period | Mean 50 ms ± 1 ms, max < 100 ms |

## Deliberately not in 2a

`cmd_vel`, SpeedAccelM1M2 (cmd 40), `cmd_vel` timeout (0.25 s), curvature-preserving
saturation (implemented and tested in `kinematics.inverse`, not called), per-wheel
multipliers (left 1.0021 / right 0.9979, parameters present at 1.0), voltage-scaled caps.
