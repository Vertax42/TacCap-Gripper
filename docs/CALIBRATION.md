# 标定

<!-- 从 README.md 拆出，保持内容不变；README 只保留入门路径。 -->

## Calibration

Mechanical slop and small post-zero drift can make the encoder report
~0.05–0.10 rad when the gripper is held "fully closed". The SDK
absorbs this two ways:

- **Auto-clamp**: `Encoder::read_once()` and `on_data` callbacks return
  `position_rad >= 0`. Negative raw drift becomes `0.0` to keep
  downstream consumers' math sane. The unclamped value is preserved
  in `raw.position_rad` (C++) / `raw_position_rad` (Python) for
  diagnostics.
- **Drift warning**: if the raw negative drift exceeds **-0.1 rad** the
  logger emits a rate-limited warning (1 / s per `Encoder` instance)
  pointing at calibration or mechanical issues.

To calibrate a gripper, run `calibrate.py` against the side you want to fix
(or its SN, if you'd rather be explicit):

```bash
python python/examples/calibrate.py left               # by side
python python/examples/calibrate.py TCGU01A24Z0001m    # by firmware SN
```

The script:

1. Resolves `left`/`right` (or the SN you passed) to one `mcu_device`, and
   prints the firmware SN it picked plus every gripper it can see, so the
   pick is verifiable. Side comes from the firmware-burned SN read over the
   wire (`Cmd::GetSn`), not the CH343 chip serial.
2. **Pre-flight:** checks the firmware implements `Cmd::EncoderMaxCal`
   (0x2C). This runs *before* anything is written — step 4 persists a new
   zero, so a pre-V2.1 gripper is refused while still untouched rather than
   left half-calibrated with a new zero and no span.
3. Prints the current encoder reading (both `raw` and clamped) so the
   existing drift is visible.
4. Prompts "hold the gripper **FULLY CLOSED**, press [Enter]", sends
   `Cmd::SetEncoderZero`, re-reads, and validates the new raw reading is
   within ± 0.01 rad.
5. Prompts "open to the **MECHANICAL LIMIT**, press [Enter]" and **stores**
   the measured angle as the travel span (`Cmd::EncoderMaxCal`). That span
   is what `normalize_position=True` divides by.
6. Live 10 Hz readout (`raw | cooked | position 0..1`) until Ctrl+C.

There is deliberately **no expected full-open angle** to check against. The
measured span *is* the calibration — it is whatever the mechanism does, and
it is what the SDK normalizes by. (An earlier 1.7 rad "design baseline" was
stale and fired false alarms on healthy hardware: three measurements across
two units gave 1.1582 / 1.1589 / 1.1486 rad, i.e. ~66°.) The only checks
left are the ones that catch a genuinely broken measurement — a non-positive
span, a zero that did not take, a write that did not stick.

`--skip-open-probe` latches only the zero; normalized position then stays
unavailable until the span is measured.

The firmware latches whatever raw count it sees the moment it
processes the command, so the gripper must already be at the target
pose before pressing Enter.

### Flash-persisted calibration records (V2.0 / V2.1)

Two more records live in MCU flash behind `g.calibration`. Note the encoder
zero above is **also** flash-persisted — `cmd_handler_set_encoder_zero` calls
`storage_write_encoder_calibration()`, so none of these need a power cycle
and none of them are lost on reboot:

| Record | Command | Scope | Accessor |
| --- | --- | --- | --- |
| Fisheye camera `fx, fy, cx, cy, k1..k4` | `0x2B` | leader + follower | `read_fisheye()` / `write_fisheye()` |
| Encoder max travel angle (rad) | `0x2C` | **leader only** | `read_encoder_max_rad()` / `write_encoder_max_rad()` |

Both survive power cycles. A record that was never written reads back as
`None` — the firmware answers `ErrorCode.CalNotSet` instead of returning
zeros, so "never calibrated" is distinguishable from "calibrated to exactly
0". Every other firmware error still raises `ProtocolError`; on a follower or
on pre-V2.1 firmware the encoder-max methods raise with `InvalidCmd`.

```bash
python python/examples/fisheye_cal.py show                  # print both records
python python/examples/fisheye_cal.py measure-encoder-max   # guided: zero, open, store
```

