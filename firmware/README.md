# Prebuilt TC-GU-01 firmware

The **latest released** firmware images, so you can upgrade a gripper without
access to the firmware source repo (which is internal and stays out of this
SDK — see the repo README's "Firmware / PC GUI reference repos").

Only the current release lives here. Older images are recoverable from this
directory's git history, not from extra files.

| Image | Role | Version | Protocol | Source | Size | CRC32 |
| --- | --- | --- | --- | --- | --- | --- |
| `tc-gu-01-master.bin` | leader (SN ends **`m`**) | **1.2.1** | V2.1 | `6b4605a` | 116,840 B | `0xEC491CBD` |
| `tc-gu-01-slave.bin` | follower (SN ends **`s`**) | **1.1.2** | V2.2 | `bf0a06e` | 156,048 B | `0x866B1800` |

Both from firmware branch `hw_v1.1.0`. `manifest.json` has the same data
machine-readably, per image — the two roles no longer share one source commit
or one protocol level, because every V2.2 command is follower-only and the
leader had no reason to be rebuilt.

> ### ⚠️ The follower image is a local build
>
> `tc-gu-01-slave.bin` 1.1.2 was built here with `arm-none-eabi-gcc 13.2.1`,
> **not** by the firmware team's release toolchain. It is a faithful build of
> `bf0a06e`, but it is not the official artifact — replace it when that lands.
>
> **Its size and CRC32 are not comparable with the 1.1.1 row's.** Rebuilding
> the *previous* image from its *own* commit with this toolchain also fails to
> reproduce it (150,044 B against the shipped 149,256 B), so roughly 800 bytes
> of the jump from 149,256 to 156,048 is toolchain, not new firmware code.
> Do not read the delta as a measure of what V2.2 added.
>
> It has **not been validated on hardware yet.** The SDK's V2.2 support is
> unit-tested against the wire format only.

**Leader 1.2.1 changes the status LED only** — protocol byte-identical to
1.2.0, so upgrading the leader is optional from the SDK's point of view:

- normal state: solid **white** at brightness 20 (was solid green at 10)
- fault state: blinks at 500 ms (was 1000 ms)
- the key-press LED reactions (click-blink, long-press-solid) are disabled

**Follower 1.1.2 adds command set V2.2** — the four follower diagnostic
commands (`0x3A` / `0x3B` startup limit torque, `0x52` fault report, `0x53`
extended motor status). It is purely additive: `GetMotorStatus` (0x50) and the
motor-status DATA stream still carry the same 31-byte payload, so everything
the SDK did against 1.1.1 behaves identically. Upgrade only if you want the
diagnostics — on 1.1.1 those four raise `ProtocolError(InvalidCmd)`.

It also retunes power-on auto-calibration: single-sample stall confirmation,
the open stall records the frame *before* the trigger, and 0.013 rad is
subtracted from the saved `max_open` as a safety margin. **Re-run calibration
after upgrading** if you depend on the exact span — expect it slightly smaller.

---

## ⚠️ Update the SDK *before* flashing

If you are on an SDK older than 0.1.7, install the new SDK **first**, then
flash. Not the other way round.

Older `OtaSession` only checked `ack.is_nack`, but firmware handler errors
come back on the echoed-command path — the command byte intact and the error
as the single payload byte — which the transport cannot tell from a valid
1-byte success. So a rejected write did not raise: the loop wrote every
remaining block, `verify()` and `apply()` swallowed their errors too, and the
update **reported success on a firmware that had aborted the session**. It
also retried with a fresh sequence number, which the firmware sees as a new
request; since it requires strictly sequential offsets and fails the whole
session on a repeat, a merely-slow ACK could kill an otherwise fine update.

Both are fixed in 0.1.7. The new SDK talks to old firmware fine — everything
up to command set V1.9 is unchanged — so upgrading the SDK first is safe.

```bash
pip install -e .          # or however you install this SDK
```

## Flashing

```bash
# 1. Which grippers are attached, and what are they running?
python python/examples/fisheye_cal.py show

# 2. Flash. --side picks the gripper; the image must match the ROLE.
#    Naming the image is enough — the script looks here for it, so this
#    line also works from a parent repo that vendors this one.
python python/examples/ota_update.py tc-gu-01-master.bin \
    --side left --target-version 1.2.1

# 3. Confirm — GetVersion returns the compiled-in constant, so the version
#    you read back is proof of what actually landed.
python python/examples/fisheye_cal.py show --sn <SN>
```

Takes about a second. The MCU reboots and re-enumerates over USB in ~1–3 s.

### Pick the image by ROLE, not by side

A gripper's role is the **last character of its firmware SN**, not which hand
it is on: `TCGU01A28Z0023m` → `m` → **master**. Two grippers on opposite sides
of the same rig are often both masters.

Flashing the wrong role's image bricks the MCU and needs an SWD probe to
recover. Check first:

```bash
python -c "from xense.taccap import scan_grippers
for g in scan_grippers(): print(g.firmware_sn, '->', 'master' if g.firmware_sn.endswith('m') else 'slave')"
```

### Verify before you flash

The manifest's CRC32 is the same value `ota_update.py` prints and sends in
`OtaStart`, so it is worth a look if the file has travelled:

```bash
python -c "
from xense.taccap import crc32_iso_hdlc
print(hex(crc32_iso_hdlc(open('firmware/tc-gu-01-master.bin','rb').read())))"
# → 0xec491cbd
```

## How OTA works, briefly

The image is written to the **inactive** flash bank and activated with the
STM32H5 bank swap, so the plain `.bin` serves both banks — it always links at
`0x08000000` and the swap remaps the target bank there. Nothing is overwritten
until `OtaApply`, and the firmware refuses to swap on a CRC mismatch, so a
failed transfer leaves the running image intact.

No SWD probe is involved; it goes over the same serial link as sensor I/O.

## Rebuilding these

The images are build artifacts of the internal firmware repo. See the main
README's [firmware section](../README.md#firmware--pc-gui-reference-repos)
for the toolchain and the two traps (conda's exported host `CFLAGS` breaking
the ARM cross-compile, and the 456 KB single-bank OTA cap).

When you refresh these files, regenerate `manifest.json` too — the CRC32 in
it is what people check against.
