# TVC Gimbal Demonstrator — Project README

Personal build reference: debugging log, firmware notes, and my own
Multisim simulation workflow, kept here so I don't have to reconstruct
any of this from memory later. This is *not* written as a tutorial for
anyone else — just my own record of what actually happened.

---

## Project Overview

Two-axis thrust vector control (TVC) gimbal demonstrator, inspired by
SpaceX Raptor's actuated engine mounts. Nested PVC ring mechanism, two
MG996R servos, ESP32 + MPU6050 + PCA9685, closed-loop PID stabilization
with a WiFi live-tuning dashboard.

Full engineering narrative, math, and results live in `TVC_Report.tex`.
This file is the messier, chronological "what actually broke and why"
companion to that.

---

## Debugging Log

Format: symptom → root cause → fix.

### PlatformIO: `SPI.h` not found after swapping MPU6050 libraries

**Symptom:** Adafruit BusIO (a dependency of the PCA9685 servo library)
failed to compile with `fatal error: SPI.h: No such file or directory`,
even after adding `SPI` explicitly to `lib_deps` and doing a full clean
rebuild.

**Cause:** `lib_ldf_mode = chain` (set earlier to fix ESPAsyncWebServer
resolution) only scans includes starting from my own source files — it
doesn't reliably discover a *nested* dependency's own internal requirement
(BusIO's SPI need, pulled in indirectly through the servo driver library).

**Fix:** Switched `lib_ldf_mode` from `chain` to `deep+`, which scans every
library's actual source files directly. Combined with a genuinely clean
`.pio` rebuild, this resolved it.

**Note:** the two LDF modes trade off against each other — fixing one
dependency issue with `chain` broke a different one that `deep+` needed
to catch. Worth remembering if I add more libraries later.

---

### Generic MPU6050 not detected by Adafruit_MPU6050 library

**Symptom:** `mpu.begin()` always returned false / "not found", even with
correct wiring confirmed via I2C scanner (device genuinely responding at
0x68).

**Cause:** The Adafruit library performs a strict chip-identity (WHO_AM_I)
check internally. My board is a generic/clone MPU6050, not a genuine
Adafruit part, and apparently doesn't match whatever the library expects.

**Fix:** Switched to `MPU6050_light` (rfetick), a more permissive library
that doesn't gate on the same identity check. Required reworking all
sensor-reading code: units changed from m/s²/rad/s to g/deg/s directly,
API changed to `mpu.getAccX()` / `mpu.getGyroX()` style calls instead of
`sensors_event_t`.

---

### Axis mapping had to be re-derived after every physical remount

**Symptom:** Pitch/yaw control repeatedly broke (wrong axis responding,
one axis laggy/unresponsive) every time the IMU's physical mounting
orientation changed.

**Root cause (recurring):** which raw sensor axis (X/Y/Z) corresponds to
pitch vs. yaw is purely a function of physical mounting orientation, not
something that can be assumed to carry over between mounts.

**The reliable diagnostic, every time:** print raw `ax, ay, az`, physically
rotate *only* the true pitch pivot, and see which raw axis stays flat —
that's the true rotation axis for that pivot. Repeat for yaw.

**A real bug I introduced along the way:** at one point I swapped the gyro
term to a different axis while leaving the accelerometer term on the old
axis — two mismatched terms fighting each other every loop, producing a
lag/backtrack symptom that looked like a hardware problem but was purely
a code mismatch.

**Also recurring:** the calibration sketch's printed "suggested offset"
formulas assume Y is the up-axis by default — when a different axis
(X, or −X) was actually up, the offsets had to be manually re-derived
by working backward through the raw averages rather than trusting the
printed suggestion directly.

---

### Servo commands losing precision to integer truncation

**Symptom:** PID corrections would stop updating the servo entirely once
the remaining error got small — system would plateau well short of true
zero.

**Cause:** `int pitchServoAngle = SERVO_CENTER + (int)(correction * ratio);`
— the `(int)` cast truncates any correction smaller than 1 full degree
before it ever reaches the servo, discarding real, valid PID output.

**Fix:** kept `pitchServoAngle`/`yawServoAngle` as `float` all the way
through, and rewrote `angleToPulse()` to accept a float and do the linear
map manually (Arduino's built-in `map()` is integer-only).

---

### Integral windup — twice, two different mechanisms

**First occurrence:** raised `INTEGRAL_LIMIT` from 20 to 60 to try to
close a persistent steady-state error. Produced a much worse problem —
the servo pinned at its hard clamp for 25+ consecutive loop iterations
during a large disturbance, then took a long time to unwind afterward,
overshooting well past the original error in the opposite direction.
**Lesson:** raising the integral ceiling without addressing *why* it
needs to be higher just gives windup more room to do damage.

**Second occurrence, more subtle:** even with a modest `INTEGRAL_LIMIT`,
holding a sustained disturbance (tilting the demo platform and holding
it) let the integral accumulate toward its cap for the whole duration,
even though the output was already saturated and couldn't move further.
On release, the pinned integral value took time to unwind, again
overshooting to the opposite extreme.

**Actual fix:** implemented proper anti-windup — conditional integration
that only accumulates the integral term when the output isn't already
saturated (or when the current error would pull it back into range).
This is the textbook-correct fix, not just picking a different constant.

**Separately:** the reason the steady-state error existed at all, even
before windup entered the picture, is that pure P-control structurally
cannot fully cancel a *sustained* disturbance (holding the platform at a
fixed tilt) — only the integral term can drive that to zero. `Ki` had
been set far too conservatively (max possible contribution was ~0.2°,
literally off by roughly 50x from what the sustained-hold scenario
actually needed).

---

### Servo direction backwards — sign convention vs. physical direction

Every axis remount required re-verifying `Kp` sign with a simple push
test (does the servo correct back toward level, or away from it?). Wrong
sign produces genuine positive feedback — the correction reinforces the
error instead of canceling it — which looks identical to "the gimbal is
broken" if you don't know to check this specifically.

**Also learned the hard way:** `Ki` and `Kd` must carry the *same* sign
as their axis's `Kp`, not be tuned independently. Mismatched signs make
the integral term fight the proportional term — worse the longer/larger
the error, producing a "works fine for small disturbances, plateaus at
larger ones" symptom that looks like an undertuning problem but is
actually an internal sign conflict.

---

### Dual power (USB + external VIN) risk

Wanted to run the ESP32 off the same external 5V rail as the servos
(so I didn't need USB connected all the time), but connecting both
USB and external VIN simultaneously without protection risks backfeeding
current into the computer's USB port if the board doesn't have its own
internal isolation diode.

**Fix:** added a series diode (1N4007 on hand; ~0.7–1V drop) between the
external supply and VIN. Confirmed via multimeter that VIN still held
comfortably above the ESP32 regulator's minimum input under load. A
Schottky (1N5819, lower drop) would be the better long-term part if
brownouts ever show up under heavier WiFi load.

---

### Onshape BOM: identical parts not merging into one row

**Screws (Derived from the standard parts library, 4×):** each Derive
operation creates its own independent part identity, even from the same
source — visually identical, same name, but the BOM doesn't recognize
them as the same part. Switching BOM view to "Flattened" didn't fix it
by itself in this case.
**Fix used:** manual consolidation in the exported BOM (delete duplicate
rows, set quantity directly) rather than fighting Onshape's Derive
identity tracking further.

**Legs (Circular Pattern, 3×):** patterned parts are *supposed* to merge
correctly (they reference one true source part), but didn't in my case —
likely because the pattern was applied to the sheet metal leg while it
was still in an "active" sheet metal state, before `Finish Sheet Metal
Model`. Correct order is: finish sheet metal first, pattern second.

---

### Sheet metal + Transform: "active sheet metal parts are not allowed"

Tried to reposition a sheet metal leg using the Transform tool and got
blocked outright. This is a real, documented Onshape limitation — several
standard part-editing tools (Transform included) don't operate on sheet
metal parts while they're still "active" (tracked with bend
table/flat-pattern state).
**Fix:** `Finish Sheet Metal Model` locks in the shape and drops the
active tracking, after which it behaves like a normal solid for
Transform, Boolean, Pattern, etc. Trade-off: the flat pattern/DXF export
freezes at that point too — grab it *before* finishing if it's still
needed.

---

## Multisim Circuit Simulation — My Process Notes

Used to validate the fan's PWM switching circuit (NPN transistor,
low-side switch) before building it on the real breadboard. Personal
reference for how I actually set this up.

### Circuit built
- PWM signal source → 220Ω base resistor → PN2222 base
- PN2222 emitter → shared GND
- PN2222 collector → node shared by: fan-resistor-stand-in (to +5V),
  flyback diode (to +5V, cathode up), and an LED indicator branch
- LED branch wired the same direction as the fan/diode branches (5V →
  330Ω → LED → collector), **not** collector → LED → GND — got this
  backwards on the first attempt, which made the LED indicate the
  *opposite* of the actual fan state. Traced it through: when the
  transistor is ON, it pulls the collector low, so an LED branch that
  only has voltage across it when the collector is *high* will light
  during OFF, not ON.

### PWM component (Power → POWER_CONTROLLERS → PWM)
- `Frequency`: 25000 (25kHz)
- `OutputVoltage`: 3.3 (matches real ESP32 GPIO high level)
- Duty cycle is **not** a field on the component itself — it's set by
  feeding a separate DC voltage source into the component's control
  input pin, with the duty cycle equal to (control voltage) / (Triangle
  range). This tripped me up initially — assumed duty was a parameter,
  not an external signal.
- `PwmMode`: left enabled (tightens simulator time-step at switching
  transitions for accurate edges).

### Oscilloscope setup
- Channel A: + to base node, − to shared ground → confirms the PWM
  signal itself is reaching the transistor correctly.
- Channel B: + to collector node, − to shared ground → shows the
  actual switching result. This trace is *inverted* relative to
  Channel A (low when transistor's on) — correct, expected behavior
  for a low-side switch, not a fault.
- Both channels' minus leads land on the same shared ground net — it's
  a difference measurement, doesn't matter exactly where on the rail.

### Timebase gotcha
First attempt at reading the switching waveform showed either a flat
line or a thick smeared band — turned out to be the timebase set far
too slow for a 25kHz signal (25kHz = 40µs per cycle). Also caught myself
once typing `15` into the Time/Div field expecting microseconds, which
Multisim read as 15 **seconds** — always specify the unit explicitly
(`15us` or `2e-5`), don't trust a bare number.

### Fan stand-in
No literal "PC fan" component exists in Multisim — modeled it as a
plain resistor (~100Ω, chosen for a safe/reasonable current, not meant
to exactly match the real fan's actual resistance) between +5V and the
collector node. The point of the simulation was validating the switching
behavior, not reproducing exact fan current.

---

## Firmware Build Markers

Each major main.cpp revision printed a distinct string in `setup()` —
useful for confirming which exact version is actually running on the
board rather than assuming based on what was last edited:

- `MAIN-1` — first WiFi dashboard + Adafruit_MPU6050 version
- `MAIN-2, MPU6050_light` — post library swap, placeholder axis mapping
- `MAIN-3, final mount` — real mounted orientation, confirmed axis
  mapping and calibration

---

## Still Open / In Progress

- Yaw axis tuning still noticeably behind pitch — separate investigation
  from the anti-windup fix.
- Final Kp/Ki/Kd values not yet locked — actively tuning via the WiFi
  dashboard.
- Need to re-verify Kp sign and recalibrate if the sensor is remounted
  again for any reason.