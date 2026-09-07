# Driver Communication Protocol

This document is the agreed contract between the user-space application
(Tasks 1-4) and the two kernel drivers, so both sides can be implemented
in parallel against a fixed format. Source of truth for the exact types
and constants is the shared headers in [`include/`](../include/):
[`haptic_feedback.h`](../include/haptic_feedback.h) and
[`proximity_warning.h`](../include/proximity_warning.h).

## 1. `/dev/haptic_feedback`

Character device controlling the buzzer/vibration motor. Binary
interface, defined in `include/haptic_feedback.h`.

**Payload** (`struct haptic_pulse`):

| Field            | Type   | Range     | Meaning                                   |
|------------------|--------|-----------|--------------------------------------------|
| `force_percent`  | `__u8` | 0-100     | PWM duty cycle / intensity                |
| `frequency_hz`   | `__u16`| 200-4000  | Buzzer tone frequency                     |
| `duration_ms`    | `__u16`| 1-1000    | Pulse length                              |

**Entry points**, both accepting the same payload:

- `write(fd, &pulse, sizeof(pulse))` — fire-and-forget, no status returned.
- `ioctl(fd, HAPTIC_IOC_PULSE, &pulse)` — same effect, returns `0` on
  success or a negative `errno` (e.g. `-EINVAL` for out-of-range fields).

**Other ioctls:**

| Command                 | Direction | Argument | Effect                                  |
|--------------------------|-----------|----------|------------------------------------------|
| `HAPTIC_IOC_PULSE`       | write     | `struct haptic_pulse` | Start a pulse                |
| `HAPTIC_IOC_STOP`        | none      | —        | Immediately silence the buzzer           |
| `HAPTIC_IOC_GET_STATUS`  | read      | `__u8`   | `0` = idle, `1` = pulse in progress      |

**Caller responsibility:** mapping impact speed to `force_percent` /
`frequency_hz` is done entirely in user space (Task 3, on collision). The
driver never inspects physics state — it only turns a pulse request into
a PWM waveform via `hrtimer`.

## 2. `proximity_warning` (sysfs)

Virtual file at `/sys/kernel/pendulum/proximity_warning`, defined in
`include/proximity_warning.h`. Plain text, not binary, so any external
tool (`cat`, a shell script, a monitoring process) can read it directly,
per spec.

**Line format** (identical for write and read):

```
<DIRECTION> <RISK>\n
```

- `<DIRECTION>`: one of `NONE`, `NORTH`, `SOUTH`, `EAST`, `WEST`
- `<RISK>`: integer, `0` = `PROXIMITY_RISK_SAFE`, `1` = `PROXIMITY_RISK_WARNING`

Example: `EAST 1\n`, `NONE 0\n`.

**Direction of data flow:**

- **Write** (`echo "EAST 1" > proximity_warning`, done programmatically by
  Task 3): reports a state change. Task 3 writes once when the ball
  crosses the critical distance (exactly 1 LED cell from a wall) in a
  given direction, and once more with `NONE 0` when it is no longer
  critical. It does **not** write on every physics tick — only on
  transition — to avoid flooding the sysfs attribute.
- **Read** (`cat proximity_warning`, done by any external process): gets
  the last line written back verbatim. The driver stores and reflects
  the value; it does not interpret direction or risk itself.

**Extensibility:** risk values `2`/`3` are reserved for a future
distance-proportional warning level. Readers must not assume `0`/`1` are
the only possible values.

## 3. Design rationale

- Haptic feedback is **binary + ioctl** because it is a one-shot
  parametrized command (three small numeric fields) issued at a precise
  real-time moment (collision) — an ioctl/write round trip is cheap and
  well-suited to that.
- Proximity warning is **sysfs + text** because the spec requires it to
  be readable by arbitrary external processes without a private client
  library, and because it represents state (current threat direction/
  level) rather than a one-shot command — sysfs attributes are the
  idiomatic Linux mechanism for exposing kernel state as a file.
