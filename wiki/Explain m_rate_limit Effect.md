## Effect of `m_rate_limit`

`m_rate_limit` is used in the *first* shaping stage as a **time‑based rate limiter / outlier rejector** on the normalized tracking error:

```cpp
// comment in code:
 // m_rate_limit is interpreted as normalized units per second
auto clampStepDt = [this, dt](double prev, double cur) {
  if (dt <= 0.0)
    return cur; // first sample after init
  const double max_step = m_rate_limit * dt;
  const double step = cur - prev;
  if (std::abs(step) > max_step) {
    return prev + std::copysign(max_step, step);
  }
  return cur;
};
double sx = clampStepDt(m_prev_dx, processed_x);
double sy = clampStepDt(m_prev_dy, processed_yz);
```

- **Input space**: `processed_x` and `processed_yz` are normalized tracking errors (later clamped to `[-0.5, 0.5]`).
- **Meaning**:  
  - `m_rate_limit` = *max allowed change per second* in these normalized units.
  - On each frame, the change allowed is `max_step = m_rate_limit * dt`.
  - If the new error jumps more than `max_step` away from the previous one, it’s *clipped* to be exactly `max_step` away instead.
- **Practical effect**:
  - **Lower `m_rate_limit`** → stronger filtering of sudden jumps, more smoothing, slower response, more lag.
  - **Higher `m_rate_limit`** → weaker filtering, faster response, but more sensitive to noise/spikes.

There is also a separate **input slew limiter** later:

```cpp
const double max_input_step = 0.3 * dt; // max 0.3 units per second
...
sx = clampInputStep(prev_input_x, sx);
sy = clampInputStep(prev_input_yz, sy);
```

So:
- The first limiter uses **`m_rate_limit`** (configurable, adaptive).
- The second uses a fixed `0.3 units/sec`, to protect the PID input.

## Interaction with Kalman filter

If Kalman is enabled and has a velocity estimate, `m_rate_limit` is increased adaptively:

```cpp
if (std::abs(vel_x) > 0.01) {
  m_rate_limit = std::min(m_rate_limit * 1.5, 0.15);
}
if (std::abs(vel_yz) > 0.01) {
  m_rate_limit = std::min(m_rate_limit * 1.5, 0.15);
}
```

- When target appears to **move** (`|vel| > 0.01`), `m_rate_limit` is multiplied by `1.5`, up to a **hard cap of `0.15`**.
- Interpretation:
  - When object is static → keep rate low → smooth & stable.
  - When object is moving → allow **faster response** (higher `m_rate_limit`) up to `0.15 units/sec`.

So internally, **effective maximum** is `0.15` even if you configure a higher number.

## Range of reasonable values

Inputs `sx`, `sy` are in `[-0.5, 0.5]`, so a full‑scale change is `1.0` unit.

- If `m_rate_limit = 0.1`:
  - Max change ≈ `0.1` per second.  
  - It takes about 5 s to go from `-0.5` to `+0.5`.
- If `m_rate_limit = 0.02`:
  - Very smooth; ~25 s for full change → strong lag.
- With adaptive bump, if movement is detected, `m_rate_limit` can rise to `0.15`:
  - ~3.3 s for full swing.

Given the internal cap at `0.15`, **useful config range** is roughly:

- **0.01 – 0.15** (normalized units / second)

Heuristics:
- **0.01 – 0.03**: very smooth, good for jittery / noisy detections, but sluggish.
- **0.05 – 0.1**: reasonable compromise (good starting range).
- **> 0.1 up to 0.15**: snappier tracking, but lets more noise through.

Values above `0.15` are effectively clamped once motion is detected, so they mainly matter only before Kalman bumps it.

---

### Summary

- **What it does**: `m_rate_limit` limits how fast the *normalized tracking error* is allowed to change per second, acting as a time-based outlier / slew filter *before* deadband, expo, and PID.
- **Units**: normalized units per second in the same space as `sx/sy` (`[-0.5, 0.5]`).
- **Typical range**: configure in about **0.01–0.1**, with the code itself capping the adaptive value at **0.15**. Lower = smoother/slower, higher = faster/more jittery.

If you tell me your current `rate_limit` config value and the update rate (`dt`), I can recommend a more specific number.