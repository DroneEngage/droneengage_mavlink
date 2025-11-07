# Cascade Chat Conversation

  Note: _This is purely the output of the chat conversation and does not contain any raw data, codebase snippets, etc. used to generate the output._

### User Input

@[/src/tracking/fcb_tracking_manager.cpp:L145] do we need expo when we have PID ? do you recommend keeping both ?

### Planner Response

## Short answer
- **You don’t need expo if your PID is well-tuned**, but a small amount of input shaping can be helpful in practice.
- **Expo and PID do different jobs**:
  - **PID**: closes the loop on error to drive it to zero.
  - **Expo**: non-linear mapping that reduces sensitivity near zero and increases it for large errors.

## When to keep both
- **Keep mild expo (or a soft deadband) before the PID** if:
  - You have noisy measurements/targets causing twitchiness near zero.
  - High actuator authority makes small errors over-correct.
  - You want smoother operator feel or camera tracking around center.
- If you keep expo, **tune the PID with expo enabled** because expo changes effective loop gain vs error magnitude.

## When to drop expo
- **Remove expo** for simpler, more predictable tuning if:
  - Sensors are clean and actuators are well-behaved.
  - You need consistent linear dynamics for controllers above/below this block.
  - Your current instability is due to extra nonlinearity complicating tuning.

## Implementation notes
- **Apply expo on the error going into the PID**, not on the PID output.
- Ensure the expo map is **monotonic** and does not make the slope too small near zero; otherwise:
  - Add **conditional integration** (integral off inside a small error band).
  - Use **anti-windup** and **derivative filtering**.
- Alternatives/adjuncts to expo:
  - Small **deadband** near zero.
  - **Setpoint/command slew-rate limiting**.
  - **Output slew limiting** if actuators need it.
  - **Low-pass filter** on the D-term or measured signal.

## Practical recommendation
- Start with **no expo**, tune PID to stable/acceptable behavior.
- If you still see jitter or overshoot near zero, add **light expo (10–20%) or a soft deadband**, then re-tune.
- Document the choice and keep only one nonlinearity where possible to keep tuning straightforward.

### Status
Answered design question and provided recommendation; no code changes made.