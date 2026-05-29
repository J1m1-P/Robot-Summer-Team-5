# Analog Read: Polling Loop vs. ISR-Timed Sampling

## The Original Method (Tight Loop)

A naive approach to collecting ADC samples at a target rate looks like this:

```cpp
for (int i = 0; i < N; i++) {
    buf[i] = (int16_t)adc1_get_raw(ADC_CH);
    delayMicroseconds(PERIOD_US); // attempt to pace at 50 kHz
}
```

**The problem:** the actual sample period is not `PERIOD_US` — it's:

```
actual_period = adc_conversion_time + loop_overhead + delayMicroseconds(PERIOD_US)
```

Each of these terms is variable:

| Source of error | Typical magnitude |
|---|---|
| `adc1_get_raw()` conversion time | ~4–10 µs, non-deterministic |
| Loop bookkeeping (increment, branch) | ~100–200 ns |
| Other ISRs stealing the CPU (WiFi, SysTick, etc.) | 0 µs to hundreds of µs |

At a target rate of **50 kHz** the ideal period is **20 µs**. If the ADC takes 8 µs and you only `delayMicroseconds(12)`, the real sample rate is whatever the CPU gets around to — not 50 kHz. Worse, it varies sample-to-sample, so the time axis of your buffer is warped.

**Impact on zero-crossing detection:** `zeroCrossingFreq()` divides crossing count by `gWindowSec`. If the actual sample intervals are uneven, the window time is still measured correctly by `micros()`, but the individual crossing timestamps are wrong — samples that *should* be 20 µs apart might be 14 µs or 28 µs apart, smearing your frequency estimate.

---

## The ISR-Timed Method (Current Code)

```cpp
void IRAM_ATTR onTimer() { gTick = true; }   // fires every PERIOD_US

for (int i = 0; i < N; i++) {
    while (!gTick);          // wait for the hardware timer tick
    gTick = false;
    buf[i] = (int16_t)adc1_get_raw(ADC_CH);
}
```

The **hardware timer** is the timing authority — it fires at a crystal-accurate 50 kHz regardless of what the CPU is doing. The main loop just waits for the flag and then samples.

**Why this is better:**

- The *gap between samples* is defined by the timer period, not by how long `adc1_get_raw()` takes.
- Jitter is now only the polling latency — the few instructions between `gTick = true` and the actual ADC call. That's tens of nanoseconds, not microseconds.
- `gWindowSec` (measured with `micros()`) captures the true elapsed wall time over all N samples, giving the denominator in `zeroCrossingFreq()` an accurate value even if there is any residual drift.

**Residual limitation:** `adc1_get_raw()` is still called from the main loop, not from inside the ISR. If the system is under heavy interrupt load (e.g., BLE advertising events) the `while (!gTick)` can miss a tick entirely — the flag is set, cleared, and set again before the loop sees it. For this application (classifying 1 kHz vs. 10 kHz) the margin is wide enough that occasional missed ticks are harmless, but for precision measurement you would move the ADC call into the ISR and use a ring buffer.

---

## Side-by-side Summary

| Property | Tight loop | ISR-timed (current) |
|---|---|---|
| Timing authority | Software delay | Hardware timer |
| Sample jitter | ~µs range | ~ns range |
| Sensitivity to other ISRs | High — delays every sample | Low — only affects post-tick latency |
| ADC conversion time affects period | Yes | No |
| `gWindowSec` needed for correction | Yes (essential) | Yes (belt-and-suspenders) |
| Missed-tick risk | N/A | Low but non-zero |
