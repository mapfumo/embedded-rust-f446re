# Development Notes --- Compilation Issues & Fixes

This document chronicles the troubleshooting journey of bringing up Rust
on the STM32F446RE for a bare‑metal blinky project.

---

## 1. Missing Peripherals Import

**Issue:**\
`error[E0405]: cannot find trait 'Peripherals'`

**Cause:**\
`pac::Peripherals` wasn't imported.

**Fix:**

```rust
use stm32f4xx_hal::pac;
```

---

## 2. Missing Core Peripherals (SYST)

**Issue:**\
`error[E0425]: cannot find value 'SYST'`

**Cause:**\
SYST is a **core** peripheral---must use `cortex_m::Peripherals`.

**Fix:**

```rust
let cp = cortex_m::Peripherals::take().unwrap();
let mut delay = cp.SYST.delay(&clocks);
```

---

## 3. `.mhz()` Frequency Literal Error

**Issue:**\
`no method named 'mhz' found`

**Cause:**\
`stm32f4xx-hal v0.20.0` uses **fugit**. Correct trait is `RateExtU32`
and method is **MHz()**.

**Fix:**

```rust
use fugit::RateExtU32;
rcc.cfgr.use_hse(8.MHz()).freeze();
```

---

## 4. Panic Unwinding Not Supported

**Issue:**\
`unwinding panics are not supported without std`

**Fix:**\
Add panic abort:

    -C panic=abort

---

## 5. Missing memory.x

**Issue:**\
`rust-lld: cannot find linker script memory.x`

**Fix:**\
Provide STM32F446 memory layout.

    FLASH  : ORIGIN = 0x08000000, LENGTH = 512K
    RAM    : ORIGIN = 0x20000000, LENGTH = 128K
    CCMRAM : ORIGIN = 0x10000000, LENGTH = 64K

---
