# 🦀 Nucleo-F446RE Blinky (Rust)

A bare-metal Rust implementation of the classic LED blinky example for the **STM32F446RE** microcontroller on the **STM32 Nucleo-F446RE** development board.

This project demonstrates modern embedded Rust development using the official HAL, with proper clock configuration, delay timing, and GPIO control—all in a safe, `no_std` environment.

---

## ✨ Features

- **Bare-metal `no_std` Rust** — No operating system overhead
- **Type-safe HAL** — Uses `stm32f4xx-hal` with the `stm32f446` feature
- **External oscillator** — Configures HSE at 8 MHz for stable timing
- **LED control** — Blinks the onboard user LED (LD2) on **PA5**
- **SysTick delays** — Uses Cortex-M core timer for precise timing
- **Complete linker setup** — Includes `memory.x` with full memory map
- **One-command flashing** — Deploy with `cargo run --release` via probe-rs

---

## 🏗️ Architecture Overview

### HAL Abstraction Layers

```
┌──────────────────────────────────┐
│   Your Application (main.rs)    │  ← High-level Rust code
├──────────────────────────────────┤
│   HAL (stm32f4xx-hal)            │  ← Safe abstractions
│   • GPIO, RCC, USART, I2C, etc.  │     (OutputPin, delay, etc.)
├──────────────────────────────────┤
│   PAC (stm32f4 crate)            │  ← Register-level access
│   • Auto-generated from SVD      │     (type-safe register writes)
├──────────────────────────────────┤
│   Cortex-M Core (cortex-m)       │  ← CPU core peripherals
│   • SYST, NVIC, SCB, etc.        │     (common to all Cortex-M)
└──────────────────────────────────┘
```

**Why this matters:**

- **PAC** provides raw register access (unsafe but complete)
- **HAL** provides safe, ergonomic APIs on top of PAC
- **Cortex-M** handles CPU core features common to all ARM chips

---

## 🔌 Hardware Pinout

### Nucleo-F446RE User LED

```
  ┌─────────────────────────────┐
  │   NUCLEO-F446RE             │
  │                             │
  │   User LED (LD2 - Green)    │
  │   ┌──────┐                  │
  │   │ PA5  │─────● LD2        │
  │   └──────┘                  │
  │                             │
  │   [STM32F446RE MCU]         │
  └─────────────────────────────┘
```

**Pin Configuration:**

- **PA5**: Configured as push-pull output
- **LD2**: Green LED (active **low** on Nucleo boards)
- Located next to the USB connector

**Note**: On Nucleo boards, the LED is typically active-low (LOW = on, HIGH = off).

---

## 📁 Project Structure

```
nucleo-f446re-blinky/
├── Cargo.toml           # Dependencies and build config
├── memory.x             # MCU memory layout (Flash/RAM)
├── .cargo/
│   └── config.toml      # Target and runner config
├── src/
│   └── main.rs          # Application code
├── README.md            # This file
├── NOTES.md             # Troubleshooting guide
└── LICENSE              # MIT License
```

---

## 🚀 Quick Start

### Prerequisites

1. **Install Rust toolchain:**

   ```bash
   curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
   ```

2. **Add ARM Cortex-M4F target:**

   ```bash
   rustup target add thumbv7em-none-eabihf
   ```

3. **Install probe-rs (for flashing):**
   ```bash
   cargo install probe-rs-tools --locked
   ```

### Building & Flashing

**Option 1: Flash and run (recommended)**

```bash
cargo run --release
```

This builds, flashes, and resets the board automatically.

**Option 2: Build only**

```bash
cargo build --release
```

**Option 3: Flash manually**

```bash
probe-rs run --chip STM32F446RETx target/thumbv7em-none-eabihf/release/nucleo-f446re-blinky
```

### Expected Behavior

After flashing, the green LED (LD2) should blink on and off with a ~1 second period (500ms on, 500ms off).

---

## ⚙️ Configuration Files

### `.cargo/config.toml`

```toml
[build]
# Default to ARM Cortex-M4F with hardware FPU
target = "thumbv7em-none-eabihf"

[target.thumbv7em-none-eabihf]
# Automatically flash and run with probe-rs
runner = "probe-rs run --chip STM32F446RETx"

# Linker configuration
rustflags = [
  "-C", "link-arg=-Tlink.x",  # Use cortex-m-rt linker script
]
```

### `memory.x` — STM32F446RE Memory Map

```ld
MEMORY
{
  /* 512 KB Flash (program code) */
  FLASH : ORIGIN = 0x08000000, LENGTH = 512K

  /* 128 KB SRAM (variables, stack, heap) */
  RAM : ORIGIN = 0x20000000, LENGTH = 128K

  /* 64 KB Core-Coupled Memory (fast, no DMA) */
  CCMRAM : ORIGIN = 0x10000000, LENGTH = 64K
}
```

**Memory Details:**

- **Flash (0x0800_0000)**: Read-only program storage, survives power cycles
- **RAM (0x2000_0000)**: Fast read/write memory for runtime data
- **CCMRAM (0x1000_0000)**: Ultra-fast RAM, ideal for stack or buffers (but no DMA access)

---

## 📊 Key Dependencies

| Crate           | Version | Purpose                            |
| --------------- | ------- | ---------------------------------- |
| `cortex-m`      | 0.7.5   | ARM Cortex-M core support          |
| `cortex-m-rt`   | 0.7.5   | Runtime & startup code             |
| `stm32f4xx-hal` | 0.20.0  | STM32F4 Hardware Abstraction Layer |
| `embedded-hal`  | 0.2.7   | Common embedded traits             |
| `fugit`         | 0.3.7   | Type-safe time/frequency types     |
| `panic-halt`    | 0.2.0   | Halt CPU on panic                  |

---

## 🐛 Troubleshooting

### LED doesn't blink

1. **Check power**: Ensure USB cable is connected
2. **Check upload**: Look for "Finished flashing" message
3. **Try reset**: Press the black reset button on the board
4. **LED polarity**: On Nucleo boards, `set_low()` typically turns the LED **on**

### Compilation errors

See **[NOTES.md](NOTES.md)** for detailed solutions to common issues:

- Missing `Peripherals` import
- `.mhz()` vs `.MHz()` confusion (fugit trait issue)
- Missing `memory.x` file
- Panic handler configuration

### Upload fails

```bash
# Try with explicit chip name and reset
probe-rs run --chip STM32F446RETx --connect-under-reset target/thumbv7em-none-eabihf/release/nucleo-f446re-blinky
```

### Check probe-rs can see your board

```bash
probe-rs list
```

Should show your ST-Link debugger.

---

## 🔬 Code Walkthrough

### Complete Application Code

```rust
#![no_std]  // No standard library (embedded environment)
#![no_main] // No standard entry point

// --- CORE CRATES ---
use cortex_m_rt::entry;  // Embedded entry point macro
use panic_halt as _;     // Halt CPU on panic
use cortex_m;            // Core peripherals (SYST, etc.)

// --- HAL CRATES ---
use stm32f4xx_hal as hal;
use hal::{
    prelude::*,          // Extension traits
    gpio::{
        gpioa::PA5,      // Type for LED pin
        Output,
        PushPull,
    },
    pac,                 // Peripheral Access Crate
    rcc::RccExt,         // Clock control trait
};

// --- CRITICAL: Frequency Literal Trait ---
// Enables .MHz() method for clock configuration
use fugit::RateExtU32;

#[entry]
fn main() -> ! {
    // 1. Take ownership of peripherals (can only be done once)
    let dp = pac::Peripherals::take().unwrap();      // Device peripherals
    let cp = cortex_m::Peripherals::take().unwrap(); // Core peripherals

    // 2. Configure system clocks
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.use_hse(8.MHz()).freeze();

    // 3. Setup delay provider using SysTick
    let mut delay = cp.SYST.delay(&clocks);

    // 4. Configure GPIO PA5 as push-pull output
    let gpioa = dp.GPIOA.split();
    let mut led: PA5<Output<PushPull>> = gpioa.pa5.into_push_pull_output();

    // 5. Main blink loop
    loop {
        led.set_low();              // LED ON (active-low)
        delay.delay_ms(500_u32);    // Wait 500ms
        led.set_high();             // LED OFF
        delay.delay_ms(500_u32);    // Wait 500ms
    }
}
```

### Key Implementation Details

#### 1. **Peripheral Ownership**

```rust
let dp = pac::Peripherals::take().unwrap();
let cp = cortex_m::Peripherals::take().unwrap();
```

- `take()` returns `Option<Peripherals>` and can only succeed once
- Ensures exclusive access to hardware (prevents conflicts)
- **dp**: Device-specific (GPIO, RCC, USART, etc.)
- **cp**: Core Cortex-M (SYST, NVIC, SCB, etc.)

#### 2. **Clock Configuration**

```rust
let clocks = rcc.cfgr.use_hse(8.MHz()).freeze();
```

- Uses 8 MHz external crystal (HSE) instead of internal RC oscillator
- `.freeze()` locks the clock configuration (prevents accidental changes)
- **Why HSE?** More accurate and stable than HSI (important for UART, USB, etc.)

#### 3. **Type-Safe GPIO**

```rust
let mut led: PA5<Output<PushPull>> = gpioa.pa5.into_push_pull_output();
```

- **PA5**: Specific pin type (compile-time safety)
- **Output\<PushPull\>**: Pin mode encoded in the type system
- Can't accidentally read from an output pin or write to an input

#### 4. **Fugit Time Types**

```rust
use fugit::RateExtU32;  // Enables .MHz() method
```

- `stm32f4xx-hal` v0.20.0 uses `fugit` for type-safe time/frequency
- `.MHz()`, `.kHz()`, `.Hz()` methods ensure correct units
- Compile-time prevention of timing errors

#### 5. **Active-Low LED Logic**

```rust
led.set_low();   // Turns LED ON
led.set_high();  // Turns LED OFF
```

- Nucleo boards typically use active-low LEDs
- LED cathode connected to PA5, anode to VCC
- Logic inverted compared to typical Arduino-style boards

---

## 🎓 Understanding the Code

### Why `no_std` and `no_main`?

```rust
#![no_std]   // No standard library
#![no_main]  // No standard entry point
```

**Embedded systems don't have:**

- File systems
- Dynamic memory allocation (by default)
- Operating system services
- Standard `main()` with command-line arguments

Instead, we use:

- `#[entry]` macro from `cortex-m-rt` for the entry point
- Custom panic handlers (we use `panic-halt`)
- Fixed memory layout defined in `memory.x`

### Clock Tree Simplified

```
HSE (8 MHz)  ──→  PLL (optional)  ──→  AHB Bus  ──→  System Clock
                                          │
                                          ├──→ APB1 (peripherals)
                                          ├──→ APB2 (peripherals)
                                          └──→ SysTick Timer
```

Our configuration:

- Uses HSE directly (no PLL multiplication in this simple example)
- System clock = 8 MHz
- All peripherals and timers derive from this

### Delay Implementation

```rust
let mut delay = cp.SYST.delay(&clocks);
delay.delay_ms(500_u32);
```

**How it works:**

1. SysTick is a 24-bit countdown timer in the Cortex-M core
2. It decrements at the system clock frequency
3. `delay_ms()` calculates required ticks and busy-waits
4. **Note**: This is blocking (CPU does nothing during delay)

**For non-blocking alternatives, consider:**

- Using hardware timers (TIM2, TIM3, etc.)
- Implementing a simple task scheduler
- Using RTIC (Real-Time Interrupt-driven Concurrency)

---

## 📚 Learning Resources

### Embedded Rust

- [The Embedded Rust Book](https://docs.rust-embedded.org/book/) — Comprehensive guide
- [Discovery Book](https://docs.rust-embedded.org/discovery/) — Hands-on tutorials
- [Awesome Embedded Rust](https://github.com/rust-embedded/awesome-embedded-rust) — Curated list

### STM32 Documentation

- [STM32F446RE Datasheet](https://www.st.com/resource/en/datasheet/stm32f446re.pdf) — Pin assignments, memory map
- [Reference Manual (RM0390)](https://www.st.com/resource/en/reference_manual/rm0390-stm32f446xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf) — Complete peripheral descriptions
- [Nucleo Board User Manual](https://www.st.com/resource/en/user_manual/um1724-stm32-nucleo64-boards-mb1136-stmicroelectronics.pdf) — Board schematics and features

### Crate Documentation

- [stm32f4xx-hal docs](https://docs.rs/stm32f4xx-hal/)
- [cortex-m docs](https://docs.rs/cortex-m/)
- [embedded-hal docs](https://docs.rs/embedded-hal/)
- [fugit docs](https://docs.rs/fugit/) — Understanding time types

---

## 🎯 Next Steps

### Beginner Level

1. **Button Input** — Use the blue user button (PC13) to toggle the LED
2. **Multiple LEDs** — Control several external LEDs on different pins
3. **LED Patterns** — Create more complex blinking sequences

### Intermediate Level

4. **UART Communication** — Send "Hello, World!" over serial (USART2)
5. **PWM Breathing LED** — Smoothly fade the LED brightness using timers
6. **ADC Reading** — Read analog voltage from a potentiometer
7. **External Interrupts** — React to button presses without polling

### Advanced Level

8. **I2C Sensors** — Communicate with temperature/humidity sensors
9. **SPI Displays** — Drive an OLED or LCD display
10. **DMA Transfers** — Offload data transfers from the CPU
11. **USB Device** — Implement USB CDC (virtual serial port)
12. **RTIC Framework** — Build a real-time interrupt-driven application

Check out the [stm32f4xx-hal examples](https://github.com/stm32-rs/stm32f4xx-hal/tree/master/examples) for inspiration!

---

## 🔧 Binary Size Optimization

Current build settings already optimize for size:

```toml
[profile.release]
codegen-units = 1
lto = "fat"           # Link-time optimization
opt-level = 3         # Maximum optimization
```

**Check your binary size:**

```bash
cargo size --release -- -A
```

**Typical output:**

```
section               size        addr
.vector_table          392   0x8000000
.text                 2048   0x8000188
.rodata                 16   0x8000988
.data                    0  0x20000000
.bss                     4  0x20000000
```

**Total flash usage: ~2.5 KB** (out of 512 KB available)

---

## 📄 License

This project is licensed under the **MIT License**

---

## 🙏 Acknowledgments

- **[stm32-rs](https://github.com/stm32-rs)** — Excellent HAL and PAC implementations
- **[Rust Embedded Working Group](https://github.com/rust-embedded)** — Making embedded Rust possible
- **STMicroelectronics** — For the fantastic Nucleo development boards
- **Community** — All the helpful folks in the embedded Rust community

---

**Happy Embedded Hacking! 🎉**
