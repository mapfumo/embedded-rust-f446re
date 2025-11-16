//! # STM32F446RE Bare-Metal LED Blinky
//!
//! This example demonstrates the fundamentals of embedded Rust on the STM32F446RE:
//! - Peripheral initialization and ownership
//! - Clock configuration using external crystal (HSE)
//! - Type-safe GPIO control
//! - Blocking delays with SysTick timer
//!
//! ## Hardware
//! - Board: STM32 Nucleo-F446RE
//! - MCU: STM32F446RET6 (Cortex-M4F, 180 MHz max, 512KB Flash, 128KB RAM)
//! - LED: LD2 (Green) connected to PA5 (active-low)
//!
//! ## Memory Usage
//! - Flash: ~2.5 KB (of 512 KB available)
//! - RAM: Minimal (stack + static data only)

#![no_std]  // Exclude Rust standard library (not available in bare-metal)
#![no_main] // We define our own entry point via #[entry], not standard main()

// ============================================================================
// CORE EMBEDDED CRATES
// ============================================================================

/// Provides the #[entry] attribute macro that marks our main function as the
/// reset handler. This is called by the hardware after power-on or reset.
/// The cortex-m-rt crate also provides the vector table and startup code.
use cortex_m_rt::entry;

/// Defines panic behavior: halt the CPU in an infinite loop.
/// In production, consider using panic-probe or panic-semihosting for debugging.
/// The underscore import ensures the panic handler is linked even if not explicitly used.
use panic_halt as _;

/// Access to Cortex-M core peripherals (common across all ARM Cortex-M chips).
/// These include: SYST (SysTick timer), NVIC (interrupts), SCB (system control), etc.
use cortex_m;

// ============================================================================
// STM32F4 HARDWARE ABSTRACTION LAYER (HAL)
// ============================================================================

/// The HAL provides safe, high-level abstractions over raw register access.
/// It handles complex initialization sequences and provides type-safe APIs.
use stm32f4xx_hal as hal;

// The prelude imports extension traits that add methods to PAC types.
// Essential traits like .constrain(), .split(), .freeze() come from here.
use hal::prelude::*;

// GPIO types for type-safe pin configuration:
// - gpioa::PA5: Type-level encoding of the specific pin (PA5)
// - Output: Pin mode (Output vs Input vs Alternate function)
// - PushPull: Output configuration (PushPull can drive high/low, OpenDrain only pulls low)
use hal::gpio::{gpioa::PA5, Output, PushPull};

// PAC (Peripheral Access Crate) - auto-generated from STM32F446 SVD file.
// Provides direct register-level access. The HAL builds on top of this.
use hal::pac;

// Extension trait that adds .constrain() method to RCC peripheral.
// This converts the raw PAC type into a HAL type with safe configuration methods.
use hal::rcc::RccExt;

// ============================================================================
// TIME/FREQUENCY TRAIT
// ============================================================================

/// Provides .Hz(), .kHz(), and .MHz() extension methods for integers.
/// This is required by stm32f4xx-hal v0.20.0+ for type-safe frequency handling.
///
/// Example: 8.MHz() creates a fugit::Rate<u32, 1, 1_000_000> representing 8 MHz.
///
/// HISTORICAL NOTE: Earlier versions used lowercase .mhz() - this was changed in v0.20.0.
/// See NOTES.md for troubleshooting details if you encounter "no method named mhz" errors.
use fugit::RateExtU32;

// ============================================================================
// APPLICATION ENTRY POINT
// ============================================================================

/// Main application entry point.
///
/// The #[entry] macro marks this as the reset handler. After power-on or reset,
/// the hardware jumps here after the cortex-m-rt startup code finishes:
/// 1. Stack pointer is initialized (from memory.x)
/// 2. .data section is copied from Flash to RAM
/// 3. .bss section is zeroed
/// 4. Static constructors run (if any)
/// 5. This function is called
///
/// The `-> !` return type (the "never" type) indicates this function never returns.
/// Embedded programs run forever - there's no operating system to return to.
#[entry]
fn main() -> ! {
    // ========================================================================
    // STEP 1: ACQUIRE PERIPHERAL OWNERSHIP
    // ========================================================================
    
    // Take ownership of all device-specific peripherals (STM32F446RE).
    //
    // .take() returns Option<Peripherals> and can only succeed once per program.
    // This ensures exclusive access and prevents multiple parts of code from
    // simultaneously accessing the same hardware (which would cause race conditions).
    //
    // Includes: GPIO ports (A-H), RCC, USART, SPI, I2C, Timers, ADC, DMA, etc.
    //
    // IMPORTANT: If this unwrap() panics, it means Peripherals::take() was
    // called twice, which should never happen in correct code.
    let dp = pac::Peripherals::take().unwrap();
    
    // Take ownership of Cortex-M core peripherals (shared across all Cortex-M chips).
    //
    // Includes:
    // - SYST: 24-bit SysTick timer (we use this for delays)
    // - NVIC: Nested Vectored Interrupt Controller
    // - SCB: System Control Block
    // - CPUID: CPU identification registers
    // - etc.
    //
    // These are separate from device peripherals because they're defined by ARM,
    // not by ST Microelectronics.
    let cp = cortex_m::Peripherals::take().unwrap();
    
    // ========================================================================
    // STEP 2: CONFIGURE SYSTEM CLOCKS
    // ========================================================================
    
    // Convert raw RCC (Reset and Clock Control) peripheral to HAL type.
    //
    // .constrain() consumes the PAC type and returns a HAL type with
    // safe configuration methods. This prevents accidentally misconfiguring
    // the clock tree which could brick the chip.
    let rcc = dp.RCC.constrain();
    
    // Configure and freeze the clock tree.
    //
    // Clock Source Selection:
    // - HSI: 16 MHz internal RC oscillator (default, less accurate, ±1% tolerance)
    // - HSE: 8 MHz external crystal on Nucleo board (±50 ppm, much more stable)
    // - PLL: Can multiply HSI or HSE to higher frequencies (up to 180 MHz for F446)
    //
    // Why HSE?
    // - UART: Requires accurate baud rate timing (HSI drift causes errors)
    // - USB: Requires very precise 48 MHz clock (impossible with HSI)
    // - Timers: More predictable timing for measurements
    //
    // Clock Tree (simplified for this example):
    //   HSE (8 MHz) → [no PLL] → SYSCLK (8 MHz) → AHB (8 MHz) → APB1/APB2
    //                                           ↓
    //                                       SysTick
    //
    // .freeze() locks the configuration and returns a Clocks struct containing
    // the actual clock frequencies. This is used by peripherals to calculate
    // timing parameters (like baud rates, PWM frequencies, etc.).
    //
    // FUTURE EXPANSION: To run at maximum speed (180 MHz):
    // ```rust
    // let clocks = rcc.cfgr
    //     .use_hse(8.MHz())
    //     .sysclk(180.MHz())  // Uses PLL automatically
    //     .freeze();
    // ```
    let clocks = rcc.cfgr.use_hse(8.MHz()).freeze();

    // ========================================================================
    // STEP 3: CONFIGURE DELAY PROVIDER (SYSTICK TIMER)
    // ========================================================================
    
    // Create a blocking delay provider using the SysTick timer.
    //
    // SysTick is a simple 24-bit countdown timer built into every Cortex-M core:
    // - Decrements at system clock frequency (8 MHz in our case)
    // - Generates an exception when it reaches zero
    // - Used here for simple blocking delays
    //
    // How .delay_ms() works:
    // 1. Calculate required ticks: ticks = (milliseconds * clock_freq) / 1000
    // 2. Load value into SysTick reload register
    // 3. Enable timer and busy-wait until countdown completes
    //
    // LIMITATIONS:
    // - Blocking: CPU does nothing during delay (wastes power and cycles)
    // - Maximum delay: 2^24 ticks / clock_freq (2.1 seconds at 8 MHz)
    //
    // ALTERNATIVES FOR NON-BLOCKING CODE:
    // - Hardware timers (TIM2, TIM3, etc.) with interrupts
    // - RTIC framework for interrupt-driven concurrency
    // - embassy-rs for async/await embedded Rust
    let mut delay = cp.SYST.delay(&clocks);

    // ========================================================================
    // STEP 4: CONFIGURE GPIO PIN (PA5 FOR LED)
    // ========================================================================
    
    // Split the GPIOA peripheral into individual pin objects.
    //
    // After .split(), each pin (PA0-PA15) becomes an independent object with
    // its own type. This prevents accidentally using the wrong pin at compile time.
    //
    // The original `dp.GPIOA` is consumed and can't be used again - this ensures
    // exclusive access to each pin.
    let gpioa = dp.GPIOA.split();
    
    // Configure PA5 as a push-pull output.
    //
    // Type signature breakdown:
    // - PA5: This is specifically pin 5 of port A (not PA4, not PB5)
    // - Output: Pin is in output mode (not Input or Alternate function)
    // - PushPull: Can drive both high (3.3V) and low (0V) actively
    //
    // Alternative: Output<OpenDrain>
    // - Can only pull low, uses external pull-up for high
    // - Used for I2C, one-wire protocols, etc.
    //
    // Pin States After Configuration:
    // - Speed: Default is low speed (2 MHz) - fine for LED blinking
    // - Pull: No pull-up/pull-down (not needed for output)
    // - Initial state: Undefined (we'll set it explicitly in the loop)
    //
    // HARDWARE DETAILS (Nucleo-F446RE):
    // - LD2 (green LED) is connected: PA5 → 470Ω resistor → LED anode → GND
    // - Therefore: LOW = LED on, HIGH = LED off (active-low)
    //
    // Type Safety Example:
    // ```rust
    // let wrong_pin = gpioa.pa4.into_push_pull_output(); // Different type!
    // // led = wrong_pin; // ← Compile error! Types don't match.
    // ```
    let mut led: PA5<Output<PushPull>> = gpioa.pa5.into_push_pull_output();

    // ========================================================================
    // STEP 5: MAIN APPLICATION LOOP
    // ========================================================================
    
    // Infinite loop - embedded systems run forever.
    //
    // Unlike desktop applications that start, run, and exit, embedded programs
    // run continuously until power is removed. The `loop` keyword creates an
    // infinite loop, and the function's `-> !` return type tells Rust that
    // this function never returns.
    //
    // POWER CONSUMPTION NOTE:
    // This implementation uses busy-waiting (CPU spins during delays), consuming
    // full power even when doing nothing. For battery-powered projects, consider:
    // - WFI (Wait For Interrupt) instruction
    // - Low-power timer modes
    // - Deep sleep modes (STOP, STANDBY)
    loop {
        // LED ON: Drive pin LOW (0V)
        //
        // On Nucleo boards, LD2 is typically wired as active-low:
        //   PA5 LOW (0V) → Current flows through LED → LED lights up
        //   PA5 HIGH (3.3V) → No voltage difference → LED off
        //
        // This is opposite of typical Arduino-style boards where HIGH = LED on.
        //
        // HARDWARE: Setting the pin low writes 0 to the corresponding bit
        // in the GPIOA_ODR (Output Data Register) at address 0x4002_0014.
        led.set_low();
        
        // Wait 500 milliseconds (0.5 seconds)
        //
        // The _u32 suffix explicitly types the literal as u32, which matches
        // the delay_ms() function signature. Without it, Rust would infer the
        // type, but being explicit prevents potential confusion.
        //
        // Actual wait time: 500ms ± clock accuracy (very precise with HSE)
        delay.delay_ms(500_u32);

        // LED OFF: Drive pin HIGH (3.3V)
        //
        // HARDWARE: Setting the pin high writes 1 to bit 5 of GPIOA_ODR.
        // The STM32 hardware atomically updates only this bit, leaving
        // other pins unchanged (this is handled by the HAL internally using
        // the BSRR - Bit Set/Reset Register for atomic operations).
        led.set_high();
        
        // Wait another 500 milliseconds
        //
        // Total period: 1000ms (1 Hz blink rate)
        // Duty cycle: 50% (equal on/off time)
        //
        // TO EXPERIMENT:
        // - Change to 100ms for faster blinking
        // - Use different values for asymmetric blinking (e.g., 100ms on, 900ms off)
        // - Calculate exact timing: For 2 Hz (0.5s period), use 250ms delays
        delay.delay_ms(500_u32);
        
        // Loop continues indefinitely...
        //
        // MEMORY NOTE: This loop uses zero heap allocation. The led and delay
        // objects live on the stack and are reused every iteration. Total RAM
        // usage is minimal (< 100 bytes for this entire program).
    }
    
    // This point is never reached - the function returns the never type (!)
}

// ============================================================================
// FUTURE ENHANCEMENTS TO TRY
// ============================================================================
//
// 1. BUTTON INPUT (PC13 - Blue button on Nucleo):
//    ```rust
//    let gpioc = dp.GPIOC.split();
//    let button = gpioc.pc13.into_pull_down_input();
//    if button.is_high() { /* button pressed */ }
//    ```
//
// 2. PWM BREATHING LED (Smooth fade in/out):
//    ```rust
//    let channels = (gpioa.pa5.into_alternate(),);
//    let pwm = dp.TIM2.pwm_hz(channels, 1.kHz(), &clocks);
//    pwm.set_duty(Channel1, pwm.get_max_duty() / 2); // 50% brightness
//    ```
//
// 3. UART OUTPUT (Serial communication):
//    ```rust
//    let tx_pin = gpioa.pa2.into_alternate();
//    let rx_pin = gpioa.pa3.into_alternate();
//    let serial = dp.USART2.serial((tx_pin, rx_pin), 115200.bps(), &clocks);
//    writeln!(serial, "Hello from Rust!").unwrap();
//    ```
//
// 4. EXTERNAL INTERRUPT (React to button without polling):
//    ```rust
//    button.make_interrupt_source(&mut syscfg);
//    button.enable_interrupt(&mut exti);
//    button.trigger_on_edge(&mut exti, Edge::Rising);
//    // Then implement EXTI15_10 interrupt handler
//    ```
//
// 5. RTIC (Real-Time Interrupt-driven Concurrency):
//    - Framework for building interrupt-driven embedded applications
//    - Provides safe shared resource access
//    - Task scheduling based on priorities
//    - See: https://rtic.rs
//
// ============================================================================
