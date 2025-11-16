#![no_std] // We are building for an environment without the standard library
#![no_main] // Disable the normal Rust runtime entry point

// --- CORE CRATES ---
use cortex_m_rt::entry; // The macro that defines the embedded entry point
use panic_halt as _; // Required: Halts the CPU on panic (instead of unwinding)
use cortex_m; // Access to core peripherals like the System Timer (SYST)

// --- HAL CRATES ---
use stm32f4xx_hal as hal;
use hal::{
    prelude::*,       // Imports essential extension traits (like `.constrain()`, `.split()`)
    gpio::{
        gpioa::PA5,   // Specific type for the LED pin on Port A, Pin 5
        Output,       
        PushPull,     
    },
    pac,              // Peripheral Access Crate (raw register access)
    rcc::RccExt,      // Extension trait for the Reset and Clock Control unit
};

// --- CRITICAL FIX: Frequency Literal Trait ---
// This is the correct import for stm32f4xx-hal v0.20.0 to enable the `.MHz()` method.
// It resolves the persistent "no method named mhz found" error.
use fugit::RateExtU32; 

// The main entry point for the application.
#[entry]
fn main() -> ! {
    // 1. Get Access to the Peripheral Access Crate (PAC) and Core Peripherals (CP)
    // These calls can only be done once, so we use `unwrap()`.
    let dp = pac::Peripherals::take().unwrap(); // Device Peripherals (GPIO, RCC, etc.)
    let cp = cortex_m::Peripherals::take().unwrap(); // Core Peripherals (SYST, SCB, etc.)
    
    // 2. Configure the Clocks (RCC)
    let rcc = dp.RCC.constrain();
    
    // Configure the main clock using the 8MHz High Speed External (HSE) crystal.
    // FIX: Uses the correct `.MHz()` (uppercase) syntax from the fugit::RateExtU32 trait.
    let clocks = rcc.cfgr.use_hse(8.MHz()).freeze(); 

    // 3. Configure a Delay/Timer (System Timer)
    // The delay provider is initialized using the configured clocks.
    let mut delay = cp.SYST.delay(&clocks); 

    // 4. Configure the GPIO Pin (PA5)
    // Split the GPIOA register into independent pins.
    let gpioa = dp.GPIOA.split(); 
    
    // Configure PA5 (LED LD2) and set it to a PushPull output mode.
    let mut led: PA5<Output<PushPull>> = gpioa.pa5.into_push_pull_output();

    // The main program loop
    loop {
        // Turn the LED on (set_low is usually 'on' for Nucleo boards)
        led.set_low(); 
        
        // Wait for 500ms
        delay.delay_ms(500_u32); 

        // Turn the LED off
        led.set_high(); 
        
        // Wait for 500ms
        delay.delay_ms(500_u32); 
    }
}