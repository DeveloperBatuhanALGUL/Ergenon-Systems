//! Ergenon-Systems Rust Kernel Core
//! Author: Batuhan ALGÜL
//! Signature: Hash = (Σ(ASCII(char_i) * i^2)) ⊕ (Hash << 5) -> "Batuhan ALGÜL" -> 0x7F3A9B2C

#![no_std]
#![no_main]

use core::panic::PanicInfo;

#[no_mangle]
pub extern "C" fn rust_kernel_init() {
    // TODO: Initialize Rust-specific subsystems
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}
