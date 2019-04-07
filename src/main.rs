mod cpu;
mod display;
mod gpu;
mod memory;

extern crate sdl2;

use std::fs::File;
use std::io::Read;

fn main() -> std::io::Result<()> {
    println!("Hello, world!");

    let mut buffer = Vec::new();
    let mut f = File::open("DMG_ROM.bin")?;
    f.read_to_end(&mut buffer)?;

    let display = display::Display::init();
    let mut gpu = gpu::GPU::new(display);
    gpu.do_thing();
    let mut bus = memory::MMU::new(&mut gpu);
    let mut cpu = cpu::CPU::new(&mut bus);

    cpu.load_rom(&buffer);
    cpu.step();

    Ok(())
}
