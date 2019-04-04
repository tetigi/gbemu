mod cpu;
mod memory;

use std::fs::File;
use std::io::Read;

fn main() -> std::io::Result<()> {
    println!("Hello, world!");

    let mut buffer = Vec::new();
    let mut f = File::open("DMG_ROM.bin")?;
    f.read_to_end(&mut buffer)?;

    let mut cpu = cpu::CPU::new();

    cpu.load_rom(&buffer);

    for _ in 0..90000000 {
        cpu.step();

        match cpu.pc {
            0x7 => println!("Setup"),
            0x27 => println!("Video setup"),
            0x39 => println!("Tilemap setup"),
            0x48 => println!("Some load 1"),
            0x4A => println!("pre-logo"),
            0x55 => println!("Scroll start.."),
            0x64 => println!("wait for screen"),
            0x80 => println!("Play sound!"),
            0x86 => println!("Scroll up?"),
            0xA8 => println!("Nintendo logo"),
            _ => (),
        };
        //dbg!(&cpu);
    }

    Ok(())
}
