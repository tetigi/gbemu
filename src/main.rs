mod cpu;

use std::fs::File;
use std::io::Read;

fn main() -> std::io::Result<()> {
    println!("Hello, world!");

    let mut buffer = Vec::new();
    let mut f = File::open("DMG_ROM.bin")?;
    f.read_to_end(&mut buffer)?;

    let mut cpu = cpu::CPU::new();

    cpu.load_rom(&buffer);

    for _ in 0..20 {
        cpu.step();
        dbg!(&cpu);
    }

    Ok(())
}
