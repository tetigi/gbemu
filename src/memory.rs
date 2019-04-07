use crate::gpu;
use std::rc::Rc;

pub struct MemoryLocation {
    pub cursor: u16,
    memory: Rc<[u8]>,
}

impl MemoryLocation {
    fn new(start_addr: u16, memory: Rc<[u8]>) -> MemoryLocation {
        MemoryLocation {
            cursor: start_addr,
            memory,
        }
    }
}

impl Iterator for MemoryLocation {
    type Item = (u16, u8);

    fn next(&mut self) -> Option<Self::Item> {
        let result = (self.cursor, self.memory[self.cursor as usize]);
        self.cursor = self.cursor.wrapping_add(1);

        Some(result)
    }
}

pub struct MMU<'a> {
    memory: Rc<[u8]>,
    gpu: &'a mut gpu::GPU,
}

pub trait MemoryBus {
    fn read_byte(&self, address: u16) -> u8;
    fn read_bytes_from(&self, address: u16) -> MemoryLocation;
    fn write_byte(&mut self, address: u16, value: u8);
}

impl<'a> MMU<'a> {
    pub fn new<'n>(gpu: &'n mut gpu::GPU) -> MMU<'n> {
        MMU {
            memory: Rc::new([0; 65_536]),
            gpu,
        }
    }
}

impl<'a> MemoryBus for MMU<'a> {
    fn read_byte(&self, address: u16) -> u8 {
        let address = address as usize;
        match address {
            gpu::VRAM_BEGIN...gpu::VRAM_END => self.gpu.read_vram(address - gpu::VRAM_BEGIN),
            _ => self.memory[address],
        }
    }

    fn read_bytes_from(&self, address: u16) -> MemoryLocation {
        MemoryLocation::new(address, Rc::clone(&self.memory))
    }

    fn write_byte(&mut self, address: u16, value: u8) {
        if let Some(mem) = Rc::get_mut(&mut self.memory) {
            let address = address as usize;
            match address {
                gpu::VRAM_BEGIN...gpu::VRAM_END => {
                    self.gpu.write_vram(address - gpu::VRAM_BEGIN, value);
                }
                _ => {
                    mem[address] = value;
                }
            }
        } else {
            panic!("Cannot mutably write whilst borrowed at 0x{:x}", address);
        };
    }
}
