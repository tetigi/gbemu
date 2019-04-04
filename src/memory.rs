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

pub struct MemoryBus {
    memory: Rc<[u8]>,
}

impl MemoryBus {
    pub fn new() -> MemoryBus {
        MemoryBus {
            memory: Rc::new([0; 65_536]),
        }
    }

    pub fn read_byte(&self, address: u16) -> u8 {
        self.memory[address as usize]
    }

    pub fn read_bytes_from(&self, address: u16) -> MemoryLocation {
        MemoryLocation::new(address, Rc::clone(&self.memory))
    }

    pub fn write_byte(&mut self, address: u16, value: u8) -> &mut Self {
        if let Some(mem) = Rc::get_mut(&mut self.memory) {
            mem[address as usize] = value;
        } else {
            panic!("Cannot mutably write whilst borrowed at 0x{:x}", address);
        }

        self
    }
}
