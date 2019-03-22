use std::rc::Rc;

#[derive(Clone, Copy, Debug, PartialEq)]
struct FlagsRegister {
    zero: bool,
    subtract: bool,
    half_carry: bool,
    carry: bool,
}

const ZERO_FLAG_BYTE_POSITION: u8 = 7;
const SUBTRACT_FLAG_BYTE_POSITION: u8 = 6;
const HALF_CARRY_FLAG_BYTE_POSITION: u8 = 5;
const CARRY_FLAG_BYTE_POSITION: u8 = 4;

impl std::convert::From<FlagsRegister> for u8 {
    fn from(flag: FlagsRegister) -> u8 {
        (if flag.zero { 1 } else { 0 }) << ZERO_FLAG_BYTE_POSITION
            | (if flag.subtract { 1 } else { 0 }) << SUBTRACT_FLAG_BYTE_POSITION
            | (if flag.half_carry { 1 } else { 0 }) << HALF_CARRY_FLAG_BYTE_POSITION
            | (if flag.carry { 1 } else { 0 }) << CARRY_FLAG_BYTE_POSITION
    }
}

impl std::convert::From<u8> for FlagsRegister {
    fn from(byte: u8) -> Self {
        let zero = ((byte >> ZERO_FLAG_BYTE_POSITION) & 0b1) != 0;
        let subtract = ((byte >> SUBTRACT_FLAG_BYTE_POSITION) & 0b1) != 0;
        let half_carry = ((byte >> HALF_CARRY_FLAG_BYTE_POSITION) & 0b1) != 0;
        let carry = ((byte >> CARRY_FLAG_BYTE_POSITION) & 0b1) != 0;

        FlagsRegister {
            zero,
            subtract,
            half_carry,
            carry,
        }
    }
}

impl FlagsRegister {
    fn new() -> FlagsRegister {
        FlagsRegister {
            zero: false,
            subtract: false,
            half_carry: false,
            carry: false,
        }
    }

    fn set_zero(&mut self) -> &mut Self {
        self.zero = true;
        self
    }

    fn set_subtract(&mut self) -> &mut Self {
        self.subtract = true;
        self
    }

    fn set_half_carry(&mut self) -> &mut Self {
        self.half_carry = true;
        self
    }
    fn set_carry(&mut self) -> &mut Self {
        self.carry = true;
        self
    }
}

struct Registers {
    a: u8,
    b: u8,
    c: u8,
    d: u8,
    e: u8,
    f: FlagsRegister,
    h: u8,
    l: u8,
}

impl Registers {
    pub fn new() -> Registers {
        Registers {
            a: 0,
            b: 0,
            c: 0,
            d: 0,
            e: 0,
            f: FlagsRegister::new(),
            h: 0,
            l: 0,
        }
    }

    pub fn get_reg(&self, r: Reg) -> u8 {
        match r {
            Reg::A => self.a,
            Reg::B => self.b,
            Reg::C => self.c,
            Reg::D => self.d,
            Reg::E => self.e,
            Reg::H => self.h,
            Reg::L => self.l,
        }
    }

    pub fn set_reg(&mut self, r: Reg, value: u8) -> &mut Self {
        match r {
            Reg::A => self.a = value,
            Reg::B => self.b = value,
            Reg::C => self.c = value,
            Reg::D => self.d = value,
            Reg::E => self.e = value,
            Reg::H => self.h = value,
            Reg::L => self.l = value,
        };

        self
    }

    pub fn get_af(&self) -> u16 {
        let f_u8: u8 = self.f.into();
        (self.a as u16) << 8 | f_u8 as u16
    }

    pub fn set_af(&mut self, value: u16) -> &mut Self {
        self.a = ((value & 0xFF00) >> 8) as u8;
        self.f = ((value & 0x00FF) as u8).into();

        self
    }

    pub fn get_bc(&self) -> u16 {
        (self.b as u16) << 8 | self.c as u16
    }

    pub fn set_bc(&mut self, value: u16) -> &mut Self {
        self.b = ((value & 0xFF00) >> 8) as u8;
        self.c = (value & 0x00FF) as u8;

        self
    }

    pub fn get_de(&self) -> u16 {
        (self.d as u16) << 8 | self.e as u16
    }

    pub fn set_de(&mut self, value: u16) -> &mut Self {
        self.d = ((value & 0xFF00) >> 8) as u8;
        self.e = (value & 0x00FF) as u8;

        self
    }

    pub fn get_hl(&self) -> u16 {
        (self.h as u16) << 8 | self.l as u16
    }

    pub fn set_hl(&mut self, value: u16) -> &mut Self {
        self.h = ((value & 0xFF00) >> 8) as u8;
        self.l = (value & 0x00FF) as u8;

        self
    }
}

enum Reg {
    A,
    B,
    C,
    D,
    E,
    H,
    L,
}

impl Reg {
    fn from_byte(value: u8) -> Reg {
        match value {
            0x07 => Reg::A,
            0x00 => Reg::B,
            0x01 => Reg::C,
            0x02 => Reg::D,
            0x03 => Reg::E,
            0x04 => Reg::H,
            0x05 => Reg::L,
            _ => panic!(
                "Byte value does not match any known register: 0x{:x}",
                value
            ),
        }
    }
}

enum ArithmeticTarget {
    Register(Reg),
    HL,
    Immediate(u8),
}

enum LoadTarget {
    Reg2Reg(Reg, Reg),
    Immediate2Reg(Reg, u8),
    HL2Reg(Reg),
    Reg2HL(Reg),
    Immediate2HL(u8),
    BC2A,
    DE2A,
    CRAM2A,
    A2CRAM,
    ImmediateRAM2A(u8),
    A2ImmediateRAM(u8),
    BigImmediateRAM2A(u16),
    A2BigImmediateRAM(u16),
    HLI2A,
    HLD2A,
    A2HLI,
    A2HLD,
    A2BC,
    A2DE,
}

enum Instruction {
    ADD(ArithmeticTarget),
    ADC,
    SUB,
    SBC,
    AND,
    OR,
    XOR,
    CP,
    INC,
    LD(LoadTarget),
    PUSH,
    POP,
    LDHL,
    DEC,
    RLCA,
    RLA,
    RRCA,
    RRA,
    RLC,
    RL,
    RRC,
    RR,
    SLA,
    SRA,
    SRL,
    SWAP,
}

impl Instruction {
    fn from_byte<I>(opcode: u8, bytes: &mut I) -> Option<Instruction>
    where
        I: Iterator<Item = (u16, u8)>,
    {
        match opcode {
            0x87 | 0x80 | 0x81 | 0x82 | 0x83 | 0x84 | 0x85 => Some(Instruction::ADD(
                ArithmeticTarget::Register(Reg::from_byte(opcode & 0x07)),
            )),
            0x86 => Some(Instruction::ADD(ArithmeticTarget::HL)),
            0xC6 => {
                if let Some((_addr, n)) = bytes.next() {
                    Some(Instruction::ADD(ArithmeticTarget::Immediate(n)))
                } else {
                    panic!("Bus overrun whilst reading 0x{:x}", opcode);
                }
            }
            0x06 | 0x0E | 0x16 | 0x1E | 0x26 | 0x2E => {
                let r = Reg::from_byte((opcode & 0x38) >> 3);
                if let Some((_addr, n)) = bytes.next() {
                    Some(Instruction::LD(LoadTarget::Immediate2Reg(r, n)))
                } else {
                    panic!("Bus overrun whilst reading 0x{:x}", opcode);
                }
            }
            0x7F | 0x78 | 0x79 | 0x7A | 0x7B | 0x7C | 0x7D => {
                let r1 = Reg::A;
                let r2 = Reg::from_byte(opcode & 0x07);

                Some(Instruction::LD(LoadTarget::Reg2Reg(r1, r2)))
            }
            0x40 | 0x41 | 0x42 | 0x43 | 0x44 | 0x45 => {
                let r1 = Reg::B;
                let r2 = Reg::from_byte(opcode & 0x07);

                Some(Instruction::LD(LoadTarget::Reg2Reg(r1, r2)))
            }
            0x48 | 0x49 | 0x4A | 0x4B | 0x4C | 0x4D => {
                let r1 = Reg::C;
                let r2 = Reg::from_byte(opcode & 0x07);

                Some(Instruction::LD(LoadTarget::Reg2Reg(r1, r2)))
            }
            0x50 | 0x51 | 0x52 | 0x53 | 0x54 | 0x55 => {
                let r1 = Reg::D;
                let r2 = Reg::from_byte(opcode & 0x07);

                Some(Instruction::LD(LoadTarget::Reg2Reg(r1, r2)))
            }
            0x58 | 0x59 | 0x5A | 0x5B | 0x5C | 0x5D => {
                let r1 = Reg::E;
                let r2 = Reg::from_byte(opcode & 0x07);

                Some(Instruction::LD(LoadTarget::Reg2Reg(r1, r2)))
            }
            0x60 | 0x61 | 0x62 | 0x63 | 0x64 | 0x65 => {
                let r1 = Reg::H;
                let r2 = Reg::from_byte(opcode & 0x07);

                Some(Instruction::LD(LoadTarget::Reg2Reg(r1, r2)))
            }
            0x68 | 0x69 | 0x6A | 0x6B | 0x6C | 0x6D => {
                let r1 = Reg::L;
                let r2 = Reg::from_byte(opcode & 0x07);

                Some(Instruction::LD(LoadTarget::Reg2Reg(r1, r2)))
            }
            _ => None,
        }
    }
}

struct MemoryLocation {
    cursor: u16,
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

struct MemoryBus {
    memory: Rc<[u8]>,
}

impl MemoryBus {
    fn new() -> MemoryBus {
        MemoryBus {
            memory: Rc::new([0; 65_536]),
        }
    }

    fn read_byte(&self, address: u16) -> u8 {
        self.memory[address as usize]
    }

    fn read_bytes_from(&self, address: u16) -> MemoryLocation {
        MemoryLocation::new(address, Rc::clone(&self.memory))
    }

    fn write_byte(&mut self, address: u16, value: u8) -> &mut Self {
        if let Some(mem) = Rc::get_mut(&mut self.memory) {
            mem[address as usize] = value;
        } else {
            panic!("Cannot mutably write whilst borrowed at 0x{:x}", address);
        }

        self
    }
}

struct CPU {
    registers: Registers,
    pc: u16,
    bus: MemoryBus,
}

impl CPU {
    pub fn new() -> CPU {
        CPU {
            registers: Registers::new(),
            pc: 0x100,
            bus: MemoryBus::new(),
        }
    }

    pub fn step(&mut self) {
        let instruction_byte = self.bus.read_byte(self.pc);
        let mut more_bytes = self
            .bus
            .read_bytes_from(self.pc.wrapping_add(1))
            .into_iter();

        if let Some(instruction) = Instruction::from_byte(instruction_byte, &mut more_bytes) {
            if let Some(new_pointer) = self.execute(instruction) {
                self.pc = new_pointer;
            } else {
                self.pc = more_bytes.cursor;
            }
        } else {
            panic!("Unknown instruction found for 0x{:x}", instruction_byte);
        }
    }

    fn execute(&mut self, instruction: Instruction) -> Option<u16> {
        match instruction {
            Instruction::ADD(target) => match target {
                /* ----------------------------------- ADD ----------------------------- */
                ArithmeticTarget::Register(r) => {
                    let value = self.registers.get_reg(r);
                    self.add(value);
                }
                ArithmeticTarget::HL => {
                    let value = self.registers.get_hl();
                    self.add((value & 0x00FF) as u8);
                }
                ArithmeticTarget::Immediate(n) => {
                    self.add(n);
                }
            },
            /* ----------------------------------- LOAD ----------------------------- */
            Instruction::LD(target) => match target {
                LoadTarget::Reg2Reg(r1, r2) => {
                    let value = self.registers.get_reg(r2);

                    self.registers.set_reg(r1, value);
                }
                LoadTarget::Immediate2Reg(r, value) => {
                    self.registers.set_reg(r, value);
                }
                LoadTarget::HL2Reg(r) => {
                    let addr = self.registers.get_hl();
                    self.registers.set_reg(r, self.bus.read_byte(addr));
                }
                LoadTarget::Reg2HL(r) => {
                    let value = self.registers.get_reg(r);
                    let addr = self.registers.get_hl();
                    self.bus.write_byte(addr, value);
                }
                LoadTarget::Immediate2HL(value) => {
                    let addr = self.registers.get_hl();
                    self.bus.write_byte(addr, value);
                }
                LoadTarget::BC2A => {
                    let addr = self.registers.get_bc();
                    let value = self.bus.read_byte(addr);
                    self.registers.a = value;
                }
                LoadTarget::A2BC => {
                    let value = self.registers.a;
                    let addr = self.registers.get_bc();
                    self.bus.write_byte(addr, value);
                }
                LoadTarget::DE2A => {
                    let addr = self.registers.get_de();
                    let value = self.bus.read_byte(addr);
                    self.registers.a = value;
                }
                LoadTarget::A2DE => {
                    let value = self.registers.a;
                    let addr = self.registers.get_de();
                    self.bus.write_byte(addr, value);
                }
                LoadTarget::CRAM2A => { /* TODO */ }
                LoadTarget::A2CRAM => { /* TODO */ }
                LoadTarget::ImmediateRAM2A(_value) => { /* TODO */ }
                LoadTarget::A2ImmediateRAM(_value) => { /* TODO */ }
                LoadTarget::BigImmediateRAM2A(_value) => { /* TODO */ }
                LoadTarget::A2BigImmediateRAM(_value) => { /* TODO */ }
                LoadTarget::HLI2A => {
                    let addr = self.registers.get_hl();
                    let value = self.bus.read_byte(addr);
                    self.registers.a = value;
                    let (new_hl, _) = addr.overflowing_add(1);
                    self.registers.set_hl(new_hl);
                }
                LoadTarget::HLD2A => {
                    let addr = self.registers.get_hl();
                    let value = self.bus.read_byte(addr);
                    self.registers.a = value;
                    let (new_hl, _) = addr.overflowing_sub(1);
                    self.registers.set_hl(new_hl);
                }
                LoadTarget::A2HLI => {
                    let addr = self.registers.get_hl();
                    let value = self.registers.a;
                    self.bus.write_byte(addr, value);
                    let (new_hl, _) = addr.overflowing_add(1);
                    self.registers.set_hl(new_hl);
                }
                LoadTarget::A2HLD => {
                    let addr = self.registers.get_hl();
                    let value = self.registers.a;
                    self.bus.write_byte(addr, value);
                    let (new_hl, _) = addr.overflowing_sub(1);
                    self.registers.set_hl(new_hl);
                }
            },
            _ => { /* TODO */ }
        };

        None
    }

    fn add(&mut self, value: u8) -> &mut Self {
        let (new_value, did_overflow) = self.registers.a.overflowing_add(value);
        self.registers.f.zero = new_value == 0;
        self.registers.f.subtract = false;
        self.registers.f.carry = did_overflow;
        self.registers.f.half_carry = (self.registers.a & 0xF) + (value & 0xF) > 0xF;

        self.registers.a = new_value;
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_add_register_register() {
        let mut cpu = CPU::new();
        cpu.registers.a = 0x3A;
        cpu.registers.b = 0xC6;

        cpu.execute(Instruction::ADD(ArithmeticTarget::Register(Reg::B)));

        assert_eq!(cpu.registers.a, 0x0);

        let mut expected_flags = FlagsRegister::new();
        expected_flags.set_zero().set_half_carry().set_carry();

        assert_eq!(cpu.registers.f, expected_flags);
    }

    #[test]
    fn test_add_register_immediate() {
        let mut cpu = CPU::new();
        cpu.registers.a = 0x3C;
        let n = 0xFF;

        cpu.execute(Instruction::ADD(ArithmeticTarget::Immediate(n)));

        assert_eq!(cpu.registers.a, 0x3B);

        let mut expected_flags = FlagsRegister::new();
        expected_flags.set_half_carry().set_carry();

        assert_eq!(cpu.registers.f, expected_flags);
    }

    #[test]
    fn test_add_register_hl() {
        let mut cpu = CPU::new();
        cpu.registers.a = 0x3C;
        cpu.registers.h = 0x0;
        cpu.registers.l = 0x12;

        cpu.execute(Instruction::ADD(ArithmeticTarget::HL));

        assert_eq!(cpu.registers.a, 0x4E);

        let expected_flags = FlagsRegister::new();

        assert_eq!(cpu.registers.f, expected_flags);
    }

    #[test]
    fn test_load_register_register() {
        let mut cpu = CPU::new();
        cpu.registers.a = 0x3C;
        cpu.registers.b = 0x01;

        cpu.execute(Instruction::LD(LoadTarget::Reg2Reg(Reg::A, Reg::B)));
        assert_eq!(cpu.registers.a, cpu.registers.b);
    }

    #[test]
    fn test_load_register_immediate() {
        let mut cpu = CPU::new();
        cpu.registers.a = 0x3C;
        let value = 0x01;

        cpu.execute(Instruction::LD(LoadTarget::Immediate2Reg(Reg::A, value)));
        assert_eq!(cpu.registers.a, value);
    }

    #[test]
    fn test_load_register_hl() {
        let mut cpu = CPU::new();
        cpu.registers.h = 0x00;
        let addr = 0x1234;
        cpu.bus.write_byte(addr, 0x5C);
        cpu.registers.set_hl(addr);

        cpu.execute(Instruction::LD(LoadTarget::HL2Reg(Reg::H)));
        assert_eq!(cpu.registers.h, 0x5C);
    }

    #[test]
    fn test_load_hl_register() {
        let mut cpu = CPU::new();
        cpu.registers.a = 0x3C;
        let addr = 0x1234;
        cpu.registers.set_hl(addr);

        cpu.execute(Instruction::LD(LoadTarget::Reg2HL(Reg::A)));
        assert_eq!(cpu.bus.read_byte(addr), 0x3C);
    }

    #[test]
    fn test_load_hl_immediate() {
        let mut cpu = CPU::new();
        let addr = 0x1234;
        cpu.registers.set_hl(addr);
        let value = 0x8;

        cpu.execute(Instruction::LD(LoadTarget::Immediate2HL(value)));
        assert_eq!(cpu.bus.read_byte(addr), value);
    }

    #[test]
    fn test_load_a_bc() {
        let mut cpu = CPU::new();
        let addr = 0x1234;
        cpu.registers.a = 0x3C;
        cpu.registers.set_bc(addr);
        cpu.bus.write_byte(addr, 0x2F);

        cpu.execute(Instruction::LD(LoadTarget::BC2A));
        assert_eq!(cpu.registers.a, 0x2F);
    }

    #[test]
    fn test_load_bc_a() {
        let mut cpu = CPU::new();
        let addr = 0x1234;
        cpu.registers.a = 0x3F;
        cpu.registers.set_bc(addr);

        cpu.execute(Instruction::LD(LoadTarget::A2BC));
        assert_eq!(cpu.bus.read_byte(addr), 0x3F);
    }

    #[test]
    fn test_load_a_de() {
        let mut cpu = CPU::new();
        let addr = 0x1234;
        cpu.registers.a = 0x3C;
        cpu.registers.set_de(addr);
        cpu.bus.write_byte(addr, 0x5F);

        cpu.execute(Instruction::LD(LoadTarget::DE2A));
        assert_eq!(cpu.registers.a, 0x5F);
    }

    #[test]
    fn test_load_de_a() {
        let mut cpu = CPU::new();
        let addr = 0x1234;
        cpu.registers.a = 0x2F;
        cpu.registers.set_de(addr);

        cpu.execute(Instruction::LD(LoadTarget::A2DE));
        assert_eq!(cpu.bus.read_byte(addr), 0x2F);
    }

    #[test]
    fn test_load_a_hli() {
        let mut cpu = CPU::new();
        cpu.registers.a = 0x0;
        let addr = 0x8A5C;
        cpu.registers.set_hl(addr);
        cpu.bus.write_byte(addr, 0x3C);

        cpu.execute(Instruction::LD(LoadTarget::HLI2A));
        assert_eq!(cpu.registers.a, 0x3C);
        assert_eq!(cpu.registers.get_hl(), 0x8A5D);
    }

    #[test]
    fn test_load_hli_a() {
        let mut cpu = CPU::new();
        cpu.registers.a = 0x56;
        let addr = 0xFFFF;
        cpu.registers.set_hl(addr);

        cpu.execute(Instruction::LD(LoadTarget::A2HLI));
        assert_eq!(cpu.bus.read_byte(addr), 0x56);
        assert_eq!(cpu.registers.get_hl(), 0x0000);
    }

    #[test]
    fn test_load_a_hld() {
        let mut cpu = CPU::new();
        cpu.registers.a = 0x0;
        let addr = 0x8A5C;
        cpu.registers.set_hl(addr);
        cpu.bus.write_byte(addr, 0x3C);

        cpu.execute(Instruction::LD(LoadTarget::HLD2A));
        assert_eq!(cpu.registers.a, 0x3C);
        assert_eq!(cpu.registers.get_hl(), 0x8A5B);
    }

    #[test]
    fn test_load_hld_a() {
        let mut cpu = CPU::new();
        cpu.registers.a = 0x05;
        let addr = 0x4000;
        cpu.registers.set_hl(addr);

        cpu.execute(Instruction::LD(LoadTarget::A2HLD));
        assert_eq!(cpu.bus.read_byte(addr), 0x05);
        assert_eq!(cpu.registers.get_hl(), 0x3FFF);
    }

    #[test]
    fn test_step_add() {
        let mut cpu = CPU::new();
        cpu.bus.write_byte(0x00, 0xC6);
        cpu.bus.write_byte(0x01, 0xFF);
        cpu.pc = 0x00;

        cpu.step();

        assert_eq!(cpu.registers.a, 0xFF);
        assert_eq!(cpu.pc, 0x02);
    }
}
