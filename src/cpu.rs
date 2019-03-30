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

    pub fn get_reg(&self, r: &Reg) -> u8 {
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

    pub fn set_reg(&mut self, r: &Reg, value: u8) -> &mut Self {
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

    pub fn set_pair(&mut self, rs: RegPair, value: u16) -> &mut Self {
        match rs {
            RegPair::BC => self.set_bc(value),
            RegPair::DE => self.set_de(value),
            RegPair::HL => self.set_hl(value),
            RegPair::AF => self.set_af(value),
            RegPair::SP => panic!("Cannot set SP as register pair!"),
        };

        self
    }

    pub fn get_pair(&self, rs: &RegPair) -> u16 {
        match rs {
            RegPair::BC => self.get_bc(),
            RegPair::DE => self.get_de(),
            RegPair::HL => self.get_hl(),
            RegPair::AF => self.get_af(),
            RegPair::SP => panic!("Cannot set SP as register pair!"),
        }
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

enum RegPair {
    BC,
    DE,
    HL,
    AF,
    SP,
}

impl RegPair {
    fn from_byte_qq(value: u8) -> RegPair {
        match value {
            0x00 => RegPair::BC,
            0x01 => RegPair::DE,
            0x02 => RegPair::HL,
            0x03 => RegPair::AF,
            _ => panic!(
                "Byte value does not match any known register pair: 0x{:x}",
                value
            ),
        }
    }

    fn from_byte_dd(value: u8) -> RegPair {
        match value {
            0x00 => RegPair::BC,
            0x01 => RegPair::DE,
            0x02 => RegPair::HL,
            0x03 => RegPair::SP,
            _ => panic!(
                "Byte value does not match any known register pair: 0x{:x}",
                value
            ),
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

enum ArithmeticTarget {
    Register(Reg),
    HL,
    Immediate(u8),
}

enum BigArithmeticTarget {
    Registers(RegPair),
    Operand(u8),
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
    BigImmediate2Regs(RegPair, u16),
    HL2SP,
    SP2RAM(u16),
}

enum Instruction {
    ADD(ArithmeticTarget),
    ADC(ArithmeticTarget),
    SUB(ArithmeticTarget),
    SBC(ArithmeticTarget),
    AND(ArithmeticTarget),
    OR(ArithmeticTarget),
    XOR(ArithmeticTarget),
    CP(ArithmeticTarget),
    INC(ArithmeticTarget),
    LD(LoadTarget),
    PUSH(RegPair),
    POP(RegPair),
    LDHL(u8),
    DEC(ArithmeticTarget),
    BIGADD(BigArithmeticTarget),
    BIGINC(RegPair),
    BIGDEC(RegPair),
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
    fn read_immediate<I>(opcode: u8, bytes: &mut I) -> u8
    where
        I: Iterator<Item = (u16, u8)>,
    {
        if let Some((_addr, n)) = bytes.next() {
            n
        } else {
            panic!("Bus overrun whilst reading 0x{:x}", opcode);
        }
    }

    fn read_big_immediate<I>(opcode: u8, bytes: &mut I) -> u16
    where
        I: Iterator<Item = (u16, u8)>,
    {
        let ns: Vec<u8> = bytes.take(2).map(|(_, b)| b).collect();

        if ns.len() == 2 {
            (ns[0] as u16) << 8 | ns[1] as u16
        } else {
            panic!("Bus overrun whilst reading 0x{:x}", opcode);
        }
    }

    fn from_byte<I>(opcode: u8, bytes: &mut I) -> Option<Instruction>
    where
        I: Iterator<Item = (u16, u8)>,
    {
        match opcode {
            0x06 | 0x0E | 0x16 | 0x1E | 0x26 | 0x2E => {
                let r = Reg::from_byte((opcode & 0x38) >> 3);
                let n = Instruction::read_immediate(opcode, bytes);
                Some(Instruction::LD(LoadTarget::Immediate2Reg(r, n)))
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
            0x7E | 0x46 | 0x4E | 0x56 | 0x5E | 0x66 | 0x6E => {
                let r = Reg::from_byte((opcode & 0x38) >> 3);

                Some(Instruction::LD(LoadTarget::HL2Reg(r)))
            }
            0x70 | 0x71 | 0x72 | 0x73 | 0x74 | 0x75 => {
                let r = Reg::from_byte(opcode & 0x07);

                Some(Instruction::LD(LoadTarget::Reg2HL(r)))
            }
            0x36 => {
                let n = Instruction::read_immediate(opcode, bytes);
                Some(Instruction::LD(LoadTarget::Immediate2HL(n)))
            }
            0x0A => Some(Instruction::LD(LoadTarget::BC2A)),
            0x1A => Some(Instruction::LD(LoadTarget::DE2A)),
            0xF0 => {
                let n = Instruction::read_immediate(opcode, bytes);
                Some(Instruction::LD(LoadTarget::ImmediateRAM2A(n)))
            }
            0xE0 => {
                let n = Instruction::read_immediate(opcode, bytes);
                Some(Instruction::LD(LoadTarget::A2ImmediateRAM(n)))
            }
            0xFA => {
                let nn = Instruction::read_big_immediate(opcode, bytes);
                Some(Instruction::LD(LoadTarget::BigImmediateRAM2A(nn)))
            }
            0xEA => {
                let nn = Instruction::read_big_immediate(opcode, bytes);
                Some(Instruction::LD(LoadTarget::A2BigImmediateRAM(nn)))
            }
            0xF2 => Some(Instruction::LD(LoadTarget::CRAM2A)),
            0xE2 => Some(Instruction::LD(LoadTarget::A2CRAM)),
            0x2A => Some(Instruction::LD(LoadTarget::A2HLI)),
            0x3A => Some(Instruction::LD(LoadTarget::A2HLD)),
            0x02 => Some(Instruction::LD(LoadTarget::A2BC)),
            0x12 => Some(Instruction::LD(LoadTarget::A2DE)),
            0x22 => Some(Instruction::LD(LoadTarget::A2HLI)),
            0x32 => Some(Instruction::LD(LoadTarget::A2HLD)),
            0x01 | 0x11 | 0x21 | 0x31 => {
                let nn = Instruction::read_big_immediate(opcode, bytes);
                let rs = RegPair::from_byte_dd((opcode >> 4) & 0x03);
                Some(Instruction::LD(LoadTarget::BigImmediate2Regs(rs, nn)))
            }
            0xF9 => Some(Instruction::LD(LoadTarget::HL2SP)),
            0xC5 | 0xD5 | 0xE5 | 0xF5 => Some(Instruction::PUSH(RegPair::from_byte_qq(
                (opcode >> 4) & 0x03,
            ))),
            0xC1 | 0xD1 | 0xE1 | 0xF1 => Some(Instruction::POP(RegPair::from_byte_qq(
                (opcode >> 4) & 0x03,
            ))),
            0x08 => {
                let nn = Instruction::read_big_immediate(opcode, bytes);
                Some(Instruction::LD(LoadTarget::SP2RAM(nn)))
            }
            0x87 | 0x80 | 0x81 | 0x82 | 0x83 | 0x84 | 0x85 => Some(Instruction::ADD(
                ArithmeticTarget::Register(Reg::from_byte(opcode & 0x07)),
            )),
            0x86 => Some(Instruction::ADD(ArithmeticTarget::HL)),
            0xC6 => {
                let n = Instruction::read_immediate(opcode, bytes);
                Some(Instruction::ADD(ArithmeticTarget::Immediate(n)))
            }
            0x8F | 0x88 | 0x89 | 0x8A | 0x8B | 0x8C | 0x8D => Some(Instruction::ADC(
                ArithmeticTarget::Register(Reg::from_byte(opcode & 0x07)),
            )),
            0x8E => Some(Instruction::ADC(ArithmeticTarget::HL)),
            0xCE => {
                let n = Instruction::read_immediate(opcode, bytes);
                Some(Instruction::ADC(ArithmeticTarget::Immediate(n)))
            }
            0x97 | 0x90 | 0x91 | 0x92 | 0x93 | 0x94 | 0x95 => Some(Instruction::SUB(
                ArithmeticTarget::Register(Reg::from_byte(opcode & 0x07)),
            )),
            0x96 => Some(Instruction::SUB(ArithmeticTarget::HL)),
            0xD6 => {
                let n = Instruction::read_immediate(opcode, bytes);
                Some(Instruction::SUB(ArithmeticTarget::Immediate(n)))
            }
            0x9F | 0x98 | 0x99 | 0x9A | 0x9B | 0x9C | 0x9D => Some(Instruction::SBC(
                ArithmeticTarget::Register(Reg::from_byte(opcode & 0x07)),
            )),
            0x9E => Some(Instruction::SBC(ArithmeticTarget::HL)),
            0xDE => {
                let n = Instruction::read_immediate(opcode, bytes);
                Some(Instruction::SBC(ArithmeticTarget::Immediate(n)))
            }
            0xA7 | 0xA0 | 0xA1 | 0xA2 | 0xA3 | 0xA4 | 0xA5 => Some(Instruction::AND(
                ArithmeticTarget::Register(Reg::from_byte(opcode & 0x07)),
            )),
            0xA6 => Some(Instruction::AND(ArithmeticTarget::HL)),
            0xE6 => {
                let n = Instruction::read_immediate(opcode, bytes);
                Some(Instruction::AND(ArithmeticTarget::Immediate(n)))
            }
            0xB7 | 0xB0 | 0xB1 | 0xB2 | 0xB3 | 0xB4 | 0xB5 => Some(Instruction::OR(
                ArithmeticTarget::Register(Reg::from_byte(opcode & 0x07)),
            )),
            0xB6 => Some(Instruction::OR(ArithmeticTarget::HL)),
            0xF6 => {
                let n = Instruction::read_immediate(opcode, bytes);
                Some(Instruction::OR(ArithmeticTarget::Immediate(n)))
            }
            0xAF | 0xA8 | 0xA9 | 0xAA | 0xAB | 0xAC | 0xAD => Some(Instruction::XOR(
                ArithmeticTarget::Register(Reg::from_byte(opcode & 0x07)),
            )),
            0xAE => Some(Instruction::XOR(ArithmeticTarget::HL)),
            0xEE => {
                let n = Instruction::read_immediate(opcode, bytes);
                Some(Instruction::XOR(ArithmeticTarget::Immediate(n)))
            }
            0xBF | 0xB8 | 0xB9 | 0xBA | 0xBB | 0xBC | 0xBD => Some(Instruction::CP(
                ArithmeticTarget::Register(Reg::from_byte(opcode & 0x07)),
            )),
            0xBE => Some(Instruction::CP(ArithmeticTarget::HL)),
            0xFE => {
                let n = Instruction::read_immediate(opcode, bytes);
                Some(Instruction::CP(ArithmeticTarget::Immediate(n)))
            }
            0x3C | 0x04 | 0x0C | 0x14 | 0x1C | 0x24 | 0x2C => Some(Instruction::INC(
                ArithmeticTarget::Register(Reg::from_byte(opcode & 0x07)),
            )),
            0x34 => Some(Instruction::INC(ArithmeticTarget::HL)),
            0x3D | 0x05 | 0x0D | 0x15 | 0x1D | 0x25 | 0x2D => Some(Instruction::DEC(
                ArithmeticTarget::Register(Reg::from_byte(opcode & 0x07)),
            )),
            0x35 => Some(Instruction::DEC(ArithmeticTarget::HL)),
            0x09 | 0x19 | 0x29 | 0x39 => Some(Instruction::BIGADD(BigArithmeticTarget::Registers(
                RegPair::from_byte_dd((opcode >> 4) & 0x03),
            ))),
            0xE8 => {
                let n = Instruction::read_immediate(opcode, bytes);
                Some(Instruction::BIGADD(BigArithmeticTarget::Operand(n)))
            }
            0x03 | 0x13 | 0x23 | 0x33 => Some(Instruction::BIGINC(RegPair::from_byte_dd(
                (opcode >> 4) & 0x03,
            ))),
            0x0B | 0x1B | 0x2B | 0x3B => Some(Instruction::BIGDEC(RegPair::from_byte_dd(
                (opcode >> 4) & 0x03,
            ))),
            0x07 => Some(Instruction::RLCA),
            0x17 => Some(Instruction::RLA),
            0x0F => Some(Instruction::RRCA),
            0x1F => Some(Instruction::RRA),
            _ => None,
        }
    }
}

struct CPU {
    registers: Registers,
    pc: u16,
    sp: u16,
    bus: MemoryBus,
}

impl CPU {
    pub fn new() -> CPU {
        CPU {
            registers: Registers::new(),
            pc: 0x100,
            sp: 0xFFFE,
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
            Instruction::LD(target) => match target {
                LoadTarget::Reg2Reg(r1, r2) => {
                    let value = self.registers.get_reg(&r2);

                    self.registers.set_reg(&r1, value);
                }
                LoadTarget::Immediate2Reg(r, value) => {
                    self.registers.set_reg(&r, value);
                }
                LoadTarget::HL2Reg(r) => {
                    let addr = self.registers.get_hl();
                    self.registers.set_reg(&r, self.bus.read_byte(addr));
                }
                LoadTarget::Reg2HL(r) => {
                    let value = self.registers.get_reg(&r);
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
                LoadTarget::CRAM2A => {
                    let addr = 0xFF00 + (self.registers.c as u16);
                    let value = self.bus.read_byte(addr);

                    self.registers.a = value;
                }
                LoadTarget::A2CRAM => {
                    let addr = 0xFF00 + (self.registers.c as u16);

                    self.bus.write_byte(addr, self.registers.a);
                }
                LoadTarget::ImmediateRAM2A(addr) => {
                    let addr = 0xFF00 + (addr as u16);
                    let value = self.bus.read_byte(addr);

                    self.registers.a = value;
                }
                LoadTarget::A2ImmediateRAM(addr) => {
                    let addr = 0xFF00 + (addr as u16);

                    self.bus.write_byte(addr, self.registers.a);
                }
                LoadTarget::BigImmediateRAM2A(addr) => {
                    let value = self.bus.read_byte(addr);

                    self.registers.a = value;
                }
                LoadTarget::A2BigImmediateRAM(addr) => {
                    self.bus.write_byte(addr, self.registers.a);
                }
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
                LoadTarget::BigImmediate2Regs(rs, n) => {
                    match rs {
                        RegPair::BC => {
                            self.registers.set_bc(n);
                        }
                        RegPair::DE => {
                            self.registers.set_de(n);
                        }
                        RegPair::HL => {
                            self.registers.set_hl(n);
                        }
                        RegPair::SP => self.sp = n,
                        RegPair::AF => panic!("Flag pair not suported in LD BigImmediate2Regs"),
                    };
                }
                LoadTarget::HL2SP => {
                    let hl = self.registers.get_hl();
                    self.sp = hl;
                }
                LoadTarget::SP2RAM(nn) => {
                    let value = self.sp;
                    let h = ((value & 0xFF00) >> 8) as u8;
                    let l = (value & 0x00FF) as u8;
                    self.bus.write_byte(nn, l);
                    self.bus.write_byte(nn + 1, h);
                }
            },
            Instruction::PUSH(rs) => {
                let value = self.registers.get_pair(&rs);
                let h = ((value & 0xFF00) >> 8) as u8;
                let l = (value & 0x00FF) as u8;

                self.bus.write_byte(self.sp - 1, h);
                self.bus.write_byte(self.sp - 2, l);
                self.sp -= 2;
            }
            Instruction::POP(rs) => {
                let l = self.bus.read_byte(self.sp);
                let h = self.bus.read_byte(self.sp + 1);

                self.registers.set_pair(rs, (h as u16) << 8 | l as u16);
                self.sp += 2;
            }
            Instruction::LDHL(e) => {
                let (value, _is_pos) = CPU::i_to_u16(e);
                let (new_value, did_overflow) = self.sp.overflowing_add(value);

                self.registers.f.zero = false;
                self.registers.f.subtract = false;
                self.registers.f.carry = did_overflow;
                self.registers.f.half_carry = (self.sp & 0xFFF) + (value & 0xFFF) > 0xFFF;

                self.registers.set_hl(new_value);
            }
            Instruction::ADD(target) => self.do_arithmetic(target, &mut CPU::add),
            Instruction::ADC(target) => self.do_arithmetic(target, &mut CPU::adc),
            Instruction::SUB(target) => self.do_arithmetic(target, &mut CPU::sub),
            Instruction::SBC(target) => self.do_arithmetic(target, &mut CPU::subc),
            Instruction::AND(target) => self.do_arithmetic(target, &mut CPU::and),
            Instruction::OR(target) => self.do_arithmetic(target, &mut CPU::or),
            Instruction::XOR(target) => self.do_arithmetic(target, &mut CPU::xor),
            Instruction::CP(target) => self.do_arithmetic(target, &mut CPU::cp),
            Instruction::INC(target) => match target {
                ArithmeticTarget::Register(r) => {
                    let v = self.registers.get_reg(&r);
                    let new_v = self.inc(v);
                    self.registers.set_reg(&r, new_v);
                }
                ArithmeticTarget::HL => {
                    let addr = self.registers.get_hl();
                    let v = self.bus.read_byte(addr);
                    let new_v = self.inc(v);
                    self.bus.write_byte(addr, new_v);
                }
                ArithmeticTarget::Immediate(_) => {
                    panic!("Immediate arithmetic not supported for INC")
                }
            },
            Instruction::DEC(target) => match target {
                ArithmeticTarget::Register(r) => {
                    let v = self.registers.get_reg(&r);
                    let new_v = self.dec(v);
                    self.registers.set_reg(&r, new_v);
                }
                ArithmeticTarget::HL => {
                    let addr = self.registers.get_hl();
                    let v = self.bus.read_byte(addr);
                    let new_v = self.dec(v);
                    self.bus.write_byte(addr, new_v);
                }
                ArithmeticTarget::Immediate(_) => {
                    panic!("Immediate arithmetic not supported for DEC")
                }
            },
            Instruction::BIGADD(target) => match target {
                BigArithmeticTarget::Registers(rs) => {
                    let value = self.registers.get_hl();
                    let operand = self.registers.get_pair(&rs);

                    let (new_value, did_overflow) = value.overflowing_add(operand);
                    self.registers.f.subtract = false;
                    self.registers.f.carry = did_overflow;
                    self.registers.f.half_carry = (value & 0xFFF) + (operand & 0xFFF) > 0xFFF;

                    self.registers.set_hl(new_value);
                }
                BigArithmeticTarget::Operand(e) => {
                    let (value, _is_pos) = CPU::i_to_u16(e);
                    let (new_value, did_overflow) = self.sp.overflowing_add(value);

                    self.registers.f.zero = false;
                    self.registers.f.subtract = false;
                    self.registers.f.carry = did_overflow;
                    self.registers.f.half_carry = (self.sp & 0xFFF) + (value & 0xFFF) > 0xFFF;

                    self.sp = new_value;
                }
            },
            Instruction::BIGINC(rs) => {
                let value = self.registers.get_pair(&rs);
                let (new_value, _) = value.overflowing_add(1);
                self.registers.set_pair(rs, new_value);
            }
            Instruction::BIGDEC(rs) => {
                let value = self.registers.get_pair(&rs);
                let (new_value, _) = value.overflowing_sub(1);
                self.registers.set_pair(rs, new_value);
            }
            Instruction::RLCA => {
                let bit_7 = self.registers.a & 0x80 == 0x80;

                self.registers.f.zero = false;
                self.registers.f.subtract = false;
                self.registers.f.half_carry = false;
                self.registers.f.carry = bit_7;

                if bit_7 {
                    self.registers.a = (self.registers.a << 1) | 0x01;
                } else {
                    self.registers.a = self.registers.a << 1;
                }
            }
            Instruction::RLA => {
                let bit_7 = self.registers.a & 0x80 == 0x80;

                self.registers.f.zero = false;
                self.registers.f.subtract = false;
                self.registers.f.half_carry = false;

                if self.registers.f.carry {
                    self.registers.a = (self.registers.a << 1) | 0x01;
                } else {
                    self.registers.a = self.registers.a << 1;
                }

                self.registers.f.carry = bit_7;
            }
            Instruction::RRCA => {
                let bit_0 = self.registers.a & 0x01 == 0x01;

                self.registers.f.zero = false;
                self.registers.f.subtract = false;
                self.registers.f.half_carry = false;
                self.registers.f.carry = bit_0;

                if bit_0 {
                    self.registers.a = (self.registers.a >> 1) | 0x80;
                } else {
                    self.registers.a = self.registers.a >> 1;
                }
            }
            Instruction::RRA => {
                let bit_0 = self.registers.a & 0x01 == 0x01;

                self.registers.f.zero = false;
                self.registers.f.subtract = false;
                self.registers.f.half_carry = false;

                if self.registers.f.carry {
                    self.registers.a = (self.registers.a >> 1) | 0x80;
                } else {
                    self.registers.a = self.registers.a >> 1;
                }

                self.registers.f.carry = bit_0;
            }
            _ => { /* TODO */ }
        };

        None
    }

    fn do_arithmetic<F>(&mut self, target: ArithmeticTarget, f: &mut F)
    where
        F: FnMut(&mut Self, u8) -> (),
    {
        match target {
            ArithmeticTarget::Register(r) => {
                let value = self.registers.get_reg(&r);
                f(self, value);
            }
            ArithmeticTarget::HL => {
                let addr = self.registers.get_hl();
                let value = self.bus.read_byte(addr);
                f(self, value);
            }
            ArithmeticTarget::Immediate(n) => {
                f(self, n);
            }
        }
    }

    fn i_to_u16(e: u8) -> (u16, bool) {
        let is_pos = e & 0x80 == 0x0;
        if is_pos {
            (e as u16, true)
        } else {
            (!((!e + 1) as u16) + 1, false)
        }
    }

    fn inc(&mut self, current_value: u8) -> u8 {
        let (new_value, _did_overflow) = current_value.overflowing_add(1);

        self.registers.f.zero = new_value == 0;
        self.registers.f.subtract = false;

        self.registers.f.half_carry = (current_value & 0xF) + 1 > 0xF;

        new_value
    }

    fn dec(&mut self, current_value: u8) -> u8 {
        let (new_value, _did_overflow) = current_value.overflowing_sub(1);

        self.registers.f.zero = new_value == 0;
        self.registers.f.subtract = true;
        self.registers.f.half_carry = (current_value & 0xF) < 1;

        new_value
    }

    fn add(&mut self, value: u8) {
        let (new_value, did_overflow) = self.registers.a.overflowing_add(value);
        self.registers.f.zero = new_value == 0;
        self.registers.f.subtract = false;
        self.registers.f.carry = did_overflow;
        self.registers.f.half_carry = (self.registers.a & 0xF) + (value & 0xF) > 0xF;

        self.registers.a = new_value;
    }

    fn adc(&mut self, value: u8) {
        let (tmp, did_overflow1) = self.registers.a.overflowing_add(value);
        let (new_value, did_overflow2) = if self.registers.f.carry {
            tmp.overflowing_add(1)
        } else {
            (tmp, false)
        };

        let did_overflow = did_overflow1 || did_overflow2;

        self.registers.f.zero = new_value == 0;
        self.registers.f.subtract = false;
        self.registers.f.carry = did_overflow;
        self.registers.f.half_carry = (self.registers.a & 0xF) + (value & 0xF) + 1 > 0xF;

        self.registers.a = new_value;
    }

    fn sub(&mut self, value: u8) {
        let (new_value, did_overflow) = self.registers.a.overflowing_sub(value);
        self.registers.f.zero = new_value == 0;
        self.registers.f.subtract = true;
        self.registers.f.carry = did_overflow;
        self.registers.f.half_carry = (self.registers.a & 0xF) < (value & 0xF);

        self.registers.a = new_value;
    }

    fn subc(&mut self, value: u8) {
        let (tmp, did_overflow1) = self.registers.a.overflowing_sub(value);
        let (new_value, did_overflow2) = if self.registers.f.carry {
            tmp.overflowing_sub(1)
        } else {
            (tmp, false)
        };

        let did_overflow = did_overflow1 || did_overflow2;

        self.registers.f.zero = new_value == 0;
        self.registers.f.subtract = true;
        self.registers.f.carry = did_overflow;
        self.registers.f.half_carry = (self.registers.a & 0xF) < (value & 0xF) + 1;

        self.registers.a = new_value;
    }

    fn and(&mut self, value: u8) {
        self.registers.a = self.registers.a & value;

        self.registers.f.half_carry = true;
        self.registers.f.carry = false;
        self.registers.f.subtract = false;
        self.registers.f.zero = self.registers.a == 0;
    }

    fn or(&mut self, value: u8) {
        self.registers.a = self.registers.a | value;

        self.registers.f.half_carry = false;
        self.registers.f.carry = false;
        self.registers.f.subtract = false;
        self.registers.f.zero = self.registers.a == 0;
    }

    fn xor(&mut self, value: u8) {
        self.registers.a = self.registers.a ^ value;

        self.registers.f.half_carry = false;
        self.registers.f.carry = false;
        self.registers.f.subtract = false;
        self.registers.f.zero = self.registers.a == 0;
    }

    fn cp(&mut self, value: u8) {
        let (new_value, did_overflow) = self.registers.a.overflowing_sub(value);
        self.registers.f.zero = new_value == 0;
        self.registers.f.subtract = true;
        self.registers.f.carry = did_overflow;
        self.registers.f.half_carry = self.registers.a - (value & 0x0F) < (self.registers.a & 0xF0);
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
        cpu.registers.l = 0x0;
        cpu.bus.write_byte(0x0000, 0x12);

        cpu.execute(Instruction::ADD(ArithmeticTarget::HL));

        assert_eq!(cpu.registers.a, 0x4E);

        let expected_flags = FlagsRegister::new();

        assert_eq!(cpu.registers.f, expected_flags);
    }

    #[test]
    fn test_adc_register_register() {
        let mut cpu = CPU::new();
        cpu.registers.a = 0xE1;
        cpu.registers.e = 0x0F;
        cpu.registers.f.set_carry();

        cpu.execute(Instruction::ADC(ArithmeticTarget::Register(Reg::E)));

        assert_eq!(cpu.registers.a, 0xF1);

        let mut expected_flags = FlagsRegister::new();
        expected_flags.set_half_carry();

        assert_eq!(cpu.registers.f, expected_flags);
    }

    #[test]
    fn test_adc_register_immediate() {
        let mut cpu = CPU::new();
        cpu.registers.a = 0xE1;
        cpu.registers.f.set_carry();
        let n = 0x3B;

        cpu.execute(Instruction::ADC(ArithmeticTarget::Immediate(n)));

        assert_eq!(cpu.registers.a, 0x1D);

        let mut expected_flags = FlagsRegister::new();
        expected_flags.set_carry();

        assert_eq!(cpu.registers.f, expected_flags);
    }

    #[test]
    fn test_adc_register_hl() {
        let mut cpu = CPU::new();
        cpu.registers.f.set_carry();
        cpu.registers.a = 0xE1;
        cpu.registers.h = 0x0;
        cpu.registers.l = 0x1;
        cpu.bus.write_byte(0x0001, 0x1E);

        cpu.execute(Instruction::ADC(ArithmeticTarget::HL));

        assert_eq!(cpu.registers.a, 0x00);

        let mut expected_flags = FlagsRegister::new();
        expected_flags.set_zero().set_half_carry().set_carry();

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

    #[test]
    fn test_ldhl() {
        let mut cpu = CPU::new();
        cpu.sp = 0xFFF8;
        cpu.execute(Instruction::LDHL(2));
        assert_eq!(cpu.registers.get_hl(), 0xFFFA);
        let expected_flags = FlagsRegister::new();

        assert_eq!(cpu.registers.f, expected_flags);
    }

    #[test]
    fn test_ldhl_overflow() {
        let mut cpu = CPU::new();
        cpu.sp = 0xFFF8;
        cpu.execute(Instruction::LDHL(9));
        assert_eq!(cpu.registers.get_hl(), 0x0001);
        let mut expected_flags = FlagsRegister::new();
        expected_flags.set_carry().set_half_carry();

        assert_eq!(cpu.registers.f, expected_flags);
    }

    #[test]
    fn test_ldhl_half_overflow() {
        let mut cpu = CPU::new();
        cpu.sp = 0x0FFF;
        cpu.execute(Instruction::LDHL(1));
        assert_eq!(cpu.registers.get_hl(), 0x1000);
        let mut expected_flags = FlagsRegister::new();
        expected_flags.set_half_carry();

        assert_eq!(cpu.registers.f, expected_flags);
    }

    #[test]
    fn test_ldhl_subtract() {
        let mut cpu = CPU::new();
        cpu.sp = 0x0FFF;
        cpu.execute(Instruction::LDHL(0b11111111));
        assert_eq!(cpu.registers.get_hl(), 0x0FFE);
        let mut expected_flags = FlagsRegister::new();
        expected_flags.set_half_carry().set_carry();

        assert_eq!(cpu.registers.f, expected_flags);
    }

    #[test]
    fn test_sub_register_register() {
        let mut cpu = CPU::new();
        cpu.registers.a = 0x3E;
        cpu.registers.e = 0x3E;
        cpu.registers.f.set_carry();

        cpu.execute(Instruction::SUB(ArithmeticTarget::Register(Reg::E)));

        assert_eq!(cpu.registers.a, 0x00);

        let mut expected_flags = FlagsRegister::new();
        expected_flags.set_zero().set_subtract();

        assert_eq!(cpu.registers.f, expected_flags);
    }

    #[test]
    fn test_sub_register_immediate() {
        let mut cpu = CPU::new();
        cpu.registers.a = 0x3E;
        cpu.registers.f.set_carry();
        let n = 0x0F;

        cpu.execute(Instruction::SUB(ArithmeticTarget::Immediate(n)));

        assert_eq!(cpu.registers.a, 0x2F);

        let mut expected_flags = FlagsRegister::new();
        expected_flags.set_half_carry().set_subtract();

        assert_eq!(cpu.registers.f, expected_flags);
    }

    #[test]
    fn test_sub_register_hl() {
        let mut cpu = CPU::new();
        cpu.registers.f.set_carry();
        cpu.registers.a = 0x3E;
        cpu.registers.h = 0x0;
        cpu.registers.l = 0x1;
        cpu.bus.write_byte(0x0001, 0x40);

        cpu.execute(Instruction::SUB(ArithmeticTarget::HL));

        assert_eq!(cpu.registers.a, 0xFE);

        let mut expected_flags = FlagsRegister::new();
        expected_flags.set_carry().set_subtract();

        assert_eq!(cpu.registers.f, expected_flags);
    }

    #[test]
    fn test_sbc_register_register() {
        let mut cpu = CPU::new();
        cpu.registers.a = 0x3B;
        cpu.registers.h = 0x2A;
        cpu.registers.f.set_carry();

        cpu.execute(Instruction::SBC(ArithmeticTarget::Register(Reg::H)));

        assert_eq!(cpu.registers.a, 0x10);

        let mut expected_flags = FlagsRegister::new();
        expected_flags.set_subtract();

        assert_eq!(cpu.registers.f, expected_flags);
    }

    #[test]
    fn test_sbc_register_immediate() {
        let mut cpu = CPU::new();
        cpu.registers.a = 0x3B;
        cpu.registers.f.set_carry();
        let n = 0x3A;

        cpu.execute(Instruction::SBC(ArithmeticTarget::Immediate(n)));

        assert_eq!(cpu.registers.a, 0x00);

        let mut expected_flags = FlagsRegister::new();
        expected_flags.set_zero().set_subtract();

        assert_eq!(cpu.registers.f, expected_flags);
    }

    #[test]
    fn test_sbc_register_hl() {
        let mut cpu = CPU::new();
        cpu.registers.f.set_carry();
        cpu.registers.a = 0x3B;
        cpu.registers.h = 0x0;
        cpu.registers.l = 0x1;
        cpu.bus.write_byte(0x0001, 0x4F);

        cpu.execute(Instruction::SBC(ArithmeticTarget::HL));

        assert_eq!(cpu.registers.a, 0xEB);

        let mut expected_flags = FlagsRegister::new();
        expected_flags.set_subtract().set_half_carry().set_carry();

        assert_eq!(cpu.registers.f, expected_flags);
    }

    #[test]
    fn test_and_register_register() {
        let mut cpu = CPU::new();
        cpu.registers.a = 0x5A;
        cpu.registers.l = 0x3F;

        cpu.execute(Instruction::AND(ArithmeticTarget::Register(Reg::L)));

        assert_eq!(cpu.registers.a, 0x1A);

        let mut expected_flags = FlagsRegister::new();
        expected_flags.set_half_carry();

        assert_eq!(cpu.registers.f, expected_flags);
    }

    #[test]
    fn test_and_register_immediate() {
        let mut cpu = CPU::new();
        cpu.registers.a = 0x5A;
        let n = 0x38;

        cpu.execute(Instruction::AND(ArithmeticTarget::Immediate(n)));

        assert_eq!(cpu.registers.a, 0x18);

        let mut expected_flags = FlagsRegister::new();
        expected_flags.set_half_carry();

        assert_eq!(cpu.registers.f, expected_flags);
    }

    #[test]
    fn test_and_register_hl() {
        let mut cpu = CPU::new();
        cpu.registers.a = 0x5A;
        cpu.registers.h = 0x0;
        cpu.registers.l = 0x1;
        cpu.bus.write_byte(0x0001, 0x0);

        cpu.execute(Instruction::AND(ArithmeticTarget::HL));

        assert_eq!(cpu.registers.a, 0x00);

        let mut expected_flags = FlagsRegister::new();
        expected_flags.set_half_carry().set_zero();

        assert_eq!(cpu.registers.f, expected_flags);
    }

    #[test]
    fn test_or_register_register() {
        let mut cpu = CPU::new();
        cpu.registers.a = 0x5A;

        cpu.execute(Instruction::OR(ArithmeticTarget::Register(Reg::A)));

        assert_eq!(cpu.registers.a, 0x5A);

        let expected_flags = FlagsRegister::new();

        assert_eq!(cpu.registers.f, expected_flags);
    }

    #[test]
    fn test_or_register_immediate() {
        let mut cpu = CPU::new();
        cpu.registers.a = 0x5A;
        let n = 0x03;

        cpu.execute(Instruction::OR(ArithmeticTarget::Immediate(n)));

        assert_eq!(cpu.registers.a, 0x5B);

        let expected_flags = FlagsRegister::new();

        assert_eq!(cpu.registers.f, expected_flags);
    }

    #[test]
    fn test_or_register_hl() {
        let mut cpu = CPU::new();
        cpu.registers.a = 0x5A;
        cpu.registers.h = 0x0;
        cpu.registers.l = 0x1;
        cpu.bus.write_byte(0x0001, 0x0F);

        cpu.execute(Instruction::OR(ArithmeticTarget::HL));

        assert_eq!(cpu.registers.a, 0x5F);

        let expected_flags = FlagsRegister::new();

        assert_eq!(cpu.registers.f, expected_flags);
    }

    #[test]
    fn test_xor_register_register() {
        let mut cpu = CPU::new();
        cpu.registers.a = 0xFF;

        cpu.execute(Instruction::XOR(ArithmeticTarget::Register(Reg::A)));

        assert_eq!(cpu.registers.a, 0x00);

        let mut expected_flags = FlagsRegister::new();
        expected_flags.set_zero();

        assert_eq!(cpu.registers.f, expected_flags);
    }

    #[test]
    fn test_xor_register_immediate() {
        let mut cpu = CPU::new();
        cpu.registers.a = 0xFF;
        let n = 0x0F;

        cpu.execute(Instruction::XOR(ArithmeticTarget::Immediate(n)));

        assert_eq!(cpu.registers.a, 0xF0);

        let expected_flags = FlagsRegister::new();

        assert_eq!(cpu.registers.f, expected_flags);
    }

    #[test]
    fn test_xor_register_hl() {
        let mut cpu = CPU::new();
        cpu.registers.a = 0xFF;
        cpu.registers.h = 0x0;
        cpu.registers.l = 0x1;
        cpu.bus.write_byte(0x0001, 0x8A);

        cpu.execute(Instruction::XOR(ArithmeticTarget::HL));

        assert_eq!(cpu.registers.a, 0x75);

        let expected_flags = FlagsRegister::new();

        assert_eq!(cpu.registers.f, expected_flags);
    }

    #[test]
    fn test_cp_register_register() {
        let mut cpu = CPU::new();
        cpu.registers.a = 0x3C;
        cpu.registers.b = 0x2F;

        cpu.execute(Instruction::CP(ArithmeticTarget::Register(Reg::B)));

        assert_eq!(cpu.registers.a, 0x3C);

        let mut expected_flags = FlagsRegister::new();
        expected_flags.set_half_carry().set_subtract();

        assert_eq!(cpu.registers.f, expected_flags);
    }

    #[test]
    fn test_cp_register_immediate() {
        let mut cpu = CPU::new();
        cpu.registers.a = 0x3C;
        let n = 0x3C;

        cpu.execute(Instruction::CP(ArithmeticTarget::Immediate(n)));

        assert_eq!(cpu.registers.a, 0x3C);

        let mut expected_flags = FlagsRegister::new();
        expected_flags.set_zero().set_subtract();

        assert_eq!(cpu.registers.f, expected_flags);
    }

    #[test]
    fn test_cp_register_hl() {
        let mut cpu = CPU::new();
        cpu.registers.a = 0x3C;
        cpu.registers.h = 0x0;
        cpu.registers.l = 0x1;
        cpu.bus.write_byte(0x0001, 0x40);

        cpu.execute(Instruction::CP(ArithmeticTarget::HL));

        assert_eq!(cpu.registers.a, 0x3C);

        let mut expected_flags = FlagsRegister::new();
        expected_flags.set_subtract().set_carry();

        assert_eq!(cpu.registers.f, expected_flags);
    }

    #[test]
    fn test_inc_register() {
        let mut cpu = CPU::new();
        cpu.registers.a = 0xFF;

        cpu.execute(Instruction::INC(ArithmeticTarget::Register(Reg::A)));

        assert_eq!(cpu.registers.a, 0x0);

        let mut expected_flags = FlagsRegister::new();
        expected_flags.set_zero().set_half_carry();

        assert_eq!(cpu.registers.f, expected_flags);
    }

    #[test]
    fn test_inc_hl() {
        let mut cpu = CPU::new();
        cpu.registers.h = 0x0;
        cpu.registers.l = 0x1;
        cpu.bus.write_byte(0x0001, 0x50);

        cpu.execute(Instruction::INC(ArithmeticTarget::HL));

        let expected_flags = FlagsRegister::new();

        assert_eq!(cpu.registers.f, expected_flags);
    }

    #[test]
    fn test_dec_register() {
        let mut cpu = CPU::new();
        cpu.registers.l = 0x01;

        cpu.execute(Instruction::DEC(ArithmeticTarget::Register(Reg::L)));

        assert_eq!(cpu.registers.l, 0x0);

        let mut expected_flags = FlagsRegister::new();
        expected_flags.set_zero().set_subtract();

        assert_eq!(cpu.registers.f, expected_flags);
    }

    #[test]
    fn test_dec_hl() {
        let mut cpu = CPU::new();
        cpu.registers.h = 0x0;
        cpu.registers.l = 0x1;
        cpu.bus.write_byte(0x0001, 0x00);

        cpu.execute(Instruction::DEC(ArithmeticTarget::HL));

        let mut expected_flags = FlagsRegister::new();
        expected_flags.set_half_carry().set_subtract();

        assert_eq!(cpu.registers.f, expected_flags);
    }

    #[test]
    fn test_big_add_registers() {
        let mut cpu = CPU::new();
        cpu.registers.set_hl(0x8A23);
        cpu.registers.set_bc(0x0605);

        cpu.execute(Instruction::BIGADD(BigArithmeticTarget::Registers(
            RegPair::BC,
        )));

        assert_eq!(cpu.registers.get_hl(), 0x9028);

        let mut expected_flags = FlagsRegister::new();
        expected_flags.set_half_carry();

        assert_eq!(cpu.registers.f, expected_flags);

        cpu.registers.set_hl(0x8A23);
        cpu.execute(Instruction::BIGADD(BigArithmeticTarget::Registers(
            RegPair::HL,
        )));

        assert_eq!(cpu.registers.get_hl(), 0x1446);

        expected_flags.set_carry();
        assert_eq!(cpu.registers.f, expected_flags);
    }

    #[test]
    fn test_big_add_operand() {
        let mut cpu = CPU::new();
        cpu.sp = 0xFFF8;

        cpu.execute(Instruction::BIGADD(BigArithmeticTarget::Operand(0x2)));

        assert_eq!(cpu.sp, 0xFFFA);

        let expected_flags = FlagsRegister::new();

        assert_eq!(cpu.registers.f, expected_flags);
    }

    #[test]
    fn test_big_inc_operand() {
        let mut cpu = CPU::new();
        cpu.registers.set_de(0x235F);

        cpu.execute(Instruction::BIGINC(RegPair::DE));

        assert_eq!(cpu.registers.get_de(), 0x2360);
    }

    #[test]
    fn test_big_dec_operand() {
        let mut cpu = CPU::new();
        cpu.registers.set_de(0x235F);

        cpu.execute(Instruction::BIGDEC(RegPair::DE));

        assert_eq!(cpu.registers.get_de(), 0x235E);
    }

    #[test]
    fn test_rlca() {
        let mut cpu = CPU::new();
        cpu.registers.a = 0x85;

        cpu.execute(Instruction::RLCA);

        assert_eq!(cpu.registers.a, 0x0A);

        let mut expected_flags = FlagsRegister::new();
        expected_flags.set_carry();

        assert_eq!(cpu.registers.f, expected_flags);
    }

    #[test]
    fn test_rla() {
        let mut cpu = CPU::new();
        cpu.registers.a = 0x95;
        cpu.registers.f.set_carry();

        cpu.execute(Instruction::RLA);

        assert_eq!(cpu.registers.a, 0x2B);

        let mut expected_flags = FlagsRegister::new();
        expected_flags.set_carry();

        assert_eq!(cpu.registers.f, expected_flags);
    }

    #[test]
    fn test_rrca() {
        let mut cpu = CPU::new();
        cpu.registers.a = 0x3B;

        cpu.execute(Instruction::RRCA);

        assert_eq!(cpu.registers.a, 0x9D);

        let mut expected_flags = FlagsRegister::new();
        expected_flags.set_carry();

        assert_eq!(cpu.registers.f, expected_flags);
    }

    #[test]
    fn test_rra() {
        let mut cpu = CPU::new();
        cpu.registers.a = 0x81;

        cpu.execute(Instruction::RRA);

        assert_eq!(cpu.registers.a, 0x40);

        let mut expected_flags = FlagsRegister::new();
        expected_flags.set_carry();

        assert_eq!(cpu.registers.f, expected_flags);
    }
}
