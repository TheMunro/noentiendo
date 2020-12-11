//INTERNAL
#include "cpu.hpp"

std::uint8_t nes_emu::cpu::read(const std::uint16_t address, bool read_only) const
{
	return bus.read(address, read_only);
}

void nes_emu::cpu::write(const std::uint16_t address, const std::uint8_t data) const
{
	return bus.write(address, data);
}

void nes_emu::cpu::clock()
{
	if (cycles_remaining == 0)
	{
		opcode = read(register_program_counter);
		register_program_counter++;

		const auto& instruction = instructions[opcode];
		cycles_remaining = instruction.cycles;

		//can't use std::invoke unless I have the method name here... templates perhaps?
		const auto additional_cycle_addressing_mode = (this->*instruction.addressing_mode)();
		const auto additional_cycle_execute = (this->*instruction.execute)();

		cycles_remaining += (additional_cycle_addressing_mode & additional_cycle_execute);
	}

	clock_count++;
	cycles_remaining--;
}

void nes_emu::cpu::reset()
{
}

void nes_emu::cpu::interrupt()
{	
}

void nes_emu::cpu::non_maskable_interrupt()
{
}

void nes_emu::cpu::fetch()
{
	//oh no, this is horrible!
	if (!(instructions[opcode].addressing_mode == &nes_emu::cpu::address_mode_implicit))
		fetched = read(register_accumulator);
}

constexpr std::array<nes_emu::instruction, 256> nes_emu::cpu::build_instructions()
{
	std::array<instruction, 256> instructions{};

	//AND
	instructions[0x61] = instruction{opcode::AND, false, 2, 6, &cpu::instruction_and, &cpu::address_mode_indexed_indirect};
	instructions[0x65] = instruction{opcode::AND, false, 2, 3, &cpu::instruction_and, &cpu::address_mode_zero_page};
	instructions[0x69] = instruction{opcode::AND, false, 2, 2, &cpu::instruction_and, &cpu::address_mode_immediate};
	instructions[0x6D] = instruction{opcode::AND, false, 3, 4, &cpu::instruction_and, &cpu::address_mode_absolute};
	instructions[0x71] = instruction{opcode::AND, false, 2, 5, &cpu::instruction_and, &cpu::address_mode_indirect_indexed};
	instructions[0x75] = instruction{opcode::AND, false, 2, 4, &cpu::instruction_and, &cpu::address_mode_zero_page_x};
	instructions[0x79] = instruction{opcode::AND, false, 3, 4, &cpu::instruction_and, &cpu::address_mode_absolute_y};
	instructions[0x7D] = instruction{opcode::AND, false, 3, 4, &cpu::instruction_and, &cpu::address_mode_absolute_x};
	
	return instructions;
}

bool nes_emu::cpu::address_mode_implicit()
{
	//In an implied instruction, the data and/or destination is mandatory for the instruction. For example,
	//the CLC instruction is implied, it is going to clear the processor's Carry flag.
	
	//working data determined by instruction
	
	//program counter increment controlled by instruction
	return false;
}

bool nes_emu::cpu::address_mode_accumulator()
{
	//These instructions have register A (the accumulator) as the target.
	
	//accumulator addressing modes skip fetch step as register value is already populated
	//set current working data to accumulator
	fetched = register_accumulator;
	
	//program counter increment controlled by instruction
	return false;
}

bool nes_emu::cpu::address_mode_immediate()
{
	//These instructions have their data defined as the next byte after the opcode. ORA #$B2 will perform a
	//logical (also called bitwise) of the value B2 with the accumulator.
	//
	//cache current program data as address
	address_absolute = register_program_counter;
	
	//program counter increment to prepare next instruction
	register_program_counter++;
	return false;
}

bool nes_emu::cpu::address_mode_zero_page()
{
	//Zero-Page is an addressing mode that is only capable of addressing the first 256 bytes of the CPU's memory map.
	//You can think of it as absolute addressing for the first 256 bytes. The instruction LDA $35 will put the value
	//stored in memory location $35 into A. The advantage of zero-page are two - the instruction takes one less byte
	//to specify, and it executes in less CPU cycles. Most programs are written to store the most frequently used
	//variables in the first 256 memory locations so they can take advantage of zero page addressing.
	
	const auto low_byte = read(register_program_counter);
	address_absolute = low_byte;
	
	//program counter increment to prepare next instruction
	register_program_counter++;
	return false;
}

bool nes_emu::cpu::address_mode_zero_page_x()
{
	//This works just like absolute indexed, but the target address is limited to the first 0xFF bytes.
	//The target address will wrap around and will always be in the zero page.If the instruction is LDA $C0, X,
	//and X is $60, then the target address will be $20.$C0 + $60 = $120, but the carry is discarded in the
	//calculation of the target address.
	
	const auto low_byte = read(register_program_counter);
	address_absolute = (low_byte + register_x) & 0x00FF;

	//program counter increment to prepare next instruction
	register_program_counter++;
	return false;
}

bool nes_emu::cpu::address_mode_zero_page_y()
{
	//This works just like absolute indexed, but the target address is limited to the first 0xFF bytes.
	//The target address will wrap around and will always be in the zero page.If the instruction is LDA $C0, X,
	//and X is $60, then the target address will be $20.$C0 + $60 = $120, but the carry is discarded in the
	//calculation of the target address.
	
	const auto low_byte = read(register_program_counter);
	address_absolute = (low_byte + register_y) & 0x00FF;

	//program counter increment to prepare next instruction
	register_program_counter++;
	return false;
}

bool nes_emu::cpu::address_mode_relative()
{
	//Relative addressing on the 6502 is only used for branch operations. The byte after the opcode
	//is the branch offset. If the branch is taken, the new address will the the current PC plus the offset.
	//The offset is a signed byte, so it can jump a maximum of 127 bytes forward, or 128 bytes backward.

	address_relative = read(register_program_counter);
	register_program_counter++;
	
	//TODO: there is a weird bit in https://github.com/OneLoneCoder/olcNES/blob/master/Part%232%20-%20CPU/olc6502.cpp
	//TODO: for this that it setting some upper byte that is then only being used to check for overflow,
	//TODO: can this be handled more cleanly elsewhere?
	return false;
}

bool nes_emu::cpu::address_mode_absolute()
{
	//Absolute addressing specifies the memory location explicitly in the two bytes following the
	//opcode. So JMP $4032 will set the PC to $4032. The hex for this is 4C 32 40.
	//The 6502 is a little endian machine, so any 16 bit (2 byte) value is stored with the LSB first.
	//All instructions that use absolute addressing are 3 bytes.
	
	const std::uint16_t lo_byte = read(register_program_counter);
	register_program_counter++;

	const std::uint16_t hi_byte = read(register_program_counter);
	register_program_counter++;

	address_absolute = static_cast<std::uint16_t>(hi_byte << 8) + lo_byte;
	
	return false;
}

bool nes_emu::cpu::address_mode_absolute_x()
{
	//This addressing mode makes the target address by adding the contents of the X or Y
	//register to an absolute address. For example, this 6502 code can be used to fill 10 bytes
	//with $FF starting at address $1009, counting down to address $1000.
	
	address_mode_absolute();

	const auto hi_byte = address_absolute & 0xFF00;
	address_absolute += register_x;
	if ((hi_byte & (address_absolute & 0xFF00)) == 0xFF00)
		return true; //new page, use result to increase cycle count

	return false;
}

bool nes_emu::cpu::address_mode_absolute_y()
{
	//This addressing mode makes the target address by adding the contents of the X or Y
	//register to an absolute address. For example, this 6502 code can be used to fill 10 bytes
	//with $FF starting at address $1009, counting down to address $1000.
	
	address_mode_absolute();

	const auto hi_byte = address_absolute & 0xFF00;
	address_absolute += register_y;
	if ((hi_byte & (address_absolute & 0xFF00)) == 0xFF00)
		return true; //new page, use result to increase cycle count

	return false;
}

bool nes_emu::cpu::address_mode_indirect()
{
	//The JMP instruction is the only instruction that uses this addressing mode.
	//It is a 3 byte instruction - the 2nd and 3rd bytes are an absolute address.
	//The set the PC to the address stored at that address. So maybe this would be clearer.

	const std::uint16_t lo_byte = read(register_program_counter);
	register_program_counter++;

	const std::uint16_t hi_byte = read(register_program_counter);
	register_program_counter++;

	address_absolute = static_cast<std::uint16_t>(hi_byte << 8) + lo_byte;
	
	return false;
}

bool nes_emu::cpu::address_mode_indexed_indirect()
{
	//This mode is only used with the X register. Consider a situation where the instruction is LDA ($20,X),
	//X contains $04, and memory at $24 contains 0024: 74 20, First, X is added to $20 to get $24.
	//The target address will be fetched from $24 resulting in a target address of $2074.
	//Register A will be loaded with the contents of memory at $2074.
	//
	//If X + the immediate byte will wrap around to a zero - page address.So you could code that
	//like targetAddress = (X + opcode[1]) & 0xFF.
	//
	//Indexed Indirect instructions are 2 bytes - the second byte is the zero - page address - $20 in
	//the example.Obviously the fetched address has to be stored in the zero page.

	auto ptr = read(register_program_counter);
	register_program_counter++;

	ptr += register_x;

	const std::uint16_t hi_byte = read(ptr) << 8;
	const std::uint16_t lo_byte = read(ptr + 1);

	address_absolute = static_cast<std::uint16_t>(hi_byte << 8) + lo_byte;
	
	return false;
}

bool nes_emu::cpu::address_mode_indirect_indexed()
{
	//This mode is only used with the Y register. It differs in the order that Y is applied to the indirectly
	//fetched address. An example instruction that uses indirect index addressing is LDA ($86),Y .
	//To calculate the target address, the CPU will first fetch the address stored at zero page location $86.
	//That address will be added to register Y to get the final target address. For LDA ($86),Y,
	//if the address stored at $86 is $4028 (memory is 0086: 28 40, remember little endian) and register Y
	//contains $10, then the final target address would be $4038. Register A will be loaded with the
	//contents of memory at $4038.
	//
	//Indirect Indexed instructions are 2 bytes - the second byte is the zero - page address - $86 in the
	//example.(So the fetched address has to be stored in the zero page.)
	//
	//While indexed indirect addressing will only generate a zero - page address, this mode's target address
	//is not wrapped - it can be anywhere in the 16-bit address space.

	const auto ptr = read(register_program_counter);
	register_program_counter++;

	const std::uint16_t hi_byte = read(ptr) << 8;
	const std::uint16_t lo_byte = read(ptr + 1);

	address_absolute = static_cast<std::uint16_t>(hi_byte << 8) + lo_byte;
	
	address_absolute += register_y;

	//check for wraparound
	if ((address_absolute & 0xFF00) != hi_byte)
		return true;
	
	return false;
}

bool nes_emu::cpu::instruction_adc()
{
	//https://www.youtube.com/watch?v=yf_YWiqqv34
	return false;
}

bool nes_emu::cpu::instruction_and()
{
	fetch();
	
	register_accumulator &= fetched;
	set_flag(processor_status_register::zero, register_accumulator == 0x00);
	set_flag(processor_status_register::negative, register_accumulator & 0x80);
	
	return true;
}

bool nes_emu::cpu::instruction_asl()
{
	return false;
}

bool nes_emu::cpu::instruction_bcc()
{
	return false;
}

bool nes_emu::cpu::instruction_bcs()
{
	return false;
}

bool nes_emu::cpu::instruction_beq()
{
	return false;
}

bool nes_emu::cpu::instruction_bit()
{
	return false;
}

bool nes_emu::cpu::instruction_bmi()
{
	return false;
}

bool nes_emu::cpu::instruction_bne()
{
	return false;
}

bool nes_emu::cpu::instruction_bpl()
{
	return false;
}

bool nes_emu::cpu::instruction_brk()
{
	return false;
}

bool nes_emu::cpu::instruction_bvc()
{
	return false;
}

bool nes_emu::cpu::instruction_bvs()
{
	return false;
}

bool nes_emu::cpu::instruction_clc()
{
	return false;
}

bool nes_emu::cpu::instruction_cld()
{
	return false;
}

bool nes_emu::cpu::instruction_cli()
{
	return false;
}

bool nes_emu::cpu::instruction_clv()
{
	return false;
}

bool nes_emu::cpu::instruction_cmp()
{
	return false;
}

bool nes_emu::cpu::instruction_cpx()
{
	return false;
}

bool nes_emu::cpu::instruction_cpy()
{
	return false;
}

bool nes_emu::cpu::instruction_dec()
{
	return false;
}

bool nes_emu::cpu::instruction_dex()
{
	return false;
}

bool nes_emu::cpu::instruction_dey()
{
	return false;
}

bool nes_emu::cpu::instruction_eor()
{
	return false;
}

bool nes_emu::cpu::instruction_inc()
{
	return false;
}

bool nes_emu::cpu::instruction_inx()
{
	return false;
}

bool nes_emu::cpu::instruction_iny()
{
	return false;
}

bool nes_emu::cpu::instruction_jmp()
{
	return false;
}

bool nes_emu::cpu::instruction_jsr()
{
	return false;
}

bool nes_emu::cpu::instruction_lda()
{
	return false;
}

bool nes_emu::cpu::instruction_ldx()
{
	return false;
}

bool nes_emu::cpu::instruction_ldy()
{
	return false;
}

bool nes_emu::cpu::instruction_lsr()
{
	return false;
}

bool nes_emu::cpu::instruction_nop()
{
	return false;
}

bool nes_emu::cpu::instruction_ora()
{
	return false;
}

bool nes_emu::cpu::instruction_pha()
{
	return false;
}

bool nes_emu::cpu::instruction_php()
{
	return false;
}

bool nes_emu::cpu::instruction_pla()
{
	return false;
}

bool nes_emu::cpu::instruction_plp()
{
	return false;
}

bool nes_emu::cpu::instruction_rol()
{
	return false;
}

bool nes_emu::cpu::instruction_ror()
{
	return false;
}

bool nes_emu::cpu::instruction_rti()
{
	return false;
}

bool nes_emu::cpu::instruction_rts()
{
	return false;
}

bool nes_emu::cpu::instruction_sbc()
{
	return false;
}

bool nes_emu::cpu::instruction_sec()
{
	return false;
}

bool nes_emu::cpu::instruction_sed()
{
	return false;
}

bool nes_emu::cpu::instruction_sei()
{
	return false;
}

bool nes_emu::cpu::instruction_sta()
{
	return false;
}

bool nes_emu::cpu::instruction_stx()
{
	return false;
}

bool nes_emu::cpu::instruction_sty()
{
	return false;
}

bool nes_emu::cpu::instruction_tax()
{
	return false;
}

bool nes_emu::cpu::instruction_tay()
{
	return false;
}

bool nes_emu::cpu::instruction_tsx()
{
	return false;
}

bool nes_emu::cpu::instruction_txa()
{
	return false;
}

bool nes_emu::cpu::instruction_txs()
{
	return false;
}

bool nes_emu::cpu::instruction_tya()
{
	return false;
}