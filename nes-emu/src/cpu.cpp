//INTERNAL
#include "cpu.hpp"

uint8_t nes_emu::cpu::read(const uint16_t address, bool read_only) const
{
	return bus.read(address, read_only);
}

void nes_emu::cpu::write(const uint16_t address, const uint8_t data) const
{
	return bus.write(address, data);
}

constexpr std::array<nes_emu::instruction, 256> nes_emu::cpu::build_instructions()
{
	std::array<instruction, 256> instructions{};
	instructions[0x00] = instruction{opcode::BRK, false, nullptr, nullptr};
	
	return instructions;
}

bool nes_emu::cpu::address_mode_implicit()
{
	//working data determined by instruction
	
	//program counter increment controlled by instruction
	return false;
}

bool nes_emu::cpu::address_mode_accumulator()
{
	//accumulator addressing modes skip fetch step as register value is already populated
	//set current working data to accumulator
	fetched = register_accumulator;
	
	//program counter increment controlled by instruction
	return false;
}

bool nes_emu::cpu::address_mode_immediate()
{
	//cache current program data as address
	address_absolute = register_program_counter;
	
	//program counter increment to prepare next instruction
	register_program_counter++;
	return false;
}

bool nes_emu::cpu::address_mode_zero_page()
{
	const auto low_byte = read(register_program_counter);
	address_absolute = low_byte;
	
	//program counter increment to prepare next instruction
	register_program_counter++;
	return false;
}

bool nes_emu::cpu::address_mode_zero_page_x()
{
	const auto low_byte = read(register_program_counter);
	address_absolute = (low_byte + register_x) & 0x00FF;

	//program counter increment to prepare next instruction
	register_program_counter++;
	return false;
}

bool nes_emu::cpu::address_mode_zero_page_y()
{
	const auto low_byte = read(register_program_counter);
	address_absolute = (low_byte + register_y) & 0x00FF;

	//program counter increment to prepare next instruction
	register_program_counter++;
	return false;
}

bool nes_emu::cpu::address_mode_relative()
{
	address_relative = read(register_program_counter);
	register_program_counter++;
	
	//TODO: there is a weird bit in https://github.com/OneLoneCoder/olcNES/blob/master/Part%232%20-%20CPU/olc6502.cpp
	//TODO: for this that it setting some upper byte that is then only being used to check for overflow,
	//TODO: can this be handled more cleanly elsewhere?
	return false;
}

bool nes_emu::cpu::address_mode_absolute()
{
	const uint16_t lo_byte = read(register_program_counter);
	register_program_counter++;

	const uint16_t hi_byte = read(register_program_counter);
	register_program_counter++;

	address_absolute = (hi_byte << 0xFF) + lo_byte;
	
	return false;
}

bool nes_emu::cpu::address_mode_absolute_x()
{
	address_mode_absolute();

	const auto hi_byte = address_absolute & 0xFF00;
	address_absolute += register_x;
	if ((hi_byte & (address_absolute & 0xFF00)) == 0xFF00)
		return true; //new page, use result to increase cycle count

	return false;
}

bool nes_emu::cpu::address_mode_absolute_y()
{
	address_mode_absolute();

	const auto hi_byte = address_absolute & 0xFF00;
	address_absolute += register_y;
	if ((hi_byte & (address_absolute & 0xFF00)) == 0xFF00)
		return true;	//new page, use result to increase cycle count

	return false;
}

bool nes_emu::cpu::address_mode_indirect()
{
	return false;
}

bool nes_emu::cpu::address_mode_indexed_indirect()
{
	return false;
}

bool nes_emu::cpu::address_mode_indirect_indexed()
{
	return false;
}
