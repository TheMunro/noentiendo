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