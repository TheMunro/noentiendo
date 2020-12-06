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