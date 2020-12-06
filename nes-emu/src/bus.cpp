//INTERNAL
#include "bus.hpp"

uint8_t nes_emu::bus::read(const uint16_t address, bool read_only) const
{
	return (*ram)[address];
}

void nes_emu::bus::write(const uint16_t address, const uint8_t data) const
{
	if (address >= 0x000 && address <= 0xFFFF)
		(*ram)[address] = data;
}