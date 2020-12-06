//INTERNAL
#include "bus.hpp"

uint8_t nes::bus::read(const uint16_t address, bool read_only) const
{
	return (*ram)[address];
}
void nes::bus::write(const uint16_t address, const uint8_t data) const
{
	if (address >= 0x000 && address <= 0xFFFF)
		(*ram)[address] = data;
}
