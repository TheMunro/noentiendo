//INTERNAL
#include "bus.hpp"

std::uint8_t noentiendo::bus::read(const std::uint16_t address, bool read_only) const
{
	return (*ram)[address];
}

void noentiendo::bus::write(const std::uint16_t address, const std::uint8_t data) const
{
	if (address >= 0x000 && address <= 0xFFFF)
		(*ram)[address] = data;
}