#pragma once

//STDLIB
#include <cstdint>

//INTERNAL
#include "bus.hpp"

namespace nes_emu
{
class cpu
{
public:
	cpu(const bus& bus)
		: bus{bus}
	{
	}

	uint8_t read(uint16_t address, bool read_only = false) const;
	void write(uint16_t address, uint8_t data) const;

private:
	const bus& bus;
};
}	 // namespace nes