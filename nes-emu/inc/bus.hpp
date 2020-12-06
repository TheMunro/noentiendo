#pragma once

//STDLIB
#include <array>
#include <memory>
#include <cstdint>

namespace nes
{
constexpr std::int32_t address_ram_size = 0xFFFF;

class bus
{
public:
	bus()
		: ram{std::make_unique<std::array<uint8_t, address_ram_size>>()}
	{
	}
	
	uint8_t read(uint16_t address, bool read_only = false) const;
	void write(uint16_t address, uint8_t data) const;

private:
	std::unique_ptr<std::array<uint8_t, address_ram_size>> ram;
};
}	 // namespace nes