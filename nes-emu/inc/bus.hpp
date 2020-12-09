#pragma once

//STDLIB
#include <array>
#include <memory>
#include <cstdint>

namespace nes_emu
{
constexpr std::int32_t address_ram_size = 0xFFFF;

class bus
{
public:
	bus()
		: ram{std::make_unique<std::array<std::uint8_t, address_ram_size>>()}
	{
	}
	
	[[nodiscard]] std::uint8_t read(std::uint16_t address, bool read_only = false) const;
	void write(std::uint16_t address, std::uint8_t data) const;

private:
	//hmm... this should possibly be another peripheral in the nes class
	//and registered with the bus, since the peripheral use the bus, but aren't owned by it?
	std::unique_ptr<std::array<std::uint8_t, address_ram_size>> ram;
};
}	 // namespace nes