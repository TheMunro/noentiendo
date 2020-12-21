#pragma once

//STDLIB
#include <memory>
#include <sstream>

//LOCAL
#include "cpu.hpp"
#include "bus.hpp"
#include "cartridge.hpp"

namespace noentiendo
{
class nes
{
public:
	[[nodiscard]] static std::unique_ptr<nes> build()
	{
		//populate motherboard
		auto bus = std::make_unique<noentiendo::bus>();
		
		//solder cpu
		auto cpu = std::make_unique<noentiendo::cpu>(*bus);
		
		//solder peripherals
		//package nicely
		//ship to distributor
		//ship to retail outlet
		//sell to consumer
		struct built_nes : nes
		{
			//https://seanmiddleditch.com/enabling-make-unique-with-private-constructors/
			built_nes(std::unique_ptr<noentiendo::cpu> _cpu, std::unique_ptr<noentiendo::bus> _bus)
				: nes{std::move(_cpu), std::move(_bus)}
			{
			}
		};
		
		return std::make_unique<built_nes>(std::move(cpu), std::move(bus));
	}

	[[nodiscard]] cpu* get_cpu() const { return cpu.get(); }
	[[nodiscard]] bus* get_bus() const { return bus.get(); }
	[[nodiscard]] bool is_active() const { return active; }
	//[[nodiscard]] const std::map<uint16_t, std::string>& get_disassembled_bytecode() const { return disassembled_bytecode; }
	[[nodiscard]] std::vector<std::string> disassemble() const { return cpu->get_current_execution_window(); }

	//TODO: Remove this hacky hack!
	void insert_cartridge()
	{
		//HACK: temporary
		std::stringstream ss;
		ss << "A2 0A 8E 00 00 A2 03 8E 01 00 AC 00 00 A9 00 18 6D 01 00 88 D0 FA 8D 02 00 EA EA EA";
		uint16_t offset = 0x8000;
		while (!ss.eof())
		{
			std::string b;
			ss >> b;
			(*bus->get_ram())[offset++] = static_cast<uint8_t>(std::stoul(b, nullptr, 16));
		}

		(*bus->get_ram())[0xFFFC] = 0x00;
		(*bus->get_ram())[0xFFFD] = 0x80;
		
		cpu->reset();
		active = true;
	}

private:
	nes(std::unique_ptr<cpu> _cpu, std::unique_ptr<bus> _bus)
		: cpu{std::move(_cpu)}
		, bus{std::move(_bus)}
		, active{false}
	{
	}

	bool active;
	std::unique_ptr<cpu> cpu;
	std::unique_ptr<bus> bus;
	//std::map<uint16_t, std::string> disassembled_bytecode;
};
}	 // namespace nes