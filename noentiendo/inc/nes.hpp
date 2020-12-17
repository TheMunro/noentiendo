#pragma once

//STDLIB
#include <memory>

#include "cpu.hpp"
#include "bus.hpp"

namespace nes_emu
{
class nes
{
public:
	[[nodiscard]] static std::unique_ptr<nes> build()
	{
		//populate motherboard
		auto bus = std::make_unique<nes_emu::bus>();
		
		//solder cpu
		auto cpu = std::make_unique<nes_emu::cpu>(*bus);
		
		//solder peripherals
		//package nicely
		//ship to distributor
		//ship to retail outlet
		//sell to consumer
		struct built_nes : nes
		{
			//https://seanmiddleditch.com/enabling-make-unique-with-private-constructors/
			built_nes(std::unique_ptr<nes_emu::cpu> _cpu, std::unique_ptr<nes_emu::bus> _bus)
				: nes{std::move(_cpu), std::move(_bus)}
			{
			}
		};
		
		return std::make_unique<built_nes>(std::move(cpu), std::move(bus));
	}

private:
	nes(std::unique_ptr<cpu> _cpu, std::unique_ptr<bus> _bus)
		: cpu{std::move(_cpu)}
		, bus{std::move(_bus)}
	{
	}

	std::unique_ptr<cpu> cpu;
	std::unique_ptr<bus> bus;
};
}	 // namespace nes