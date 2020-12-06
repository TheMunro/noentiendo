#pragma once

//STDLIB
#include <memory>

//INTERNAL
#include "bus.hpp"

namespace nes
{
class cpu
{
	cpu()
		: bus{std::make_unique<nes::bus>()}
	{
	}

	std::unique_ptr<bus> bus;
};
}	 // namespace nes