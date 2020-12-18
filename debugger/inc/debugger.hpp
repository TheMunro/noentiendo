#pragma once

#include "nes.hpp"

class debugger
{
public:
	debugger();
	
	void run();

private:
	std::unique_ptr<nes_emu::nes> nes;
};