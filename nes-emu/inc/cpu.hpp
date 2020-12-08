#pragma once

//STDLIB
#include <cstdint>
#include <vector>

//INTERNAL
#include "bus.hpp"

namespace nes_emu
{
enum class processor_status_register : std::uint8_t
{
	none            = 0,
	carry           = 1 << 0, //C
	zero            = 1 << 1, //Z
	irq_disable     = 1 << 2, //I
	decimal_mode    = 1 << 3, //D
	break_command   = 1 << 4, //B
	unused          = 1 << 5,
	overflow        = 1 << 6, //V
	negative        = 1 << 7  //N
};

enum class opcode
{
	BRK
	//...and all the rest
};

class cpu;
class instruction
{
public:
	opcode opcode;
	bool complete = false;
	//uint8_t size;
	//uint8_t duration;

	uint8_t (cpu::*execute)() = nullptr;
	uint8_t (cpu::*addressing_mode)() = nullptr;
};


class cpu
{
public:
	cpu(const bus& bus)
		: bus{bus}
		, instructions{build_instructions()}
	{
	}

	//inputs
	void clock();
	void reset();
	void interrupt();
	void non_maskable_interrupt();

	//data
	[[nodiscard]] uint8_t fetch();
	[[nodiscard]] uint8_t read(uint16_t address, bool read_only = false) const;
	void write(uint16_t address, uint8_t data) const;


	//addressing
	bool address_mode_implicit();
	bool address_mode_accumulator();
	bool address_mode_immediate();
	
	bool address_mode_zero_page();
	bool address_mode_zero_page_x();
	bool address_mode_zero_page_y();
	
	bool address_mode_relative();
	bool address_mode_absolute();
	bool address_mode_absolute_x();
	bool address_mode_absolute_y();
	
	bool address_mode_indirect();
	bool address_mode_indexed_indirect();
	bool address_mode_indirect_indexed();


	//opcodes
	
private:
	[[nodiscard]] static constexpr std::array<instruction, 256> build_instructions();
	
	const bus& bus;
	const std::array<instruction, 256> instructions;

	//registers
	uint8_t register_accumulator = 0x00; 
	uint8_t register_x = 0x00;
	uint8_t register_y = 0x00;
	uint8_t register_stack_pointer = 0x00;
	uint16_t register_program_counter = 0x0000;
	processor_status_register register_status = processor_status_register::none;

	//internals
	uint8_t fetched = 0x00;                //working data for current instruction, if required
	uint16_t address_absolute = 0x0000;    //used for direct addressing
	uint8_t address_relative = 0x00;       //used for branching operations [-127, 128]
	uint8_t opcode = 0x00;                 //current instruction
	uint8_t cycles_remaining = 0;          //remaining cycles for current instruction
	uint32_t clock_count = 0;              //clock count accumulator
};
}	 // namespace nes