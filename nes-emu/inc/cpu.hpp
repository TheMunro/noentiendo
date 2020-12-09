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
	uint8_t cycles = 0;

	bool (cpu::*execute)() = nullptr;
	bool (cpu::*addressing_mode)() = nullptr;
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
	[[nodiscard]] std::uint8_t fetch();
	[[nodiscard]] std::uint8_t read(std::uint16_t address, bool read_only = false) const;
	void write(std::uint16_t address, std::uint8_t data) const;

	//addressing
	//http://www.emulator101.com/6502-addressing-modes.html
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
	bool instruction_adc();
	bool instruction_and();
	bool instruction_asl();
	bool instruction_bcc();
	bool instruction_bcs();
	bool instruction_beq();
	bool instruction_bit();
	bool instruction_bmi();
	bool instruction_bne();
	bool instruction_bpl();
	bool instruction_brk();
	bool instruction_bvc();
	bool instruction_bvs();
	bool instruction_clc();
	bool instruction_cld();
	bool instruction_cli();
	bool instruction_clv();
	bool instruction_cmp();
	bool instruction_cpx();
	bool instruction_cpy();
	bool instruction_dec();
	bool instruction_dex();
	bool instruction_dey();
	bool instruction_eor();
	bool instruction_inc();
	bool instruction_inx();
	bool instruction_iny();
	bool instruction_jmp();
	bool instruction_jsr();
	bool instruction_lda();
	bool instruction_ldx();
	bool instruction_ldy();
	bool instruction_lsr();
	bool instruction_nop();
	bool instruction_ora();
	bool instruction_pha();
	bool instruction_php();
	bool instruction_pla();
	bool instruction_plp();
	bool instruction_rol();
	bool instruction_ror();
	bool instruction_rti();
	bool instruction_rts();
	bool instruction_sbc();
	bool instruction_sec();
	bool instruction_sed();
	bool instruction_sei();
	bool instruction_sta();
	bool instruction_stx();
	bool instruction_sty();
	bool instruction_tax();
	bool instruction_tay();
	bool instruction_tsx();
	bool instruction_txa();
	bool instruction_txs();
	bool instruction_tya();
	
private:
	[[nodiscard]] static constexpr std::array<instruction, 256> build_instructions();
	
	const bus& bus;
	const std::array<instruction, 256> instructions;

	//registers
	std::uint8_t register_accumulator = 0x00; 
	std::uint8_t register_x = 0x00;
	std::uint8_t register_y = 0x00;
	std::uint8_t register_stack_pointer = 0x00;
	std::uint16_t register_program_counter = 0x0000;
	processor_status_register register_status = processor_status_register::none;

	//internals
	std::uint8_t fetched = 0x00;                //working data for current instruction, if required
	std::uint16_t address_absolute = 0x0000;    //used for direct addressing
	std::uint16_t address_relative = 0x0000;	   //used for branching operations [-127, 128]
	std::uint8_t opcode = 0x00;                 //current instruction
	std::uint8_t cycles_remaining = 0;          //remaining cycles for current instruction
	std::uint32_t clock_count = 0;              //clock count accumulator
};
}	 // namespace nes