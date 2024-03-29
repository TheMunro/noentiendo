#pragma once

//STDLIB
#include <cstdint>
#include <vector>
#include <string>

//INTERNAL
#include "bus.hpp"
#include "bitfield.hpp"

namespace noentiendo
{
enum class processor_status_register : std::uint8_t
{
	none              = 0,
	carry             = 1 << 0, //C
	zero              = 1 << 1, //Z
	interrupt_disable = 1 << 2, //I
	decimal_mode      = 1 << 3, //D
	break_command     = 1 << 4, //B
	unused            = 1 << 5,
	overflow          = 1 << 6, //V
	negative          = 1 << 7  //N
};


enum class opcode
{
	//please don't hurt me for doing this... I'm not like this normally
	None = 0,
	ADC, AND, ASL, BCC,
	BCS, BEQ, BIT, BMI,
	BNE, BPL, BRK, BVC,
	BVS, CLC, CLD, CLI,
	CLV, CMP, CPX, CPY,
	DEC, DEX, DEY, EOR,
	INC, INX, INY, JMP,
	JSR, LDA, LDX, LDY,
	LSR, NOP, ORA, PHA,
	PHP, PLA, PLP, ROL,
	ROR, RTI, RTS, SBC,
	SEC, SED, SEI, STA,
	STX, STY, TAX, TAY,
	TSX, TXA, TXS, TYA,
	XXX
};

enum class addressing_mode
{
	IMPLICIT,
	ACCUMULATOR,
	IMMEDIATE,
	ZERO_PAGE,
	ZERO_PAGE_X,
	ZERO_PAGE_Y,
	RELATIVE,
	ABSOLUTE,
	ABSOLUTE_X,
	ABSOLUTE_Y,
	INDIRECT,
	INDEXED_INDIRECT,
	INDIRECT_INDEXED
};

class cpu;

class instruction
{
protected:
	using ExecuteFunc = void(cpu::*)();

public:
	opcode opcode;
	uint8_t bytes = 0;
	uint8_t cycles = 0;
	addressing_mode addressing_mode; 

	ExecuteFunc execute;
};


class cpu
{
protected:
	using AddressingModeFunc = bool(cpu::*)();

public:
	explicit cpu(const bus& input_bus);

	//inputs
	void clock();
	void reset();
	void irq();
	void nmi();

	//data
	template <noentiendo::cpu::AddressingModeFunc Mode>
	void fetch();
	[[nodiscard]] std::uint8_t read(std::uint16_t address, bool read_only = false) const;
	void write(std::uint16_t address, std::uint8_t data) const;

	//debugger use
	[[nodiscard]] std::uint8_t get_register_accumulator() const { return register_accumulator; }
	[[nodiscard]] std::uint8_t get_register_x() const { return register_x; }
	[[nodiscard]] std::uint8_t get_register_y() const { return register_y; }
	[[nodiscard]] std::uint8_t get_register_stack_pointer() const { return register_stack_pointer; }
	[[nodiscard]] std::uint16_t get_register_program_counter() const { return register_program_counter; }
	[[nodiscard]] bitfield<processor_status_register> get_register_status() const { return register_status; }

	[[nodiscard]] std::uint8_t get_fetched() const { return fetched; }
	[[nodiscard]] std::uint16_t get_address_absolute() const { return address_absolute; }
	[[nodiscard]] std::uint16_t get_address_relative() const { return address_relative; }
	[[nodiscard]] std::uint8_t get_opcode() const { return opcode; }
	[[nodiscard]] std::uint8_t get_cycles_remaining() const { return cycles_remaining; }
	[[nodiscard]] std::uint32_t get_clock_count() const { return clock_count; }

	[[nodiscard]] const instruction& get_instruction() const { return instructions[opcode]; }
	[[nodiscard]] std::vector<std::string> disassemble() const;

private:
	[[nodiscard]] std::string instruction_to_string(uint16_t current, uint16_t& next) const;
	[[nodiscard]] bool find_previous_instruction(const uint16_t current, uint16_t& previous) const;
	[[nodiscard]] bool find_next_instruction(const uint16_t current, uint16_t& next) const;

private:
	//addressing
	//http://www.emulator101.com/6502-addressing-modes.html
	[[nodiscard]] bool address_mode_implicit();
	[[nodiscard]] bool address_mode_accumulator();
	[[nodiscard]] bool address_mode_immediate();

	[[nodiscard]] bool address_mode_zero_page();
	[[nodiscard]] bool address_mode_zero_page_x();
	[[nodiscard]] bool address_mode_zero_page_y();

	[[nodiscard]] bool address_mode_relative();
	[[nodiscard]] bool address_mode_absolute();
	[[nodiscard]] bool address_mode_absolute_x();
	[[nodiscard]] bool address_mode_absolute_y();

	[[nodiscard]] bool address_mode_indirect();
	[[nodiscard]] bool address_mode_indexed_indirect();
	[[nodiscard]] bool address_mode_indirect_indexed();
	
	//opcodes
	template <AddressingModeFunc Mode, processor_status_register flag, bool is_set>
	void branch();
	template <AddressingModeFunc Mode>
	void compare(std::uint8_t& target_register);
	template <AddressingModeFunc Mode>
	void load_register(std::uint8_t& target_register);
	//I can only apologise here...
	template<AddressingModeFunc Mode> void instruction_adc(); template<AddressingModeFunc Mode> void instruction_and(); template<AddressingModeFunc Mode> void instruction_asl(); template<AddressingModeFunc Mode> void instruction_bcc();
	template<AddressingModeFunc Mode> void instruction_bcs(); template<AddressingModeFunc Mode> void instruction_beq(); template<AddressingModeFunc Mode> void instruction_bit(); template<AddressingModeFunc Mode> void instruction_bmi();
	template<AddressingModeFunc Mode> void instruction_bne(); template<AddressingModeFunc Mode> void instruction_bpl(); template<AddressingModeFunc Mode> void instruction_brk(); template<AddressingModeFunc Mode> void instruction_bvc();
	template<AddressingModeFunc Mode> void instruction_bvs(); template<AddressingModeFunc Mode> void instruction_clc(); template<AddressingModeFunc Mode> void instruction_cld(); template<AddressingModeFunc Mode> void instruction_cli();
	template<AddressingModeFunc Mode> void instruction_clv(); template<AddressingModeFunc Mode> void instruction_cmp(); template<AddressingModeFunc Mode> void instruction_cpx(); template<AddressingModeFunc Mode> void instruction_cpy();
	template<AddressingModeFunc Mode> void instruction_dec(); template<AddressingModeFunc Mode> void instruction_dex(); template<AddressingModeFunc Mode> void instruction_dey(); template<AddressingModeFunc Mode> void instruction_eor();
	template<AddressingModeFunc Mode> void instruction_inc(); template<AddressingModeFunc Mode> void instruction_inx(); template<AddressingModeFunc Mode> void instruction_iny(); template<AddressingModeFunc Mode> void instruction_jmp();
	template<AddressingModeFunc Mode> void instruction_jsr(); template<AddressingModeFunc Mode> void instruction_lda(); template<AddressingModeFunc Mode> void instruction_ldx(); template<AddressingModeFunc Mode> void instruction_ldy();
	template<AddressingModeFunc Mode> void instruction_lsr(); template<AddressingModeFunc Mode> void instruction_nop(); template<AddressingModeFunc Mode> void instruction_ora(); template<AddressingModeFunc Mode> void instruction_pha();
	template<AddressingModeFunc Mode> void instruction_php(); template<AddressingModeFunc Mode> void instruction_pla(); template<AddressingModeFunc Mode> void instruction_plp(); template<AddressingModeFunc Mode> void instruction_rol();
	template<AddressingModeFunc Mode> void instruction_ror(); template<AddressingModeFunc Mode> void instruction_rti(); template<AddressingModeFunc Mode> void instruction_rts(); template<AddressingModeFunc Mode> void instruction_sbc();
	template<AddressingModeFunc Mode> void instruction_sec(); template<AddressingModeFunc Mode> void instruction_sed(); template<AddressingModeFunc Mode> void instruction_sei(); template<AddressingModeFunc Mode> void instruction_sta();
	template<AddressingModeFunc Mode> void instruction_stx(); template<AddressingModeFunc Mode> void instruction_sty(); template<AddressingModeFunc Mode> void instruction_tax(); template<AddressingModeFunc Mode> void instruction_tay();
	template<AddressingModeFunc Mode> void instruction_tsx(); template<AddressingModeFunc Mode> void instruction_txa(); template<AddressingModeFunc Mode> void instruction_txs(); template<AddressingModeFunc Mode> void instruction_tya();

	void instruction_xxx(){}
	
	//flags
	[[nodiscard]] bool get_flag(const processor_status_register flag) const
	{
		return (register_status & flag).value() == flag;
	}

	void set_flag(const processor_status_register flag, const std::uint8_t value)
	{
		if (value)
			register_status |= flag;
		else
			register_status &= ~static_cast<uint8_t>(flag);
	}

	//void clear_flag(const processor_status_register flag, const std::uint8_t value)
	//{
	//	if (value)
	//		register_status &= ~static_cast<uint8_t>(flag);
	//}

private:
	[[nodiscard]] constexpr std::array<instruction, 256> build_instructions() const;
	
	const bus& bus;
	const std::array<instruction, 256> instructions;

	//registers
	std::uint8_t register_accumulator = 0x00; 
	std::uint8_t register_x = 0x00;
	std::uint8_t register_y = 0x00;
	std::uint8_t register_stack_pointer = 0x00;
	std::uint16_t register_program_counter = 0x0000;
	bitfield<processor_status_register> register_status = processor_status_register::none;

	//internals
	std::uint8_t fetched = 0x00;                //working data for current instruction, if required
	std::uint16_t address_absolute = 0x0000;    //used for direct addressing
	std::uint16_t address_relative = 0x0000;	   //used for branching operations [-127, 128]
	std::uint8_t opcode = 0x00;                 //current instruction
	std::uint8_t cycles_remaining = 0;          //remaining cycles for current instruction
	std::uint32_t clock_count = 0;              //clock count accumulator

	static constexpr std::uint16_t stack_address_offset = 0x0100;
};

template <noentiendo::cpu::AddressingModeFunc Mode>
void noentiendo::cpu::fetch()
{
	if constexpr (Mode != &noentiendo::cpu::address_mode_implicit)
		fetched = read(address_absolute);
}

}	 // namespace noentiendo

#include "instructions.hpp"