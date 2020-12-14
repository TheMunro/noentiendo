#pragma once

//STDLIB
#include <cstdint>
#include <vector>
#include <type_traits>

//INTERNAL
#include "bus.hpp"

namespace nes_emu
{
enum class processor_status_register : std::uint8_t
{
	none            = 0,
	carry           = 1 << 0, //C
	zero            = 1 << 1, //Z
	interrupt_disable     = 1 << 2, //I
	decimal_mode    = 1 << 3, //D
	break_command   = 1 << 4, //B
	unused          = 1 << 5,
	overflow        = 1 << 6, //V
	negative        = 1 << 7  //N
};

template<typename enum_class_type>
class bitfield
{
	using underlying_type = std::underlying_type_t<enum_class_type>;
	
public:
	constexpr bitfield() noexcept
		: flags{}
	{
	}
	
	template <class U>
	constexpr bitfield(U initial_flags) noexcept
		: flags(static_cast<underlying_type>(initial_flags))
	{
	}

	//can I constexpr all the things?
	//https://en.cppreference.com/w/cpp/named_req/BitmaskType
    constexpr bitfield operator~() const noexcept
	{
		return bitfield {~flags};
	}

	constexpr bitfield operator&(const bitfield& r) const noexcept
	{
		return bitfield {flags & r.flags};
	}

	constexpr bitfield operator|(const bitfield& r) const noexcept
	{
		return bitfield {flags | r.flags};
	}

	constexpr bitfield operator^(const bitfield& r) const noexcept
	{
		return bitfield {flags ^ r.flags};
	}

	constexpr bitfield& operator|=(const bitfield& r) noexcept
	{
		flags |= r.flags;
		return *this;
	}

	constexpr bitfield& operator&=(const bitfield& r) noexcept
	{
		flags &= r.flags;
		return *this;
	}

	constexpr bitfield& operator^=(const bitfield& r) noexcept
	{
		flags ^= r.flags;
		return *this;
	}

	//too lazy to add equality operators for now
	[[nodiscard]] constexpr enum_class_type value() const
	{
		return static_cast<enum_class_type>(flags);
	}

private:
	underlying_type flags;
};

//consider overloading |=, &= and ^= instead
//void set_flag(const enum_class_type flag)
//{
//	flags |= static_cast<underlying_type>(flag);
//}
//
//void clear_flag(const enum_class_type flag)
//{
//	flags &= ~static_cast<underlying_type>(flag);
//}
//
//void toggle_flag(const enum_class_type flag)
//{
//	flags ^= static_cast<underlying_type>(flag);
//}


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
};


class cpu;
class instruction
{
public:
	opcode opcode;
	bool complete = false;
	uint8_t bytes = 0;
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
	void fetch();
	[[nodiscard]] std::uint8_t read(std::uint16_t address, bool read_only = false) const;
	void write(std::uint16_t address, std::uint8_t data) const;

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
	template<processor_status_register flag, bool is_set>
	[[nodiscard]] bool branch();
	[[nodiscard]] bool compare(std::uint8_t target_register);
	//I can only apologise here...
	[[nodiscard]] bool instruction_adc(); [[nodiscard]] bool instruction_and(); [[nodiscard]] bool instruction_asl(); [[nodiscard]] bool instruction_bcc();
	[[nodiscard]] bool instruction_bcs(); [[nodiscard]] bool instruction_beq(); [[nodiscard]] bool instruction_bit(); [[nodiscard]] bool instruction_bmi();
	[[nodiscard]] bool instruction_bne(); [[nodiscard]] bool instruction_bpl(); [[nodiscard]] bool instruction_brk(); [[nodiscard]] bool instruction_bvc();
	[[nodiscard]] bool instruction_bvs(); [[nodiscard]] bool instruction_clc(); [[nodiscard]] bool instruction_cld(); [[nodiscard]] bool instruction_cli();
	[[nodiscard]] bool instruction_clv(); [[nodiscard]] bool instruction_cmp(); [[nodiscard]] bool instruction_cpx(); [[nodiscard]] bool instruction_cpy();
	[[nodiscard]] bool instruction_dec(); [[nodiscard]] bool instruction_dex(); [[nodiscard]] bool instruction_dey(); [[nodiscard]] bool instruction_eor();
	[[nodiscard]] bool instruction_inc(); [[nodiscard]] bool instruction_inx(); [[nodiscard]] bool instruction_iny(); [[nodiscard]] bool instruction_jmp();
	[[nodiscard]] bool instruction_jsr(); [[nodiscard]] bool instruction_lda(); [[nodiscard]] bool instruction_ldx(); [[nodiscard]] bool instruction_ldy();
	[[nodiscard]] bool instruction_lsr(); [[nodiscard]] bool instruction_nop(); [[nodiscard]] bool instruction_ora(); [[nodiscard]] bool instruction_pha();
	[[nodiscard]] bool instruction_php(); [[nodiscard]] bool instruction_pla(); [[nodiscard]] bool instruction_plp(); [[nodiscard]] bool instruction_rol();
	[[nodiscard]] bool instruction_ror(); [[nodiscard]] bool instruction_rti(); [[nodiscard]] bool instruction_rts(); [[nodiscard]] bool instruction_sbc();
	[[nodiscard]] bool instruction_sec(); [[nodiscard]] bool instruction_sed(); [[nodiscard]] bool instruction_sei(); [[nodiscard]] bool instruction_sta();
	[[nodiscard]] bool instruction_stx(); [[nodiscard]] bool instruction_sty(); [[nodiscard]] bool instruction_tax(); [[nodiscard]] bool instruction_tay();
	[[nodiscard]] bool instruction_tsx(); [[nodiscard]] bool instruction_txa(); [[nodiscard]] bool instruction_txs(); [[nodiscard]] bool instruction_tya();

	//flags
	[[nodiscard]] bool get_flag(const processor_status_register flag) const
	{
		return (register_status & flag).value() == flag;
	}
	
	void set_flag(const processor_status_register flag, const std::uint8_t value)
	{
		if (value)
			register_status &= flag;
	}

	void clear_flag(const processor_status_register flag, const std::uint8_t value)
	{
		if (value)
			register_status |= flag;
	}

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
	bitfield<processor_status_register> register_status = processor_status_register::none;

	//internals
	std::uint8_t fetched = 0x00;                //working data for current instruction, if required
	std::uint16_t address_absolute = 0x0000;    //used for direct addressing
	std::uint16_t address_relative = 0x0000;	   //used for branching operations [-127, 128]
	std::uint8_t opcode = 0x00;                 //current instruction
	std::uint8_t cycles_remaining = 0;          //remaining cycles for current instruction
	std::uint32_t clock_count = 0;              //clock count accumulator

	std::uint16_t stack_address_offset = 0x0100;
};

template <processor_status_register flag, bool is_set>
bool nes_emu::cpu::branch()
{
	if (get_flag(flag) == is_set)
	{
		++cycles_remaining;
		address_absolute = register_program_counter + address_relative;

		if ((address_absolute & 0xFF00) != (register_program_counter & 0xFF00))
			++cycles_remaining;

		register_program_counter = address_absolute;
	}

	return false;
}

}	 // namespace nes_emu