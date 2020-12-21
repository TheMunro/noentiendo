//EXTERNAL
#include "fmt/format.h"

//INTERNAL
#include "cpu.hpp"
#include "util.hpp"

noentiendo::cpu::cpu(const noentiendo::bus& input_bus)
	: bus{input_bus}
	, instructions{build_instructions()}
{
}

std::uint8_t noentiendo::cpu::read(const std::uint16_t address, bool read_only) const
{
	return bus.read(address, read_only);
}

void noentiendo::cpu::write(const std::uint16_t address, const std::uint8_t data) const
{
	return bus.write(address, data);
}

void noentiendo::cpu::clock()
{
	opcode = read(register_program_counter);
  	if (cycles_remaining == 0)
	{
		++register_program_counter;

		const auto& instruction = instructions[opcode];
		cycles_remaining = instruction.cycles;

		std::invoke(instruction.execute, *this);
	}

	++clock_count;
	--cycles_remaining;
}

void noentiendo::cpu::reset()
{
	// Get address to set program counter to
	address_absolute = 0xFFFC;
	const uint16_t lo_byte = read(address_absolute + 0);
	const uint16_t hi_byte = read(address_absolute + 1);

	// Set it
	register_program_counter = (hi_byte << 8) + lo_byte;

	// Reset internal registers
	register_accumulator = 0;
	register_x = 0;
	register_y = 0;
	register_stack_pointer = 0xFD;
	register_status = 0x00;

	address_relative = 0x0000;
	address_absolute = 0x0000;
	fetched = 0x00;

	// Reset takes time
	cycles_remaining = 8;
}

void noentiendo::cpu::irq()
{	
}

void noentiendo::cpu::nmi()
{
}

constexpr std::array<noentiendo::instruction, 256> noentiendo::cpu::build_instructions() const
{
	std::array<instruction, 256> instructions{};

    for (auto i = 0; i < 256; ++i)
		instructions[i] = instruction{opcode::XXX, 1, 0, addressing_mode ::IMPLICIT, &cpu::instruction_xxx};

	//1 of 4
	instructions[0x69] = instruction{opcode::ADC, 2, 2, addressing_mode::IMMEDIATE, &cpu::instruction_adc<&cpu::address_mode_immediate>};
	instructions[0x65] = instruction{opcode::ADC, 2, 3, addressing_mode::ZERO_PAGE, &cpu::instruction_adc<&cpu::address_mode_zero_page>};
	instructions[0x75] = instruction{opcode::ADC, 2, 4, addressing_mode::ZERO_PAGE_X, &cpu::instruction_adc<&cpu::address_mode_zero_page_x>};
	instructions[0x6D] = instruction{opcode::ADC, 3, 4, addressing_mode::ABSOLUTE, &cpu::instruction_adc<&cpu::address_mode_absolute>};
	instructions[0x7D] = instruction{opcode::ADC, 3, 4, addressing_mode::ABSOLUTE_X,	&cpu::instruction_adc<&cpu::address_mode_absolute_x>};
	instructions[0x79] = instruction{opcode::ADC, 3, 4, addressing_mode::ABSOLUTE_Y,	&cpu::instruction_adc<&cpu::address_mode_absolute_y>};
	instructions[0x61] = instruction{opcode::ADC, 2, 6, addressing_mode::INDEXED_INDIRECT, &cpu::instruction_adc<&cpu::address_mode_indexed_indirect>};
	instructions[0x71] = instruction{opcode::ADC, 2, 5, addressing_mode::INDIRECT_INDEXED, &cpu::instruction_adc<&cpu::address_mode_indirect_indexed>};

	instructions[0x29] = instruction{opcode::AND, 2, 2, addressing_mode::IMMEDIATE, &cpu::instruction_and<&cpu::address_mode_immediate>};
	instructions[0x25] = instruction{opcode::AND, 2, 3, addressing_mode::ZERO_PAGE, &cpu::instruction_and<&cpu::address_mode_zero_page>};
	instructions[0x35] = instruction{opcode::AND, 2, 4, addressing_mode::ZERO_PAGE_X, &cpu::instruction_and<&cpu::address_mode_zero_page_x>};
	instructions[0x2D] = instruction{opcode::AND, 3, 4, addressing_mode::ABSOLUTE, &cpu::instruction_and<&cpu::address_mode_absolute>};
	instructions[0x3D] = instruction{opcode::AND, 3, 4, addressing_mode::ABSOLUTE_X, &cpu::instruction_and<&cpu::address_mode_absolute_x>};
	instructions[0x39] = instruction{opcode::AND, 3, 4, addressing_mode::ABSOLUTE_Y, &cpu::instruction_and<&cpu::address_mode_absolute_y>};
	instructions[0x21] = instruction{opcode::AND, 2, 6, addressing_mode::INDEXED_INDIRECT, &cpu::instruction_and<&cpu::address_mode_indexed_indirect>};
	instructions[0x31] = instruction{opcode::AND, 2, 5, addressing_mode::INDIRECT_INDEXED, &cpu::instruction_and<&cpu::address_mode_indirect_indexed>};
	
	instructions[0x0A] = instruction{opcode::ASL, 1, 2, addressing_mode::ACCUMULATOR, &cpu::instruction_asl<&cpu::address_mode_accumulator>};
	instructions[0x06] = instruction{opcode::ASL, 2, 5, addressing_mode::ZERO_PAGE, &cpu::instruction_asl<&cpu::address_mode_zero_page>};
	instructions[0x16] = instruction{opcode::ASL, 2, 6, addressing_mode::ZERO_PAGE_X, &cpu::instruction_asl<&cpu::address_mode_zero_page_x>};
	instructions[0x0E] = instruction{opcode::ASL, 3, 6, addressing_mode::ABSOLUTE, &cpu::instruction_asl<&cpu::address_mode_absolute>};
	instructions[0x1E] = instruction{opcode::ASL, 3, 7, addressing_mode::ABSOLUTE_X, &cpu::instruction_asl<&cpu::address_mode_absolute_x>};

	instructions[0x90] = instruction{opcode::BCC, 2, 2, addressing_mode::RELATIVE, &cpu::instruction_bcc<&cpu::address_mode_relative>};
	
	instructions[0xB0] = instruction{opcode::BCS, 2, 2, addressing_mode::RELATIVE, &cpu::instruction_bcs<&cpu::address_mode_relative>};

	instructions[0xF0] = instruction{opcode::BEQ, 2, 2, addressing_mode::RELATIVE, &cpu::instruction_beq<&cpu::address_mode_relative>};

	instructions[0x24] = instruction{opcode::BIT, 2, 2, addressing_mode::ZERO_PAGE, &cpu::instruction_bit<&cpu::address_mode_zero_page>};
	instructions[0x2C] = instruction{opcode::BIT, 3, 4, addressing_mode::ABSOLUTE, &cpu::instruction_bit<&cpu::address_mode_absolute>};

	instructions[0x30] = instruction{opcode::BMI, 2, 2, addressing_mode::RELATIVE, &cpu::instruction_bmi<&cpu::address_mode_relative>};

	instructions[0xD0] = instruction{opcode::BNE, 2, 2, addressing_mode::RELATIVE, &cpu::instruction_bne<&cpu::address_mode_relative>};

	instructions[0x10] = instruction{opcode::BPL, 2, 2, addressing_mode::RELATIVE, &cpu::instruction_bpl<&cpu::address_mode_relative>};

	instructions[0x00] = instruction{opcode::BRK, 1, 7, addressing_mode::IMPLICIT, &cpu::instruction_brk<&cpu::address_mode_implicit>};

	instructions[0x50] = instruction{opcode::BVC, 2, 2, addressing_mode::IMPLICIT, &cpu::instruction_bvc<&cpu::address_mode_relative>};

	instructions[0x70] = instruction{opcode::BVS, 2, 2, addressing_mode::IMPLICIT, &cpu::instruction_bvs<&cpu::address_mode_relative>};

	instructions[0x18] = instruction{opcode::CLC, 1, 2, addressing_mode::IMPLICIT, &cpu::instruction_clc<&cpu::address_mode_implicit>};

	//2 of 4
	instructions[0xD8] = instruction{opcode::CLD, 1, 2, addressing_mode::IMPLICIT, &cpu::instruction_cld<&cpu::address_mode_implicit>};

	instructions[0x58] = instruction{opcode::CLI, 1, 2, addressing_mode::IMPLICIT, &cpu::instruction_cli<&cpu::address_mode_implicit>};

	instructions[0xB8] = instruction{opcode::CLV, 1, 2, addressing_mode::IMPLICIT, &cpu::instruction_clv<&cpu::address_mode_implicit>};

	instructions[0xC9] = instruction{opcode::CMP, 2, 2, addressing_mode::IMMEDIATE, &cpu::instruction_cmp<&cpu::address_mode_immediate>};
	instructions[0xC5] = instruction{opcode::CMP, 2, 3, addressing_mode::ZERO_PAGE, &cpu::instruction_cmp<&cpu::address_mode_zero_page>};
	instructions[0xD5] = instruction{opcode::CMP, 2, 4, addressing_mode::ZERO_PAGE_X, &cpu::instruction_cmp<&cpu::address_mode_zero_page_x>};
	instructions[0xCD] = instruction{opcode::CMP, 3, 4, addressing_mode::ABSOLUTE, &cpu::instruction_cmp<&cpu::address_mode_absolute>};
	instructions[0xDD] = instruction{opcode::CMP, 3, 4, addressing_mode::ABSOLUTE_X, &cpu::instruction_cmp<&cpu::address_mode_absolute_x>};
	instructions[0xD9] = instruction{opcode::CMP, 3, 4, addressing_mode::ABSOLUTE_Y, &cpu::instruction_cmp<&cpu::address_mode_absolute_y>};
	instructions[0xC1] = instruction{opcode::CMP, 2, 6, addressing_mode::INDEXED_INDIRECT, &cpu::instruction_cmp<&cpu::address_mode_indexed_indirect>};
	instructions[0xD1] = instruction{opcode::CMP, 2, 5, addressing_mode::INDIRECT_INDEXED, &cpu::instruction_cmp<&cpu::address_mode_indirect_indexed>};

	instructions[0xE0] = instruction{opcode::CPX, 2, 2, addressing_mode::IMMEDIATE, &cpu::instruction_cpx<&cpu::address_mode_immediate>};
	instructions[0xE4] = instruction{opcode::CPX, 2, 3, addressing_mode::ZERO_PAGE, &cpu::instruction_cpx<&cpu::address_mode_zero_page>};
	instructions[0xEC] = instruction{opcode::CPX, 3, 4, addressing_mode::ABSOLUTE, &cpu::instruction_cpx<&cpu::address_mode_absolute>};

	instructions[0xC0] = instruction{opcode::CPY, 2, 2, addressing_mode::IMMEDIATE, & cpu::instruction_cpy<&cpu::address_mode_immediate>};
	instructions[0xC4] = instruction{opcode::CPY, 2, 3, addressing_mode::ZERO_PAGE, &cpu::instruction_cpy<&cpu::address_mode_zero_page>};
	instructions[0xCC] = instruction{opcode::CPY, 3, 4, addressing_mode::ABSOLUTE, &cpu::instruction_cpy<&cpu::address_mode_absolute>};

	instructions[0xC6] = instruction{opcode::DEC, 2, 5, addressing_mode::ZERO_PAGE, &cpu::instruction_dec<&cpu::address_mode_zero_page>};
	instructions[0xD6] = instruction{opcode::DEC, 2, 6, addressing_mode::ZERO_PAGE_X, &cpu::instruction_dec<&cpu::address_mode_zero_page_x>};
	instructions[0xCE] = instruction{opcode::DEC, 3, 6, addressing_mode::ABSOLUTE, &cpu::instruction_dec<&cpu::address_mode_absolute>};
	instructions[0xDE] = instruction{opcode::DEC, 3, 7, addressing_mode::ABSOLUTE_X, &cpu::instruction_dec<&cpu::address_mode_absolute_x>};

	instructions[0xCA] = instruction{opcode::DEX, 1, 2, addressing_mode::IMPLICIT, &cpu::instruction_dex<&cpu::address_mode_implicit>};

	instructions[0x88] = instruction{opcode::DEY, 1, 2, addressing_mode::IMPLICIT, &cpu::instruction_dey<&cpu::address_mode_implicit>};

	instructions[0x49] = instruction{opcode::EOR, 2, 2, addressing_mode::IMMEDIATE, &cpu::instruction_eor<&cpu::address_mode_immediate>};
	instructions[0x45] = instruction{opcode::EOR, 2, 3, addressing_mode::ZERO_PAGE, &cpu::instruction_eor<&cpu::address_mode_zero_page>};
	instructions[0x55] = instruction{opcode::EOR, 2, 4, addressing_mode::ZERO_PAGE_X, &cpu::instruction_eor<&cpu::address_mode_zero_page_x>};
	instructions[0x4D] = instruction{opcode::EOR, 3, 4, addressing_mode::ABSOLUTE, &cpu::instruction_eor<&cpu::address_mode_absolute>};
	instructions[0x5D] = instruction{opcode::EOR, 3, 4, addressing_mode::ABSOLUTE_X, &cpu::instruction_eor<&cpu::address_mode_absolute_x>};
	instructions[0x59] = instruction{opcode::EOR, 3, 4, addressing_mode::ABSOLUTE_Y, &cpu::instruction_eor<&cpu::address_mode_absolute_y>};
	instructions[0x41] = instruction{opcode::EOR, 2, 6, addressing_mode::INDEXED_INDIRECT, &cpu::instruction_eor<&cpu::address_mode_indexed_indirect>};
	instructions[0x51] = instruction{opcode::EOR, 2, 5, addressing_mode::INDIRECT_INDEXED, &cpu::instruction_eor<&cpu::address_mode_indirect_indexed>};
	
	instructions[0xE6] = instruction{opcode::INC, 2, 5, addressing_mode::ZERO_PAGE, &cpu::instruction_inc<&cpu::address_mode_zero_page>};
	instructions[0xF6] = instruction{opcode::INC, 2, 6, addressing_mode::ZERO_PAGE_X, &cpu::instruction_inc<&cpu::address_mode_zero_page_x>};
	instructions[0xEE] = instruction{opcode::INC, 3, 6, addressing_mode::ABSOLUTE, &cpu::instruction_inc<&cpu::address_mode_absolute>};
	instructions[0xFE] = instruction{opcode::INC, 3, 7, addressing_mode::ABSOLUTE_X, &cpu::instruction_inc<&cpu::address_mode_absolute_x>};

	instructions[0xE8] = instruction{opcode::INX, 1, 2, addressing_mode::IMPLICIT, &cpu::instruction_inx<&cpu::address_mode_implicit>};

	instructions[0xC8] = instruction{opcode::INY, 1, 2, addressing_mode::IMPLICIT, &cpu::instruction_iny<&cpu::address_mode_implicit>};

	instructions[0x4C] = instruction{opcode::JMP, 3, 3, addressing_mode::ABSOLUTE, &cpu::instruction_jmp<&cpu::address_mode_absolute>};
	instructions[0x6C] = instruction{opcode::JMP, 3, 5, addressing_mode::INDIRECT, &cpu::instruction_jmp<&cpu::address_mode_indirect>};
		
	//3 of 4
	instructions[0x20] = instruction{opcode::JSR, 3, 6, addressing_mode::ABSOLUTE, &cpu::instruction_jsr<&cpu::address_mode_absolute>};

	instructions[0xA9] = instruction{opcode::LDA, 2, 2, addressing_mode::IMMEDIATE, &cpu::instruction_lda<&cpu::address_mode_immediate>};
	instructions[0xA5] = instruction{opcode::LDA, 2, 3, addressing_mode::ZERO_PAGE, &cpu::instruction_lda<&cpu::address_mode_zero_page>};
	instructions[0xB5] = instruction{opcode::LDA, 2, 4, addressing_mode::ZERO_PAGE_X, &cpu::instruction_lda<&cpu::address_mode_zero_page_x>};
	instructions[0xAD] = instruction{opcode::LDA, 3, 4, addressing_mode::ABSOLUTE, &cpu::instruction_lda<&cpu::address_mode_absolute>};
	instructions[0xBD] = instruction{opcode::LDA, 3, 4, addressing_mode::ABSOLUTE_X, &cpu::instruction_lda<&cpu::address_mode_absolute_x>};
	instructions[0xB9] = instruction{opcode::LDA, 3, 4, addressing_mode::ABSOLUTE_Y, &cpu::instruction_lda<&cpu::address_mode_absolute_y>};
	instructions[0xA1] = instruction{opcode::LDA, 2, 6, addressing_mode::INDEXED_INDIRECT, &cpu::instruction_lda<&cpu::address_mode_indexed_indirect>};
	instructions[0xB1] = instruction{opcode::LDA, 2, 5, addressing_mode::INDIRECT_INDEXED, &cpu::instruction_lda<&cpu::address_mode_indirect_indexed>};

	instructions[0xA2] = instruction{opcode::LDX, 2, 2, addressing_mode::IMMEDIATE, &cpu::instruction_ldx<&cpu::address_mode_immediate>};
	instructions[0xA6] = instruction{opcode::LDX, 2, 3, addressing_mode::ZERO_PAGE, &cpu::instruction_ldx<&cpu::address_mode_zero_page>};
	instructions[0xB6] = instruction{opcode::LDX, 2, 4, addressing_mode::ZERO_PAGE_Y, &cpu::instruction_ldx<&cpu::address_mode_zero_page_y>};
	instructions[0xAE] = instruction{opcode::LDX, 3, 4, addressing_mode::ABSOLUTE, &cpu::instruction_ldx<&cpu::address_mode_absolute>};
	instructions[0xBE] = instruction{opcode::LDX, 3, 4, addressing_mode::ABSOLUTE_Y, &cpu::instruction_ldx<&cpu::address_mode_absolute_y>};

	instructions[0xA0] = instruction{opcode::LDY, 2, 2, addressing_mode::IMMEDIATE, &cpu::instruction_ldy<&cpu::address_mode_immediate>};
	instructions[0xA4] = instruction{opcode::LDY, 2, 3, addressing_mode::ZERO_PAGE, &cpu::instruction_ldy<&cpu::address_mode_zero_page>};
	instructions[0xB4] = instruction{opcode::LDY, 2, 4, addressing_mode::ZERO_PAGE_X, &cpu::instruction_ldy<&cpu::address_mode_zero_page_x>};
	instructions[0xAC] = instruction{opcode::LDY, 3, 4, addressing_mode::ABSOLUTE, &cpu::instruction_ldy<&cpu::address_mode_absolute>};
	instructions[0xBC] = instruction{opcode::LDY, 3, 4, addressing_mode::ABSOLUTE_X, &cpu::instruction_ldy<&cpu::address_mode_absolute_x>};

	instructions[0x4A] = instruction{opcode::LSR, 1, 2, addressing_mode::IMMEDIATE, &cpu::instruction_lsr<&cpu::address_mode_immediate>};
	instructions[0x46] = instruction{opcode::LSR, 2, 5, addressing_mode::ZERO_PAGE, &cpu::instruction_lsr<&cpu::address_mode_zero_page>};
	instructions[0x56] = instruction{opcode::LSR, 2, 6, addressing_mode::ZERO_PAGE_X, &cpu::instruction_lsr<&cpu::address_mode_zero_page_x>};
	instructions[0x4E] = instruction{opcode::LSR, 3, 6, addressing_mode::ABSOLUTE, &cpu::instruction_lsr<&cpu::address_mode_absolute>};
	instructions[0x5E] = instruction{opcode::LSR, 3, 7, addressing_mode::ABSOLUTE_X, &cpu::instruction_lsr<&cpu::address_mode_absolute_x>};
	
	instructions[0xEA] = instruction{opcode::NOP, 1, 2, addressing_mode::IMPLICIT, &cpu::instruction_nop<&cpu::address_mode_implicit>};

	instructions[0x09] = instruction{opcode::ORA, 2, 2, addressing_mode::IMMEDIATE, &cpu::instruction_ora<&cpu::address_mode_immediate>};
	instructions[0x05] = instruction{opcode::ORA, 2, 3, addressing_mode::ZERO_PAGE, &cpu::instruction_ora<&cpu::address_mode_zero_page>};
	instructions[0x15] = instruction{opcode::ORA, 2, 4, addressing_mode::ZERO_PAGE_X, &cpu::instruction_ora<&cpu::address_mode_zero_page_x>};
	instructions[0x0D] = instruction{opcode::ORA, 3, 4, addressing_mode::ABSOLUTE, &cpu::instruction_ora<&cpu::address_mode_absolute>};
	instructions[0x1D] = instruction{opcode::ORA, 3, 4, addressing_mode::ABSOLUTE_X, &cpu::instruction_ora<&cpu::address_mode_absolute_x>};
	instructions[0x19] = instruction{opcode::ORA, 3, 4, addressing_mode::ABSOLUTE_Y, &cpu::instruction_ora<&cpu::address_mode_absolute_y>};
	instructions[0x01] = instruction{opcode::ORA, 2, 6, addressing_mode::INDEXED_INDIRECT, &cpu::instruction_ora<&cpu::address_mode_indexed_indirect>};
	instructions[0x11] = instruction{opcode::ORA, 2, 5, addressing_mode::INDIRECT_INDEXED, &cpu::instruction_ora<&cpu::address_mode_indirect_indexed>};

	instructions[0x48] = instruction{opcode::PHA, 1, 3, addressing_mode::IMPLICIT, &cpu::instruction_pha<&cpu::address_mode_implicit>};

	instructions[0x08] = instruction{opcode::PHP, 1, 3, addressing_mode::IMPLICIT, &cpu::instruction_php<&cpu::address_mode_implicit>};

	instructions[0x68] = instruction{opcode::PLA, 1, 4, addressing_mode::IMPLICIT, &cpu::instruction_pla<&cpu::address_mode_implicit>};

	instructions[0x28] = instruction{opcode::PLP, 1, 4, addressing_mode::IMPLICIT, &cpu::instruction_plp<&cpu::address_mode_implicit>};

	instructions[0x2A] = instruction{opcode::ROL, 1, 2, addressing_mode::ACCUMULATOR, &cpu::instruction_rol<&cpu::address_mode_accumulator>};
	instructions[0x26] = instruction{opcode::ROL, 2, 5, addressing_mode::ZERO_PAGE, &cpu::instruction_rol<&cpu::address_mode_zero_page>};
	instructions[0x36] = instruction{opcode::ROL, 2, 6, addressing_mode::ZERO_PAGE_X, &cpu::instruction_rol<&cpu::address_mode_zero_page_x>};
	instructions[0x2E] = instruction{opcode::ROL, 3, 6, addressing_mode::ABSOLUTE, &cpu::instruction_rol<&cpu::address_mode_absolute>};
	instructions[0x3E] = instruction{opcode::ROL, 3, 7, addressing_mode::ABSOLUTE_X, &cpu::instruction_rol<&cpu::address_mode_absolute_x>};

	instructions[0x6A] = instruction{opcode::ROR, 1, 2, addressing_mode::ACCUMULATOR, &cpu::instruction_ror<&cpu::address_mode_accumulator>};
	instructions[0x66] = instruction{opcode::ROR, 2, 5, addressing_mode::ZERO_PAGE, &cpu::instruction_ror<&cpu::address_mode_zero_page>};
	instructions[0x76] = instruction{opcode::ROR, 2, 6, addressing_mode::ZERO_PAGE_X, &cpu::instruction_ror<&cpu::address_mode_zero_page_x>};
	instructions[0x6E] = instruction{opcode::ROR, 3, 6, addressing_mode::ABSOLUTE, &cpu::instruction_ror<&cpu::address_mode_absolute>};
	instructions[0x7E] = instruction{opcode::ROR, 3, 7, addressing_mode::ABSOLUTE_X, &cpu::instruction_ror<&cpu::address_mode_absolute_x>};

	instructions[0x40] = instruction{opcode::RTI, 1, 6, addressing_mode::IMPLICIT, &cpu::instruction_rti<&cpu::address_mode_implicit>};
	
	//4 of 4
	instructions[0x60] = instruction{opcode::RTS, 1, 6, addressing_mode::IMPLICIT, &cpu::instruction_rts<&cpu::address_mode_implicit>};

	instructions[0xE9] = instruction{opcode::SBC, 2, 2, addressing_mode::IMMEDIATE, &cpu::instruction_sbc<&cpu::address_mode_immediate>};
	instructions[0xE5] = instruction{opcode::SBC, 2, 3, addressing_mode::ZERO_PAGE, &cpu::instruction_sbc<&cpu::address_mode_zero_page>};
	instructions[0xF5] = instruction{opcode::SBC, 2, 4, addressing_mode::ZERO_PAGE_X, &cpu::instruction_sbc<&cpu::address_mode_zero_page_x>};
	instructions[0xED] = instruction{opcode::SBC, 3, 4, addressing_mode::ABSOLUTE, &cpu::instruction_sbc<&cpu::address_mode_absolute>};
	instructions[0xFD] = instruction{opcode::SBC, 3, 4, addressing_mode::ABSOLUTE_X, &cpu::instruction_sbc<&cpu::address_mode_absolute_x>};
	instructions[0xF9] = instruction{opcode::SBC, 3, 4, addressing_mode::ABSOLUTE_Y, &cpu::instruction_sbc<&cpu::address_mode_absolute_y>};
	instructions[0xE1] = instruction{opcode::SBC, 2, 6, addressing_mode::INDEXED_INDIRECT, &cpu::instruction_sbc<&cpu::address_mode_indexed_indirect>};
	instructions[0xF1] = instruction{opcode::SBC, 2, 5, addressing_mode::INDIRECT_INDEXED, &cpu::instruction_sbc<&cpu::address_mode_indirect_indexed>};

	instructions[0x38] = instruction{opcode::SEC, 1, 2, addressing_mode::IMPLICIT, &cpu::instruction_sec<&cpu::address_mode_implicit>};

	instructions[0xF8] = instruction{opcode::SED, 1, 2, addressing_mode::IMPLICIT, &cpu::instruction_sed<&cpu::address_mode_implicit>};

	instructions[0x78] = instruction{opcode::SEI, 1, 2, addressing_mode::IMPLICIT, &cpu::instruction_sed<&cpu::address_mode_implicit>};
	
	instructions[0x85] = instruction{opcode::STA, 2, 3, addressing_mode::ZERO_PAGE, &cpu::instruction_sta<&cpu::address_mode_zero_page>};
	instructions[0x95] = instruction{opcode::STA, 2, 4, addressing_mode::ZERO_PAGE_X, &cpu::instruction_sta<&cpu::address_mode_zero_page_x>};
	instructions[0x8D] = instruction{opcode::STA, 3, 4, addressing_mode::ABSOLUTE, &cpu::instruction_sta<&cpu::address_mode_absolute>};
	instructions[0x9D] = instruction{opcode::STA, 3, 4, addressing_mode::ABSOLUTE_X, &cpu::instruction_sta<&cpu::address_mode_absolute_x>};
	instructions[0x99] = instruction{opcode::STA, 3, 4, addressing_mode::ABSOLUTE_Y, &cpu::instruction_sta<&cpu::address_mode_absolute_y>};
	instructions[0x81] = instruction{opcode::STA, 2, 6, addressing_mode::INDEXED_INDIRECT, &cpu::instruction_sta<&cpu::address_mode_indexed_indirect>};
	instructions[0x91] = instruction{opcode::STA, 2, 6, addressing_mode::INDIRECT_INDEXED, &cpu::instruction_sta<&cpu::address_mode_indirect_indexed>};

	instructions[0x86] = instruction{opcode::STX, 2, 3, addressing_mode::ZERO_PAGE, &cpu::instruction_stx<&cpu::address_mode_zero_page>};
	instructions[0x96] = instruction{opcode::STX, 2, 4, addressing_mode::ZERO_PAGE_Y, &cpu::instruction_stx<&cpu::address_mode_zero_page_y>};
	instructions[0x8E] = instruction{opcode::STX, 3, 4, addressing_mode::ABSOLUTE, &cpu::instruction_stx<&cpu::address_mode_absolute>};

	instructions[0x84] = instruction{opcode::STY, 2, 3, addressing_mode::ZERO_PAGE, &cpu::instruction_sty<&cpu::address_mode_zero_page>};
	instructions[0x94] = instruction{opcode::STY, 2, 4, addressing_mode::ZERO_PAGE_X, &cpu::instruction_sty<&cpu::address_mode_zero_page_x>};
	instructions[0x8C] = instruction{opcode::STY, 3, 4, addressing_mode::ABSOLUTE, &cpu::instruction_sty<&cpu::address_mode_absolute>};

	instructions[0xAA] = instruction{opcode::TAX, 1, 2, addressing_mode::IMPLICIT, &cpu::instruction_tax<&cpu::address_mode_implicit>};

	instructions[0xA8] = instruction{opcode::TAY, 1, 2, addressing_mode::IMPLICIT, &cpu::instruction_tay<&cpu::address_mode_implicit>};

	instructions[0xBA] = instruction{opcode::TSX, 1, 2, addressing_mode::IMPLICIT, &cpu::instruction_tsx<&cpu::address_mode_implicit>};

	instructions[0x8A] = instruction{opcode::TXA, 1, 2, addressing_mode::IMPLICIT, &cpu::instruction_txa<&cpu::address_mode_implicit>};

	instructions[0x9A] = instruction{opcode::TXS, 1, 2, addressing_mode::IMPLICIT, &cpu::instruction_txs<&cpu::address_mode_implicit>};

	instructions[0x98] = instruction{opcode::TYA, 1, 2, addressing_mode::IMPLICIT, & cpu::instruction_tya<&cpu::address_mode_implicit>};

	return instructions;
}

bool noentiendo::cpu::address_mode_implicit()
{
	//In an implied instruction, the data and/or destination is mandatory for the instruction. For example,
	//the CLC instruction is implied, it is going to clear the processor's Carry flag.
	
	//working data determined by instruction
	
	//program counter increment controlled by instruction
	return false;
}

bool noentiendo::cpu::address_mode_accumulator()
{
	//These instructions have register A (the accumulator) as the target.
	
	//accumulator addressing modes skip fetch step as register value is already populated
	//set current working data to accumulator
	fetched = register_accumulator;
	
	//program counter increment controlled by instruction
	return false;
}

bool noentiendo::cpu::address_mode_immediate()
{
	//These instructions have their data defined as the next byte after the opcode. ORA #$B2 will perform a
	//logical (also called bitwise) of the value B2 with the accumulator.
	//
	//cache current program data as address
	address_absolute = register_program_counter;
	
	//program counter increment to prepare next instruction
	++register_program_counter;
	return false;
}

bool noentiendo::cpu::address_mode_zero_page()
{
	//Zero-Page is an addressing mode that is only capable of addressing the first 256 bytes of the CPU's memory map.
	//You can think of it as absolute addressing for the first 256 bytes. The instruction LDA $35 will put the value
	//stored in memory location $35 into A. The advantage of zero-page are two - the instruction takes one less byte
	//to specify, and it executes in less CPU cycles. Most programs are written to store the most frequently used
	//variables in the first 256 memory locations so they can take advantage of zero page addressing.
	
	const auto low_byte = read(register_program_counter);
	address_absolute = low_byte;
	
	//program counter increment to prepare next instruction
	++register_program_counter;
	return false;
}

bool noentiendo::cpu::address_mode_zero_page_x()
{
	//This works just like absolute indexed, but the target address is limited to the first 0xFF bytes.
	//The target address will wrap around and will always be in the zero page.If the instruction is LDA $C0, X,
	//and X is $60, then the target address will be $20.$C0 + $60 = $120, but the carry is discarded in the
	//calculation of the target address.
	
	const auto low_byte = read(register_program_counter);
	address_absolute = (low_byte + register_x) & 0x00FF;

	//program counter increment to prepare next instruction
	++register_program_counter;
	return false;
}

bool noentiendo::cpu::address_mode_zero_page_y()
{
	//This works just like absolute indexed, but the target address is limited to the first 0xFF bytes.
	//The target address will wrap around and will always be in the zero page.If the instruction is LDA $C0, X,
	//and X is $60, then the target address will be $20.$C0 + $60 = $120, but the carry is discarded in the
	//calculation of the target address.
	
	const auto low_byte = read(register_program_counter);
	address_absolute = (low_byte + register_y) & 0x00FF;

	//program counter increment to prepare next instruction
	++register_program_counter;
	return false;
}

bool noentiendo::cpu::address_mode_relative()
{
	//Relative addressing on the 6502 is only used for branch operations. The byte after the opcode
	//is the branch offset. If the branch is taken, the new address will the the current PC plus the offset.
	//The offset is a signed byte, so it can jump a maximum of 127 bytes forward, or 128 bytes backward.

	address_relative = read(register_program_counter);
	++register_program_counter;
	
	//TODO: there is a weird bit in https://github.com/OneLoneCoder/olcNES/blob/master/Part%232%20-%20CPU/olc6502.cpp
	//TODO: for this that it setting some upper byte that is then only being used to check for overflow,
	//TODO: can this be handled more cleanly elsewhere?
	return false;
}

bool noentiendo::cpu::address_mode_absolute()
{
	//Absolute addressing specifies the memory location explicitly in the two bytes following the
	//opcode. So JMP $4032 will set the PC to $4032. The hex for this is 4C 32 40.
	//The 6502 is a little endian machine, so any 16 bit (2 byte) value is stored with the LSB first.
	//All instructions that use absolute addressing are 3 bytes.
	
	const std::uint16_t lo_byte = read(register_program_counter);
	++register_program_counter;

	const std::uint16_t hi_byte = read(register_program_counter);
	++register_program_counter;

	address_absolute = static_cast<std::uint16_t>(hi_byte << 8) + lo_byte;
	
	return false;
}

bool noentiendo::cpu::address_mode_absolute_x()
{
	//This addressing mode makes the target address by adding the contents of the X or Y
	//register to an absolute address. For example, this 6502 code can be used to fill 10 bytes
	//with $FF starting at address $1009, counting down to address $1000.
	
	const auto ret = address_mode_absolute();

	const auto hi_byte = address_absolute & 0xFF00;
	address_absolute += register_x;
	if ((hi_byte & (address_absolute & 0xFF00)) == 0xFF00)
		return true; //new page, use result to increase cycle count

	return ret & false;
}

bool noentiendo::cpu::address_mode_absolute_y()
{
	//This addressing mode makes the target address by adding the contents of the X or Y
	//register to an absolute address. For example, this 6502 code can be used to fill 10 bytes
	//with $FF starting at address $1009, counting down to address $1000.
	
	const auto ret = address_mode_absolute();

	const auto hi_byte = address_absolute & 0xFF00;
	address_absolute += register_y;
	if ((hi_byte & (address_absolute & 0xFF00)) == 0xFF00)
		return true; //new page, use result to increase cycle count

	return ret & false;
}

bool noentiendo::cpu::address_mode_indirect()
{
	//The JMP instruction is the only instruction that uses this addressing mode.
	//It is a 3 byte instruction - the 2nd and 3rd bytes are an absolute address.
	//The set the PC to the address stored at that address. So maybe this would be clearer.

	const std::uint16_t lo_byte = read(register_program_counter);
	++register_program_counter;

	const std::uint16_t hi_byte = read(register_program_counter);
	++register_program_counter;

	address_absolute = static_cast<std::uint16_t>(hi_byte << 8) + lo_byte;
	
	return false;
}

bool noentiendo::cpu::address_mode_indexed_indirect()
{
	//This mode is only used with the X register. Consider a situation where the instruction is LDA ($20,X),
	//X contains $04, and memory at $24 contains 0024: 74 20, First, X is added to $20 to get $24.
	//The target address will be fetched from $24 resulting in a target address of $2074.
	//Register A will be loaded with the contents of memory at $2074.
	//
	//If X + the immediate byte will wrap around to a zero - page address.So you could code that
	//like targetAddress = (X + opcode[1]) & 0xFF.
	//
	//Indexed Indirect instructions are 2 bytes - the second byte is the zero - page address - $20 in
	//the example.Obviously the fetched address has to be stored in the zero page.

	auto ptr = read(register_program_counter);
	++register_program_counter;

	ptr += register_x;

	const std::uint16_t hi_byte = read(ptr) << 8;
	const std::uint16_t lo_byte = read(ptr + 1);

	address_absolute = static_cast<std::uint16_t>(hi_byte << 8) + lo_byte;
	
	return false;
}

bool noentiendo::cpu::address_mode_indirect_indexed()
{
	//This mode is only used with the Y register. It differs in the order that Y is applied to the indirectly
	//fetched address. An example instruction that uses indirect index addressing is LDA ($86),Y .
	//To calculate the target address, the CPU will first fetch the address stored at zero page location $86.
	//That address will be added to register Y to get the final target address. For LDA ($86),Y,
	//if the address stored at $86 is $4028 (memory is 0086: 28 40, remember little endian) and register Y
	//contains $10, then the final target address would be $4038. Register A will be loaded with the
	//contents of memory at $4038.
	//
	//Indirect Indexed instructions are 2 bytes - the second byte is the zero - page address - $86 in the
	//example.(So the fetched address has to be stored in the zero page.)
	//
	//While indexed indirect addressing will only generate a zero - page address, this mode's target address
	//is not wrapped - it can be anywhere in the 16-bit address space.

	const auto ptr = read(register_program_counter);
	++register_program_counter;

	const std::uint16_t hi_byte = read(ptr) << 8;
	const std::uint16_t lo_byte = read(ptr + 1);

	address_absolute = static_cast<std::uint16_t>(hi_byte << 8) + lo_byte;
	
	address_absolute += register_y;

	//check for wraparound
	if ((address_absolute & 0xFF00) != hi_byte)
		return true;
	
	return false;
}
std::vector<std::string> noentiendo::cpu::disassemble() const
{
	std::vector<std::string> result;
	result.reserve(21);
	
	const auto start_address = 0x0000;
	const auto end_address = 0xFFFF;
	const auto initial = register_program_counter;

	auto current = initial;
	auto prev_count = 10;
	while (prev_count > 0)
	{
		auto previous = current;
		if (!find_previous_instruction(current, previous))
			break;
		
		current = previous;
		--prev_count;
	}
	
	while (prev_count-- > 0)
		result.emplace_back("$????:XXX");
	
	while (current < initial)
		result.emplace_back(instruction_to_string(current, current));
	
	result.emplace_back(instruction_to_string(initial, current));
	const auto after_initial = current;

	auto next_count = 10;
	while (next_count > 0)
	{
		auto next = current;
		if (!find_next_instruction(current, next))
			break;

		current = next;
		--next_count;
	}
	
	const auto next = current;
	current = after_initial;
	while (current < next)
		result.emplace_back(instruction_to_string(current, current));
	
	while (next_count-- > 0)
		result.emplace_back("$????:XXX");

	return result;
}

//Adapted from https://github.com/OneLoneCoder/olcNES/blob/1db7fcad56591d22de36300b53da603a7c996125/Part%232%20-%20CPU/olc6502.cpp#L1472
std::string noentiendo::cpu::instruction_to_string(uint16_t current, uint16_t& next) const 
{
	next = current;
	uint8_t value = 0x00;
	uint8_t lo = 0x00;
	uint8_t hi = 0x00;

	auto hex = [](uint32_t n, uint8_t d) {
		return fmt::format("{:0{}X}", n, d);
	};

	auto opcode = bus.read(current, true);
	auto instruction = fmt::format("${}:{} ", hex(current, 4), opcode_to_string(instructions[opcode].opcode));

	const auto current_opcode = bus.read(next, true);
	next++;

	if (instructions[current_opcode].addressing_mode == addressing_mode::IMPLICIT)
	{
		instruction += "{IMP}";
	}
	else if(instructions[current_opcode].addressing_mode == addressing_mode::ACCUMULATOR)
	{
		instruction += "{ACC}";
	}
	else if (instructions[current_opcode].addressing_mode == addressing_mode::IMMEDIATE)
	{
		value = bus.read(next, true);
		next++;
		instruction += "{IMM} #$" + hex(value, 2) ;
	}
	else if (instructions[current_opcode].addressing_mode == addressing_mode::ZERO_PAGE)
	{
		lo = bus.read(next, true);
		next++;
		hi = 0x00;
		instruction += "{ZP0} $" + hex(lo, 2);
	}
	else if (instructions[current_opcode].addressing_mode == addressing_mode::ZERO_PAGE_X)
	{
		lo = bus.read(next, true);
		next++;
		hi = 0x00;
		instruction += "{ZPX} $" + hex(lo, 2) + ", X";
	}
	else if (instructions[current_opcode].addressing_mode == addressing_mode::ZERO_PAGE_Y)
	{
		lo = bus.read(next, true);
		next++;
		hi = 0x00;
		instruction += "{ZPY} $" + hex(lo, 2) + ", Y";
	}
	else if (instructions[current_opcode].addressing_mode == addressing_mode::INDEXED_INDIRECT)
	{
		lo = bus.read(next, true);
		next++;
		hi = 0x00;
		instruction += "{IZX} ($" + hex(lo, 2) + ", X)";
	}
	else if (instructions[current_opcode].addressing_mode == addressing_mode::INDIRECT_INDEXED)
	{
		lo = bus.read(next, true);
		next++;
		hi = 0x00;
		instruction += "{IZY} ($" + hex(lo, 2) + "), Y";
	}
	else if (instructions[current_opcode].addressing_mode == addressing_mode::ABSOLUTE)
	{
		lo = bus.read(next, true);
		next++;
		hi = bus.read(next, true);
		next++;
		instruction += "{ABS} $" + hex(static_cast<uint16_t>(hi << 8) + lo, 4);
	}
	else if (instructions[current_opcode].addressing_mode == addressing_mode::ABSOLUTE_X)
	{
		lo = bus.read(next, true);
		next++;
		hi = bus.read(next, true);
		next++;
		instruction += "{ABX} $" + hex(static_cast<uint16_t>(hi << 8) + lo, 4) + ", X";
	}
	else if (instructions[current_opcode].addressing_mode == addressing_mode::ABSOLUTE_Y)
	{
		lo = bus.read(next, true);
		next++;
		hi = bus.read(next, true);
		next++;
		instruction += "{ABY} $" + hex(static_cast<uint16_t>(hi << 8) + lo, 4) + ", Y";
	}
	else if (instructions[current_opcode].addressing_mode == addressing_mode::INDIRECT)
	{
		lo = bus.read(next, true);
		next++;
		hi = bus.read(next, true);
		next++;
		instruction += "{IND} ($" + hex(static_cast<uint16_t>(hi << 8) + lo, 4) + ")";
	}
	else if (instructions[current_opcode].addressing_mode == addressing_mode::RELATIVE)
	{
		value = bus.read(next, true);
		next++;
		instruction += "{REL} $" + hex(value, 2) + " [$" + hex(next + value, 4) + "]";
	}

	return instruction;
}

bool noentiendo::cpu::find_previous_instruction(const uint16_t current, uint16_t& previous) const
{
	auto check_instruction = [&current, &previous, *this](auto distance) {

		if (current == 0x0000 || current == 0xFFFF)
		{
			return false;
		}

		auto offset = current - distance;
		
		const auto opcode = bus.read(offset, true);
		if (instructions[opcode].bytes == distance)
		{
			previous = offset;
			return true;
		}
		
		return false;
	};
	
	 return check_instruction(3) || check_instruction(2) || check_instruction(1);
}

bool noentiendo::cpu::find_next_instruction(const uint16_t current, uint16_t& next) const
{
	auto check_instruction = [&current, &next, *this]() {
		if (current == 0x0000 || current == 0xFFFF)
		{
			return false;
		}

		const auto opcode = bus.read(current, true);
		if (static_cast<noentiendo::opcode>(opcode) == opcode::XXX)
			return false;
		
		next = current + instructions[opcode].bytes;
		return true;
	};

	return check_instruction();
}