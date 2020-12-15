#pragma once
#include "cpu.hpp"

template <nes_emu::cpu::AddressingModeFunc Mode>
void nes_emu::cpu::instruction_adc()
{
	const auto page = std::invoke(Mode, *this);

	if (page)
		++cycles_remaining;
}

template <nes_emu::cpu::AddressingModeFunc Mode>
void nes_emu::cpu::instruction_and()
{
	const auto page = std::invoke(Mode, *this);
	
	fetch<Mode>();

	register_accumulator &= fetched;

	set_flag(processor_status_register::zero, register_accumulator == 0x00);
	set_flag(processor_status_register::negative, register_accumulator & 0x80);
}

template <nes_emu::cpu::AddressingModeFunc Mode>
void nes_emu::cpu::instruction_asl()
{
	const auto page = std::invoke(Mode, *this);
	
	fetch<Mode>();

	const std::uint16_t result = static_cast<std::uint16_t>(fetched) << 1;

	set_flag(processor_status_register::carry, (result & 0xFF00) != 0x0000);
	set_flag(processor_status_register::zero, (result & 0xFF) == 0x00);
	set_flag(processor_status_register::negative, result & 0x80);

	if ((instructions[opcode].addressing_mode == &nes_emu::cpu::address_mode_implicit))
		register_accumulator = result & 0x00FF;
	else
		write(address_absolute, result & 0x00FF);
	
	if (page)
		++cycles_remaining;
}

template <nes_emu::cpu::AddressingModeFunc Mode, nes_emu::processor_status_register flag, bool is_set>
void nes_emu::cpu::branch()
{
	const auto page = std::invoke(Mode, *this);
	
	if (get_flag(flag) == is_set)
	{
		++cycles_remaining;
		address_absolute = register_program_counter + address_relative;

		if ((address_absolute & 0xFF00) != (register_program_counter & 0xFF00))
			++cycles_remaining;

		register_program_counter = address_absolute;
	}
	
	if (page)
		++cycles_remaining;
}

template <nes_emu::cpu::AddressingModeFunc Mode>
void nes_emu::cpu::instruction_bcc()
{
	branch<mode, processor_status_register::carry, false>();
}

template <nes_emu::cpu::AddressingModeFunc Mode>
void nes_emu::cpu::instruction_bcs()
{
	branch<mode, processor_status_register::carry, true>();
}

template <nes_emu::cpu::AddressingModeFunc Mode>
void nes_emu::cpu::instruction_beq()
{
	branch<mode, processor_status_register::zero, true>();
}

template <nes_emu::cpu::AddressingModeFunc Mode>
void nes_emu::cpu::instruction_bit()
{
	const auto page = std::invoke(Mode, *this);
	
	fetch<Mode>();

	const std::uint16_t result = register_accumulator & fetched;

	set_flag(processor_status_register::zero, (result & 0xFF) == 0x00);
	set_flag(processor_status_register::overflow, fetched & 0x40);
	set_flag(processor_status_register::negative, fetched & 0x80);
}

template <nes_emu::cpu::AddressingModeFunc Mode>
void nes_emu::cpu::instruction_bmi()
{
	const auto page = std::invoke(Mode, *this);
	branch<mode, processor_status_register::negative, true>();
}

template <nes_emu::cpu::AddressingModeFunc Mode>
void nes_emu::cpu::instruction_bne()
{
	const auto page = std::invoke(Mode, *this);
	branch<mode, processor_status_register::zero, false>();
}

template <nes_emu::cpu::AddressingModeFunc Mode>
void nes_emu::cpu::instruction_bpl()
{
	const auto page = std::invoke(Mode, *this);
	branch<mode, processor_status_register::negative, false>();
}

template <nes_emu::cpu::AddressingModeFunc Mode>
void nes_emu::cpu::instruction_brk()
{
	const auto page = std::invoke(Mode, *this);
	//program counter is incremented here as brk uses implicit addressing, which does not modify the program counter
	//so we have to increment here to ensure that we get the next opcode to cache in the stack
	++register_program_counter;

	const auto hi_byte = static_cast<std::uint8_t>(register_program_counter >> 8);
	const auto lo_byte = static_cast<std::uint8_t>(register_program_counter);

	set_flag(processor_status_register::interrupt_disable, true);

	//push program counter to the stack
	//TODO: Check for overflow of the stack memory page?
	write(stack_address_offset + register_stack_pointer, hi_byte);
	register_stack_pointer++;
	write(stack_address_offset + register_stack_pointer, lo_byte);
	register_stack_pointer++;

	//push status to the stack
	set_flag(processor_status_register::break_command, true);
	write(stack_address_offset + register_stack_pointer, static_cast<std::uint8_t>(register_program_counter));
	register_stack_pointer++;
	set_flag(processor_status_register::break_command, false);

	//read interrupt
	const auto irq_hi_byte = read(0xFFFF);
	const auto irg_lo_byte = read(0xFFFE);

	register_program_counter = static_cast<std::uint16_t>(irq_hi_byte << 8) + irg_lo_byte;
}

template <nes_emu::cpu::AddressingModeFunc Mode>
void nes_emu::cpu::instruction_bvc()
{
	const auto page = std::invoke(Mode, *this);
	branch<mode, processor_status_register::overflow, false>();
}

template <nes_emu::cpu::AddressingModeFunc Mode>
void nes_emu::cpu::instruction_bvs()
{
	const auto page = std::invoke(Mode, *this);
	branch<Mode, processor_status_register::overflow, true>();
}

template <nes_emu::cpu::AddressingModeFunc Mode>
void nes_emu::cpu::instruction_clc()
{
	const auto page = std::invoke(Mode, *this);
	set_flag(processor_status_register::carry, false);
}

template <nes_emu::cpu::AddressingModeFunc Mode>
void nes_emu::cpu::instruction_cld()
{
	const auto page = std::invoke(Mode, *this);
	set_flag(processor_status_register::decimal_mode, false);
}

template <nes_emu::cpu::AddressingModeFunc Mode>
void nes_emu::cpu::instruction_cli()
{
	const auto page = std::invoke(Mode, *this);
	set_flag(processor_status_register::interrupt_disable, false);
}

template <nes_emu::cpu::AddressingModeFunc Mode>
void nes_emu::cpu::instruction_clv()
{
	const auto page = std::invoke(Mode, *this);
	set_flag(processor_status_register::overflow, false);
}

template <nes_emu::cpu::AddressingModeFunc Mode>
void nes_emu::cpu::compare(std::uint8_t& target_register)
{
	const auto page = std::invoke(Mode, *this);
	
	fetch<Mode>();

	const std::uint16_t result = target_register - fetched;

	set_flag(processor_status_register::carry, register_accumulator >= fetched);
	set_flag(processor_status_register::overflow, (register_accumulator & fetched) == 0x00);
	set_flag(processor_status_register::negative, result & 0x80);
}

template <nes_emu::cpu::AddressingModeFunc Mode>
void nes_emu::cpu::instruction_cmp()
{
	compare<mode>(register_accumulator);
}

template <nes_emu::cpu::AddressingModeFunc Mode>
void nes_emu::cpu::instruction_cpx()
{
	compare<mode>(register_x);
}

template <nes_emu::cpu::AddressingModeFunc Mode>
void nes_emu::cpu::instruction_cpy()
{
	compare<mode>(register_y);
}

template <nes_emu::cpu::AddressingModeFunc Mode>
void nes_emu::cpu::instruction_dec()
{
	const auto page = std::invoke(Mode, *this);
	fetch<Mode>();

	const std::uint16_t result = fetched - 1;
	write(address_absolute, result & 0x00FF);

	set_flag(processor_status_register::zero, result == 0x00);
	set_flag(processor_status_register::negative, result & 0x80);
}

template <nes_emu::cpu::AddressingModeFunc Mode>
void nes_emu::cpu::instruction_dex()
{
	const auto page = std::invoke(Mode, *this);
	--register_x;

	set_flag(processor_status_register::zero, register_x == 0x00);
	set_flag(processor_status_register::negative, register_x & 0x80);
}

template <nes_emu::cpu::AddressingModeFunc Mode>
void nes_emu::cpu::instruction_dey()
{
	const auto page = std::invoke(Mode, *this);
	--register_y;

	set_flag(processor_status_register::zero, register_y == 0x00);
	set_flag(processor_status_register::negative, register_y & 0x80);
}

template <nes_emu::cpu::AddressingModeFunc Mode>
void nes_emu::cpu::instruction_eor()
{
	const auto page = std::invoke(Mode, *this);
	fetch<Mode>();

	register_accumulator ^= fetched;

	set_flag(processor_status_register::zero, register_accumulator == 0x00);
	set_flag(processor_status_register::negative, register_accumulator & 0x80);
}

template <nes_emu::cpu::AddressingModeFunc Mode>
void nes_emu::cpu::instruction_inc()
{
	const auto page = std::invoke(Mode, *this);
	fetch<Mode>();

	const std::uint16_t result = fetched + 1;
	write(address_absolute, result & 0x00FF);

	set_flag(processor_status_register::zero, result == 0x00);
	set_flag(processor_status_register::negative, result & 0x80);
}

template <nes_emu::cpu::AddressingModeFunc Mode>
void nes_emu::cpu::instruction_inx()
{
	const auto page = std::invoke(Mode, *this);
	++register_x;

	set_flag(processor_status_register::zero, register_x == 0x00);
	set_flag(processor_status_register::negative, register_x & 0x80);
}

template <nes_emu::cpu::AddressingModeFunc Mode>
void nes_emu::cpu::instruction_iny()
{
	const auto page = std::invoke(Mode, *this);
	++register_y;

	set_flag(processor_status_register::zero, register_y == 0x00);
	set_flag(processor_status_register::negative, register_y & 0x80);
}

template <nes_emu::cpu::AddressingModeFunc Mode>
void nes_emu::cpu::instruction_jmp()
{
	const auto page = std::invoke(Mode, *this);
	register_program_counter = address_absolute;
}

template <nes_emu::cpu::AddressingModeFunc Mode>
void nes_emu::cpu::instruction_jsr()
{
	const auto page = std::invoke(Mode, *this);

	--register_program_counter;

	write(stack_address_offset + register_stack_pointer, register_program_counter >> 8);
	++register_stack_pointer;
	write(stack_address_offset + register_stack_pointer, register_program_counter);
	++register_stack_pointer;
	
	register_program_counter = address_absolute;
}

template <nes_emu::cpu::AddressingModeFunc Mode>
void nes_emu::cpu::load_register(std::uint8_t& target_register)
{
	const auto page = std::invoke(Mode, *this);

	fetch<Mode>();

	target_register = fetched;

	set_flag(processor_status_register::zero, target_register == 0x00);
	set_flag(processor_status_register::negative, target_register & 0x80);

	if (page)
		++cycles_remaining;
}

template <nes_emu::cpu::AddressingModeFunc Mode>
void nes_emu::cpu::instruction_lda()
{
	load_register<Mode>(register_accumulator);
}

template <nes_emu::cpu::AddressingModeFunc Mode>
void nes_emu::cpu::instruction_ldx()
{
	load_register<Mode>(register_x);
}

template <nes_emu::cpu::AddressingModeFunc Mode>
void nes_emu::cpu::instruction_ldy()
{
	load_register<Mode>(register_y);
}

template <nes_emu::cpu::AddressingModeFunc Mode>
void nes_emu::cpu::instruction_lsr()
{
	const auto page = std::invoke(Mode, *this);
	
	fetch<Mode>();
	
	set_flag(processor_status_register::carry, fetched & 0x01);

	const std::uint16_t result = fetched >> 1;

	if (Mode == &cpu::address_mode_accumulator)
		register_accumulator = result;
	else
		write(address_absolute, result);

	set_flag(processor_status_register::zero, result & 0x00);
	set_flag(processor_status_register::negative, result & 0x80);
}

template <nes_emu::cpu::AddressingModeFunc Mode>
void nes_emu::cpu::instruction_nop()
{
	const auto page = std::invoke(Mode, *this);
}

template <nes_emu::cpu::AddressingModeFunc Mode>
void nes_emu::cpu::instruction_ora()
{
	const auto page = std::invoke(Mode, *this);
	fetch<Mode>();

	register_accumulator |= fetched;

	set_flag(processor_status_register::zero, register_accumulator == 0x00);
	set_flag(processor_status_register::negative, register_accumulator & 0x80);

	if (page)
		++cycles_remaining;
}

template <nes_emu::cpu::AddressingModeFunc Mode>
void nes_emu::cpu::instruction_pha()
{
	const auto page = std::invoke(Mode, *this);
}

template <nes_emu::cpu::AddressingModeFunc Mode>
void nes_emu::cpu::instruction_php()
{
	const auto page = std::invoke(Mode, *this);
}

template <nes_emu::cpu::AddressingModeFunc Mode>
void nes_emu::cpu::instruction_pla()
{
	const auto page = std::invoke(Mode, *this);
}

template <nes_emu::cpu::AddressingModeFunc Mode>
void nes_emu::cpu::instruction_plp()
{
	const auto page = std::invoke(Mode, *this);
}
template <nes_emu::cpu::AddressingModeFunc Mode>
void nes_emu::cpu::instruction_rol()
{
	const auto page = std::invoke(Mode, *this);
}

template <nes_emu::cpu::AddressingModeFunc Mode>
void nes_emu::cpu::instruction_ror()
{
	const auto page = std::invoke(Mode, *this);
}

template <nes_emu::cpu::AddressingModeFunc Mode>
void nes_emu::cpu::instruction_rti()
{
	const auto page = std::invoke(Mode, *this);
}

template <nes_emu::cpu::AddressingModeFunc Mode>
void nes_emu::cpu::instruction_rts()
{
	const auto page = std::invoke(Mode, *this);
}

template <nes_emu::cpu::AddressingModeFunc Mode>
void nes_emu::cpu::instruction_sbc()
{
	const auto page = std::invoke(Mode, *this);
}

template <nes_emu::cpu::AddressingModeFunc Mode>
void nes_emu::cpu::instruction_sec()
{
	const auto page = std::invoke(Mode, *this);
}

template <nes_emu::cpu::AddressingModeFunc Mode>
void nes_emu::cpu::instruction_sed()
{
	const auto page = std::invoke(Mode, *this);
}

template <nes_emu::cpu::AddressingModeFunc Mode>
void nes_emu::cpu::instruction_sei()
{
	const auto page = std::invoke(Mode, *this);
}

template <nes_emu::cpu::AddressingModeFunc Mode>
void nes_emu::cpu::instruction_sta()
{
	const auto page = std::invoke(Mode, *this);
}

template <nes_emu::cpu::AddressingModeFunc Mode>
void nes_emu::cpu::instruction_stx()
{
	const auto page = std::invoke(Mode, *this);
}

template <nes_emu::cpu::AddressingModeFunc Mode>
void nes_emu::cpu::instruction_sty()
{
	const auto page = std::invoke(Mode, *this);
}

template <nes_emu::cpu::AddressingModeFunc Mode>
void nes_emu::cpu::instruction_tax()
{
	const auto page = std::invoke(Mode, *this);
}

template <nes_emu::cpu::AddressingModeFunc Mode>
void nes_emu::cpu::instruction_tay()
{
	const auto page = std::invoke(Mode, *this);
}

template <nes_emu::cpu::AddressingModeFunc Mode>
void nes_emu::cpu::instruction_tsx()
{
	const auto page = std::invoke(Mode, *this);
}

template <nes_emu::cpu::AddressingModeFunc Mode>
void nes_emu::cpu::instruction_txa()
{
	const auto page = std::invoke(Mode, *this);
}

template <nes_emu::cpu::AddressingModeFunc Mode>
void nes_emu::cpu::instruction_txs()
{
	const auto page = std::invoke(Mode, *this);
}

template <nes_emu::cpu::AddressingModeFunc Mode>
void nes_emu::cpu::instruction_tya()
{
	const auto page = std::invoke(Mode, *this);
}