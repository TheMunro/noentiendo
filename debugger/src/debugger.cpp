//STDLIB
#include <string>
#include <utility>

//EXTERNAL
#include <imgui.h>
#include <fmt/format.h>

//LOCAL
#include "debugger.hpp"

#include <imgui_internal.h>


#include "cartridge.hpp"
#include "memory_editor.hpp"


debugger::debugger::debugger(std::string name)
	: nes{noentiendo::nes::build()}
	, window{std::move(name)}
	, input{window}
	, renderer{window}
{
}

void debugger::debugger::run()
{
	input.update();
	renderer.update();

	//do ui stuff
	render();

	//render
	renderer.render(clear_color);
}

std::string get_register_status(noentiendo::bitfield<noentiendo::processor_status_register> status)
{


	//return fmt::format("ST: ", );
	return "";
}

void debugger::debugger::render()
{
	// 1. Show the big demo window (Most of the sample code is in ImGui::ShowDemoWindow()! You can browse its code to learn more about Dear ImGui!).
	if (show_demo_window)
		ImGui::ShowDemoWindow(&show_demo_window);

	// 2. Show a simple window that we create ourselves. We use a Begin/End pair to created a named window.
	if (show_cpu_window)
	{
		ImGui::Begin("CPU");	  // Create a window called "Hello, world!" and append into it.


		if (ImGui::IsKeyPressed(ImGui::GetKeyIndex(ImGuiKey_Space)))
		{
			do
			{
				nes->get_cpu()->clock();
			} while (nes->get_cpu()->get_cycles_remaining() != 0);
				
		}

		if (ImGui::Button("Load"))
		{
			nes->insert_cartridge();
		}

		if(ImGui::Button("Reset"))
		{
			nes->get_cpu()->reset();
		}

		//auto red = static_cast<ImVec4>(ImColor::HSV(0.0f, 0.6f, 0.6f));
		//auto green = static_cast<ImVec4>(ImColor::HSV(0.2f, 0.6f, 0.6f));
	    ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);
		auto flags = static_cast<int>(nes->get_cpu()->get_register_status().value());

		ImGui::CheckboxFlags("N", &flags, static_cast<int>(noentiendo::processor_status_register::negative));
		ImGui::SameLine();
		ImGui::CheckboxFlags("V", &flags, static_cast<int>(noentiendo::processor_status_register::overflow));
		ImGui::SameLine();
		ImGui::CheckboxFlags("U", &flags, static_cast<int>(noentiendo::processor_status_register::unused));
		ImGui::SameLine();
		ImGui::CheckboxFlags("B", &flags, static_cast<int>(noentiendo::processor_status_register::break_command));
		
		ImGui::CheckboxFlags("D", &flags, static_cast<int>(noentiendo::processor_status_register::decimal_mode));
		ImGui::SameLine();
		ImGui::CheckboxFlags("I", &flags, static_cast<int>(noentiendo::processor_status_register::interrupt_disable));
		ImGui::SameLine();
		ImGui::CheckboxFlags("Z", &flags, static_cast<int>(noentiendo::processor_status_register::zero));
		ImGui::SameLine();
		ImGui::CheckboxFlags("C", &flags, static_cast<int>(noentiendo::processor_status_register::carry));

		ImGui::PushStyleColor(ImGuiCol_CheckMark, (ImVec4)ImColor::HSV(0.0f, 0.6f, 0.6f));
		ImGui::PopStyleColor(1);
		ImGui::PopItemFlag();

		
		const auto program_counter = fmt::format("PC: {:04X}", nes->get_cpu()->get_register_program_counter());
		const auto stack_pointer = fmt::format("SP: {:02X}", nes->get_cpu()->get_register_stack_pointer());
		const auto accumulator = fmt::format("A:  {:02X}", nes->get_cpu()->get_register_accumulator());
		const auto register_x = fmt::format("X:  {:02X}", nes->get_cpu()->get_register_x());
		const auto register_y = fmt::format("Y:  {:02X}", nes->get_cpu()->get_register_y());

		ImGui::Text(program_counter.c_str());
		ImGui::Text(stack_pointer.c_str());
		ImGui::Text(accumulator.c_str());
		ImGui::Text(register_x.c_str());
		ImGui::Text(register_y.c_str());

		if(show_program_window)
		{
			ImGui::Begin("Program", &show_cpu_window);

			if (nes->is_active())
			{
				auto disassembly = nes->disassemble();
				for (const auto& item : disassembly)
					ImGui::Text(item.c_str());
			}

			ImGui::End();
		}

		if (show_memory_window)
		{
			static MemoryEditor memory_editor;
			
			ImGui::Begin("Memory", &show_cpu_window);
			auto* data = nes->get_bus()->get_ram()->data();
			const auto size = nes->get_bus()->get_ram()->size() * sizeof(std::uint8_t);
			memory_editor.DrawContents(data, size);
			ImGui::End();
		}

		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
		ImGui::End();
	}
}