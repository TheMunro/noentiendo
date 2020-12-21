#pragma once

#include <imgui.h>


#include "input.hpp"
#include "nes.hpp"
#include "renderer.hpp"
#include "window.hpp"

namespace debugger
{

class debugger
{
public:
	debugger(std::string name);

	void run();

	[[nodiscard]] bool exit() const { return input.exit(); }

private:
	void render();

private:
	std::unique_ptr<nes_emu::nes> nes;
	
	window window;
	input input;
	renderer renderer;

	bool show_demo_window = true;
	bool show_program_window = true;
	bool show_memory_window = true;
	bool show_cpu_window = true;

	ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
};

}	 // namespace debugger