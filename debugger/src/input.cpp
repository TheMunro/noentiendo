#include "input.hpp"
#include "window.hpp"
#include "fmt/format.h"

#include <imgui.h>
#include <SDL_events.h>
#include <backends/imgui_impl_sdl.h>

debugger::input::input(debugger::window& window)
	: window{window}
{
	auto& io = ImGui::GetIO();
	(void)io;
	//io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
	//io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

	ImGui_ImplSDL2_InitForVulkan(window.get_handle());
}

debugger::input::~input()
{
	ImGui_ImplSDL2_Shutdown();
}

void debugger::input::update()
{
	// Poll and handle events (inputs, window resize, etc.)
    // You can read the io.WantCaptureMouse, io.WantCaptureKeyboard flags to tell if dear imgui wants to use your inputs.
    // - When io.WantCaptureMouse is true, do not dispatch mouse input data to your main application.
    // - When io.WantCaptureKeyboard is true, do not dispatch keyboard input data to your main application.
    // Generally you may always pass all inputs to dear imgui, and hide them from your application based on those two flags.
	SDL_Event event;
	while (SDL_PollEvent(&event))
	{
		ImGui_ImplSDL2_ProcessEvent(&event);
		if (event.type == SDL_QUIT)
			done = true;
		if (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_CLOSE && event.window.windowID == SDL_GetWindowID(window.get_handle()))
			done = true;
	}
	
	ImGui_ImplSDL2_NewFrame(window.get_handle());
}