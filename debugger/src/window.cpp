#include "window.hpp"
#include "fmt/format.h"

debugger::window::window(std::string name, const int width, const int height)
{
    // Setup SDL
	if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER | SDL_INIT_GAMECONTROLLER) != 0)
		throw (fmt::format("Error: %s\n", SDL_GetError()));
	
	// Setup window
	const auto window_flags = static_cast<SDL_WindowFlags>(SDL_WINDOW_VULKAN | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
	sdl_window = SDL_CreateWindow(name.c_str(), SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, width, height, window_flags);

	// Setup Dear ImGui context
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	
	// Setup Dear ImGui style
	ImGui::StyleColorsDark();
	//ImGui::StyleColorsClassic();
}


debugger::window::~window()
{
	ImGui::DestroyContext();
	SDL_DestroyWindow(sdl_window);
	SDL_Quit();
}