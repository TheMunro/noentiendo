#pragma once

#include "backends/imgui_impl_sdl.h"
#include <SDL.h>

#include <string>

namespace debugger
{

class window
{
public:
	explicit window(std::string name, int width = 1280, int height = 720);
	~window();
	
	window(const window&) = delete;
	window(window&&) = delete;
	window operator=(const window&) = delete;
	window operator=(window&&) = delete;

	[[nodiscard]] SDL_Window* get_handle() const { return sdl_window; }

private:
	SDL_Window* sdl_window;
};

}	 // namespace debugger