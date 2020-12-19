#pragma once

namespace debugger
{

class window;

class input
{
public:
	explicit input(window& window);
	~input();

	void update();

	[[nodiscard]] bool exit() const { return done; }

private:
	window& window;
	bool done = false;
};

}	 // namespace debugger