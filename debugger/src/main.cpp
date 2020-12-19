#include <SDL.h>

#include "debugger.hpp"

int main(int, char**)
{
	debugger::debugger debugger("noentiendo");
	
    while (!debugger.exit())
    {
    	//do ui stuff
        debugger.run();
    }


    return 0;
}
