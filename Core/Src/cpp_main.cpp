#include "Config.hpp"
#include "Timer.hpp"
#include "main.h"
#include <cstdio>

int cpp_main()
{
    main_timer::activate();
    cycle::dt = 0.05f;

    Timer timer;
    float elapsed_time = 0.0f;
    timer.reset();
    timer.start();

    while (true)
    {

        while(timer.read() - elapsed_time < cycle::dt){};
        elapsed_time = timer.read();
    }

    return 0;
}