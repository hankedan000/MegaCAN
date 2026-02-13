#pragma once

#ifdef ARDUINO
#include <avr/wdt.h>
#endif

namespace MegaCAN
{

#ifdef ARDUINO
    inline void resetWatchdog() {wdt_reset();}
#else
    inline void resetWatchdog() {/* do nothing in normal c++*/}
#endif

}