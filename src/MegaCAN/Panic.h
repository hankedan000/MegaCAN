#pragma once

#include "MegaCAN/Logging.h"

#ifdef ARDUINO
#else
#include <stdexcept>
#endif

#ifdef ARDUINO
#define MC_PANIC(MSG) MC_LOG_ERROR(MSG) MegaCAN::__panic();
#else
#define MC_PANIC(MSG) MegaCAN::__panic(MSG);
#endif

namespace MegaCAN
{
#ifdef ARDUINO
    [[noreturn]] inline void __panic() {
        while (true) {}
    }
#else
    [[noreturn]] inline void __panic(const char* msg) {
        throw std::runtime_error(msg);
    }
#endif
}