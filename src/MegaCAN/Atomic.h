#pragma once

#ifdef ARDUINO
#include <util/atomic.h>
#define MC_ATOMIC_START ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
#define MC_ATOMIC_END }
#else
#define MC_ATOMIC_START
#define MC_ATOMIC_END
#endif