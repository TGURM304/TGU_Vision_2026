#include "tools/logger.hpp"
#include <cmath>

#define RAD (180.f / M_PI)

#ifndef NDEBUG
// Debug (NOT Release)
#define DEBUG(fmt, ...) tools::logger()->debug(fmt, ##__VA_ARGS__);
#else
// Release
#define DEBUG(fmt, ...) ((void)0)
#endif
