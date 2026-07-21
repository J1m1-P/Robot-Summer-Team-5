#ifndef VL53L0X_PLATFORM_LOG_H_
#define VL53L0X_PLATFORM_LOG_H_

#include <string.h>

enum {
    TRACE_LEVEL_NONE,
    TRACE_LEVEL_ERRORS,
    TRACE_LEVEL_WARNING,
    TRACE_LEVEL_INFO,
    TRACE_LEVEL_DEBUG,
    TRACE_LEVEL_ALL,
    TRACE_LEVEL_IGNORE
};

enum {
    TRACE_FUNCTION_NONE = 0,
    TRACE_FUNCTION_I2C = 1,
    TRACE_FUNCTION_ALL = 0x7fffffff
};

enum {
    TRACE_MODULE_NONE = 0,
    TRACE_MODULE_API = 1,
    TRACE_MODULE_PLATFORM = 2,
    TRACE_MODULE_ALL = 0x7fffffff
};

#define VL53L0X_ErrLog(...) ((void)0)
#define _LOG_FUNCTION_START(module, fmt, ...) ((void)0)
#define _LOG_FUNCTION_END(module, status, ...) ((void)0)
#define _LOG_FUNCTION_END_FMT(module, status, fmt, ...) ((void)0)
#define VL53L0X_COPYSTRING(destination, source) strcpy((destination), (source))

#endif
