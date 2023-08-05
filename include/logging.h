#ifndef __LOGGING_H__
#define __LOGGING_H__

#include <dlg/dlg.h>
#include <assert.h>

#ifdef NDEBUG
#define log_assert(x) (void)(x)
#else
#define log_assert(x) assert(x)
#endif

#define LOG_DEBUG dlg_debug
#define LOG_INFO dlg_info
#define LOG_WARN dlg_warn
#define LOG_ERROR dlg_error

#define LOG(_level, _format, _args...) \
    LOG_##_level(_format, ##_args)

void logging_init(void);

#endif // __LOGGING_H__
