#include "MegaCAN/Logging.h"

namespace MegaCAN
{
  // declare extern variables
  Logger Logging = {};

#ifdef ARDUINO
  void Logger::log(const LogLevel lvl, PGM_P fmt, ...)
#else
  void Logger::log(const LogLevel lvl, const char * fmt, ...)
#endif
  {
#ifdef MC_LOG_ENABLED
    if ( ! __logCallback)
    {
      return;
    }

    // format the log message with out fixed buffer
    va_list args;
    va_start(args, fmt);
#ifdef ARDUINO
    vsnprintf_P(buffer_, sizeof(buffer_), fmt, args);
#else
    vsnprintf(buffer_, sizeof(buffer_), fmt, args);
#endif
    va_end(args);

    // send the formatted message to user callback
    __logCallback(lvl, buffer_);
#else
    // avoid unused arg warnings
    (void)lvl;
    (void)fmt;
#endif
  }
}