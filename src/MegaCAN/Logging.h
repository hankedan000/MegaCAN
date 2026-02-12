#ifndef MEGACAN_LOGGING_H_
#define MEGACAN_LOGGING_H_

// default threshold to INFO if not set within translation unit
#ifndef MC_LOG_THRESHOLD
#define MC_LOG_THRESHOLD MC_LOG_LVL_INFO_I
#endif

// default buffer size used for log message formatting
#ifndef MC_LOG_BUFFER_SIZE
#define MC_LOG_BUFFER_SIZE 128
#endif

#include <stdarg.h> // for va_list
#ifndef ARDUINO
#include <stdio.h> // snprintf()
#endif

#define MC_LOG_LVL_DEBUG_I   10
#define MC_LOG_LVL_INFO_I    20
#define MC_LOG_LVL_WARN_I    30
#define MC_LOG_LVL_ERROR_I   40
#define MC_LOG_LVL_NO_LOGS_I 60

// the "##" will remove comma when there are no variable arguments provided
#ifdef ARDUINO
#include <avr/pgmspace.h>
#define MC_LOG(lvl,fmt,...) MegaCAN::Logging.log(lvl,PSTR(fmt),##__VA_ARGS__);
#else
#define MC_LOG(lvl,fmt,...) MegaCAN::Logging.log(lvl,fmt,##__VA_ARGS__);
#endif

#if (MC_LOG_LVL_DEBUG_I < MC_LOG_THRESHOLD)
  #define MC_LOG_DEBUG(fmt,...) {}
#else
  #define MC_LOG_DEBUG(fmt,...) MC_LOG(MegaCAN::LogLevel::DEBUG,fmt,##__VA_ARGS__)
#endif

#if (MC_LOG_LVL_INFO_I < MC_LOG_THRESHOLD)
  #define MC_LOG_INFO(fmt,...) {}
#else
  #define MC_LOG_INFO(fmt,...) MC_LOG(MegaCAN::LogLevel::INFO,fmt,##__VA_ARGS__)
#endif

#if (MC_LOG_LVL_WARN_I < MC_LOG_THRESHOLD)
  #define MC_LOG_WARN(fmt,...) {}
#else
  #define MC_LOG_WARN(fmt,...) MC_LOG(MegaCAN::LogLevel::WARN,fmt,##__VA_ARGS__)
#endif

#if (MC_LOG_LVL_ERROR_I < MC_LOG_THRESHOLD)
  #define MC_LOG_ERROR(fmt,...) {}
#else
  #define MC_LOG_ERROR(fmt,...) MC_LOG(MegaCAN::LogLevel::ERROR,fmt,##__VA_ARGS__)
#endif

namespace MegaCAN
{
  enum struct LogLevel
  {
    DEBUG   = MC_LOG_LVL_DEBUG_I,
    INFO    = MC_LOG_LVL_INFO_I,
    WARN    = MC_LOG_LVL_WARN_I,
    ERROR   = MC_LOG_LVL_ERROR_I,
    NO_LOGS = MC_LOG_LVL_NO_LOGS_I
  };

  using LogCallback = void (*)(const LogLevel lvl, const char * msg);

  class Logger
  {
    public:
      inline void setCallback(const LogCallback newCallback)
      {
#ifdef MC_LOG_ENABLED
        logCallback_ = newCallback;
#else
        // avoid unused arg warnings
        (void)newCallback;
#endif
      }

#ifdef ARDUINO
      void log(const LogLevel lvl, PGM_P fmt, ...) __attribute__((format(printf, 3, 4)));
#else
      void log(const LogLevel lvl, const char * fmt, ...) __attribute__((format(printf, 3, 4)));
#endif
    
    private:
#ifdef MC_LOG_ENABLED
      LogCallback logCallback_ = nullptr;
      char buffer_[MC_LOG_BUFFER_SIZE];
#endif
  };

  extern Logger Logging;
}

#endif