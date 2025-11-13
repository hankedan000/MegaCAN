#pragma once

namespace EndianUtils
{

/**
 * Sets a value in little endian format (native) to big-endian format
 */
template <typename T>
void
setBE(
  T     & var,
  const T val)
{
  if (sizeof(T) == 2) {
    var = __builtin_bswap16(val);
  } else if (sizeof(T) == 4) {
    var = __builtin_bswap32(val);
  } else if (sizeof(T) == 8) {
    var = __builtin_bswap64(val);
  }
}

/**
 * Gets a value in big-endian format and converts to little-endian format (native)
 */
template <typename T>
T
getBE(
  const T var)
{
  if (sizeof(T) == 2) {
    return __builtin_bswap16(var);
  } else if (sizeof(T) == 4) {
    return __builtin_bswap32(var);
  } else if (sizeof(T) == 8) {
    return __builtin_bswap64(var);
  }
}

}
