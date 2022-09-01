#ifndef ENDIAN_UTILS_H_
#define ENDIAN_UTILS_H_

namespace EndianUtils
{

/**
 * Sets a value in little endian format (native) to big-endian format
 */
template <typename T>
void
setBE(T &var, const T &val)
{
  size_t bytesize = sizeof(T);
  if (bytesize == 2) {
    var = __builtin_bswap16(val);
  }
  else if (bytesize == 4) {
    var = __builtin_bswap32(val);
  }
  else if (bytesize == 8) {
    var = __builtin_bswap64(val);
  }
}

/**
 * Gets a value in big-endian format and converts to little-endian format (native)
 */
template <typename T>
T
getBE(const T &var)
{
  size_t bytesize = sizeof(T);
  if (bytesize == 2) {
    return __builtin_bswap16(var);
  }
  else if (bytesize == 4) {
    return __builtin_bswap32(var);
  }
  else if (bytesize == 8) {
    return __builtin_bswap64(var);
  }
}

}

#endif