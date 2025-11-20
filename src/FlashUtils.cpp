#include "FlashUtils.h"

#include <Arduino.h>

namespace FlashUtils
{
  
  template <>
  uint8_t
  readBE(
    const fsize_t offset)
  {
    return EEPROM.read(offset);
  }

  template <>
  int8_t
  readBE(
    const fsize_t offset)
  {
    return EEPROM.read(offset);
  }
  
  template <>
  uint16_t
  readBE(
    const fsize_t offset)
  {
    return EEPROM_GetBigU16(offset);
  }

  template <>
  int16_t
  readBE(
    const fsize_t offset)
  {
    return EEPROM_GetBigS16(offset);
  }

  template <>
  uint32_t
  readBE(
    const fsize_t offset)
  {
    return EEPROM_GetBigU32(offset);
  }

  template <>
  int32_t
  readBE(
    const fsize_t offset)
  {
    return EEPROM_GetBigS32(offset);
  }

  template <>
  float
  readBE(
    const fsize_t offset)
  {
    uint32_t u32 = EEPROM_GetBigU32(offset);
    return *(float*)((void*)(&u32));
  }

  template <>
  double
  readBE(
    const fsize_t offset)
  {
    // AVR libc doesn't support 64bit doubles (they're really just 32bits floats)
    static_assert(sizeof(double) == 4);
    uint32_t u32 = EEPROM_GetBigU32(offset);
    return *(double*)((void*)(&u32));
  }

  template <>
  void
  writeBE(
    const fsize_t offset,
    const uint8_t value)
  {
    EEPROM.write(offset,value);
  }

  template <>
  void
  writeBE(
    const fsize_t offset,
    const int8_t  value)
  {
    EEPROM.write(offset,value);
  }
  
  template <>
  void
  writeBE(
    const fsize_t  offset,
    const uint16_t value)
  {
    EEPROM_SetBigU16(offset,value);
  }
  
  template <>
  void
  writeBE(
    const fsize_t offset,
    const int16_t value)
  {
    EEPROM_SetBigS16(offset,value);
  }

  template <>
  void
  writeBE(
    const fsize_t  offset,
    const uint32_t value)
  {
    EEPROM_SetBigU32(offset,value);
  }

  template <>
  void
  writeBE(
    const fsize_t offset,
    const int32_t value)
  {
    EEPROM_SetBigS32(offset,value);
  }

  template <>
  void
  writeBE(
    const fsize_t offset,
    const float   value)
  {
    EEPROM_SetBigU32(offset,*reinterpret_cast<const uint32_t *>(&value));
  }

  template <>
  void
  writeBE(
    const fsize_t offset,
    const double  value)
  {
    // AVR libc doesn't support 64bit doubles (they're really just 32bits floats)
    static_assert(sizeof(double) == 4);
    EEPROM_SetBigU32(offset,*reinterpret_cast<const uint32_t *>(&value));
  }

}
