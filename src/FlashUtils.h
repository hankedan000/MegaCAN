#pragma once

#include <stdint.h>
#include <EEPROM.h>

#include "EndianUtils.h"

// reads a big endian 16bit signed/unsigned word from EEPROM flash
#define EEPROM_GetBigS16(ADDR) (int16_t)(((uint16_t)(EEPROM.read(ADDR)) << 8) | EEPROM.read(ADDR + 1))
#define EEPROM_GetBigU16(ADDR) (uint16_t)(((uint16_t)(EEPROM.read(ADDR)) << 8) | EEPROM.read(ADDR + 1))

// writes a big endian 16bit signed/unsigned word to EEPROM flash
#define EEPROM_SetBigU16(ADDR,VAL) EEPROM.write(ADDR,(VAL)>>8&0xFF);EEPROM.write(ADDR + 1,(VAL)&0xFF)
#define EEPROM_SetBigS16(ADDR,VAL) EEPROM_SetBigU16(ADDR,VAL)

// reads a big endian 32bit signed/unsigned word from EEPROM flash
#define EEPROM_GetBigS32(ADDR) (int32_t)(((uint32_t)(EEPROM.read(ADDR)) << 24) | ((uint32_t)(EEPROM.read(ADDR + 1)) << 16) | ((uint32_t)(EEPROM.read(ADDR + 2)) << 8) | EEPROM.read(ADDR + 3))
#define EEPROM_GetBigU32(ADDR) (uint32_t)(((uint32_t)(EEPROM.read(ADDR)) << 24) | ((uint32_t)(EEPROM.read(ADDR + 1)) << 16) | ((uint32_t)(EEPROM.read(ADDR + 2)) << 8) | EEPROM.read(ADDR + 3))

// writes a big endian 32bit signed/unsigned word to EEPROM flash
#define EEPROM_SetBigU32(ADDR,VAL) EEPROM.write(ADDR,(VAL)>>24&0xFF);EEPROM.write(ADDR + 1,(VAL>>16)&0xFF);EEPROM.write(ADDR + 2,(VAL>>8)&0xFF);EEPROM.write(ADDR + 3,(VAL)&0xFF)
#define EEPROM_SetBigS32(ADDR,VAL) EEPROM_SetBigU32(ADDR,VAL)

// type used to define an address offset into EEPROM flash
using fsize_t = uint16_t;

namespace FlashUtils
{

/**
 * Reads a trivial type from flash that was stored in big endian format.
 * The read value is convert into native litte endian format.
 * 
 * @param[in] offset
 * The flash byte offset to read from
 */
template <typename T>
T
readBE(
  const fsize_t offset);

/**
 * Writes a trivial type into flash, converting it from little endian
 * into big endian prior to storage.
 * 
 * @param[in] offset
 * The flash byte offset to write to
 */
template <typename T>
void
writeBE(
  const fsize_t offset,
  const T       value);

template <typename X_T, typename Y_T>
class LUT
{
public:
  using x_t = X_T;
  using y_t = Y_T;

  LUT() = default;

  explicit
  LUT(
    const uint8_t nBins)
   : nBins_(nBins)
  {}

  uint8_t nBins() const {return nBins_;}

  virtual
  X_T
  getX(
    const uint8_t idx) const = 0;
  
  virtual
  Y_T
  getY(
    const uint8_t idx) const = 0;
  
  Y_T
  lerp(
    const X_T value) const
  {
    if (nBins_ <= 1u)
    {
      return value;
    }

    auto loX_Val = getX(0);

    // handle case where value is below the x-axis (-) limit
    if (value <= loX_Val)
    {
      return getY(0);
    }

    for (uint8_t i=1; i<nBins_; i++)
    {
      const auto hiX_Val = getX(i);

      if (loX_Val <= value && value <= hiX_Val)
      {// found bins to interpolate between
        auto loY_Val = getY(i-1u);
        auto hiY_Val = getY(i);
        return static_cast<Y_T>(map(value,loX_Val,hiX_Val,loY_Val,hiY_Val));
      }

      // increment to look at next two bins
      loX_Val = hiX_Val;
    }

    /**
     * never found any values to interpolate between, so value must be higher than
     * the x-axis (+) limit value. return y-axis (+) limit value as result.
     */
    return getY(nBins_ - 1u);
  }

private:
  uint8_t nBins_ = 0u;

};

template <typename X_T, typename Y_T>
class FlashLUT : public LUT<X_T, Y_T>
{
public:
  using lut_t = LUT<X_T, Y_T>;
  static constexpr fsize_t x_size = sizeof(X_T);
  static constexpr fsize_t y_size = sizeof(Y_T);

  FlashLUT() = default;

  FlashLUT(
    const fsize_t xOffset,
    const fsize_t yOffset,
    const uint8_t nBins)
   : lut_t(nBins)
   , xOffset_(xOffset)
   , yOffset_(yOffset)
  {}

  X_T
  getX(
    const uint8_t idx) const override final
  {
    return readBE<X_T>(xOffset_ + idx * x_size);
  }
  
  Y_T
  getY(
    const uint8_t idx) const override final
  {
    return readBE<Y_T>(yOffset_ + idx * y_size);
  }

private:
  fsize_t xOffset_ = 0u;
  fsize_t yOffset_ = 0u;

};

template <typename X_T, typename Y_T>
class RAM_LUT : public LUT<X_T, Y_T>
{
public:
  using lut_t = LUT<X_T, Y_T>;

  RAM_LUT() = default;

  RAM_LUT(
    const X_T   * xBins,
    const Y_T   * yBins,
    const uint8_t nBins)
   : lut_t(nBins)
   , xBins_(xBins)
   , yBins_(yBins)
  {}

  X_T
  getX(
    const uint8_t idx) const override final
  {
    return EndianUtils::getBE(xBins_[idx]);
  }
  
  Y_T
  getY(
    const uint8_t idx) const override final
  {
    return EndianUtils::getBE(yBins_[idx]);
  }

private:
  const X_T * xBins_ = nullptr;
  const Y_T * yBins_ = nullptr;

};

}
