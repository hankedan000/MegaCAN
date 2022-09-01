#ifndef EXTENDED_CAN_TYPES_H_
#define EXTENDED_CAN_TYPES_H_

#include "stdint.h"

namespace MegaCAN
{
namespace Ext
{

enum TableType_E
{
  eRam, eFlash, eNull
};

struct TableDescriptor_t
{
  void *tableData;
  uint16_t tableSize;
  TableType_E tableType;
  uint16_t flashOffset;
};

}
}

#endif