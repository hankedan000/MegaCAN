#pragma once

#ifdef ARDUINO
#else
#include <memory>
#endif

namespace MegaCAN
{
#ifdef ARDUINO
    template <typename T>
    using SharedPtr = T *;
#else
#include <memory>
    template <typename T>
    using SharedPtr = std::shared_ptr<T>;
#endif
}