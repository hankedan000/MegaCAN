#pragma once

#include <stddef.h>
#include <stdint.h>

#include "MegaCAN/Panic.h"

namespace MegaCAN
{

    template<
        typename T,
        size_t Capacity,
        typename SizeType = size_t
    >
    class StaticVector {
        static_assert(Capacity <= static_cast<SizeType>(-1),
                    "Capacity exceeds SizeType range");

    public:
        using value_type = T;
        using size_type = SizeType;
        using iterator = T*;
        using const_iterator = const T*;

        StaticVector() = default;

        // ----- Capacity -----

        constexpr size_t capacity() const {
            return Capacity;
        }

        size_type size() const {
            return size_;
        }

        constexpr bool empty() const {
            return size_ == 0;
        }

        constexpr bool full() const {
            return size_ >= Capacity;
        }

        // ----- Element Access -----

        T& operator[](size_type index) {
            return data_[index];  // no bounds check (like std::vector)
        }

        const T& operator[](size_type index) const {
            return data_[index];
        }

        T& at(size_type index) {
            if (index >= size_) {
                MC_PANIC("StaticVector::at - bad index")
            }
            return data_[index];
        }

        const T& at(size_type index) const {
            if (index >= size_) {
                MC_PANIC("StaticVector::at - bad index")
            }
            return data_[index];
        }

        T& front() {
            if (size_ == 0) {
                MC_PANIC("StaticVector::front - vector empty")
            }
            return data_[0];
        }

        T& back() {
            if (size_ == 0) {
                MC_PANIC("StaticVector::back - vector empty")
            }
            return data_[size_ - 1];
        }

        const T& front() const {
            if (size_ == 0) {
                MC_PANIC("StaticVector::front - vector empty")
            }
            return data_[0];
        }

        const T& back() const {
            if (size_ == 0) {
                MC_PANIC("StaticVector::back - vector empty")
            }
            return data_[size_ - 1];
        }

        T* data() { return data_; }
        const T* data() const { return data_; }

        // ----- Modifiers -----

        void push_back(const T& value) {
            if (size_ >= Capacity) {
                MC_PANIC("StaticVector::push_back - vector full")
            }
            data_[size_++] = value;
        }

        void pop_back() {
            if (size_ == 0) {
                MC_PANIC("StaticVector::pop_back - vector empty")
            }
            --size_;
        }

        void clear() {
            size_ = 0;
        }

        void resize(size_type new_size, const T& fill_value = T()) {
            if (new_size > Capacity) {
                MC_PANIC("StaticVector::resize - bad size")
            }

            if (new_size > size_) {
                for (size_type i = size_; i < new_size; ++i) {
                    data_[i] = fill_value;
                }
            }

            size_ = new_size;
        }

        // ----- Iterators -----

        iterator begin() { return data_; }
        iterator end() { return data_ + size_; }

        const_iterator begin() const { return data_; }
        const_iterator end() const { return data_ + size_; }

    private:
        T data_[Capacity];
        size_type size_ = 0u;
        
    };

}
