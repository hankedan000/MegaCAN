#pragma once

#include <stddef.h> // for size_t

#include "MegaCAN/Panic.h"

namespace MegaCAN
{

    template<typename T, size_t N>
    class StaticArray {
    public:
        using value_type = T;
        using size_type = size_t;
        using iterator = T*;
        using const_iterator = const T*;

        // Element access
        T& operator[](size_t index) {
            return data_[index];
        }

        const T& operator[](size_t index) const {
            return data_[index];
        }

        T& at(size_t index) {
            // Optional bounds check (remove if you want max performance)
            if (index >= N) {
                MC_PANIC("StaticArray::at - bad index")
            }
            return data_[index];
        }

        const T& at(size_t index) const {
            if (index >= N) {
                MC_PANIC("StaticArray::at - bad index")
            }
            return data_[index];
        }

        T& front() {
            return data_[0];
        }

        T& back() {
            return data_[N - 1];
        }

        const T& front() const {
            return data_[0];
        }

        const T& back() const {
            return data_[N - 1];
        }

        // Capacity
        constexpr size_t size() const {
            return N;
        }

        constexpr bool empty() const {
            return N == 0;
        }

        // Iterators (range-for support)
        iterator begin() {
            return data_;
        }

        iterator end() {
            return data_ + N;
        }

        const_iterator begin() const {
            return data_;
        }

        const_iterator end() const {
            return data_ + N;
        }

        const_iterator cbegin() const {
            return data_;
        }

        const_iterator cend() const {
            return data_ + N;
        }

        // Fill helper
        void fill(const T& value) {
            for (size_t i = 0; i < N; ++i) {
                data_[i] = value;
            }
        }

        // Raw data pointer
        T* data() {
            return data_;
        }

        const T* data() const {
            return data_;
        }

    private:
        T data_[N];
    };

}
