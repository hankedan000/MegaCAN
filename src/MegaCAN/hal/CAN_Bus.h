#pragma once

#include <stddef.h> // for size_t
#include <stdint.h>

#include "MegaCAN/Atomic.h"
#include "MegaCAN/StaticArray.h"
#include "MegaCAN/StaticVector.h"

#ifndef MC_CAN_MSG_QUEUE_DEPTH
#define MC_CAN_MSG_QUEUE_DEPTH 40
#endif

namespace MegaCAN::HAL
{

    static constexpr unsigned int MAX_CAN_DATA_BYTES = 8u;
    using CAN_DataBuffer = StaticVector<uint8_t, MAX_CAN_DATA_BYTES, uint8_t>;

    struct CAN_Msg
    {
        // 11bit or 29bit identifier
        uint32_t id;
        // true if id is 29bits
        uint8_t ext;
        // buffer of data bytes
        CAN_DataBuffer data;
    };

    class CAN_MsgQueue
    {
    public:
        void
        clear()
        {
            front_ = 0;
            back_ = 0;
            size_ = 0;
        }

        bool
        isFull() const
        {
            MC_ATOMIC_START
            return next(back_) == front_;
            MC_ATOMIC_END
        }

        bool
        isEmpty() const
        {
            MC_ATOMIC_START
            return size_ == 0;
            MC_ATOMIC_END
        }

        void
        push()
        {
            MC_ATOMIC_START
            if (next(back_) != front_)// make sure it's not full
            {
                back_ = next(back_);
                size_++;
            }
            MC_ATOMIC_END
        }

        void
        pop()
        {
            MC_ATOMIC_START
            if (size_ != 0)// make sure it's not empty
            {
                front_ = next(front_);
                size_--;
            }
            MC_ATOMIC_END
        }

        unsigned int
        size()
        {
            MC_ATOMIC_START
            if (back_ >= front_)
            {
                return back_ - front_;
            }
            else
            {
                return back_ + buff_.size() - front_;
            }
            MC_ATOMIC_END
        }

        uint8_t
        capacity() const
        {
            return buff_.size();
        }

        CAN_Msg *
        getFrontPtr()
        {
            return &buff_[front_];
        }

        CAN_Msg *
        getBackPtr()
        {
            return &buff_[back_];
        }

    private:
        uint8_t
        next(
                uint8_t idx) const
        {
            // idx is a local copy, so it's safe to increment
            idx++;
            if (idx >= buff_.size())
            {
                idx = 0;
            }
            return idx;
        }

    private:
        // block of CAN messages to use for the queue
        MegaCAN::StaticArray<CAN_Msg, MC_CAN_MSG_QUEUE_DEPTH> buff_;

        volatile uint8_t front_ = 0;
        volatile uint8_t back_ = 0;
        volatile uint8_t size_ = 0;

    };

    class CAN_Bus
    {
    public:
        enum struct RetCode {
            OK,
            NO_MSG,
            BUFFER_BUSY,
            SEND_TIMEOUT,
            INVALID_ARG
        };

        virtual ~CAN_Bus() = default;

        /**
         * Attempt to read a message from any Rx buffer
         * @param[inout] msg - message to read into
         * @return
         * OK - on read success
         * NO_MSG - no received messages
         */
        virtual RetCode readAny(CAN_Msg & msg) = 0;

        /**
         * Attempt to send a message using any available Tx buffer
         * @param[in] msg - the message to send
         * @return
         * OK - on success
         * BUFFER_BUSY - no tx buffers were available
         * SEND_TIMEOUT - failed to transmit the message in time
         */
        virtual RetCode sendAny(const CAN_Msg & msg, const bool waitForSend = 1u) = 0;
    };
}