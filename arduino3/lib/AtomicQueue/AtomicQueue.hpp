#ifndef ATOMIC_QUEUE_HPP
#define ATOMIC_QUEUE_HPP

#include <Arduino.h>

template <typename T, uint16_t SIZE>
class AtomicQueue
{
    // Compile-time check: Size MUST be a power of 2 for the bitwise masking to work!
    static_assert((SIZE != 0) && ((SIZE & (SIZE - 1)) == 0), "Queue size must be a power of 2 (e.g., 16, 32, 64, 128)");

public:
    AtomicQueue() = default;

    bool push(const T &item)
    {
        // Save current interrupt state and globally disable interrupts
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        // Calculate next head position (bit-wise modulo trick, 1 CPU cycle)
        uint16_t nextHead = (_head + 1) & (SIZE - 1);

        // If full, advance the tail to "delete" the oldest item
        if (nextHead == _tail)
            _tail = (_tail + 1) & (SIZE - 1);

        // Write data and update head
        _buffer[_head] = item;
        _head          = nextHead;
        // Restore interrupt state
        __set_PRIMASK(primask);
        return true;
    }

    bool pop(T &item)
    {
        // Save current interrupt state and globally disable interrupts
        uint32_t primask = __get_PRIMASK();
        __disable_irq();

        // Check if queue is empty
        if (_head == _tail)
        {
            __set_PRIMASK(primask);
            return false;
        }

        // Read data and update tail
        item = _buffer[_tail];
        // Bit-wise modulo trick, 1 CPU cycle
        _tail = (_tail + 1) & (SIZE - 1);

        // Restore interrupt state
        __set_PRIMASK(primask);
        return true;
    }

    // Check if empty (safe to call without disabling IRQs)
    bool isEmpty() const
    {
        return _head == _tail;
    }

private:
    T _buffer[SIZE];
    volatile uint16_t _head = 0;
    volatile uint16_t _tail = 0;
};

#endif
