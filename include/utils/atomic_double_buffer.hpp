#pragma once
#include <atomic>
#include <utility>

/**
 * @brief A lightweight, lock-free double buffer for real-time data sharing between threads.
 *
 * This class maintains two copies (buffers) of an object of type T.
 * One buffer is always "active" and visible to readers, while the other
 * is used by the writer to update data. After each write, the two buffers
 * are atomically swapped to make the new data visible.
 *
 * Typical use case:
 * - A high-frequency control loop (e.g. 1 kHz) writes data.
 * - A slower monitoring thread (e.g. 100 Hz) reads snapshots.
 *
 * @tparam T The data type to store (must be copyable or movable).
 */
template <typename T>
class AtomicDoubleBuffer {
public:
    explicit AtomicDoubleBuffer(const T& init_value)
        : buffer1_(init_value),
          buffer2_(init_value),
          front_buffer_(&buffer1_),
          back_buffer_(&buffer2_),
          active_(&buffer1_) {}

    // Non-copyable
    AtomicDoubleBuffer(const AtomicDoubleBuffer&) = delete;
    AtomicDoubleBuffer& operator=(const AtomicDoubleBuffer&) = delete;

    void write(const T& value) noexcept {
        *back_buffer_ = value;
        active_.store(back_buffer_, std::memory_order_release);
        swapBuffers();
    }

    void write(T&& value) noexcept {
        *back_buffer_ = std::move(value);
        active_.store(back_buffer_, std::memory_order_release);
        swapBuffers();
    }

    T read() const noexcept {
        return *active_.load(std::memory_order_acquire);
    }

    const T* readPtr() const noexcept {
        return active_.load(std::memory_order_acquire);
    }

private:
    void swapBuffers() noexcept {
        back_buffer_ = (back_buffer_ == &buffer1_) ? &buffer2_ : &buffer1_;
    }

    mutable T buffer1_;
    mutable T buffer2_;
    T* front_buffer_;
    T* back_buffer_;
    std::atomic<T*> active_;
};

