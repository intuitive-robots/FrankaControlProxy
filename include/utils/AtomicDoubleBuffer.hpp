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
    AtomicDoubleBuffer() {
        front_buffer_ = &buffer1_;
        back_buffer_  = &buffer2_;
        active_.store(front_buffer_, std::memory_order_release);
    }

    // Non-copyable and non-assignable
    AtomicDoubleBuffer(const AtomicDoubleBuffer&) = delete;
    AtomicDoubleBuffer& operator=(const AtomicDoubleBuffer&) = delete;

    /**
     * @brief Write a new value into the buffer.
     * 
     * This replaces the contents of the inactive buffer, then atomically swaps
     * the active pointer so that readers can see the new data immediately.
     *
     * Thread-safe and lock-free.
     *
     * @param value The new data to write.
     */
    void write(const T& value) noexcept {
        *back_buffer_ = value;
        active_.store(back_buffer_, std::memory_order_release);
        swapBuffers();
    }

    /**
     * @brief Write using move semantics (if T supports move).
     */
    void write(T&& value) noexcept {
        *back_buffer_ = std::move(value);
        active_.store(back_buffer_, std::memory_order_release);
        swapBuffers();
    }

    /**
     * @brief Get a copy of the most recent data snapshot.
     *
     * @return A copy of the currently active data.
     */
    T read() const noexcept {
        return *active_.load(std::memory_order_acquire);
    }

    /**
     * @brief Get a pointer to the currently active buffer (read-only).
     *
     * @return const T* Pointer to the latest data.
     */
    const T* readPtr() const noexcept {
        return active_.load(std::memory_order_acquire);
    }

private:
    /**
     * @brief Swap the back and front buffer pointers.
     */
    void swapBuffers() noexcept {
        back_buffer_ = (back_buffer_ == &buffer1_) ? &buffer2_ : &buffer1_;
    }

    // Two buffers to alternate between writes and reads
    mutable T buffer1_;
    mutable T buffer2_;

    // Pointers to current front/back buffers
    T* front_buffer_;
    T* back_buffer_;

    // Atomic pointer to the currently active buffer
    std::atomic<T*> active_;
};
