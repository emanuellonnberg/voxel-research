/**
 * ThreadSafeVector.h
 *
 * Week 13 Day 44: Thread-Safe Data Structures
 *
 * Thread-safe wrapper around std::vector using reader-writer locks.
 * Allows multiple concurrent readers or exclusive writers.
 *
 * Use cases:
 * - Debris/rubble lists updated by multiple physics threads
 * - Particle systems spawning from multiple sources
 * - Any shared vector accessed by multiple threads
 *
 * Performance notes:
 * - Read operations (snapshot, size) can run concurrently
 * - Write operations (push_back, remove, clear) are exclusive
 * - snapshot() returns a copy to avoid holding locks during iteration
 */

#pragma once

#include <vector>
#include <shared_mutex>
#include <algorithm>

template<typename T>
class ThreadSafeVector {
public:
    ThreadSafeVector() = default;
    ~ThreadSafeVector() = default;

    // Thread-safe operations

    /**
     * Add an item to the vector (exclusive write lock)
     */
    void push_back(const T& item) {
        std::unique_lock<std::shared_mutex> lock(mutex);
        data.push_back(item);
    }

    /**
     * Add an item to the vector (move semantics)
     */
    void push_back(T&& item) {
        std::unique_lock<std::shared_mutex> lock(mutex);
        data.push_back(std::move(item));
    }

    /**
     * Remove first occurrence of item (exclusive write lock)
     * Returns true if item was found and removed
     */
    bool remove(const T& item) {
        std::unique_lock<std::shared_mutex> lock(mutex);
        auto it = std::find(data.begin(), data.end(), item);
        if (it != data.end()) {
            data.erase(it);
            return true;
        }
        return false;
    }

    /**
     * Remove all items (exclusive write lock)
     */
    void clear() {
        std::unique_lock<std::shared_mutex> lock(mutex);
        data.clear();
    }

    /**
     * Get a snapshot copy of the vector (shared read lock)
     * Returns a copy to avoid holding locks during iteration
     */
    std::vector<T> snapshot() const {
        std::shared_lock<std::shared_mutex> lock(mutex);
        return data;  // Copy constructor
    }

    /**
     * Get current size (shared read lock)
     */
    size_t size() const {
        std::shared_lock<std::shared_mutex> lock(mutex);
        return data.size();
    }

    /**
     * Check if empty (shared read lock)
     */
    bool empty() const {
        std::shared_lock<std::shared_mutex> lock(mutex);
        return data.empty();
    }

    /**
     * Reserve capacity (exclusive write lock)
     */
    void reserve(size_t capacity) {
        std::unique_lock<std::shared_mutex> lock(mutex);
        data.reserve(capacity);
    }

    /**
     * Apply a function to each element (shared read lock)
     * Note: The function should not modify the elements
     */
    template<typename Func>
    void for_each(Func func) const {
        std::shared_lock<std::shared_mutex> lock(mutex);
        for (const auto& item : data) {
            func(item);
        }
    }

    /**
     * Apply a function to each element with potential modification (exclusive write lock)
     */
    template<typename Func>
    void for_each_mut(Func func) {
        std::unique_lock<std::shared_mutex> lock(mutex);
        for (auto& item : data) {
            func(item);
        }
    }

    // Deleted copy/move constructors (mutex cannot be copied)
    ThreadSafeVector(const ThreadSafeVector&) = delete;
    ThreadSafeVector& operator=(const ThreadSafeVector&) = delete;
    ThreadSafeVector(ThreadSafeVector&&) = delete;
    ThreadSafeVector& operator=(ThreadSafeVector&&) = delete;

private:
    std::vector<T> data;
    mutable std::shared_mutex mutex;
};
