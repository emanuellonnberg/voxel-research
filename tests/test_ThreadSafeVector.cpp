#include <gtest/gtest.h>
#include "ThreadSafeVector.h"
#include <thread>
#include <atomic>
#include <chrono>

// Basic functionality tests
TEST(ThreadSafeVectorTest, PushBack) {
    ThreadSafeVector<int> vec;
    vec.push_back(1);
    vec.push_back(2);
    vec.push_back(3);

    EXPECT_EQ(vec.size(), 3);

    auto snapshot = vec.snapshot();
    EXPECT_EQ(snapshot.size(), 3);
    EXPECT_EQ(snapshot[0], 1);
    EXPECT_EQ(snapshot[1], 2);
    EXPECT_EQ(snapshot[2], 3);
}

TEST(ThreadSafeVectorTest, Remove) {
    ThreadSafeVector<int> vec;
    vec.push_back(1);
    vec.push_back(2);
    vec.push_back(3);

    EXPECT_TRUE(vec.remove(2));
    EXPECT_EQ(vec.size(), 2);

    auto snapshot = vec.snapshot();
    EXPECT_EQ(snapshot[0], 1);
    EXPECT_EQ(snapshot[1], 3);

    EXPECT_FALSE(vec.remove(99));  // Not found
}

TEST(ThreadSafeVectorTest, Clear) {
    ThreadSafeVector<int> vec;
    vec.push_back(1);
    vec.push_back(2);
    vec.push_back(3);

    EXPECT_FALSE(vec.empty());
    vec.clear();
    EXPECT_TRUE(vec.empty());
    EXPECT_EQ(vec.size(), 0);
}

TEST(ThreadSafeVectorTest, ForEach) {
    ThreadSafeVector<int> vec;
    vec.push_back(1);
    vec.push_back(2);
    vec.push_back(3);

    int sum = 0;
    vec.for_each([&](int val) {
        sum += val;
    });

    EXPECT_EQ(sum, 6);
}

TEST(ThreadSafeVectorTest, ForEachMut) {
    ThreadSafeVector<int> vec;
    vec.push_back(1);
    vec.push_back(2);
    vec.push_back(3);

    vec.for_each_mut([](int& val) {
        val *= 2;
    });

    auto snapshot = vec.snapshot();
    EXPECT_EQ(snapshot[0], 2);
    EXPECT_EQ(snapshot[1], 4);
    EXPECT_EQ(snapshot[2], 6);
}

// Thread safety tests
TEST(ThreadSafeVectorTest, ConcurrentPushBack) {
    ThreadSafeVector<int> vec;
    const int num_threads = 4;
    const int items_per_thread = 1000;

    std::vector<std::thread> threads;

    // Spawn multiple threads pushing items
    for (int t = 0; t < num_threads; t++) {
        threads.emplace_back([&vec, t, items_per_thread]() {
            for (int i = 0; i < items_per_thread; i++) {
                vec.push_back(t * items_per_thread + i);
            }
        });
    }

    // Wait for all threads
    for (auto& thread : threads) {
        thread.join();
    }

    // Verify total count
    EXPECT_EQ(vec.size(), num_threads * items_per_thread);
}

TEST(ThreadSafeVectorTest, ConcurrentReads) {
    ThreadSafeVector<int> vec;

    // Populate vector
    for (int i = 0; i < 1000; i++) {
        vec.push_back(i);
    }

    std::atomic<int> read_count(0);
    const int num_readers = 8;
    std::vector<std::thread> readers;

    // Spawn multiple reader threads
    for (int t = 0; t < num_readers; t++) {
        readers.emplace_back([&vec, &read_count]() {
            for (int i = 0; i < 100; i++) {
                auto snapshot = vec.snapshot();
                if (snapshot.size() == 1000) {
                    read_count++;
                }
            }
        });
    }

    // Wait for all readers
    for (auto& reader : readers) {
        reader.join();
    }

    // All reads should succeed
    EXPECT_EQ(read_count.load(), num_readers * 100);
}

TEST(ThreadSafeVectorTest, ConcurrentReadWrite) {
    ThreadSafeVector<int> vec;

    std::atomic<bool> stop_flag(false);
    std::atomic<int> write_count(0);
    std::atomic<int> read_count(0);

    // Writer thread
    std::thread writer([&]() {
        for (int i = 0; i < 500; i++) {
            vec.push_back(i);
            write_count++;
            std::this_thread::sleep_for(std::chrono::microseconds(10));
        }
        stop_flag = true;
    });

    // Reader threads
    std::vector<std::thread> readers;
    for (int t = 0; t < 4; t++) {
        readers.emplace_back([&]() {
            while (!stop_flag.load()) {
                auto snapshot = vec.snapshot();
                read_count++;
                std::this_thread::sleep_for(std::chrono::microseconds(50));
            }
        });
    }

    // Wait for completion
    writer.join();
    for (auto& reader : readers) {
        reader.join();
    }

    // Verify final state
    EXPECT_EQ(vec.size(), 500);
    EXPECT_EQ(write_count.load(), 500);
    EXPECT_GT(read_count.load(), 0);  // Some reads should have happened
}

// Performance test
TEST(ThreadSafeVectorTest, Performance) {
    ThreadSafeVector<int> vec;
    const int iterations = 10000;

    auto start = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < iterations; i++) {
        vec.push_back(i);
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

    // Should be reasonably fast (< 100 microseconds per operation)
    EXPECT_LT(duration.count(), iterations * 100);
}
