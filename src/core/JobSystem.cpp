/**
 * JobSystem Implementation
 * Week 13 Day 41: Multi-Threading Foundation
 */

#include "JobSystem.h"
#include <iostream>
#include <algorithm>

// Global instance
JobSystem* g_JobSystem = nullptr;

JobSystem::JobSystem()
    : initialized(false), shutdown_flag(false) {
}

JobSystem::~JobSystem() {
    if (IsRunning()) {
        Shutdown();
    }
}

void JobSystem::Initialize(int num_threads) {
    if (IsRunning()) {
        std::cerr << "JobSystem: Already initialized\n";
        return;
    }

    // Auto-detect core count if requested
    if (num_threads <= 0) {
        num_threads = std::thread::hardware_concurrency();
        if (num_threads == 0) {
            num_threads = 4;  // Fallback if detection fails
        }
    }

    std::cout << "JobSystem: Starting " << num_threads << " worker threads\n";

    shutdown_flag = false;
    initialized = false;  // Will be set to true after threads are created

    // Create worker threads
    for (int i = 0; i < num_threads; i++) {
        workers.emplace_back([this]() {
            WorkerThread();
        });
    }

    initialized = true;  // Now fully initialized
}

void JobSystem::Shutdown() {
    if (!IsRunning()) {
        return;
    }

    std::cout << "JobSystem: Shutting down...\n";

    // Signal shutdown
    shutdown_flag = true;

    // Wake up all threads
    queue_cv.notify_all();

    // Wait for all workers to finish
    for (auto& worker : workers) {
        if (worker.joinable()) {
            worker.join();
        }
    }

    workers.clear();

    // Clear any remaining jobs
    {
        std::lock_guard<std::mutex> lock(queue_mutex);
        while (!job_queue.empty()) {
            Job* job = job_queue.front();
            job_queue.pop();
            delete job;
        }
    }

    initialized = false;  // No longer initialized

    std::cout << "JobSystem: Shutdown complete\n";
}

JobHandle JobSystem::Submit(std::function<void()> work) {
    if (!IsRunning()) {
        std::cerr << "JobSystem: Cannot submit job - system not initialized\n";
        return JobHandle();
    }

    // Create job
    Job* job = new Job();
    job->work = work;
    job->unfinished_jobs = 1;
    job->parent = nullptr;

    // Add to queue
    {
        std::lock_guard<std::mutex> lock(queue_mutex);
        job_queue.push(job);
    }

    // Wake up a worker
    queue_cv.notify_one();

    return JobHandle(job);
}

void JobSystem::Wait(JobHandle handle) {
    if (!handle.IsValid()) {
        return;
    }

    // Busy-wait for job completion
    // Could be optimized with condition variable, but simple busy-wait is often faster
    while (handle.job->unfinished_jobs.load() > 0) {
        // Help with work while waiting
        Job* job = GetNextJob();
        if (job) {
            job->work();
            FinishJob(job);
        } else {
            // No work available, yield
            std::this_thread::yield();
        }
    }

    // Job is complete, clean up
    delete handle.job;
}

void JobSystem::ParallelFor(int count, std::function<void(int)> work) {
    if (count <= 0) {
        return;
    }

    if (!IsRunning()) {
        // Fallback to serial execution if job system not initialized
        for (int i = 0; i < count; i++) {
            work(i);
        }
        return;
    }

    const int num_workers = GetWorkerCount();
    const int batch_size = std::max(1, count / (num_workers * 4));

    // Shared atomic counter for work distribution
    std::atomic<int> next_index(0);

    // Create batch work function
    auto batch_work = [&]() {
        while (true) {
            int start = next_index.fetch_add(batch_size);
            if (start >= count) break;

            int end = std::min(start + batch_size, count);
            for (int i = start; i < end; i++) {
                work(i);
            }
        }
    };

    // Submit jobs (one per worker thread)
    std::vector<JobHandle> handles;
    handles.reserve(num_workers);

    for (int i = 0; i < num_workers; i++) {
        handles.push_back(Submit(batch_work));
    }

    // Wait for all to complete
    for (auto& handle : handles) {
        Wait(handle);
    }
}

void JobSystem::WorkerThread() {
    while (!shutdown_flag.load()) {
        Job* job = nullptr;

        // Get job from queue
        {
            std::unique_lock<std::mutex> lock(queue_mutex);

            // Wait for work or shutdown
            queue_cv.wait(lock, [this]() {
                return !job_queue.empty() || shutdown_flag.load();
            });

            if (shutdown_flag.load()) {
                break;
            }

            if (!job_queue.empty()) {
                job = job_queue.front();
                job_queue.pop();
            }
        }

        // Execute job
        if (job) {
            job->work();
            FinishJob(job);
        }
    }
}

void JobSystem::FinishJob(Job* job) {
    // Decrement counter
    const int unfinished = job->unfinished_jobs.fetch_sub(1) - 1;

    if (unfinished == 0 && job->parent) {
        // This job and all children done, notify parent
        FinishJob(job->parent);
    }
}

Job* JobSystem::GetNextJob() {
    std::lock_guard<std::mutex> lock(queue_mutex);

    if (job_queue.empty()) {
        return nullptr;
    }

    Job* job = job_queue.front();
    job_queue.pop();
    return job;
}
