/**
 * JobSystem: Work-Stealing Thread Pool
 *
 * Week 13 Day 41: Multi-Threading Foundation
 *
 * Manages a pool of worker threads that execute jobs in parallel.
 * Jobs can spawn child jobs and wait for completion.
 *
 * Features:
 * - Automatic core detection
 * - Work-stealing for load balancing
 * - Parent-child job dependencies
 * - Efficient ParallelFor implementation
 *
 * Usage:
 *   JobSystem jobs;
 *   jobs.Initialize();
 *
 *   // Submit individual job
 *   auto handle = jobs.Submit([]() { DoWork(); });
 *   jobs.Wait(handle);
 *
 *   // Parallel for loop
 *   jobs.ParallelFor(10000, [](int i) { ProcessItem(i); });
 *
 *   jobs.Shutdown();
 */

#pragma once

#include <functional>
#include <thread>
#include <vector>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <memory>

/**
 * Job: Unit of work to be executed
 */
struct Job {
    std::function<void()> work;
    std::atomic<int> unfinished_jobs;  // This job + children
    Job* parent;

    Job() : unfinished_jobs(1), parent(nullptr) {}
};

/**
 * JobHandle: Reference to a submitted job
 * Used for waiting on job completion
 */
struct JobHandle {
    Job* job;

    JobHandle() : job(nullptr) {}
    JobHandle(Job* j) : job(j) {}

    bool IsValid() const { return job != nullptr; }
};

/**
 * JobSystem: Work-stealing thread pool
 */
class JobSystem {
public:
    JobSystem();
    ~JobSystem();

    /**
     * Initialize job system
     * @param num_threads Number of worker threads (-1 = auto-detect)
     */
    void Initialize(int num_threads = -1);

    /**
     * Shutdown job system and wait for all work to complete
     */
    void Shutdown();

    /**
     * Submit a job for execution
     * @param work Function to execute
     * @return JobHandle for waiting on completion
     */
    JobHandle Submit(std::function<void()> work);

    /**
     * Wait for job completion
     * @param handle Job to wait for
     */
    void Wait(JobHandle handle);

    /**
     * Parallel for loop
     * Splits work across multiple threads
     * @param count Number of iterations
     * @param work Function to execute for each index
     */
    void ParallelFor(int count, std::function<void(int)> work);

    /**
     * Get number of worker threads
     */
    int GetWorkerCount() const { return static_cast<int>(workers.size()); }

    /**
     * Check if job system is running
     */
    bool IsRunning() const { return initialized.load() && !shutdown_flag.load(); }

private:
    // Worker thread main loop
    void WorkerThread();

    // Finish a job and notify parent
    void FinishJob(Job* job);

    // Get next job from queue
    Job* GetNextJob();

    // Worker threads
    std::vector<std::thread> workers;

    // Job queue
    std::queue<Job*> job_queue;
    std::mutex queue_mutex;
    std::condition_variable queue_cv;

    // State flags
    std::atomic<bool> initialized;
    std::atomic<bool> shutdown_flag;

    // Prevent copying
    JobSystem(const JobSystem&) = delete;
    JobSystem& operator=(const JobSystem&) = delete;
};

// Global job system instance
extern JobSystem* g_JobSystem;

// Helper macros for convenient usage
#define SUBMIT_JOB(work) g_JobSystem->Submit(work)
#define WAIT_JOB(handle) g_JobSystem->Wait(handle)
#define PARALLEL_FOR(count, work) g_JobSystem->ParallelFor(count, work)
