/**
 * test_jobsystem_simple.cpp
 *
 * Simple test to verify JobSystem works correctly
 */

#include "JobSystem.h"
#include <iostream>

int main() {
    std::cout << "Testing JobSystem initialization...\n";

    // Test 1: Create and initialize
    std::cout << "Creating JobSystem...\n";
    g_JobSystem = new JobSystem();

    std::cout << "Initializing with 4 threads...\n";
    g_JobSystem->Initialize(4);

    std::cout << "Worker count: " << g_JobSystem->GetWorkerCount() << "\n";
    std::cout << "Is running: " << (g_JobSystem->IsRunning() ? "yes" : "no") << "\n";

    // Test 2: Submit a simple job
    std::cout << "Submitting test job...\n";
    auto handle = g_JobSystem->Submit([]() {
        std::cout << "Job executed!\n";
    });

    g_JobSystem->Wait(handle);
    std::cout << "Job completed.\n";

    // Test 3: ParallelFor
    std::cout << "Testing ParallelFor...\n";
    int sum = 0;
    g_JobSystem->ParallelFor(10, [&](int i) {
        sum += i;
    });
    std::cout << "ParallelFor completed. Sum = " << sum << "\n";

    // Shutdown
    std::cout << "Shutting down...\n";
    g_JobSystem->Shutdown();
    delete g_JobSystem;
    g_JobSystem = nullptr;

    std::cout << "Test complete!\n";
    return 0;
}
