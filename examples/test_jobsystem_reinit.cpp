/**
 * test_jobsystem_reinit.cpp
 *
 * Test if JobSystem can be shut down and re-initialized
 */

#include "JobSystem.h"
#include <iostream>

int main() {
    std::cout << "Creating JobSystem...\n";
    g_JobSystem = new JobSystem();

    std::cout << "\n=== First initialization ===\n";
    g_JobSystem->Initialize(4);
    std::cout << "Worker count: " << g_JobSystem->GetWorkerCount() << "\n";

    auto handle1 = g_JobSystem->Submit([]() {
        std::cout << "Job 1 executed!\n";
    });
    g_JobSystem->Wait(handle1);

    std::cout << "\n=== Shutting down ===\n";
    g_JobSystem->Shutdown();
    std::cout << "Is running: " << (g_JobSystem->IsRunning() ? "yes" : "no") << "\n";

    std::cout << "\n=== Second initialization ===\n";
    g_JobSystem->Initialize(4);
    std::cout << "Worker count: " << g_JobSystem->GetWorkerCount() << "\n";
    std::cout << "Is running: " << (g_JobSystem->IsRunning() ? "yes" : "no") << "\n";

    auto handle2 = g_JobSystem->Submit([]() {
        std::cout << "Job 2 executed!\n";
    });
    g_JobSystem->Wait(handle2);

    std::cout << "\n=== Final shutdown ===\n";
    g_JobSystem->Shutdown();
    delete g_JobSystem;
    g_JobSystem = nullptr;

    std::cout << "\nTest complete!\n";
    return 0;
}
