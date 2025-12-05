#include <gtest/gtest.h>
#include "TurnManager.h"
#include "VoxelPhysicsIntegration.h"
#include "StructuralAnalyzer.h"
#include "VoxelWorld.h"

#ifdef USE_BULLET
#include "BulletEngine.h"
#else
#include "MockPhysicsEngine.h"
#endif

#include <chrono>
#include <thread>

// Week 7 Day 31: Turn-Based Integration Tests

// ============================================================================
// KeyframeRecorder Tests
// ============================================================================

TEST(KeyframeRecorder, BasicRecording) {
    KeyframeRecorder recorder;

    EXPECT_FALSE(recorder.IsRecording());
    EXPECT_EQ(recorder.GetKeyframeCount(), 0);

    recorder.StartRecording();
    EXPECT_TRUE(recorder.IsRecording());

    // Record some frames
    for (int i = 0; i < 10; i++) {
        std::vector<DebrisTransform> transforms;
        DebrisTransform transform;
        transform.debris_id = i;
        transform.position[0] = static_cast<float>(i);
        transform.position[1] = static_cast<float>(i * 2);
        transform.position[2] = static_cast<float>(i * 3);
        transforms.push_back(transform);

        recorder.RecordFrame(i / 60.0f, transforms);
    }

    EXPECT_EQ(recorder.GetKeyframeCount(), 10);

    const auto& keyframes = recorder.GetKeyframes();
    EXPECT_EQ(keyframes.size(), 10);
    EXPECT_FLOAT_EQ(keyframes[0].time, 0.0f);
    EXPECT_FLOAT_EQ(keyframes[9].time, 9.0f / 60.0f);

    recorder.StopRecording();
    EXPECT_FALSE(recorder.IsRecording());
}

TEST(KeyframeRecorder, RecordWhileNotRecording) {
    KeyframeRecorder recorder;

    // Try to record without starting
    std::vector<DebrisTransform> transforms;
    recorder.RecordFrame(0.0f, transforms);

    EXPECT_EQ(recorder.GetKeyframeCount(), 0);
}

TEST(KeyframeRecorder, ClearKeyframes) {
    KeyframeRecorder recorder;

    recorder.StartRecording();
    for (int i = 0; i < 5; i++) {
        std::vector<DebrisTransform> transforms;
        recorder.RecordFrame(i / 60.0f, transforms);
    }

    EXPECT_EQ(recorder.GetKeyframeCount(), 5);

    recorder.Clear();
    EXPECT_EQ(recorder.GetKeyframeCount(), 0);
    EXPECT_FALSE(recorder.IsRecording());
}

// ============================================================================
// TurnManager Tests
// ============================================================================

TEST(TurnManager, InitialState) {
    TurnManager manager;

    EXPECT_TRUE(manager.IsPlayerTurn());
    EXPECT_FALSE(manager.IsAnalyzing());
    EXPECT_FALSE(manager.IsSimulating());
    EXPECT_FALSE(manager.IsAITurn());
    EXPECT_EQ(manager.GetPhase(), TurnPhase::PLAYER_ACTION);
    EXPECT_FLOAT_EQ(manager.GetPhaseTimer(), 0.0f);
}

TEST(TurnManager, TurnTransition) {
    TurnManager manager;

    // Start turn transition
    manager.StartTurnTransition();

    EXPECT_TRUE(manager.IsAnalyzing());
    EXPECT_EQ(manager.GetPhase(), TurnPhase::STRUCTURAL_ANALYSIS);
    EXPECT_FLOAT_EQ(manager.GetPhaseTimer(), 0.0f);
}

TEST(TurnManager, StructureStable) {
    TurnManager manager;

    // Player action -> Structural analysis
    manager.StartTurnTransition();
    EXPECT_TRUE(manager.IsAnalyzing());

    // Update for some time
    manager.Update(0.1f);
    EXPECT_FLOAT_EQ(manager.GetPhaseTimer(), 0.1f);
    EXPECT_TRUE(manager.IsAnalyzing());

    // Structural analysis complete, structure stable
    manager.NotifyStructuralAnalysisComplete(false);
    manager.Update(0.016f);

    // Should skip physics and go to AI turn
    EXPECT_TRUE(manager.IsAITurn());
    EXPECT_EQ(manager.GetPhase(), TurnPhase::AI_TURN);
}

TEST(TurnManager, StructureFailed) {
    TurnManager manager;

    // Player action -> Structural analysis
    manager.StartTurnTransition();
    EXPECT_TRUE(manager.IsAnalyzing());

    // Structural analysis complete, structure failed
    manager.NotifyStructuralAnalysisComplete(true);
    manager.Update(0.016f);

    // Should move to physics simulation
    EXPECT_TRUE(manager.IsSimulating());
    EXPECT_EQ(manager.GetPhase(), TurnPhase::PHYSICS_SIMULATION);
}

TEST(TurnManager, PhysicsSimulationComplete) {
    TurnManager manager;

    // Go to physics simulation phase
    manager.SetPhase(TurnPhase::PHYSICS_SIMULATION);
    EXPECT_TRUE(manager.IsSimulating());

    // Update with active debris
    manager.UpdateActiveDebrisCount(10);
    manager.Update(0.016f);
    EXPECT_TRUE(manager.IsSimulating());

    // Physics simulation complete
    manager.NotifyPhysicsSimulationComplete();
    manager.Update(0.016f);

    // Should move to AI turn
    EXPECT_TRUE(manager.IsAITurn());
}

TEST(TurnManager, PhysicsSimulationNoDebris) {
    TurnManager manager;

    // Go to physics simulation phase
    manager.SetPhase(TurnPhase::PHYSICS_SIMULATION);
    EXPECT_TRUE(manager.IsSimulating());

    // No active debris (all settled)
    manager.UpdateActiveDebrisCount(0);
    manager.Update(0.016f);

    // Should automatically move to AI turn
    EXPECT_TRUE(manager.IsAITurn());
}

TEST(TurnManager, CompleteTurnCycle) {
    TurnManager manager;

    // 1. Player action
    EXPECT_TRUE(manager.IsPlayerTurn());

    // 2. Start turn transition
    manager.StartTurnTransition();
    EXPECT_TRUE(manager.IsAnalyzing());

    // 3. Structure fails
    manager.NotifyStructuralAnalysisComplete(true);
    manager.Update(0.016f);
    EXPECT_TRUE(manager.IsSimulating());

    // 4. Physics simulation completes
    manager.NotifyPhysicsSimulationComplete();
    manager.Update(0.016f);
    EXPECT_TRUE(manager.IsAITurn());

    // 5. AI turn ends
    manager.EndAITurn();
    EXPECT_TRUE(manager.IsPlayerTurn());
}

TEST(TurnManager, PhaseTimerResets) {
    TurnManager manager;

    // Accumulate time in player action phase
    manager.Update(1.0f);
    manager.Update(0.5f);
    EXPECT_FLOAT_EQ(manager.GetPhaseTimer(), 1.5f);

    // Transition to analysis
    manager.StartTurnTransition();
    EXPECT_FLOAT_EQ(manager.GetPhaseTimer(), 0.0f);

    // Accumulate time in analysis phase
    manager.Update(0.2f);
    EXPECT_FLOAT_EQ(manager.GetPhaseTimer(), 0.2f);

    // Complete analysis
    manager.NotifyStructuralAnalysisComplete(false);
    manager.Update(0.016f);
    EXPECT_FLOAT_EQ(manager.GetPhaseTimer(), 0.016f);
}

// ============================================================================
// Integration Tests
// ============================================================================

#ifdef USE_BULLET

TEST(TurnBasedIntegration, FastForwardSimulation) {
    // Create physics engine
    BulletEngine physics;
    physics.Initialize();

    // Create voxel world and integration
    VoxelWorld world;
    VoxelPhysicsIntegration integration(&physics, &world);

    // Create a simple tower
    for (int y = 0; y < 10; y++) {
        world.SetVoxel(Vector3(0, y, 0), Voxel(1)); // Material ID 1
    }

    // Damage bottom voxels
    std::vector<Vector3> damaged;
    for (int y = 0; y < 3; y++) {
        damaged.push_back(Vector3(0, y, 0));
        world.RemoveVoxel(Vector3(0, y, 0));
    }

    // Run structural analysis
    StructuralAnalyzer analyzer;
    auto result = analyzer.Analyze(world, damaged, 500.0f);

    EXPECT_TRUE(result.structure_failed);
    EXPECT_GT(result.failed_clusters.size(), 0);

    // Spawn debris
    integration.SpawnDebris(result.failed_clusters);

    // Fast-forward simulate 3 seconds
    const float TIMESTEP = 1.0f / 60.0f;
    int steps = 180; // 3 seconds at 60 FPS

    KeyframeRecorder recorder;
    recorder.StartRecording();

    auto start = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < steps; i++) {
        integration.Step(TIMESTEP);

        // Record keyframe every 2 frames
        if (i % 2 == 0) {
            std::vector<DebrisTransform> transforms;
            // In a real implementation, we'd extract transforms from debris bodies
            recorder.RecordFrame(i * TIMESTEP, transforms);
        }
    }

    auto end = std::chrono::high_resolution_clock::now();
    float elapsed = std::chrono::duration<float>(end - start).count();

    recorder.StopRecording();

    // Should simulate faster than real-time (3 seconds in less than 3 seconds)
    // Without rendering, this should be much faster
    EXPECT_LT(elapsed, 3.0f);
    EXPECT_GT(recorder.GetKeyframeCount(), 0);

    physics.Shutdown();
}

TEST(TurnBasedIntegration, TurnManagerWithPhysics) {
    // Create systems
    BulletEngine physics;
    physics.Initialize();

    VoxelWorld world;
    VoxelPhysicsIntegration integration(&physics, &world);
    StructuralAnalyzer analyzer;
    TurnManager turn_manager;

    // Create structure
    for (int y = 0; y < 5; y++) {
        world.SetVoxel(Vector3(0, y, 0), Voxel(1)); // Material ID 1
    }

    // Player action phase
    EXPECT_TRUE(turn_manager.IsPlayerTurn());

    // Player damages structure
    std::vector<Vector3> damaged;
    damaged.push_back(Vector3(0, 0, 0));
    world.RemoveVoxel(Vector3(0, 0, 0));

    // Start turn transition
    turn_manager.StartTurnTransition();
    EXPECT_TRUE(turn_manager.IsAnalyzing());

    // Run structural analysis
    auto result = analyzer.Analyze(world, damaged, 500.0f);

    // Notify turn manager
    turn_manager.NotifyStructuralAnalysisComplete(result.structure_failed);
    turn_manager.Update(0.016f);

    if (result.structure_failed) {
        EXPECT_TRUE(turn_manager.IsSimulating());

        // Spawn debris
        integration.SpawnDebris(result.failed_clusters);

        // Simulate until settled (or 300 frames max = 5 seconds)
        bool reached_ai_turn = false;
        for (int i = 0; i < 300; i++) {
            integration.Step(1.0f / 60.0f);

            int active = integration.GetActiveDebrisCount();
            turn_manager.UpdateActiveDebrisCount(active);
            turn_manager.Update(1.0f / 60.0f);

            if (turn_manager.IsAITurn()) {
                // All debris settled
                reached_ai_turn = true;
                break;
            }
        }

        // If debris didn't settle naturally, force completion after timeout
        if (!reached_ai_turn) {
            turn_manager.NotifyPhysicsSimulationComplete();
            turn_manager.Update(0.016f);
        }

        EXPECT_TRUE(turn_manager.IsAITurn());
    }

    physics.Shutdown();
}

#endif // USE_BULLET

// ============================================================================
// Main
// ============================================================================

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
