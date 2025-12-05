#include <gtest/gtest.h>
#include "VisualFeedback.h"
#include "VoxelWorld.h"
#include "StructuralAnalyzer.h"

#ifdef USE_BULLET
#include "BulletEngine.h"
#include "VoxelPhysicsIntegration.h"
#endif

// Week 7 Day 32: Visual Feedback System Tests

// ============================================================================
// VoxelVisualizer Tests
// ============================================================================

TEST(VoxelVisualizer, HighlightDamagedVoxels) {
    VoxelVisualizer visualizer;

    std::vector<Vector3> damaged;
    damaged.push_back(Vector3(1, 2, 3));
    damaged.push_back(Vector3(4, 5, 6));
    damaged.push_back(Vector3(7, 8, 9));

    visualizer.HighlightDamagedVoxels(damaged);

    const auto& highlighted = visualizer.GetHighlightedVoxels();
    EXPECT_EQ(highlighted.size(), 3);
    EXPECT_EQ(highlighted[0], Vector3(1, 2, 3));
    EXPECT_EQ(highlighted[1], Vector3(4, 5, 6));
    EXPECT_EQ(highlighted[2], Vector3(7, 8, 9));
}

TEST(VoxelVisualizer, ShowFailedClusters) {
    VoxelVisualizer visualizer;

    std::vector<VoxelCluster> clusters;
    VoxelCluster cluster1;
    cluster1.cluster_id = 1;
    cluster1.voxel_positions.push_back(Vector3(0, 0, 0));
    cluster1.voxel_positions.push_back(Vector3(0, 1, 0));
    cluster1.center_of_mass = Vector3(0, 0.5f, 0);
    clusters.push_back(cluster1);

    visualizer.ShowFailedClusters(clusters);

    const auto& failed = visualizer.GetFailedClusters();
    EXPECT_EQ(failed.size(), 1);
    EXPECT_EQ(failed[0].cluster_id, 1);
    EXPECT_EQ(failed[0].voxel_positions.size(), 2);
}

TEST(VoxelVisualizer, ClearVisualization) {
    VoxelVisualizer visualizer;

    std::vector<Vector3> damaged{Vector3(1, 2, 3)};
    visualizer.HighlightDamagedVoxels(damaged);

    EXPECT_EQ(visualizer.GetHighlightedVoxels().size(), 1);

    visualizer.Clear();

    EXPECT_EQ(visualizer.GetHighlightedVoxels().size(), 0);
    EXPECT_EQ(visualizer.GetFailedClusters().size(), 0);
}

TEST(VoxelVisualizer, AnimateCracking) {
    VoxelVisualizer visualizer;

    // Should not crash
    visualizer.AnimateCracking(Vector3(5, 5, 5), 0.8f);
    visualizer.AnimateCracking(Vector3(10, 10, 10), 0.2f);
}

// ============================================================================
// DestructionUI Tests
// ============================================================================

TEST(DestructionUI, ShowStructuralAnalysis) {
    DestructionUI ui;

    AnalysisResult result;
    result.structure_failed = true;
    result.calculation_time_ms = 12.5f;

    VoxelCluster cluster;
    cluster.cluster_id = 1;
    cluster.voxel_positions.push_back(Vector3(0, 0, 0));
    cluster.total_mass = 100.0f;
    result.failed_clusters.push_back(cluster);

    // Should not crash
    ui.ShowStructuralAnalysis(result);
}

TEST(DestructionUI, ShowWarning) {
    DestructionUI ui;

    // Should not crash
    ui.ShowWarning("Test warning message");
    ui.ShowWarning("Structure unstable!");
}

#ifdef USE_BULLET

TEST(DestructionUI, ShowPhysicsStats) {
    BulletEngine physics;
    physics.Initialize();

    VoxelWorld world;
    VoxelPhysicsIntegration integration(&physics, &world);

    DestructionUI ui;

    // Should not crash
    ui.ShowPhysicsStats(integration);

    physics.Shutdown();
}

TEST(DestructionUI, ShowDebrisBreakdown) {
    BulletEngine physics;
    physics.Initialize();

    VoxelWorld world;
    VoxelPhysicsIntegration integration(&physics, &world);

    DestructionUI ui;

    // Should not crash
    ui.ShowDebrisBreakdown(integration);

    physics.Shutdown();
}

#endif // USE_BULLET

// ============================================================================
// CameraEffects Tests
// ============================================================================

TEST(CameraEffects, InitialState) {
    CameraEffects camera;

    EXPECT_FALSE(camera.IsShaking());
    EXPECT_EQ(camera.GetShakeOffset(), Vector3(0, 0, 0));
}

TEST(CameraEffects, TriggerShake) {
    CameraEffects camera;

    camera.TriggerShake(0.5f, 2.0f);

    EXPECT_TRUE(camera.IsShaking());
}

TEST(CameraEffects, ShakeDecay) {
    CameraEffects camera;

    camera.TriggerShake(0.5f, 0.1f);

    EXPECT_TRUE(camera.IsShaking());

    // Update for longer than duration
    camera.Update(0.2f);

    EXPECT_FALSE(camera.IsShaking());
    EXPECT_EQ(camera.GetShakeOffset(), Vector3(0, 0, 0));
}

TEST(CameraEffects, ShakeUpdate) {
    CameraEffects camera;

    camera.TriggerShake(1.0f, 1.0f);

    Vector3 offset1 = camera.GetShakeOffset();

    camera.Update(0.016f);

    Vector3 offset2 = camera.GetShakeOffset();

    // Shake offset should change each frame (random)
    // Note: There's a tiny chance they could be equal by random chance,
    // but it's extremely unlikely
    EXPECT_NE(offset1, offset2);
}

TEST(CameraEffects, StopShake) {
    CameraEffects camera;

    camera.TriggerShake(1.0f, 5.0f);
    EXPECT_TRUE(camera.IsShaking());

    camera.Stop();
    EXPECT_FALSE(camera.IsShaking());
    EXPECT_EQ(camera.GetShakeOffset(), Vector3(0, 0, 0));
}

// ============================================================================
// TimeScale Tests
// ============================================================================

TEST(TimeScale, InitialState) {
    TimeScale time_scale;

    EXPECT_FLOAT_EQ(time_scale.GetScale(), 1.0f);
    EXPECT_FALSE(time_scale.IsSlowMotion());
}

TEST(TimeScale, SetSlowMotionInstant) {
    TimeScale time_scale;

    time_scale.SetSlowMotion(0.3f, 0.0f);

    EXPECT_FLOAT_EQ(time_scale.GetScale(), 0.3f);
    EXPECT_TRUE(time_scale.IsSlowMotion());
}

TEST(TimeScale, SetSlowMotionSmooth) {
    TimeScale time_scale;

    time_scale.SetSlowMotion(0.5f, 1.0f);

    // Should transition smoothly
    time_scale.Update(0.5f);

    float scale = time_scale.GetScale();
    EXPECT_LT(scale, 1.0f);
    EXPECT_GT(scale, 0.5f);
}

TEST(TimeScale, SetNormalSpeed) {
    TimeScale time_scale;

    time_scale.SetSlowMotion(0.3f, 0.0f);
    EXPECT_TRUE(time_scale.IsSlowMotion());

    time_scale.SetNormalSpeed(0.0f);
    EXPECT_FLOAT_EQ(time_scale.GetScale(), 1.0f);
    EXPECT_FALSE(time_scale.IsSlowMotion());
}

TEST(TimeScale, GetScaledDelta) {
    TimeScale time_scale;

    time_scale.SetSlowMotion(0.5f, 0.0f);

    float real_delta = 0.016f;
    float scaled_delta = time_scale.GetScaledDelta(real_delta);

    EXPECT_FLOAT_EQ(scaled_delta, 0.008f);
}

TEST(TimeScale, TransitionCompletion) {
    TimeScale time_scale;

    time_scale.SetSlowMotion(0.2f, 0.1f);

    // Update past transition duration
    time_scale.Update(0.2f);

    EXPECT_FLOAT_EQ(time_scale.GetScale(), 0.2f);
}

TEST(TimeScale, ClampingBounds) {
    TimeScale time_scale;

    // Test lower bound
    time_scale.SetSlowMotion(-0.5f, 0.0f);
    EXPECT_FLOAT_EQ(time_scale.GetScale(), 0.0f);

    // Test upper bound
    time_scale.SetSlowMotion(2.0f, 0.0f);
    EXPECT_FLOAT_EQ(time_scale.GetScale(), 1.0f);
}

// ============================================================================
// Integration Tests
// ============================================================================

#ifdef USE_BULLET

TEST(VisualFeedbackIntegration, CompleteWorkflow) {
    // Create systems
    BulletEngine physics;
    physics.Initialize();

    VoxelWorld world;
    VoxelPhysicsIntegration integration(&physics, &world);
    StructuralAnalyzer analyzer;

    VoxelVisualizer visualizer;
    DestructionUI ui;
    CameraEffects camera;
    TimeScale time_scale;

    // Create simple structure
    for (int y = 0; y < 5; y++) {
        world.SetVoxel(Vector3(0, y, 0), Voxel(1));
    }

    // Damage structure
    std::vector<Vector3> damaged;
    damaged.push_back(Vector3(0, 0, 0));
    world.RemoveVoxel(Vector3(0, 0, 0));

    // Visualize damage
    visualizer.HighlightDamagedVoxels(damaged);

    // Analyze
    auto result = analyzer.Analyze(world, damaged, 500.0f);

    // Show results
    ui.ShowStructuralAnalysis(result);

    if (result.structure_failed) {
        // Show failed clusters
        visualizer.ShowFailedClusters(result.failed_clusters);

        // Trigger effects
        camera.TriggerShake(0.5f, 2.0f);
        time_scale.SetSlowMotion(0.3f, 1.0f);

        // Spawn debris
        integration.SpawnDebris(result.failed_clusters);

        // Simulate with effects
        for (int i = 0; i < 60; i++) {
            float dt = time_scale.GetScaledDelta(1.0f / 60.0f);

            integration.Step(dt);
            camera.Update(dt);
            time_scale.Update(dt);
        }

        // Show physics stats
        ui.ShowPhysicsStats(integration);
    }

    physics.Shutdown();
}

#endif // USE_BULLET
