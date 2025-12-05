#pragma once

#include "Vector3.h"
#include "VoxelCluster.h"
#include "StructuralAnalyzer.h"
#include "VoxelPhysicsIntegration.h"
#include "Material.h"
#include <vector>
#include <string>
#include <functional>

// Week 7 Day 32: Visual Feedback Systems

/**
 * @brief Visualizes voxel damage and structural failures
 *
 * Provides visual feedback for:
 * - Damaged voxel highlighting
 * - Failed cluster visualization
 * - Debug rendering (center of mass, bounding boxes)
 *
 * Currently uses console output. Ready for OpenGL integration.
 */
class VoxelVisualizer {
private:
    std::vector<Vector3> highlighted_voxels;
    std::vector<VoxelCluster> failed_clusters;
    bool show_highlights;
    bool show_clusters;

public:
    VoxelVisualizer();

    /**
     * @brief Highlight damaged voxels (for visual feedback)
     * @param positions Positions of damaged voxels
     */
    void HighlightDamagedVoxels(const std::vector<Vector3>& positions);

    /**
     * @brief Show failed clusters with visual effects
     * @param clusters Clusters that failed structural analysis
     */
    void ShowFailedClusters(const std::vector<VoxelCluster>& clusters);

    /**
     * @brief Animate cracking effect at position
     * @param position Crack position
     * @param intensity Crack intensity (0.0-1.0)
     */
    void AnimateCracking(const Vector3& position, float intensity);

    /**
     * @brief Clear all visualizations
     */
    void Clear();

    /**
     * @brief Get highlighted voxels (for rendering)
     */
    const std::vector<Vector3>& GetHighlightedVoxels() const { return highlighted_voxels; }

    /**
     * @brief Get failed clusters (for rendering)
     */
    const std::vector<VoxelCluster>& GetFailedClusters() const { return failed_clusters; }

    /**
     * @brief Print visualization to console
     */
    void PrintStatus() const;
};

/**
 * @brief Console-based UI for destruction feedback
 *
 * Shows structural analysis results, physics stats, and warnings.
 * Currently uses console output. Ready for ImGui integration.
 */
class DestructionUI {
public:
    DestructionUI() = default;

    /**
     * @brief Show structural analysis results
     * @param result Analysis result from StructuralAnalyzer
     */
    void ShowStructuralAnalysis(const AnalysisResult& result);

    /**
     * @brief Show physics simulation statistics
     * @param integration VoxelPhysicsIntegration instance
     */
    void ShowPhysicsStats(const VoxelPhysicsIntegration& integration);

    /**
     * @brief Show warning message
     * @param message Warning text
     */
    void ShowWarning(const std::string& message);

    /**
     * @brief Show debris state breakdown
     * @param integration VoxelPhysicsIntegration instance
     */
    void ShowDebrisBreakdown(const VoxelPhysicsIntegration& integration);
};

/**
 * @brief Camera shake effects for impacts
 *
 * Provides procedural camera shake with intensity and duration control.
 * Shake fades out over time for smooth transitions.
 */
class CameraEffects {
private:
    Vector3 shake_offset;
    float shake_intensity;
    float shake_duration;
    float shake_timer;

public:
    CameraEffects();

    /**
     * @brief Trigger camera shake effect
     * @param intensity Shake intensity (0.0-1.0, typical: 0.5)
     * @param duration Shake duration in seconds
     */
    void TriggerShake(float intensity, float duration);

    /**
     * @brief Update camera shake (call each frame)
     * @param deltaTime Time step in seconds
     */
    void Update(float deltaTime);

    /**
     * @brief Get current shake offset to apply to camera
     */
    Vector3 GetShakeOffset() const { return shake_offset; }

    /**
     * @brief Check if shake is active
     */
    bool IsShaking() const { return shake_timer > 0.0f; }

    /**
     * @brief Stop shake immediately
     */
    void Stop();
};

/**
 * @brief Time scale controller for slow-motion effects
 *
 * Provides smooth time scaling for dramatic slow-motion during destruction.
 * Supports smooth transitions between normal and slow-motion speeds.
 */
class TimeScale {
private:
    float current_scale;
    float target_scale;
    float transition_speed;

public:
    TimeScale();

    /**
     * @brief Set slow-motion effect
     * @param scale Time scale factor (0.0-1.0, e.g., 0.3 = 30% speed)
     * @param duration Duration in seconds (0 = instant)
     */
    void SetSlowMotion(float scale, float duration = 0.0f);

    /**
     * @brief Return to normal speed
     * @param duration Transition duration in seconds
     */
    void SetNormalSpeed(float duration = 0.5f);

    /**
     * @brief Update time scale (call each frame)
     * @param deltaTime Real-time delta in seconds
     */
    void Update(float deltaTime);

    /**
     * @brief Get current time scale
     * @return Scale factor (0.0-1.0)
     */
    float GetScale() const { return current_scale; }

    /**
     * @brief Get scaled delta time
     * @param deltaTime Real-time delta
     * @return Scaled delta time
     */
    float GetScaledDelta(float deltaTime) const { return deltaTime * current_scale; }

    /**
     * @brief Check if in slow-motion
     */
    bool IsSlowMotion() const { return current_scale < 1.0f; }
};
