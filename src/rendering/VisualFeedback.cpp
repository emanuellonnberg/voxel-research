#include "VisualFeedback.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <cstdlib>

// Week 7 Day 32: Visual Feedback Systems Implementation

// ============================================================================
// VoxelVisualizer Implementation
// ============================================================================

VoxelVisualizer::VoxelVisualizer()
    : show_highlights(false), show_clusters(false) {
}

void VoxelVisualizer::HighlightDamagedVoxels(const std::vector<Vector3>& positions) {
    highlighted_voxels = positions;
    show_highlights = !positions.empty();

    if (show_highlights) {
        std::cout << "[VoxelVisualizer] Highlighting " << positions.size()
                  << " damaged voxels" << std::endl;
    }
}

void VoxelVisualizer::ShowFailedClusters(const std::vector<VoxelCluster>& clusters) {
    failed_clusters = clusters;
    show_clusters = !clusters.empty();

    if (show_clusters) {
        std::cout << "[VoxelVisualizer] Showing " << clusters.size()
                  << " failed clusters:" << std::endl;

        for (size_t i = 0; i < clusters.size(); i++) {
            const auto& cluster = clusters[i];
            std::cout << "  Cluster " << i << ": "
                      << cluster.voxel_positions.size() << " voxels, "
                      << "COM (" << cluster.center_of_mass.x << ", "
                      << cluster.center_of_mass.y << ", "
                      << cluster.center_of_mass.z << ")" << std::endl;
        }
    }
}

void VoxelVisualizer::AnimateCracking(const Vector3& position, float intensity) {
    std::cout << "[VoxelVisualizer] Crack animation at ("
              << position.x << ", " << position.y << ", " << position.z
              << ") intensity: " << intensity << std::endl;
}

void VoxelVisualizer::Clear() {
    highlighted_voxels.clear();
    failed_clusters.clear();
    show_highlights = false;
    show_clusters = false;
}

void VoxelVisualizer::PrintStatus() const {
    std::cout << "\n=== Visualization Status ===" << std::endl;
    std::cout << "Highlighted voxels: " << highlighted_voxels.size() << std::endl;
    std::cout << "Failed clusters: " << failed_clusters.size() << std::endl;
    std::cout << "===========================\n" << std::endl;
}

// ============================================================================
// DestructionUI Implementation
// ============================================================================

void DestructionUI::ShowStructuralAnalysis(const AnalysisResult& result) {
    std::cout << "\n╔════════════════════════════════════╗" << std::endl;
    std::cout << "║   STRUCTURAL ANALYSIS RESULTS      ║" << std::endl;
    std::cout << "╠════════════════════════════════════╣" << std::endl;

    std::cout << "║ Status: " << std::setw(27) << std::left
              << (result.structure_failed ? "FAILED" : "STABLE") << "║" << std::endl;

    std::cout << std::fixed << std::setprecision(1);
    std::cout << "║ Analysis Time: " << std::setw(17) << std::left
              << (std::to_string(result.calculation_time_ms) + " ms") << "║" << std::endl;

    if (result.structure_failed) {
        std::cout << "║ Failed Clusters: " << std::setw(15)
                  << result.failed_clusters.size() << "║" << std::endl;

        std::cout << "╠════════════════════════════════════╣" << std::endl;

        for (size_t i = 0; i < result.failed_clusters.size() && i < 5; i++) {
            const auto& cluster = result.failed_clusters[i];
            std::cout << "║ Cluster " << i << ": "
                      << std::setw(5) << cluster.voxel_positions.size() << " voxels"
                      << std::setw(13) << (" " + std::to_string(static_cast<int>(cluster.total_mass)) + " kg")
                      << " ║" << std::endl;
        }

        if (result.failed_clusters.size() > 5) {
            std::cout << "║ ... and " << (result.failed_clusters.size() - 5)
                      << " more clusters" << std::setw(11) << " " << "║" << std::endl;
        }
    }

    std::cout << "╚════════════════════════════════════╝" << std::endl;
}

void DestructionUI::ShowPhysicsStats(const VoxelPhysicsIntegration& integration) {
    int total = integration.GetDebrisCount();
    int active = integration.GetActiveDebrisCount();
    int settled = integration.GetSettledDebrisCount();

    std::cout << "\n╔════════════════════════════════════╗" << std::endl;
    std::cout << "║      PHYSICS SIMULATION STATS      ║" << std::endl;
    std::cout << "╠════════════════════════════════════╣" << std::endl;
    std::cout << "║ Active Debris:  " << std::setw(5) << active
              << " / " << std::setw(5) << total << std::setw(8) << " " << "║" << std::endl;
    std::cout << "║ Settled Debris: " << std::setw(17) << settled << "║" << std::endl;

    // Progress bar
    if (total > 0) {
        int bar_width = 30;
        float progress = static_cast<float>(settled) / total;
        int filled = static_cast<int>(progress * bar_width);

        std::cout << "║ Progress: [";
        for (int i = 0; i < bar_width; i++) {
            std::cout << (i < filled ? "█" : "░");
        }
        std::cout << "] ║" << std::endl;
    }

    std::cout << "╚════════════════════════════════════╝" << std::endl;
}

void DestructionUI::ShowWarning(const std::string& message) {
    std::cout << "\n⚠️  WARNING: " << message << std::endl;
}

void DestructionUI::ShowDebrisBreakdown(const VoxelPhysicsIntegration& integration) {
    int total = integration.GetDebrisCount();
    int active = integration.GetActiveDebrisCount();
    int settled = integration.GetSettledDebrisCount();
    int settling = total - active - settled;

    std::cout << "\n╔════════════════════════════════════╗" << std::endl;
    std::cout << "║       DEBRIS STATE BREAKDOWN       ║" << std::endl;
    std::cout << "╠════════════════════════════════════╣" << std::endl;
    std::cout << "║ Active:   " << std::setw(24) << active << "║" << std::endl;
    std::cout << "║ Settling: " << std::setw(24) << settling << "║" << std::endl;
    std::cout << "║ Settled:  " << std::setw(24) << settled << "║" << std::endl;
    std::cout << "╠════════════════════════════════════╣" << std::endl;
    std::cout << "║ Total:    " << std::setw(24) << total << "║" << std::endl;
    std::cout << "╚════════════════════════════════════╝" << std::endl;
}

// ============================================================================
// CameraEffects Implementation
// ============================================================================

CameraEffects::CameraEffects()
    : shake_offset(0, 0, 0)
    , shake_intensity(0.0f)
    , shake_duration(0.0f)
    , shake_timer(0.0f) {
}

void CameraEffects::TriggerShake(float intensity, float duration) {
    shake_intensity = intensity;
    shake_duration = duration;
    shake_timer = duration;

    std::cout << "[CameraEffects] Shake triggered: intensity=" << intensity
              << " duration=" << duration << "s" << std::endl;
}

void CameraEffects::Update(float deltaTime) {
    if (shake_timer > 0.0f) {
        // Generate random shake offset
        float rand_x = (static_cast<float>(rand()) / RAND_MAX) * 2.0f - 1.0f;
        float rand_y = (static_cast<float>(rand()) / RAND_MAX) * 2.0f - 1.0f;
        float rand_z = (static_cast<float>(rand()) / RAND_MAX) * 2.0f - 1.0f;

        shake_offset = Vector3(rand_x, rand_y, rand_z) * shake_intensity;

        // Update timer
        shake_timer -= deltaTime;

        // Fade out intensity
        float fade_factor = shake_timer / shake_duration;
        shake_intensity *= (0.9f + 0.1f * fade_factor);

        if (shake_timer <= 0.0f) {
            shake_offset = Vector3(0, 0, 0);
            shake_intensity = 0.0f;
        }
    } else {
        shake_offset = Vector3(0, 0, 0);
    }
}

void CameraEffects::Stop() {
    shake_timer = 0.0f;
    shake_offset = Vector3(0, 0, 0);
    shake_intensity = 0.0f;
}

// ============================================================================
// TimeScale Implementation
// ============================================================================

TimeScale::TimeScale()
    : current_scale(1.0f)
    , target_scale(1.0f)
    , transition_speed(2.0f) {
}

void TimeScale::SetSlowMotion(float scale, float duration) {
    target_scale = std::max(0.0f, std::min(1.0f, scale));

    if (duration > 0.0f) {
        transition_speed = std::abs(current_scale - target_scale) / duration;
    } else {
        current_scale = target_scale;
        transition_speed = 2.0f;
    }

    std::cout << "[TimeScale] Slow-motion set: scale=" << scale
              << " duration=" << duration << "s" << std::endl;
}

void TimeScale::SetNormalSpeed(float duration) {
    SetSlowMotion(1.0f, duration);
}

void TimeScale::Update(float deltaTime) {
    if (current_scale != target_scale) {
        float difference = target_scale - current_scale;
        float step = transition_speed * deltaTime;

        if (std::abs(difference) <= step) {
            current_scale = target_scale;
        } else {
            current_scale += (difference > 0 ? step : -step);
        }
    }
}
