#pragma once

#include "Vector3.h"
#include "VoxelWorld.h"
#include "VoxelCluster.h"
#include <vector>
#include <unordered_set>
#include <unordered_map>

/**
 * StabilityInfo - Stability analysis results for a cluster
 *
 * Contains metrics describing how stable a voxel cluster is.
 * Used to determine if structures should collapse after damage.
 */
struct StabilityInfo {
    bool is_grounded;              // Does cluster touch the ground?
    bool is_stable;                // Is the structure stable (COM over support)?
    float stability_score;         // 0.0 (unstable) to 1.0 (very stable)
    float tip_risk;                // 0.0 (safe) to 1.0 (about to tip)
    Vector3 center_of_mass;        // Center of mass
    Vector3 support_center;        // Center of support polygon (2D)
    float support_area;            // Area of support base (m²)
    int num_support_voxels;        // Number of voxels providing support
    int num_unsupported_voxels;    // Number of voxels with no support path

    StabilityInfo()
        : is_grounded(false)
        , is_stable(false)
        , stability_score(0.0f)
        , tip_risk(1.0f)
        , center_of_mass(Vector3::Zero())
        , support_center(Vector3::Zero())
        , support_area(0.0f)
        , num_support_voxels(0)
        , num_unsupported_voxels(0)
    {}
};

/**
 * SupportType - Classification of voxel support
 */
enum class SupportType {
    GROUNDED,      // Directly touching ground
    SUPPORTED,     // Indirectly supported via other voxels
    UNSUPPORTED    // No support path to ground
};

/**
 * VoxelStability - Static utility class for structural stability analysis
 *
 * Analyzes voxel structures to determine:
 * - Which voxels are supported (have a path to ground)
 * - Stability of structures (center of mass vs support)
 * - Risk of tipping or collapse
 *
 * Used by game logic to decide when structures should fall after damage.
 */
class VoxelStability {
public:
    /**
     * Analyze stability of a voxel cluster
     *
     * Performs comprehensive stability analysis including:
     * - Ground contact detection
     * - Support chain analysis
     * - Center of mass vs support base
     * - Tip risk calculation
     *
     * @param world The voxel world
     * @param cluster The cluster to analyze
     * @param ground_level Y-coordinate of ground plane (default: 0.0)
     * @return Stability information
     */
    static StabilityInfo AnalyzeStability(
        const VoxelWorld& world,
        const VoxelCluster& cluster,
        float ground_level = 0.0f
    );

    /**
     * Find all voxels that provide direct ground support
     *
     * Returns voxels that are touching the ground plane.
     * These are the foundation voxels that support the structure.
     *
     * @param world The voxel world
     * @param cluster The cluster to analyze
     * @param ground_level Y-coordinate of ground plane (default: 0.0)
     * @return Set of voxel positions providing ground support
     */
    static std::unordered_set<Vector3, Vector3::Hash> FindGroundSupportVoxels(
        const VoxelWorld& world,
        const VoxelCluster& cluster,
        float ground_level = 0.0f
    );

    /**
     * Find all voxels that are supported (have path to ground)
     *
     * Uses flood-fill from ground voxels to find all voxels
     * that have a structural path to the ground.
     *
     * @param world The voxel world
     * @param cluster The cluster to analyze
     * @param ground_level Y-coordinate of ground plane (default: 0.0)
     * @return Set of supported voxel positions
     */
    static std::unordered_set<Vector3, Vector3::Hash> FindSupportedVoxels(
        const VoxelWorld& world,
        const VoxelCluster& cluster,
        float ground_level = 0.0f
    );

    /**
     * Classify support type for each voxel in cluster
     *
     * Determines if each voxel is:
     * - GROUNDED: touching ground
     * - SUPPORTED: supported via other voxels
     * - UNSUPPORTED: no path to ground (should fall)
     *
     * @param world The voxel world
     * @param cluster The cluster to analyze
     * @param ground_level Y-coordinate of ground plane (default: 0.0)
     * @return Map of voxel positions to support types
     */
    static std::unordered_map<Vector3, SupportType, Vector3::Hash> ClassifySupport(
        const VoxelWorld& world,
        const VoxelCluster& cluster,
        float ground_level = 0.0f
    );

    /**
     * Check if a specific voxel has support path to ground
     *
     * Uses BFS to find if there's a connected path from the voxel
     * to any ground-touching voxel.
     *
     * @param world The voxel world
     * @param position Voxel position to check
     * @param ground_level Y-coordinate of ground plane (default: 0.0)
     * @return True if voxel has support path to ground
     */
    static bool HasSupportPath(
        const VoxelWorld& world,
        const Vector3& position,
        float ground_level = 0.0f
    );

    /**
     * Calculate stability score for a cluster
     *
     * Score from 0.0 (unstable) to 1.0 (very stable) based on:
     * - How far COM is from support center (lower = more stable)
     * - Support area relative to structure size (larger = more stable)
     * - Support distribution (more uniform = more stable)
     *
     * @param cluster The cluster with calculated COM
     * @param support_voxels Voxels providing ground support
     * @return Stability score (0.0 to 1.0)
     */
    static float CalculateStabilityScore(
        const VoxelCluster& cluster,
        const std::unordered_set<Vector3, Vector3::Hash>& support_voxels
    );

    /**
     * Calculate tip risk for a cluster
     *
     * Measures how close the center of mass is to the edge of the
     * support polygon. Higher values = more likely to tip over.
     *
     * @param cluster The cluster with calculated COM
     * @param support_voxels Voxels providing ground support
     * @return Tip risk (0.0 = safe, 1.0 = about to tip)
     */
    static float CalculateTipRisk(
        const VoxelCluster& cluster,
        const std::unordered_set<Vector3, Vector3::Hash>& support_voxels
    );

    /**
     * Check if center of mass is over support polygon
     *
     * Projects COM and support voxels to 2D (XZ plane) and checks
     * if COM falls within the convex hull of support points.
     *
     * @param cluster The cluster with calculated COM
     * @param support_voxels Voxels providing ground support
     * @return True if COM is over support base
     */
    static bool IsCenterOfMassSupported(
        const VoxelCluster& cluster,
        const std::unordered_set<Vector3, Vector3::Hash>& support_voxels
    );

    /**
     * Calculate center of support polygon (2D projection)
     *
     * Computes the centroid of support voxels in the XZ plane.
     *
     * @param support_voxels Voxels providing ground support
     * @return Center point of support (y=0)
     */
    static Vector3 CalculateSupportCenter(
        const std::unordered_set<Vector3, Vector3::Hash>& support_voxels
    );

    /**
     * Calculate support area (2D projection in XZ plane)
     *
     * Estimates the area covered by support voxels.
     * Currently uses bounding rectangle approximation.
     *
     * @param support_voxels Voxels providing ground support
     * @return Support area in square meters
     */
    static float CalculateSupportArea(
        const std::unordered_set<Vector3, Vector3::Hash>& support_voxels
    );

private:
    /**
     * Check if a voxel is touching the ground
     *
     * @param position Voxel position
     * @param ground_level Y-coordinate of ground plane
     * @return True if voxel is at or below ground level
     */
    static bool IsGroundVoxel(const Vector3& position, float ground_level);
};
