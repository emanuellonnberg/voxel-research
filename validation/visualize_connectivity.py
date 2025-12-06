#!/usr/bin/env python3
"""
Visualize the L-Shape connectivity issue that causes FEM/analyzer disagreement.
"""

VOXEL_SIZE = 0.05  # meters

def visualize_l_shape():
    """Show the L-shape before and after damage."""

    print("=" * 70)
    print("L-SHAPE CONNECTIVITY ISSUE")
    print("=" * 70)
    print()

    print("BEFORE DAMAGE:")
    print()
    print("    Side view (2D):")
    print("    ")
    print("    Y")
    print("    ↑")
    print("    |  [H3]──[H2]──[H1]")
    print("    |               |")
    print("    |              [C]  ← Corner")
    print("    |               |")
    print("    |              [V]")
    print("    |               |")
    print("    └──[G]──────────────→ X")
    print("       ground")
    print()
    print("    Positions:")
    print("      G: (0.00, 0.00, 0)  - Ground")
    print("      V: (0.00, 0.05, 0)  - Vertical")
    print("      C: (0.00, 0.10, 0)  - Corner")
    print("     H1: (0.05, 0.10, 0)  - Horizontal 1")
    print("     H2: (0.10, 0.10, 0)  - Horizontal 2")
    print("     H3: (0.15, 0.10, 0)  - Horizontal 3")
    print()

    print("AFTER DAMAGE (Remove Corner C):")
    print()
    print("    Side view:")
    print("    ")
    print("    Y")
    print("    ↑")
    print("    |  [H3]──[H2]──[H1]  ← FLOATING?")
    print("    |")
    print("    |               X    ← Corner REMOVED")
    print("    |")
    print("    |              [V]")
    print("    |               |")
    print("    └──[G]──────────────→ X")
    print("       ground")
    print()

    print("=" * 70)
    print("CONNECTIVITY ANALYSIS")
    print("=" * 70)
    print()

    # Calculate distances
    v_pos = (0.00, 0.05, 0.00)
    h1_pos = (0.05, 0.10, 0.00)

    dx = h1_pos[0] - v_pos[0]
    dy = h1_pos[1] - v_pos[1]
    dz = h1_pos[2] - v_pos[2]

    dist = (dx**2 + dy**2 + dz**2)**0.5
    dist_in_voxels = dist / VOXEL_SIZE

    print(f"Distance from V to H1:")
    print(f"  Δx = {dx:.3f}m = {dx/VOXEL_SIZE:.1f} voxel")
    print(f"  Δy = {dy:.3f}m = {dy/VOXEL_SIZE:.1f} voxel")
    print(f"  Δz = {dz:.3f}m = {dz/VOXEL_SIZE:.1f} voxel")
    print(f"  ")
    print(f"  Total distance = {dist:.4f}m = {dist_in_voxels:.3f} voxels")
    print()

    print("=" * 70)
    print("ANALYZER COMPARISONS")
    print("=" * 70)
    print()

    print("1. FEM SOLVER (Buggy - Too Permissive)")
    print(f"   Threshold: 1.5 × voxel_size = {1.5 * VOXEL_SIZE:.4f}m")
    print(f"   Distance: {dist:.4f}m")
    print(f"   Connected? {dist <= 1.5 * VOXEL_SIZE} ← BUG!")
    print(f"   Result: Structure is STABLE (WRONG)")
    print()

    print("2. GAME ANALYZERS (Correct - Orthogonal Only)")
    print("   Check 6 neighbors: ±X, ±Y, ±Z")
    print("   From V (0.00, 0.05, 0):")
    print("     +X → (0.05, 0.05, 0) - No voxel")
    print("     -X → (-0.05, 0.05, 0) - No voxel")
    print("     +Y → (0.00, 0.10, 0) - REMOVED (was corner)")
    print("     -Y → (0.00, 0.00, 0) - Has G ✓")
    print("     +Z → (0.00, 0.05, 0.05) - No voxel")
    print("     -Z → (0.00, 0.05, -0.05) - No voxel")
    print()
    print("   No orthogonal path to H1!")
    print("   Connected? False")
    print("   Result: Structure FAILS (CORRECT)")
    print()

    print("=" * 70)
    print("PHYSICAL REALITY")
    print("=" * 70)
    print()
    print("Voxels are CUBES that touch face-to-face:")
    print()
    print("  Face contact ✓          Corner contact ✗")
    print("  ")
    print("    [2]                      [2]   ← Would slide off!")
    print("    [1]                         [1]")
    print("  ──[0]──                   ──[0]──")
    print("    ground                    ground")
    print()
    print("After removing corner, horizontal voxels have")
    print("CORNER-TO-CORNER contact only = NOT supported")
    print()

    print("=" * 70)
    print("CONCLUSION")
    print("=" * 70)
    print()
    print("✓ Max-Flow is CORRECT - structure should fail")
    print("✓ Spring is CORRECT - structure should fail")
    print("✗ FEM solver is WRONG - diagonal tolerance bug")
    print()
    print("Fix: Change FEM threshold from 1.5× to 1.05× (orthogonal + float tolerance)")
    print("Expected result after fix: Max-Flow = 100% accuracy!")
    print()


if __name__ == "__main__":
    visualize_l_shape()
