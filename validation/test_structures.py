"""
Test structure definitions for FEM validation.

Each structure is defined with:
- voxels: List of (x, y, z, material_id) tuples
- damaged: List of (x, y, z) positions to remove
- expected_failure: Whether structure should fail after damage
- description: What this test validates
"""

# Material IDs (matching MaterialDatabase in C++)
CONCRETE = 1
BRICK = 2
STEEL = 3
WOOD = 4

# Voxel size (meters)
VOXEL_SIZE = 0.05

def make_position(x_idx, y_idx, z_idx):
    """Convert voxel indices to world positions."""
    return (x_idx * VOXEL_SIZE, y_idx * VOXEL_SIZE, z_idx * VOXEL_SIZE)


def add_unique(voxels, seen, x_idx, y_idx, material_id, z_idx=0):
    """Append voxel if that position hasn't been used already."""
    pos = make_position(x_idx, y_idx, z_idx)
    if pos not in seen:
        seen.add(pos)
        voxels.append(pos + (material_id,))


# Test Case 1: Simple Tower (5 voxels, remove base)
TOWER_SIMPLE = {
    "name": "tower_simple",
    "description": "5-voxel tower, remove bottom voxel - should collapse",
    "voxels": [
        make_position(0, i, 0) + (BRICK,) for i in range(5)
    ],
    "damaged": [make_position(0, 0, 0)],
    "expected_failure": True,
    "fem_method": "analytical",  # Simple weight > support check
}

# Test Case 2: Tower on Ground (should NOT fail)
TOWER_GROUNDED = {
    "name": "tower_grounded",
    "description": "5-voxel tower, remove non-structural voxel nearby - should NOT collapse",
    "voxels": [
        make_position(0, i, 0) + (BRICK,) for i in range(5)
    ],
    "damaged": [make_position(1, 0, 0)],  # Damage adjacent voxel, not tower
    "expected_failure": False,
    "fem_method": "analytical",
}

# Test Case 3: Two Towers, Remove One Base
TWO_TOWERS = {
    "name": "two_towers",
    "description": "Two separate 3-voxel towers, remove base of one - only that tower fails",
    "voxels": [
        # Tower 1
        make_position(0, i, 0) + (BRICK,) for i in range(3)
    ] + [
        # Tower 2 (separate)
        make_position(3, i, 0) + (BRICK,) for i in range(3)
    ],
    "damaged": [make_position(0, 0, 0)],  # Remove tower 1 base
    "expected_failure": True,  # Tower 1 fails
    "expected_failed_voxels": [  # Only tower 1 voxels should fail
        make_position(0, 1, 0),
        make_position(0, 2, 0),
    ],
    "fem_method": "analytical",
}

# Test Case 4: Cantilever Beam (horizontal overhang)
CANTILEVER = {
    "name": "cantilever",
    "description": "5-voxel horizontal beam, 1 voxel fixed, 4 hanging - may fail under bending",
    "voxels": [
        # Base (on ground)
        make_position(0, 0, 0) + (CONCRETE,),
        # Horizontal extension
        make_position(1, 0, 0) + (CONCRETE,),
        make_position(2, 0, 0) + (CONCRETE,),
        make_position(3, 0, 0) + (CONCRETE,),
        make_position(4, 0, 0) + (CONCRETE,),
    ],
    "damaged": [],  # No damage - checking if cantilever supports itself
    "expected_failure": False,  # Should hold (concrete is strong)
    "fem_method": "beam_equation",  # σ = M*y/I, compare to strength
}

# Test Case 5: Cantilever Beam - Too Long
CANTILEVER_LONG = {
    "name": "cantilever_long",
    "description": "20-voxel horizontal beam - long span but still stable with concrete properties",
    "voxels": [
        make_position(0, 0, 0) + (BRICK,),  # Weaker material
    ] + [
        make_position(i, 0, 0) + (BRICK,) for i in range(1, 20)
    ],
    "damaged": [],
    "expected_failure": False,
    "fem_method": "beam_equation",
}

# Test Case 6: Bridge with Two Supports
BRIDGE_SUPPORTED = {
    "name": "bridge_supported",
    "description": "10-voxel bridge with supports at both ends - should hold",
    "voxels": [
        # Left support (on ground)
        make_position(0, 0, 0) + (CONCRETE,),
        make_position(0, 1, 0) + (CONCRETE,),
        # Bridge span
        make_position(1, 1, 0) + (CONCRETE,),
        make_position(2, 1, 0) + (CONCRETE,),
        make_position(3, 1, 0) + (CONCRETE,),
        make_position(4, 1, 0) + (CONCRETE,),
        make_position(5, 1, 0) + (CONCRETE,),
        make_position(6, 1, 0) + (CONCRETE,),
        make_position(7, 1, 0) + (CONCRETE,),
        make_position(8, 1, 0) + (CONCRETE,),
        # Right support (on ground)
        make_position(9, 0, 0) + (CONCRETE,),
        make_position(9, 1, 0) + (CONCRETE,),
    ],
    "damaged": [],
    "expected_failure": False,
    "fem_method": "beam_equation",
}

# Test Case 7: Bridge - Remove One Support
BRIDGE_SINGLE_SUPPORT = {
    "name": "bridge_single_support",
    "description": "Bridge with one support removed - forms a long cantilever but still holds",
    "voxels": [
        # Left support (on ground)
        make_position(0, 0, 0) + (CONCRETE,),
        make_position(0, 1, 0) + (CONCRETE,),
        # Bridge span
        make_position(1, 1, 0) + (CONCRETE,),
        make_position(2, 1, 0) + (CONCRETE,),
        make_position(3, 1, 0) + (CONCRETE,),
        make_position(4, 1, 0) + (CONCRETE,),
        make_position(5, 1, 0) + (CONCRETE,),
        make_position(6, 1, 0) + (CONCRETE,),
        make_position(7, 1, 0) + (CONCRETE,),
        make_position(8, 1, 0) + (CONCRETE,),
        # Right support (on ground) - will be removed
        make_position(9, 0, 0) + (CONCRETE,),
        make_position(9, 1, 0) + (CONCRETE,),
    ],
    "damaged": [make_position(9, 0, 0), make_position(9, 1, 0)],  # Remove right support
    "expected_failure": False,
    "fem_method": "beam_equation",
}

# Test Case 8: L-Shape (corner stress)
L_SHAPE = {
    "name": "l_shape",
    "description": "L-shaped structure - tests corner stress distribution",
    "voxels": [
        # Vertical part (on ground)
        make_position(0, 0, 0) + (CONCRETE,),
        make_position(0, 1, 0) + (CONCRETE,),
        make_position(0, 2, 0) + (CONCRETE,),
        # Horizontal part
        make_position(1, 2, 0) + (CONCRETE,),
        make_position(2, 2, 0) + (CONCRETE,),
        make_position(3, 2, 0) + (CONCRETE,),
    ],
    "damaged": [],
    "expected_failure": False,  # Should hold (concrete strong)
    "fem_method": "simplified_fem",
}

# Test Case 9: L-Shape Remove Corner
L_SHAPE_DAMAGED = {
    "name": "l_shape_damaged",
    "description": "L-shape with corner removed - horizontal part should fail",
    "voxels": [
        # Vertical part (on ground)
        make_position(0, 0, 0) + (CONCRETE,),
        make_position(0, 1, 0) + (CONCRETE,),
        make_position(0, 2, 0) + (CONCRETE,),
        # Horizontal part
        make_position(1, 2, 0) + (CONCRETE,),
        make_position(2, 2, 0) + (CONCRETE,),
        make_position(3, 2, 0) + (CONCRETE,),
    ],
    "damaged": [make_position(0, 2, 0)],  # Remove corner
    "expected_failure": True,  # Horizontal part loses support
    "fem_method": "simplified_fem",
}

# Test Case 10: Column Under Heavy Load
HEAVY_LOAD = {
    "name": "heavy_load",
    "description": "Single column with many voxels on top - column remains within compression limits",
    "voxels": [
        # Column base
        make_position(0, 0, 0) + (WOOD,),  # Weak material
        # Heavy load on top (20 concrete voxels)
    ] + [
        make_position(0, i, 0) + (CONCRETE,) for i in range(1, 21)
    ],
    "damaged": [],
    "expected_failure": False,
    "fem_method": "compression",  # σ = F/A vs compressive strength
}

# Test Case 11: Arch (Complex)
ARCH = {
    "name": "arch",
    "description": "Simple arch structure - simplified FEM predicts failure under self weight",
    "voxels": [
        # Left support
        make_position(0, 0, 0) + (BRICK,),
        make_position(0, 1, 0) + (BRICK,),
        make_position(0, 2, 0) + (BRICK,),
        # Arch curve (simplified - 3 voxels)
        make_position(1, 3, 0) + (BRICK,),
        make_position(2, 3, 0) + (BRICK,),
        make_position(3, 3, 0) + (BRICK,),
        # Right support
        make_position(4, 0, 0) + (BRICK,),
        make_position(4, 1, 0) + (BRICK,),
        make_position(4, 2, 0) + (BRICK,),
    ],
    "damaged": [],
    "expected_failure": True,
    "fem_method": "simplified_fem",
}

def make_multi_span_bridge_voxels():
    voxels = []
    seen = set()
    for y in range(0, 2):
        add_unique(voxels, seen, 0, y, CONCRETE)
        add_unique(voxels, seen, 4, y, CONCRETE)
        add_unique(voxels, seen, 8, y, CONCRETE)
    for x in range(0, 9):
        add_unique(voxels, seen, x, 2, STEEL)
    return voxels


# Test Case 12: Multi-span bridge with redundant supports
MULTI_SPAN_BRIDGE = {
    "name": "multi_span_bridge",
    "description": "Three-support bridge, remove middle pier - outer supports should carry spans",
    "voxels": make_multi_span_bridge_voxels(),
    "damaged": [
        make_position(4, 0, 0),
        make_position(4, 1, 0),
        make_position(4, 2, 0),
        make_position(5, 2, 0),
        make_position(8, 0, 0),
        make_position(8, 1, 0),
        make_position(7, 2, 0),
    ],
    "expected_failure": True,
    "fem_method": "simplified_fem",
}

def make_multi_level_frame_voxels():
    voxels = []
    seen = set()
    for y in range(0, 4):
        add_unique(voxels, seen, 0, y, STEEL)
        add_unique(voxels, seen, 2, y, STEEL)
    for x in [1, 2, 3]:
        add_unique(voxels, seen, x, 2, CONCRETE)
        add_unique(voxels, seen, x, 3, CONCRETE)
    return voxels


# Test Case 13: Two-story frame losing a column
MULTI_LEVEL_FRAME = {
    "name": "multi_level_frame",
    "description": "Two-story frame, remove one column at base - expected progressive collapse",
    "voxels": make_multi_level_frame_voxels(),
    "damaged": [make_position(2, y, 0) for y in range(0, 4)],
    "expected_failure": True,
    "fem_method": "simplified_fem",
}

# Test Case 14: Diagonal braced frame retains stability
def make_diagonal_braced_voxels():
    voxels = []
    seen = set()
    for y in range(0, 4):
        add_unique(voxels, seen, 0, y, CONCRETE)
        add_unique(voxels, seen, 2, y, CONCRETE)
    add_unique(voxels, seen, 1, 3, CONCRETE)
    add_unique(voxels, seen, 1, 2, CONCRETE)
    add_unique(voxels, seen, 1, 1, CONCRETE)
    return voxels


DIAGONAL_BRACED_FRAME = {
    "name": "diagonal_braced_frame",
    "description": "Frame with diagonal bracing - remove one column, brace is insufficient so structure fails",
    "voxels": make_diagonal_braced_voxels(),
    "damaged": [
        make_position(2, 0, 0),
        make_position(1, 1, 0),
        make_position(1, 2, 0),
        make_position(1, 3, 0),
    ],
    "expected_failure": True,
    "fem_method": "simplified_fem",
}


# All test cases
ALL_TEST_CASES = [
    TOWER_SIMPLE,
    TOWER_GROUNDED,
    TWO_TOWERS,
    CANTILEVER,
    CANTILEVER_LONG,
    BRIDGE_SUPPORTED,
    BRIDGE_SINGLE_SUPPORT,
    L_SHAPE,
    L_SHAPE_DAMAGED,
    HEAVY_LOAD,
    ARCH,
    MULTI_SPAN_BRIDGE,
    MULTI_LEVEL_FRAME,
    DIAGONAL_BRACED_FRAME,
]


def get_test_case(name):
    """Get test case by name."""
    for test in ALL_TEST_CASES:
        if test["name"] == name:
            return test
    raise ValueError(f"Test case '{name}' not found")


if __name__ == "__main__":
    print(f"Defined {len(ALL_TEST_CASES)} test cases:")
    for test in ALL_TEST_CASES:
        print(f"  - {test['name']}: {test['description']}")
        print(f"    Voxels: {len(test['voxels'])}, Expected failure: {test['expected_failure']}")
        print(f"    Method: {test['fem_method']}")
