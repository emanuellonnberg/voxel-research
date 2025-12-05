"""
Simplified FEM solver for voxel structure validation.

Uses analytical solutions where possible, numerical approximations otherwise.
This provides ground truth for validating the Spring and Max-Flow analyzers.
"""

import math
import json
from typing import List, Tuple, Dict, Set

# Material properties (matching MaterialDatabase.cpp)
MATERIALS = {
    1: {  # CONCRETE
        "name": "Concrete",
        "density": 2400.0,  # kg/m³
        "compressive_strength": 30e6,  # Pa (30 MPa)
        "tensile_strength": 3e6,  # Pa (10x weaker in tension)
        "youngs_modulus": 30e9,  # Pa
    },
    2: {  # BRICK
        "name": "Brick",
        "density": 1800.0,
        "compressive_strength": 20e6,
        "tensile_strength": 2e6,
        "youngs_modulus": 15e9,
    },
    3: {  # STEEL
        "name": "Steel",
        "density": 7850.0,
        "compressive_strength": 400e6,
        "tensile_strength": 400e6,
        "youngs_modulus": 200e9,
    },
    4: {  # WOOD
        "name": "Wood",
        "density": 600.0,
        "compressive_strength": 30e6,
        "tensile_strength": 40e6,  # Good in tension
        "youngs_modulus": 10e9,
    },
}

GRAVITY = 9.81  # m/s²
VOXEL_SIZE = 0.05  # m
VOXEL_VOLUME = VOXEL_SIZE ** 3  # m³


class FEMSolver:
    """Simplified FEM solver using analytical methods."""

    def __init__(self, test_structure: Dict):
        self.structure = test_structure
        self.voxels = test_structure["voxels"]
        self.damaged = test_structure.get("damaged", [])
        self.voxel_size = VOXEL_SIZE

        # Remove damaged voxels
        damaged_set = set(self.damaged)
        self.voxels = [v for v in self.voxels if (v[0], v[1], v[2]) not in damaged_set]

        # Parse voxel data
        self.positions = [(v[0], v[1], v[2]) for v in self.voxels]
        self.material_ids = [v[3] for v in self.voxels]

    def solve(self) -> Dict:
        """Run FEM analysis and return results."""
        method = self.structure.get("fem_method", "simplified_fem")

        if method == "analytical":
            return self._solve_analytical()
        elif method == "beam_equation":
            return self._solve_beam()
        elif method == "compression":
            return self._solve_compression()
        elif method == "simplified_fem":
            return self._solve_simplified_fem()
        else:
            raise ValueError(f"Unknown FEM method: {method}")

    def _solve_analytical(self) -> Dict:
        """
        Analytical solution for simple cases.
        Checks if any voxel is unsupported (not connected to ground).
        """
        # Find ground-connected voxels (BFS)
        ground_tolerance = 0.01
        grounded = set()

        for i, pos in enumerate(self.positions):
            if pos[1] <= ground_tolerance:
                grounded.add(i)

        # BFS to find all connected voxels
        supported = set(grounded)
        queue = list(grounded)

        while queue:
            current = queue.pop(0)
            current_pos = self.positions[current]

            for i, pos in enumerate(self.positions):
                if i in supported:
                    continue

                # Check if adjacent (within 1.5 * voxel_size)
                dist = math.sqrt(
                    (pos[0] - current_pos[0]) ** 2 +
                    (pos[1] - current_pos[1]) ** 2 +
                    (pos[2] - current_pos[2]) ** 2
                )

                if dist <= 1.5 * self.voxel_size:
                    supported.add(i)
                    queue.append(i)

        # Failed voxels = unsupported voxels
        failed_indices = set(range(len(self.positions))) - supported
        failed_voxels = [
            {
                "position": list(self.positions[i]),
                "stress": 0.0,  # Unsupported = infinite stress
                "failure_mode": "unsupported"
            }
            for i in failed_indices
        ]

        return {
            "structure_name": self.structure["name"],
            "fem_solver": "analytical_connectivity",
            "should_fail": len(failed_voxels) > 0,
            "failed_voxels": failed_voxels,
            "method_description": "BFS connectivity check - voxels not connected to ground fail",
        }

    def _solve_beam(self) -> Dict:
        """
        Beam equation solution for cantilevers and bridges.
        Uses σ = M*y/I where M = bending moment, y = distance from neutral axis, I = moment of inertia.
        """
        if len(self.positions) == 0:
            return self._empty_result()

        # Find fixed end (lowest Y, leftmost X)
        fixed_pos = min(self.positions, key=lambda p: (p[1], p[0]))

        # Find maximum distance from fixed end (cantilever length)
        max_dist = 0.0
        for pos in self.positions:
            dist = abs(pos[0] - fixed_pos[0])  # Horizontal distance
            max_dist = max(max_dist, dist)

        if max_dist < self.voxel_size:
            # No cantilever, just a vertical structure
            return self._solve_analytical()

        # Calculate bending stress at fixed end
        # Simplified: uniform distributed load (self-weight)
        total_mass = sum(
            MATERIALS[mat_id]["density"] * VOXEL_VOLUME
            for mat_id in self.material_ids
        )
        total_weight = total_mass * GRAVITY  # N

        # Bending moment at fixed end: M = w * L² / 2 (uniform load)
        L = max_dist  # cantilever length
        M = total_weight * L / 2  # Simplified moment

        # Section properties (rectangular cross-section, 1 voxel wide)
        b = self.voxel_size  # width
        h = self.voxel_size  # height
        I = (b * h ** 3) / 12  # moment of inertia
        y = h / 2  # distance to outer fiber

        # Bending stress
        sigma = M * y / I  # Pa

        # Check if stress exceeds material strength
        # Use first voxel's material as reference
        mat = MATERIALS[self.material_ids[0]]
        failure_stress = mat["tensile_strength"]  # Bending creates tension on bottom

        failed_voxels = []
        if sigma > failure_stress:
            # All voxels fail if max stress exceeded
            for i, pos in enumerate(self.positions):
                failed_voxels.append({
                    "position": list(pos),
                    "stress": sigma,
                    "failure_mode": "bending_tension",
                    "stress_ratio": sigma / failure_stress
                })

        return {
            "structure_name": self.structure["name"],
            "fem_solver": "beam_equation",
            "should_fail": len(failed_voxels) > 0,
            "failed_voxels": failed_voxels,
            "max_stress": sigma,
            "failure_stress": failure_stress,
            "cantilever_length": L,
            "method_description": f"Beam bending: σ={sigma/1e6:.1f} MPa vs {failure_stress/1e6:.1f} MPa limit",
        }

    def _solve_compression(self) -> Dict:
        """
        Compression stress solution.
        Uses σ = F/A where F = weight above, A = cross-sectional area.
        """
        failed_voxels = []

        for i, pos in enumerate(self.positions):
            # Calculate weight above this voxel
            weight_above = 0.0
            for j, other_pos in enumerate(self.positions):
                if other_pos[1] > pos[1]:  # Above current voxel
                    mat = MATERIALS[self.material_ids[j]]
                    mass = mat["density"] * VOXEL_VOLUME
                    weight_above += mass * GRAVITY

            # Cross-sectional area (single voxel)
            area = self.voxel_size ** 2

            # Compressive stress
            sigma = weight_above / area if area > 0 else 0.0

            # Check against material strength
            mat = MATERIALS[self.material_ids[i]]
            failure_stress = mat["compressive_strength"]

            if sigma > failure_stress:
                failed_voxels.append({
                    "position": list(pos),
                    "stress": sigma,
                    "failure_mode": "compression",
                    "stress_ratio": sigma / failure_stress,
                    "weight_above_N": weight_above
                })

        return {
            "structure_name": self.structure["name"],
            "fem_solver": "compression_stress",
            "should_fail": len(failed_voxels) > 0,
            "failed_voxels": failed_voxels,
            "method_description": "Axial compression: σ = F/A vs compressive strength",
        }

    def _solve_simplified_fem(self) -> Dict:
        """
        Simplified FEM for complex structures.
        Combines connectivity check with stress estimation.
        """
        # First check connectivity
        connectivity_result = self._solve_analytical()

        if connectivity_result["should_fail"]:
            return connectivity_result

        # If connected, estimate stress distribution
        # For now, use compression check as approximation
        compression_result = self._solve_compression()

        return {
            **compression_result,
            "fem_solver": "simplified_fem_hybrid",
            "method_description": "Connectivity + compression stress estimation",
        }

    def _empty_result(self) -> Dict:
        """Return result for empty structure."""
        return {
            "structure_name": self.structure["name"],
            "fem_solver": "empty",
            "should_fail": True,  # No voxels = failed
            "failed_voxels": [],
            "method_description": "Empty structure after damage",
        }


def solve_test_case(test_structure: Dict) -> Dict:
    """
    Solve a single test case and return FEM ground truth.
    """
    solver = FEMSolver(test_structure)
    result = solver.solve()

    # Add metadata
    result["voxel_count"] = len(test_structure["voxels"])
    result["damaged_count"] = len(test_structure.get("damaged", []))
    result["expected_failure"] = test_structure.get("expected_failure", None)

    return result


if __name__ == "__main__":
    # Test the solver
    from test_structures import TOWER_SIMPLE, CANTILEVER, HEAVY_LOAD

    print("Testing FEM Solver\n")
    print("=" * 60)

    for test in [TOWER_SIMPLE, CANTILEVER, HEAVY_LOAD]:
        print(f"\nTest: {test['name']}")
        print(f"Description: {test['description']}")

        result = solve_test_case(test)

        print(f"Method: {result['fem_solver']}")
        print(f"Should fail: {result['should_fail']}")
        print(f"Failed voxels: {len(result['failed_voxels'])}")
        if result['should_fail']:
            print(f"Failure mode: {result['failed_voxels'][0]['failure_mode'] if result['failed_voxels'] else 'N/A'}")
        print("-" * 60)
