"""
Simplified FEM solver for voxel structure validation.

Uses analytical solutions where possible, numerical approximations otherwise.
This provides ground truth for validating the Spring and Max-Flow analyzers.
"""

import math
from collections import deque
from typing import Dict, List, Set, Tuple

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
ORTHOGONAL_OFFSETS = [
    (1, 0, 0),
    (-1, 0, 0),
    (0, 1, 0),
    (0, -1, 0),
    (0, 0, 1),
    (0, 0, -1),
]


class FEMSolver:
    """Simplified FEM solver using analytical methods."""

    def __init__(self, test_structure: Dict):
        self.structure = test_structure
        self.voxels = test_structure["voxels"]
        self.damaged = test_structure.get("damaged", [])
        self.voxel_size = VOXEL_SIZE

        damaged_set = set(self.damaged)
        self.voxels = [v for v in self.voxels if (v[0], v[1], v[2]) not in damaged_set]

        self.positions = [(v[0], v[1], v[2]) for v in self.voxels]
        self.material_ids = [v[3] for v in self.voxels]
        self.grid_positions = [self._to_grid_coords(pos) for pos in self.positions]
        self.position_to_index = {
            grid_pos: idx for idx, grid_pos in enumerate(self.grid_positions)
        }

    def _to_grid_coords(self, pos: Tuple[float, float, float]) -> Tuple[int, int, int]:
        return tuple(int(round(coord / self.voxel_size)) for coord in pos)

    def _neighbor_indices(self, idx: int) -> List[int]:
        neighbors: List[int] = []
        gx, gy, gz = self.grid_positions[idx]
        for dx, dy, dz in ORTHOGONAL_OFFSETS:
            neighbor_pos = (gx + dx, gy + dy, gz + dz)
            neighbor_idx = self.position_to_index.get(neighbor_pos)
            if neighbor_idx is not None:
                neighbors.append(neighbor_idx)
        return neighbors

    def solve(self) -> Dict:
        method = self.structure.get("fem_method", "simplified_fem")
        if method == "analytical":
            return self._solve_analytical()
        if method == "beam_equation":
            return self._solve_beam()
        if method == "compression":
            return self._solve_compression()
        if method == "simplified_fem":
            return self._solve_simplified_fem()
        raise ValueError(f"Unknown FEM method: {method}")

    def _compute_connectivity(self) -> Tuple[Set[int], List[float], List[int]]:
        n = len(self.positions)
        if n == 0:
            return set(), [], []

        ground_nodes = [
            i for i, (_, gy, _) in enumerate(self.grid_positions) if gy <= 0
        ]

        distances = [math.inf] * n
        supported: Set[int] = set()
        queue: deque[int] = deque()

        for idx in ground_nodes:
            distances[idx] = 0.0
            supported.add(idx)
            queue.append(idx)

        while queue:
            current = queue.popleft()
            for neighbor in self._neighbor_indices(current):
                if distances[neighbor] != math.inf:
                    continue
                distances[neighbor] = distances[current] + 1.0
                supported.add(neighbor)
                queue.append(neighbor)

        failed_indices = [i for i in range(n) if distances[i] == math.inf]
        return supported, distances, failed_indices

    def _solve_analytical(self) -> Dict:
        _, distances, failed_indices = self._compute_connectivity()
        failed_voxels = [
            {
                "position": list(self.positions[i]),
                "stress": 0.0,
                "failure_mode": "unsupported",
            }
            for i in failed_indices
        ]

        return {
            "structure_name": self.structure["name"],
            "fem_solver": "analytical_connectivity",
            "should_fail": len(failed_voxels) > 0,
            "failed_voxels": failed_voxels,
            "distance_to_ground": distances,
            "method_description": "BFS connectivity check - voxels not connected to ground fail",
        }

    def _solve_beam(self) -> Dict:
        if not self.positions:
            return self._empty_result()

        fixed_pos = min(self.positions, key=lambda p: (p[1], p[0]))
        max_dist = max(abs(pos[0] - fixed_pos[0]) for pos in self.positions)

        if max_dist < self.voxel_size:
            return self._solve_analytical()

        total_mass = sum(
            MATERIALS[mat_id]["density"] * VOXEL_VOLUME for mat_id in self.material_ids
        )
        total_weight = total_mass * GRAVITY

        L = max_dist
        M = total_weight * L / 2.0
        b = self.voxel_size
        h = self.voxel_size
        I = (b * h**3) / 12.0
        y = h / 2.0
        sigma = M * y / I

        mat = MATERIALS[self.material_ids[0]]
        failure_stress = mat["tensile_strength"]

        failed_voxels = []
        if sigma > failure_stress:
            for pos in self.positions:
                failed_voxels.append(
                    {
                        "position": list(pos),
                        "stress": sigma,
                        "failure_mode": "bending_tension",
                        "stress_ratio": sigma / failure_stress,
                    }
                )

        return {
            "structure_name": self.structure["name"],
            "fem_solver": "beam_equation",
            "should_fail": len(failed_voxels) > 0,
            "failed_voxels": failed_voxels,
            "max_stress": sigma,
            "failure_stress": failure_stress,
            "cantilever_length": L,
            "method_description": f"Beam bending: s={sigma/1e6:.1f} MPa vs {failure_stress/1e6:.1f} MPa limit",
        }

    def _solve_compression(self) -> Dict:
        failed_voxels = []

        for i, pos in enumerate(self.positions):
            weight_above = 0.0
            for j, other_pos in enumerate(self.positions):
                if other_pos[1] > pos[1]:
                    mat = MATERIALS[self.material_ids[j]]
                    mass = mat["density"] * VOXEL_VOLUME
                    weight_above += mass * GRAVITY

            area = self.voxel_size**2
            sigma = weight_above / area if area > 0 else float("inf")
            mat = MATERIALS[self.material_ids[i]]
            failure_stress = mat["compressive_strength"]

            if sigma > failure_stress:
                failed_voxels.append(
                    {
                        "position": list(pos),
                        "stress": sigma,
                        "failure_mode": "compression",
                        "stress_ratio": sigma / failure_stress,
                        "weight_above_N": weight_above,
                    }
                )

        return {
            "structure_name": self.structure["name"],
            "fem_solver": "compression_stress",
            "should_fail": len(failed_voxels) > 0,
            "failed_voxels": failed_voxels,
            "method_description": "Axial compression: s = F/A vs compressive strength",
        }

    def _solve_load_distribution(self, distance_to_ground: List[float]) -> Dict:
        n = len(self.positions)
        if n == 0:
            return self._empty_result()

        voxel_masses = [
            MATERIALS[self.material_ids[i]]["density"] * VOXEL_VOLUME
            for i in range(n)
        ]
        accumulated_mass = voxel_masses[:]
        unsupported: Set[int] = set()

        order = sorted(range(n), key=lambda i: distance_to_ground[i], reverse=True)
        for idx in order:
            dist = distance_to_ground[idx]
            if dist == math.inf:
                unsupported.add(idx)
                continue
            if dist == 0.0:
                continue

            supports = [
                neighbor
                for neighbor in self._neighbor_indices(idx)
                if distance_to_ground[neighbor] < dist
            ]

            if not supports:
                unsupported.add(idx)
                continue

            share = accumulated_mass[idx] / len(supports)
            for neighbor in supports:
                accumulated_mass[neighbor] += share

        area = self.voxel_size**2
        failed_voxels = [
            {
                "position": list(self.positions[i]),
                "stress": 0.0,
                "failure_mode": "unsupported",
            }
            for i in sorted(unsupported)
        ]

        for idx in range(n):
            if idx in unsupported:
                continue
            force = accumulated_mass[idx] * GRAVITY
            stress = force / area if area > 0 else float("inf")
            failure_stress = MATERIALS[self.material_ids[idx]]["compressive_strength"]
            if stress > failure_stress:
                failed_voxels.append(
                    {
                        "position": list(self.positions[idx]),
                        "stress": stress,
                        "failure_mode": "compression",
                        "stress_ratio": stress / failure_stress,
                    }
                )

        return {
            "structure_name": self.structure["name"],
            "fem_solver": "load_distribution",
            "should_fail": len(failed_voxels) > 0,
            "failed_voxels": failed_voxels,
            "method_description": "Connectivity-respecting load distribution with compression checks",
        }

    def _solve_simplified_fem(self) -> Dict:
        _, distances, failed_indices = self._compute_connectivity()
        if failed_indices:
            failed_voxels = [
                {
                    "position": list(self.positions[i]),
                    "stress": 0.0,
                    "failure_mode": "unsupported",
                }
                for i in failed_indices
            ]
            return {
                "structure_name": self.structure["name"],
                "fem_solver": "simplified_fem_connectivity",
                "should_fail": True,
                "failed_voxels": failed_voxels,
                "method_description": "Connectivity analysis detected unsupported voxels",
            }

        load_result = self._solve_load_distribution(distances)
        if load_result["should_fail"]:
            return load_result

        compression_result = self._solve_compression()
        return {
            **compression_result,
            "fem_solver": "simplified_fem_hybrid",
            "method_description": "Connectivity + load propagation + compression stress estimation",
        }

    def _empty_result(self) -> Dict:
        return {
            "structure_name": self.structure["name"],
            "fem_solver": "empty",
            "should_fail": True,
            "failed_voxels": [],
            "method_description": "Empty structure after damage",
        }


def solve_test_case(test_structure: Dict) -> Dict:
    solver = FEMSolver(test_structure)
    result = solver.solve()
    result["voxel_count"] = len(test_structure["voxels"])
    result["damaged_count"] = len(test_structure.get("damaged", []))
    result["expected_failure"] = test_structure.get("expected_failure", None)
    return result


if __name__ == "__main__":
    from test_structures import CANTILEVER, HEAVY_LOAD, TOWER_SIMPLE

    print("Testing FEM Solver")
    print("=" * 60)

    for test in [TOWER_SIMPLE, CANTILEVER, HEAVY_LOAD]:
        print(f"\nTest: {test['name']}")
        print(f"Description: {test['description']}")
        result = solve_test_case(test)
        print(f"Method: {result['fem_solver']}")
        print(f"Should fail: {result['should_fail']}")
        print(f"Failed voxels: {len(result['failed_voxels'])}")
        if result["should_fail"]:
            print(
                "Failure mode:",
                result["failed_voxels"][0]["failure_mode"]
                if result["failed_voxels"]
                else "N/A",
            )
        print("-" * 60)
