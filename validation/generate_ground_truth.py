#!/usr/bin/env python3
"""
Generate FEM ground truth data for all test cases.

Runs FEM solver on each test structure and exports results as JSON files.
"""

import json
import os
from pathlib import Path

from test_structures import ALL_TEST_CASES
from fem_solver import solve_test_case


def generate_all_ground_truth(output_dir="results"):
    """Generate ground truth JSON files for all test cases."""
    # Create output directory
    output_path = Path(__file__).parent / output_dir
    output_path.mkdir(exist_ok=True)

    print("=" * 70)
    print("FEM Ground Truth Generation")
    print("=" * 70)
    print(f"Output directory: {output_path}")
    print(f"Test cases: {len(ALL_TEST_CASES)}\n")

    results_summary = []

    for i, test_case in enumerate(ALL_TEST_CASES, 1):
        name = test_case["name"]
        print(f"[{i}/{len(ALL_TEST_CASES)}] Processing: {name}")
        print(f"    {test_case['description']}")

        # Solve with FEM
        try:
            result = solve_test_case(test_case)

            # Save to JSON
            output_file = output_path / f"{name}.json"
            with open(output_file, 'w') as f:
                json.dump(result, f, indent=2)

            print(f"    ✓ Method: {result['fem_solver']}")
            print(f"    ✓ Should fail: {result['should_fail']}")
            print(f"    ✓ Failed voxels: {len(result['failed_voxels'])}")
            print(f"    ✓ Saved to: {output_file.name}")

            # Check if result matches expectation
            expected = test_case.get("expected_failure")
            actual = result["should_fail"]
            match = "✓ MATCH" if expected == actual else "✗ MISMATCH"
            print(f"    {match} (expected={expected}, actual={actual})")

            results_summary.append({
                "name": name,
                "expected": expected,
                "actual": actual,
                "match": expected == actual,
                "method": result["fem_solver"],
                "failed_voxels": len(result["failed_voxels"])
            })

        except Exception as e:
            print(f"    ✗ ERROR: {e}")
            results_summary.append({
                "name": name,
                "error": str(e)
            })

        print()

    # Print summary
    print("=" * 70)
    print("Summary")
    print("=" * 70)

    total = len(results_summary)
    matches = sum(1 for r in results_summary if r.get("match", False))
    errors = sum(1 for r in results_summary if "error" in r)

    print(f"Total test cases: {total}")
    print(f"FEM matches expectations: {matches}/{total - errors} ({100 * matches / (total - errors):.1f}%)")
    print(f"Errors: {errors}")
    print()

    if errors == 0 and matches == total:
        print("✓ All test cases processed successfully and match expectations!")
    elif errors == 0:
        print("⚠ Some FEM results don't match expectations - review test case definitions")
    else:
        print("✗ Some test cases failed to process")

    print()
    print(f"Ground truth files generated in: {output_path}/")
    print("Next step: Run C++ validation tests to compare analyzers against FEM")
    print()

    # Save summary
    summary_file = output_path / "_summary.json"
    with open(summary_file, 'w') as f:
        json.dump(results_summary, f, indent=2)
    print(f"Summary saved to: {summary_file}")


if __name__ == "__main__":
    generate_all_ground_truth()
