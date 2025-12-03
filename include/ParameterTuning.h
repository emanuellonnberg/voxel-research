#pragma once

#include "StructuralAnalyzer.h"
#include "VoxelWorld.h"
#include <vector>
#include <string>
#include <functional>

/**
 * ParameterSensitivityResult - Results from sensitivity analysis
 *
 * Measures how changing a single parameter affects analysis performance
 * and results across a range of values.
 */
struct ParameterSensitivityResult {
    std::string parameter_name;
    std::vector<float> test_values;              // Parameter values tested
    std::vector<float> calculation_times_ms;     // Time taken for each value
    std::vector<int> iterations_used;            // Iterations for each value
    std::vector<bool> converged;                 // Whether analysis converged
    std::vector<int> nodes_analyzed;             // Number of nodes analyzed
    std::vector<int> failed_nodes;               // Number of failed nodes

    // Summary statistics
    float min_time_ms;
    float max_time_ms;
    float avg_time_ms;
    float optimal_value;                         // Value with best performance
    std::string analysis_summary;
};

/**
 * AutoTuningResult - Results from automatic parameter optimization
 *
 * Contains the best parameters found through grid search or other
 * optimization methods.
 */
struct AutoTuningResult {
    StructuralAnalyzer::Parameters optimal_params;
    float avg_calculation_time_ms;
    float avg_iterations;
    int total_tests_run;
    std::string tuning_method;                   // "grid_search", "random", etc.
    std::string summary;
};

/**
 * ParameterTuning - Utilities for parameter analysis and optimization
 *
 * Day 17: Tools for understanding parameter sensitivity and finding
 * optimal parameter values for specific use cases.
 */
class ParameterTuning {
public:
    /**
     * Analyze sensitivity of a single parameter
     *
     * Tests how a parameter affects performance across a range of values,
     * while keeping all other parameters constant.
     *
     * @param analyzer The analyzer to test (params will be modified)
     * @param world Test world to analyze
     * @param damaged_positions Damage to apply
     * @param param_name Name of parameter to test
     * @param test_values Values to test for this parameter
     * @return Sensitivity analysis results
     */
    static ParameterSensitivityResult AnalyzeParameterSensitivity(
        StructuralAnalyzer& analyzer,
        VoxelWorld& world,
        const std::vector<Vector3>& damaged_positions,
        const std::string& param_name,
        const std::vector<float>& test_values
    );

    /**
     * Run auto-tuning with grid search
     *
     * Systematically tests combinations of parameter values to find
     * the optimal configuration for a given test scenario.
     *
     * @param world Test world to use for tuning
     * @param damaged_positions Damage scenario for testing
     * @param num_iterations Number of grid search iterations
     * @return Optimal parameters found
     */
    static AutoTuningResult AutoTuneParameters(
        VoxelWorld& world,
        const std::vector<Vector3>& damaged_positions,
        int num_iterations = 10
    );

    /**
     * Save sensitivity results to CSV file
     *
     * @param result Sensitivity analysis results
     * @param filename Output CSV file path
     * @return true if successful
     */
    static bool SaveSensitivityResultsCSV(
        const ParameterSensitivityResult& result,
        const std::string& filename
    );

    /**
     * Print sensitivity results to console
     *
     * @param result Sensitivity analysis results
     */
    static void PrintSensitivityResults(const ParameterSensitivityResult& result);

private:
    // Helper to set a parameter by name
    static void SetParameterByName(
        StructuralAnalyzer::Parameters& params,
        const std::string& name,
        float value
    );

    // Helper to score a parameter configuration
    static float ScoreConfiguration(
        float avg_time_ms,
        float avg_iterations,
        bool converged
    );
};
