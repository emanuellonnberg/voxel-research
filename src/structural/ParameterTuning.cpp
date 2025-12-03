#include "ParameterTuning.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <limits>

// ============================================================================
// Day 17: Parameter Sensitivity Analysis
// ============================================================================

ParameterSensitivityResult ParameterTuning::AnalyzeParameterSensitivity(
    StructuralAnalyzer& analyzer,
    VoxelWorld& world,
    const std::vector<Vector3>& damaged_positions,
    const std::string& param_name,
    const std::vector<float>& test_values)
{
    ParameterSensitivityResult result;
    result.parameter_name = param_name;
    result.test_values = test_values;
    result.min_time_ms = std::numeric_limits<float>::max();
    result.max_time_ms = 0.0f;
    result.avg_time_ms = 0.0f;
    result.optimal_value = test_values.empty() ? 0.0f : test_values[0];

    std::cout << "\n[ParameterTuning] Analyzing sensitivity of: " << param_name << "\n";
    std::cout << "Testing " << test_values.size() << " values...\n";

    // Save original parameters
    auto original_params = analyzer.params;
    float best_time = std::numeric_limits<float>::max();

    // Test each value
    for (float value : test_values) {
        // Set parameter
        SetParameterByName(analyzer.params, param_name, value);

        // Run analysis
        analyzer.Clear();
        analyzer.ClearMassCache();
        auto analysis_result = analyzer.Analyze(world, damaged_positions);

        // Record results
        result.calculation_times_ms.push_back(analysis_result.calculation_time_ms);
        result.iterations_used.push_back(analysis_result.iterations_used);
        result.converged.push_back(analysis_result.converged);
        result.nodes_analyzed.push_back(analysis_result.nodes_analyzed);
        result.failed_nodes.push_back(analysis_result.num_failed_nodes);

        // Update statistics
        result.min_time_ms = std::min(result.min_time_ms, analysis_result.calculation_time_ms);
        result.max_time_ms = std::max(result.max_time_ms, analysis_result.calculation_time_ms);
        result.avg_time_ms += analysis_result.calculation_time_ms;

        // Track optimal value (fastest that converges)
        if (analysis_result.converged && analysis_result.calculation_time_ms < best_time) {
            best_time = analysis_result.calculation_time_ms;
            result.optimal_value = value;
        }

        std::cout << "  " << param_name << "=" << value
                  << " -> " << analysis_result.calculation_time_ms << "ms, "
                  << analysis_result.iterations_used << " iters, "
                  << (analysis_result.converged ? "converged" : "not converged") << "\n";
    }

    // Calculate average
    if (!test_values.empty()) {
        result.avg_time_ms /= test_values.size();
    }

    // Generate summary
    std::ostringstream summary;
    summary << "Sensitivity analysis for " << param_name << ":\n";
    summary << "  Time range: " << result.min_time_ms << "ms - " << result.max_time_ms << "ms\n";
    summary << "  Average time: " << result.avg_time_ms << "ms\n";
    summary << "  Optimal value: " << result.optimal_value << " (" << best_time << "ms)\n";
    summary << "  Performance impact: " << (result.max_time_ms / result.min_time_ms) << "x variation\n";
    result.analysis_summary = summary.str();

    // Restore original parameters
    analyzer.params = original_params;

    std::cout << "\n" << result.analysis_summary;
    return result;
}

// ============================================================================
// Day 17: Auto-Tuning with Grid Search
// ============================================================================

AutoTuningResult ParameterTuning::AutoTuneParameters(
    VoxelWorld& world,
    const std::vector<Vector3>& damaged_positions,
    int num_iterations)
{
    AutoTuningResult result;
    result.tuning_method = "grid_search";
    result.total_tests_run = 0;

    std::cout << "\n[ParameterTuning] Starting auto-tuning with " << num_iterations << " iterations\n";

    StructuralAnalyzer analyzer;
    float best_score = std::numeric_limits<float>::max();
    auto best_params = analyzer.params;

    // Define parameter ranges to test (simplified grid search)
    std::vector<float> timestep_values = {0.005f, 0.01f, 0.02f};
    std::vector<float> damping_values = {0.85f, 0.9f, 0.95f};
    std::vector<int> max_iter_values = {50, 100, 150};
    std::vector<float> influence_radius_values = {3.0f, 5.0f, 10.0f};

    // Grid search
    for (float timestep : timestep_values) {
        for (float damping : damping_values) {
            for (int max_iter : max_iter_values) {
                for (float influence_radius : influence_radius_values) {
                    // Set parameters
                    analyzer.params.timestep = timestep;
                    analyzer.params.damping = damping;
                    analyzer.params.max_iterations = max_iter;
                    analyzer.params.influence_radius = influence_radius;

                    // Run analysis
                    analyzer.Clear();
                    analyzer.ClearMassCache();
                    auto analysis = analyzer.Analyze(world, damaged_positions);

                    // Score this configuration
                    float score = ScoreConfiguration(
                        analysis.calculation_time_ms,
                        static_cast<float>(analysis.iterations_used),
                        analysis.converged
                    );

                    result.total_tests_run++;

                    // Check if this is better
                    if (score < best_score) {
                        best_score = score;
                        best_params = analyzer.params;
                        result.avg_calculation_time_ms = analysis.calculation_time_ms;
                        result.avg_iterations = static_cast<float>(analysis.iterations_used);

                        std::cout << "  New best: timestep=" << timestep
                                  << ", damping=" << damping
                                  << ", max_iter=" << max_iter
                                  << ", influence_radius=" << influence_radius
                                  << " -> " << analysis.calculation_time_ms << "ms (score: " << score << ")\n";
                    }

                    // Early exit if we've tested enough
                    if (result.total_tests_run >= num_iterations) {
                        goto done;
                    }
                }
            }
        }
    }

done:
    result.optimal_params = best_params;

    // Generate summary
    std::ostringstream summary;
    summary << "Auto-tuning complete:\n";
    summary << "  Tests run: " << result.total_tests_run << "\n";
    summary << "  Best configuration:\n";
    summary << "    timestep=" << result.optimal_params.timestep << "\n";
    summary << "    damping=" << result.optimal_params.damping << "\n";
    summary << "    max_iterations=" << result.optimal_params.max_iterations << "\n";
    summary << "    convergence_threshold=" << result.optimal_params.convergence_threshold << "\n";
    summary << "    influence_radius=" << result.optimal_params.influence_radius << "\n";
    summary << "  Performance: " << result.avg_calculation_time_ms << "ms avg, "
            << result.avg_iterations << " iterations avg\n";
    result.summary = summary.str();

    std::cout << "\n" << result.summary;
    return result;
}

// ============================================================================
// Helper Functions
// ============================================================================

void ParameterTuning::SetParameterByName(
    StructuralAnalyzer::Parameters& params,
    const std::string& name,
    float value)
{
    if (name == "timestep") {
        params.timestep = value;
    } else if (name == "damping") {
        params.damping = value;
    } else if (name == "max_iterations") {
        params.max_iterations = static_cast<int>(value);
    } else if (name == "convergence_threshold") {
        params.convergence_threshold = value;
    } else if (name == "ground_level") {
        params.ground_level = value;
    } else if (name == "influence_radius") {
        params.influence_radius = value;
    } else if (name == "use_surface_only") {
        params.use_surface_only = (value != 0.0f);
    } else if (name == "use_parallel_mass_calc") {
        params.use_parallel_mass_calc = (value != 0.0f);
    } else if (name == "use_early_termination") {
        params.use_early_termination = (value != 0.0f);
    } else {
        std::cerr << "[ParameterTuning] Unknown parameter: " << name << "\n";
    }
}

float ParameterTuning::ScoreConfiguration(
    float avg_time_ms,
    float avg_iterations,
    bool converged)
{
    // Lower score is better
    // Penalize non-convergence heavily
    if (!converged) {
        return 1000000.0f + avg_time_ms;
    }

    // Balance time and iterations (prefer fast convergence)
    return avg_time_ms + (avg_iterations * 0.1f);
}

bool ParameterTuning::SaveSensitivityResultsCSV(
    const ParameterSensitivityResult& result,
    const std::string& filename)
{
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "[ParameterTuning] Failed to open file for writing: " << filename << "\n";
        return false;
    }

    // Write header
    file << "parameter_value,time_ms,iterations,converged,nodes_analyzed,failed_nodes\n";

    // Write data
    for (size_t i = 0; i < result.test_values.size(); i++) {
        file << result.test_values[i] << ","
             << result.calculation_times_ms[i] << ","
             << result.iterations_used[i] << ","
             << (result.converged[i] ? 1 : 0) << ","
             << result.nodes_analyzed[i] << ","
             << result.failed_nodes[i] << "\n";
    }

    file.close();
    std::cout << "[ParameterTuning] Sensitivity results saved to: " << filename << "\n";
    return true;
}

void ParameterTuning::PrintSensitivityResults(const ParameterSensitivityResult& result) {
    std::cout << "\n=== Sensitivity Analysis: " << result.parameter_name << " ===\n";
    std::cout << result.analysis_summary;
    std::cout << "\nDetailed results:\n";
    std::cout << "  Value       | Time (ms) | Iterations | Converged | Nodes | Failed\n";
    std::cout << "  --------------------------------------------------------------------\n";

    for (size_t i = 0; i < result.test_values.size(); i++) {
        printf("  %-11.4f | %-9.2f | %-10d | %-9s | %-5d | %-6d\n",
               result.test_values[i],
               result.calculation_times_ms[i],
               result.iterations_used[i],
               result.converged[i] ? "yes" : "no",
               result.nodes_analyzed[i],
               result.failed_nodes[i]);
    }
    std::cout << "===================================================================\n\n";
}
