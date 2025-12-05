#pragma once

#include <chrono>
#include <vector>
#include <map>

// Week 7 Day 31: Turn-Based Integration

/**
 * @brief Turn phase enumeration for turn-based gameplay
 */
enum class TurnPhase {
    PLAYER_ACTION,       // Player moving/shooting/destroying voxels
    STRUCTURAL_ANALYSIS, // Track 1 running (StructuralAnalyzer)
    PHYSICS_SIMULATION,  // Track 3 running (debris falling, settling)
    AI_TURN              // Enemy turn
};

/**
 * @brief Transform snapshot for debris at a specific time
 */
struct DebrisTransform {
    float position[3];   // x, y, z
    float rotation[4];   // quaternion x, y, z, w
    int debris_id;

    DebrisTransform() : debris_id(-1) {
        position[0] = position[1] = position[2] = 0.0f;
        rotation[0] = rotation[1] = rotation[2] = 0.0f;
        rotation[3] = 1.0f;  // w component of identity quaternion
    }
};

/**
 * @brief Keyframe containing transforms of all debris at a specific time
 */
struct TransformKeyframe {
    float time;                                  // Game time in seconds
    std::vector<DebrisTransform> transforms;     // Debris transforms at this time

    TransformKeyframe() : time(0.0f) {}
    TransformKeyframe(float t) : time(t) {}
};

/**
 * @brief Manages keyframe recording for physics simulation playback
 *
 * Records debris transforms during fast-forward simulation, allowing
 * smooth playback at 60 FPS for visual feedback in turn-based games.
 */
class KeyframeRecorder {
private:
    std::vector<TransformKeyframe> keyframes;
    bool recording;
    float start_time;

public:
    KeyframeRecorder();

    /**
     * @brief Start recording keyframes
     */
    void StartRecording();

    /**
     * @brief Stop recording keyframes
     */
    void StopRecording();

    /**
     * @brief Record current frame at specified time
     * @param time Game time in seconds
     * @param transforms Debris transforms to record
     */
    void RecordFrame(float time, const std::vector<DebrisTransform>& transforms);

    /**
     * @brief Clear all recorded keyframes
     */
    void Clear();

    /**
     * @brief Get all recorded keyframes
     */
    const std::vector<TransformKeyframe>& GetKeyframes() const { return keyframes; }

    /**
     * @brief Get keyframe count
     */
    int GetKeyframeCount() const { return static_cast<int>(keyframes.size()); }

    /**
     * @brief Check if currently recording
     */
    bool IsRecording() const { return recording; }
};

/**
 * @brief Manages turn-based gameplay flow with destruction integration
 *
 * Coordinates turn phases:
 * 1. PLAYER_ACTION - Player makes moves
 * 2. STRUCTURAL_ANALYSIS - Analyze damaged structures (Track 1)
 * 3. PHYSICS_SIMULATION - Simulate debris collapse (Track 3)
 * 4. AI_TURN - AI takes its turn
 *
 * Week 7 Day 31: Turn-based timing control
 */
class TurnManager {
private:
    TurnPhase current_phase;
    float phase_timer;

    // Analysis state
    bool structural_analysis_complete;
    bool structure_failed;

    // Physics simulation state
    bool physics_simulation_complete;
    int active_debris_count;

public:
    TurnManager();

    /**
     * @brief Start turn transition (from PLAYER_ACTION to next phase)
     */
    void StartTurnTransition();

    /**
     * @brief Update turn manager state
     * @param deltaTime Time step in seconds
     */
    void Update(float deltaTime);

    /**
     * @brief Get current turn phase
     */
    TurnPhase GetPhase() const { return current_phase; }

    /**
     * @brief Get time elapsed in current phase
     */
    float GetPhaseTimer() const { return phase_timer; }

    /**
     * @brief Set phase directly (for testing)
     */
    void SetPhase(TurnPhase phase);

    /**
     * @brief Notify that structural analysis is complete
     * @param failed Whether structure failed
     */
    void NotifyStructuralAnalysisComplete(bool failed);

    /**
     * @brief Notify that physics simulation is complete
     */
    void NotifyPhysicsSimulationComplete();

    /**
     * @brief Update active debris count for physics phase
     * @param count Number of active debris objects
     */
    void UpdateActiveDebrisCount(int count);

    /**
     * @brief Check if in player's turn
     */
    bool IsPlayerTurn() const { return current_phase == TurnPhase::PLAYER_ACTION; }

    /**
     * @brief Check if waiting for analysis
     */
    bool IsAnalyzing() const { return current_phase == TurnPhase::STRUCTURAL_ANALYSIS; }

    /**
     * @brief Check if simulating physics
     */
    bool IsSimulating() const { return current_phase == TurnPhase::PHYSICS_SIMULATION; }

    /**
     * @brief Check if AI turn
     */
    bool IsAITurn() const { return current_phase == TurnPhase::AI_TURN; }

    /**
     * @brief End AI turn and return control to player
     */
    void EndAITurn();
};
