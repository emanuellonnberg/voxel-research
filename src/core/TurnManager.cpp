#include "TurnManager.h"

// Week 7 Day 31: Turn-Based Integration

// ============================================================================
// KeyframeRecorder Implementation
// ============================================================================

KeyframeRecorder::KeyframeRecorder()
    : recording(false), start_time(0.0f) {
}

void KeyframeRecorder::StartRecording() {
    recording = true;
    keyframes.clear();
    start_time = 0.0f;
}

void KeyframeRecorder::StopRecording() {
    recording = false;
}

void KeyframeRecorder::RecordFrame(float time, const std::vector<DebrisTransform>& transforms) {
    if (!recording) return;

    TransformKeyframe keyframe(time);
    keyframe.transforms = transforms;
    keyframes.push_back(keyframe);
}

void KeyframeRecorder::Clear() {
    keyframes.clear();
    recording = false;
    start_time = 0.0f;
}

// ============================================================================
// TurnManager Implementation
// ============================================================================

TurnManager::TurnManager()
    : current_phase(TurnPhase::PLAYER_ACTION)
    , phase_timer(0.0f)
    , structural_analysis_complete(false)
    , structure_failed(false)
    , physics_simulation_complete(false)
    , active_debris_count(0)
{
}

void TurnManager::StartTurnTransition() {
    // Player action complete, start structural analysis
    current_phase = TurnPhase::STRUCTURAL_ANALYSIS;
    phase_timer = 0.0f;
    structural_analysis_complete = false;
    structure_failed = false;
}

void TurnManager::Update(float deltaTime) {
    // Check for phase transitions first, before incrementing timer
    switch (current_phase) {
        case TurnPhase::PLAYER_ACTION:
            // Waiting for player to complete their actions
            // StartTurnTransition() is called when player ends turn
            break;

        case TurnPhase::STRUCTURAL_ANALYSIS:
            // Waiting for structural analysis to complete
            if (structural_analysis_complete) {
                if (structure_failed) {
                    // Structure failed, start physics simulation
                    current_phase = TurnPhase::PHYSICS_SIMULATION;
                    physics_simulation_complete = false;
                } else {
                    // Structure stable, skip physics and go to AI turn
                    current_phase = TurnPhase::AI_TURN;
                }
                phase_timer = 0.0f;
                structural_analysis_complete = false;  // Reset for next turn
            }
            break;

        case TurnPhase::PHYSICS_SIMULATION:
            // Waiting for all debris to settle
            if (physics_simulation_complete || active_debris_count == 0) {
                // All debris settled, move to AI turn
                current_phase = TurnPhase::AI_TURN;
                phase_timer = 0.0f;
                physics_simulation_complete = false;  // Reset for next turn
            }
            break;

        case TurnPhase::AI_TURN:
            // AI is taking its turn
            // EndAITurn() is called when AI completes
            break;
    }

    // Increment timer AFTER checking transitions (so new phase starts at 0 + deltaTime)
    phase_timer += deltaTime;
}

void TurnManager::SetPhase(TurnPhase phase) {
    current_phase = phase;
    phase_timer = 0.0f;
}

void TurnManager::NotifyStructuralAnalysisComplete(bool failed) {
    structural_analysis_complete = true;
    structure_failed = failed;
}

void TurnManager::NotifyPhysicsSimulationComplete() {
    physics_simulation_complete = true;
}

void TurnManager::UpdateActiveDebrisCount(int count) {
    active_debris_count = count;
}

void TurnManager::EndAITurn() {
    // AI turn complete, return control to player
    current_phase = TurnPhase::PLAYER_ACTION;
    phase_timer = 0.0f;
}
