#include "RubbleSystem.h"
#include <iostream>
#include <cmath>
#include <algorithm>

RubbleSystem::RubbleSystem() {
    std::cout << "[RubbleSystem] Initialized\n";
}

// ===== Rubble Management =====

void RubbleSystem::AddRubble(const Vector3& position, float height, uint8_t material_id, float radius) {
    RubbleObject rubble(position, height, radius, material_id);
    rubble_pieces.push_back(rubble);

    std::cout << "[RubbleSystem] Added rubble at ("
              << position.x << ", " << position.y << ", " << position.z
              << ") height=" << height << "m\n";
}

int RubbleSystem::RemoveRubbleNear(const Vector3& position, float removal_radius) {
    int removed_count = 0;

    auto it = rubble_pieces.begin();
    while (it != rubble_pieces.end()) {
        float dist = (it->position - position).Length();

        if (dist <= removal_radius) {
            it = rubble_pieces.erase(it);
            removed_count++;
        } else {
            ++it;
        }
    }

    if (removed_count > 0) {
        std::cout << "[RubbleSystem] Removed " << removed_count
                  << " rubble pieces near (" << position.x << ", " << position.y << ", " << position.z << ")\n";
    }

    return removed_count;
}

void RubbleSystem::ClearAll() {
    int count = rubble_pieces.size();
    rubble_pieces.clear();

    if (count > 0) {
        std::cout << "[RubbleSystem] Cleared " << count << " rubble pieces\n";
    }
}

// ===== Queries =====

std::vector<RubbleObject> RubbleSystem::GetRubbleNearby(const Vector3& position, float radius) const {
    std::vector<RubbleObject> nearby;

    for (const auto& rubble : rubble_pieces) {
        float dist = (rubble.position - position).Length();

        if (dist <= radius) {
            nearby.push_back(rubble);
        }
    }

    return nearby;
}

float RubbleSystem::GetCoverHeight(const Vector3& position) const {
    float total_height = 0.0f;
    int count = 0;

    for (const auto& rubble : rubble_pieces) {
        // Use 2D distance (ignore height difference)
        float dist = Distance2D(rubble.position, position);

        // Check if position is within rubble's influence radius
        if (dist <= rubble.radius) {
            total_height += rubble.height;
            count++;
        }
    }

    if (count == 0) {
        return 0.0f;
    }

    return total_height / count;
}

bool RubbleSystem::HasCover(const Vector3& position, float min_height) const {
    float cover_height = GetCoverHeight(position);
    return cover_height >= min_height;
}

int RubbleSystem::GetRubbleCount() const {
    return static_cast<int>(rubble_pieces.size());
}

const std::vector<RubbleObject>& RubbleSystem::GetAllRubble() const {
    return rubble_pieces;
}

// ===== Helper Functions =====

float RubbleSystem::Distance2D(const Vector3& a, const Vector3& b) const {
    float dx = a.x - b.x;
    float dz = a.z - b.z;
    return std::sqrt(dx * dx + dz * dz);
}
