#include <gtest/gtest.h>
#include "RubbleSystem.h"

// Test fixture
class RubbleSystemTest : public ::testing::Test {
protected:
    void SetUp() override {
        rubble = new RubbleSystem();
    }

    void TearDown() override {
        delete rubble;
    }

    RubbleSystem* rubble;
};

// ===== Basic Rubble Management =====

TEST_F(RubbleSystemTest, Initialize) {
    EXPECT_EQ(rubble->GetRubbleCount(), 0);
}

TEST_F(RubbleSystemTest, AddRubbleSingle) {
    rubble->AddRubble(Vector3(0, 0, 0), 1.5f, MaterialDatabase::CONCRETE);

    EXPECT_EQ(rubble->GetRubbleCount(), 1);
}

TEST_F(RubbleSystemTest, AddRubbleMultiple) {
    rubble->AddRubble(Vector3(0, 0, 0), 1.5f, MaterialDatabase::CONCRETE);
    rubble->AddRubble(Vector3(5, 0, 0), 2.0f, MaterialDatabase::WOOD);
    rubble->AddRubble(Vector3(10, 0, 0), 1.0f, MaterialDatabase::BRICK);

    EXPECT_EQ(rubble->GetRubbleCount(), 3);
}

TEST_F(RubbleSystemTest, ClearAll) {
    rubble->AddRubble(Vector3(0, 0, 0), 1.5f, MaterialDatabase::CONCRETE);
    rubble->AddRubble(Vector3(5, 0, 0), 2.0f, MaterialDatabase::WOOD);

    EXPECT_EQ(rubble->GetRubbleCount(), 2);

    rubble->ClearAll();

    EXPECT_EQ(rubble->GetRubbleCount(), 0);
}

// ===== Rubble Removal =====

TEST_F(RubbleSystemTest, RemoveRubbleNear) {
    rubble->AddRubble(Vector3(0, 0, 0), 1.5f, MaterialDatabase::CONCRETE);
    rubble->AddRubble(Vector3(1, 0, 0), 2.0f, MaterialDatabase::WOOD);
    rubble->AddRubble(Vector3(10, 0, 0), 1.0f, MaterialDatabase::BRICK);

    EXPECT_EQ(rubble->GetRubbleCount(), 3);

    // Remove rubble within 2m of origin
    int removed = rubble->RemoveRubbleNear(Vector3(0, 0, 0), 2.0f);

    EXPECT_EQ(removed, 2);  // First two should be removed
    EXPECT_EQ(rubble->GetRubbleCount(), 1);  // One remains
}

TEST_F(RubbleSystemTest, RemoveRubbleNearNone) {
    rubble->AddRubble(Vector3(0, 0, 0), 1.5f, MaterialDatabase::CONCRETE);

    // Try to remove at faraway position
    int removed = rubble->RemoveRubbleNear(Vector3(100, 0, 0), 1.0f);

    EXPECT_EQ(removed, 0);
    EXPECT_EQ(rubble->GetRubbleCount(), 1);
}

// ===== Cover Height Queries =====

TEST_F(RubbleSystemTest, GetCoverHeightSingle) {
    rubble->AddRubble(Vector3(0, 0, 0), 1.5f, MaterialDatabase::CONCRETE, 1.0f);  // 1m radius

    // Query at rubble position
    float height = rubble->GetCoverHeight(Vector3(0, 0, 0));

    EXPECT_FLOAT_EQ(height, 1.5f);
}

TEST_F(RubbleSystemTest, GetCoverHeightMultiple) {
    rubble->AddRubble(Vector3(0, 0, 0), 1.0f, MaterialDatabase::CONCRETE, 1.0f);
    rubble->AddRubble(Vector3(0.5f, 0, 0), 2.0f, MaterialDatabase::WOOD, 1.0f);

    // Query at position covered by both rubble pieces
    float height = rubble->GetCoverHeight(Vector3(0.3f, 0, 0));

    // Should average the two heights
    EXPECT_FLOAT_EQ(height, 1.5f);  // (1.0 + 2.0) / 2
}

TEST_F(RubbleSystemTest, GetCoverHeightOutside) {
    rubble->AddRubble(Vector3(0, 0, 0), 1.5f, MaterialDatabase::CONCRETE, 1.0f);

    // Query far from rubble
    float height = rubble->GetCoverHeight(Vector3(10, 0, 0));

    EXPECT_FLOAT_EQ(height, 0.0f);
}

TEST_F(RubbleSystemTest, HasCover) {
    rubble->AddRubble(Vector3(0, 0, 0), 1.5f, MaterialDatabase::CONCRETE, 1.0f);

    // Should have cover at rubble position with default threshold (0.5m)
    EXPECT_TRUE(rubble->HasCover(Vector3(0, 0, 0)));
    EXPECT_TRUE(rubble->HasCover(Vector3(0, 0, 0), 1.0f));  // Custom threshold
    EXPECT_FALSE(rubble->HasCover(Vector3(0, 0, 0), 2.0f));  // Too high threshold

    // Should not have cover far away
    EXPECT_FALSE(rubble->HasCover(Vector3(10, 0, 0)));
}

// ===== Nearby Queries =====

TEST_F(RubbleSystemTest, GetRubbleNearby) {
    rubble->AddRubble(Vector3(0, 0, 0), 1.5f, MaterialDatabase::CONCRETE);
    rubble->AddRubble(Vector3(2, 0, 0), 2.0f, MaterialDatabase::WOOD);
    rubble->AddRubble(Vector3(10, 0, 0), 1.0f, MaterialDatabase::BRICK);

    // Query within 3m of origin
    std::vector<RubbleObject> nearby = rubble->GetRubbleNearby(Vector3(0, 0, 0), 3.0f);

    EXPECT_EQ(nearby.size(), 2);  // First two should be within range
}

TEST_F(RubbleSystemTest, GetRubbleNearbyNone) {
    rubble->AddRubble(Vector3(0, 0, 0), 1.5f, MaterialDatabase::CONCRETE);

    // Query far away
    std::vector<RubbleObject> nearby = rubble->GetRubbleNearby(Vector3(100, 0, 0), 1.0f);

    EXPECT_EQ(nearby.size(), 0);
}

// ===== GetAllRubble =====

TEST_F(RubbleSystemTest, GetAllRubble) {
    rubble->AddRubble(Vector3(0, 0, 0), 1.5f, MaterialDatabase::CONCRETE);
    rubble->AddRubble(Vector3(5, 0, 0), 2.0f, MaterialDatabase::WOOD);

    const std::vector<RubbleObject>& all = rubble->GetAllRubble();

    EXPECT_EQ(all.size(), 2);
    EXPECT_FLOAT_EQ(all[0].height, 1.5f);
    EXPECT_FLOAT_EQ(all[1].height, 2.0f);
}
