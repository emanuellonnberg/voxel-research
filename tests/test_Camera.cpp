#include <gtest/gtest.h>
#include "Camera.h"
#include <cmath>

class CameraTest : public ::testing::Test {
protected:
    Camera camera;

    void SetUp() override {
        camera = Camera(Vector3(0, 2, 5));
    }
};

TEST_F(CameraTest, DefaultConstruction) {
    Camera cam;
    EXPECT_EQ(cam.GetPosition(), Vector3(0, 2, 5));
    EXPECT_FLOAT_EQ(cam.GetYaw(), -90.0f);
    EXPECT_FLOAT_EQ(cam.GetPitch(), 0.0f);
    EXPECT_FLOAT_EQ(cam.GetFov(), 45.0f);
}

TEST_F(CameraTest, ParameterizedConstruction) {
    Camera cam(Vector3(10, 20, 30));
    EXPECT_EQ(cam.GetPosition(), Vector3(10, 20, 30));
}

TEST_F(CameraTest, InitialForwardDirection) {
    // With yaw = -90 and pitch = 0, camera should point forward
    Vector3 forward = camera.GetForward();

    // Forward vector should be normalized and point in some direction
    EXPECT_NEAR(forward.Length(), 1.0f, 0.01f);

    // With yaw=-90, should point along an axis
    EXPECT_NEAR(forward.y, 0.0f, 0.01f);  // No vertical component with pitch=0
}

TEST_F(CameraTest, KeyboardMovementForward) {
    Vector3 initial_pos = camera.GetPosition();
    float deltaTime = 0.1f;

    // Move forward
    camera.ProcessKeyboard(1.0f, 0.0f, 0.0f, deltaTime);

    Vector3 new_pos = camera.GetPosition();

    // Should have moved in forward direction
    float distance_moved = initial_pos.Distance(new_pos);
    EXPECT_GT(distance_moved, 0.0f);
}

TEST_F(CameraTest, KeyboardMovementRight) {
    Vector3 initial_pos = camera.GetPosition();
    float deltaTime = 0.1f;

    // Move right
    camera.ProcessKeyboard(0.0f, 1.0f, 0.0f, deltaTime);

    Vector3 new_pos = camera.GetPosition();

    // Should have moved
    float distance_moved = initial_pos.Distance(new_pos);
    EXPECT_GT(distance_moved, 0.0f);
}

TEST_F(CameraTest, KeyboardMovementUp) {
    Vector3 initial_pos = camera.GetPosition();
    float deltaTime = 0.1f;

    // Move up
    camera.ProcessKeyboard(0.0f, 0.0f, 1.0f, deltaTime);

    Vector3 new_pos = camera.GetPosition();

    // Y coordinate should have increased
    EXPECT_GT(new_pos.y, initial_pos.y);
}

TEST_F(CameraTest, MouseLookYaw) {
    float initial_yaw = camera.GetYaw();

    // Move mouse right (positive X offset)
    camera.ProcessMouseMovement(10.0f, 0.0f);

    float new_yaw = camera.GetYaw();
    EXPECT_GT(new_yaw, initial_yaw);
}

TEST_F(CameraTest, MouseLookPitch) {
    float initial_pitch = camera.GetPitch();

    // Move mouse up (positive Y offset)
    camera.ProcessMouseMovement(0.0f, 10.0f);

    float new_pitch = camera.GetPitch();
    EXPECT_GT(new_pitch, initial_pitch);
}

TEST_F(CameraTest, PitchClamping) {
    // Try to pitch beyond limits
    camera.ProcessMouseMovement(0.0f, 1000.0f);  // Way up
    EXPECT_LE(camera.GetPitch(), 89.0f);
    EXPECT_GE(camera.GetPitch(), -89.0f);

    camera.ProcessMouseMovement(0.0f, -2000.0f);  // Way down
    EXPECT_LE(camera.GetPitch(), 89.0f);
    EXPECT_GE(camera.GetPitch(), -89.0f);
}

TEST_F(CameraTest, MouseScrollZoom) {
    float initial_fov = camera.GetFov();

    // Scroll up (zoom in) - reduces FOV
    camera.ProcessMouseScroll(5.0f);
    float zoomed_in_fov = camera.GetFov();
    EXPECT_LT(zoomed_in_fov, initial_fov);

    // Scroll down (zoom out) - increases FOV
    camera.ProcessMouseScroll(-10.0f);
    float zoomed_out_fov = camera.GetFov();
    EXPECT_GT(zoomed_out_fov, zoomed_in_fov);
}

TEST_F(CameraTest, FovClamping) {
    // Try to zoom beyond limits
    camera.ProcessMouseScroll(1000.0f);  // Way in
    EXPECT_GE(camera.GetFov(), 1.0f);

    camera.ProcessMouseScroll(-2000.0f);  // Way out
    EXPECT_LE(camera.GetFov(), 45.0f);
}

TEST_F(CameraTest, SetPosition) {
    Vector3 new_pos(100, 200, 300);
    camera.SetPosition(new_pos);
    EXPECT_EQ(camera.GetPosition(), new_pos);
}

TEST_F(CameraTest, SetSpeed) {
    camera.SetSpeed(10.0f);

    Vector3 initial_pos = camera.GetPosition();
    camera.ProcessKeyboard(1.0f, 0.0f, 0.0f, 0.1f);
    Vector3 new_pos = camera.GetPosition();

    float distance = initial_pos.Distance(new_pos);
    EXPECT_NEAR(distance, 1.0f, 0.1f);  // speed=10, dt=0.1 -> 1.0 units
}

TEST_F(CameraTest, SetSensitivity) {
    camera.SetSensitivity(0.5f);

    float initial_yaw = camera.GetYaw();
    camera.ProcessMouseMovement(10.0f, 0.0f);
    float delta_yaw = camera.GetYaw() - initial_yaw;

    EXPECT_NEAR(delta_yaw, 5.0f, 0.1f);  // 10 * 0.5 = 5
}

TEST_F(CameraTest, ViewMatrixFormat) {
    float view_matrix[16];
    camera.GetViewMatrix(view_matrix);

    // Matrix should be 4x4 = 16 elements
    // Last element (index 15) should be 1 (homogeneous coordinate)
    EXPECT_FLOAT_EQ(view_matrix[15], 1.0f);
}

TEST_F(CameraTest, ProjectionMatrixFormat) {
    float proj_matrix[16];
    camera.GetProjectionMatrix(proj_matrix, 16.0f / 9.0f);

    // Matrix should be 4x4 = 16 elements
    // For perspective projection, element [11] should be -1
    EXPECT_FLOAT_EQ(proj_matrix[11], -1.0f);
}

TEST_F(CameraTest, RightVectorPerpendicular) {
    Vector3 forward = camera.GetForward();
    Vector3 right = camera.GetRight();

    // Right should be perpendicular to forward
    float dot = Dot(forward, right);
    EXPECT_NEAR(dot, 0.0f, 0.01f);
}

TEST_F(CameraTest, UpVectorPerpendicular) {
    Vector3 forward = camera.GetForward();
    Vector3 up = camera.GetUp();
    Vector3 right = camera.GetRight();

    // Up should be perpendicular to both forward and right
    EXPECT_NEAR(Dot(forward, up), 0.0f, 0.01f);
    EXPECT_NEAR(Dot(right, up), 0.0f, 0.01f);
}

TEST_F(CameraTest, VectorsNormalized) {
    // All direction vectors should be unit length
    EXPECT_NEAR(camera.GetForward().Length(), 1.0f, 0.01f);
    EXPECT_NEAR(camera.GetRight().Length(), 1.0f, 0.01f);
    EXPECT_NEAR(camera.GetUp().Length(), 1.0f, 0.01f);
}

TEST_F(CameraTest, CombinedMovement) {
    Vector3 initial_pos = camera.GetPosition();

    // Move forward and right simultaneously
    camera.ProcessKeyboard(1.0f, 1.0f, 0.0f, 0.1f);

    Vector3 new_pos = camera.GetPosition();

    // Should have moved diagonally
    float distance = initial_pos.Distance(new_pos);
    EXPECT_GT(distance, 0.0f);
}

TEST_F(CameraTest, NoMovementWithZeroInput) {
    Vector3 initial_pos = camera.GetPosition();

    // No input
    camera.ProcessKeyboard(0.0f, 0.0f, 0.0f, 0.1f);

    Vector3 new_pos = camera.GetPosition();

    // Position should not change
    EXPECT_EQ(initial_pos, new_pos);
}
