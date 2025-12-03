#pragma once

#include "Vector3.h"

/**
 * Camera - First-person camera for 3D navigation
 *
 * Supports:
 * - WASD movement (forward, back, left, right)
 * - Mouse look (yaw and pitch)
 * - Configurable speed and sensitivity
 * - View and projection matrix generation
 *
 * Used for navigating the voxel world.
 */
class Camera {
public:
    Camera(const Vector3& position = Vector3(0, 2, 5));

    // Input handling
    void ProcessKeyboard(float forward, float right, float up, float deltaTime);
    void ProcessMouseMovement(float xoffset, float yoffset);
    void ProcessMouseScroll(float yoffset);

    // Matrix getters (these would return glm::mat4 in actual OpenGL code)
    void GetViewMatrix(float* out_matrix) const;
    void GetProjectionMatrix(float* out_matrix, float aspect_ratio) const;

    // Accessors
    Vector3 GetPosition() const { return position; }
    Vector3 GetForward() const { return forward; }
    Vector3 GetRight() const { return right; }
    Vector3 GetUp() const { return up; }

    float GetYaw() const { return yaw; }
    float GetPitch() const { return pitch; }
    float GetFov() const { return fov; }

    // Settings
    void SetPosition(const Vector3& pos) { position = pos; UpdateVectors(); }
    void SetSpeed(float speed) { movement_speed = speed; }
    void SetSensitivity(float sensitivity) { mouse_sensitivity = sensitivity; }

private:
    // Camera attributes
    Vector3 position;
    Vector3 forward;
    Vector3 right;
    Vector3 up;
    Vector3 world_up;

    // Euler angles (degrees)
    float yaw;    // Rotation around Y axis
    float pitch;  // Rotation around X axis

    // Camera options
    float movement_speed;
    float mouse_sensitivity;
    float fov;  // Field of view

    // Update camera vectors from Euler angles
    void UpdateVectors();
};
