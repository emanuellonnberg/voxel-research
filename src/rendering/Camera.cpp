#include "Camera.h"
#include <cmath>
#include <algorithm>

Camera::Camera(const Vector3& position)
    : position(position)
    , world_up(0, 1, 0)
    , yaw(-90.0f)      // Point forward (negative Z) initially
    , pitch(0.0f)
    , movement_speed(5.0f)
    , mouse_sensitivity(0.1f)
    , fov(45.0f)
{
    UpdateVectors();
}

void Camera::ProcessKeyboard(float forward_input, float right_input, float up_input, float deltaTime) {
    float velocity = movement_speed * deltaTime;

    // Forward/backward (W/S)
    position += forward * forward_input * velocity;

    // Left/right (A/D)
    position += right * right_input * velocity;

    // Up/down (Space/Shift)
    position += world_up * up_input * velocity;
}

void Camera::ProcessMouseMovement(float xoffset, float yoffset) {
    xoffset *= mouse_sensitivity;
    yoffset *= mouse_sensitivity;

    yaw += xoffset;
    pitch += yoffset;

    // Constrain pitch to prevent gimbal lock
    pitch = std::max(-89.0f, std::min(89.0f, pitch));

    UpdateVectors();
}

void Camera::ProcessMouseScroll(float yoffset) {
    fov -= yoffset;
    fov = std::max(1.0f, std::min(45.0f, fov));
}

void Camera::UpdateVectors() {
    // Calculate new forward vector from Euler angles
    float yaw_rad = yaw * 3.14159265f / 180.0f;
    float pitch_rad = pitch * 3.14159265f / 180.0f;

    Vector3 new_forward;
    new_forward.x = std::cos(yaw_rad) * std::cos(pitch_rad);
    new_forward.y = std::sin(pitch_rad);
    new_forward.z = std::sin(yaw_rad) * std::cos(pitch_rad);
    forward = new_forward.Normalized();

    // Calculate right and up vectors
    right = Cross(forward, world_up).Normalized();
    up = Cross(right, forward).Normalized();
}

void Camera::GetViewMatrix(float* out_matrix) const {
    // Calculate look-at matrix
    // This is a placeholder - in real OpenGL code, this would use glm::lookAt
    // For now, we'll just document the structure

    Vector3 target = position + forward;
    Vector3 z_axis = (position - target).Normalized();  // Camera Z points backward
    Vector3 x_axis = Cross(world_up, z_axis).Normalized();
    Vector3 y_axis = Cross(z_axis, x_axis);

    // Column-major 4x4 matrix
    out_matrix[0] = x_axis.x;
    out_matrix[1] = y_axis.x;
    out_matrix[2] = z_axis.x;
    out_matrix[3] = 0;

    out_matrix[4] = x_axis.y;
    out_matrix[5] = y_axis.y;
    out_matrix[6] = z_axis.y;
    out_matrix[7] = 0;

    out_matrix[8] = x_axis.z;
    out_matrix[9] = y_axis.z;
    out_matrix[10] = z_axis.z;
    out_matrix[11] = 0;

    out_matrix[12] = -Dot(x_axis, position);
    out_matrix[13] = -Dot(y_axis, position);
    out_matrix[14] = -Dot(z_axis, position);
    out_matrix[15] = 1;
}

void Camera::GetProjectionMatrix(float* out_matrix, float aspect_ratio) const {
    // Perspective projection matrix
    // This is a placeholder - in real OpenGL code, this would use glm::perspective

    float near = 0.1f;
    float far = 100.0f;
    float fov_rad = fov * 3.14159265f / 180.0f;
    float tan_half_fov = std::tan(fov_rad / 2.0f);

    // Clear matrix
    for (int i = 0; i < 16; i++) out_matrix[i] = 0;

    out_matrix[0] = 1.0f / (aspect_ratio * tan_half_fov);
    out_matrix[5] = 1.0f / tan_half_fov;
    out_matrix[10] = -(far + near) / (far - near);
    out_matrix[11] = -1.0f;
    out_matrix[14] = -(2.0f * far * near) / (far - near);
}
