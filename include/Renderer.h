#pragma once

#include "VoxelWorld.h"
#include "Material.h"
#include "Camera.h"
#include <vector>
#include <string>

/**
 * VoxelRenderer - OpenGL-based voxel rendering
 *
 * Renders voxels as instanced cubes with per-voxel colors.
 * Uses modern OpenGL (3.3+) with:
 * - Vertex Buffer Objects (VBO)
 * - Vertex Array Objects (VAO)
 * - Instanced rendering (single draw call for all voxels)
 * - Simple Phong lighting
 *
 * Performance target: 60 FPS with 10K voxels
 *
 * Note: This implementation requires OpenGL. In environments without OpenGL,
 * the methods will be no-ops or return dummy values.
 */
class VoxelRenderer {
public:
    VoxelRenderer();
    ~VoxelRenderer();

    // Initialization
    bool Initialize();
    void Shutdown();

    // Update voxels to render
    void UpdateVoxels(const VoxelWorld& world);
    void UpdateVoxels(const std::vector<Vector3>& positions,
                     const std::vector<Color>& colors);

    // Rendering
    void Render(const Camera& camera, float aspect_ratio);

    // Settings
    void SetWireframeMode(bool enabled) { wireframe_mode = enabled; }
    void SetLightingEnabled(bool enabled) { lighting_enabled = enabled; }

    // Stats
    int GetRenderedVoxelCount() const { return rendered_voxel_count; }
    float GetLastFrameTime() const { return last_frame_time_ms; }

private:
    // OpenGL object handles (would be GLuint in real OpenGL)
    unsigned int vao;  // Vertex Array Object
    unsigned int vbo_cube;  // Cube geometry VBO
    unsigned int vbo_instances;  // Instance data VBO
    unsigned int ebo;  // Element Buffer Object (indices)
    unsigned int shader_program;  // Shader program

    // Render data
    std::vector<float> instance_matrices;  // 4x4 matrices for each voxel
    std::vector<float> instance_colors;    // RGB colors for each voxel
    int rendered_voxel_count;

    // Settings
    bool wireframe_mode;
    bool lighting_enabled;
    float last_frame_time_ms;

    // Helper methods
    void CreateCubeGeometry();
    void CreateShaders();
    std::string LoadShaderSource(const char* vertex_src, const char* fragment_src);
    void SetupInstancedRendering();
};

/**
 * Window - GLFW window wrapper
 *
 * Manages window creation, input handling, and OpenGL context.
 */
class Window {
public:
    Window(int width = 1280, int height = 720, const char* title = "Voxel Destruction");
    ~Window();

    bool Initialize();
    void Shutdown();

    bool ShouldClose() const;
    void SwapBuffers();
    void PollEvents();

    // Input
    bool IsKeyPressed(int key) const;
    void GetCursorPos(double* xpos, double* ypos) const;
    void SetCursorMode(bool captured);

    // Getters
    int GetWidth() const { return width; }
    int GetHeight() const { return height; }
    float GetAspectRatio() const { return (float)width / (float)height; }
    float GetDeltaTime() const { return delta_time; }

    void UpdateDeltaTime();

private:
    void* window_handle;  // Would be GLFWwindow* in real implementation
    int width, height;
    float delta_time;
    float last_frame_time;
    bool cursor_captured;
};
