#pragma once

#include "VoxelWorld.h"
#include "Material.h"
#include "Camera.h"
#include <vector>
#include <string>

#ifdef RENDERING_ENABLED
#include <glad/gl.h>
#include <GLFW/glfw3.h>
#endif

#ifdef RENDERING_ENABLED
using RendererHandle = GLuint;
#else
using RendererHandle = unsigned int;
#endif

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
    void UpdateDebrisInstances(const std::vector<Vector3>& positions,
                               const std::vector<Color>& colors,
                               float voxel_size);
    void ClearDebrisInstances();

    // Settings
    void SetWireframeMode(bool enabled) { wireframe_mode = enabled; }
    void SetLightingEnabled(bool enabled) { lighting_enabled = enabled; }

    // Stats
    int GetRenderedVoxelCount() const { return rendered_voxel_count; }
    float GetLastFrameTime() const { return last_frame_time_ms; }

private:
    // OpenGL object handles
    RendererHandle vao_static;             // Static VAO
    RendererHandle vao_dynamic;            // Dynamic VAO
    RendererHandle vbo_cube;               // Cube geometry VBO
    RendererHandle vbo_instance_matrices;  // Static instance transform VBO
    RendererHandle vbo_instance_colors;    // Static instance color VBO
    RendererHandle vbo_dynamic_matrices;   // Dynamic instance transform VBO
    RendererHandle vbo_dynamic_colors;     // Dynamic instance color VBO
    RendererHandle ebo;                    // Element Buffer Object (indices)
    RendererHandle shader_program;         // Shader program
    int index_count;

    // Render data
    std::vector<float> instance_matrices;  // 4x4 matrices for static voxels
    std::vector<float> instance_colors;    // RGB colors for static voxels
    std::vector<float> dynamic_matrices;   // 4x4 matrices for debris
    std::vector<float> dynamic_colors;     // RGB colors for debris
    int rendered_voxel_count;
    int dynamic_instance_count;

    // Settings
    bool wireframe_mode;
    bool lighting_enabled;
    float last_frame_time_ms;
    double last_render_timestamp;

    // Helper methods
    bool CreateCubeGeometry();
    bool CreateShaders();
    bool SetupInstancedRendering();
    void UploadStaticInstances();
    void UploadDynamicInstances();
    void DestroyResources();
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
    bool IsMouseButtonPressed(int button) const;
    void GetCursorPos(double* xpos, double* ypos) const;
    void SetCursorMode(bool captured);
    void OnFramebufferResized(int new_width, int new_height);
    void Close();
    bool IsCursorCaptured() const { return cursor_captured; }

    // Getters
    int GetWidth() const { return width; }
    int GetHeight() const { return height; }
    float GetAspectRatio() const { return (float)width / (float)height; }
    float GetDeltaTime() const { return delta_time; }

    void UpdateDeltaTime();

private:
#ifdef RENDERING_ENABLED
    GLFWwindow* window_handle;
#else
    void* window_handle;  // Stub placeholder
#endif
    int width, height;
    float delta_time;
    float last_frame_time;
    bool cursor_captured;
    std::string title;
    bool glfw_initialized;
    bool should_close;
};
