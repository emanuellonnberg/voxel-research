# OpenGL Rendering Implementation Plan

## 1. Current State

The rendering path is entirely stubbed out so the demo never touches OpenGL even when the toolchain finds GLFW/GLAD/GLM:

- `VoxelRenderer` and `Window` are implemented as no-ops inside `src/rendering/Renderer.cpp`, only logging actions.
- `include/Renderer.h` defines GPU object handles as plain `unsigned int`/`void*` placeholders.
- `Camera` returns raw float arrays but nothing uploads them to uniforms.
- The build already adds `glad`, `glfw`, `glm`, and defines `RENDERING_ENABLED`, yet the sources do not guard stub vs. real paths.

## 2. Functional Requirements

1. **Context / Platform Layer**
   - Initialize GLFW and create an OpenGL 3.3 core context.
   - Load OpenGL symbols via GLAD.
   - Provide window/input services (queries, callbacks, delta time).

2. **GPU Resource Management**
   - Allocate VAO/VBO/EBO for cube geometry and instanced attributes.
   - Compile and link GLSL shaders with error handling.
   - Stream per-voxel transforms and colors to GPU buffers.
   - Clean up all GL objects on shutdown.

3. **Rendering Loop**
   - Set frame state (clear color/depth, depth test, culling, wireframe toggle).
   - Upload camera matrices and lighting uniforms.
   - Issue instanced draw calls (`glDrawElementsInstanced`) each frame.

4. **Input / Camera Integration**
   - Map GLFW key/mouse events into the existing `Camera` API.
   - Support cursor capture/release and scroll-based FOV zoom.

5. **Fallback**
   - Keep the existing stub for environments without OpenGL by wrapping the real implementation in `#ifdef RENDERING_ENABLED`.

## 3. Implementation Plan

### Phase A – Infrastructure (Window + Loader)
1. Update `include/Renderer.h` to include GLFW/GLAD headers inside `#ifdef RENDERING_ENABLED`, swap placeholder types for `GLFWwindow*`/`GLuint`, and keep stub members in the `#else` branch.
2. In `Window::Initialize`, call `glfwInit`, set the 3.3 core profile hints, create the window, make the context current, load GLAD, configure vsync, and set up input callbacks (key, cursor, scroll, framebuffer resize).
3. Implement `ShouldClose`, `SwapBuffers`, `PollEvents`, `IsKeyPressed`, `GetCursorPos`, `SetCursorMode`, and `UpdateDeltaTime` using GLFW APIs. Track last-frame timestamp via `glfwGetTime`.

### Phase B – Renderer Core
4. Split `src/rendering/Renderer.cpp` into conditional compilation sections: full OpenGL implementation vs. stub fallback.
5. Implement GPU resource creation in `VoxelRenderer::Initialize`:
   - `glGenVertexArrays/Buffers`, upload cube vertices/indices, configure attribute pointers.
   - Build shader program using the existing GLSL strings and add compile/link log reporting.
6. Replace `CreateCubeGeometry`, `CreateShaders`, and `SetupInstancedRendering` with real GL calls (or collapse them into helper methods).
7. In `UpdateVoxels`, upload transform/color data to instance buffers (two VBOs or one interleaved) using `glBufferData` or persistent mapping; update `rendered_voxel_count`.
8. Implement `Render` to:
   - Clear buffers, enable depth testing, set polygon mode based on `wireframe_mode`.
   - Bind shader, set uniforms (view, projection, light position, booleans), bind VAO, and call `glDrawElementsInstanced`.
   - Measure frame time (difference between `glfwGetTime` samples) and store in `last_frame_time_ms`.
9. Implement `Shutdown` to delete VAO/VBO/EBO and shader program.

### Phase C – Input Wiring & Verification
10. Update `src/main.cpp` loop to read actual keyboard/mouse state from `Window`, feed into `Camera::ProcessKeyboard`, `ProcessMouseMovement`, and `ProcessMouseScroll`, and keep the loop running until `Window::ShouldClose()` is true.
11. Add logging/diagnostics guards so the demo clearly reports whether it is running with the OpenGL path or the stub.
12. (Optional) Add small smoke tests under `tests/test_Camera.cpp` gated by `RENDERING_ENABLED` to ensure matrix outputs remain compatible with GL uniforms.

## 4. Proposed Starting Steps

1. **Introduce the `RENDERING_ENABLED` guards** around the renderer/window headers and source sections, keeping the stub as the `#else` branch.
2. **Implement the GLFW window/context setup** end-to-end (Phase A) and verify that the application opens a blank window, processes input, and closes cleanly.
3. **Bring up GL buffers + shaders (Phase B steps 5–6)** to render a single cube with static transforms before wiring instancing and voxel uploads.

Once those steps are stable, continue with the remaining items in the plan to reach full instanced voxel rendering.
