#include "Renderer.h"
#include <cstdint>
#include <cstring>
#include <iostream>

#ifdef RENDERING_ENABLED

namespace {
// Cube vertices: positions only (centered at origin, size 1)
constexpr float kCubeVertices[] = {
    -0.5f, -0.5f, -0.5f,
     0.5f, -0.5f, -0.5f,
     0.5f,  0.5f, -0.5f,
    -0.5f,  0.5f, -0.5f,
    -0.5f, -0.5f,  0.5f,
     0.5f, -0.5f,  0.5f,
     0.5f,  0.5f,  0.5f,
    -0.5f,  0.5f,  0.5f
};

// 12 triangles (36 indices) forming the cube
constexpr unsigned int kCubeIndices[] = {
    0, 1, 2,  2, 3, 0,
    4, 5, 6,  6, 7, 4,
    0, 3, 7,  7, 4, 0,
    1, 5, 6,  6, 2, 1,
    0, 1, 5,  5, 4, 0,
    3, 2, 6,  6, 7, 3
};

constexpr GLsizei kCubeIndexCount = static_cast<GLsizei>(sizeof(kCubeIndices) / sizeof(unsigned int));

const char* kVertexShaderSrc = R"(
    #version 330 core
    layout (location = 0) in vec3 aPos;
    layout (location = 1) in mat4 aInstanceMatrix;
    layout (location = 5) in vec3 aInstanceColor;

    uniform mat4 view;
    uniform mat4 projection;

    out vec3 FragPos;
    out vec3 Color;
    out vec3 Normal;

    void main() {
        vec4 worldPos = aInstanceMatrix * vec4(aPos, 1.0);
        FragPos = worldPos.xyz;
        Color = aInstanceColor;
        gl_Position = projection * view * worldPos;

        mat3 normalMat = mat3(transpose(inverse(aInstanceMatrix)));
        Normal = normalize(normalMat * vec3(0.0, 1.0, 0.0));
    }
)";

const char* kFragmentShaderSrc = R"(
    #version 330 core
    in vec3 FragPos;
    in vec3 Color;
    in vec3 Normal;

    out vec4 FragColor;

    uniform vec3 lightPos;
    uniform vec3 viewPos;
    uniform bool enableLighting;

    void main() {
        vec3 finalColor = Color;
        if (enableLighting) {
            vec3 lightColor = vec3(1.0, 1.0, 1.0);
            float ambientStrength = 0.3;
            vec3 ambient = ambientStrength * lightColor;

            vec3 norm = normalize(Normal);
            vec3 lightDir = normalize(lightPos - FragPos);
            float diff = max(dot(norm, lightDir), 0.0);
            vec3 diffuse = diff * lightColor;

            finalColor = (ambient + diffuse) * Color;
        }

        FragColor = vec4(finalColor, 1.0);
    }
)";

void GLFWErrorCallback(int error, const char* description) {
    std::cerr << "[GLFW] Error " << error << ": " << (description ? description : "") << "\n";
}

void FramebufferSizeCallback(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
    if (auto* self = static_cast<Window*>(glfwGetWindowUserPointer(window))) {
        self->OnFramebufferResized(width, height);
    }
}

GLuint CompileShader(GLenum type, const char* source) {
    GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &source, nullptr);
    glCompileShader(shader);

    GLint success = 0;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        GLint length = 0;
        glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &length);
        std::string log(length, '\0');
        glGetShaderInfoLog(shader, length, nullptr, log.data());
        std::cerr << "[VoxelRenderer] Shader compile error: " << log << "\n";
        glDeleteShader(shader);
        return 0;
    }

    return shader;
}

GLuint CreateShaderProgram() {
    GLuint vertex_shader = CompileShader(GL_VERTEX_SHADER, kVertexShaderSrc);
    if (!vertex_shader) {
        return 0;
    }

    GLuint fragment_shader = CompileShader(GL_FRAGMENT_SHADER, kFragmentShaderSrc);
    if (!fragment_shader) {
        glDeleteShader(vertex_shader);
        return 0;
    }

    GLuint program = glCreateProgram();
    glAttachShader(program, vertex_shader);
    glAttachShader(program, fragment_shader);
    glLinkProgram(program);

    GLint success = 0;
    glGetProgramiv(program, GL_LINK_STATUS, &success);
    if (!success) {
        GLint length = 0;
        glGetProgramiv(program, GL_INFO_LOG_LENGTH, &length);
        std::string log(length, '\0');
        glGetProgramInfoLog(program, length, nullptr, log.data());
        std::cerr << "[VoxelRenderer] Program link error: " << log << "\n";
        glDeleteProgram(program);
        program = 0;
    }

    glDeleteShader(vertex_shader);
    glDeleteShader(fragment_shader);

    return program;
}

} // namespace

VoxelRenderer::VoxelRenderer()
    : vao_static(0)
    , vao_dynamic(0)
    , vbo_cube(0)
    , vbo_instance_matrices(0)
    , vbo_instance_colors(0)
    , vbo_dynamic_matrices(0)
    , vbo_dynamic_colors(0)
    , ebo(0)
    , shader_program(0)
    , index_count(0)
    , rendered_voxel_count(0)
    , dynamic_instance_count(0)
    , wireframe_mode(false)
    , lighting_enabled(true)
    , last_frame_time_ms(0.0f)
    , last_render_timestamp(0.0) {
}

VoxelRenderer::~VoxelRenderer() {
    Shutdown();
}

bool VoxelRenderer::Initialize() {
    std::cout << "[VoxelRenderer] Initialize (OpenGL)\n";

    glEnable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE); // Render voxel walls double-sided so they are visible from any direction

    if (!CreateCubeGeometry() || !CreateShaders() || !SetupInstancedRendering()) {
        std::cerr << "[VoxelRenderer] Failed to initialize OpenGL resources\n";
        return false;
    }

    return true;
}

void VoxelRenderer::Shutdown() {
    DestroyResources();
    rendered_voxel_count = 0;
    instance_matrices.clear();
    instance_colors.clear();
    std::cout << "[VoxelRenderer] Shutdown (OpenGL)\n";
}

bool VoxelRenderer::CreateCubeGeometry() {
    glGenVertexArrays(1, &vao_static);
    glBindVertexArray(vao_static);

    glGenBuffers(1, &vbo_cube);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_cube);
    glBufferData(GL_ARRAY_BUFFER, sizeof(kCubeVertices), kCubeVertices, GL_STATIC_DRAW);

    glGenBuffers(1, &ebo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(kCubeIndices), kCubeIndices, GL_STATIC_DRAW);
    index_count = kCubeIndexCount;

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);

    glGenVertexArrays(1, &vao_dynamic);
    glBindVertexArray(vao_dynamic);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_cube);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);

    glBindVertexArray(0);
    return true;
}

bool VoxelRenderer::CreateShaders() {
    shader_program = CreateShaderProgram();
    return shader_program != 0;
}

bool VoxelRenderer::SetupInstancedRendering() {
    if (!vao_static || !vao_dynamic) {
        return false;
    }

    const GLsizei vec4_size = static_cast<GLsizei>(sizeof(float) * 4);
    const GLsizei mat4_size = static_cast<GLsizei>(sizeof(float) * 16);

    glBindVertexArray(vao_static);

    glGenBuffers(1, &vbo_instance_matrices);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_instance_matrices);
    glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);

    for (int i = 0; i < 4; i++) {
        glEnableVertexAttribArray(1 + i);
        glVertexAttribPointer(1 + i, 4, GL_FLOAT, GL_FALSE, mat4_size, (void*)(static_cast<uintptr_t>(vec4_size * i)));
        glVertexAttribDivisor(1 + i, 1);
    }

    glGenBuffers(1, &vbo_instance_colors);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_instance_colors);
    glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(5);
    glVertexAttribPointer(5, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glVertexAttribDivisor(5, 1);

    glBindVertexArray(vao_dynamic);

    glGenBuffers(1, &vbo_dynamic_matrices);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_dynamic_matrices);
    glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);

    for (int i = 0; i < 4; i++) {
        glEnableVertexAttribArray(1 + i);
        glVertexAttribPointer(1 + i, 4, GL_FLOAT, GL_FALSE, mat4_size, (void*)(static_cast<uintptr_t>(vec4_size * i)));
        glVertexAttribDivisor(1 + i, 1);
    }

    glGenBuffers(1, &vbo_dynamic_colors);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_dynamic_colors);
    glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(5);
    glVertexAttribPointer(5, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glVertexAttribDivisor(5, 1);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    return true;
}

void VoxelRenderer::UploadStaticInstances() {
    if (!vao_static) {
        return;
    }

    glBindVertexArray(vao_static);

    glBindBuffer(GL_ARRAY_BUFFER, vbo_instance_matrices);
    glBufferData(GL_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(instance_matrices.size() * sizeof(float)),
                 instance_matrices.empty() ? nullptr : instance_matrices.data(),
                 GL_DYNAMIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, vbo_instance_colors);
    glBufferData(GL_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(instance_colors.size() * sizeof(float)),
                 instance_colors.empty() ? nullptr : instance_colors.data(),
                 GL_DYNAMIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

void VoxelRenderer::UploadDynamicInstances() {
    if (!vao_dynamic) {
        return;
    }

    glBindVertexArray(vao_dynamic);

    glBindBuffer(GL_ARRAY_BUFFER, vbo_dynamic_matrices);
    glBufferData(GL_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(dynamic_matrices.size() * sizeof(float)),
                 dynamic_matrices.empty() ? nullptr : dynamic_matrices.data(),
                 GL_DYNAMIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, vbo_dynamic_colors);
    glBufferData(GL_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(dynamic_colors.size() * sizeof(float)),
                 dynamic_colors.empty() ? nullptr : dynamic_colors.data(),
                 GL_DYNAMIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

void VoxelRenderer::DestroyResources() {
    if (vbo_instance_colors) {
        glDeleteBuffers(1, &vbo_instance_colors);
        vbo_instance_colors = 0;
    }
    if (vbo_dynamic_colors) {
        glDeleteBuffers(1, &vbo_dynamic_colors);
        vbo_dynamic_colors = 0;
    }
    if (vbo_dynamic_matrices) {
        glDeleteBuffers(1, &vbo_dynamic_matrices);
        vbo_dynamic_matrices = 0;
    }
    if (vbo_instance_matrices) {
        glDeleteBuffers(1, &vbo_instance_matrices);
        vbo_instance_matrices = 0;
    }
    if (vbo_cube) {
        glDeleteBuffers(1, &vbo_cube);
        vbo_cube = 0;
    }
    if (ebo) {
        glDeleteBuffers(1, &ebo);
        ebo = 0;
    }
    if (vao_static) {
        glDeleteVertexArrays(1, &vao_static);
        vao_static = 0;
    }
    if (vao_dynamic) {
        glDeleteVertexArrays(1, &vao_dynamic);
        vao_dynamic = 0;
    }
    if (shader_program) {
        glDeleteProgram(shader_program);
        shader_program = 0;
    }
    index_count = 0;
}

void VoxelRenderer::UpdateVoxels(const VoxelWorld& world) {
    auto positions = world.GetAllVoxelPositions();
    std::vector<Color> colors;
    colors.reserve(positions.size());

    for (const auto& pos : positions) {
        Voxel voxel = world.GetVoxel(pos);
        const Material& mat = g_materials.GetMaterial(voxel.material_id);
        colors.push_back(mat.color);
    }

    UpdateVoxels(positions, colors);
}

void VoxelRenderer::UpdateVoxels(const std::vector<Vector3>& positions,
                                 const std::vector<Color>& colors) {
    rendered_voxel_count = static_cast<int>(positions.size());

    instance_matrices.clear();
    instance_colors.clear();

    instance_matrices.reserve(positions.size() * 16);
    instance_colors.reserve(positions.size() * 3);

    float voxel_size = 0.05f;

    for (size_t i = 0; i < positions.size(); i++) {
        const Vector3& pos = positions[i];

        float matrix[16] = {
            voxel_size, 0, 0, 0,
            0, voxel_size, 0, 0,
            0, 0, voxel_size, 0,
            pos.x, pos.y, pos.z, 1
        };

        instance_matrices.insert(instance_matrices.end(), matrix, matrix + 16);

        const Color& color = (i < colors.size()) ? colors[i] : Color::Gray();
        instance_colors.push_back(color.r);
        instance_colors.push_back(color.g);
        instance_colors.push_back(color.b);
    }

    UploadStaticInstances();

    std::cout << "[VoxelRenderer] Updated " << rendered_voxel_count << " voxels (OpenGL)\n";
}

void VoxelRenderer::UpdateDebrisInstances(const std::vector<Vector3>& positions,
                                          const std::vector<Color>& colors,
                                          float voxel_size) {
    dynamic_instance_count = static_cast<int>(positions.size());

    dynamic_matrices.clear();
    dynamic_colors.clear();

    dynamic_matrices.reserve(positions.size() * 16);
    dynamic_colors.reserve(positions.size() * 3);

    for (size_t i = 0; i < positions.size(); i++) {
        const Vector3& pos = positions[i];

        float matrix[16] = {
            voxel_size, 0, 0, 0,
            0, voxel_size, 0, 0,
            0, 0, voxel_size, 0,
            pos.x, pos.y, pos.z, 1
        };

        dynamic_matrices.insert(dynamic_matrices.end(), matrix, matrix + 16);

        const Color& color = (i < colors.size()) ? colors[i] : Color::Gray();
        dynamic_colors.push_back(color.r);
        dynamic_colors.push_back(color.g);
        dynamic_colors.push_back(color.b);
    }

    UploadDynamicInstances();
}

void VoxelRenderer::ClearDebrisInstances() {
    dynamic_instance_count = 0;
    dynamic_matrices.clear();
    dynamic_colors.clear();
    UploadDynamicInstances();
}

void VoxelRenderer::Render(const Camera& camera, float aspect_ratio) {
    if (!shader_program || !vao_static) {
        return;
    }

    glEnable(GL_DEPTH_TEST);
    glClearColor(0.05f, 0.05f, 0.08f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glPolygonMode(GL_FRONT_AND_BACK, wireframe_mode ? GL_LINE : GL_FILL);

    glUseProgram(shader_program);

    float view_matrix[16];
    float proj_matrix[16];
    camera.GetViewMatrix(view_matrix);
    camera.GetProjectionMatrix(proj_matrix, aspect_ratio);

    GLint view_loc = glGetUniformLocation(shader_program, "view");
    GLint proj_loc = glGetUniformLocation(shader_program, "projection");
    GLint enable_lighting_loc = glGetUniformLocation(shader_program, "enableLighting");
    GLint light_pos_loc = glGetUniformLocation(shader_program, "lightPos");
    GLint view_pos_loc = glGetUniformLocation(shader_program, "viewPos");

    glUniformMatrix4fv(view_loc, 1, GL_FALSE, view_matrix);
    glUniformMatrix4fv(proj_loc, 1, GL_FALSE, proj_matrix);
    glUniform1i(enable_lighting_loc, lighting_enabled ? 1 : 0);

    Vector3 light_pos(5.0f, 8.0f, 5.0f);
    glUniform3f(light_pos_loc, light_pos.x, light_pos.y, light_pos.z);

    Vector3 cam_pos = camera.GetPosition();
    glUniform3f(view_pos_loc, cam_pos.x, cam_pos.y, cam_pos.z);

    glBindVertexArray(vao);
    if (rendered_voxel_count > 0) {
        glDrawElementsInstanced(GL_TRIANGLES, index_count, GL_UNSIGNED_INT, nullptr, rendered_voxel_count);
    }
    glBindVertexArray(0);

    double now = glfwGetTime();
    if (last_render_timestamp > 0.0) {
        last_frame_time_ms = static_cast<float>((now - last_render_timestamp) * 1000.0);
    } else {
        last_frame_time_ms = 0.0f;
    }
    last_render_timestamp = now;
}

Window::Window(int width, int height, const char* title_text)
    : window_handle(nullptr)
    , width(width)
    , height(height)
    , delta_time(0.0f)
    , last_frame_time(0.0f)
    , cursor_captured(false)
    , title(title_text ? title_text : "Voxel Destruction")
    , glfw_initialized(false)
    , should_close(false) {
    std::cout << "[Window] Created " << width << "x" << height
              << " '" << this->title << "' (OpenGL)\n";
}

Window::~Window() {
    Shutdown();
}

bool Window::Initialize() {
    std::cout << "[Window] Initialize (GLFW)\n";

    glfwSetErrorCallback(GLFWErrorCallback);

    if (!glfwInit()) {
        std::cerr << "[Window] Failed to initialize GLFW\n";
        return false;
    }
    glfw_initialized = true;
    should_close = false;

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif
    glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_TRUE);

    window_handle = glfwCreateWindow(width, height, title.c_str(), nullptr, nullptr);
    if (!window_handle) {
        std::cerr << "[Window] Failed to create GLFW window\n";
        Shutdown();
        return false;
    }

    glfwMakeContextCurrent(window_handle);
    glfwSwapInterval(1);

    if (!gladLoadGL(glfwGetProcAddress)) {
        std::cerr << "[Window] Failed to initialize GLAD\n";
        Shutdown();
        return false;
    }

    glfwSetWindowUserPointer(window_handle, this);
    glfwSetFramebufferSizeCallback(window_handle, FramebufferSizeCallback);

    int fb_width = 0;
    int fb_height = 0;
    glfwGetFramebufferSize(window_handle, &fb_width, &fb_height);
    glViewport(0, 0, fb_width, fb_height);
    width = fb_width;
    height = fb_height;

    last_frame_time = 0.0f;
    delta_time = 0.0f;

    return true;
}

void Window::Shutdown() {
    if (window_handle) {
        glfwDestroyWindow(window_handle);
        window_handle = nullptr;
    }

    if (glfw_initialized) {
        glfwTerminate();
        glfw_initialized = false;
    }

    should_close = true;
    std::cout << "[Window] Shutdown (OpenGL)\n";
}

bool Window::ShouldClose() const {
    if (!window_handle) {
        return true;
    }

    return should_close || glfwWindowShouldClose(window_handle);
}

void Window::SwapBuffers() {
    if (window_handle) {
        glfwSwapBuffers(window_handle);
    }
}

void Window::PollEvents() {
    if (glfw_initialized) {
        glfwPollEvents();
    }
}

bool Window::IsKeyPressed(int key) const {
    if (!window_handle) {
        return false;
    }
    return glfwGetKey(window_handle, key) == GLFW_PRESS;
}

bool Window::IsMouseButtonPressed(int button) const {
    if (!window_handle) {
        return false;
    }
    return glfwGetMouseButton(window_handle, button) == GLFW_PRESS;
}

void Window::GetCursorPos(double* xpos, double* ypos) const {
    if (!window_handle) {
        if (xpos) *xpos = 0;
        if (ypos) *ypos = 0;
        return;
    }
    glfwGetCursorPos(window_handle, xpos, ypos);
}

void Window::SetCursorMode(bool captured) {
    cursor_captured = captured;
    if (window_handle) {
        glfwSetInputMode(window_handle, GLFW_CURSOR,
                         captured ? GLFW_CURSOR_DISABLED : GLFW_CURSOR_NORMAL);
    }
}

void Window::Close() {
    should_close = true;
    if (window_handle) {
        glfwSetWindowShouldClose(window_handle, GLFW_TRUE);
    }
}

void Window::UpdateDeltaTime() {
    if (!window_handle) {
        delta_time = 0.0f;
        return;
    }

    float current_time = static_cast<float>(glfwGetTime());
    if (last_frame_time == 0.0f) {
        delta_time = 0.0f;
    } else {
        delta_time = current_time - last_frame_time;
    }
    last_frame_time = current_time;
}

void Window::OnFramebufferResized(int new_width, int new_height) {
    width = new_width;
    height = new_height;
}

#else

VoxelRenderer::VoxelRenderer()
    : vao_static(0)
    , vao_dynamic(0)
    , vbo_cube(0)
    , vbo_instance_matrices(0)
    , vbo_instance_colors(0)
    , vbo_dynamic_matrices(0)
    , vbo_dynamic_colors(0)
    , ebo(0)
    , shader_program(0)
    , index_count(0)
    , rendered_voxel_count(0)
    , dynamic_instance_count(0)
    , wireframe_mode(false)
    , lighting_enabled(true)
    , last_frame_time_ms(0.0f)
    , last_render_timestamp(0.0) {
}

VoxelRenderer::~VoxelRenderer() {
    Shutdown();
}

bool VoxelRenderer::Initialize() {
    std::cout << "[VoxelRenderer] Initialize (stub - OpenGL not available)\n";

    CreateCubeGeometry();
    CreateShaders();
    SetupInstancedRendering();

    return true;
}

void VoxelRenderer::Shutdown() {
    std::cout << "[VoxelRenderer] Shutdown (stub)\n";
}

bool VoxelRenderer::CreateCubeGeometry() {
    std::cout << "[VoxelRenderer] Created cube geometry (stub)\n";
    return true;
}

bool VoxelRenderer::CreateShaders() {
    std::cout << "[VoxelRenderer] Compiled shaders (stub)\n";
    return true;
}

bool VoxelRenderer::SetupInstancedRendering() {
    std::cout << "[VoxelRenderer] Set up instanced rendering (stub)\n";
    return true;
}

void VoxelRenderer::UploadStaticInstances() {}
void VoxelRenderer::UploadDynamicInstances() {}
void VoxelRenderer::DestroyResources() {}

void VoxelRenderer::UpdateVoxels(const VoxelWorld& world) {
    auto positions = world.GetAllVoxelPositions();
    std::vector<Color> colors;
    colors.reserve(positions.size());

    for (const auto& pos : positions) {
        Voxel voxel = world.GetVoxel(pos);
        const Material& mat = g_materials.GetMaterial(voxel.material_id);
        colors.push_back(mat.color);
    }

    UpdateVoxels(positions, colors);
}

void VoxelRenderer::UpdateVoxels(const std::vector<Vector3>& positions,
                                 const std::vector<Color>& colors) {
    rendered_voxel_count = static_cast<int>(positions.size());

    instance_matrices.clear();
    instance_colors.clear();

    instance_matrices.reserve(positions.size() * 16);
    instance_colors.reserve(positions.size() * 3);

    float voxel_size = 0.05f;

    for (size_t i = 0; i < positions.size(); i++) {
        const Vector3& pos = positions[i];

        float matrix[16] = {
            voxel_size, 0, 0, 0,
            0, voxel_size, 0, 0,
            0, 0, voxel_size, 0,
            pos.x, pos.y, pos.z, 1
        };

        instance_matrices.insert(instance_matrices.end(), matrix, matrix + 16);

        const Color& color = (i < colors.size()) ? colors[i] : Color::Gray();
        instance_colors.push_back(color.r);
        instance_colors.push_back(color.g);
        instance_colors.push_back(color.b);
    }

    std::cout << "[VoxelRenderer] Updated " << rendered_voxel_count << " voxels (stub)\n";
}

void VoxelRenderer::UpdateDebrisInstances(const std::vector<Vector3>& positions,
                                          const std::vector<Color>& colors,
                                          float voxel_size) {
    (void)positions;
    (void)colors;
    (void)voxel_size;
    dynamic_instance_count = static_cast<int>(positions.size());
}

void VoxelRenderer::ClearDebrisInstances() {
    dynamic_instance_count = 0;
}

void VoxelRenderer::Render(const Camera& camera, float aspect_ratio) {
    float view_matrix[16];
    float proj_matrix[16];
    camera.GetViewMatrix(view_matrix);
    camera.GetProjectionMatrix(proj_matrix, aspect_ratio);

    std::cout << "[VoxelRenderer] Render " << rendered_voxel_count
              << " voxels (stub)\n";

    (void)view_matrix;
    (void)proj_matrix;
    last_frame_time_ms = 16.0f;
}

Window::Window(int width, int height, const char* title_text)
    : window_handle(nullptr)
    , width(width)
    , height(height)
    , delta_time(0)
    , last_frame_time(0)
    , cursor_captured(false)
    , title(title_text ? title_text : "Voxel Destruction")
    , glfw_initialized(false)
    , should_close(false) {
    std::cout << "[Window] Created " << width << "x" << height
              << " '" << this->title << "'\n";
}

Window::~Window() {
    Shutdown();
}

bool Window::Initialize() {
    std::cout << "[Window] Initialize (stub - GLFW not available)\n";
    should_close = false;
    return true;
}

void Window::Shutdown() {
    should_close = true;
    std::cout << "[Window] Shutdown (stub)\n";
}

bool Window::ShouldClose() const {
    return should_close;
}

void Window::SwapBuffers() {
}

void Window::PollEvents() {
}

bool Window::IsKeyPressed(int key) const {
    (void)key;
    return false;
}

bool Window::IsMouseButtonPressed(int button) const {
    (void)button;
    return false;
}

void Window::GetCursorPos(double* xpos, double* ypos) const {
    if (xpos) *xpos = 0;
    if (ypos) *ypos = 0;
}

void Window::SetCursorMode(bool captured) {
    cursor_captured = captured;
}

void Window::Close() {
    should_close = true;
}

void Window::UpdateDeltaTime() {
    delta_time = 0.016f;
}

void Window::OnFramebufferResized(int new_width, int new_height) {
    width = new_width;
    height = new_height;
}

#endif
