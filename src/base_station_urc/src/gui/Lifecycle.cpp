/**
 * @file
 * Implementation of the GUI window setup and rendering (adapted from Dear ImGui's GLFW + OpenGL3 example)
 */
#include "Lifecycle.h"
#include <filesystem>

/// Print GLFW errors to the terminal (e.g. "Failed to open display" when X11 isn't reachable)
static void glfw_error_callback(int error, const char* description)
{
    fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

/**
 * Steps:
 *   1. Start GLFW and choose an OpenGL version for this platform (return nullptr if there's no display)
 *   2. Create the 1280x720 window and turn on vsync (return nullptr if that fails)
 *   3. Create the ImGui context with keyboard navigation, docking, and multi-viewport support
 *   4. Connect ImGui to GLFW (input) and OpenGL (drawing)
 */
GLFWwindow* setupIMGUI()
{
    glfwSetErrorCallback(glfw_error_callback);
    // Returning nullptr instead of calling exit(): exiting here, with ROS threads still running, crashed with a segfault
    if (!glfwInit()) {
        return nullptr;
    }

    // Syntax: #if / #elif / #else are preprocessor checks, so only one branch is compiled, chosen by platform
#if defined(IMGUI_IMPL_OPENGL_ES2)
    // GL ES 2.0 + GLSL 100
    const char* glsl_version = "#version 100";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_ES_API);
#elif defined(__APPLE__)
    // GL 3.2 + GLSL 150
    const char* glsl_version = "#version 150";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);  // Required on Mac
#else
    // GL 3.0 + GLSL 130 (the Linux/Docker case)
    const char* glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
#endif

    GLFWwindow* window = glfwCreateWindow(1280, 720, "LUSI Ground Station", NULL, NULL);
    if (window == NULL) {
        glfwTerminate();
        return nullptr;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);  // Enable vsync (wait for the screen refresh before showing each frame)

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    // Syntax: "ImGuiIO& io" is a reference, another name for the existing settings object (not a copy)
    ImGuiIO& io = ImGui::GetIO();
    (void)io;  // Syntax: casting to void silences "unused variable" warnings
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;  // Enable keyboard controls
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;  // Let panels dock into each other
    io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;  // Let panels be dragged out as separate OS windows

    // Saving the panel layout under /root, which launchScript.sh mounts from ./ground_station_volume on the host
    // That keeps the layout across container restarts and rebuilds
    io.IniFilename = "/root/ui.ini";

    ImGui::StyleColorsDark();

    // When viewports are enabled, square off windows and make them opaque so OS windows look like docked ones
    ImGuiStyle& style = ImGui::GetStyle();
    if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
        style.WindowRounding = 0.0f;
        style.Colors[ImGuiCol_WindowBg].w = 1.0f;
    }

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    // No custom fonts are loaded, so ImGui uses its built-in default font (see libs/imgui/docs/FONTS.md to add one)

    return window;
}

/**
 * Steps:
 *   1. Finish the ImGui frame and clear the window to the background color
 *   2. Draw ImGui's output with OpenGL
 *   3. Draw any panels that were dragged out into their own OS windows
 *   4. Swap buffers to show the finished frame
 */
void renderFrame(GLFWwindow* window, ImVec4 clear_color)
{
    ImGuiIO& io = ImGui::GetIO();
    (void)io;
    ImGui::Render();
    int display_w, display_h;
    glfwGetFramebufferSize(window, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    // Rendering extra OS windows can switch the current OpenGL context, so save and restore it around that
    if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
        GLFWwindow* backup_current_context = glfwGetCurrentContext();
        ImGui::UpdatePlatformWindows();
        ImGui::RenderPlatformWindowsDefault();
        glfwMakeContextCurrent(backup_current_context);
    }

    // OpenGL draws into a hidden back buffer; swapping makes it the visible one
    glfwSwapBuffers(window);
}
