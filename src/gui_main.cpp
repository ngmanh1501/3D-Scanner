// ================================================================
//  gui_main.cpp
//  ----------------------------------------------------------------
//  Entry point MOI cho ban GUI. Khi bien dich file nay, LOAI main.cpp
//  cu ra khoi project (hoac doi ten ham main() trong main.cpp thanh
//  main_console() neu muon giu ca 2 che do) — 1 project executable
//  chi duoc co 1 ham main().
//
//  Yeu cau them vao project (xem DESIGN.md, muc 5):
//    - Dear ImGui (core: imgui.cpp/.h, imgui_draw.cpp, imgui_tables.cpp,
//      imgui_widgets.cpp) + backends/imgui_impl_glfw.cpp/.h +
//      backends/imgui_impl_opengl3.cpp/.h
//      (tai tu https://github.com/ocornut/imgui, nhanh nhat: copy
//      thu muc goc + thu muc "backends" vao project, add vao build)
//    - Link them opengl32.lib (Windows) ben canh glfw3.lib da co san.
// ================================================================

#include "gui_common.h"
#include "gui_panels.h"
#include "app_config.h"

#include <GLFW/glfw3.h>
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include <iostream>
#include <cstdio>

static void glfwErrorCallback(int error, const char* description)
{
    std::fprintf(stderr, "[GLFW] Error %d: %s\n", error, description);
}

int main()
{
    // ---- Redirect std::cout -> GuiLogSink (panel Log) ----
    // TOAN BO code cu (camera_calib.cpp, object_capture.cpp,
    // triangulator.cpp, multipose_triangulator.cpp, pose_estimator.cpp...)
    // van "cout <<" y het truoc gio — chi khac noi no hien ra man hinh.
    static GuiLogSink logSink;
    std::streambuf* oldCoutBuf = std::cout.rdbuf(&logSink);

    // ---- Load config.json (giong main.cpp cu) ----
    static AppConfig cfg;
    AppConfig::load("config.json", cfg);
    cfg.print();

    // ---- GLFW + OpenGL context ----
    glfwSetErrorCallback(glfwErrorCallback);
    if (!glfwInit())
    {
        std::cout.rdbuf(oldCoutBuf);
        return 1;
    }

    const char* glslVersion = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

    GLFWwindow* window = glfwCreateWindow(1280, 800, "3D Scan Studio", nullptr, nullptr);
    if (!window)
    {
        glfwTerminate();
        std::cout.rdbuf(oldCoutBuf);
        return 1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);   // vsync

    // ---- Dear ImGui ----
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    (void)io;
    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glslVersion);

    // ---- State chia se giua cac panel ----
    GuiState state;
    state.cfgPtr = &cfg;
    state.log = &logSink;

    // ================================================================
    //  Main loop
    // ================================================================
    while (!glfwWindowShouldClose(window))
    {
        glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        int fbW, fbH;
        glfwGetFramebufferSize(window, &fbW, &fbH);
        ImGui::SetNextWindowPos(ImVec2(0, 0));
        ImGui::SetNextWindowSize(ImVec2((float)fbW, (float)fbH));

        ImGuiWindowFlags flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
                                  ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse |
                                  ImGuiWindowFlags_NoBringToFrontOnFocus;
        ImGui::Begin("3D Scan Studio", nullptr, flags);

        // Bo cuc: sidebar trai (180px) | panel phai | log duoi cung (180px)
        float logHeight = 180.0f;
        float contentHeight = ImGui::GetContentRegionAvail().y - logHeight - 8.0f;

        ImGui::BeginChild("sidebar", ImVec2(180, contentHeight), true);
        DrawSidebar(state);
        ImGui::EndChild();

        ImGui::SameLine();

        ImGui::BeginChild("panel", ImVec2(0, contentHeight), true);
        switch (state.activePanel)
        {
            case 0: DrawCameraCalibPanel(state);    break;
            case 1: DrawProjectorCalibPanel(state); break;
            case 2: DrawCapturePanel(state);        break;
            case 3: DrawTriPanel(state);            break;
            case 4: DrawTriMultiPanel(state);       break;
            case 5: DrawAutoCalibPanel(state);      break;
            case 6: DrawBuildModelPanel(state);     break;
            case 7: DrawLocatePanel(state);         break;
            case 8: DrawSettingsPanel(state);       break;
        }
        ImGui::EndChild();

        ImGui::Separator();
        ImGui::BeginChild("status_log", ImVec2(0, logHeight - 8.0f), false);
        DrawStatusAndLog(state);
        ImGui::EndChild();

        ImGui::End();

        // ---- Render ----
        ImGui::Render();
        glViewport(0, 0, fbW, fbH);
        glClearColor(0.10f, 0.10f, 0.11f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }

    // ---- Cleanup ----
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();

    std::cout.rdbuf(oldCoutBuf);
    return 0;
}
