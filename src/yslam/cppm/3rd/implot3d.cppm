module;
#include "imgui.h"
#include "imgui_impl_sdl3.h"
#include "imgui_impl_sdlrenderer3.h"
#include <SDL3/SDL.h>
#include <cmath>
#include <implot3d.h>
#include <opencv2/opencv.hpp>
#include <stdio.h> // printf, fprintf
#include <stdlib.h> // abort

// #include <implot3d_internal.h>
export module implot3d;
import std;

export namespace ImGui {
using ImGui::Begin;
using ImGui::BeginMenu;
using ImGui::BeginMenuBar;
using ImGui::BeginTabBar;
using ImGui::BeginTabItem;
using ImGui::End;
using ImGui::EndMenu;
using ImGui::EndMenuBar;
using ImGui::EndTabBar;
using ImGui::EndTabItem;
using ImGui::GetTime;
using ImGui::Image;
using ImGui::MenuItem;
using ImGui::Spacing;
using ImGui::Text;
using ImGui::TreeNode;
using ImGui::TreeNodeEx;
using ImGui::TreePop;

void ShowImage(ImTextureID texture_id, int width, int height)
{
    if (texture_id && width > 0 && height > 0) {
        ImGui::Image(texture_id, ImVec2(width, height));
    } else {
        ImGui::Text("No image data available");
    }
}

}
export namespace ImPlot3D {

using ImPlot3D::BeginPlot;
using ImPlot3D::EndPlot;
using ImPlot3D::GetStyle;
using ImPlot3D::PlotLine;
using ImPlot3D::PlotScatter;
using ImPlot3D::SetNextLineStyle;
using ImPlot3D::SetNextMarkerStyle;
using ImPlot3D::SetupAxes;
using ImPlot3D::SetupBoxInitialRotation;
using ImPlot3D::SetupBoxRotation;
using ImPlot3D::SetupBoxScale;
using ImPlot3D::ShowDemoWindow;
using ImPlot3D::ShowStyleEditor;

using ::ImPlot3DLineFlags_;
using ::ImPlot3DMarker_;

void DemoLinePlots()
{
    static float xs1[1001], ys1[1001], zs1[1001];
    for (int i = 0; i < 1001; i++) {
        xs1[i] = i * 0.001f;
        ys1[i] = 0.5f + 0.5f * cosf(50 * (xs1[i] + (float)ImGui::GetTime() / 10));
        zs1[i] = 0.5f + 0.5f * sinf(50 * (xs1[i] + (float)ImGui::GetTime() / 10));
    }
    static double xs2[20], ys2[20], zs2[20];
    for (int i = 0; i < 20; i++) {
        xs2[i] = i * 1 / 19.0f;
        ys2[i] = xs2[i] * xs2[i];
        zs2[i] = xs2[i] * ys2[i];
    }
    if (ImPlot3D::BeginPlot("Line Plots")) {
        ImPlot3D::SetupAxes("x", "y", "z");
        ImPlot3D::PlotLine("f(x)", xs1, ys1, zs1, 1001);
        ImPlot3D::SetNextMarkerStyle(ImPlot3DMarker_Circle);
        ImPlot3D::PlotLine("g(x)", xs2, ys2, zs2, 20, ImPlot3DLineFlags_Segments);
        ImPlot3D::EndPlot();
    }
}
void DemoHeader(const char* label, std::function<void()> demo)
{
    if (ImGui::TreeNodeEx(label)) {
        demo();
        ImGui::TreePop();
    }
}

void show()
{
    static bool show_implot3d_metrics = false;
    static bool show_implot3d_style_editor = false;
    static bool show_imgui_metrics = false;
    static bool show_imgui_style_editor = false;
    static bool show_imgui_demo = false;

    // if (show_implot3d_metrics)
    //     ImPlot3D::ShowMetricsWindow(&show_implot3d_metrics);
    if (show_implot3d_style_editor) {
        ImGui::Begin("Style Editor (ImPlot3D)", &show_implot3d_style_editor);
        ImPlot3D::ShowStyleEditor();
        ImGui::End();
    }
    if (show_imgui_style_editor) {
        ImGui::Begin("Style Editor (ImGui)", &show_imgui_style_editor);
        ImGui::ShowStyleEditor();
        ImGui::End();
    }
    if (show_imgui_metrics)
        ImGui::ShowMetricsWindow(&show_imgui_metrics);
    if (show_imgui_demo)
        ImGui::ShowDemoWindow(&show_imgui_demo);
    ImGui::SetNextWindowPos(ImVec2(100, 100), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(600, 750), ImGuiCond_FirstUseEver);
    static bool implot3d_demo_open = false;
    ImGui::Begin("ImPlot3D Demo", &implot3d_demo_open, ImGuiWindowFlags_MenuBar);
    if (ImGui::BeginMenuBar()) {
        if (ImGui::BeginMenu("Tools")) {
            ImGui::MenuItem("Metrics", nullptr, &show_implot3d_metrics);
            ImGui::MenuItem("Style Editor", nullptr, &show_implot3d_style_editor);
            ImGui::Separator();
            ImGui::MenuItem("ImGui Metrics", nullptr, &show_imgui_metrics);
            ImGui::MenuItem("ImGui Style Editor", nullptr, &show_imgui_style_editor);
            ImGui::MenuItem("ImGui Demo", nullptr, &show_imgui_demo);
            ImGui::EndMenu();
        }
        ImGui::EndMenuBar();
    }

    ImGui::Text("ImPlot3D says olá! (%s)", IMPLOT3D_VERSION);
    ImGui::Spacing();
    if (ImGui::BeginTabBar("ImPlot3DDemoTabs")) {
        if (ImGui::BeginTabItem("Plots")) {
            DemoHeader("Line Plots", DemoLinePlots);
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }
    ImGui::End();
}

SDL_Renderer* renderer;
int Run(std::function<void()> func = show)
{
    // Setup SDL
    // [If using SDL_MAIN_USE_CALLBACKS: all code below until the main loop starts would likely be your SDL_AppInit() function]
    if (!SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMEPAD)) {
        printf("Error: SDL_Init(): %s\n", SDL_GetError());
        return 1;
    }

    // Create window with SDL_Renderer graphics context
    float main_scale = SDL_GetDisplayContentScale(SDL_GetPrimaryDisplay());
    SDL_WindowFlags window_flags = SDL_WINDOW_RESIZABLE | SDL_WINDOW_HIDDEN | SDL_WINDOW_HIGH_PIXEL_DENSITY;
    SDL_Window* window = SDL_CreateWindow("Dear ImGui SDL3+SDL_Renderer example", (int)(1280 * main_scale), (int)(800 * main_scale), window_flags);
    if (window == nullptr) {
        printf("Error: SDL_CreateWindow(): %s\n", SDL_GetError());
        return 1;
    }
    renderer = SDL_CreateRenderer(window, nullptr);
    SDL_SetRenderVSync(renderer, 1);
    if (renderer == nullptr) {
        SDL_Log("Error: SDL_CreateRenderer(): %s\n", SDL_GetError());
        return 1;
    }
    SDL_SetWindowPosition(window, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED);
    SDL_ShowWindow(window);

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot3D::CreateContext();
    // ImPlot3D::SetupBoxRotation();
    // ImPlot3D::SetupBoxScale(float x, float y, float z);

    ImGuiIO& io = ImGui::GetIO();
    (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard; // Enable Keyboard Controls
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad; // Enable Gamepad Controls

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    // ImGui::StyleColorsLight();

    // Setup scaling
    ImGuiStyle& style = ImGui::GetStyle();
    style.ScaleAllSizes(main_scale); // Bake a fixed style scale. (until we have a solution for dynamic style scaling, changing this requires resetting Style + calling this again)
    // style.FontScaleDpi = main_scale; // Set initial font scale. (using io.ConfigDpiScaleFonts=true makes this unnecessary. We leave both here for documentation purpose)

    // Setup Platform/Renderer backends
    ImGui_ImplSDL3_InitForSDLRenderer(window, renderer);
    ImGui_ImplSDLRenderer3_Init(renderer);

    // Load Fonts
    // - If no fonts are loaded, dear imgui will use the default font. You can also load multiple fonts and use ImGui::PushFont()/PopFont() to select them.
    // - AddFontFromFileTTF() will return the ImFont* so you can store it if you need to select the font among multiple.
    // - If the file cannot be loaded, the function will return a nullptr. Please handle those errors in your application (e.g. use an assertion, or display an error and quit).
    // - Use '#define IMGUI_ENABLE_FREETYPE' in your imconfig file to use Freetype for higher quality font rendering.
    // - Read 'docs/FONTS.md' for more instructions and details. If you like the default font but want it to scale better, consider using the 'ProggyVector' from the same author!
    // - Remember that in C/C++ if you want to include a backslash \ in a string literal you need to write a double backslash \\ !
    // - Our Emscripten build process allows embedding fonts to be accessible at runtime from the "fonts/" folder. See Makefile.emscripten for details.
    // style.FontSizeBase = 20.0f;
    // io.Fonts->AddFontDefault();
    // io.Fonts->AddFontFromFileTTF("c:\\Windows\\Fonts\\segoeui.ttf");
    // io.Fonts->AddFontFromFileTTF("../../misc/fonts/DroidSans.ttf");
    // io.Fonts->AddFontFromFileTTF("../../misc/fonts/Roboto-Medium.ttf");
    // io.Fonts->AddFontFromFileTTF("../../misc/fonts/Cousine-Regular.ttf");
    // ImFont* font = io.Fonts->AddFontFromFileTTF("c:\\Windows\\Fonts\\ArialUni.ttf");
    // IM_ASSERT(font != nullptr);

    // Our state
    bool show_demo_window = true;
    bool show_another_window = false;
    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

    // Main loop
    bool done = false;
#ifdef __EMSCRIPTEN__
    // For an Emscripten build we are disabling file-system access, so let's not attempt to do a fopen() of the imgui.ini file.
    // You may manually call LoadIniSettingsFromMemory() to load settings from your own storage.
    io.IniFilename = nullptr;
    EMSCRIPTEN_MAINLOOP_BEGIN
#else
    while (!done)
#endif
    {
        // Poll and handle events (inputs, window resize, etc.)
        // You can read the io.WantCaptureMouse, io.WantCaptureKeyboard flags to tell if dear imgui wants to use your inputs.
        // - When io.WantCaptureMouse is true, do not dispatch mouse input data to your main application, or clear/overwrite your copy of the mouse data.
        // - When io.WantCaptureKeyboard is true, do not dispatch keyboard input data to your main application, or clear/overwrite your copy of the keyboard data.
        // Generally you may always pass all inputs to dear imgui, and hide them from your application based on those two flags.
        // [If using SDL_MAIN_USE_CALLBACKS: call ImGui_ImplSDL3_ProcessEvent() from your SDL_AppEvent() function]
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            ImGui_ImplSDL3_ProcessEvent(&event);
            if (event.type == SDL_EVENT_QUIT)
                done = true;
            if (event.type == SDL_EVENT_WINDOW_CLOSE_REQUESTED && event.window.windowID == SDL_GetWindowID(window))
                done = true;
        }

        // [If using SDL_MAIN_USE_CALLBACKS: all code below would likely be your SDL_AppIterate() function]
        if (SDL_GetWindowFlags(window) & SDL_WINDOW_MINIMIZED) {
            SDL_Delay(10);
            continue;
        }

        // Start the Dear ImGui frame
        ImGui_ImplSDLRenderer3_NewFrame();
        ImGui_ImplSDL3_NewFrame();
        ImGui::NewFrame();

        // Show
        func();

        // Rendering
        ImGui::Render();
        SDL_SetRenderScale(renderer, io.DisplayFramebufferScale.x, io.DisplayFramebufferScale.y);
        SDL_SetRenderDrawColorFloat(renderer, clear_color.x, clear_color.y, clear_color.z, clear_color.w);
        SDL_RenderClear(renderer);
        ImGui_ImplSDLRenderer3_RenderDrawData(ImGui::GetDrawData(), renderer);
        SDL_RenderPresent(renderer);
    }
#ifdef __EMSCRIPTEN__
    EMSCRIPTEN_MAINLOOP_END;
#endif

    // Cleanup
    // [If using SDL_MAIN_USE_CALLBACKS: all code below would likely be your SDL_AppQuit() function]
    ImGui_ImplSDLRenderer3_Shutdown();
    ImGui_ImplSDL3_Shutdown();
    ImPlot3D::DestroyContext();
    ImGui::DestroyContext();

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}

void ShowMat(const cv::Mat& mat)
{
    // 确保输入矩阵不为空
    if (mat.empty())
        return;

    // 转换为适合SDL的格式 (通常是RGBA)
    cv::Mat sdl_mat;
    if (mat.channels() == 1) {
        // 灰度图转换为RGBA
        cv::cvtColor(mat, sdl_mat, cv::COLOR_GRAY2RGBA);
    } else if (mat.channels() == 3) {
        // BGR转换为RGBA
        cv::cvtColor(mat, sdl_mat, cv::COLOR_BGR2RGBA);
    } else {
        // 已经是RGBA格式
        sdl_mat = mat;
    }

    SDL_Surface* surface = SDL_CreateSurfaceFrom(sdl_mat.cols, sdl_mat.rows, SDL_PIXELFORMAT_RGBA32, (void*)sdl_mat.data, sdl_mat.step[0]);
    if (surface == nullptr) {
        fprintf(stderr, "Failed to create SDL surface: %s\n", SDL_GetError());
        return;
    }

    SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer, surface);
    if (texture == nullptr)
        fprintf(stderr, "Failed to create SDL texture: %s\n", SDL_GetError());

    ImGui::Text("pointer = %p", texture);
    ImGui::Text("size = %d x %d", sdl_mat.cols, sdl_mat.rows);
    ImGui::Image((ImTextureID)(intptr_t)texture, ImVec2((float)sdl_mat.cols, (float)sdl_mat.rows));
}

}