#pragma once

#include <SDL.h>
#include <SDL_vulkan.h>
#include <stdio.h>	   // printf, fprintf
#include <stdlib.h>	   // abort
#include <vulkan/vulkan.h>
#include "backends/imgui_impl_sdl.h"
#include "backends/imgui_impl_vulkan.h"
#include "imgui.h"

//#define IMGUI_UNLIMITED_FRAME_RATE
#ifdef _DEBUG
#	define IMGUI_VULKAN_DEBUG_REPORT
#endif

namespace debugger
{
class window;

class renderer
{
public:
	renderer(window& window);
	~renderer();

	void update();
	void render(const ImVec4& clear_color);

private:
	void SetupVulkan(const char** extensions, uint32_t extensions_count);
	void SetupVulkanWindow(ImGui_ImplVulkanH_Window* wd, VkSurfaceKHR surface, int width, int height);
	void CleanupVulkan();
	void CleanupVulkanWindow();
	void FrameRender(ImGui_ImplVulkanH_Window* wd, ImDrawData* draw_data);
	void FramePresent(ImGui_ImplVulkanH_Window* wd);
	
private:
	window& window;

	VkAllocationCallbacks* g_Allocator = NULL;
	VkInstance g_Instance = VK_NULL_HANDLE;
	VkPhysicalDevice g_PhysicalDevice = VK_NULL_HANDLE;
	VkDevice g_Device = VK_NULL_HANDLE;
	uint32_t g_QueueFamily = (uint32_t)-1;
	VkQueue g_Queue = VK_NULL_HANDLE;
	VkDebugReportCallbackEXT g_DebugReport = VK_NULL_HANDLE;
	VkPipelineCache g_PipelineCache = VK_NULL_HANDLE;
	VkDescriptorPool g_DescriptorPool = VK_NULL_HANDLE;

	ImGui_ImplVulkanH_Window g_MainWindowData;
	uint32_t g_MinImageCount = 2;
	bool g_SwapChainRebuild = false;
};

}	 // namespace debugger