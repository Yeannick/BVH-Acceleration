
#include <cuda_runtime.h>
#include "vld.h"
#include "SDL.h"
#include "SDL_surface.h"
#undef main

//Standard includes
#include <iostream>

// include framework
#include "Timer.h"
#include "Renderer.h"
#include "SceneGraph.h"
#include "Camera.h"
#include "Utils.h"
#include "BenchMarker.h"
//#include "PerspectiveCamera.h"

Camera* m_Camera = new Camera();
void ShutDown(SDL_Window* pWindow)
{
	SDL_DestroyWindow(pWindow);
	SDL_Quit();
}

void TestScene()
{
	/*TriangleMesh* m_pMesh = new TriangleMesh();

	int count = 0;

	bool test = Utils::ParseObj("Resources/lowpoly_bunny.obj", m_pMesh->triangles , count);
	
	m_pMesh->triangleCount = count;
	m_pMesh->mat = LambertMaterial{ 1.f,ColorRGB{1.0f,1.f,1.f} };
	m_pMesh->mode = TriangleCullMode::BackFaceCulling;*/
	
}


int main(int argc, char* args[])
{
	//Unreferenced parameters
	(void)argc;
	(void)args;
	
	m_Camera->origin = { 7.5f,6.f,-10.f };
	m_Camera->fovAngle = 60.f;

	cudaError_t cudaStatus;
	cudaStatus = cudaSetDevice(0);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaSetDevice failed!  Do you have a CUDA-capable GPU installed?");
	}
	/*cudaError_t stat;
	size_t myStackSize =64*4;
	stat = cudaDeviceSetLimit(cudaLimitStackSize, myStackSize);*/
	//TestScene();
	// Create Window + Surface
	SDL_Init(SDL_INIT_VIDEO);

	const uint32_t width = 1280;
	const uint32_t height = 720;
	SDL_Window* pWindow = SDL_CreateWindow(
		"RayTracing AS - *Yannick Godeau*",
		SDL_WINDOWPOS_UNDEFINED,
		SDL_WINDOWPOS_UNDEFINED,
		width, height, 0);

	if (!pWindow)
		return 1;

	SDL_Renderer* pWindowRenderer = SDL_CreateRenderer(pWindow, -1, SDL_RENDERER_ACCELERATED);;
	// Initialize framework
	Raytracing::Timer* pTimer = new Raytracing::Timer();
	Raytracing::Renderer* pRenderer = new Raytracing::Renderer(pWindow ,pWindowRenderer, m_Camera);
	
	pTimer->Start();
	float printTimer = 0.f;
	bool isLooping = true;
	bool takeScreenshot = false;
	
	while (isLooping)
	{
		//--------- Get input events ---------
		SDL_Event e;
		while (SDL_PollEvent(&e))
		{
			switch (e.type)
			{
			case SDL_QUIT:
				isLooping = false;
				break;
			case SDL_KEYUP:
				if (e.key.keysym.scancode == SDL_SCANCODE_X)
					takeScreenshot = true;
				if (e.key.keysym.scancode == SDL_SCANCODE_Z)
					m_Camera->origin = { 0,3,-9 };
				break;
			default:
				break;
			}
		}
		BenchMarker::GetInstance().StartBenchMark();
		pRenderer->Update(pTimer);
		pRenderer->Render();
		BenchMarker::GetInstance().StopBenchMark();
		m_Camera->Update(pTimer);
		pTimer->Update();
		printTimer += pTimer->GetElapsed();
		if (printTimer >= 1.f)
		{
			printTimer = 0.f;
			std::cout << "FPS: " << pTimer->GetFPS() << std::endl;
			if (pTimer->GetTotal() > 15.f)
			{
				//isLooping = false;
			}
		}
	}
	pTimer->Stop();

	BenchMarker::GetInstance().WriteFile();
	delete pRenderer;
	delete pTimer;
	cudaStatus = cudaDeviceReset();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaDeviceReset failed!");
		return 1;
	}
	ShutDown(pWindow);
	SceneGraph::CleanUp();
	

	return 0;
}