#pragma once
#ifndef ELITE_RAYTRACING_RENDERER
#define	ELITE_RAYTRACING_RENDERER

#include <cstdint>
#include <cuda_runtime.h>
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include "PrimitiveTypes.h"
#include "Camera.h"
#include "BenchMarker.h"
#include "ColorRGB.h"
#include "Light.h"
//#include "EMath.h"

struct SDL_Window;
struct SDL_Surface;
#
//enum LightToggle
//{
//	all, irradiance, brdf
//};

enum RenderType
{
	none,
	BoundingVolume,
	BVH
};
namespace Raytracing
{
	class Renderer final
	{
	public:
		 Renderer(SDL_Window* pWindow, SDL_Renderer* pRenderer, Camera* cam);
		
		~Renderer() 
		{ 
		SDL_FreeSurface(m_pFrontBuffer); 
		SDL_DestroyTexture(m_RenderTexture);
		cudaFree(m_MDeviceMeshes);
		cudaFree(m_pCuda2BackBufferPixels);
		free(m_Color2BackBufferPixels);
		//free(m_MHostMeshes->triangles);
		cudaFree(m_MHostMeshes->triangles);
		//free(m_MHostMeshes->triangles);
		//cudaFree(m_TotalInterSectionCountDevice);
		//cudaFree(m_BoxInterSectionCountDevice);
		//cudaFree(m_TriangleInterSectionCountDevice);
		/*free(m_DHostMeshes);
		free(m_MHostMeshes);*/
		};

		Renderer(const Renderer&) = delete;
		Renderer(Renderer&&) noexcept = delete;
		Renderer& operator=(const Renderer&) = delete;
		Renderer& operator=(Renderer&&) noexcept = delete;

		void Render();
		void Update(Timer* pTimer);
		void TraceRayKernelLaunch();
		//bool SaveBackbufferToImage() const;
		//FPoint2 GetScreenSpace(float c, float r, float aspectratio, float fov);
		//void ToggleShadows();
		//void ToggleLights();
		void AllocateCudaMemory();
		void HostToDeviceMemcpy();
		void DeviceToHostMemcpy();

		void CreateSceneObjects();
		void CreateSceneObjectsNone();
		void CreateSceneObjectsBV();
		TriangleMesh* CreateTriangleMesh();
		void CreateFloor(int x, int y);
		void CreateBunny(int x);
		
	private:
		uint32_t m_Width = 0;
		uint32_t m_Height = 0;
		SDL_Window* m_pWindow = nullptr;
		SDL_Surface* m_pFrontBuffer = nullptr;

		SDL_Surface* m_pCudaBackBuffer;
		SDL_Surface* m_pBackBuffer = nullptr;

		uint32_t* m_pBackBufferPixels;

		float3* m_pCudaBackBufferPixels;
		ColorRGB* m_pCuda2BackBufferPixels;
		float3* m_ColorBackBufferPixels;
		ColorRGB* m_Color2BackBufferPixels;

		RenderType m_RenderType = RenderType::BoundingVolume;

		int *m_IntersectionCounter = 0;
		int * m_HostIC= 0;
		/// <OBjects>
		/// 
		/// </OBjects>
		Sphere * m_HostSpheres;
		int m_SphereCount = 2;
		Sphere *m_DevSpheres;

		Light* m_HostLights;
		int m_LightCount = 3;
		Light* m_DeviceLights;

		Plane* m_HostPlanes;
		int m_PlaneCount = 0;
		Plane* m_DevPlanes;

		Triangle* m_HostTriangles;
		int m_TriangleCount = 1;
		Triangle* m_DeviceTriangles;//

		CTriangleMesh* m_HostTriangleMeshes;
		int m_MeshCount = 1;
		CTriangleMesh* m_DeviceMeshes;

		CTriangleMesh* m_MHostMeshes;
		CTriangleMesh* m_DHostMeshes;
		int m_MMeshCount = 1024;
		
		CTriangleMesh* m_MDeviceMeshes;
		
		BVHNode* m_BVHTree;
		BVHNode* m_BVHHost;
		int* m_MeshIdx;
		int* m_MeshIdxHost;


		Camera* m_Camera;
		Camera* m_CudaCamera;

		SDL_Renderer* m_pRenderer = nullptr;
		SDL_Texture* m_RenderTexture = nullptr;
		//	bool ShadowsOn = true;
		//LightToggle m_LightToggle{ LightToggle::all };

		std::vector<TriangleMesh*> m_pMeshes;

		//------------------- TESTING VARIABLES -----------------//
		int* m_TotalInterSectionCountHost;
		int* m_TotalInterSectionCountDevice;

		int* m_TriangleInterSectionCountHost;
		int* m_TriangleInterSectionCountDevice;

		int* m_BoxInterSectionCountHost;
		int* m_BoxInterSectionCountDevice;

		int* m_SphereInterSectionCountHost;
		int* m_SphereInterSectionCountDevice;

		int* m_FalsePositiveInterSectionCountHost;
		int* m_FalsePositiveInterSectionCountDevice;

		int* m_FalsePositiveBVHInterSectionCountHost;
		int* m_FalsePositiveBVHInterSectionCountDevice;

		float* m_TriangleIntersectionTimeHost;
		float* m_TriangleIntersectionTimeDevice;

		float* m_BoxIntersectionTimeHost;
		float* m_BoxIntersectionTimeDevice;

		float* m_SphereIntersectionTimeHost;
		float* m_SphereIntersectionTimeDevice;

		float* m_BVHBuildTimeHost;
		float* m_BVHBuildTimeDevice;

		float* m_BVHTraversalTimeHost;
		float* m_BVHTraversalTimeDevice;
	};
}

#endif
