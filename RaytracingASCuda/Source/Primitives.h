#pragma once
#include <cuda.h>
#include <cuda_runtime.h>
#include "Material.h"

extern "C" struct Primitive
{
	Material material;
};
//extern "C" struct Sphere : Primitive
//{
//	float3 position;
//	float radius;
//	Material pMaterial;
//};
extern "C" struct Triangle : Primitive
{
	int3 vertices, normals;
	Material material;
};
extern "C" struct Mesh : Primitive
{
	float3* vertices;
	Triangle* triangles;
	int triangleCount;
	Material material;
};