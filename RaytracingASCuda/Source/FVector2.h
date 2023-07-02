#pragma once
#include <cuda_runtime.h>

struct FVector2
{
	float x, y;
	__device__ FVector2() = default;
	__device__ FVector2(float _x, float _y) : x(_x), y(_y) {}
};