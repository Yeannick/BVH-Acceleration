#pragma once
#include <cuda_runtime.h>
#include "FVector3.h"
#include "ColorRGB.h"
#include "PrimitiveTypes.h"

struct Material
{
	__device__ __host__ Material(ColorRGB _color) : color(_color) {}
	ColorRGB color{};
	float roughness{};
	float metalness{};
};

