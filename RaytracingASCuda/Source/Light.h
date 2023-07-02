#pragma once
#include "FVector3.h"
#include "ColorRGB.h"

__device__ enum class LightType {point , Directional};

struct Light
{
	FVector3 origin;
	FVector3 direction;
	ColorRGB color;
	float intensity;
	LightType type;


};