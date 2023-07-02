#pragma once
#include <cuda_runtime.h>
#include "ColorRGB.h"
#include "MathHelpers.h"
#include "PrimitiveTypes.h"


static __device__ ColorRGB Lambert(float kd, const ColorRGB& cd)
{
	return (cd * kd) / PI;
}

static __device__ ColorRGB Lambert(const ColorRGB& kd, const ColorRGB& cd)
{
	return (cd * kd) / PI;
}

/**
 * \brief todo
 * \param ks Specular Reflection Coefficient
 * \param exp Phong Exponent
 * \param l Incoming (incident) Light Direction
 * \param v View Direction
 * \param n Normal of the Surface
 * \return Phong Specular Color
 */
static __device__ ColorRGB Phong(float ks, float exp, const FVector3& l, const FVector3& v, const FVector3& n)
{
	FVector3 r = FVector3::Reflect(n, l);
	const float rDotv = FVector3::Dot(r, v);
	float d = 0;
	if (rDotv > 0.f)
	{
		d = ks * pow(rDotv, exp);
	}

	return ColorRGB{ d, d, d };
}

/**
 * \brief BRDF Fresnel Function >> Schlick
 * \param h Normalized Halfvector between View and Light directions
 * \param v Normalized View direction
 * \param f0 Base reflectivity of a surface based on IOR (Indices Of Refrection), this is different for Dielectrics (Non-Metal) and Conductors (Metal)
 * \return
 */
static __device__ ColorRGB FresnelFunction_Schlick(const FVector3& h, const FVector3& v, const ColorRGB& f0)
{
	const float HdotV = fmaxf(FVector3::Dot(h, v), 0.f);
	return f0 + (ColorRGB(1.f, 1.f, 1.f) - f0) * pow(1.f - HdotV, 5.f);
}

/**
 * \brief BRDF NormalDistribution >> Trowbridge-Reitz GGX (UE4 implemetation - squared(roughness))
 * \param n Surface normal
 * \param h Normalized half vector
 * \param roughness Roughness of the material
 * \return BRDF Normal Distribution Term using Trowbridge-Reitz GGX
 */
static __device__ float NormalDistribution_GGX(const FVector3& n, const FVector3& h, float roughness)
{
	float a = roughness * roughness;
	const float a_sqr = a*a;
	const float NdotH = fmaxf(FVector3::Dot(n, h), 0.f);
	return a_sqr / (PI * Square(Square(NdotH) * (a_sqr - 1.f) + 1.f));
}


/**
 * \brief BRDF Geometry Function >> Schlick GGX (Direct Lighting + UE4 implementation - squared(roughness))
 * \param n Normal of the surface
 * \param v Normalized view direction
 * \param roughness Roughness of the material
 * \return BRDF Geometry Term using SchlickGGX
 */
__device__ static float GeometryFunction_SchlickGGX(const FVector3& n, const FVector3& v, float roughness)
{
	//Direct Lighting + UE4 (squared roughness) Implementation
	float k = Square(roughness + 1) / 8.f; //For direct lighting (div_2 for indirect)
	float NdotV = fmaxf(FVector3::Dot(n, v), 0.f);
	return NdotV / (NdotV * (1.f - k) + k);
}

/**
 * \brief BRDF Geometry Function >> Smith (Direct Lighting)
 * \param n Normal of the surface
 * \param v Normalized view direction
 * \param l Normalized light direction
 * \param roughness Roughness of the material
 * \return BRDF Geometry Term using Smith (> SchlickGGX(n,v,roughness) * SchlickGGX(n,l,roughness))
 */
__device__ static float GeometryFunction_Smith(const FVector3& n, const FVector3& v, const FVector3& l, float roughness)
{
	float ggx1 = GeometryFunction_SchlickGGX(n, v, roughness);
	float ggx2 = GeometryFunction_SchlickGGX(n, l, roughness);
	return ggx1 * ggx2;
}