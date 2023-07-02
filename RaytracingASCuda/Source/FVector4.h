#pragma once

#include <cuda_runtime.h>

#include <cmath>
#include <cassert>
#include "FVector3.h"


struct FVector3;


struct FVector4
{
	float x;
	float y;
	float z;
	float w;

	__device__ __host__ FVector4() = default;
	__device__ __host__ FVector4(float _x, float _y, float _z, float _w) :
		x(_x),
		y(_y),
		z(_z),
		w(_w) {}
	__device__ __host__ FVector4(const FVector3& v, float _w);

	__device__ __host__ float Magnitude() const
	{
		return sqrtf(x * x + y * y + z * z + w * w);
	}
	__device__ __host__ float SqrMagnitude() const
	{
		return x * x + y * y + z * z + w * w;
	}
	__device__ __host__ float Normalize()
	{
		const float m = Magnitude();
		x /= m;
		y /= m;
		z /= m;
		w /= m;

		return m;
	}
	__device__ __host__ FVector4 Normalized() const
	{
		const float m = Magnitude();
		return { x / m, y / m, z / m, w / m };
	}

	__device__ __host__ static float Dot(const FVector4& v1, const FVector4& v2)
	{
		return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z + v1.w * v2.w;
	}

	// operator overloading
	__device__ __host__ FVector4 operator*(float scale) const
	{
		return { x * scale, y * scale, z * scale, w * scale };
	}
	__device__ __host__ FVector4 operator+(const FVector4& v) const
	{
		return { x + v.x, y + v.y, z + v.z, w + v.w };
	}
	__device__ __host__ FVector4 operator-(const FVector4& v) const
	{
		return { x - v.x, y - v.y, z - v.z, w - v.w };
	}
	__device__ __host__ FVector4& operator+=(const FVector4& v)
	{
		x += v.x;
		y += v.y;
		z += v.z;
		w += v.w;
		return *this;
	}
	__device__ __host__ float& operator[](int index)
	{
		assert(index <= 3 && index >= 0);

		if (index == 0)return x;
		if (index == 1)return y;
		if (index == 2)return z;
		return w;
	}
	__device__ __host__ float operator[](int index) const
	{
		assert(index <= 3 && index >= 0);

		if (index == 0)return x;
		if (index == 1)return y;
		if (index == 2)return z;
		return w;
	}

};