#pragma once
#include <cuda_runtime.h>
#include <cmath>
#include <cassert>
#include "FVector4.h"
#include <math.h>

struct FVector4;


struct FVector3
{

	float x; 
	float y;
	float z;

	__device__ __host__ FVector3() = default;
	__device__ __host__ FVector3(float r, float g, float b) : x(r) , y(g) , z(b)
	{}
	__device__ __host__ FVector3(const FVector3& from, const FVector3& to) : x(to.x - from.x), y(to.y - from.y), z(to.z - from.z){}
	__device__ __host__ FVector3(const FVector4& v);
	__device__ __host__ FVector3(float v) : x(v), y(v), z(v)
	{
	}
	// -- FUNCTIONS -- //
	
	__device__ __host__ static float Dot(const FVector3& v1, const FVector3& v2)
	{
		return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
	}
	__device__ __host__ static FVector3 Cross(const FVector3& v1, const FVector3& v2)
	{

		return FVector3{
			v1.y * v2.z - v1.z * v2.y,
			v1.z * v2.x - v1.x * v2.z,
			v1.x * v2.y - v1.y * v2.x
		};
	}
	__device__ __host__ float Normalize()
	{
		const float m = Magnitude();
		x /= m;
		y /= m;
		z /= m;

		return m;
	}
	__device__ __host__ float SqrMagnitude() const
	{
		return x * x + y * y + z * z;
	}
	__device__ __host__ FVector3 Normalized() const
	{
		const float m = Magnitude();
		return { x / m, y / m, z / m };
	}
	__device__ __host__ static FVector3 Reflect(const FVector3& v1, const FVector3& v2);
	__device__ __host__ float Magnitude() const
	{
		return sqrtf(x * x + y * y + z * z);
	}
	// -- OPERATOR OVERLOADING -- //
	__device__ __host__ FVector3 operator*(float scale) const
	{
		return { x * scale, y * scale, z * scale };
	}
	__device__ __host__ FVector3 operator/(float scale) const
	{
		return { x / scale, y / scale, z / scale };
	}

	__device__ __host__ FVector3 operator+(const FVector3& v) const
	{
		return { x + v.x, y + v.y, z + v.z };
	}

	__device__ __host__ FVector3 operator-(const FVector3& v) const
	{
		return { x - v.x, y - v.y, z - v.z };
	}
	__device__ __host__ FVector3 operator+( FVector3& v) const
	{
		return { x + v.x, y + v.y, z + v.z };
	}

	__device__ __host__ FVector3 operator-( FVector3& v) const
	{
		return { x - v.x, y - v.y, z - v.z };
	}
	__device__ __host__ FVector3 operator-() const
	{
		return { -x ,-y,-z };
	}
	__device__ __host__ FVector3& operator+=(const FVector3& v)
	{
		x += v.x;
		y += v.y;
		z += v.z;
		return *this;
	}
	__device__ __host__ FVector3& operator-=(const FVector3& v)
	{
		x -= v.x;
		y -= v.y;
		z -= v.z;
		return *this;
		return *this;
	}
	__device__ __host__ FVector3& operator/=(float scale)
	{
		x /= scale;
		y /= scale;
		z /= scale;
		return *this;
	}
	__device__ __host__ FVector3& operator*=(float scale)
	{
		x *= scale;
		y *= scale;
		z *= scale;
		return *this;
	}
	__device__ __host__ float operator[](int index)
	{
		assert(index <= 2 && index >= 0);

		if (index == 0) return x;
		if (index == 1) return y;
		return z;
	}
	__device__ __host__ float operator[](int index) const
	{
		assert(index <= 2 && index >= 0);

		if (index == 0)
		{
			return x;
		}
		if (index == 1)
		{
			return y;
		}
		return z;
		
	}

	__device__ __host__ static FVector3 Max( const FVector3& v1, const FVector3& v2)
	{
		
		return FVector3{
			/*(v1.x > v2.x ? v1.x : v2.x),
			(v1.y > v2.y ? v1.y : v2.y),
			(v1.z > v2.z ? v1.z : v2.z)*/
			fmaxf(v1.x,v2.x),
			fmaxf(v1.y, v2.y),
			fmaxf(v1.z, v2.z)
		};
	}
	__device__ __host__ static FVector3 Min(const FVector3& v1, const FVector3& v2)
	{

		return FVector3{

			/*(v1.x < v2.x ? v1.x : v2.x),
			(v1.y < v2.y ? v1.y : v2.y),
			(v1.z < v2.z ? v1.z : v2.z)*/
			fminf(v1.x,v2.x),
			fminf(v1.y, v2.y),
			fminf(v1.z, v2.z)
		};
	}

	//template <typename T>
	//__global__ void cudaMax(T v1, T v2)
	//{
	//	fmaxf()
	//}
	static const FVector3 UnitX;
	static const FVector3 UnitY;
	static const FVector3 UnitZ;
	static const FVector3 Zero;
};


//Global Operators
__device__ __host__ inline FVector3 operator*(float scale, const FVector3& v)
{
	return { v.x * scale, v.y * scale, v.z * scale };
}