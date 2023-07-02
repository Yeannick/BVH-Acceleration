#pragma once
#include <cuda_runtime.h>
#include "FVector3.h"
#include "FVector4.h"

struct FMatrix
{
	__device__ __host__ FMatrix() = default;

	__device__ __host__ FMatrix(
		const FVector3& xAxis,
		const FVector3& yAxis,
		const FVector3& zAxis,
		const FVector3& t):
		FMatrix(
			FVector4{ xAxis,0 },
			FVector4{ yAxis,0 },
			FVector4{ zAxis,0 },
			FVector4{ t,0 }
		)
	{
	}

	__device__ __host__ FMatrix(
		const FVector4& xAxis,
		const FVector4& yAxis,
		const FVector4& zAxis,
		const FVector4& t)
	{
		data[0] = xAxis;
		data[1] = yAxis;
		data[2] = zAxis;
		data[3] = t;
	}

	/*__device__ __host__ FMatrix(const FMatrix& m)
	{
		data[0] = m[0];
		data[1] = m[1];
		data[2] = m[2];
		data[3] = m[3];
	}*/
	/*__device__ __host__ FMatrix(FMatrix& m)
	{
		data[0] = m[0];
		data[1] = m[1];
		data[2] = m[2];
		data[3] = m[3];
	}*/

	__device__ __host__ FVector3 TransformVector(const FVector3& v) const
	{
		return TransformVector(v[0], v[1], v[2]);
	}
	__device__ __host__ FVector3 TransformVector(float x, float y, float z) const
	{
		return FVector3{
			data[0].x * x + data[1].x * y + data[2].x * z,
			data[0].y * x + data[1].y * y + data[2].y * z,
			data[0].z * x + data[1].z * y + data[2].z * z
		};
	}
	__device__ __host__ FVector3 TransformPoint(const FVector3& p) const
	{
		return TransformPoint(p[0], p[1], p[2]);
	}
	__device__ __host__ FVector3 TransformPoint(float x, float y, float z) const
	{
		return FVector3{
			data[0].x * x + data[1].x * y + data[2].x * z + data[3].x,
			data[0].y * x + data[1].y * y + data[2].y * z + data[3].y,
			data[0].z * x + data[1].z * y + data[2].z * z + data[3].z,
		};
	}
	__device__ __host__ const FMatrix& Transpose()
	{
		FMatrix result{};
		for (int r{ 0 }; r < 4; ++r)
		{
			for (int c{ 0 }; c < 4; ++c)
			{
				result[r][c] = data[c][r];
			}
		}

		data[0] = result[0];
		data[1] = result[1];
		data[2] = result[2];
		data[3] = result[3];

		return *this;
	}
	__device__ __host__ static FMatrix Transpose(const FMatrix& m)
	{
		FMatrix out{ m };
		out.Transpose();

		return out;
	}

	__device__ __host__ static FMatrix CreateTranslation(float x, float y, float z);
	__device__ __host__ static FMatrix CreateTranslation(const FVector3& t)
	{
		return { FVector3{ 1, 0, 0 },  FVector3{ 0, 1, 0 },  FVector3{ 0, 0, 1 }, t };
	}
	__device__ __host__ static FMatrix CreateRotationX(float pitch)
	{
		return {
			{1, 0, 0, 0},
			{0, cos(pitch), -sin(pitch), 0},
			{0, sin(pitch), cos(pitch), 0},
			{0, 0, 0, 1}
		} ;
	}
	__device__ __host__ static FMatrix CreateRotationY(float yaw)
	{
		return {
			{cos(yaw), 0, -sin(yaw), 0},
			{0, 1, 0, 0},
			{sin(yaw), 0, cos(yaw), 0},
			{0, 0, 0, 1}
		};
	}
	__device__ __host__ static FMatrix CreateRotationZ(float roll)
	{
		return {
			{cos(roll), sin(roll), 0, 0},
			{-sin(roll), cos(roll), 0, 0},
			{0, 0, 1, 0},
			{0, 0, 0, 1}
		};
	}
	__device__ __host__ static FMatrix CreateRotation(float pitch, float yaw, float roll)
	{
		return CreateRotation({ pitch, yaw, roll });
	}
	__device__ __host__ static FMatrix CreateRotation(const FVector3& r)
	{
		return CreateRotationX(r[0]) * CreateRotationY(r[1]) * CreateRotationZ(r[2]);

	}
	__device__ __host__ static FMatrix CreateScale(float sx, float sy, float sz)
	{
		return { {sx, 0, 0}, {0, sy, 0}, {0, 0, sz}, {0,0,0} };
	}
	__device__ __host__ static FMatrix CreateScale(const FVector3& s)
	{
		return CreateScale(s[0], s[1], s[2]);
	}

	__device__ __host__ FVector3 GetAxisX() const { return data[0]; }
	__device__ __host__ FVector3 GetAxisY() const { return data[1]; }
	__device__ __host__ FVector3 GetAxisZ() const { return data[2]; }
	__device__ __host__ FVector3 GetTranslation() const { return data[3]; }

#pragma region Operator Overloads
	FVector4& operator[](int index)
	{
		assert(index <= 3 && index >= 0);
		return data[index];
	}
	FVector4 operator[](int index) const
	{
		assert(index <= 3 && index >= 0);
		return data[index];
	}
	FMatrix operator*(const FMatrix& m) const
	{
		FMatrix result{};
		FMatrix m_transposed = Transpose(m);

		for (int r{ 0 }; r < 4; ++r)
		{
			for (int c{ 0 }; c < 4; ++c)
			{
				result[r][c] = FVector4::Dot(data[r], m_transposed[c]);
			}
		}

		return result;
	}
	const FMatrix& operator*=(const FMatrix& m)
	{
		FMatrix copy{ m };
		FMatrix m_transposed = Transpose(*this);

		for (int r{ 0 }; r < 4; ++r)
		{
			for (int c{ 0 }; c < 4; ++c)
			{
				data[r][c] = FVector4::Dot(copy[r], m_transposed[c]);
			}
		}

		return *this;
	}
#pragma endregion
private:

	FVector4 data[4]
	{
		{1,0,0,0}, //xAxis
		{0,1,0,0}, //yAxis
		{0,0,1,0}, //zAxis
		{0,0,0,1}  //T
	};
};