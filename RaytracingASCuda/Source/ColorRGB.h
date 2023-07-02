#pragma once
#include "MathHelpers.h"
#include <algorithm>
#include <cuda_runtime.h>


struct ColorRGB
{
	float r{};
	float g{};
	float b{};

	__host__ __device__ ColorRGB() {};
	__host__ __device__ ColorRGB(float _r, float _g, float _b) : r(_r),g(_g),b(_b)
	{}

		__device__ void MaxToOne()
		{
			float maxvalue;
			if (r >= g && r >= b)
				maxvalue = r;

			// check if n2 is the largest number
			else if (g >= r && g >= b)
				maxvalue = g;

			// if neither n1 nor n2 are the largest, n3 is the largest
			else
				maxvalue = b;

			//const float maxValue = std::max(r, std::max(g, b));
			if (maxvalue > 1.f)
				*this /= maxvalue;
		}

		__device__ static ColorRGB Lerp(const ColorRGB& c1, const ColorRGB& c2, float factor)
		{
			return { Lerpf(c1.r, c2.r, factor), Lerpf(c1.g, c2.g, factor), Lerpf(c1.b, c2.b, factor) };
		}

		#pragma region ColorRGB (Member) Operators
		__device__ const ColorRGB& operator+=(const ColorRGB& c)
		{
			r += c.r;
			g += c.g;
			b += c.b;

			return *this;
		}

		__device__ const ColorRGB& operator+(const ColorRGB& c)
		{
			return *this += c;
		}

		__device__ ColorRGB operator+(const ColorRGB& c) const
		{
			return { r + c.r, g + c.g, b + c.b };
		}

		__device__ const ColorRGB& operator-=(const ColorRGB& c)
		{
			r -= c.r;
			g -= c.g;
			b -= c.b;

			return *this;
		}

		__device__ const ColorRGB& operator-(const ColorRGB& c)
		{
			return *this -= c;
		}

		__device__ ColorRGB operator-(const ColorRGB& c) const
		{
			return { r - c.r, g - c.g, b - c.b };
		}

		__device__ const ColorRGB& operator*=(const ColorRGB& c)
		{
			r *= c.r;
			g *= c.g;
			b *= c.b;

			return *this;
		}

		__device__ const ColorRGB& operator*(const ColorRGB& c)
		{
			return *this *= c;
		}

		__device__ ColorRGB operator*(const ColorRGB& c) const
		{
			return { r * c.r, g * c.g, b * c.b };
		}

		__device__ const ColorRGB& operator/=(const ColorRGB& c)
		{
			r /= c.r;
			g /= c.g;
			b /= c.b;

			return *this;
		}

		__device__ const ColorRGB& operator/(const ColorRGB& c)
		{
			return *this /= c;
		}

		__device__ const ColorRGB& operator*=(float s)
		{
			r *= s;
			g *= s;
			b *= s;

			return *this;
		}

		__device__ const ColorRGB& operator*(float s)
		{
			return *this *= s;
		}

		__device__ ColorRGB operator*(float s) const
		{
			return { r * s, g * s,b * s };
		}

		__device__ const ColorRGB& operator/=(float s)
		{
			r /= s;
			g /= s;
			b /= s;

			return *this;
		}

		__device__ const ColorRGB& operator/(float s)
		{
			return *this /= s;
		}
		#pragma endregion
	};

	//ColorRGB (Global) Operators
	__device__ inline ColorRGB operator*(float s, const ColorRGB& c)
	{
		return c * s;
	}

	namespace colors
	{
	  static ColorRGB Red{ 1,0,0 };
	  static ColorRGB Blue{ 0,0,1 };
	  static ColorRGB Green{ 0,1,0 };
	  static ColorRGB Yellow{ 1,1,0 };
	  static ColorRGB Cyan{ 0,1,1 };
	  static ColorRGB Magenta{ 1,0,1 };
	  static ColorRGB White{ 1,1,1 };
	  static ColorRGB Black{ 0,0,0 };
	  static ColorRGB Gray{ 0.5f,0.5f,0.5f };
	}
