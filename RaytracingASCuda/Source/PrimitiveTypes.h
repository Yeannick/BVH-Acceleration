#pragma once
#include <cuda_runtime.h>
#include "FVector3.h"
#include <cfloat>
#include "ColorRGB.h"
#include <iostream>
#include <vector>
#include "FMatrix.h"
#include "Material.h"

struct LambertMaterial
{
	__device__ __host__ LambertMaterial() = default;
	__device__ __host__ LambertMaterial(float reflectance, ColorRGB color) :diffuseReflectance(reflectance), diffuseColor(color) {};
	float diffuseReflectance = 1.f;
	ColorRGB diffuseColor{};

	//__device__ ColorRGB Lambert(const float DiffuseReflectance, const ColorRGB diffuseColor);
	//__device__ ColorRGB Shade();
};
struct PBRMaterial
{
	__device__ __host__ PBRMaterial() = default;
	__device__ __host__ PBRMaterial(ColorRGB color, float rougness, float metalness) : m_Albedo(color), m_Metalness(metalness), m_Roughness(rougness)
	{
	}
	ColorRGB m_Albedo{ 0.955f, 0.637f, 0.538f }; //Copper
	float m_Metalness{ 1.0f };
	float m_Roughness{ 0.1f };
};
struct Ray
{
	FVector3 origin;
	FVector3 direction;

	float min{ 0.0001f };
	float max{ FLT_MAX };
};
struct HitRecord
{
	FVector3 origin{};
	FVector3 normal{};
	float t = FLT_MAX;

	PBRMaterial mat{};
	bool didHit{ false };
};

struct Sphere
{
	FVector3 origin;
	float radius;
	LambertMaterial mat;
};
struct Plane
{
	FVector3 origin{};
	FVector3 normal{};
	PBRMaterial mat;
};
__device__ __host__ enum class TriangleCullMode
{
	FrontFaceCulling = 0,
	BackFaceCulling = 1,
	NoCulling = 2
};
struct Triangle
{
	Triangle() = default;
	Triangle(const FVector3& _v0, const FVector3& _v1, const FVector3& _v2, const FVector3& _normal) :
		v0(_v0),
		v1(_v1),
		v2(_v2),
		normal(_normal)
	{}
	Triangle(const FVector3& _v0, const FVector3& _v1, const FVector3& _v2) :
		v0(_v0),
		v1(_v1),
		v2(_v2)
	{
		const FVector3 edgeV0V1 = v1 - v0;
		const FVector3 edgeV0V2 = v2 - v0;
		normal = FVector3::Cross(edgeV0V1, edgeV0V2).Normalized();
	
	}
	__device__ __host__ static FVector3 Max(Triangle &t)
	{

		return FVector3::Max(FVector3::Max(t.v0,t.v1),t.v2);
	}
	__device__ __host__ static FVector3 Min(Triangle& t)
	{
		return FVector3::Min(FVector3::Min(t.v0, t.v1), t.v2);
	}

	FVector3 v0 {};
	FVector3 v1 {};
	FVector3 v2 {};

	FVector3 normal;

	//TriangleCullMode mode{ TriangleCullMode::BackFaceCulling };
	PBRMaterial mat;
};
struct AABB
{
	AABB() = default;
	FVector3 minAABB{FLT_MAX};
	FVector3 maxAABB{-FLT_MAX };

	FVector3 transformedMinAABB{};
	FVector3 transformedMaxAABB{};

	FVector3 Centroid{};

	__device__ void Grow(FVector3 p)
	{

		minAABB = FVector3::Min(minAABB, p);
		maxAABB = FVector3::Max(maxAABB, p);
	}
	__device__ float Area()
	{
		FVector3 extent = maxAABB - minAABB;
		float area = (extent.x * extent.y) + (extent.y * extent.z) + (extent.z * extent.x);
		return area;
	}
	// Get Center
	// Get Extents ?
};

struct TriangleMesh
{
	TriangleMesh();
	TriangleMesh(const std::string& filename);
	TriangleMesh( Triangle* _triangles , int _triangleCount) :
		triangles(_triangles),
		triangleCount(_triangleCount)
		
	{}
	
	FMatrix rotation{}, translation{}, scale{};
	std::vector<FVector3> positions{};
	std::vector<FVector3> normals{};
	std::vector<int> indices{};
	Triangle* triangles{};
	int triangleCount = 0;
	
};

__device__ __host__ struct CTriangleMesh
{
	Triangle* triangles{};
	int triangleCount;
	
	

	AABB boundingVolume;
	FMatrix rotation, translation, scale;
	//Sphere boundingSphere;
	void UpdateTransforms()
	{
	
			FMatrix finalTransform = scale*  translation * rotation;

			for (int i = 0; i < triangleCount; i++)
			{
				triangles[i].v0 = finalTransform.TransformPoint(triangles[i].v0);
				triangles[i].v1 = finalTransform.TransformPoint(triangles[i].v1);
				triangles[i].v2 = finalTransform.TransformPoint(triangles[i].v2);
				triangles[i].normal = finalTransform.TransformVector(triangles[i].normal).Normalized();
			}
			UpdateTransformedAABB(finalTransform);
			rotation = FMatrix{};
			translation = FMatrix{};
			scale = FMatrix{};
				
			
	}
	void SetMaterial(PBRMaterial material)
	{
		for (size_t i = 0; i < triangleCount; i++)
		{
			triangles[i].mat = material;
		}
	}
	void Translate(FVector3 _translation)
	{
		translation = FMatrix::CreateTranslation(_translation);
	}
	void RotateY(float yaw)
	{
		rotation = FMatrix::CreateRotationY(yaw);
	}
	void RotateYAxis(float yaw , FVector3 pos)
	{
		rotation = FMatrix::CreateRotationY(yaw) * FMatrix::CreateTranslation(pos);
	}
	void RotateX(float angle)
	{
		angle *= TO_RADIANS;
		rotation = FMatrix::CreateRotationX(angle);
	}
	void RotateZ(float angle)
	{
		angle *= TO_RADIANS;
		rotation = FMatrix::CreateRotationZ(angle);
	}
	void Scale(const FVector3& _scale)
	{
		scale = FMatrix::CreateScale(_scale);
	}

	//void CalculateBoundingSphere()
	//{
	//	//boundingSphere.radius = (boundingVolume.maxAABB - boundingVolume.minAABB).Magnitude() * 0.5f;
	//	boundingSphere.origin = (boundingVolume.minAABB + boundingVolume.maxAABB) * 0.5f;
	//	if (triangleCount>0)
	//	{
	//		//FVector3 farthest = triangles[0].v0;
	//		float current = 0.f;
	//		for (size_t i = 0; i < triangleCount; i++)
	//		{
	//			float Dv0 = (triangles[i].v0 - boundingSphere.origin).Magnitude();
	//			float Dv1 = (triangles[i].v1 - boundingSphere.origin).Magnitude();
	//			float Dv2 = (triangles[i].v2 - boundingSphere.origin).Magnitude();
	//			float temp = fmaxf(fmaxf(Dv0,Dv1) ,Dv2 );
	//			if (temp > current)
	//			{
	//				current = temp;
	//		
	//			}
	//		}
	//		boundingSphere.radius = current;
	//	}
	//	
	//	
	//}
	void UpdateAABB()
	{
		if (triangleCount > 0)
		{
			boundingVolume.minAABB =  triangles[0].v0;
			boundingVolume.maxAABB = triangles[0].v0;
			
			for (size_t i = 0; i < triangleCount; i++)
			{
				boundingVolume.minAABB = FVector3::Min(Triangle::Min(triangles[i]), boundingVolume.minAABB);
				
				boundingVolume.maxAABB = FVector3::Max(Triangle::Max(triangles[i]), boundingVolume.maxAABB);
			}
			boundingVolume.Centroid = (boundingVolume.minAABB + boundingVolume.maxAABB) * 0.5f;
		}
	}
	void UpdateTransformedAABB(const FMatrix& finalTransform)
	{
		FVector3 tMinAABB = finalTransform.TransformPoint(boundingVolume.minAABB);
		FVector3 tMaxAABB = tMinAABB;

		FVector3 tAABB = finalTransform.TransformPoint(boundingVolume.maxAABB.x, boundingVolume.minAABB.y, boundingVolume.minAABB.z);
		tMinAABB = FVector3::Min(tAABB, tMinAABB);
		tMaxAABB = FVector3::Max(tAABB, tMaxAABB);
		// (xmax,ymin,zmax)
		tAABB = finalTransform.TransformPoint(boundingVolume.maxAABB.x, boundingVolume.minAABB.y, boundingVolume.maxAABB.z);
		tMinAABB = FVector3::Min(tAABB, tMinAABB);
		tMaxAABB = FVector3::Max(tAABB, tMaxAABB);
		// (xmin,ymin,zmax)
		tAABB = finalTransform.TransformPoint(boundingVolume.minAABB.x, boundingVolume.minAABB.y, boundingVolume.maxAABB.z);
		tMinAABB = FVector3::Min(tAABB, tMinAABB);
		tMaxAABB = FVector3::Max(tAABB, tMaxAABB);
		// (xmin,ymax,zmin)
		tAABB = finalTransform.TransformPoint(boundingVolume.minAABB.x, boundingVolume.maxAABB.y, boundingVolume.minAABB.z);
		tMinAABB = FVector3::Min(tAABB, tMinAABB);
		tMaxAABB = FVector3::Max(tAABB, tMaxAABB);
		// (xmax,ymax,zmin)
		tAABB = finalTransform.TransformPoint(boundingVolume.maxAABB.x, boundingVolume.maxAABB.y, boundingVolume.minAABB.z);
		tMinAABB = FVector3::Min(tAABB, tMinAABB);
		tMaxAABB = FVector3::Max(tAABB, tMaxAABB);
		// (xmax,ymax,zmax)
		tAABB = finalTransform.TransformPoint(boundingVolume.maxAABB);
		tMinAABB = FVector3::Min(tAABB, tMinAABB);
		tMaxAABB = FVector3::Max(tAABB, tMaxAABB);
		// (xmin,ymax,zmax)
		tAABB = finalTransform.TransformPoint(boundingVolume.minAABB.x, boundingVolume.maxAABB.y, boundingVolume.minAABB.z);
		tMinAABB = FVector3::Min(tAABB, tMinAABB);
		tMaxAABB = FVector3::Max(tAABB, tMaxAABB);

		boundingVolume.transformedMinAABB = tMinAABB;
		boundingVolume.transformedMaxAABB = tMaxAABB;
		boundingVolume.minAABB = tMinAABB;
		boundingVolume.maxAABB = tMaxAABB;
		/*boundingVolume.transformedMinAABB = finalTransform.TransformVector(boundingVolume.minAABB);
		boundingVolume.transformedMaxAABB = finalTransform.TransformVector(boundingVolume.maxAABB);*/

		
	}
};

__device__ __host__ struct BVHNode
{
	AABB boundingVolume;
	int left = 0;
	int firstPrim =0 , primCount = 0;


};

