#include <cuda_runtime.h>
#include "cuda.h"
#include "device_launch_parameters.h"

#include <iostream>
#include "Raytrace.h"
#include "Renderer.h"
#include "SceneGraph.h"
#include "Math.h"
#include "FVector3.h"
#include "FVector2.h"
#include "PrimitiveTypes.h"
#include "BRDF.h"
#include <cfloat>
#include "Light.h"
#include <math.h>
#include "sm_61_intrinsics.h"
#include "Utils.h"
#include "MathHelpers.h"
#include <ppl.h> //parallel_for
#include "thrust/swap.h"
#include "thrust/device_vector.h"
#define E_PI	3.14159265358979323846

using namespace Raytracing;

__device__ const CTriangleMesh* meshes;
__device__ int MeshCount=0;

__device__ int* totalIntersectionCount;
__device__ int totalInterSectionCounter = 0;

__device__ int* boxIntersectionCount;
__device__ int boxInterSectionCounter = 0;

//__device__ int* sphereIntersectionCount;
//__device__ int sphereInterSectionCounter = 0;

__device__ int* FPIntersectionCount;
__device__ int FPInterSectionCounter = 0;

__device__ int* triangleIntersectionCount;
__device__ int triangleInterSectionCounter = 0;

//__device__ float* TriangleTime;
//__device__ float triangleTiming = 0.0f;

__device__ Light* lights;
__device__ int lightCount = 0;
__device__ struct Bin 
{ 
	AABB bounds{ FLT_MAX,-FLT_MAX}; 
	int primCount = 0;
};
__device__ const int BINS = 8;

//__device__  int MeshIdx[16];
//__device__ struct BVHNode
//{
//	AABB boundingVolume;
//	unsigned int left;
//	unsigned int firstPrim, primCount;
//
//
//};
template <typename T> __device__ void swap_device(T& a, T& b)
{
	T c(a); a = b; b = c;
}
__device__ void inline swapInt_device(unsigned int a,unsigned int b)
{
	int c(a); a = b; b = c;
}
__device__ void inline swapFloat_device(float a, float b)
{
	int c(a); a = b; b = c;
}
__device__ bool IntersectAABB(const AABB& box, const Ray& ray);
__device__ void UpdateBounds(unsigned int nodeIdx);
__device__ void IntersectMesh(const Ray& ray, const CTriangleMesh& mesh, HitRecord& closestHit);
__device__ void Subdivide(unsigned int nodeIdx);
__device__ void SubdivideSAH(unsigned int nodeIdx);
__device__ void SubdivideSAHBinning(unsigned int nodeIdx);
__device__ float EvaluateSAH(int nodeIdx, int axis, float pos);
__device__ float FindBestSplitPlane(const BVHNode& node, int& axis, float& splitPos);
__device__ FVector3 Min(const FVector3& a, const FVector3& b);
__device__ float CalculateNodeCost(const BVHNode& node);

__device__ BVHNode *bvhNode;


__device__ int* MeshIdx;
__device__ int rootNodeIdx = 0, nodesUsed = 1;
__global__ void BuildBVH(BVHNode* bvhTree,CTriangleMesh* _meshes , int meshcount , Light* _lights , int lightcount , int* _MeshIdx )
{
	bvhNode = bvhTree;
	meshes = _meshes;
	MeshCount = meshcount;
	const int N = meshcount;
	lights = _lights;
	lightCount = lightcount;
	MeshIdx = _MeshIdx;
	/*for (size_t i = 0; i < meshcount; i++)
	{
		MeshIdx[i] = i;
	}*/
	
	BVHNode& root = bvhNode[rootNodeIdx];

	root.left = 0;
	root.firstPrim = 0, root.primCount = MeshCount;

	UpdateBounds(rootNodeIdx);
	//SubdivideSAH(rootNodeIdx);
	SubdivideSAHBinning(rootNodeIdx);
	// Subdivide(rootNodeIdx);
	// UPDATE NODE BOUNDS
	
	// SUBDIVIDE RECURSIVELY
	
	bvhTree = bvhNode;
	//__syncthreads();
}
__device__ void UpdateBounds(unsigned int nodeIdx)
{
	BVHNode& node = bvhNode[nodeIdx];


	node.boundingVolume.minAABB = FVector3(FLT_MAX);
	node.boundingVolume.maxAABB = FVector3(-FLT_MAX);

	for (int i = 0, int first = node.firstPrim; i < node.primCount; i++)
	{
		int& leafMeshIdx = MeshIdx[first + i];
		const CTriangleMesh& leafMesh = meshes[leafMeshIdx];

		node.boundingVolume.minAABB = FVector3::Min(node.boundingVolume.minAABB, leafMesh.boundingVolume.minAABB);
		node.boundingVolume.maxAABB = FVector3::Max(node.boundingVolume.maxAABB, leafMesh.boundingVolume.maxAABB);

	}
}
__device__ void Subdivide(unsigned int nodeIdx)
{
	BVHNode& node = bvhNode[nodeIdx];

	if (node.primCount <= 2)
	{
		return;
	}

	FVector3 extent = node.boundingVolume.maxAABB - node.boundingVolume.minAABB;

	int axis = 0;
	if (extent.y > extent.x)
	{
		axis = 1;
	}
	if (extent.z > extent[axis])
	{
		axis = 2;
	}

	float splitPos = node.boundingVolume.minAABB[axis] + (extent[axis] * 0.5f);
	int j = 0;
	int i = node.firstPrim;
	j = i + node.primCount - 1;

	while (i <= j)
	{
		if (meshes[MeshIdx[i]].boundingVolume.Centroid[axis] < splitPos)
		{
			i++;
		}
		else
		{
			int temp = MeshIdx[j];
			MeshIdx[j--] = MeshIdx[i];
			MeshIdx[i] = temp;
			
		}
	}

	int leftCount = i - node.firstPrim;
	if (leftCount == 0 || leftCount == node.primCount)
	{
		return;
	}
	int leftChildIdx = nodesUsed++;
	int rightChildIdx = nodesUsed++;

	node.left = leftChildIdx;
	bvhNode[leftChildIdx].firstPrim = node.firstPrim;
	bvhNode[leftChildIdx].primCount = leftCount;
	bvhNode[rightChildIdx].firstPrim = i;
	bvhNode[rightChildIdx].primCount = node.primCount - leftCount;
	node.primCount = 0;
	node.firstPrim = 0;
	node.left = leftChildIdx;

	UpdateBounds(leftChildIdx);
	UpdateBounds(rightChildIdx);

	//Recurse
	Subdivide(leftChildIdx);
	Subdivide(rightChildIdx);

}

__device__ void SubdivideSAH(unsigned int nodeIdx)
{
	BVHNode& node = bvhNode[nodeIdx];

	if (node.primCount <= 1)
	{
		return;
	}
	/*int axis = 0;
	float splitPos = 0;
	float splitCost = 0;
	int amountOfPlanes = 100;
	float bestCost = 1e30f;
	for (int a = 0; a < 3; a++)
	{

		float boundsMin = node.boundingVolume.minAABB[a];
		float boundsMax = node.boundingVolume.maxAABB[a];
		float scale = (boundsMax - boundsMin) / amountOfPlanes;
		for (int i = 1; i < amountOfPlanes; i++)
		{
			float candidatePos = boundsMin + i * scale;
			float cost = EvaluateSAH(node, a, candidatePos);
			if (cost < bestCost)
			{
				splitPos = candidatePos;
				axis = a;
				bestCost = cost;
			}
		}
	}*/
	//FVector3 extent = node.boundingVolume.maxAABB - node.boundingVolume.minAABB;
	/*int axis = -1;
	float splitPos = 0;
	float splitCost =0;
	float bestCost = 1e30f;
	float noSplitCost = CalculateNodeCost(node);
	for (int a = 0; a < 3; a++) 
	{
		for (int i = 0; i < node.primCount; i++)
		{
			const CTriangleMesh mesh = meshes[MeshIdx[node.firstPrim + i]];
			float candidatePos = mesh.boundingVolume.Centroid[a];
			float cost = EvaluateSAH(node, a, candidatePos);

			if (cost < bestCost)
			{
				splitPos = candidatePos, axis = a, bestCost = cost;
			}
		}
		
	}*/
	float splitCost = 0;
	int bestAxis = -1;
	float bestPos = 0;
	float bestCost = FLT_MAX;
	
	for (int axis = 0; axis < 3; axis++)
	{
		for (int i = 0; i < node.primCount; i++)
		{
			float candidatePos = meshes[MeshIdx[node.firstPrim + i]].boundingVolume.Centroid[axis];
			float cost = EvaluateSAH(nodeIdx, axis, candidatePos);
			if (cost < bestCost)
			{
				
				bestPos = candidatePos;
				bestAxis = axis;
				bestCost = cost;
			}
		}
	}
	int Axis = bestAxis;
	float splitPos = bestPos;
	splitCost = bestCost;
	float noSplitCost = CalculateNodeCost(node);
	if (splitCost >= noSplitCost)
	{
		return;
	}
	int j = 0;
	int i = node.firstPrim;
	j = i + node.primCount - 1;

	while (i <= j)
	{
		if (meshes[MeshIdx[i]].boundingVolume.Centroid[Axis] < splitPos)
		{
			i++;
		}
		else
		{
			int temp = MeshIdx[j];
			MeshIdx[j--] = MeshIdx[i];
			MeshIdx[i] = temp;
		}
	}

	int leftCount = i - node.firstPrim;
	if (leftCount == 0 || leftCount == node.primCount)
	{
		return;
	}
	int leftChildIdx = nodesUsed++;
	int rightChildIdx = nodesUsed++;

	node.left = leftChildIdx;
	bvhNode[leftChildIdx].firstPrim = node.firstPrim;
	bvhNode[leftChildIdx].primCount = leftCount;
	bvhNode[rightChildIdx].firstPrim = i;
	bvhNode[rightChildIdx].primCount = node.primCount - leftCount;
	node.primCount = 0;
	node.firstPrim = 0;
	//node.left = leftChildIdx;

	UpdateBounds(leftChildIdx);
	

	UpdateBounds(rightChildIdx);
	SubdivideSAH(leftChildIdx);
	SubdivideSAH(rightChildIdx);
}
__device__ void SubdivideSAHBinning(unsigned int nodeIdx)
{
	BVHNode& node = bvhNode[nodeIdx];

	if (node.primCount <= 1)
	{
		return;
	}
	
	float splitCost = 0;
	int bestAxis = -1;
	float bestPos = 0;
	//splitCost = FindBestSplitPlane(node,bestAxis, bestPos);

	float bestCost = FLT_MAX;
	int amountOfSplits = 12;

	for (int a = 0; a < 3; a++)
	{
		float boundsMin = node.boundingVolume.minAABB[a];
		float boundsMax = node.boundingVolume.maxAABB[a];
		if (boundsMin == boundsMax)
		{
			continue;
		}
		float scale = (boundsMax - boundsMin) / amountOfSplits;
		for (int i = 1; i < amountOfSplits; i++)
		{
			float candidatePos = boundsMin + (i * scale);
			float cost = EvaluateSAH(nodeIdx, a, candidatePos);

			if (cost < bestCost)
			{
				bestPos = candidatePos;
				bestAxis = a;
				bestCost = cost;
			}
		}
	}
	splitCost = bestCost;
	int axis = bestAxis;
	float splitPos = bestPos;

	float noSplitCost = CalculateNodeCost(node);
	if (splitCost >= noSplitCost)
	{
		return;
	}
	int j = 0;
	int i = node.firstPrim;
	j = i + node.primCount - 1;

	while (i <= j)
	{
		if (meshes[MeshIdx[i]].boundingVolume.Centroid[axis] < splitPos)
		{
			i++;
		}
		else
		{
			int temp = MeshIdx[j];
			MeshIdx[j--] = MeshIdx[i];
			MeshIdx[i] = temp;
		}
	}
	
	int leftCount = i - node.firstPrim;
	if (leftCount == 0 || leftCount == node.primCount)
	{
		return;
	}
	int leftChildIdx = nodesUsed++;
	int rightChildIdx = nodesUsed++;

	if (node.firstPrim < MeshCount-1 && node.primCount <= MeshCount)
	{
		node.left = leftChildIdx;
		bvhNode[leftChildIdx].firstPrim = node.firstPrim;
		bvhNode[leftChildIdx].primCount = leftCount;
		bvhNode[rightChildIdx].firstPrim = i;
		bvhNode[rightChildIdx].primCount = node.primCount - leftCount;
		node.primCount = 0;
		node.firstPrim = 0;
		UpdateBounds(leftChildIdx);
		UpdateBounds(rightChildIdx);
		SubdivideSAHBinning(leftChildIdx);
		SubdivideSAHBinning(rightChildIdx);
	}
	
	//node.left = leftChildIdx;


	
}
//__device__ int intersectionCounter = 0;
__device__ float EvaluateSAH(int nodeIdx, int axis, float pos)
{
	BVHNode& node = bvhNode[nodeIdx];
	AABB leftBox, rightBox;
	leftBox.minAABB = FVector3(FLT_MAX);
	rightBox.minAABB = FVector3(FLT_MAX);
	leftBox.maxAABB = FVector3(-FLT_MAX);
	rightBox.maxAABB = FVector3(-FLT_MAX);
	int leftCount = 0, rightCount = 0;
	if (node.primCount > MeshCount || node.firstPrim > MeshCount)
	{
		return FLT_MAX;
	}

	for (int i = 0; i < node.primCount; i++)
	{
		//int& leafMeshIdx = MeshIdx[node.firstPrim + i];
		const CTriangleMesh& mesh = meshes[MeshIdx[node.firstPrim + i]];
	
		if (mesh.boundingVolume.Centroid[axis] < pos)
		{
			leftCount++;
			/*leftBox.Grow(mesh.boundingVolume.minAABB);
			leftBox.Grow(mesh.boundingVolume.maxAABB);*/
			leftBox.minAABB = FVector3::Min(mesh.boundingVolume.minAABB, leftBox.minAABB);
			leftBox.maxAABB = FVector3::Max(mesh.boundingVolume.maxAABB,leftBox.maxAABB);

		}
		else
		{
			rightCount++;
			/*rightBox.Grow(mesh.boundingVolume.minAABB);
			rightBox.Grow(mesh.boundingVolume.maxAABB);*/
			rightBox.minAABB = FVector3::Min( mesh.boundingVolume.minAABB,rightBox.minAABB);
			rightBox.maxAABB = FVector3::Max( mesh.boundingVolume.maxAABB,rightBox.maxAABB);
		}
	}
	FVector3 extent = leftBox.maxAABB - leftBox.minAABB;
	float area = (extent.x * extent.y) + (extent.y * extent.z) + (extent.z * extent.x);
	float costLeft = leftCount * area;
	if (!costLeft)
	{
		costLeft = 0;
	}
	FVector3 extentR = rightBox.maxAABB - rightBox.minAABB;
	float areaR = (extent.x * extent.y) + (extent.y * extent.z) + (extent.z * extent.x);
	float costRight = rightCount * areaR;
	if (!costRight)
	{
		costRight = 0;
	}
	float cost = costLeft + costRight;

	return cost > 0 ? cost : FLT_MAX;
}
//__device__ float FindBestSplitMeshes(const BVHNode& node, int& axis, float& splitPos);
__device__ float FindBestSplitPlane( const BVHNode& node, int& axis, float& splitPos)
{
	float bestCost = FLT_MAX;
	int amountOfSplits = 100;

	for (int a = 0; a < 3; a++)
	{
		float boundsMin = node.boundingVolume.minAABB[a];
		float boundsMax = node.boundingVolume.maxAABB[a];

		if (boundsMin == boundsMax)
		{
			continue;
		}
		float scale = (boundsMax - boundsMin) / amountOfSplits;
		for (int i = 1; i < amountOfSplits; i++)
		{
			float candidatePos = boundsMin + (i * scale);
	//		float cost = EvaluateSAH(node, a, candidatePos);

		/*	if (cost < bestCost)
			{
				splitPos = candidatePos;
				axis = a;
				bestCost = cost;
			}*/
		}
	}
	return bestCost;
	/*float bestCost = 1e30f;
	
	for (int a = 0; a < 3; a++)
	{

		float boundsMin = FLT_MAX;
		float boundsMax = -FLT_MAX;

		for (int i = 0; i < node.primCount; i++)
		{
			meshes[MeshIdx[node.firstPrim + i]].boundingVolume.Centroid[axis]
			const CTriangleMesh mesh = meshes[MeshIdx[node.firstPrim + i]];
			boundsMin = fminf(meshes[MeshIdx[node.firstPrim + i]].boundingVolume.Centroid[a],boundsMin);
			boundsMax = fmaxf(meshes[MeshIdx[node.firstPrim + i]].boundingVolume.Centroid[a],boundsMin);
		}
		if (boundsMin == boundsMax) continue;
		Bin bin[BINS];
		
		float scale = BINS/(boundsMax - boundsMin);
		for (int i = 0; i < node.primCount; i++)
		{
			const CTriangleMesh& mesh = meshes[MeshIdx[node.firstPrim + i]];
			int binIdx = fminf(BINS - 1,(int)((mesh.boundingVolume.Centroid[a] - boundsMin) * scale));
			bin[binIdx].primCount++;

			bin[binIdx].bounds.minAABB = FVector3::Min( mesh.boundingVolume.minAABB,bin[binIdx].bounds.minAABB);
			bin[binIdx].bounds.maxAABB = FVector3::Max( mesh.boundingVolume.maxAABB,bin[binIdx].bounds.maxAABB);

		}
		float leftArea[BINS - 1], rightArea[BINS - 1];
		int leftCount[BINS - 1], rightCount[BINS - 1];
		AABB leftBox, rightBox;
		leftBox.minAABB = FVector3(FLT_MAX);
		rightBox.minAABB = FVector3(FLT_MAX);
		leftBox.maxAABB = FVector3(-FLT_MAX);
		rightBox.maxAABB = FVector3(-FLT_MAX);
		
		int leftSum = 0, rightSum = 0;
		for (int i = 0; i < BINS - 1; i++)
		{
			leftSum += bin[i].primCount;
			leftCount[i] = leftSum;
			leftBox.minAABB = FVector3::Min(leftBox.minAABB,bin[i].bounds.minAABB );
			leftBox.maxAABB = FVector3::Max(leftBox.maxAABB,bin[i].bounds.maxAABB );
			FVector3 extent = leftBox.maxAABB - leftBox.minAABB;
			leftArea[i] = (extent.x * extent.y) + (extent.y * extent.z) + (extent.z * extent.x);

			leftArea[i] = leftBox.Area();

			rightSum += bin[BINS - 1 - i].primCount;
			rightCount[BINS - 2 - i] = rightSum;
			rightBox.minAABB = FVector3::Min( rightBox.minAABB,bin[i].bounds.minAABB);
			rightBox.maxAABB = FVector3::Max( rightBox.maxAABB,bin[i].bounds.maxAABB);
			FVector3 extentR = rightBox.maxAABB - rightBox.minAABB;
			rightArea[BINS - 2 - i] = (extentR.x * extentR.y) + (extentR.y * extentR.z) + (extentR.z * extentR.x);
		}
		scale = (boundsMax - boundsMin) / BINS;
		for (int i = 0; i < BINS - 1; i++)
		{
			float planeCost = leftCount[i] * leftArea[i] + rightCount[i] * rightArea[i];
			if (planeCost < bestCost)
			{
				axis = a;
				splitPos = boundsMin + scale * (i + 1);
				bestCost = planeCost;
			}
				
		}
	}
	return bestCost;*/
}

//FVector3 Min(const FVector3& a, const FVector3& b)
//{
//	return FVector3(
//			(a.x < b.x ? a.x :b.x),
//			(a.y < b.y ? a.y :b.y),
//			(a.z < b.z ? a.z :b.z)
//			/*fmaxf(v1.x,v2.x),
//			fmaxf(v1.y, v2.y),
//			fmaxf(v1.z, v2.z)*/
//	);
//}

__device__ float CalculateNodeCost(const BVHNode& node)
{
	FVector3 e = node.boundingVolume.maxAABB - node.boundingVolume.minAABB; // extent of the node
	float surfaceArea = (e.x * e.y) + (e.y * e.z) + (e.z * e.x);
	return node.primCount * surfaceArea;
}
__global__ void RefitBVH()
{
	for (int i = nodesUsed - 1; i >= 0; i--)if (i != 1)
	{
		BVHNode& node = bvhNode[i];
		if (node.primCount > 0)
		{
			// leaf node: adjust bounds to contained triangles
			UpdateBounds(i);
			continue;
		}
		// interior node: adjust bounds to child node bounds
		BVHNode& leftChild = bvhNode[node.left];
		BVHNode& rightChild = bvhNode[node.left + 1];
		node.boundingVolume.minAABB = FVector3::Min(leftChild.boundingVolume.minAABB, rightChild.boundingVolume.minAABB);
		node.boundingVolume.maxAABB = FVector3::Max(leftChild.boundingVolume.maxAABB, rightChild.boundingVolume.maxAABB);
	}
}

__device__ ColorRGB Shade(const HitRecord& hit, const FVector3& l, const FVector3& v)
{
	ColorRGB f0 = ColorRGB(.04f, .04f, .04f); //Default dielectric (non-metal) surface reflectivity
	f0 = ColorRGB::Lerp(f0, hit.mat.m_Albedo, hit.mat.m_Metalness); //Default reflectivity if dielectric (metalness=0) OR albedo of conductor (metalness=1)

	//Specular (Cook-Torrance)
	//************************
	ColorRGB specular{};

	const FVector3 halfVector = (v + l).Normalized();

	ColorRGB F = FresnelFunction_Schlick(halfVector, v, f0);
	float D = NormalDistribution_GGX(hit.normal, halfVector, hit.mat.m_Roughness);
	float G = GeometryFunction_Smith(hit.normal, v, l, hit.mat.m_Roughness);

	const float denominator = 4.f * fmaxf(FVector3::Dot(hit.normal, v), 0.f) * fmaxf(FVector3::Dot(hit.normal, l), 0.f);
	specular += (F * D * G) / fmaxf(denominator, 0.0001f); //prevent any divide by zero!

	//Diffuse (Lambert)
	//*****************
	ColorRGB diffuse{};
	ColorRGB kd = ColorRGB(1.f, 1.f, 1.f) - F; //F >> Fresnel or Specular Reflection Coefficient
	kd *= 1.f - hit.mat.m_Metalness; //Nullify kd if surface is conductor (metalness==1) > No refraction

	diffuse += Lambert(kd, hit.mat.m_Albedo);

	//Final Color
	//***********
	return diffuse + specular;
}
__device__ FVector3 GetDirectionToLight(const Light& light, const FVector3 origin)
{
	if (light.type == LightType::point)
	{
		return { origin, light.origin };
	}

	return -light.direction;
}
__device__ ColorRGB GetRadiance(const Light& light, const FVector3& target)
{
	if (light.type == LightType::point)
	{
		return light.color * (light.intensity / FVector3(light.origin, target).SqrMagnitude());
	}

	return light.color * light.intensity;
}
__device__ float ToRadians(float fov)
{
	return fov * (static_cast<float>(E_PI) / static_cast<float>(180.f));
}
__device__ FVector2 GetScreenSpace(size_t c, size_t r, float aspectratio, float fov)
{
	FVector2 screenspace;
	float _fov = tan(ToRadians(fov) / 2);
	screenspace.x = (2.f * ((c + 0.5f) / float(640)) - 1.f) * 640  /* * aspectratio * fov*/;
	screenspace.y = (1.f - 2.f * ((r + 0.5f) / float(480))) * 480;

	return screenspace;
}



//__device__ bool PlaneHit(const Plane& plane, const Ray& ray, HitRecord& hit)
//{
//	/*atomicAdd(&intersectionCounter, 1);
//	__syncthreads();*/
//	const FVector3 L = plane.origin - ray.origin; //Ray To Plane Direction
//	const float t = FVector3::Dot(L, plane.normal) / FVector3::Dot(ray.direction, plane.normal);
//
//	if (t >= ray.min && t <= ray.max)
//	{
//
//
//		hit.t = t;
//		hit.origin = ray.origin + (t * ray.direction);
//		hit.normal = plane.normal;
//		hit.didHit = true;
//		hit.mat = plane.mat;
//
//		return true;
//	}
//
//	hit.didHit = false;
//	return false;
//}
__device__ HitRecord MollerTriangleHit( Triangle* triangle, const Ray& ray)
{
	HitRecord Temp{};
	
	const FVector3 edge1 = triangle->v1 - triangle->v0;
	const FVector3 edge2 = triangle->v2 - triangle->v0;

	const FVector3 h = FVector3::Cross(ray.direction, edge2);

	const float a = FVector3::Dot(edge1, h);

	if (a > -0.0000001f && a < 0.0000001f)
	{
		return Temp;
	}

	const float f = 1 / a;

	const FVector3 s = ray.origin - triangle->v0;
	const float u = FVector3::Dot(s, h) * f;

	if (u < 0 || u > 1) return Temp;

	const FVector3 q = FVector3::Cross(s, edge1);
	const float v = FVector3::Dot(ray.direction, q) * f;

	if (v < 0 || u + v > 1) return Temp;

	const float t = FVector3::Dot(edge2, q) * f;
	if (t > 0.0000001f)
	{
		Temp.t = t;
		Temp.origin = ray.origin + (t * ray.direction);
		Temp.normal = triangle->normal;
		Temp.mat = triangle->mat;
		Temp.didHit = true;
		return Temp;
	}
	else
	{
		return Temp;
	}

}
__device__ bool MollerTriangleHit( Triangle triangle,  Ray ray, HitRecord& hit)
{
	atomicAdd(&totalInterSectionCounter, 1);
	atomicAdd(&triangleInterSectionCounter, 1);
	FVector3 edge1 = triangle.v1 - triangle.v0;
	FVector3 edge2 = triangle.v2 - triangle.v0;

	FVector3 h = FVector3::Cross(ray.direction, edge2);

	float a = FVector3::Dot(edge1, h);

	if ( a > -0.0000001f && a < 0.0000001f)
	{
		return false;
	}

	float f = 1 / a;

	FVector3 s = ray.origin - triangle.v0;
	float u = FVector3::Dot(s, h) * f;

	if (u < 0 || u > 1) return false;

	FVector3 q = FVector3::Cross(s, edge1);
	float v = FVector3::Dot(ray.direction, q) * f;

	if (v < 0 || u+v > 1) return false;

	float t = FVector3::Dot(edge2, q) * f;
	if (t > 0.0000001f)
	{
		hit.t = t;
		hit.origin = ray.origin + (t * ray.direction);
		hit.normal = triangle.normal;
		hit.mat = triangle.mat;
		hit.didHit = true;
		return true;
	}
	else
	{
		return false;
	}
	
}
__device__ bool TriangleHit(const Triangle& triangle, const Ray& ray, HitRecord& hit)
{

	atomicAdd(&totalInterSectionCounter, 1);
	atomicAdd(&triangleInterSectionCounter, 1);
	/*atomicAdd(&intersectionCounter, 1);
	__syncthreads();*/
	hit.didHit = false;

	//Normal vs Ray Check (Perpendicular = skip)
	float nDotr = FVector3::Dot(triangle.normal, ray.direction);
	if (AreEqual(nDotr, 0.f))
		return false;

	//CullMode Check (Not visible = skip)
	
	/*	if (nDotr > 0.f && triangle.mode == TriangleCullMode::BackFaceCulling
			|| nDotr < 0.f && triangle.mode == TriangleCullMode::FrontFaceCulling)
			return false;*/
	
	

	//Ray-Plane Intersection + Range Check
	//Calculate t
	FVector3 center = FVector3((FVector3(triangle.v0) + FVector3(triangle.v1) + FVector3(triangle.v2)) / 3.f);
	FVector3 L = center - ray.origin;
	float t = FVector3::Dot(L, triangle.normal) / FVector3::Dot(ray.direction, triangle.normal);

	//Within range
	if (t < ray.min || t > ray.max)
		return false;

	//Inside Triangle Check
	//Calculate intersection point on parallelogram
	FVector3 p = ray.origin + t * ray.direction;

	//Check if this point is inside the triangle! Called the Inside-Outside test.
	FVector3 c = {};
	FVector3 side = {};
	FVector3 pointToSide = {};

	//Edge A
	side = triangle.v1 - triangle.v0;
	pointToSide = p - triangle.v0;
	c = FVector3::Cross(side, pointToSide);
	if (FVector3::Dot(triangle.normal, c) < 0.f) //point is on the right side, so outside the triangle
		return false;
	//Edge B
	side = triangle.v2 - triangle.v1;
	pointToSide = p - triangle.v1;
	c = FVector3::Cross(side, pointToSide);
	if (FVector3::Dot(triangle.normal, c) < 0.f) //point is on the right side, so outside the triangle
		return false;
	//Edge C
	side = triangle.v0 - triangle.v2;
	pointToSide = p - triangle.v2;
	c = FVector3::Cross(side, pointToSide);
	if (FVector3::Dot(triangle.normal, c) < 0.f) //point is on the right side, so outside the triangle
		return false;

	//If the inside-outside test succeeded, we have hit in our triangle, so store info
	hit.t = t;
	hit.origin = p;
	hit.normal = triangle.normal;
	hit.mat = triangle.mat;
	hit.didHit = true;
	return true;
}

__device__ bool DoesSphereHit(const Sphere& sphere, const Ray& ray)
{
	const FVector3 L = ray.origin - sphere.origin;

	const float a = FVector3::Dot(ray.direction, ray.direction);
	const float b = 2.f * FVector3::Dot(ray.direction, L);
	const float c = FVector3::Dot(L, L) - Square(sphere.radius);

	float discriminant = Square(b) - 4.f * a * c;

	//Check for intersection
	if (discriminant > 0)
	{
		discriminant = sqrt(discriminant);
		float t = (-b - discriminant) / (2.f * a);

		//Check if Valid
		if (t < ray.min)
			t = (-b + discriminant) / (2.f * a);

		if (t < ray.min || t > ray.max)
			return false; //no hit
		return true;
	}

	return false;
}

__device__ bool DoesPlaneHit(const Plane& plane, const Ray& ray)
{
	const FVector3 L = plane.origin - ray.origin; //Ray To Plane Direction
	const float t = FVector3::Dot(L, plane.normal) / FVector3::Dot(ray.direction, plane.normal);

	if (t >= ray.min && t <= ray.max)
	{
		return true;


	}
	return false;
}

__device__ bool DoesTriangleHit(const Triangle& triangle, const Ray& ray)
{
	//Normal vs Ray Check (Perpendicular = skip)
	float nDotr = FVector3::Dot(triangle.normal, ray.direction);
	if (AreEqual(nDotr, 0.f))
		return false;
	//CullMode Check (Not visible = skip)
		/*if (nDotr > 0.f && triangle.mode == TriangleCullMode::FrontFaceCulling
			|| nDotr < 0.f && triangle.mode == TriangleCullMode::BackFaceCulling)
		return false;*/

	//Ray-Plane Intersection + Range Check
	//Calculate t
	FVector3 center = FVector3((FVector3(triangle.v0) + FVector3(triangle.v1) + FVector3(triangle.v2)) / 3.f);
	FVector3 L = center - ray.origin;
	float t = FVector3::Dot(L, triangle.normal) / FVector3::Dot(ray.direction, triangle.normal);

	//Within range
	if (t < ray.min || t > ray.max)
		return false;

	//Inside Triangle Check
	//Calculate intersection point on parallelogram
	FVector3 p = ray.origin + t * ray.direction;

	//Check if this point is inside the triangle! Called the Inside-Outside test.
	FVector3 c = {};
	FVector3 side = {};
	FVector3 pointToSide = {};

	//Edge A
	side = triangle.v1 - triangle.v0;
	pointToSide = p - triangle.v0;
	c = FVector3::Cross(side, pointToSide);
	if (FVector3::Dot(triangle.normal, c) < 0.f) //point is on the right side, so outside the triangle
		return false;
	//Edge B
	side = triangle.v2 - triangle.v1;
	pointToSide = p - triangle.v1;
	c = FVector3::Cross(side, pointToSide);
	if (FVector3::Dot(triangle.normal, c) < 0.f) //point is on the right side, so outside the triangle
		return false;
	//Edge C
	side = triangle.v0 - triangle.v2;
	pointToSide = p - triangle.v2;
	c = FVector3::Cross(side, pointToSide);
	if (FVector3::Dot(triangle.normal, c) < 0.f) //point is on the right side, so outside the triangle
		return false;

	
	return true;


}

//__device__ bool IntersectAABB(const AABB& box, const Ray& ray)
//{
//	float tx1 = (box.transformedMinAABB.x - ray.origin.x) / ray.direction.x;
//	float tx2 = (box.transformedMaxAABB.x - ray.origin.x) / ray.direction.x;
//
//	float tmin = fminf(tx1, tx2);
//	float tmax = fmaxf(tx1, tx2);
//
//	float ty1 = (box.transformedMinAABB.y - ray.origin.y) / ray.direction.y;
//	float ty2 = (box.transformedMaxAABB.y - ray.origin.y) / ray.direction.y;
//
//	tmin = fmaxf(tmin, fminf(ty1, ty2));
//	tmax = fminf(tmax, fmaxf(ty1, ty2));
//
//	float tz1 = (box.transformedMinAABB.z - ray.origin.z) / ray.direction.z;
//	float tz2 = (box.transformedMaxAABB.z - ray.origin.z) / ray.direction.z;
//
//	tmin = fmaxf(tmin, fminf(tz1, tz2));
//	tmax = fminf(tmax, fmaxf(tz1, tz2));
//
//	return tmax > 0 && tmax >= tmin;
//}
__device__ float FBoundingBoxHit(const AABB& box, const Ray& ray)
{
	atomicAdd(&totalInterSectionCounter, 1);
	atomicAdd(&boxInterSectionCounter, 1);
	float tx1 = (box.minAABB.x - ray.origin.x) / ray.direction.x;
	float tx2 = (box.maxAABB.x - ray.origin.x) / ray.direction.x;

	float tmin = fminf(tx1, tx2);
	float tmax = fmaxf(tx1, tx2);

	float ty1 = (box.minAABB.y - ray.origin.y) / ray.direction.y;
	float ty2 = (box.maxAABB.y - ray.origin.y) / ray.direction.y;

	tmin = fmaxf(tmin, fminf(ty1, ty2));
	tmax = fminf(tmax, fmaxf(ty1, ty2));

	float tz1 = (box.minAABB.z - ray.origin.z) / ray.direction.z;
	float tz2 = (box.maxAABB.z - ray.origin.z) / ray.direction.z;

	tmin = fmaxf(tmin, fminf(tz1, tz2));
	tmax = fminf(tmax, fmaxf(tz1, tz2));

	if (tmax >= tmin && tmin < ray.max && tmax > 0)
	{
		return tmin;
	}
	else return 1e30f;
	

}
__device__ bool BoundingBoxHit( AABB box,  Ray ray)
{
	atomicAdd(&totalInterSectionCounter, 1);
	atomicAdd(&boxInterSectionCounter, 1);
	float tx1 = (box.minAABB.x - ray.origin.x) / ray.direction.x;
	float tx2 = (box.maxAABB.x - ray.origin.x) / ray.direction.x;

	float tmin = fminf(tx1, tx2);
	float tmax = fmaxf(tx1, tx2);

	float ty1 = (box.minAABB.y - ray.origin.y) / ray.direction.y;
	float ty2 = (box.maxAABB.y - ray.origin.y) / ray.direction.y;

	tmin = fmaxf(tmin, fminf(ty1, ty2));
	tmax = fminf(tmax, fmaxf(ty1, ty2));

	float tz1 = (box.minAABB.z - ray.origin.z) / ray.direction.z;
	float tz2 = (box.maxAABB.z - ray.origin.z) / ray.direction.z;

	tmin = fmaxf(tmin, fminf(tz1, tz2));
	tmax = fminf(tmax, fmaxf(tz1, tz2));

	return tmax > 0 && tmax >= tmin && tmin < ray.max;
}
__device__ bool BoundingBoxHit(const CTriangleMesh& mesh , const Ray& ray)
{
	atomicAdd(&totalInterSectionCounter, 1);
	atomicAdd(&boxInterSectionCounter, 1);
	//atomicAdd(&intersectionCounter, 1);
	// 
	//__syncthreads();
	float tx1 = (mesh.boundingVolume.minAABB.x - ray.origin.x) / ray.direction.x;
	float tx2 = (mesh.boundingVolume.maxAABB.x - ray.origin.x) / ray.direction.x;

	float tmin = fminf(tx1, tx2);
	float tmax = fmaxf(tx1, tx2);

	float ty1 = (mesh.boundingVolume.minAABB.y - ray.origin.y) / ray.direction.y;
	float ty2 = (mesh.boundingVolume.maxAABB.y - ray.origin.y) / ray.direction.y;

	tmin = fmaxf(tmin, fminf(ty1, ty2));
	tmax = fminf(tmax, fmaxf(ty1, ty2));

	float tz1 = (mesh.boundingVolume.minAABB.z - ray.origin.z) / ray.direction.z;
	float tz2 = (mesh.boundingVolume.maxAABB.z - ray.origin.z) / ray.direction.z;

	tmin = fmaxf(tmin, fminf(tz1, tz2));
	tmax = fminf(tmax, fmaxf(tz1, tz2));

	return tmax > 0 && tmax >= tmin && tmin < ray.max;
}
__device__ void IntersectMesh(const Ray& ray ,const CTriangleMesh& mesh , HitRecord& closestHit)
{
	HitRecord tempHit;
	/*if (BoundingBoxHit(mesh, ray))
	{*/
			for (int j = 0; j < mesh.triangleCount; j++)
			{
				const Triangle& Current = mesh.triangles[j];
				
				if (MollerTriangleHit(Current, ray, tempHit))
				{
					if (tempHit.didHit && tempHit.t <= closestHit.t)
						closestHit = tempHit;
				}

			}
		
	/*}*/
	
}
__device__ bool DoesHitMesh(const Ray& ray, const CTriangleMesh& meshes)
{
	if (BoundingBoxHit(meshes, ray))//DoesSphereHit(meshes.boundingSphere,ray)
	{
		for (size_t j = 0; j < meshes.triangleCount; j++)
		{
			if (DoesTriangleHit(meshes.triangles[j], ray))
			{
				return true;
			}
		}
	}
	return false;
}

__device__ void TraverseBVH(const Ray& ray, HitRecord& closest,  BVHNode* tree)
{
	const BVHNode* node = &tree[rootNodeIdx],*stack[64];
	unsigned int stackPtr = 0;
	while (1)
	{
		if (node->primCount > 0)
		{
			for (int i = 0; i < node->primCount; i++)
			{
				IntersectMesh(ray, meshes[MeshIdx[node->firstPrim + i]], closest);
			}
			if (stackPtr == 0)break; else node = stack[--stackPtr];
			continue;
		}
		BVHNode* child1 = &tree[node->left];
		BVHNode* child2 = &tree[node->left +1];

		float dist1 = FBoundingBoxHit(child1->boundingVolume, ray);
		float dist2 = FBoundingBoxHit(child2->boundingVolume, ray);

		if (dist1 > dist2)
		{
			float temp = dist1;
			dist1 = dist2;
			dist2 = temp;
			//swapFloat_device(dist1, dist2);

			BVHNode* tempC = child1;
			child1 = child2;
			child2 = tempC;
			//swap_device(child1, child2);
		}
		if (dist1 == 1e30f)
		{
			if (stackPtr == 0) break;
			else node = stack[--stackPtr];
		}
		else
		{
			node = child1;
			if (dist2 != 1e30f) stack[stackPtr++] = child2;
		}
		
	}

	//if (!BoundingBoxHit(node->boundingVolume, ray))
	//{
	//	return;
	//}

	//if (node->primCount > 0)
	//{
	//	for (int i = 0; i < node->primCount; i++)
	//	{
	//		IntersectMesh(ray, meshes[MeshIdx[node->firstPrim + i]], closest);

	//	}
	//	//return;
	//}
	//else
	//{
	//	TraverseBVH(ray, node->left, closest);
	//	TraverseBVH(ray, node->left + 1, closest);
	//}
}
__device__ bool IntersectBVH(const Ray& ray, const unsigned int nodeIdx)
{
	BVHNode& node = bvhNode[nodeIdx];

	if (!BoundingBoxHit(node.boundingVolume, ray))
	{
		return false;
	}

	if (node.primCount > 0)
	{
		for (int i = 0; i < node.primCount; i++)
		{
			if (DoesHitMesh(ray, meshes[MeshIdx[node.firstPrim + i]]))
			{
				return true;
			}
		}
		
	}
	else
	{
		IntersectBVH(ray, node.left);
		IntersectBVH(ray, node.left + 1);
	}
	return false;
}
__device__ void TraverseBVH(const BVHNode* tree, const Ray& ray, const unsigned int nodeIdx, HitRecord& closest,const CTriangleMesh* _meshes)
{
	//BVHNode* node = &bvhNode[nodeIdx];
	//BVHNode* node = &bvhNode[nodeIdx];
	const BVHNode* node = &tree[nodeIdx];

	if (!BoundingBoxHit(node->boundingVolume, ray))
	{
		return;
	}
	
	if (node->primCount > 0)
	{
		
		
			IntersectMesh(ray, _meshes[MeshIdx[node->firstPrim]], closest);
		
	//	/*if (BoundingBoxHit(meshes[MeshIdx[node.firstPrim]], ray))
	//	{*/
	//	if (node->firstPrim > 0 && node->firstPrim < MeshCount)
	//	{
	//		for (int j = 0; j < meshes[MeshIdx[node->firstPrim]].triangleCount; j++)
	//		{

	//			//meshes[MeshIdx[node.firstPrim]].triangles[j].mat.m_Albedo = ColorRGB{1,1,1};
	//			//meshes[MeshIdx[i]].triangles[j]
	//			auto temp = meshes[MeshIdx[node->firstPrim]].triangles[j];
	//			auto tempRay = ray;
	//			//if (MollerTriangleHit(temp,tempRay,tempHit))
	//			//{
	//			//	if (tempHit.didHit && tempHit.t <= closest.t)
	//			//		closest = tempHit;
	//			//}
	//			//HitRecord tempHit = MollerTriangleHit(&meshes[MeshIdx[node->firstPrim]].triangles[j], ray);
	//			//if (tempHit.didHit && tempHit.t <= closest.t)
	//			//{
	//			//	/*closest.didHit = tempHit.didHit;
	//			//	closest.t = tempHit.t;
	//			//	closest.mat = tempHit.mat;
	//			//	closest.origin = tempHit.origin;
	//			//	closest.normal = FVector3{1,0,0};*/
	//			//}
	//		}

	//	

	//	}
	//	
	}
	else
	{
		if (node->left == 0 || node->left > nodesUsed)
		{
			return;
		}
		TraverseBVH(tree, ray, node->left, closest, _meshes);
		if (node->left + 1 >= (MeshCount*2-1))
		{
			return;
		}
		TraverseBVH(tree, ray, node->left + 1, closest, _meshes);
	}
}
__device__ void GetClosestHitMesh(const Ray& ray, HitRecord& closesthit)
{
	//clock_t start = clock();
	HitRecord tempHit;

	for (int i = 0; i < MeshCount; i++)
	{
		if (BoundingBoxHit(meshes[MeshIdx[i]], ray))
		{
			for (int j = 0; j < meshes[MeshIdx[i]].triangleCount; j++)
			{
				if (MollerTriangleHit(meshes[MeshIdx[i]].triangles[j], ray, tempHit))
				{
					if (tempHit.didHit && tempHit.t <= closesthit.t)
						closesthit = tempHit;
				}

			}

		}
	
	}
		
	/*
	}*/
}



__device__ bool DoesHit(const Ray& ray)
{
	for (size_t j = 0; j < MeshCount; j++)
	{
		if (DoesHitMesh(ray, meshes[j]))
		{
			return true;
		}
	}
	return false;
}



__global__ void TraceRayMesh(BVHNode * bvhTree , ColorRGB* pBackBufferPixels, Camera* cam ,  CTriangleMesh* _meshes)
{
	size_t c = (size_t)blockIdx.x * blockDim.x + threadIdx.x;
	size_t r = (size_t)blockIdx.y * blockDim.y + threadIdx.y;
	const CTriangleMesh* meshes = _meshes;
	//FVector2 sspos = GetScreenSpace(c, r, 4 / 3, cam.fovAngle);
	const float fovAngle = cam->fovAngle * (M_PI / 180.f);
	const float fov = tan(fovAngle / 2.f);

	auto cameraPosition = cam->origin;
	float rx = c + 0.5f;
	float ry = r + 0.5f;

	float cx = (2 * (rx / float(1280)) - 1) * 1.778f * fov;
	float cy = (1 - (2 * (ry / float(720)))) * fov;


	FVector3 rayDirection = FVector3(cx, cy, 1);
	rayDirection = cam->cameraToWorld.TransformVector(rayDirection).Normalized();

	const Ray ray{ cameraPosition,rayDirection };

	HitRecord closestHit{};
	ColorRGB finalColor{0,0,0};

	
	//GetClosestHitMesh(ray, closestHit);
	 TraverseBVH(ray, closestHit,bvhTree);
	//TraverseBVH(bvhTree, ray,rootNodeIdx, closestHit, meshes);
	
	if (closestHit.didHit)
	{
	
		for (size_t i = 0; i < lightCount; i++)
		{
			const FVector3 offsetPoint = closestHit.origin + (closestHit.normal * 0.01f);
			FVector3 invLightDir = GetDirectionToLight(lights[i], offsetPoint);
			const float lightDistance = invLightDir.Normalize();
			Ray lightRay = { offsetPoint, invLightDir, 0.0001f, lightDistance };

		/*	if (DoesHit(lightRay))
			{
				continue;
			}*/
			/*if (IntersectBVH(lightRay,rootNodeIdx))
			{
				continue;
			}*/
			
			const float observedArea = FVector3::Dot(closestHit.normal, invLightDir);
			if (observedArea <= 0.f) continue;

			const ColorRGB lightRadiance = GetRadiance(lights[i], closestHit.origin);
			const ColorRGB brdf = Shade(closestHit, lightRay.direction, -ray.direction);

			finalColor += brdf * lightRadiance * observedArea;
		}
	}
	
	finalColor.MaxToOne();


	pBackBufferPixels[c + (r * 1280)] = finalColor;
	totalIntersectionCount[0] = totalInterSectionCounter;
	boxIntersectionCount[0] = boxInterSectionCounter;
	//sphereIntersectionCount[0] = sphereInterSectionCounter;
	triangleIntersectionCount[0] = triangleInterSectionCounter;
	FPIntersectionCount[0] = FPInterSectionCounter;
	//TriangleTime[0] = triangleTiming;
	//intersectionAmount[0] = intersectionCounter;
}

__global__ void SetTestingVariables(int* totalIntersection, int* BoxIntersection, int* triangleIntersection, int* FPIntersection)
{
	totalIntersectionCount = totalIntersection;
	boxIntersectionCount = BoxIntersection;
	triangleIntersectionCount = triangleIntersection;
	/*TriangleTime = triangleTimings;*/
	//sphereIntersectionCount = SphereIntersections;
	FPIntersectionCount = FPIntersection;
	//TriangleTime = triangleTimings;
}
Renderer::Renderer(SDL_Window* pWindow,SDL_Renderer* pRenderer ,Camera* cam)
{
	//cudaError_t cudaStatus;
	m_pWindow = pWindow;
	m_pRenderer = pRenderer;
	m_pFrontBuffer = SDL_GetWindowSurface(pWindow);
	
	int width, height = 0;
	SDL_GetWindowSize(pWindow, &width, &height);
	m_Width = static_cast<uint32_t>(width);
	m_Height = static_cast<uint32_t>(height);

	m_Camera = cam;

	// Allocate host memory and initialize host data
	m_pBackBuffer = SDL_CreateRGBSurface(0, m_Width, m_Height, 32, 0, 0, 0, 0);
	m_pBackBufferPixels = (uint32_t*)m_pBackBuffer->pixels;

	m_Color2BackBufferPixels = (ColorRGB*)malloc(sizeof(ColorRGB) * m_Width * m_Height);
	m_HostIC = new int{ 0 };
	
	m_BVHHost = new BVHNode[m_MMeshCount * 2 - 1];

	m_TotalInterSectionCountHost = (int*)malloc(sizeof(int));
	m_TotalInterSectionCountHost[0] = 0;

	m_BoxInterSectionCountHost = (int*)malloc(sizeof(int));
	m_BoxInterSectionCountHost[0] = 0;

	//m_SphereInterSectionCountHost = (int*)malloc(sizeof(int));
	//m_SphereInterSectionCountHost[0] = 0;

	m_TriangleInterSectionCountHost = (int*)malloc(sizeof(int));
	m_TriangleInterSectionCountHost[0] = 0;

	/*m_TriangleIntersectionTimeHost = (float*)malloc(sizeof(float));
	m_TriangleIntersectionTimeHost[0] = 0.0f;*/

	m_FalsePositiveInterSectionCountHost = (int*)malloc(sizeof(float));
	m_FalsePositiveInterSectionCountHost[0] = 0.0f;

	m_MeshIdxHost = (int*)malloc(sizeof(int) * m_MMeshCount);

	for (int i = 0; i < m_MMeshCount; i++)
	{
		m_MeshIdxHost[i] = i;
	}
	CreateSceneObjects();
	// ALLOCATE DEVICE MEMORY
	cudaError_t cudaStatus;

	AllocateCudaMemory();
	HostToDeviceMemcpy();
	SetTestingVariables << <1, 1 >> > (m_TotalInterSectionCountDevice, m_BoxInterSectionCountDevice, m_TriangleInterSectionCountDevice, m_FalsePositiveInterSectionCountDevice);
	cudaStatus = cudaGetLastError();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, " Test launch failed: %s\n", cudaGetErrorString(cudaStatus));
	}
	cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaDeviceSynchronize returned error code %d after launching  Test!\n", cudaGetErrorString(cudaStatus));
	}
	
	dim3 block(1, 1, 1);
	dim3 grid(1,1, 1);
	BenchMarker::GetInstance().StartBuildBenchMark();
	BuildBVH<<<grid,block>>>(m_BVHTree,m_MDeviceMeshes, m_MMeshCount, m_DeviceLights , m_LightCount , m_MeshIdx);
	//BenchMarker::GetInstance().StopBuildBenchMark();

	//cudaError_t cudaStatus;
	cudaStatus = cudaGetLastError();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "BUILD BVH launch failed: %s\n", cudaGetErrorString(cudaStatus));
	}
	cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaDeviceSynchronize returned error code %d after launching BVH BUILD!\n", cudaGetErrorString(cudaStatus));
	}
	BenchMarker::GetInstance().StopBuildBenchMark();
	std::cout << "Build is Done\n";
	// cuda malloc for backbuffer

	DeviceToHostMemcpy();
	//for (size_t i = 0; i < m_MMeshCount * 2 - 1; i++)
	//{
	//	auto temp = m_BVHHost[i];
	//	std::cout <<"[" << i << "]" << temp.primCount << "," << temp.firstPrim << "," << temp.left << std::endl;

	//	//std::cout << temp.primCount << std::endl;

	//}
	//for (size_t i = 0; i < m_MMeshCount; i++)
	//{
	//	auto temp = m_MeshIdxHost[i];
	//	std::cout << "[" << i << "]"<< temp << std::endl;
	//	//std::cout << temp.primCount << std::endl;

	//}
	//std::cout << std::endl;
	
}
void Renderer::Render()
{
	
	m_Camera->CalculateCameraToWorld();
	SDL_LockSurface(m_pBackBuffer);
	HostToDeviceMemcpy();
	
	
	TraceRayKernelLaunch();


	// Device To Host of backbuffer and backBufferPixels
	DeviceToHostMemcpy();
	concurrency::parallel_for(0u, m_Width * m_Height, [=](int i) {
		m_pBackBufferPixels[i] = SDL_MapRGB(m_pBackBuffer->format,
			static_cast<uint8_t>(m_Color2BackBufferPixels[i].r * 255),
			static_cast<uint8_t>(m_Color2BackBufferPixels[i].g * 255),
			static_cast<uint8_t>(m_Color2BackBufferPixels[i].b * 255));
		});
	BenchMarker::GetInstance().AddInterSectionAmount(m_TotalInterSectionCountHost[0]);
	BenchMarker::GetInstance().AddInterSectionAmountBox(m_BoxInterSectionCountHost[0]);
	BenchMarker::GetInstance().AddInterSectionAmountTriangle(m_TriangleInterSectionCountHost[0]);

	BenchMarker::GetInstance().AddInterSectionAmountFP(m_FalsePositiveInterSectionCountHost[0]);
//	
	//std::cout << m_TriangleIntersectionTimeDevice[0] << std::endl;
	SDL_UnlockSurface(m_pBackBuffer);
	
	SDL_BlitSurface(m_pBackBuffer, 0, m_pFrontBuffer, 0);
	SDL_UpdateWindowSurface(m_pWindow);

}
void Raytracing::Renderer::Update(Timer* pTimer)
{

	const auto yawAngle = ((sin(pTimer->GetTotal()) + 1.f) / 2.f * PI_2 * pTimer->GetElapsed());
	const float translationX = (sin(pTimer->GetTotal())) / 2.f * PI_2 * pTimer->GetElapsed();
	//m_DHostMeshes[1].Translate({ translationX,0,0 });
	for (size_t i = 0; i < m_pMeshes.size(); i++)
	{
		
		//m_DHostMeshes[i].RotateY(yawAngle);
		m_DHostMeshes[i].UpdateTransforms();
		m_DHostMeshes[i].UpdateAABB();
	//	m_DHostMeshes->CalculateBoundingSphere();
		
		m_MHostMeshes[i].boundingVolume = m_DHostMeshes[i].boundingVolume;
	//	m_MHostMeshes[i].boundingSphere = m_DHostMeshes[i].boundingSphere;
		m_MHostMeshes[i].triangleCount = m_DHostMeshes[i].triangleCount;
		cudaError_t cudaStatus;
		cudaStatus = cudaMemcpy(m_MHostMeshes[i].triangles, m_DHostMeshes[i].triangles, sizeof(Triangle) * m_DHostMeshes[i].triangleCount, cudaMemcpyHostToDevice);
		if (cudaStatus != cudaSuccess) {
			//fprintf(stderr, "Triangles \n", cudaGetErrorString(cudaStatus));
		}
	
	}
	RefitBVH <<<1,1>>>();
}
void Raytracing::Renderer::TraceRayKernelLaunch()
{
	cudaError_t cudaStatus;


	dim3 block(8, 8, 1);
	dim3 grid(ceil((float)m_Width / block.x), ceil((float)m_Height / block.y), 1);
	
	//TraceRay <<<grid, block >>> (m_pCuda2BackBufferPixels, m_DevSpheres, m_SphereCount, m_DevPlanes, m_PlaneCount, m_DeviceTriangles, m_TriangleCount, m_DeviceLights, m_LightCount, m_CudaCamera);
	TraceRayMesh<<<grid,block>>> (m_BVHTree, m_pCuda2BackBufferPixels, m_CudaCamera , m_MDeviceMeshes);
	// Check if Kernel Launch failed and if device Sync fails
	cudaStatus = cudaGetLastError();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "TraceRay launch failed: %s\n", cudaGetErrorString(cudaStatus));
	}
	cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaDeviceSynchronize returned error code %d after launching TraceRay Kernel!\n", cudaStatus);
	}

}
void Raytracing::Renderer::AllocateCudaMemory()
{
	cudaError_t cudaStatus;
	cudaStatus = cudaMalloc((void**)&m_BVHTree, sizeof(BVHNode) * (m_MMeshCount * 2 - 1));
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr," BVH malloc failed: %s\n", cudaGetErrorString(cudaStatus));
	}
	cudaStatus = cudaMalloc((void**)&m_MeshIdx, sizeof(int) * m_MMeshCount);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, " MeshIdx malloc failed: %s\n", cudaGetErrorString(cudaStatus));
	}
	/*cudaStatus = cudaMalloc((void**)&m_IntersectionCounter, sizeof(int));
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "CudaMalloc failed! ");
	}*/

	// cuda malloc for backbuffer pixels
	cudaStatus = cudaMalloc((void**)&m_pCuda2BackBufferPixels, sizeof(ColorRGB) * m_Width * m_Height);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "CudaMalloc failed! ");
	}
	//cudaStatus = cudaMalloc((void**)&m_DevSpheres, sizeof(Sphere) * m_SphereCount);
	//if (cudaStatus != cudaSuccess) {
	//	fprintf(stderr, "CudaMalloc failed! ");
	//}
	/*cudaStatus = cudaMalloc((void**)&m_DevPlanes, sizeof(Plane) * m_PlaneCount);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "CudaMalloc failed! ");
	}
	*/
	cudaStatus = cudaMalloc((void**)&m_TotalInterSectionCountDevice, sizeof(int));
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "CudaMalloc failed! ");
	}
	cudaStatus = cudaMalloc((void**)&m_BoxInterSectionCountDevice, sizeof(int));
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "CudaMalloc failed! ");
	}
	//cudaStatus = cudaMalloc((void**)&m_SphereInterSectionCountDevice, sizeof(int));
	//if (cudaStatus != cudaSuccess) {
	//	fprintf(stderr, "CudaMalloc failed! ");
	//}
	cudaStatus = cudaMalloc((void**)&m_TriangleInterSectionCountDevice, sizeof(int));
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "CudaMalloc failed! ");
	}
	//cudaStatus = cudaMalloc((void**)&m_TriangleIntersectionTimeDevice, sizeof(float));
	//if (cudaStatus != cudaSuccess) {
	//	fprintf(stderr, "CudaMalloc failed! ");
	//}
	cudaStatus = cudaMalloc((void**)&m_FalsePositiveInterSectionCountDevice, sizeof(float));
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "CudaMalloc failed! ");
	}

	
	
	cudaStatus = cudaMalloc((void**)&m_DeviceLights, sizeof(Light) * m_LightCount);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "CudaMalloc failed! ");
	}
	cudaStatus = cudaMalloc((void**)&m_CudaCamera, sizeof(Camera));
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "CudaMalloc failed! ");
	}

}
void Raytracing::Renderer::HostToDeviceMemcpy()
{
	cudaError_t cudaStatus;
	/*cudaStatus = cudaMemcpy(m_IntersectionCounter, m_HostIC, sizeof(int), cudaMemcpyHostToDevice);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaMemCpy counter failed!\n", cudaGetErrorString(cudaStatus));
	}*/
	cudaStatus = cudaMemcpy(m_BVHTree, m_BVHHost, sizeof(BVHNode) * (m_MMeshCount * 2 - 1), cudaMemcpyHostToDevice);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaMemCpy bvh to device failed!\n", cudaGetErrorString(cudaStatus));
	}
	cudaStatus = cudaMemcpy(m_MeshIdx, m_MeshIdxHost, sizeof(int) * m_MMeshCount, cudaMemcpyHostToDevice);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaMemCpy bvh to device failed!\n", cudaGetErrorString(cudaStatus));
	}
	cudaStatus = cudaMemcpy(m_pCuda2BackBufferPixels, m_Color2BackBufferPixels, sizeof(ColorRGB) * m_Width * m_Height, cudaMemcpyHostToDevice);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaMemCpy backbufferpixels failed!\n", cudaGetErrorString(cudaStatus));
	}
	cudaStatus = cudaMemcpy(m_TotalInterSectionCountDevice, m_TotalInterSectionCountHost, sizeof(int), cudaMemcpyHostToDevice);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaMemCpy intersections failed!\n", cudaGetErrorString(cudaStatus));
	}
	cudaStatus = cudaMemcpy(m_BoxInterSectionCountDevice, m_BoxInterSectionCountHost, sizeof(int), cudaMemcpyHostToDevice);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaMemCpy intersections failed!\n", cudaGetErrorString(cudaStatus));
	}
	/*cudaStatus = cudaMemcpy(m_SphereInterSectionCountDevice, m_SphereInterSectionCountHost, sizeof(int), cudaMemcpyHostToDevice);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaMemCpy intersections failed!\n", cudaGetErrorString(cudaStatus));
	}*/
	cudaStatus = cudaMemcpy(m_TriangleInterSectionCountDevice, m_TriangleInterSectionCountHost, sizeof(int), cudaMemcpyHostToDevice);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaMemCpy intersections failed!\n", cudaGetErrorString(cudaStatus));
	}
	//cudaStatus = cudaMemcpy(m_TriangleIntersectionTimeDevice, m_TriangleIntersectionTimeHost, sizeof(float), cudaMemcpyHostToDevice);
	//if (cudaStatus != cudaSuccess) {
	//	fprintf(stderr, "cudaMemCpy intersections failed!\n", cudaGetErrorString(cudaStatus));
	//}
	cudaStatus = cudaMemcpy(m_FalsePositiveInterSectionCountDevice, m_FalsePositiveInterSectionCountHost, sizeof(float), cudaMemcpyHostToDevice);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaMemCpy intersections failed!\n", cudaGetErrorString(cudaStatus));
	}

	cudaStatus = cudaMemcpy(m_MDeviceMeshes, m_MHostMeshes, sizeof(CTriangleMesh) * m_MMeshCount, cudaMemcpyHostToDevice);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaMemCpy meshes to device failed!\n", cudaGetErrorString(cudaStatus));
	}
	
	cudaStatus = cudaMemcpy(m_DeviceLights, m_HostLights, sizeof(Light) * m_LightCount, cudaMemcpyHostToDevice);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaMemCpy lights failed!\n", cudaGetErrorString(cudaStatus));
	}
	cudaStatus = cudaMemcpy(m_CudaCamera, m_Camera, sizeof(Camera) , cudaMemcpyHostToDevice);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaMemCpy camera failed!\n", cudaGetErrorString(cudaStatus));
	}
	
}
void Raytracing::Renderer::DeviceToHostMemcpy()
{
	cudaError_t cudaStatus;

	/*cudaStatus = cudaMemcpy(m_HostIC, m_IntersectionCounter, sizeof(int) , cudaMemcpyDeviceToHost);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaMemCpy to device failed! \n", cudaGetErrorString(cudaStatus));
	}*/
	cudaStatus = cudaMemcpy(m_BVHHost, m_BVHTree, sizeof(BVHNode) * (m_MMeshCount * 2 - 1), cudaMemcpyDeviceToHost);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaMemCpy BVH to host failed!\n", cudaGetErrorString(cudaStatus));
	}
	cudaStatus = cudaMemcpy(m_MeshIdxHost, m_MeshIdx, sizeof(int) * m_MMeshCount, cudaMemcpyDeviceToHost);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaMemCpy BVH to host failed!\n", cudaGetErrorString(cudaStatus));
	}
	cudaStatus = cudaMemcpy(m_Color2BackBufferPixels, m_pCuda2BackBufferPixels, sizeof(ColorRGB) * m_Width * m_Height, cudaMemcpyDeviceToHost);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaMemCpy to device failed! \n", cudaGetErrorString(cudaStatus));
	}
	
		
	cudaStatus = cudaMemcpy(m_TotalInterSectionCountHost, m_TotalInterSectionCountDevice, sizeof(int), cudaMemcpyDeviceToHost);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaMemCpy backbufferpixels failed!\n", cudaGetErrorString(cudaStatus));
	}
	cudaStatus = cudaMemcpy(m_BoxInterSectionCountHost, m_BoxInterSectionCountDevice, sizeof(int), cudaMemcpyDeviceToHost);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaMemCpy backbufferpixels failed!\n", cudaGetErrorString(cudaStatus));
	}
	//cudaStatus = cudaMemcpy(m_SphereInterSectionCountHost, m_SphereInterSectionCountDevice, sizeof(int), cudaMemcpyDeviceToHost);
	//if (cudaStatus != cudaSuccess) {
	//	fprintf(stderr, "cudaMemCpy backbufferpixels failed!\n", cudaGetErrorString(cudaStatus));
	//}
	cudaStatus = cudaMemcpy(m_TriangleInterSectionCountHost, m_TriangleInterSectionCountDevice, sizeof(int), cudaMemcpyDeviceToHost);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaMemCpy backbufferpixels failed!\n", cudaGetErrorString(cudaStatus));
	}
	/*cudaStatus = cudaMemcpy(m_TriangleIntersectionTimeHost, m_TriangleIntersectionTimeDevice, sizeof(float), cudaMemcpyDeviceToHost);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaMemCpy intersections failed!\n", cudaGetErrorString(cudaStatus));
	}*/
	cudaStatus = cudaMemcpy(m_FalsePositiveInterSectionCountHost, m_FalsePositiveInterSectionCountDevice, sizeof(int), cudaMemcpyDeviceToHost);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaMemCpy intersections failed!\n", cudaGetErrorString(cudaStatus));
	}

	cudaStatus = cudaMemcpy(m_MHostMeshes, m_MDeviceMeshes, sizeof(CTriangleMesh) * m_MMeshCount, cudaMemcpyDeviceToHost);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaMemCpy Meshes to host failed!\n", cudaGetErrorString(cudaStatus));
	}
	
	cudaStatus = cudaMemcpy(m_HostLights, m_DeviceLights, sizeof(Light) * m_LightCount, cudaMemcpyDeviceToHost);
	if (cudaStatus != cudaSuccess) {
		//fprintf(stderr, "cudaMemCpy backbufferpixels failed!\n", cudaGetErrorString(cudaStatus));
	}

	cudaStatus = cudaMemcpy(m_Camera, m_CudaCamera, sizeof(Camera), cudaMemcpyDeviceToHost);
	if (cudaStatus != cudaSuccess) {
		//fprintf(stderr, "cudaMemCpy backbufferpixels failed!\n", cudaGetErrorString(cudaStatus));
	}

}

void Raytracing::Renderer::CreateSceneObjects()
{
	
	//CreateSceneObjectsNone();
	CreateSceneObjectsBV();
	
	for (int i = 0; i < m_MMeshCount ; i++)
	{
		float x =static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
		float y = static_cast <float> (rand()) / static_cast <float> (RAND_MAX) ;
		float z =static_cast <float> (rand()) / static_cast <float> (RAND_MAX)  ;
		float s =static_cast <float> (rand()) / static_cast <float> (RAND_MAX)  ;
		//m_DHostMeshes[i].Translate(FVector3{ 0,0,0 });
		//m_DHostMeshes[i].Scale(FVector3{0.1f });
		//m_DHostMeshes[i].UpdateTransforms(); 
		//m_DHostMeshes[i].Translate(FVector3{ 5.f * (i%8) , 3.f * ( i/8 %8)  , 5.f * (i / 64 % 8)});
	/*	m_DHostMeshes[i].Translate(FVector3{ 0,0,0});
		m_DHostMeshes[i].Scale(FVector3{ s*3 });
		m_DHostMeshes[i].UpdateTransforms();*/
		//m_DHostMeshes[i].Translate(FVector3{ -16.f + 2 * (i%4) , -2.f+  2.f * ( i/4 %4)  , 2.f * (i / 16 % 4) });
		m_DHostMeshes[i].Translate(FVector3{0.f + x * 32.f , 0+ y * 32.f, 8 +z * 32.f });
		
		m_DHostMeshes[i].UpdateTransforms();
		
		m_DHostMeshes[i].UpdateAABB();
		m_DHostMeshes[i].SetMaterial( PBRMaterial{ {0.6,.8,1},0.3,1 });
	}

	for (size_t i = 0; i < m_pMeshes.size(); i++)
	{
		//m_DHostMeshes[i].UpdateAABB();
		m_MHostMeshes[i].boundingVolume.minAABB = m_DHostMeshes[i].boundingVolume.minAABB;
		m_MHostMeshes[i].boundingVolume.maxAABB = m_DHostMeshes[i].boundingVolume.maxAABB;
		m_MHostMeshes[i].boundingVolume.Centroid = m_DHostMeshes[i].boundingVolume.Centroid;
		m_MHostMeshes[i].boundingVolume.transformedMaxAABB = m_DHostMeshes[i].boundingVolume.transformedMaxAABB;
		m_MHostMeshes[i].boundingVolume.transformedMinAABB = m_DHostMeshes[i].boundingVolume.transformedMinAABB;
		cudaError_t cudaStatus;
		cudaStatus = cudaMemcpy(m_MHostMeshes[i].triangles, m_DHostMeshes[i].triangles, sizeof(Triangle) * m_DHostMeshes[i].triangleCount, cudaMemcpyHostToDevice);
		if (cudaStatus != cudaSuccess) {
			fprintf(stderr, "Triangles \n", cudaGetErrorString(cudaStatus));
		}
		

	}
	
	// LIGHTS
	m_HostLights = (Light*)malloc(sizeof(Light) * m_LightCount);
	Light l;
	l.origin = { 50.f,25.f,30.f };
	l.intensity = 50.f;
	l.direction = { 0,-1,-1 };
	l.color = colors::White;
	l.type = LightType::Directional;
	m_HostLights[2] = l;

	l.origin = { 12.f,35.f,0.f };
	l.intensity = 50.f;
	l.direction = { 0,-1,0 };
	l.color = colors::White;
	l.type = LightType::Directional;
	m_HostLights[1] = l;
	
	l.origin = { 12.f,12.f,-10.f };
	l.intensity = 20.f;
	l.direction = { 0,-1,1 };
	l.color = colors::White;
	l.type = LightType::Directional;
	m_HostLights[0] = l;
}

void Raytracing::Renderer::CreateSceneObjectsNone()
{
	//m_HostSpheres = (Sphere*)malloc(sizeof(Sphere) * m_SphereCount);
	//m_HostSpheres[0] = { FVector3{-0.5f, 1.f, 0.f},0.75f ,LambertMaterial{1.f,ColorRGB{1.0f,0.f,0.f}} };
	//m_HostSpheres[1] = { FVector3{0.5f, 1.f, 0.f},0.75f,LambertMaterial{1.f,ColorRGB{0.0f,0.f,1.f}} };

	//// PLANES
	//m_HostPlanes = (Plane*)malloc(sizeof(Plane) * m_PlaneCount);
	//m_HostPlanes[0] = { FVector3{ 0.f, .25f, 0.f }, FVector3{ 0.f, 1.f,0.f } ,LambertMaterial{1.f,ColorRGB{1.0f,1.f,0.f}} };

	////TRIANGLES

	//Triangle triangle = Triangle{ {-.75f,2.5f,.0f},{-.75f,4.f, .0f}, {.75f,2.5f,0.f} };
	//triangle.mode = TriangleCullMode::NoCulling;
	//triangle.mat = LambertMaterial{ 1.f,ColorRGB{1.f,1.f,1.f} };


	//m_HostTriangles = (Triangle*)malloc(sizeof(Triangle) * m_TriangleCount);
	//m_HostTriangles[0] = triangle;
}

void Raytracing::Renderer::CreateSceneObjectsBV()
{

	//CreateFloor(2, 2);

	CreateBunny(m_MMeshCount);
	
	m_MHostMeshes = (CTriangleMesh*)malloc(sizeof(CTriangleMesh) * m_MMeshCount);
	m_DHostMeshes = (CTriangleMesh*)malloc(sizeof(CTriangleMesh) * m_MMeshCount);
	std::vector<Triangle> tempVec;

	srand(static_cast <unsigned> (time(0)));

	for (size_t i = 0; i < m_pMeshes.size(); i++)
	{
		
		Triangle* t = (Triangle*)malloc(sizeof(Triangle) * m_pMeshes[i]->triangleCount);
		for (size_t j = 0; j < m_pMeshes[i]->indices.size(); j += 3)
		{
			Triangle temp;
			temp.v0 = m_pMeshes[i]->positions[m_pMeshes[i]->indices[j]];
			temp.v1 = m_pMeshes[i]->positions[m_pMeshes[i]->indices[j + 1]];
			temp.v2 = m_pMeshes[i]->positions[m_pMeshes[i]->indices[j + 2]];
			float r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			float g = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			float b = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			temp.mat =  PBRMaterial{ColorRGB{r,g,b},0.1f,0.9f };

			tempVec.push_back(temp);
		}
		for (size_t k = 0; k < m_pMeshes[i]->triangleCount; k++)
		{
			t[k] = tempVec[k];
			t[k].normal = m_pMeshes[i]->normals[k];
		//	t[k].mode = TriangleCullMode::NoCulling;
		}
	
		
		m_MHostMeshes[i].triangles = (Triangle*)malloc(sizeof(Triangle) * m_pMeshes[i]->triangleCount);
		m_MHostMeshes[i].triangleCount = m_pMeshes[i]->triangleCount;
		m_MHostMeshes[i].triangles = (Triangle*)memcpy(m_MHostMeshes[i].triangles, t, sizeof(Triangle) * m_pMeshes[i]->triangleCount);
		m_MHostMeshes[i].rotation = m_pMeshes[i]->rotation;
		m_MHostMeshes[i].translation = m_pMeshes[i]->translation;
		m_MHostMeshes[i].scale = m_pMeshes[i]->scale;
		m_MHostMeshes[i].UpdateTransforms();
		m_MHostMeshes[i].UpdateAABB();
		/*m_DHostMeshes[i].UpdateTransforms();
		m_DHostMeshes[i].UpdateAABB();*/
		//m_MHostMeshes[i].CalculateBoundingSphere();
		
		m_DHostMeshes[i] = m_MHostMeshes[i];
		
		cudaMalloc((void**)&m_MHostMeshes[i].triangles, sizeof(Triangle) * m_pMeshes[i]->triangleCount);
		
		cudaMemcpy(m_MHostMeshes[i].triangles, t, sizeof(Triangle) * m_pMeshes[i]->triangleCount, cudaMemcpyHostToDevice);
		/*cudaMalloc((void**)&m_MHostMeshes[i].boundingVolume, sizeof(AABB) );

		cudaMemcpy(&m_MHostMeshes[i].boundingVolume, &m_DHostMeshes[i].boundingVolume, sizeof(AABB) , cudaMemcpyHostToDevice);
	*/
		
		tempVec.clear();
		free(t);
	}
	cudaError_t cudaStatus;
	cudaStatus = cudaMalloc(&m_MDeviceMeshes, sizeof(CTriangleMesh) * m_MMeshCount);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "CudaMalloc failed! ");
	}
	/*cudaStatus = cudaMalloc(&m_MDeviceMeshes->triangles, sizeof(Triangle) * m_MHostMeshes->triangleCount);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "CudaMalloc failed! ");
	}*/
	cudaStatus = cudaMemcpy(m_MDeviceMeshes, m_MHostMeshes, sizeof(CTriangleMesh) * m_MMeshCount, cudaMemcpyHostToDevice);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "CudaMemCpy failed! ");
	}
	
	
}

void Raytracing::Renderer::CreateFloor(int x, int y)
{
	for (size_t i = 0; i < x+y; i++)
	{
		m_pMeshes.push_back(new TriangleMesh());
	}
}

void Raytracing::Renderer::CreateBunny(int x)
{
	for (size_t i = 0; i < x; i++)
	{
		m_pMeshes.push_back(new TriangleMesh("Resources/lowpoly_bunny.obj"));
	}
	
}




