#pragma once
#include "Singleton.h"
#include <vector>
#include <chrono>
#include <fstream>
class BenchMarker : public Singleton<BenchMarker>
{
public :
	void SetDeltaTime(float dt);
	float GetDeltaTime() const;


	void StartBenchMark();
	void StopBenchMark();
	void StartBuildBenchMark();
	void StopBuildBenchMark();
	void AddInterSectionAmount(int amount);
	void AddInterSectionAmountTriangle(int amount);
	void AddInterSectionAmountBox(int amount);
	void AddInterSectionAmountSphere(int amount);
	void AddInterSectionAmountFP(int amount);

	void AddTriangleTiming(float time);
	float GetAverage(std::vector<float> timeVec);

	void AddMeshInfo(int meshCount, int meshTriangles);
	void WriteFile();
private:
	friend class Singleton<BenchMarker>;
	BenchMarker() = default;
	
	float m_DeltaTime = 0.0f;

	int m_MeshCountInfo;
	int m_TriangleCountInfo;


	std::vector<float> m_BenchMarkTimesFrames;
	std::vector<float> m_BenchMarkTimesBuild;
	std::vector<float> m_BenchMarkTimesTriangles;
	std::vector<float> m_BenchMarkTimesBox;
	std::vector<float> m_BenchMarkTimesSphere;
	std::vector<float> m_BenchMarkTimesBVHTraversal;
	std::vector<float> m_BenchMarkTimesBVHBuild;

	std::vector<unsigned int> m_AmountOfIntersections;
	std::vector<unsigned int> m_AmountOfTriangleIntersections;
	std::vector<unsigned int> m_AmountOfBoxIntersections;
	std::vector<unsigned int> m_AmountOfSphereIntersections;
	std::vector<unsigned int> m_AmountOfFPIntersections;
	std::vector<unsigned int> m_AmountOfFPBVHIntersections;

	std::chrono::high_resolution_clock::time_point m_StartTime;
	std::chrono::high_resolution_clock::time_point m_StopTime;

	std::chrono::high_resolution_clock::time_point m_StartTimeBuild;
	std::chrono::high_resolution_clock::time_point m_StopTimeBuild;
};