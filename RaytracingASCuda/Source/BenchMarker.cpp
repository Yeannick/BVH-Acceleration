#include "BenchMarker.h"


using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;

void BenchMarker::SetDeltaTime(float dt)
{
    m_DeltaTime = dt;
}

float BenchMarker::GetDeltaTime() const
{
    return m_DeltaTime;
}

void BenchMarker::StartBenchMark()
{
    m_StartTime = high_resolution_clock::now();

}

void BenchMarker::StopBenchMark()
{
    m_StopTime = high_resolution_clock::now();

    /* Getting number of milliseconds as a float. */
    duration<float, std::milli> ms_float = m_StopTime - m_StartTime;

    m_BenchMarkTimesFrames.push_back(ms_float.count());
}

void BenchMarker::StartBuildBenchMark()
{
    m_StartTimeBuild = high_resolution_clock::now();
}

void BenchMarker::StopBuildBenchMark()
{
    m_StopTimeBuild = high_resolution_clock::now();

    /* Getting number of milliseconds as a float. */
    duration<float, std::milli> ms_float = m_StopTimeBuild - m_StartTimeBuild;

    m_BenchMarkTimesBuild.push_back(ms_float.count());
}

void BenchMarker::AddInterSectionAmount(int amount)
{
    m_AmountOfIntersections.push_back(amount);
}

void BenchMarker::AddInterSectionAmountTriangle(int amount)
{
    m_AmountOfTriangleIntersections.push_back(amount);
}

void BenchMarker::AddInterSectionAmountBox(int amount)
{
    m_AmountOfBoxIntersections.push_back(amount);
}

void BenchMarker::AddInterSectionAmountSphere(int amount)
{
    m_AmountOfSphereIntersections.push_back(amount);
}

void BenchMarker::AddInterSectionAmountFP(int amount)
{
    m_AmountOfFPIntersections.push_back(amount);
}

void BenchMarker::AddTriangleTiming(float time)
{
    m_BenchMarkTimesTriangles.push_back(time);
}

float BenchMarker::GetAverage(std::vector<float> timeVec)
{
    float total = 0.0f;
    int size = timeVec.size();

    for (size_t i = 0; i < timeVec.size(); i++)
    {
        total += timeVec[i];
    }
    return total / size;
}

void BenchMarker::AddMeshInfo(int meshCount, int meshTriangles)
{
}

void BenchMarker::WriteFile()
{
    std::fstream file;
    file.open("Data/TestData.txt", std::ios_base::out);
    file << "-- Mesh Info --" << std::endl;

    file << m_MeshCountInfo << std::endl;
    file << m_TriangleCountInfo << std::endl;
    file << m_MeshCountInfo * m_TriangleCountInfo << std::endl;
    file << "-- Time Avergages --" << std::endl;
    file << GetAverage(m_BenchMarkTimesFrames) << std::endl;
    file << GetAverage(m_BenchMarkTimesTriangles) << std::endl;
    file << "-- Total INTERSECTION AMOUNT --" << std::endl;
    file << m_AmountOfIntersections[0] << std::endl;

    file << "-- Triangle INTERSECTION AMOUNT --" << std::endl;

    file << m_AmountOfTriangleIntersections[0] << std::endl;

    file << "-- Box INTERSECTION AMOUNT --" << std::endl;

    file << m_AmountOfBoxIntersections[0] << std::endl;

    file << "-- FalsePositive INTERSECTION AMOUNT --" << std::endl;

    file << m_AmountOfFPIntersections[0] << std::endl;

    file << "-- FalsePositive BVH INTERSECTION AMOUNT --" << std::endl;
    for (int i = 0; i < m_AmountOfFPBVHIntersections.size(); i++)
    {
        file << m_AmountOfFPBVHIntersections[i] << std::endl;
    }
    file << "-- Sphere INTERSECTION AMOUNT --" << std::endl;

    //file << m_AmountOfSphereIntersections[0] << std::endl;


    /*  file << "-- Render Frame time --" << std::endl;
      for (int i = 0; i < m_BenchMarkTimesFrames.size(); i++)
      {
          file << m_BenchMarkTimesFrames[i] << std::endl;
      }
      file << "-- Triangle Intersection time --" << std::endl;
      for (int i = 0; i < m_BenchMarkTimesTriangles.size(); i++)
      {
          file << m_BenchMarkTimesTriangles[i] << std::endl;
      }*/
    file << "-- BVHBuild time --" << std::endl;
    file << m_BenchMarkTimesBuild[0] << std::endl;
    file << "-- BVHTraversal time --" << std::endl;
    /*  for (int i = 0; i < m_BenchMarkTimesBVHTraversal.size(); i++)
      {
          file << m_BenchMarkTimesBVHTraversal[i] << std::endl;
      }*/
    file.close();
}