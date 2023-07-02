#pragma once

#ifndef RAYTRACE_TIMER
#define RAYTRACE_TIMER

//Standard includes
#include <cstdint>

namespace Raytracing
{
	class Timer
	{

	public : 
		Timer();
		virtual ~Timer() = default;

		Timer(const Timer&) = delete;
		Timer(Timer&&) noexcept = delete;
		Timer& operator=(const Timer&) = delete;
		Timer& operator=(Timer&&) noexcept = delete;

		void Reset();
		void Start();
		void Update();
		void Stop();

		uint32_t GetFPS() const { return m_FPS; };
		float GetElapsed() const { return m_ElapsedTime; };
		float GetTotal() const { return m_TotalTime; };
		bool IsRunning() const { return !m_IsStopped; };



		//void WriteData();

	private:
		uint64_t m_BaseTime = 0;
		uint64_t m_PausedTime = 0;
		uint64_t m_StopTime = 0;
		uint64_t m_PreviousTime = 0;
		uint64_t m_CurrentTime = 0;

		uint32_t m_FPS = 0;
		uint32_t m_FPSCount = 0;

		float m_TotalTime = 0.0f;
		float m_ElapsedTime = 0.0f;
		float m_SecondsPerCount = 0.0f;
		float m_ElapsedUpperBound = 0.03f;
		float m_FPSTimer = 0.0f;

		bool m_IsStopped = true;
		bool m_ForceElapsedUpperBound = false;

		//std::vector<float> m_BenchmarkTimes;
	};
}



#endif // RAYTRACE_TIMER

