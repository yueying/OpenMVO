/*******************************************************************************
 * 文件： timer.h
 * 时间： 2014/11/09 22:15
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 *
 * 说明： 这个类实现了一个高性能的码表.
 *
 ********************************************************************************/
#ifndef OPENMVO_UTILS_TIMER_H_
#define OPENMVO_UTILS_TIMER_H_

#include <stdio.h>
#ifdef _WIN32
# include <windows.h>
#else
# include <sys/time.h>
#endif
#include <iostream>

namespace mvo
{

	class Timer
	{
	private:
#ifndef _WIN32
		timeval start_time_;
#endif
		double time_;
		double accumulated_;

		double start_;
#ifdef _WIN32
		double frequency_;
#endif
	public:

		/// The constructor directly starts the timer.
		Timer() :
			time_(0.0),
			accumulated_(0.0)
		{
#ifdef _WIN32
			LARGE_INTEGER freq;
			if (!QueryPerformanceFrequency(&freq))
			{
				const char *msg = "Failed to initialize high resolution timer!";
				std::cerr << msg << std::endl;
				throw std::runtime_error(msg);
			}
			frequency_ = static_cast<double>(freq.QuadPart);
#endif
			start();
		}

		~Timer()
		{}

		inline void start()
		{
			accumulated_ = 0.0;
			
#if _WIN32
			LARGE_INTEGER li_start_;
			QueryPerformanceCounter(&li_start_);
			start_ = static_cast<double>(li_start_.QuadPart);
#else
                        gettimeofday(&start_time_, NULL);
#endif
		}

		inline void resume()
		{		
#if _WIN32
			LARGE_INTEGER li_start_;
			QueryPerformanceCounter(&li_start_);
			start_ = static_cast<double>(li_start_.QuadPart);
#else
			gettimeofday(&start_time_, NULL);
#endif
		}

		inline double stop()
		{		
#if _WIN32
			LARGE_INTEGER end_;
			QueryPerformanceCounter(&end_);
			time_ = (static_cast<double>(end_.QuadPart) - start_) / frequency_;
			accumulated_ = time_;
			return time_;
#else
			timeval end_time;
			gettimeofday(&end_time, NULL);
			long seconds  = end_time.tv_sec  - start_time_.tv_sec;
			long useconds = end_time.tv_usec - start_time_.tv_usec;
			time_ = ((seconds) + useconds*0.000001) + accumulated_;
			accumulated_ = time_;
			return time_;
#endif
		}

		inline double getTime() const
		{
			return time_;
		}

		inline void reset()
		{
			time_ = 0.0;
			accumulated_ = 0.0;
		}

	};

} // end namespace mvo

#endif // OPENMVO_UTILS_TIMER_H_
