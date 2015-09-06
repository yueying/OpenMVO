/*******************************************************************************
 * 文件： timer.cpp
 * 时间： 2014/11/20 11:00
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 *
 * 说明： 高性能的码表
 *
 ********************************************************************************/

#include "base_precomp.h"  // 预编译头

#include "openmvo/utils/timer.h"

#include <windows.h>

#include <cstring>
#include <cassert>

namespace mvo{
	// 通过宏来方便的将largeInts的地址给LARGE_INTEGER指针，本来largeInts所指向的值占64位，而LARGE_INTEGER占8位，这样就可以通过数组移位的形式保存值
#define	LARGE_INTEGER_NUMS	reinterpret_cast<LARGE_INTEGER*>(largeInts)


	/*---------------------------------------------------------------
	构造函数
	---------------------------------------------------------------*/
	Timer::Timer()
	{
		memset(largeInts, 0, sizeof(largeInts));

		assert(sizeof(largeInts) > 3 * sizeof(LARGE_INTEGER));//这边要保存至少3个LARGE_INTEGER
		LARGE_INTEGER *l = LARGE_INTEGER_NUMS;
		QueryPerformanceFrequency(&l[0]);//返回硬件支持的高精度计数器的频率。

		Start();
	}

	/*---------------------------------------------------------------
	析构函数
	---------------------------------------------------------------*/
	Timer::~Timer()
	{
	}

	/*---------------------------------------------------------------
	启动码表
	---------------------------------------------------------------*/
	void	Timer::Start()
	{
		LARGE_INTEGER *l = LARGE_INTEGER_NUMS;
		QueryPerformanceCounter(&l[1]);//要求计算机从硬件上支持高精度定时器,获得初始值
	}

	/*---------------------------------------------------------------
	暂停码表，计算时间
	---------------------------------------------------------------*/
	double	Timer::Stop()
	{
		LARGE_INTEGER *l = LARGE_INTEGER_NUMS;
		QueryPerformanceCounter(&l[2]);//获得终止值
		return (l[2].QuadPart - l[1].QuadPart) / static_cast<double>(l[0].QuadPart);//获得对应的时间值
	}
}
