/*************************************************************************
 * 文件名： image_utils
 *
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 * 时间： 2015/8/6
 *
 * 说明： 给出图像处理中的相关工具函数
 *************************************************************************/
#ifndef OPENMVO_UTILS_IMAGE_UTILS_H_
#define OPENMVO_UTILS_IMAGE_UTILS_H_

#include <stdint.h>
#if __SSE2__
# include <emmintrin.h>
#endif
#include <opencv2/opencv.hpp>
#include "openmvo/utils/aligned_mem.h"

namespace mvo
{
#ifdef __SSE2__
	inline void halfSampleSSE2(const unsigned char* in, unsigned char* out, int w, int h)
	{
		const unsigned long long mask[2] = { 0x00FF00FF00FF00FFull, 0x00FF00FF00FF00FFull };
		const unsigned char* nextRow = in + w;
		__m128i m = _mm_loadu_si128((const __m128i*)mask);
		int sw = w >> 4;
		int sh = h >> 1;
		for (int i = 0; i < sh; i++)
		{
			for (int j = 0; j < sw; j++)
			{
				__m128i here = _mm_load_si128((const __m128i*)in);
				__m128i next = _mm_load_si128((const __m128i*)nextRow);
				here = _mm_avg_epu8(here, next);
				next = _mm_and_si128(_mm_srli_si128(here, 1), m);
				here = _mm_and_si128(here, m);
				here = _mm_avg_epu16(here, next);
				_mm_storel_epi64((__m128i*)out, _mm_packus_epi16(here, here));
				in += 16;
				nextRow += 16;
				out += 8;
			}
			in += w;
			nextRow += w;
		}
	}
#endif 

	inline void halfSample(const cv::Mat& in, cv::Mat& out)
	{
		assert(in.rows / 2 == out.rows && in.cols / 2 == out.cols);
		assert(in.type() == CV_8U && out.type() == CV_8U);

#ifdef __SSE2__
		if (is_aligned16(in.data) && is_aligned16(out.data) && ((in.cols % 16) == 0))
		{
			halfSampleSSE2(in.data, out.data, in.cols, in.rows);
			return;
		}
#endif 
		// step.p[0]中存储第一维空间槽间步长
		const int stride = in.step.p[0];
		uint8_t* top = (uint8_t*)in.data;
		uint8_t* bottom = top + stride;
		uint8_t* end = top + stride*in.rows;
		const int out_width = out.cols;
		uint8_t* p = (uint8_t*)out.data;
		while (bottom < end)
		{
			for (int j = 0; j < out_width; j++)
			{
				*p = static_cast<uint8_t>((uint16_t(top[0]) + top[1] + bottom[0] + bottom[1]) / 4);
				p++;
				top += 2;
				bottom += 2;
			}
			top += stride;
			bottom += stride;
		}
	}

	/**	通过双边差值返回像素值从0到255，注意这边没有检查x，y是否在border内
	 */
	inline float interpolateMat_8u(const cv::Mat& mat, float u, float v)
	{
		assert(mat.type() == CV_8U);
		int x = floor(u);
		int y = floor(v);
		float subpix_x = u - x;
		float subpix_y = v - y;

		float w00 = (1.0f - subpix_x)*(1.0f - subpix_y);
		float w01 = (1.0f - subpix_x)*subpix_y;
		float w10 = subpix_x*(1.0f - subpix_y);
		float w11 = 1.0f - w00 - w01 - w10;

		const int stride = mat.step.p[0];
		unsigned char* ptr = mat.data + y*stride + x;
		return w00*ptr[0] + w01*ptr[stride] + w10*ptr[1] + w11*ptr[stride + 1];
	}

}


#endif // OPENMVO_UTILS_IMAGE_UTILS_H_