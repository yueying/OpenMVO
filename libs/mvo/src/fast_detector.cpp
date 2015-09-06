/*************************************************************************
 * 文件名： fast_detector
 *
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 * 时间： 2015/8/4
 *
 * 说明： 
 *************************************************************************/
#include <fast/fast.h>
#include "openmvo/mvo/fast_detector.h"

namespace mvo
{
	FastDetector::FastDetector(
		const int img_width,
		const int img_height,
		const int cell_size,
		const int n_pyr_levels) :
		AbstractDetector(img_width, img_height, cell_size, n_pyr_levels)
	{}

	void FastDetector::detect(
		Frame* frame,
		const ImgPyr& img_pyr,
		const double detection_threshold,
		Features& fts)
	{
		Corners corners(grid_n_cols_*grid_n_rows_, Corner(0, 0, detection_threshold, 0, 0.0f));
		// 对每层金字塔都进行fast特征检测
		for (int L = 0; L < n_pyr_levels_; ++L)
		{
			const int scale = (1 << L);
			std::vector<fast::fast_xy> fast_corners;
#if __SSE2__
			fast::fast_corner_detect_10_sse2(
				(fast::fast_byte*) img_pyr[L].data, img_pyr[L].cols,
				img_pyr[L].rows, img_pyr[L].cols, 20, fast_corners);
#else
			fast::fast_corner_detect_10(
				(fast::fast_byte*) img_pyr[L].data, img_pyr[L].cols,
				img_pyr[L].rows, img_pyr[L].cols, 20, fast_corners);
#endif
			std::vector<int> scores, nm_corners;
			fast::fast_corner_score_10((fast::fast_byte*) img_pyr[L].data, img_pyr[L].cols, fast_corners, 20, scores);
			fast::fast_nonmax_3x3(fast_corners, scores, nm_corners);

			for (auto it = nm_corners.begin(), ite = nm_corners.end(); it != ite; ++it)
			{
				fast::fast_xy& xy = fast_corners.at(*it);
				const int k = static_cast<int>((xy.y*scale) / cell_size_)*grid_n_cols_
					+ static_cast<int>((xy.x*scale) / cell_size_);// 获得一维列的索引
				if (grid_occupancy_[k])// 如果这个格子里面已经有特征，则该特征可以不必再进行计算了
					continue;
				const float score = shiTomasiScore(img_pyr[L], xy.x, xy.y);//计算shi-Tomasi角点检测，根据阈值选择更好的角点
				if (score > corners.at(k).score)
					corners.at(k) = Corner(xy.x*scale, xy.y*scale, score, L, 0.0f);
			}
		}

		// 返回所有的特征
		std::for_each(corners.begin(), corners.end(), [&](Corner& c) {
			if (c.score > detection_threshold)
				fts.push_back(new Feature(frame, Vector2d(c.x, c.y), c.level));
		});

		resetGrid();
	}

	float FastDetector::shiTomasiScore(const cv::Mat& img, int u, int v)
	{
		assert(img.type() == CV_8UC1);

		float dXX = 0.0;
		float dYY = 0.0;
		float dXY = 0.0;
		const int halfbox_size = 4;
		const int box_size = 2 * halfbox_size;
		const int box_area = box_size*box_size;
		const int x_min = u - halfbox_size;
		const int x_max = u + halfbox_size;
		const int y_min = v - halfbox_size;
		const int y_max = v + halfbox_size;

		if (x_min < 1 || x_max >= img.cols - 1 || y_min < 1 || y_max >= img.rows - 1)
			return 0.0; // 面片太靠近边界，返回0

		const int stride = img.step.p[0];//一行元素的个数
		for (int y = y_min; y < y_max; ++y)
		{
			const uint8_t* ptr_left = img.data + stride*y + x_min - 1;
			const uint8_t* ptr_right = img.data + stride*y + x_min + 1;
			const uint8_t* ptr_top = img.data + stride*(y - 1) + x_min;
			const uint8_t* ptr_bottom = img.data + stride*(y + 1) + x_min;
			for (int x = 0; x < box_size; ++x, ++ptr_left, ++ptr_right, ++ptr_top, ++ptr_bottom)
			{
				float dx = *ptr_right - *ptr_left;
				float dy = *ptr_bottom - *ptr_top;
				dXX += dx*dx;
				dYY += dy*dy;
				dXY += dx*dy;
			}
		}

		// 返回小的特征值
		dXX = dXX / (2.0 * box_area);
		dYY = dYY / (2.0 * box_area);
		dXY = dXY / (2.0 * box_area);
		return 0.5 * (dXX + dYY - sqrt((dXX + dYY) * (dXX + dYY) - 4 * (dXX * dYY - dXY * dXY)));
	}
}