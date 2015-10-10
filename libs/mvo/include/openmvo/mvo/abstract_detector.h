/*************************************************************************
 * 文件名： abstract_detector
 *
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 * 时间： 2015/8/2
 *
 * 说明： 参考rpg_svo(https://github.com/uzh-rpg/rpg_svo)
 *************************************************************************/
#ifndef OPENMVO_MVO_ABSTRACT_DETECTOR_H_
#define OPENMVO_MVO_ABSTRACT_DETECTOR_H_

#include <memory>
#include <vector>
#include "openmvo/mvo/frame.h"
#include "openmvo/mvo/feature.h"

namespace mvo
{
	/// 特征检测的抽象类
	class AbstractDetector
	{
	public:
		AbstractDetector(
			const int img_width,
			const int img_height,
			const int cell_size,
			const int n_pyr_levels);

		virtual ~AbstractDetector() {};

		virtual void detect(
			Frame* frame,
			const ImgPyr& img_pyr,
			const double detection_threshold,
			Features& fts) = 0;

		/// 将已经存在特征的单元格设置为被占用
		void setExistingFeatures(const Features& fts);

		/// 标识设置单元格已经被占用
		void setGridOccupancy(const Vector2d& px);
	protected:
		/// 将所有格子重新设置，设置为没有占用
		void resetGrid();

	protected:

		const int cell_size_;             //!< 设置寻找角点单元格的大小
		const int n_pyr_levels_;          //!< 图像金字塔的等级
		const int grid_n_cols_;           //!< 将图像划分为格子后的列数
		const int grid_n_rows_;           //!< 将图像划分为格子后的行数
		std::vector<bool> grid_occupancy_;//!< 设定划分的所有格子数是否被占用
	};
	typedef std::shared_ptr<AbstractDetector> DetectorPtr;
}

#endif // OPENMVO_MVO_ABSTRACT_DETECTOR_H_