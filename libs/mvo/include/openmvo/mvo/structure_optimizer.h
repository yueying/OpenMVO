/*************************************************************************
 * 文件名： structure_optimizer
 *
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 * 时间： 2015/8/24
 *
 * 说明： 
 *************************************************************************/
#ifndef OPENMVO_MVO_STRUCTURE_OPTIMIZER_H_
#define OPENMVO_MVO_STRUCTURE_OPTIMIZER_H_
#include <openmvo/mvo/frame.h>

namespace mvo
{
	/// 对一些观察到的3D点进行优化
	void structureOptimize(FramePtr frame, size_t max_n_pts, int max_iter);
}

#endif // OPENMVO_MVO_STRUCTURE_OPTIMIZER_H_