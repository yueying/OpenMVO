/*************************************************************************
 * 文件名： pose_optimizer
 *
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 * 时间： 2015/8/23
 *
 * 说明： 
 *************************************************************************/
#ifndef OPENMVO_MVO_POSE_OPTIMIZER_H_
#define OPENMVO_MVO_POSE_OPTIMIZER_H_

#include <openmvo/mvo/frame.h>

namespace mvo
{
	/**	主要是针对仅有运动的BA，最小化单帧的投影误差
	 */
	void poseOptimize(
		const double reproj_thresh,
		const size_t n_iter,
		const bool verbose,
		FramePtr& frame,
		double& estimated_scale,
		double& error_init,
		double& error_final,
		size_t& num_obs);
}

#endif // OPENMVO_MVO_POSE_OPTIMIZER_H_