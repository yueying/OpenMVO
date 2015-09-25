/*************************************************************************
 * 文件名： structure_optimizer
 *
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 * 时间： 2015/9/24
 *
 * 说明： 
 *************************************************************************/
#include <openmvo/mvo/structure_optimizer.h>
#include <deque>
#include <openmvo/mvo/point3d.h>
#include <openmvo/mvo/feature.h>

namespace mvo
{
	bool ptLastOptimComparator(Point3D* lhs, Point3D* rhs)
	{
		return (lhs->last_structure_optim_ < rhs->last_structure_optim_);
	}

	void structureOptimize(
		FramePtr frame,
		size_t max_n_pts,
		int max_iter)
	{
		std::deque<Point3D*> pts;
		for (Features::iterator it = frame->fts_.begin(); it != frame->fts_.end(); ++it)
		{
			if ((*it)->point != NULL)
				pts.push_back((*it)->point);
		}
		max_n_pts = std::min(max_n_pts, pts.size());
		std::nth_element(pts.begin(), pts.begin() + max_n_pts, pts.end(), ptLastOptimComparator);
		for (std::deque<Point3D*>::iterator it = pts.begin(); it != pts.begin() + max_n_pts; ++it)
		{
			(*it)->optimize(max_iter);
			(*it)->last_structure_optim_ = frame->id_;
		}
	}
}