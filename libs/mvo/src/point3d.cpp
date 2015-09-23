/*************************************************************************
 * 文件名： point3d
 *
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 * 时间： 2015/9/7
 *
 * 说明： 特征对应的3D点
 *************************************************************************/
#include "openmvo/mvo/point3d.h"
#include "openmvo/mvo/feature.h"

namespace mvo
{
	int Point3D::point_counter_ = 0;

	/// 构造函数，给出点的世界坐标
	Point3D::Point3D(const Vector3d& pos) :
		id_(point_counter_++),
		pos_(pos)
	{}

	Point3D::~Point3D()
	{}

	void Point3D::addFrameRef(Feature* ftr)
	{
		obs_.push_front(ftr);
	}

	bool Point3D::getCloseViewObs(const Vector3d& framepos, Feature*& ftr) const
	{
		// TODO: 后期要确保点是相同的视图和相同的金字塔层
		// 得到观察的方向向量
		Vector3d obs_dir(framepos - pos_); 
		obs_dir.normalize();
		auto min_it = obs_.begin();
		double min_cos_angle = 0;
		for (auto it = obs_.begin(), ite = obs_.end(); it != ite; ++it)
		{
			Vector3d dir((*it)->frame->pos() - pos_); 
			dir.normalize();
			double cos_angle = obs_dir.dot(dir);// 单位向量点乘得到cos角度
			if (cos_angle > min_cos_angle)//保证特征是距离较近的两个帧
			{
				min_cos_angle = cos_angle;
				min_it = it;
			}
		}
		ftr = *min_it;
		if (min_cos_angle < 0.5) // 假设观察夹角大于60度没有用
			return false;
		return true;
	}
}