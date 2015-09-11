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
}