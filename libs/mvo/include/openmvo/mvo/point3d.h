/*************************************************************************
 * 文件名： point3d
 *
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 * 时间： 2015/9/7
 *
 * 说明： 特征对应的3D点
 *************************************************************************/
#ifndef OPENMVO_MVO_POINT3D_H_
#define OPENMVO_MVO_POINT3D_H_

#include <list>
#include <Eigen/Core>
#include "openmvo/utils/noncopyable.h"

namespace mvo
{
	class Feature;

	using namespace Eigen;
	/**	确保点对象唯一
	 */
	class Point3D : Noncopyable
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		Point3D(const Vector3d& pos);
		~Point3D();
		/// 添加特征到一个帧中
		void addFrameRef(Feature* ftr);

	public:
		static int                  point_counter_;           //!< 创建点的计数，用于设置唯一的id
		int                         id_;                      //!< 点唯一的id
		Vector3d                    pos_;                     //!< 点在世界坐标系中的位置
		std::list<Feature*>         obs_;                     //!< 对应这个点的特征
	};
}

#endif // OPENMVO_MVO_POINT3D_H_