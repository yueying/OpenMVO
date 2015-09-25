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
	typedef Matrix<double, 2, 3> Matrix23d;
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
		/// 得到具有相近视角的观察特征
		bool getCloseViewObs(const Vector3d& pos, Feature*& obs) const;
		/// 优化点的位置通过最小重投影误差
		void optimize(const size_t n_iter);

		/// 3D点投影到单位平面(focal length = 1)对点求的雅克比矩阵
		inline static void jacobian_xyz2uv(
			const Vector3d& p_in_f,
			const Matrix3d& R_f_w,
			Matrix23d& point_jac)
		{
			const double z_inv = 1.0 / p_in_f[2];
			const double z_inv_sq = z_inv*z_inv;
			point_jac(0, 0) = z_inv;
			point_jac(0, 1) = 0.0;
			point_jac(0, 2) = -p_in_f[0] * z_inv_sq;
			point_jac(1, 0) = 0.0;
			point_jac(1, 1) = z_inv;
			point_jac(1, 2) = -p_in_f[1] * z_inv_sq;
			point_jac = -point_jac * R_f_w;
		}
	public:
		static int                  point_counter_;           //!< 创建点的计数，用于设置唯一的id
		int                         id_;                      //!< 点唯一的id
		Vector3d                    pos_;                     //!< 点在世界坐标系中的位置
		std::list<Feature*>         obs_;                     //!< 对应这个点的特征
		int                         last_projected_kf_id_;    //!< 重投影的标识，不对同一个点重投影两次
		int                         n_failed_reproj_;         //!< 重投影失败的数量，用于评价点的质量
		int                         n_succeeded_reproj_;      //!< 重投影成功的数量，用于评价点的质量
		int                         last_structure_optim_;    //!< 最近点优化的时间戳
	};

}

#endif // OPENMVO_MVO_POINT3D_H_