/*************************************************************************
 * 文件名： point3d
 *
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 * 时间： 2015/8/7
 *
 * 说明： 特征对应的3D点
 *************************************************************************/
#include "openmvo/mvo/point3d.h"
#include "openmvo/mvo/feature.h"
#include <openmvo/utils/math_utils.h>

namespace mvo
{
	const double EPS = 0.0000000001;
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

	void Point3D::optimize(const size_t n_iter)
	{
		Vector3d old_point = pos_;
		double chi2 = 0.0;
		Matrix3d A;
		Vector3d b;

		for (size_t i = 0; i < n_iter; i++)
		{
			A.setZero();
			b.setZero();
			double new_chi2 = 0.0;

			// 计算残差
			for (auto it = obs_.begin(); it != obs_.end(); ++it)
			{
				Matrix23d J;
				const Vector3d p_in_f((*it)->frame->T_f_w_ * pos_);
				Point3D::jacobian_xyz2uv(p_in_f, (*it)->frame->T_f_w_.rotation_matrix(), J);
				const Vector2d e(project2d((*it)->f) - project2d(p_in_f));
				new_chi2 += e.squaredNorm();
				A.noalias() += J.transpose() * J;
				b.noalias() -= J.transpose() * e;
			}

			// 求解线性系统
			const Vector3d dp(A.ldlt().solve(b));

			// 检测误差有没有增长
			if ((i > 0 && new_chi2 > chi2) || (bool)std::isnan((double)dp[0]))
			{
				pos_ = old_point; // 回滚
				break;
			}

			// 更新模型
			Vector3d new_point = pos_ + dp;
			old_point = pos_;
			pos_ = new_point;
			chi2 = new_chi2;

			// 收敛则停止
			if (norm_max(dp) <= EPS)
				break;
		}

	}
}