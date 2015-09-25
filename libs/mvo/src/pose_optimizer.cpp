/*************************************************************************
 * 文件名： pose_optimizer
 *
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 * 时间： 2015/9/23
 *
 * 说明： 
 *************************************************************************/
#include <openmvo/mvo/pose_optimizer.h>
#include <iostream>
#include <Eigen/Core>
#include <sophus/se3.h>
#include <openmvo/math/robust_cost.h>
#include <openmvo/utils/math_utils.h>
#include <openmvo/mvo/feature.h>

namespace mvo
{
	using namespace Eigen;
	using namespace Sophus;

	typedef Matrix<double, 6, 6> Matrix6d;
	typedef Matrix<double, 2, 6> Matrix26d;
	typedef Matrix<double, 6, 1> Vector6d;
	const double EPS = 0.0000000001;

	void poseOptimize(
		const double reproj_thresh,
		const size_t n_iter,
		const bool verbose,
		FramePtr& frame,
		double& estimated_scale,
		double& error_init,
		double& error_final,
		size_t& num_obs)
	{
		// 初始化
		double chi2(0.0);
		std::vector<double> chi2_vec_init, chi2_vec_final;
		TukeyWeightFunction weight_function;
		SE3 T_old(frame->T_f_w_);
		Matrix6d A;
		Vector6d b;

		// 计算投影误差
		std::vector<float> errors; errors.reserve(frame->fts_.size());
		for (auto it = frame->fts_.begin(); it != frame->fts_.end(); ++it)
		{
			if ((*it)->point == NULL)
				continue;
			//将投影与feature align计算的特征计算差值
			Vector2d e = project2d((*it)->f)
				- project2d(frame->T_f_w_ * (*it)->point->pos_);
			e *= 1.0 / (1 << (*it)->level);
			errors.push_back(e.norm());
		}
		if (errors.empty())
			return;
		MADScaleEstimator scale_estimator;
		estimated_scale = scale_estimator.compute(errors);//计算绝对中值偏差

		num_obs = errors.size();
		chi2_vec_init.reserve(num_obs);
		chi2_vec_final.reserve(num_obs);
		double scale = estimated_scale;
		for (size_t iter = 0; iter < n_iter; iter++)
		{
			// ？？？为什么在迭代次数为5的时候重写scale
			if (iter == 5)
				scale = 0.85 / frame->cam_->getFocalLength();

			b.setZero();
			A.setZero();
			double new_chi2(0.0);

			// 计算残差
			for (auto it = frame->fts_.begin(); it != frame->fts_.end(); ++it)
			{
				if ((*it)->point == NULL)
					continue;
				Matrix26d J;
				Vector3d xyz_f(frame->T_f_w_ * (*it)->point->pos_);
				Frame::jacobian_xyz2uv(xyz_f, J);
				Vector2d e = project2d((*it)->f) - project2d(xyz_f);
				double sqrt_inv_cov = 1.0 / (1 << (*it)->level);
				e *= sqrt_inv_cov;
				if (iter == 0)
					chi2_vec_init.push_back(e.squaredNorm()); // 主要用于调试，看结果
				J *= sqrt_inv_cov;
				double weight = weight_function.value(e.norm() / scale);
				A.noalias() += J.transpose()*J*weight;
				b.noalias() -= J.transpose()*e*weight;//这边为负号，是因为雅克比矩阵已经添加负号了
				new_chi2 += e.squaredNorm()*weight;
			}

			//求解线性方程
			const Vector6d dT(A.ldlt().solve(b));

			// 检测误差是否增加
			if ((iter > 0 && new_chi2 > chi2) || (bool)std::isnan((double)dT[0]))
			{
				if (verbose)
					std::cout << "it " << iter
					<< "\t FAILURE \t new_chi2 = " << new_chi2 << std::endl;
				frame->T_f_w_ = T_old; // 如果误差增加则回滚
				break;
			}

			// 更新模型
			SE3 T_new = SE3::exp(dT)*frame->T_f_w_;
			T_old = frame->T_f_w_;
			frame->T_f_w_ = T_new;
			chi2 = new_chi2;
			if (verbose)
				std::cout << "it " << iter
				<< "\t Success \t new_chi2 = " << new_chi2
				<< "\t norm(dT) = " << norm_max(dT) << std::endl;

			// 如果收敛则结束
			if (norm_max(dT) <= EPS)
				break;
		}

		// 移除残差较大的投影的测量
		double reproj_thresh_scaled = reproj_thresh / frame->cam_->getFocalLength();
		size_t n_deleted_refs = 0;
		for (Features::iterator it = frame->fts_.begin(); it != frame->fts_.end(); ++it)
		{
			if ((*it)->point == NULL)
				continue;
			Vector2d e = project2d((*it)->f) - project2d(frame->T_f_w_ * (*it)->point->pos_);
			double sqrt_inv_cov = 1.0 / (1 << (*it)->level);
			e *= sqrt_inv_cov;
			chi2_vec_final.push_back(e.squaredNorm());
			if (e.norm() > reproj_thresh_scaled)
			{
				// 我们不需要删除这个指针，因为还没有创建
				(*it)->point = NULL;
				++n_deleted_refs;
			}
		}

		error_init = 0.0;
		error_final = 0.0;
		if (!chi2_vec_init.empty())
			error_init = sqrt(getMedian(chi2_vec_init))*frame->cam_->getFocalLength();
		if (!chi2_vec_final.empty())
			error_final = sqrt(getMedian(chi2_vec_final))*frame->cam_->getFocalLength();

		estimated_scale *= frame->cam_->getFocalLength();
		if (verbose)
			std::cout << "n deleted obs = " << n_deleted_refs
			<< "\t scale = " << estimated_scale
			<< "\t error init = " << error_init
			<< "\t error end = " << error_final << std::endl;
		num_obs -= n_deleted_refs;
	}
}