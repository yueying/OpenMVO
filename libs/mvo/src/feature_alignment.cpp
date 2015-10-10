/*************************************************************************
 * 文件名： feature_alignment
 *
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 * 时间： 2015/8/18
 *
 * 说明：
 *************************************************************************/
#include <openmvo/mvo/feature_alignment.h>
#include <Eigen/Core>
#include <Eigen/LU>

namespace mvo
{
	namespace feature_alignment
	{

		bool align2D(
			const cv::Mat& cur_img,
			uint8_t* ref_patch_with_border,
			uint8_t* ref_patch,
			const int n_iter,
			Vector2d& cur_px_estimate,
			bool no_simd)
		{
			const int halfpatch_size_ = 4;
			const int patch_size_ = 8;
			const int patch_area_ = 64;
			bool converged = false;

#ifdef _MSC_VER
			__declspec(align(16)) float ref_patch_dx[patch_area_];
			__declspec(align(16)) float ref_patch_dy[patch_area_];
#else
			float __attribute__((__aligned__(16))) ref_patch_dx[patch_area_];
			float __attribute__((__aligned__(16))) ref_patch_dy[patch_area_];
#endif
			Matrix3f H; H.setZero();

			// 计算梯度和hessian
			const int ref_step = patch_size_ + 2;
			float *it_dx = ref_patch_dx;
			float *it_dy = ref_patch_dy;
			for (int y = 0; y < patch_size_; ++y)
			{
				uint8_t* it = ref_patch_with_border + (y + 1)*ref_step + 1;
				for (int x = 0; x < patch_size_; ++x, ++it, ++it_dx, ++it_dy)
				{
					Vector3f J;
					J[0] = 0.5 * (it[1] - it[-1]);//x方向梯度值
					J[1] = 0.5 * (it[ref_step] - it[-ref_step]);//y方向梯度值
					J[2] = 1;
					*it_dx = J[0];
					*it_dy = J[1];
					H += J*J.transpose();
				}
			}
			Matrix3f Hinv = H.inverse();
			float mean_diff = 0;

			// 计算在新图像中的像素位置
			float u = cur_px_estimate.x();
			float v = cur_px_estimate.y();

			// 终止条件
			const float min_update_squared = 0.03*0.03;
			const int cur_step = cur_img.step.p[0];
			Vector3f update; 
			update.setZero();
			for (int iter = 0; iter < n_iter; ++iter)
			{
				int u_r = floor(u);
				int v_r = floor(v);
				if (u_r < halfpatch_size_ || v_r < halfpatch_size_ || u_r >= cur_img.cols - halfpatch_size_ || v_r >= cur_img.rows - halfpatch_size_)
					break;
				// TODO very rarely this can happen, maybe H is singular? should not be at corner.. check
				if (isnan(u) || isnan(v)) 
					return false;

				// 计算插值权重
				float subpix_x = u - u_r;
				float subpix_y = v - v_r;
				float wTL = (1.0 - subpix_x)*(1.0 - subpix_y);
				float wTR = subpix_x * (1.0 - subpix_y);
				float wBL = (1.0 - subpix_x)*subpix_y;
				float wBR = subpix_x * subpix_y;

				// 循环遍历 插值
				uint8_t* it_ref = ref_patch;
				float* it_ref_dx = ref_patch_dx;
				float* it_ref_dy = ref_patch_dy;
				
				Vector3f Jres; Jres.setZero();
				for (int y = 0; y < patch_size_; ++y)
				{
					uint8_t* it = (uint8_t*)cur_img.data + (v_r + y - halfpatch_size_)*cur_step + u_r - halfpatch_size_;
					for (int x = 0; x < patch_size_; ++x, ++it, ++it_ref, ++it_ref_dx, ++it_ref_dy)
					{
						float search_pixel = wTL*it[0] + wTR*it[1] + wBL*it[cur_step] + wBR*it[cur_step + 1];
						float res = search_pixel - *it_ref + mean_diff;
						Jres[0] -= res*(*it_ref_dx);
						Jres[1] -= res*(*it_ref_dy);
						Jres[2] -= res;
					}
				}

				update = Hinv * Jres;//负号前面已经添加
				u += update[0];
				v += update[1];
				mean_diff += update[2];

				if (update[0] * update[0] + update[1] * update[1] < min_update_squared)
				{
					converged = true;
					break;
				}
			}

			cur_px_estimate << u, v;
			return converged;
		}
	}
}