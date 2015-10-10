/*************************************************************************
 * 文件名： sparse_img_align
 *
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 * 时间： 2015/8/25
 *
 * 说明： 参考rpg_svo(https://github.com/uzh-rpg/rpg_svo)
 *************************************************************************/
#ifndef OPENMVO_MVO_SPARSE_IMG_ALIGN_H_
#define OPENMVO_MVO_SPARSE_IMG_ALIGN_H_

#include <opencv2/opencv.hpp>
#include <sophus/se3.h>
#include "openmvo/math/nlls_solver.h"
#include "openmvo/mvo/frame.h"

namespace mvo {

	using namespace Sophus;
	using namespace Eigen;

	class AbstractCamera;
	class Feature;

	/// 通过对帧之间的特征片进行光度误差最小化，优化帧位置
	class SparseImgAlign : public NLLSSolver < 6, SE3 >
	{
		static const int patch_halfsize_ = 2;//!< 面片的半尺寸为2
		static const int patch_size_ = 2 * patch_halfsize_;// 论文中将patch大小设置为4*4
		static const int patch_area_ = patch_size_*patch_size_;// 每个特征周围采用4*4的patch
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		cv::Mat resimg_;

		SparseImgAlign(
			int n_levels,
			int min_level,
			int n_iter,
			Method method,
			bool display,
			bool verbose);

		size_t run(FramePtr ref_frame,FramePtr cur_frame);

	protected:
		FramePtr ref_frame_;            //!< 参考帧，已经获得深度信息
		FramePtr cur_frame_;            //!< 当前帧，目前只知道图像信息
		int level_;                     //!< 当前金字塔等级
		bool display_;                  //!< 是否显示残差图像
		int max_level_;                 //!< 用于对齐的最粗糙即最大的金字塔等级
		int min_level_;                 //!< 用于对齐的最好的即最小的金字塔等级

		// cache:
		Matrix<double, 6, Dynamic, ColMajor> jacobian_cache_;
		bool have_ref_patch_cache_;
		cv::Mat ref_patch_cache_;
		std::vector<bool> visible_fts_;

		void precomputeReferencePatches();

		virtual double computeResiduals(const SE3& model, bool linearize_system, bool compute_weight_scale = false);
		virtual int solve();
		virtual void update(const ModelType& old_model, ModelType& new_model);

		virtual void startIteration();
		virtual void finishIteration();
	};

} // namespace mvo


#endif // OPENMVO_MVO_SPARSE_IMG_ALIGN_H_
