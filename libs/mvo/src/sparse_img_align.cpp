/*************************************************************************
 * 文件名： sparse_img_align
 *
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 * 时间： 2015/8/15
 *
 * 说明： 通过图像中稀疏特征的对齐进行运动估计
 *************************************************************************/
#include <algorithm>
#include <openmvo/mvo/sparse_img_align.h>
#include <openmvo/mvo/frame.h>
#include <openmvo/mvo/feature.h>
#include <openmvo/mvo/point3d.h>
#include <openmvo/mvo/abstract_camera.h>

namespace mvo {

	SparseImgAlign::SparseImgAlign(
		int max_level, int min_level, int n_iter,
		Method method, bool display, bool verbose) :
		display_(display),
		max_level_(max_level),
		min_level_(min_level)
	{
		n_iter_ = n_iter;
		n_iter_init_ = n_iter_;
		method_ = method;
		verbose_ = verbose;
		eps_ = 0.000001;
	}

	size_t SparseImgAlign::run(FramePtr ref_frame, FramePtr cur_frame)
	{
		reset();
		// 首先确保参考帧中特征不为空
		if (ref_frame->fts_.empty())
		{
			std::cerr<<"SparseImgAlign: no features to track!";
			return 0;
		}

		ref_frame_ = ref_frame;
		cur_frame_ = cur_frame;
		ref_patch_cache_ = cv::Mat(ref_frame_->fts_.size(), patch_area_, CV_32F);// 对所有的特征构建4*4的矩阵
		jacobian_cache_.resize(Eigen::NoChange, ref_patch_cache_.rows*patch_area_);
		visible_fts_.resize(ref_patch_cache_.rows, false); // TODO: 这边特征是否可见应设置在不同的尺度下

		SE3 T_cur_from_ref(cur_frame_->T_f_w_ * ref_frame_->T_f_w_.inverse());// 初始实际就是单位阵

		for (level_ = max_level_; level_ >= min_level_; --level_)
		{
			mu_ = 0.1;
			jacobian_cache_.setZero();
			have_ref_patch_cache_ = false;
			if (verbose_)
				printf("\nPYRAMID LEVEL %i\n---------------\n", level_);
			optimize(T_cur_from_ref);
		}
		cur_frame_->T_f_w_ = T_cur_from_ref * ref_frame_->T_f_w_;
		return n_meas_ / patch_area_;
	}


	void SparseImgAlign::precomputeReferencePatches()
	{
		const int border = patch_halfsize_ + 1;
		const cv::Mat& ref_img = ref_frame_->img_pyr_.at(level_);//得到当前金字塔等级参考图像
		const int stride = ref_img.cols;
		const float scale = 1.0f / (1 << level_);
		const Vector3d ref_pos = ref_frame_->pos();//当前帧世界坐标系中的坐标
		const double focal_length = ref_frame_->cam_->getFocalLength();
		size_t feature_counter = 0;
		std::vector<bool>::iterator visiblity_it = visible_fts_.begin();
		for (auto it = ref_frame_->fts_.begin(), ite = ref_frame_->fts_.end();
			it != ite; ++it, ++feature_counter, ++visiblity_it)
		{
			// 确保面片在图像内
			const float u_ref = (*it)->px[0] * scale;
			const float v_ref = (*it)->px[1] * scale;
			const int u_ref_i = floorf(u_ref);
			const int v_ref_i = floorf(v_ref);
			if ((*it)->point == NULL || u_ref_i - border < 0 || v_ref_i - border < 0 || u_ref_i + border >= ref_img.cols || v_ref_i + border >= ref_img.rows)
				continue;
			*visiblity_it = true;

			// 这边不能直接使用3d点的坐标，会存在重投影的误差，而是通过单位点乘以深度的方式进行计算
			const double depth(((*it)->point->pos_ - ref_pos).norm());
			const Vector3d xyz_ref((*it)->f*depth);

			// 估计投影的雅克比矩阵
			Matrix<double, 2, 6> frame_jac;
			Frame::jacobian_xyz2uv(xyz_ref, frame_jac);

			// 对参考图像进行双边差值操作
			const float subpix_u_ref = u_ref - u_ref_i;
			const float subpix_v_ref = v_ref - v_ref_i;
			const float w_ref_tl = (1.0 - subpix_u_ref) * (1.0 - subpix_v_ref);
			const float w_ref_tr = subpix_u_ref * (1.0 - subpix_v_ref);
			const float w_ref_bl = (1.0 - subpix_u_ref) * subpix_v_ref;
			const float w_ref_br = subpix_u_ref * subpix_v_ref;
			size_t pixel_counter = 0;
			float* cache_ptr = reinterpret_cast<float*>(ref_patch_cache_.data) + patch_area_*feature_counter;
			// 对一个面片进行计算
			for (int y = 0; y < patch_size_; ++y)
			{
				uint8_t* ref_img_ptr = (uint8_t*)ref_img.data + (v_ref_i + y - patch_halfsize_)*stride + (u_ref_i - patch_halfsize_);
				for (int x = 0; x < patch_size_; ++x, ++ref_img_ptr, ++cache_ptr, ++pixel_counter)
				{
					// 通过插值计算每个特征面片灰度值，主要进行测试查看
					*cache_ptr = w_ref_tl*ref_img_ptr[0] + w_ref_tr*ref_img_ptr[1] + w_ref_bl*ref_img_ptr[stride] + w_ref_br*ref_img_ptr[stride + 1];

					// 采用逆向组合算法(inverse compositional): 通过采取梯度总是在相同位置这一性质
					// 得到warped的图像 
					float dx = 0.5f * ((w_ref_tl*ref_img_ptr[1] + w_ref_tr*ref_img_ptr[2] + w_ref_bl*ref_img_ptr[stride + 1] + w_ref_br*ref_img_ptr[stride + 2])
						- (w_ref_tl*ref_img_ptr[-1] + w_ref_tr*ref_img_ptr[0] + w_ref_bl*ref_img_ptr[stride - 1] + w_ref_br*ref_img_ptr[stride]));
					float dy = 0.5f * ((w_ref_tl*ref_img_ptr[stride] + w_ref_tr*ref_img_ptr[1 + stride] + w_ref_bl*ref_img_ptr[stride * 2] + w_ref_br*ref_img_ptr[stride * 2 + 1])
						- (w_ref_tl*ref_img_ptr[-stride] + w_ref_tr*ref_img_ptr[1 - stride] + w_ref_bl*ref_img_ptr[0] + w_ref_br*ref_img_ptr[1]));

					// cache the jacobian
					jacobian_cache_.col(feature_counter*patch_area_ + pixel_counter) =
						(dx*frame_jac.row(0) + dy*frame_jac.row(1))*(focal_length / (1 << level_));
				}
			}
		}
		have_ref_patch_cache_ = true;
	}

	double SparseImgAlign::computeResiduals(
		const SE3& T_cur_from_ref,
		bool linearize_system,
		bool compute_weight_scale)
	{
		// 对当前图像进行Warp处理以对应参考图像
		const cv::Mat& cur_img = cur_frame_->img_pyr_.at(level_);

		if (linearize_system && display_)
			resimg_ = cv::Mat(cur_img.size(), CV_32F, cv::Scalar(0));

		if (have_ref_patch_cache_ == false)
			precomputeReferencePatches();

		std::vector<float> errors;
		if (compute_weight_scale)
			errors.reserve(visible_fts_.size());
		const int stride = cur_img.cols;
		const int border = patch_halfsize_ + 1;
		const float scale = 1.0f / (1 << level_);
		const Vector3d ref_pos(ref_frame_->pos());
		float chi2 = 0.0;
		size_t feature_counter = 0; // 计算cached jacobian的索引，对应每个特征
		std::vector<bool>::iterator visiblity_it = visible_fts_.begin();
		for (auto it = ref_frame_->fts_.begin(); it != ref_frame_->fts_.end();
			++it, ++feature_counter, ++visiblity_it)
		{
			// 检测特征在图像中是否可见
			if (!*visiblity_it)
				continue;

			// 计算在当前图像中投影的像素位置
			const double depth = ((*it)->point->pos_ - ref_pos).norm();
			const Vector3d xyz_ref((*it)->f*depth);//避免了重投影的误差
			const Vector3d xyz_cur(T_cur_from_ref * xyz_ref);
			const Vector2f uv_cur_pyr(cur_frame_->cam_->world2cam(xyz_cur).cast<float>() * scale);// 计算投影到当前帧的像素
			const float u_cur = uv_cur_pyr[0];
			const float v_cur = uv_cur_pyr[1];
			const int u_cur_i = floorf(u_cur);
			const int v_cur_i = floorf(v_cur);

			// 检测投影值是否在图像中
			if (u_cur_i < 0 || v_cur_i < 0 || u_cur_i - border < 0 || v_cur_i - border < 0 || u_cur_i + border >= cur_img.cols || v_cur_i + border >= cur_img.rows)
				continue;

			// 对当前图像进行双边插值加权
			const float subpix_u_cur = u_cur - u_cur_i;
			const float subpix_v_cur = v_cur - v_cur_i;
			const float w_cur_tl = (1.0 - subpix_u_cur) * (1.0 - subpix_v_cur);
			const float w_cur_tr = subpix_u_cur * (1.0 - subpix_v_cur);
			const float w_cur_bl = (1.0 - subpix_u_cur) * subpix_v_cur;
			const float w_cur_br = subpix_u_cur * subpix_v_cur;
			float* ref_patch_cache_ptr = reinterpret_cast<float*>(ref_patch_cache_.data) + patch_area_*feature_counter;
			size_t pixel_counter = 0; // 用于计算cached jacobian的索引，每个特征对应的像素索引
			for (int y = 0; y < patch_size_; ++y)
			{
				uint8_t* cur_img_ptr = (uint8_t*)cur_img.data + (v_cur_i + y - patch_halfsize_)*stride + (u_cur_i - patch_halfsize_);

				for (int x = 0; x < patch_size_; ++x, ++pixel_counter, ++cur_img_ptr, ++ref_patch_cache_ptr)
				{
					// 计算残差
					const float intensity_cur = w_cur_tl*cur_img_ptr[0] + w_cur_tr*cur_img_ptr[1] + w_cur_bl*cur_img_ptr[stride] + w_cur_br*cur_img_ptr[stride + 1];
					const float res = intensity_cur - (*ref_patch_cache_ptr);

					// 用于计算scale用于robust cost
					if (compute_weight_scale)
						errors.push_back(fabsf(res));

					// 给出权重
					float weight = 1.0;
					if (use_weights_) {
						weight = weight_function_->value(res / scale_);
					}

					chi2 += res*res*weight;
					n_meas_++;

					if (linearize_system)
					{
						// 计算Jacobian, 带权重的Hessian 和残差图像
						const Vector6d J(jacobian_cache_.col(feature_counter*patch_area_ + pixel_counter));
						H_.noalias() += J*J.transpose()*weight;
						Jres_.noalias() -= J*res*weight;
						if (display_)
							resimg_.at<float>((int)v_cur + y - patch_halfsize_, (int)u_cur + x - patch_halfsize_) = res / 255.0;
					}
				}
			}
		}

		// 在第一次迭代时计算权重
		if (compute_weight_scale && iter_ == 0)
			scale_ = scale_estimator_->compute(errors);

		return chi2 / n_meas_;
	}

	int SparseImgAlign::solve()
	{
		x_ = H_.ldlt().solve(Jres_);
		if ((bool)std::isnan((double)x_[0]))
			return 0;
		return 1;
	}

	void SparseImgAlign::update(
		const ModelType& T_curold_from_ref,
		ModelType& T_curnew_from_ref)
	{
		T_curnew_from_ref = T_curold_from_ref * SE3::exp(-x_);
	}

	void SparseImgAlign::startIteration()
	{}

	void SparseImgAlign::finishIteration()
	{
		if (display_)
		{
			cv::namedWindow("residuals", CV_WINDOW_AUTOSIZE);
			cv::imshow("residuals", resimg_ * 10);
			cv::waitKey(0);
		}
	}

} // namespace svo

