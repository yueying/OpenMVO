/*************************************************************************
 * 文件名： initialization
 *
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 * 时间： 2015/9/7
 *
 * 说明： 初始相关关系确定
 *************************************************************************/
#include "openmvo/mvo/initialization.h"
#include <opencv2/opencv.hpp>

#include "openmvo/utils/math_utils.h"
#include "openmvo/mvo/homography.h"
#include "openmvo/mvo/fast_detector.h"
#include "openmvo/mvo/point3d.h"

namespace mvo
{
	//  确保添加的第一帧检测到的特征点数大于100
	InitResult Initialization::addFirstFrame(FramePtr frame_ref)
	{
		reset();
		detectFeatures(frame_ref, px_ref_, f_ref_);
		if (px_ref_.size() < 100)
		{
			std::cerr << "First image has less than 100 features. Retry in more textured environment." << std::endl;
			return FAILURE;
		}
		int detect_features_num = px_ref_.size();
		// 将这一帧图像做为参考帧
		frame_ref_ = frame_ref;
		// 先设置当前帧的特征与参考帧的特征一致
		px_cur_.insert(px_cur_.begin(), px_ref_.begin(), px_ref_.end());
		return SUCCESS;
	}

	InitResult Initialization::addSecondFrame(FramePtr frame_cur)
	{
		trackKlt(frame_ref_, frame_cur, px_ref_, px_cur_, f_ref_, f_cur_, disparities_);
		std::cout << "Init: KLT tracked " << disparities_.size() << " features" << std::endl;

		// 符合光流跟踪的特征数
		if (disparities_.size() < 50)
			return FAILURE;

		// 对两帧光流跟踪之后像素差值的中值
		double disparity = getMedian(disparities_);
		std::cout << "Init: KLT " << disparity << "px average disparity." << std::endl;
		//  如果中值小于给定配置参数，则表明这一帧不是关键帧，也就是刚开始的时候两帧不能太近
		if (disparity < 50.0)
			return NO_KEYFRAME;
		//  计算单应矩阵
		computeHomography(
			f_ref_, f_cur_,
			frame_ref_->cam_->getFocalLength(), 2.0,
			inliers_, xyz_in_cur_, T_cur_from_ref_);
		std::cout << "Init: Homography RANSAC " << inliers_.size() << " inliers." << std::endl;
		// 根据计算单应矩阵之后，内点个数判断是否跟踪
		if (inliers_.size() < 40)
		{
			std::cerr << "Init WARNING: 40 inliers minimum required." << std::endl;
			return FAILURE;
		}

		// 通过单应矩阵，对两帧之间的特征形成的3d点进行计算，计算这些3d的深度中值，转换到指定的scale
		std::vector<double> depth_vec;
		for (size_t i = 0; i < xyz_in_cur_.size(); ++i)
			depth_vec.push_back((xyz_in_cur_[i]).z());
		double scene_depth_median = getMedian(depth_vec);
		double scale = 1.0 / scene_depth_median;
		// 计算相对变换SE3
		frame_cur->T_f_w_ = T_cur_from_ref_ * frame_ref_->T_f_w_;

		// 对位移变换添加尺度
		frame_cur->T_f_w_.translation() =
			-frame_cur->T_f_w_.rotation_matrix()*(frame_ref_->pos() + scale*(frame_cur->pos() - frame_ref_->pos()));
		
		// 对每个内点创建3D点，设置特征，添加到这两帧中
		SE3 T_world_cur = frame_cur->T_f_w_.inverse();
		for (std::vector<int>::iterator it = inliers_.begin(); it != inliers_.end(); ++it)
		{
			Vector2d px_cur(px_cur_[*it].x, px_cur_[*it].y);
			Vector2d px_ref(px_ref_[*it].x, px_ref_[*it].y);
			if (frame_ref_->cam_->isInFrame(px_cur.cast<int>(), 10) && frame_ref_->cam_->isInFrame(px_ref.cast<int>(), 10) && xyz_in_cur_[*it].z() > 0)
			{
				Vector3d pos = T_world_cur * (xyz_in_cur_[*it] * scale);// 将相机下的点坐标转世界坐标
				Point3D *new_point = new Point3D(pos);

				Feature* ftr_cur = new Feature(frame_cur.get(), new_point, px_cur, f_cur_[*it], 0);
				frame_cur->addFeature(ftr_cur);
				// 将同一个点对应的特征保存起来，这样点删除了，对应的特征都可以删除
				new_point->addFrameRef(ftr_cur);

				Feature* ftr_ref = new Feature(frame_ref_.get(), new_point, px_ref, f_ref_[*it], 0);
				frame_ref_->addFeature(ftr_ref);
				new_point->addFrameRef(ftr_ref);
			}
		}
		return SUCCESS;
	}

	// 清楚当前帧中的特征点，清楚参考帧
	void Initialization::reset()
	{
		px_cur_.clear();
		frame_ref_.reset();
	}

	/// 检测fast角度，输出的是对应的点和点的方向向量（可以考虑为点的反投影坐标）
	void Initialization::detectFeatures(
		FramePtr frame,
		std::vector<cv::Point2f>& px_vec,
		std::vector<Vector3d>& f_vec)
	{
		Features new_features;
		FastDetector detector(
			frame->img().cols, frame->img().rows, 25, 3);
		detector.detect(frame.get(), frame->img_pyr_, 20.0, new_features);

		// 返回特征位置和特征的单位向量
		px_vec.clear(); px_vec.reserve(new_features.size());
		f_vec.clear(); f_vec.reserve(new_features.size());
		std::for_each(new_features.begin(), new_features.end(), [&](Feature* ftr){
			px_vec.push_back(cv::Point2f(ftr->px[0], ftr->px[1]));
			f_vec.push_back(ftr->f);
			delete ftr;
		});
	}

	void Initialization::trackKlt(
		FramePtr frame_ref,
		FramePtr frame_cur,
		std::vector<cv::Point2f>& px_ref,
		std::vector<cv::Point2f>& px_cur,
		std::vector<Vector3d>& f_ref,
		std::vector<Vector3d>& f_cur,
		std::vector<double>& disparities)
	{
		const double klt_win_size = 30.0;
		const int klt_max_iter = 30;
		const double klt_eps = 0.001;
		std::vector<uchar> status;
		std::vector<float> error;
		std::vector<float> min_eig_vec;
		cv::TermCriteria termcrit(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, klt_max_iter, klt_eps);
		cv::calcOpticalFlowPyrLK(frame_ref->img_pyr_[0], frame_cur->img_pyr_[0],
			px_ref, px_cur,
			status, error,
			cv::Size2i(klt_win_size, klt_win_size),
			4, termcrit, 0);//cv::OPTFLOW_USE_INITIAL_FLOW

		std::vector<cv::Point2f>::iterator px_ref_it = px_ref.begin();
		std::vector<cv::Point2f>::iterator px_cur_it = px_cur.begin();
		std::vector<Vector3d>::iterator f_ref_it = f_ref.begin();
		f_cur.clear(); f_cur.reserve(px_cur.size());
		disparities.clear(); disparities.reserve(px_cur.size());
		for (size_t i = 0; px_ref_it != px_ref.end(); ++i)
		{
			if (!status[i])//如果光流没有发现，则删除
			{
				px_ref_it = px_ref.erase(px_ref_it);
				px_cur_it = px_cur.erase(px_cur_it);
				f_ref_it = f_ref.erase(f_ref_it);
				continue;
			}
			f_cur.push_back(frame_cur->c2f(px_cur_it->x, px_cur_it->y));//添加当前特征对应的单位向量
			disparities.push_back(Vector2d(px_ref_it->x - px_cur_it->x, px_ref_it->y - px_cur_it->y).norm());//添加对应特征之间的距离
			++px_ref_it;
			++px_cur_it;
			++f_ref_it;
		}
	}

	void Initialization::computeHomography(
		const std::vector<Vector3d>& f_ref,
		const std::vector<Vector3d>& f_cur,
		double focal_length,
		double reprojection_threshold,
		std::vector<int>& inliers,
		std::vector<Vector3d>& xyz_in_cur,
		SE3& T_cur_from_ref)
	{
		std::vector<Vector2d, aligned_allocator<Vector2d> > uv_ref(f_ref.size());
		std::vector<Vector2d, aligned_allocator<Vector2d> > uv_cur(f_cur.size());
		for (size_t i = 0, i_max = f_ref.size(); i < i_max; ++i)
		{
			uv_ref[i] = project2d(f_ref[i]);
			uv_cur[i] = project2d(f_cur[i]);
		}
		Homography Homography(uv_ref, uv_cur, focal_length, reprojection_threshold);
		Homography.computeSE3fromMatches();
		std::vector<int> outliers;
		computeInliers(f_cur, f_ref,
			Homography.T_c2_from_c1_.rotation_matrix(), Homography.T_c2_from_c1_.translation(),
			reprojection_threshold, focal_length,
			xyz_in_cur, inliers, outliers);
		T_cur_from_ref = Homography.T_c2_from_c1_;
	}
}