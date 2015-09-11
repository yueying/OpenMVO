/*************************************************************************
 * 文件名： initialization
 *
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 * 时间： 2015/9/7
 *
 * 说明： 初始化，确定初始位置，参考rpg_svo(https://github.com/uzh-rpg/rpg_svo)
 *************************************************************************/
#ifndef OPENMVO_MVO_INITIALIZATION_H_
#define OPENMVO_MVO_INITIALIZATION_H_

#include <Eigen/Core>
#include <sophus/se3.h>
#include "openmvo/mvo/frame.h"

namespace mvo
{
	using namespace Eigen;
	using namespace Sophus;

	enum InitResult
	{
		FAILURE, // 失败
		NO_KEYFRAME, // 没有关键帧
		SUCCESS // 成功
	};

	class Initialization {

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		
		Initialization() {};
		~Initialization() {};
		InitResult addFirstFrame(FramePtr frame_ref);
		InitResult addSecondFrame(FramePtr frame_ref);
		void reset();

	protected:
		/// 检测图像中Fast角点
		void detectFeatures(
			FramePtr frame,
			std::vector<cv::Point2f>& px_vec,
			std::vector<Vector3d>& f_vec);

		/// 对选择的特征点计算光流(Lucas Kanade)
		void trackKlt(
			FramePtr frame_ref,
			FramePtr frame_cur,
			std::vector<cv::Point2f>& px_ref,
			std::vector<cv::Point2f>& px_cur,
			std::vector<Vector3d>& f_ref,
			std::vector<Vector3d>& f_cur,
			std::vector<double>& disparities);

		/// 计算单应矩阵
		void computeHomography(
			const std::vector<Vector3d>& f_ref,
			const std::vector<Vector3d>& f_cur,
			double focal_length,
			double reprojection_threshold,
			std::vector<int>& inliers,
			std::vector<Vector3d>& xyz_in_cur,
			SE3& T_cur_from_ref);

	protected:
		FramePtr frame_ref_;                   //!< 参考帧
		std::vector<cv::Point2f> px_ref_;      //!< 在参考帧中用于跟踪的特征点
		std::vector<cv::Point2f> px_cur_;      //!< 在当前帧中跟踪的特征点
		std::vector<Vector3d> f_ref_;          //!< 对应参考图像中的特征点单位方向向量
		std::vector<Vector3d> f_cur_;          //!< 对应当前图像中的特征点单位方向向量
		std::vector<double> disparities_;      //!< 第一帧与第二帧对应光流跟踪的特征之间的像素差值
		std::vector<int> inliers_;             //!< 进行几何检测之后的内点（如单应变换）
		std::vector<Vector3d> xyz_in_cur_;     //!< 当前计算的3D点
		SE3 T_cur_from_ref_;                   //!< 计算开始的两帧的变换关系
	};
}

#endif // OPENMVO_MVO_INITIALIZATION_H_