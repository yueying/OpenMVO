#ifndef OPENMVO_MVO_FRAME_HANDLER_MONO_H_
#define OPENMVO_MVO_FRAME_HANDLER_MONO_H_

#include <set>
#include <openmvo/mvo/abstract_camera.h>
#include <openmvo/mvo/frame_handler_base.h>
#include <openmvo/mvo/reprojector.h>
#include <openmvo/mvo/initialization.h>

namespace mvo {

	/// 单目视觉里程计的处理流
	class FrameHandlerMono : public FrameHandlerBase
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

			FrameHandlerMono(AbstractCamera* cam);
		virtual ~FrameHandlerMono();

		/// 提供一个图像
		void addImage(const cv::Mat& img, double timestamp);

		/// 获取被处理的最后一帧
		FramePtr lastFrame() { return last_frame_; }

		/// 一个外面的位置识别模块，用于确定在什么地方需要重新定位，目前用不着
		bool relocalizeFrameAtPose(
			const int keyframe_id,
			const SE3& T_kf_f,
			const cv::Mat& img,
			const double timestamp);

	protected:
		AbstractCamera* cam_;                     //!< 相机模型，目前为针孔相机模型，可以是ATAN或Ocam模型
		Reprojector reprojector_;                     //!< 将其他帧上对应的3D点投影到当前帧上
		FramePtr new_frame_;                          //!< 当前帧
		FramePtr last_frame_;                         //!< 上一帧，不必一定是关键帧
		std::set<FramePtr> core_kfs_;                      //!< 靠近邻居的关键帧
		std::vector< std::pair<FramePtr, size_t> > overlap_kfs_; //!< 所有的关键帧有重叠视野. pair的第二个数表示有多少个共同的可观测点
		Initialization klt_homography_init_; //!< 用来估计开始两个关键帧的位姿通过估计两帧的单应矩阵
		DepthFilter* depth_filter_;                   //!< 深度估计算法，其运行在单独的线程，用来初始化新的3D点

		/// 单目视觉里程计算法初始化
		virtual void initialize();

		/// 处理第一帧，并设置这一帧为关键帧
		virtual UpdateResult processFirstFrame();

		/// 处理接下来的所有帧，直到下一个关键帧被选择
		virtual UpdateResult processSecondFrame();

		/// 处理完开始的两个关键帧后处理所有帧
		virtual UpdateResult processFrame();

		/// 根据上一帧的位置进行重新定位
		virtual UpdateResult relocalizeFrame(
			const SE3& T_cur_ref,
			FramePtr ref_keyframe);

		/// 重新设置关键处理程序
		virtual void resetAll();

		/// 关键帧选择标准
		virtual bool needNewKf(double scene_depth_mean);

	};

} // namespace mvo


#endif // OPENMVO_MVO_FRAME_HANDLER_MONO_H_
