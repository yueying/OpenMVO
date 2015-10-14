#ifndef OPENMVO_MVO_FRAME_HANDLER_BASE_H_
#define OPENMVO_MVO_FRAME_HANDLER_BASE_H_

#include <queue>
#include <openmvo/utils/noncopyable.h>
#include <openmvo/mvo/map.h>
#include <openmvo/utils/timer.h>

namespace mvo
{
	class Point3D;
	class Matcher;
	class DepthFilter;
	class AbstractCamera;

	/// 视觉里程计帧处理基类，用于管理地图和状态机
	class FrameHandlerBase : Noncopyable
	{
	public:
		//算法所处的状态
		enum Stage {
			STAGE_PAUSED,//暂停
			STAGE_FIRST_FRAME,//第一帧
			STAGE_SECOND_FRAME,//第二帧
			STAGE_DEFAULT_FRAME,//默认帧
			STAGE_RELOCALIZING//重新定位
		};
		//跟踪质量
		enum TrackingQuality {
			TRACKING_INSUFFICIENT,//不够跟踪
			TRACKING_BAD,//不好的跟踪
			TRACKING_GOOD//好的跟踪
		};
		//更新结果
		enum UpdateResult {
			RESULT_NO_KEYFRAME,//不是关键帧
			RESULT_IS_KEYFRAME,//是关键帧
			RESULT_FAILURE//失败
		};

		FrameHandlerBase();

		virtual ~FrameHandlerBase();

		/// 得到当前地图
		const Map& map() const { return map_; }

		/// 当前帧处理完成后就重置地图
		void reset() { set_reset_ = true; }

		/// 开始处理
		void start() { set_start_ = true; }

		/// 得到当前算法所处的阶段
		Stage stage() const { return stage_; }

		/// 得到目前跟踪质量
		TrackingQuality trackingQuality() const { return tracking_quality_; }

		/// 得到上一次迭代的处理时间
		double lastProcessingTime() const { return timer_.getTime(); }

		/// 得到上一帧观察到的特征数目
		size_t lastNumObservations() const { return num_obs_last_; }

	protected:
		Stage stage_;                 //!< 算法当前所处的阶段
		bool set_reset_;              //!< 用户可以设置的标识，表示在下一次迭代之前重新设置系统
		bool set_start_;              //!< 用户可以设置的标识，表示在获得下一个图像后启动系统
		Map map_;                     //!< 由关键帧构建的地图
		Timer timer_;             //!< 码表用于测量处理时间
		size_t num_obs_last_;                         //!< 在前一帧中观察到的特征数
		TrackingQuality tracking_quality_;            //!< 跟踪质量的估计，根据跟踪的特征数

		/// 在帧处理之前，该函数被调用
		bool startFrameProcessingCommon(const double timestamp);

		/// 帧处理完成之后，该函数被调用
		int finishFrameProcessingCommon(
			const size_t update_id,
			const UpdateResult dropout,
			const size_t num_observations);

		/// 重置的地图和帧的处理程序，从零开始
		void resetCommon();

		/// 重置帧处理程序，在派生类中实现
		virtual void resetAll() { resetCommon(); }

		/// 设置跟踪质量，跟踪质量取决于跟踪的特征数
		virtual void setTrackingQuality(const size_t num_observations);

	};

} // namespace mvo

#endif // OPENMVO_MVO_FRAME_HANDLER_BASE_H_
