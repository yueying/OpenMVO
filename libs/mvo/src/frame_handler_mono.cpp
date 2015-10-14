#define NOMINMAX
#include <openmvo/mvo/config.h>
#include <openmvo/mvo/frame_handler_mono.h>
#include <openmvo/mvo/map.h>
#include <openmvo/mvo/frame.h>
#include <openmvo/mvo/feature.h>
#include <openmvo/mvo/point3d.h>
#include <openmvo/mvo/pose_optimizer.h>
#include <openmvo/mvo/sparse_img_align.h>
#include <openmvo/mvo/depth_filter.h>
#include <openmvo/mvo/fast_detector.h>
#include <openmvo/mvo/structure_optimizer.h>

namespace mvo {

	FrameHandlerMono::FrameHandlerMono(AbstractCamera* cam) :
		FrameHandlerBase(),
		cam_(cam),
		reprojector_(cam_, map_),
		depth_filter_(NULL)
	{
		initialize();
	}

	void FrameHandlerMono::initialize()
	{
		DetectorPtr feature_detector(
			new FastDetector(
			cam_->width(), cam_->height(), Config::gridSize(), Config::nPyrLevels()));
		DepthFilter::callback_t depth_filter_cb = std::bind(
			&MapPointCandidates::newCandidatePoint, &map_.point_candidates_, std::placeholders::_1, std::placeholders::_2);
		depth_filter_ = new DepthFilter(feature_detector, depth_filter_cb);
		// 在初始化的时候，深度估计算法线程启动
		depth_filter_->startThread(); 
	}

	FrameHandlerMono::~FrameHandlerMono()
	{
		delete depth_filter_;
	}

	void FrameHandlerMono::addImage(const cv::Mat& img, const double timestamp)
	{
		if (!startFrameProcessingCommon(timestamp))
			return;

		// 对上一次迭代的清除，之所以在这边处理，为了可视化的需求
		core_kfs_.clear();
		overlap_kfs_.clear();

		// 创建新帧
		new_frame_.reset(new Frame(cam_, img.clone(), timestamp));

		// 处理帧
		UpdateResult res = RESULT_FAILURE;
		if (stage_ == STAGE_DEFAULT_FRAME)
			res = processFrame();
		else if (stage_ == STAGE_SECOND_FRAME)
			res = processSecondFrame();
		else if (stage_ == STAGE_FIRST_FRAME)
			res = processFirstFrame();
		else if (stage_ == STAGE_RELOCALIZING)
			res = relocalizeFrame(SE3(Matrix3d::Identity(), Vector3d::Zero()),
			map_.getClosestKeyframe(last_frame_));

		// 设置最近的一帧
		last_frame_ = new_frame_;
		new_frame_.reset();

		// 结束处理
		finishFrameProcessingCommon(last_frame_->id_, res, last_frame_->nObs());
	}

	// 根据检测到的特征点数目确保进入第一帧图像，并确定关键的5个特征
	FrameHandlerMono::UpdateResult FrameHandlerMono::processFirstFrame()
	{
		new_frame_->T_f_w_ = SE3(Matrix3d::Identity(), Vector3d::Zero());
		if (klt_homography_init_.addFirstFrame(new_frame_) == FAILURE)
			return RESULT_NO_KEYFRAME;
		// 选择第一帧没有失败，则设置这一帧为关键帧，设置关键帧中的5个特征（分散的特征）
		new_frame_->setKeyframe();
		// 将关键帧添加到地图中
		map_.addKeyframe(new_frame_);
		// 修改状态，表明第一帧已经处理完成
		stage_ = STAGE_SECOND_FRAME;
		return RESULT_IS_KEYFRAME;
	}

	FrameHandlerBase::UpdateResult FrameHandlerMono::processSecondFrame()
	{
		// 光流跟踪成功，几何校正通过单应关系成功，确定第二帧，最终确定两帧特征
		InitResult res = klt_homography_init_.addSecondFrame(new_frame_);
		if (res == FAILURE)
			return RESULT_FAILURE;
		else if (res == NO_KEYFRAME)
			return RESULT_NO_KEYFRAME;

		// 设置这个帧为关键帧
		new_frame_->setKeyframe();
		// 计算帧对应特征的平均深度，及最小深度，传入深度估计线程中
		double depth_mean, depth_min;
		frame_utils::getSceneDepth(*new_frame_, depth_mean, depth_min);
		// 将关键帧添加到深度估计算法队列中，由深度估计线程处理
		depth_filter_->addKeyframe(new_frame_, depth_mean, 0.5*depth_min);

		// 将关键帧添加到地图中
		map_.addKeyframe(new_frame_);
		stage_ = STAGE_DEFAULT_FRAME;// 设置状态处理默认帧
		klt_homography_init_.reset();
		std::cout<<"Init: Selected second frame, triangulated initial map.";
		return RESULT_IS_KEYFRAME;
	}

	FrameHandlerBase::UpdateResult FrameHandlerMono::processFrame()
	{
		// 设置初始位姿使用之前的
		new_frame_->T_f_w_ = last_frame_->T_f_w_;

		// Sparse Imgage Align
		SparseImgAlign img_align(Config::kltMaxLevel(), Config::kltMinLevel(),
			30, SparseImgAlign::GaussNewton, false, false);
		size_t img_align_n_tracked = img_align.run(last_frame_, new_frame_);
		std::cout<<"Img Align:\t Tracked = " << img_align_n_tracked<<std::endl;

		// map reprojection & feature alignment
		reprojector_.reprojectMap(new_frame_, overlap_kfs_);
		const size_t repr_n_new_references = reprojector_.n_matches_;
		const size_t repr_n_mps = reprojector_.n_trials_;

		std::cout<<"Reprojection:\t nPoints = " << repr_n_mps << "\t \t nMatches = " << repr_n_new_references<<std::endl;

		if (repr_n_new_references < Config::qualityMinFts())
		{
			std::cerr<<"Not enough matched features.";
			new_frame_->T_f_w_ = last_frame_->T_f_w_; // 跟踪数较小时，设置这帧为上一帧，避免突然的跳动
			tracking_quality_ = TRACKING_INSUFFICIENT;
			return RESULT_FAILURE;
		}

		// pose optimization
		size_t sfba_n_edges_final;
		double sfba_thresh, sfba_error_init, sfba_error_final;
		poseOptimize(
			Config::poseOptimThresh(), Config::poseOptimNumIter(), false,
			new_frame_, sfba_thresh, sfba_error_init, sfba_error_final, sfba_n_edges_final);

		std::cout<<"PoseOptimizer:\t ErrInit = " << sfba_error_init << "px\t thresh = " << sfba_thresh<<std::endl;
		std::cout<<"PoseOptimizer:\t ErrFin. = " << sfba_error_final << "px\t nObsFin. = " << sfba_n_edges_final<<std::endl;
		if (sfba_n_edges_final < 20)
			return RESULT_FAILURE;

		// 结构优化
		structureOptimize(new_frame_, Config::structureOptimMaxPts(), Config::structureOptimNumIter());

		// 选择关键帧
		core_kfs_.insert(new_frame_);
		setTrackingQuality(sfba_n_edges_final);
		if (tracking_quality_ == TRACKING_INSUFFICIENT)
		{
			new_frame_->T_f_w_ = last_frame_->T_f_w_; // reset to avoid crazy pose jumps
			return RESULT_FAILURE;
		}
		double depth_mean, depth_min;
		frame_utils::getSceneDepth(*new_frame_, depth_mean, depth_min);
		if (!needNewKf(depth_mean) || tracking_quality_ == TRACKING_BAD)
		{
			depth_filter_->addFrame(new_frame_);
			return RESULT_NO_KEYFRAME;
		}
		new_frame_->setKeyframe();
		std::cout<<"New keyframe selected."<<std::endl;

		// 选择新的关键帧
		for (Features::iterator it = new_frame_->fts_.begin(); it != new_frame_->fts_.end(); ++it)
			if ((*it)->point != NULL)
				(*it)->point->addFrameRef(*it);
		map_.point_candidates_.addCandidatePointToFrame(new_frame_);

		// 初始化depth-filters
		depth_filter_->addKeyframe(new_frame_, depth_mean, 0.5*depth_min);

		// 如果达到了关键帧的最大数目，则删除最远的关键帧
		if (Config::maxNKfs() > 2 && map_.size() >= Config::maxNKfs())
		{
			FramePtr furthest_frame = map_.getFurthestKeyframe(new_frame_->pos());
			depth_filter_->removeKeyframe(furthest_frame); // TODO this interrupts the mapper thread, maybe we can solve this better
			map_.safeDeleteFrame(furthest_frame);
		}

		// 添加关键帧到map中
		map_.addKeyframe(new_frame_);

		return RESULT_IS_KEYFRAME;
	}

	FrameHandlerMono::UpdateResult FrameHandlerMono::relocalizeFrame(
		const SE3& T_cur_ref,
		FramePtr ref_keyframe)
	{
		std::cout<<"Relocalizing frame"<<std::endl;
		if (ref_keyframe == nullptr)
		{
			std::cerr<<"No reference keyframe."<<std::endl;
			return RESULT_FAILURE;
		}
		SparseImgAlign img_align(Config::kltMaxLevel(), Config::kltMinLevel(),
			30, SparseImgAlign::GaussNewton, false, false);
		size_t img_align_n_tracked = img_align.run(ref_keyframe, new_frame_);
		if (img_align_n_tracked > 30)
		{
			SE3 T_f_w_last = last_frame_->T_f_w_;
			last_frame_ = ref_keyframe;
			FrameHandlerMono::UpdateResult res = processFrame();
			if (res != RESULT_FAILURE)
			{
				stage_ = STAGE_DEFAULT_FRAME;
				std::cout<<"Relocalization successful."<<std::endl;
			}
			else
				new_frame_->T_f_w_ = T_f_w_last; // reset to last well localized pose
			return res;
		}
		return RESULT_FAILURE;
	}

	bool FrameHandlerMono::relocalizeFrameAtPose(
		const int keyframe_id,
		const SE3& T_f_kf,
		const cv::Mat& img,
		const double timestamp)
	{
		FramePtr ref_keyframe;
		if (!map_.getKeyframeById(keyframe_id, ref_keyframe))
			return false;
		new_frame_.reset(new Frame(cam_, img.clone(), timestamp));
		UpdateResult res = relocalizeFrame(T_f_kf, ref_keyframe);
		if (res != RESULT_FAILURE) {
			last_frame_ = new_frame_;
			return true;
		}
		return false;
	}

	void FrameHandlerMono::resetAll()
	{
		resetCommon();
		last_frame_.reset();
		new_frame_.reset();
		core_kfs_.clear();
		overlap_kfs_.clear();
		depth_filter_->reset();
	}

	bool FrameHandlerMono::needNewKf(double scene_depth_mean)
	{
		for (auto it = overlap_kfs_.begin(), ite = overlap_kfs_.end(); it != ite; ++it)
		{
			Vector3d relpos = new_frame_->w2f(it->first->pos());
			if (fabs(relpos.x()) / scene_depth_mean < Config::kfSelectMinDist() &&
				fabs(relpos.y()) / scene_depth_mean < Config::kfSelectMinDist()*0.8 &&
				fabs(relpos.z()) / scene_depth_mean < Config::kfSelectMinDist()*1.3)
				return false;
		}
		return true;
	}

} // namespace svo
