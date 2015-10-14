#define  NOMINMAX
#include <openmvo/mvo/frame_handler_base.h>
#include <fstream>
#include <algorithm>

#include <Eigen/StdVector>

#include <openmvo/mvo/abstract_camera.h>
#include <openmvo/mvo/config.h>
#include <openmvo/mvo/feature.h>
#include <openmvo/mvo/matcher.h>
#include <openmvo/mvo/map.h>
#include <openmvo/mvo/point3d.h>

namespace mvo
{
	FrameHandlerBase::FrameHandlerBase() :
		stage_(STAGE_PAUSED),
		set_reset_(false),
		set_start_(false),
		num_obs_last_(0),
		tracking_quality_(TRACKING_INSUFFICIENT)
	{
	}

	FrameHandlerBase::~FrameHandlerBase()
	{
	}

	bool FrameHandlerBase::startFrameProcessingCommon(const double timestamp)
	{
		if (set_start_)
		{
			resetAll();
			stage_ = STAGE_FIRST_FRAME;
		}

		if (stage_ == STAGE_PAUSED)
			return false;

		timer_.start();

		// 对最近一次迭代的清理工作，不在之前做主要是因为可视化
		map_.emptyTrash();
		return true;
	}

	int FrameHandlerBase::finishFrameProcessingCommon(
		const size_t update_id,
		const UpdateResult dropout,
		const size_t num_observations)
	{
		std::cout << "Frame: " << update_id << std::endl;
		num_obs_last_ = num_observations;


		if (dropout == RESULT_FAILURE &&
			(stage_ == STAGE_DEFAULT_FRAME || stage_ == STAGE_RELOCALIZING))
		{
			stage_ = STAGE_RELOCALIZING;
			tracking_quality_ = TRACKING_INSUFFICIENT;
		}
		else if (dropout == RESULT_FAILURE)
			resetAll();
		if (set_reset_)
			resetAll();

		return 0;
	}

	void FrameHandlerBase::resetCommon()
	{
		map_.reset();
		stage_ = STAGE_PAUSED;
		set_reset_ = false;
		set_start_ = false;
		tracking_quality_ = TRACKING_INSUFFICIENT;
		num_obs_last_ = 0;
	}

	void FrameHandlerBase::setTrackingQuality(const size_t num_observations)
	{
		tracking_quality_ = TRACKING_GOOD;
		if (num_observations < Config::qualityMinFts())
		{
			std::cout << "Tracking less than " << Config::qualityMinFts() << " features!" << std::endl;
			tracking_quality_ = TRACKING_INSUFFICIENT;
		}
		const int feature_drop = static_cast<int>(std::min(num_obs_last_, Config::maxFts())) - num_observations;
		if (feature_drop > Config::qualityMaxFtsDrop())
		{
			std::cout << "Lost " << feature_drop << " features!" << std::endl;
			tracking_quality_ = TRACKING_INSUFFICIENT;
		}
	}

} // namespace mvo
