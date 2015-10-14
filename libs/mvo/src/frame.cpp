/*************************************************************************
 * 文件名： frame
 *
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 * 时间： 2015/8/6
 *
 * 说明： 帧
 *************************************************************************/

#include "openmvo/mvo/frame.h"
#include "openmvo/utils/image_utils.h"
#include "openmvo/mvo/feature.h"
#include <openmvo/utils/math_utils.h>

namespace mvo
{
	int Frame::frame_counter_ = 0;

	Frame::Frame(AbstractCamera* cam, const cv::Mat& img, double timestamp) :
		id_(frame_counter_++),
		timestamp_(timestamp),
		cam_(cam),
		key_pts_(5),
		is_keyframe_(false)
	{
		initFrame(img);
	}

	Frame::~Frame()
	{
		//std::for_each(fts_.begin(), fts_.end(), [&](Feature* i){if (i != NULL) { delete i; i = NULL; } });
	}

	void Frame::initFrame(const cv::Mat& img)
	{
		// 检测图像，保证图像大小与相机模型大小一致，以及图像为灰度图像
		if (img.empty() || img.type() != CV_8UC1 || img.cols != cam_->width() || img.rows != cam_->height())
			throw std::runtime_error("Frame: provided image has not the same size as the camera model or image is not grayscale");
		// 设置关键帧对应的5个关键特征点初始化为 NULL
		std::for_each(key_pts_.begin(), key_pts_.end(), [&](Feature* ftr){ ftr = NULL; });
		// 构建图像金字塔，默认金字塔的等级为5
		createImgPyramid(img, 5, img_pyr_);
	}

	void Frame::setKeyframe()
	{
		// 标识这一帧为关键帧
		is_keyframe_ = true;
		setKeyPoints();
	}

	/// 给帧中添加特征
	void Frame::addFeature(Feature* ftr)
	{
		fts_.push_back(ftr);
	}

	void Frame::setKeyPoints()
	{
		// 如果特征指向的3d点为空，则设置该特征为NULL
		for (size_t i = 0; i < 5; ++i)
			if (key_pts_[i] != NULL)
				if (key_pts_[i]->point == NULL)
					key_pts_[i] = NULL;
		// 找到5个特征
		std::for_each(fts_.begin(), fts_.end(), [&](Feature* ftr){ if (ftr->point != NULL) checkKeyPoints(ftr); });
	}

	void Frame::checkKeyPoints(Feature* ftr)
	{
		// 首先得到相机中心点
		const int cu = cam_->width() / 2;
		const int cv = cam_->height() / 2;

		// 如果第一个特征为空，则进入的第一个特征转为关键特征，如果不为空，则判断新进入的特征是否比之前特征
		// 更接近中心，是，则替换该特征
		if (key_pts_[0] == NULL)
			key_pts_[0] = ftr;
		else if (std::max(std::fabs(ftr->px[0] - cu), std::fabs(ftr->px[1] - cv))
			< std::max(std::fabs(key_pts_[0]->px[0] - cu), std::fabs(key_pts_[0]->px[1] - cv)))
			key_pts_[0] = ftr;
		// 找到中间的特征之后，将图片分成4块，在每块中找出1个特征，离中心越远的特征
		if (ftr->px[0] >= cu && ftr->px[1] >= cv)
		{
			if (key_pts_[1] == NULL)
				key_pts_[1] = ftr;
			else if ((ftr->px[0] - cu) * (ftr->px[1] - cv)
			> (key_pts_[1]->px[0] - cu) * (key_pts_[1]->px[1] - cv))
			key_pts_[1] = ftr;
		}
		if (ftr->px[0] >= cu && ftr->px[1] < cv)
		{
			if (key_pts_[2] == NULL)
				key_pts_[2] = ftr;
			else if ((ftr->px[0] - cu) * (ftr->px[1] - cv)
		> (key_pts_[2]->px[0] - cu) * (key_pts_[2]->px[1] - cv))
		key_pts_[2] = ftr;
		}
		if (ftr->px[0] < cu && ftr->px[1] < cv)
		{
			if (key_pts_[3] == NULL)
				key_pts_[3] = ftr;
			else if ((ftr->px[0] - cu) * (ftr->px[1] - cv)
		> (key_pts_[3]->px[0] - cu) * (key_pts_[3]->px[1] - cv))
		key_pts_[3] = ftr;
		}
		if (ftr->px[0] < cu && ftr->px[1] >= cv)
		{
			if (key_pts_[4] == NULL)
				key_pts_[4] = ftr;
			else if ((ftr->px[0] - cu) * (ftr->px[1] - cv)
		> (key_pts_[4]->px[0] - cu) * (key_pts_[4]->px[1] - cv))
		key_pts_[4] = ftr;
		}
	}

	void Frame::removeKeyPoint(Feature* ftr)
	{
		bool found = false;
		std::for_each(key_pts_.begin(), key_pts_.end(), [&](Feature*& i){
			if (i == ftr) {
				i = NULL;
				found = true;
			}
		});
		if (found)
			setKeyPoints();
	}

	/// 检测在世界坐标系的点是否在图像中可见
	bool Frame::isVisible(const Vector3d& xyz_w) const
	{
		Vector3d xyz_f = T_f_w_*xyz_w;
		if (xyz_f.z() < 0.0)
			return false; // 点在相机的背后
		Vector2d px = f2c(xyz_f);
		if (px[0] >= 0.0 && px[1] >= 0.0 && px[0] < cam_->width() && px[1] < cam_->height())
			return true;
		return false;
	}

	// 创建图像金字塔
	void Frame::createImgPyramid(const cv::Mat& img_level_0, int n_levels, ImgPyr& pyr)
	{
		pyr.resize(n_levels);// 构建金字塔图像，共5层
		pyr[0] = img_level_0;
		for (int i = 1; i < n_levels; ++i)
		{
			pyr[i] = cv::Mat(pyr[i - 1].rows / 2, pyr[i - 1].cols / 2, CV_8U);
			halfSample(pyr[i - 1], pyr[i]);
		}
	}
	/// 帧处理的实用函数
	namespace frame_utils {

		// 计算一帧中特征中有对应的3D点，计算其离相机的深度，给出最小深度，和平均深度
		bool getSceneDepth(const Frame& frame, double& depth_mean, double& depth_min)
		{
			std::vector<double> depth_vec;
			depth_vec.reserve(frame.fts_.size());
			depth_min = std::numeric_limits<double>::max();
			for (auto it = frame.fts_.begin(), ite = frame.fts_.end(); it != ite; ++it)
			{
				if ((*it)->point != NULL)
				{
					// 计算离相机的深度
					const double z = frame.w2f((*it)->point->pos_).z();
					depth_vec.push_back(z);
					depth_min = fmin(z, depth_min);
				}
			}
			if (depth_vec.empty())
			{
				return false;
			}
			depth_mean = getMedian(depth_vec);
			return true;
		}

	} // namespace frame_utils
	
}