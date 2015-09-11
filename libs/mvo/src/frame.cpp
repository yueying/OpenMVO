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

namespace mvo
{
	int Frame::frame_counter_ = 0;

	Frame::Frame(AbstractCamera* cam, const cv::Mat& img, double timestamp) :
		id_(frame_counter_++),
		timestamp_(timestamp),
		cam_(cam)
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

		// 构建图像金字塔，默认金字塔的等级为5
		createImgPyramid(img, 5, img_pyr_);
	}

	/// 给帧中添加特征
	void Frame::addFeature(Feature* ftr)
	{
		fts_.push_back(ftr);
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

	
}