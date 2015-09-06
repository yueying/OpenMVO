/*************************************************************************
 * 文件名： frame
 *
 * 作者： 冯兵
 * 时间： 2015/8/1
 *
 * 说明： 帧定义
 *************************************************************************/
#ifndef OPENMVO_MVO_FRAME_H_
#define OPENMVO_MVO_FRAME_H_

#include <vector>
#include <list>
#include <memory>
#include <opencv2/core/core.hpp>
#include "openmvo/utils/noncopyable.h"
#include "openmvo/mvo/abstract_camera.h"

namespace mvo{

	struct Feature;

	typedef std::list<Feature*> Features;//特征list
	typedef std::vector<cv::Mat> ImgPyr;//图像金字塔

	/**	定义帧，保证帧的唯一性
	 */
	class Frame : public Noncopyable
	{
	public:
		/**	帧的实例化，通过传入相机参数，获得的当前帧，及时间戳来确定
		 */
		Frame(AbstractCamera* cam, const cv::Mat& img, double timestamp);
		~Frame();
		/// 初始化新的图像帧，创建图像金字塔
		void initFrame(const cv::Mat& img);

	private:
		/// 通过半采用的方式创建图像金字塔
		void createImgPyramid(const cv::Mat& img_level_0, int n_levels, ImgPyr& pyr);
		
	public:
		static int                    frame_counter_;         //!< 创建帧的计数器，用于设置帧的唯一id
		int                           id_;                    //!< 帧的唯一id
		double                        timestamp_;             //!< 帧被记录的时间戳
		AbstractCamera                *cam_;                  //!< 相机模型
		ImgPyr                        img_pyr_;               //!< 图像金字塔
		Features                      fts_;                   //!< 图像中的特征List
	};
	typedef std::shared_ptr<Frame> FramePtr;
}

#endif // OPENMVO_MVO_FRAME_H_

