/*************************************************************************
 * 文件名： frame
 *
 * 作者： 冯兵
 * 时间： 2015/8/1
 *
 * 说明： 帧定义参考rpg_svo(https://github.com/uzh-rpg/rpg_svo)
 *************************************************************************/
#ifndef OPENMVO_MVO_FRAME_H_
#define OPENMVO_MVO_FRAME_H_

#include <vector>
#include <list>
#include <memory>
#include <opencv2/core/core.hpp>
#include <sophus/se3.h>
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
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		/**	帧的实例化，通过传入相机参数，获得的当前帧，及时间戳来确定
		 */
		Frame(AbstractCamera* cam, const cv::Mat& img, double timestamp);
		~Frame();
		/// 初始化新的图像帧，创建图像金字塔
		void initFrame(const cv::Mat& img);

		/// 往帧中添加特征
		void addFeature(Feature* ftr);

		/// 得到帧所对应的原始图像
		inline const cv::Mat& img() const { return img_pyr_[0]; }
		/// 将世界坐标系中的点转到像素坐标
		inline Vector2d w2c(const Vector3d& xyz_w) const { return cam_->world2cam(T_f_w_ * xyz_w); }

		/// 将像素坐标转到单位摄像机坐标
		inline Vector3d c2f(const Vector2d& px) const { return cam_->cam2world(px[0], px[1]); }

		/// 将像素坐标转到单位摄像机坐标
		inline Vector3d c2f(const double x, const double y) const { return cam_->cam2world(x, y); }

		/// 将世界坐标系的点转到相机坐标系
		inline Vector3d w2f(const Vector3d& xyz_w) const { return T_f_w_ * xyz_w; }

		///将相机坐标系下的点转到世界坐标系下
		inline Vector3d f2w(const Vector3d& f) const { return T_f_w_.inverse() * f; }

		/// 摄像机坐标系下的点转像素坐标
		inline Vector2d f2c(const Vector3d& f) const { return cam_->world2cam(f); }
		/// 返回帧在世界坐标系中的位置
		inline Vector3d pos() const { return T_f_w_.inverse().translation(); }

	private:
		/// 通过半采用的方式创建图像金字塔
		void createImgPyramid(const cv::Mat& img_level_0, int n_levels, ImgPyr& pyr);
		
	public:
		static int                    frame_counter_;         //!< 创建帧的计数器，用于设置帧的唯一id
		int                           id_;                    //!< 帧的唯一id
		double                        timestamp_;             //!< 帧被记录的时间戳
		AbstractCamera                *cam_;                  //!< 相机模型
		Sophus::SE3                   T_f_w_;                 //!< 从世界坐标系(w)orld转到摄像机坐标系(f)rame，刚性变换Rt
		ImgPyr                        img_pyr_;               //!< 图像金字塔
		Features                      fts_;                   //!< 图像中的特征List
	};
	typedef std::shared_ptr<Frame> FramePtr;
}

#endif // OPENMVO_MVO_FRAME_H_

