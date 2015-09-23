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

		/// 选择这个帧作为关键帧，主要用于设置关键帧的关键点
		void setKeyframe();

		/// 往帧中添加特征
		void addFeature(Feature* ftr);

		/// 这些点用于快速检测是否两个帧有重叠的视野，选取5个特征，一个在图像中点
		/// 另外4个靠近图像的4个边角，并且这5个特征都要有对应的3D点。
		void setKeyPoints();

		/// 检测是否我们选择了5个较好的特征点
		void checkKeyPoints(Feature* ftr);

		/// 如果一个特征被删除，我们必须移除与其可能对应的关键特征
		void removeKeyPoint(Feature* ftr);

		/// 检测在世界坐标系的点是否在图像中可见
		bool isVisible(const Vector3d& xyz_w) const;

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

		/// 当前摄像机坐标系下3D点xyz到单位平面坐标uv(focal length = 1)的雅克比矩阵
		inline static void jacobian_xyz2uv(
			const Vector3d& xyz_in_f,
			Matrix<double, 2, 6>& J)
		{
			const double x = xyz_in_f[0];
			const double y = xyz_in_f[1];
			const double z_inv = 1. / xyz_in_f[2];
			const double z_inv_2 = z_inv*z_inv;

			J(0, 0) = -z_inv;              // -1/z
			J(0, 1) = 0.0;                 // 0
			J(0, 2) = x*z_inv_2;           // x/z^2
			J(0, 3) = y*J(0, 2);            // x*y/z^2
			J(0, 4) = -(1.0 + x*J(0, 2));   // -(1.0 + x^2/z^2)
			J(0, 5) = y*z_inv;             // y/z

			J(1, 0) = 0.0;                 // 0
			J(1, 1) = -z_inv;              // -1/z
			J(1, 2) = y*z_inv_2;           // y/z^2
			J(1, 3) = 1.0 + y*J(1, 2);      // 1.0 + y^2/z^2
			J(1, 4) = -J(0, 3);             // -x*y/z^2
			J(1, 5) = -x*z_inv;            // x/z
		}
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
		std::vector<Feature*>         key_pts_;               //!< 使用5个特征及关联的3D点，用于检测两帧之间是否有重叠的视野
		bool                          is_keyframe_;           //!< 该帧是否选择为关键帧
	};
	typedef std::shared_ptr<Frame> FramePtr;
}

#endif // OPENMVO_MVO_FRAME_H_

