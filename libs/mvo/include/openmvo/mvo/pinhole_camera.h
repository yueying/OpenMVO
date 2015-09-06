/*************************************************************************
 * 文件名： pinhole_camera
 *
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 * 时间： 2015/8/1
 *
 * 说明： 参考rpg_svo(https://github.com/uzh-rpg/rpg_svo)
 *************************************************************************/
#ifndef OPENMVO_MVO_PINHOLE_CAMERA_H_
#define OPENMVO_MVO_PINHOLE_CAMERA_H_

#include <opencv2/opencv.hpp>
#include "openmvo/mvo/abstract_camera.h"

namespace mvo
{
	class PinholeCamera : public AbstractCamera {
	
	public:
		// 考虑畸变参数k1,k2,p1,p2,k3
		PinholeCamera(double width, double height,
			double fx, double fy, double cx, double cy,
			double k1 = 0.0, double k2 = 0.0, double p1 = 0.0, double p2 = 0.0, double k3 = 0.0);

		~PinholeCamera();

		/// 图像像素坐标转摄像机坐标系下的点
		virtual Vector3d cam2world(const double& x, const double& y) const;

		/// 图像像素坐标转摄像机坐标系下的点
		virtual Vector3d cam2world(const Vector2d& px) const;

		/// 摄像机坐标系下的点转图像像素坐标
		virtual Vector2d world2cam(const Vector3d& xyz_c) const;

		/// 图像平面像素的世界坐标转像素坐标
		virtual Vector2d world2cam(const Vector2d& uv) const;

		/// 返回x方向的焦距值
		virtual double getFocalLength() const
		{
			return fabs(fx_);
		}

		/// 获得矫正之后的图像，主要用于显示
		void undistortImage(const cv::Mat& raw, cv::Mat& rectified);

		/// 分别得到相机矩阵的4个参数
		inline double fx() const { return fx_; };
		inline double fy() const { return fy_; };
		inline double cx() const { return cx_; };
		inline double cy() const { return cy_; };

	private:
		double fx_, fy_;  //!< 相机两个方向的焦距值
		double cx_, cy_;  //!< 相机的中心点
		bool distortion_; //!< 是单纯的小孔相机模型，还是带有畸变？
		double d_[5];     //!< 畸变参数，参考 http://docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
		cv::Mat cvK_, cvD_;//!< 通过OpenCV表示的相机的相机矩阵和相机畸变参数
		cv::Mat undist_map1_, undist_map2_;//!<相机畸变在两个方向的map，提供给remap函数使用
	};
}

#endif // OPENMVO_MVO_PINHOLE_CAMERA_H_
