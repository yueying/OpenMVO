/*************************************************************************
 * 文件名： pinhole_camera
 *
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 * 时间： 2015/8/1
 *
 * 说明： 参考rpg_svo(https://github.com/uzh-rpg/rpg_svo)
 *************************************************************************/
#include "openmvo/mvo/pinhole_camera.h"

namespace mvo
{
	// 考虑畸变参数k1,k2,p1,p2,k3
	PinholeCamera::PinholeCamera(double width, double height,
		double fx, double fy,
		double cx, double cy,
		double k1, double k2, double p1, double p2, double k3) :
		AbstractCamera(width, height),
		fx_(fx), fy_(fy), cx_(cx), cy_(cy),
		distortion_(fabs(k1) > 0.0000001),
		undist_map1_(height_, width_, CV_16SC2),
		undist_map2_(height_, width_, CV_16SC2)
	{
		// 径向畸变参数
		d_[0] = k1; d_[1] = k2; d_[2] = p1; d_[3] = p2; d_[4] = k3;
		cvK_ = (cv::Mat_<float>(3, 3) << fx_, 0.0, cx_, 0.0, fy_, cy_, 0.0, 0.0, 1.0);
		cvD_ = (cv::Mat_<float>(1, 5) << d_[0], d_[1], d_[2], d_[3], d_[4]);
		// 根据相机矩阵和畸变参数构建map
		cv::initUndistortRectifyMap(cvK_, cvD_, cv::Mat_<double>::eye(3, 3), cvK_,
			cv::Size(width_, height_), CV_16SC2, undist_map1_, undist_map2_);
	}

	PinholeCamera::~PinholeCamera(){}

	Vector3d PinholeCamera::cam2world(const double& u, const double& v) const
	{
		Vector3d xyz;
		if (!distortion_)
		{
			xyz[0] = (u - cx_) / fx_;
			xyz[1] = (v - cy_) / fy_;
			xyz[2] = 1.0;
		}
		else
		{
			cv::Point2f uv(u, v), px;
			const cv::Mat src_pt(1, 1, CV_32FC2, &uv.x);
			cv::Mat dst_pt(1, 1, CV_32FC2, &px.x);
			cv::undistortPoints(src_pt, dst_pt, cvK_, cvD_);
			xyz[0] = px.x;
			xyz[1] = px.y;
			xyz[2] = 1.0;
		}
		return xyz.normalized();
	}

	Vector3d PinholeCamera::cam2world(const Vector2d& uv) const
	{
		return cam2world(uv[0], uv[1]);
	}

	/// 摄像机坐标系下的点转图像像素坐标
	Vector2d PinholeCamera::world2cam(const Vector3d& xyz) const
	{
		Vector2d  uv = xyz.head<2>() / xyz[2];
		return world2cam(uv);
	}

	/// 图像平面像素的世界坐标转像素坐标
	Vector2d PinholeCamera::world2cam(const Vector2d& uv) const
	{
		Vector2d px;
		if (!distortion_)
		{
			px[0] = fx_*uv[0] + cx_;
			px[1] = fy_*uv[1] + cy_;
		}
		else
		{
			double x, y, r2, r4, r6, a1, a2, a3, cdist, xd, yd;
			x = uv[0];
			y = uv[1];
			r2 = x*x + y*y;
			r4 = r2*r2;
			r6 = r4*r2;
			a1 = 2 * x*y;
			a2 = r2 + 2 * x*x;
			a3 = r2 + 2 * y*y;
			cdist = 1 + d_[0] * r2 + d_[1] * r4 + d_[4] * r6;//1+k1r2+k2r4+k3r6
			xd = x*cdist + d_[2] * a1 + d_[3] * a2;
			yd = y*cdist + d_[2] * a3 + d_[3] * a1;
			px[0] = xd*fx_ + cx_;
			px[1] = yd*fy_ + cy_;
		}
		return px;
	}

	void PinholeCamera::undistortImage(const cv::Mat& raw, cv::Mat& rectified)
	{
		if (distortion_)
			cv::remap(raw, rectified, undist_map1_, undist_map2_, CV_INTER_LINEAR);
		else
			rectified = raw.clone();
	}
}
