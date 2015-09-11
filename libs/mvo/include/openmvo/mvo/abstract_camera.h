/*************************************************************************
 * 文件名： abstract_camera
 *
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 * 时间： 2015/8/1
 *
 * 说明： 参考rpg_svo(https://github.com/uzh-rpg/rpg_svo)
 *************************************************************************/
#ifndef OPENMVO_MVO_ABSTRACT_CAMERA_H_
#define OPENMVO_MVO_ABSTRACT_CAMERA_H_

#include "openmvo/mvo/link_pragmas.h"
#include <Eigen/Core>

namespace mvo
{
	using namespace Eigen;

	/**	定义抽象相机类
	 */
	class MVO_IMPEXP AbstractCamera
	{
	public:
		AbstractCamera() {}; // 此构造函数供全景相机模型使用
		AbstractCamera(int width, int height) : width_(width), height_(height) {};

		virtual ~AbstractCamera() {};

		/// 图像像素坐标转摄像机坐标系下的点
		virtual Vector3d cam2world(const double& x, const double& y) const = 0;

		/// 图像像素坐标转摄像机坐标系下的点
		virtual Vector3d cam2world(const Vector2d& px) const = 0;

		/// 摄像机坐标系下的点转图像像素坐标
		virtual Vector2d world2cam(const Vector3d& xyz_c) const = 0;

		/// 图像平面像素的世界坐标转像素坐标
		virtual Vector2d world2cam(const Vector2d& uv) const = 0;

		/// 返回x方向的焦距值
		virtual double getFocalLength() const = 0;

		/// 返回相机分辨率的宽度
		inline int width() const { return width_; }
		/// 返回相机分辨率的高度
		inline int height() const { return height_; }

		///  判断像素是否在图像帧中
		inline bool isInFrame(const Vector2i & obs, int boundary = 0) const
		{
			if (obs[0] >= boundary && obs[0] < width() - boundary
				&& obs[1] >= boundary && obs[1] < height() - boundary)
				return true;
			return false;
		}

		/// 判断像素是否在图像帧中，考虑图像尺度
		inline bool isInFrame(const Vector2i &obs, int boundary, int level) const
		{
			if (obs[0] >= boundary && obs[0] < width() / (1 << level) - boundary
				&& obs[1] >= boundary && obs[1] < height() / (1 << level) - boundary)
				return true;
			return false;
		}

	protected:
		int width_; //!< 相机分辨率的宽度
		int height_; //!< 相机分辨率的高度
	};
}

#endif // OPENMVO_MVO_ABSTRACT_CAMERA_H_

