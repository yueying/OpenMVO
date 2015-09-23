/*************************************************************************
 * 文件名： matcher
 *
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 * 时间： 2015/9/17
 *
 * 说明： 
 *************************************************************************/
#ifndef OPENMVO_MVO_MATCHER_H_
#define OPENMVO_MVO_MATCHER_H_

#include <stdint.h>
#include <Eigen/Core>
#include <sophus/se3.h>
#include <opencv2/opencv.hpp>

namespace mvo{

	using namespace Eigen;
	using namespace Sophus;

	class Point3D;
	class Frame;
	class AbstractCamera;

	/**	面片匹配，主要通过投影匹配以及级线搜索
	 */
	class Matcher
	{
		static const int halfpatch_size_ = 4;
		static const int patch_size_ = 8;
	public:
		struct Options
		{
			int align_max_iter;         //!< 高斯牛顿法的迭代次数，用于对特征进行对齐
			Options() :
				align_max_iter(10)
			{}
		} options_;

		//生成内联默认构造函数
		Matcher() = default;
		~Matcher() = default;

		/// 应用亚像素直接进行匹配
		/// 这边要注意!这个函数假设当前特征点px_cur跟最后估计的结果只相差2-3像素
		bool findMatchDirect(
			const Point3D& pt,
			const Frame& frame,
			Vector2d& px_cur);
	private:
		void getWarpMatrixAffine(
			const AbstractCamera& cam_ref,
			const AbstractCamera& cam_cur,
			const Vector2d& px_ref,
			const Vector3d& f_ref,
			const double depth_ref,
			const SE3& T_cur_ref,
			const int level_ref,
			Matrix2d& A_cur_ref);

		int getBestSearchLevel(const Matrix2d& A_cur_ref, const int max_level);

		void warpAffine(
			const Matrix2d& A_cur_ref,
			const cv::Mat& img_ref,
			const Vector2d& px_ref,
			const int level_ref,
			const int level_cur,
			const int halfpatch_size,
			uint8_t* patch);
		/// 从带边框的面片创建面片
		void createPatchFromPatchWithBorder();

	public:
		Matrix2d A_cur_ref_;          //!< 仿射变换矩阵
		int search_level_;            //!< 用于计算特征在其它图像中所在的金字塔的等级

	private:
#ifdef _MSC_VER
		__declspec (align(16)) uint8_t patch_[patch_size_*patch_size_];//!< 面片
		__declspec (align(16)) uint8_t patch_with_border_[(patch_size_ + 2)*(patch_size_ + 2)];//!<面片宽高+2
#else
		uint8_t __attribute__((aligned(16))) patch_[patch_size_*patch_size_];
		uint8_t __attribute__((aligned(16))) patch_with_border_[(patch_size_ + 2)*(patch_size_ + 2)];
#endif
	};
}
#endif // OPENMVO_MVO_MATCHER_H_