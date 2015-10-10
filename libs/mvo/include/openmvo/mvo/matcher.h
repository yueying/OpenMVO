/*************************************************************************
 * 文件名： matcher
 *
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 * 时间： 2015/8/17
 *
 * 说明： 参考rpg_svo(https://github.com/uzh-rpg/rpg_svo)
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
	class Feature;
	namespace patch_score {
		template<int HALF_PATCH_SIZE> class ZMSSD;
	}

	/**	面片匹配，主要通过投影匹配以及级线搜索
	 */
	class Matcher
	{
		static const int halfpatch_size_ = 4;
		static const int patch_size_ = 8;
		typedef patch_score::ZMSSD<halfpatch_size_> PatchScore;
	public:
		struct Options
		{
			int align_max_iter;         //!< 高斯牛顿法的迭代次数，用于对特征进行对齐
			size_t max_epi_search_steps;//!< 沿着极线方向走的最大的步伐数目
			bool subpix_refinement;     //!< 每一次极线搜索之后，进行feature align
			Options() :
				align_max_iter(10),
				max_epi_search_steps(1000),
				subpix_refinement(true)
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

		/// 沿着级线搜索特征
		bool findEpipolarMatchDirect(
			const Frame& ref_frame,
			const Frame& cur_frame,
			const Feature& ref_ftr,
			const double d_estimate,
			const double d_min,
			const double d_max,
			double& depth);
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

		bool depthFromTriangulation(
			const SE3& T_search_ref,
			const Vector3d& f_ref,
			const Vector3d& f_cur,
			double& depth);

	public:
		Matrix2d A_cur_ref_;          //!< 仿射变换矩阵
		int search_level_;            //!< 用于计算特征在其它图像中所在的金字塔的等级
		Vector2d px_cur_;
	private:
#ifdef _MSC_VER
		__declspec (align(16)) uint8_t patch_[patch_size_*patch_size_];//!< 面片
		__declspec (align(16)) uint8_t patch_with_border_[(patch_size_ + 2)*(patch_size_ + 2)];//!<面片宽高+2
#else
		uint8_t __attribute__((aligned(16))) patch_[patch_size_*patch_size_];
		uint8_t __attribute__((aligned(16))) patch_with_border_[(patch_size_ + 2)*(patch_size_ + 2)];
#endif

		Vector2d epi_dir_;
		double epi_length_;           //!< 极线段的长度，主要用于极线搜索
		
	};
}
#endif // OPENMVO_MVO_MATCHER_H_