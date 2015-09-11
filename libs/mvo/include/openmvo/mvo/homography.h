/*************************************************************************
 * 文件名： homography
 *
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 * 时间： 2015/9/7
 *
 * 说明： 
 *************************************************************************/
#ifndef OPENMVO_MVO_HOMOGRAPHY_H_
#define OPENMVO_MVO_HOMOGRAPHY_H_

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <Eigen/SVD>
#include <sophus/se3.h>

namespace mvo {

	using namespace Eigen;
	using namespace std;

	/// 单应矩阵分解
	struct HomographyDecomposition
	{
		Vector3d t;
		Matrix3d R;
		double   d;
		Vector3d n;

		Sophus::SE3 T; //!< 第二幅图像到第一幅图像的相对旋转和平移
		int score;
	};

	class Homography
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

			/// 单应变换 两幅图像中对应的特征点，焦距及重投影误差
			Homography(const vector<Vector2d, aligned_allocator<Vector2d> >& fts1,
			const vector<Vector2d, aligned_allocator<Vector2d> >& fts2,
			double focal_length,
			double thresh_in_px);

		void calcFromPlaneParams(const Vector3d & normal,
			const Vector3d & point_on_plane);

		void calcFromMatches();

		size_t computeMatchesInliers();

		bool computeSE3fromMatches();

		bool decompose();

		void findBestDecomposition();


	public:

		double thresh_;//!< 重投影的阈值
		double focal_length_;//!< 给出计算单应过程中ransac的阈值，这边为焦距值？
		const std::vector<Vector2d, aligned_allocator<Vector2d> >& fts_c1_; //!< 在第一张图像上的特征
		const std::vector<Vector2d, aligned_allocator<Vector2d> >& fts_c2_; //!< 在第二张图像上的特征
		std::vector<bool> inliers_;
		Sophus::SE3 T_c2_from_c1_;             //!< 两幅图像的相对旋转和平移
		Matrix3d H_c2_from_c1_;                   //!< 单应矩阵
		//list<HomographyDecomposition> decompositions;
		HomographyDecomposition decompositions_[8];
		size_t decomp_size_;
	};
}

#endif // OPENMVO_MVO_HOMOGRAPHY_H_
