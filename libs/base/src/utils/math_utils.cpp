/*************************************************************************
 * 文件名： math_utils
 *
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 * 时间： 2015/8/7
 *
 * 说明： 
 *************************************************************************/
#include "openmvo/utils/math_utils.h"

namespace mvo
{
	using namespace Eigen;

	/// 进行三角定位
	Vector3d triangulateFeatureNonLin(const Matrix3d& R, const Vector3d& t,
		const Vector3d& feature1, const Vector3d& feature2)
	{
		Vector3d f2 = R * feature2;
		Vector2d b;
		b[0] = t.dot(feature1);
		b[1] = t.dot(f2);
		Matrix2d A;
		A(0, 0) = feature1.dot(feature1);
		A(1, 0) = feature1.dot(f2);
		A(0, 1) = -A(1, 0);
		A(1, 1) = -f2.dot(f2);
		Vector2d lambda = A.inverse() * b;
		Vector3d xm = lambda[0] * feature1;
		Vector3d xn = t + lambda[1] * f2;
		return (xm + xn) / 2;
	}

	double reprojError(const Vector3d& f1,
		const Vector3d& f2,
		double focal_length)
	{
		Vector2d e = project2d(f1) - project2d(f2);
		return focal_length * e.norm();
	}

	double computeInliers(const std::vector<Vector3d>& features1, // c1
		const std::vector<Vector3d>& features2, // c2
		const Matrix3d& R,                 // R_c1_c2
		const Vector3d& t,                 // c1_t
		const double reproj_thresh,
		double focal_length,
		std::vector<Vector3d>& xyz_vec,         // in frame c1
		std::vector<int>& inliers,
		std::vector<int>& outliers)
	{
		inliers.clear(); inliers.reserve(features1.size());
		outliers.clear(); outliers.reserve(features1.size());
		xyz_vec.clear(); xyz_vec.reserve(features1.size());
		double tot_error = 0;
		//三角化所有特征，然后计算投影误差和内点
		for (size_t j = 0; j<features1.size(); ++j)
		{
			xyz_vec.push_back(triangulateFeatureNonLin(R, t, features1[j], features2[j]));
			double e1 = reprojError(features1[j], xyz_vec.back(), focal_length);
			double e2 = reprojError(features2[j], R.transpose()*(xyz_vec.back() - t), focal_length);
			if (e1 > reproj_thresh || e2 > reproj_thresh)
				outliers.push_back(j);
			else
			{
				inliers.push_back(j);
				tot_error += e1 + e2;
			}
		}
		return tot_error;
	}

	double sampsonusError(const Vector2d &v2Dash, const Matrix3d& essential, const Vector2d& v2)
	{
		Vector3d v3Dash = unproject2d(v2Dash);
		Vector3d v3 = unproject2d(v2);

		double error = v3Dash.transpose() * essential * v3;

		Vector3d fv3 = essential * v3;
		Vector3d fTv3Dash = essential.transpose() * v3Dash;

		Vector2d fv3Slice = fv3.head<2>();
		Vector2d fTv3DashSlice = fTv3Dash.head<2>();

		return (error * error / (fv3Slice.dot(fv3Slice) + fTv3DashSlice.dot(fTv3DashSlice)));
	}
}