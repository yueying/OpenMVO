/*************************************************************************
 * 文件名： math_utils
 *
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 * 时间： 2015/8/7
 *
 * 说明： 
 *************************************************************************/
#ifndef OPENMVO_UTILS_MATH_UTILS_H_
#define OPENMVO_UTILS_MATH_UTILS_H_
#include <vector>
#include <algorithm>
#include <Eigen/Dense>

namespace mvo
{
	using namespace Eigen;

	Vector3d triangulateFeatureNonLin(
		const Matrix3d& R,
		const Vector3d& t,
		const Vector3d& feature1,
		const Vector3d& feature2);

	double reprojError(
		const Vector3d& f1,
		const Vector3d& f2,
		double focal_length);

	double computeInliers(
		const std::vector<Vector3d>& features1,
		const std::vector<Vector3d>& features2,
		const Matrix3d& R,
		const Vector3d& t,
		const double reproj_thresh,
		double focal_length,
		std::vector<Vector3d>& xyz_vec,
		std::vector<int>& inliers,
		std::vector<int>& outliers);

	/// 投影，摄像机坐标系下坐标转像素世界坐标
	inline Vector2d project2d(const Vector3d& v)
	{
		return v.head<2>() / v[2];
	}

	/// 反投影，将像素的世界坐标转相对于的摄像机坐标
	inline Vector3d unproject2d(const Vector2d& v)
	{
		return Vector3d(v[0], v[1], 1.0);
	}
	///给出反对称矩阵
	inline Matrix3d sqew(const Vector3d& v)
	{
		Matrix3d v_sqew;
		v_sqew << 0, -v[2], v[1],
			v[2], 0, -v[0],
			-v[1], v[0], 0;
		return v_sqew;
	}

	inline double norm_max(const Eigen::VectorXd & v)
	{
		double max = -1;
		for (int i = 0; i < v.size(); i++)
		{
			double abs = fabs(v[i]);
			if (abs > max){
				max = abs;
			}
		}
		return max;
	}

	double sampsonusError(
		const Vector2d &v2Dash,
		const Matrix3d& m3Essential,
		const Vector2d& v2);
	///给出中值
	template<class T>
	T getMedian(std::vector<T>& data_vec)
	{
		assert(!data_vec.empty());
		typename std::vector<T>::iterator it = data_vec.begin() + floor(data_vec.size() / 2);
		nth_element(data_vec.begin(), it, data_vec.end());//对前n个数进行排序
		return *it;
	}
}

#endif // OPENMVO_UTILS_MATH_UTILS_H_