/*************************************************************************
 * 文件名： homography
 *
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 * 时间： 2015/8/7
 *
 * 说明： 
 *************************************************************************/
#include "openmvo/mvo/homography.h"
#include <opencv2/opencv.hpp>
#include "openmvo/utils/math_utils.h"

namespace mvo
{
	Homography::Homography(const vector<Vector2d, aligned_allocator<Vector2d> >& fts1,
		const vector<Vector2d, aligned_allocator<Vector2d> >& fts2,
		double focal_length,
		double thresh_in_px) :
		thresh_(thresh_in_px),
		focal_length_(focal_length),
		fts_c1_(fts1),
		fts_c2_(fts2)
	{
	}

	void Homography::calcFromPlaneParams(const Vector3d& n_c1, const Vector3d& xyz_c1)
	{
		double d = n_c1.dot(xyz_c1); // normal distance from plane to KF
		H_c2_from_c1_ = T_c2_from_c1_.rotation_matrix() + (T_c2_from_c1_.translation()*n_c1.transpose()) / d;
	}

	void Homography::calcFromMatches()
	{
		std::vector<cv::Point2f> src_pts(fts_c1_.size()), dst_pts(fts_c1_.size());
		for (size_t i = 0; i < fts_c1_.size(); ++i)
		{
			src_pts[i] = cv::Point2f(fts_c1_[i][0], fts_c1_[i][1]);
			dst_pts[i] = cv::Point2f(fts_c2_[i][0], fts_c2_[i][1]);
		}

		// TODO: 替换该函数，移除对opencv的依赖
		cv::Mat cvH = cv::findHomography(src_pts, dst_pts, CV_RANSAC, 2. / focal_length_);
		H_c2_from_c1_(0, 0) = cvH.at<double>(0, 0);
		H_c2_from_c1_(0, 1) = cvH.at<double>(0, 1);
		H_c2_from_c1_(0, 2) = cvH.at<double>(0, 2);
		H_c2_from_c1_(1, 0) = cvH.at<double>(1, 0);
		H_c2_from_c1_(1, 1) = cvH.at<double>(1, 1);
		H_c2_from_c1_(1, 2) = cvH.at<double>(1, 2);
		H_c2_from_c1_(2, 0) = cvH.at<double>(2, 0);
		H_c2_from_c1_(2, 1) = cvH.at<double>(2, 1);
		H_c2_from_c1_(2, 2) = cvH.at<double>(2, 2);
	}

	bool Homography::computeSE3fromMatches()
	{
		calcFromMatches();//计算单应
		bool res = decompose();// 将单应矩阵进行分解
		if (!res)
			return false;
		computeMatchesInliers();// 计算匹配的内点数
		findBestDecomposition();// 找出最好的分解
		T_c2_from_c1_ = decompositions_[0].T;
		return true;
	}

	bool Homography::decompose()
	{
		decomp_size_ = 0;
		JacobiSVD<MatrixXd> svd(H_c2_from_c1_, ComputeThinU | ComputeThinV);

		Vector3d singular_values = svd.singularValues();
		// 获得3个特征量
		double d1 = fabs(singular_values[0]); 
		double d2 = fabs(singular_values[1]); 
		double d3 = fabs(singular_values[2]);

		Matrix3d U = svd.matrixU();
		Matrix3d V = svd.matrixV();                    // VT^T

		double s = U.determinant() * V.determinant();

		double dPrime_PM = d2;

		int nCase;
		if (d1 != d2 && d2 != d3)
			nCase = 1;
		else if (d1 == d2 && d2 == d3)
			nCase = 3;
		else
			nCase = 2;

		if (nCase != 1)
		{
			printf("FATAL Homography Initialization: This motion case is not implemented or is degenerate. Try again. ");
			return false;
		}

		double x1_PM;
		double x2;
		double x3_PM;

		// 在情况1下(d1 != d3)
		{ // Eq. 12
			x1_PM = sqrt((d1*d1 - d2*d2) / (d1*d1 - d3*d3));
			x2 = 0;
			x3_PM = sqrt((d2*d2 - d3*d3) / (d1*d1 - d3*d3));
		};

		double e1[4] = { 1.0, -1.0, 1.0, -1.0 };
		double e3[4] = { 1.0, 1.0, -1.0, -1.0 };

		Vector3d np;
		HomographyDecomposition decomp;

		// Case 1, d' > 0:
		decomp.d = s * dPrime_PM;
		for (size_t signs = 0; signs < 4; signs++)
		{
			// Eq 13
			decomp.R = Matrix3d::Identity();
			double dSinTheta = (d1 - d3) * x1_PM * x3_PM * e1[signs] * e3[signs] / d2;
			double dCosTheta = (d1 * x3_PM * x3_PM + d3 * x1_PM * x1_PM) / d2;
			decomp.R(0, 0) = dCosTheta;
			decomp.R(0, 2) = -dSinTheta;
			decomp.R(2, 0) = dSinTheta;
			decomp.R(2, 2) = dCosTheta;

			// Eq 14
			decomp.t[0] = (d1 - d3) * x1_PM * e1[signs];
			decomp.t[1] = 0.0;
			decomp.t[2] = (d1 - d3) * -x3_PM * e3[signs];

			np[0] = x1_PM * e1[signs];
			np[1] = x2;
			np[2] = x3_PM * e3[signs];
			decomp.n = V * np;

			decompositions_[decomp_size_++] = decomp;
		}

		// Case 1, d' < 0:
		decomp.d = s * -dPrime_PM;
		for (size_t signs = 0; signs < 4; signs++)
		{
			// Eq 15
			decomp.R = -1 * Matrix3d::Identity();
			double dSinPhi = (d1 + d3) * x1_PM * x3_PM * e1[signs] * e3[signs] / d2;
			double dCosPhi = (d3 * x1_PM * x1_PM - d1 * x3_PM * x3_PM) / d2;
			decomp.R(0, 0) = dCosPhi;
			decomp.R(0, 2) = dSinPhi;
			decomp.R(2, 0) = dSinPhi;
			decomp.R(2, 2) = -dCosPhi;

			// Eq 16
			decomp.t[0] = (d1 + d3) * x1_PM * e1[signs];
			decomp.t[1] = 0.0;
			decomp.t[2] = (d1 + d3) * x3_PM * e3[signs];

			np[0] = x1_PM * e1[signs];
			np[1] = x2;
			np[2] = x3_PM * e3[signs];
			decomp.n = V * np;

			decompositions_[decomp_size_++] = decomp;
		}

		// 保存分解之后的旋转和平移，Eq 8
		for (unsigned int i = 0; i < decomp_size_; i++)
		{
			Matrix3d R = s * U * decompositions_[i].R * V.transpose();
			Vector3d t = U * decompositions_[i].t;
			decompositions_[i].T = Sophus::SE3(R, t);
		}
		return true;
	}

	/// 返回匹配的内点数
	size_t Homography::computeMatchesInliers()
	{
		inliers_.clear(); inliers_.resize(fts_c1_.size());
		size_t n_inliers = 0;
		for (size_t i = 0; i < fts_c1_.size(); i++)
		{
			Vector2d projected = project2d(H_c2_from_c1_ * unproject2d(fts_c1_[i]));
			Vector2d e = fts_c2_[i] - projected;
			double e_px = focal_length_ * e.norm();//化为像素距离
			inliers_[i] = (e_px < thresh_);//也就是残差在阈值范围内
			n_inliers += inliers_[i];
		}
		return n_inliers;
	}

	bool operator<(const HomographyDecomposition & lhs, const HomographyDecomposition & rhs)
	{
		return lhs.score < rhs.score;
	}

	void Homography::findBestDecomposition()
	{
		assert(decomp_size_ == 8);// 验证是否计算出了8组解
		for (size_t i = 0; i < decomp_size_; i++)
		{
			HomographyDecomposition &decom = decompositions_[i];
			int positive = 0;
			for (size_t m = 0; m < fts_c1_.size(); m++)
			{
				if (!inliers_[m])
					continue;
				const Vector2d& v2 = fts_c1_[m];
				double visibility_test = (H_c2_from_c1_(2, 0) * v2[0] + H_c2_from_c1_(2, 1) * v2[1] + H_c2_from_c1_(2, 2)) / decom.d;
				if (visibility_test > 0.0)
					positive++;
			}
			decom.score = -positive;
		}

		sort(std::begin(decompositions_), std::begin(decompositions_) + decomp_size_);

		decomp_size_ = 4;

		for (size_t i = 0; i < decomp_size_; i++)
		{
			HomographyDecomposition &decom = decompositions_[i];
			int positive = 0;
			for (size_t m = 0; m < fts_c1_.size(); m++)
			{
				if (!inliers_[m])
					continue;
				Vector3d v3 = unproject2d(fts_c1_[m]);
				double visibility_test = v3.dot(decom.n) / decom.d;
				if (visibility_test > 0.0)
					positive++;
			};
			decom.score = -positive;
		}

		sort(std::begin(decompositions_), std::begin(decompositions_) + decomp_size_);
		decomp_size_ = 2;

		// 最后还有两组解，看比例
		double ratio = (double)decompositions_[1].score / (double)decompositions_[0].score;

		if (ratio < 0.9) { // 则确定
			decomp_size_ = 1;
		}
		else  // two-way ambiguity. Resolve by sampsonus score of all points.
		{
			double error_squared_limit = thresh_ * thresh_ * 4;
			double sampsonus_scores[2];
			for (size_t i = 0; i < 2; i++)
			{
				Sophus::SE3 T = decompositions_[i].T;
				Matrix3d essential = T.rotation_matrix()*sqew(T.translation());//得到本质矩阵
				double sum_error = 0;
				for (size_t m = 0; m < fts_c1_.size(); m++)
				{
					double d = sampsonusError(fts_c1_[m], essential, fts_c2_[m]);
					if (d > error_squared_limit)
						d = error_squared_limit;
					sum_error += d;
				}
				sampsonus_scores[i] = sum_error;
			}

			if (sampsonus_scores[0] <= sampsonus_scores[1]) {
				decomp_size_ = 1;
			}
			else {
				decompositions_[0] = decompositions_[1];
				decomp_size_ = 1;
			}

		}
	}

}