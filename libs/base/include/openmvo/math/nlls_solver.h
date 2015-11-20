#ifndef OPENMVO_MATH_NLLS_SOLVER_H_
#define OPENMVO_MATH_NLLS_SOLVER_H_

#include <stdint.h>
#include <stdio.h>
#include <iostream>
#include <math.h>
#include <Eigen/Core>
#include <Eigen/StdVector>

#include <openmvo/math/robust_cost.h>

namespace mvo {

	using namespace std;
	using namespace Eigen;

	/**
	* \brief 抽象类，用于求解非线性最小二乘问题(NLLS)
	*
	* 这个函数主要实现两个算法: Levenberg Marquardt 和 Gauss Newton
	*
	* 模板参数:
	* D  : 残差的维度
	* T  : 模型的类型, 例如. SE2, SE3
	*/
	template <int D, typename T>
	class NLLSSolver {

	public:
		typedef T ModelType;//!< 模型类型
		enum Method{ GaussNewton, LevenbergMarquardt };//!<模型方法枚举
		enum ScaleEstimatorType{ UnitScale, TDistScale, MADScale, NormalScale };
		enum WeightFunctionType{ UnitWeight, TDistWeight, TukeyWeight, HuberWeight };

	protected:
		Matrix<double, D, D>  H_;       //!< Hessian approximation
		Matrix<double, D, 1>  Jres_;    //!< Jacobian x Residual
		Matrix<double, D, 1>  x_;       //!< 更新量
		bool                  have_prior_;//!< 是否提供初始值
		ModelType prior_;
		Matrix<double, D, D>  I_prior_; //!< 前信息矩阵(协方差矩阵的逆)
		double                chi2_;
		double                rho_;
		Method                method_;

		/// 如果标识为线性问题，也必须执行计算成员变量 H_, Jres_
		virtual double computeResiduals(const ModelType& model,
			bool linearize_system,
			bool compute_weight_scale) = 0;

		/// 用于求解线性系统 H*x = Jres. 这个函数的目的是设置函数update的变量x_
		/// 返回true表示求解成功，false表示是奇异的 
		virtual int solve() = 0;

		virtual void update(const ModelType& old_model, ModelType& new_model) = 0;

		virtual void applyPrior(const ModelType& current_model) { }

		virtual void startIteration() { }

		virtual void finishIteration() { }

		virtual void finishTrial() { }

	public:

		/// 阻尼参数，如果mu > 0，则表示系数矩阵是正定的，这确保了方向一直向下，如果mu很大
		/// 则表明步伐很大，则在迭代较远距离的时候比较好。如果mu比较小，采用L-M，防止Jacobian
		/// 阵奇异或者接近奇异时试探步长过长。
		double                mu_init_, mu_;
		double                nu_init_, nu_;          //!< 失败之后，对因子mu进行增加
		size_t                n_iter_init_, n_iter_;  //!< 迭代次数
		size_t                n_trials_;              //!< 尝试次数
		size_t                n_trials_max_;          //!< 最大尝试次数
		size_t                n_meas_;                //!< 测量值的数目
		bool                  stop_;                  //!< 停止标识
		bool                  verbose_;               //!< 输出统计数据
		double                eps_;                   //!< 如果更新的时候norm的值小于eps，则停止
		size_t                iter_;                  //!< 当前迭代位置

		//用于鲁棒性的最小二乘
		bool                  use_weights_;// 是否使用权重
		float                 scale_;
		ScaleEstimatorPtr scale_estimator_;
		WeightFunctionPtr weight_function_;

		NLLSSolver() :
			have_prior_(false),
			method_(LevenbergMarquardt),
			mu_init_(0.01f),
			mu_(mu_init_),
			nu_init_(2.0),
			nu_(nu_init_),
			n_iter_init_(15),
			n_iter_(n_iter_init_),
			n_trials_(0),
			n_trials_max_(5),
			n_meas_(0),
			stop_(false),
			verbose_(true),
			eps_(0.0000000001),
			iter_(0),
			use_weights_(false),
			scale_(0.0),
			scale_estimator_(NULL),
			weight_function_(NULL)
		{ }

		virtual ~NLLSSolver() {}

		/// 调用GaussNewton或LevenbergMarquardt方法进行优化
		void optimize(ModelType& model);

		/// 调用Gauss Newton优化方法
		void optimizeGaussNewton(ModelType& model);

		/// 调用Levenberg Marquardt优化方法
		void optimizeLevenbergMarquardt(ModelType& model);

		/// 指定使用robust cost和scale estimator
		void setRobustCostFunction(
			ScaleEstimatorType scale_estimator,
			WeightFunctionType weight_function);

		/// 添加优化初始值
		void setPrior(
			const ModelType&  prior,
			const Matrix<double, D, D>&  Information);

		/// 重新设置所有参数后重新进行优化
		void reset();

		/// 得到平方误差
		const double& getChi2() const;

		///得到信息矩阵（协方差的逆矩阵）
		const Matrix<double, D, D>& getInformationMatrix() const;
	};

} // end namespace mvo

#include "nlls_solver_impl.hpp"

#endif // OPENMVO_MATH_NLLS_SOLVER_H_
