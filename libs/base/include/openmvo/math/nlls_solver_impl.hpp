#ifndef OPENMVO_MATH_NLLS_SOLVER_IMPL_H_
#define OPENMVO_MATH_NLLS_SOLVER_IMPL_H_

#include <stdexcept>
#include <openmvo/utils/math_utils.h>
#include <openmvo/math/robust_cost.h>

template <int D, typename T>
void mvo::NLLSSolver<D, T>::optimize(ModelType& model)
{
	if (method_ == GaussNewton)
		optimizeGaussNewton(model);
	else if (method_ == LevenbergMarquardt)
		optimizeLevenbergMarquardt(model);
}

template <int D, typename T>
void mvo::NLLSSolver<D, T>::optimizeGaussNewton(ModelType& model)
{
	// 计算权重scale
	if (use_weights_)
		computeResiduals(model, false, true);

	// 保存旧的模型用于在更新失败的时候回滚
	ModelType old_model(model);

	// 执行迭代估计
	for (iter_ = 0; iter_ < n_iter_; ++iter_)
	{
		rho_ = 0;
		startIteration();

		H_.setZero();
		Jres_.setZero();

		// 计算初始残差
		n_meas_ = 0;
		double new_chi2 = computeResiduals(model, true, false);

		// 添加先前估计量
		if (have_prior_)
			applyPrior(model);

		// 计算线性问题
		if (!solve())
		{
			// 矩阵接近奇异，停止计算
			std::cout << "Matrix is close to singular! Stop Optimizing." << std::endl;
			std::cout << "H = " << H_ << std::endl;
			std::cout << "Jres = " << Jres_ << std::endl;
			stop_ = true;
		}

		// 检测误差有没有增加，如果增加则回到上一个模型
		if ((iter_ > 0 && new_chi2 > chi2_) || stop_)
		{
			if (verbose_)
			{
				std::cout << "It. " << iter_
					<< "\t Failure"
					<< "\t new_chi2 = " << new_chi2
					<< "\t Error increased. Stop optimizing."
					<< std::endl;
			}
			model = old_model; // 回滚到上一个模型
			break;
		}

		// 更新模型
		ModelType new_model;
		update(model, new_model);
		old_model = model;
		model = new_model;

		chi2_ = new_chi2;

		if (verbose_)
		{
			std::cout << "It. " << iter_
				<< "\t Success"
				<< "\t new_chi2 = " << new_chi2
				<< "\t n_meas = " << n_meas_
				<< "\t x_norm = " << mvo::norm_max(x_)
				<< std::endl;
		}

		finishIteration();

		//收敛的时候，停止，即更新步伐太小
		if (mvo::norm_max(x_) <= eps_)
			break;
	}
}

template <int D, typename T>
void mvo::NLLSSolver<D, T>::optimizeLevenbergMarquardt(ModelType& model)
{
	// 计算权重scale
	if (use_weights_)
		computeResiduals(model, false, true);

	// 计算初始残差
	chi2_ = computeResiduals(model, true, false);

	if (verbose_)
		cout << "init chi2 = " << chi2_
		<< "\t n_meas = " << n_meas_
		<< endl;

	// TODO: 计算初始lambda
	// Hartley and Zisserman: "A typical init value of lambda is 10^-3 times the
	// average of the diagonal elements of J'J"

	// 计算初始lambda
	if (mu_ < 0)
	{
		double H_max_diag = 0;
		double tau = 1e-4;
		for (size_t j = 0; j < D; ++j)
			H_max_diag = max(H_max_diag, fabs(H_(j, j)));
		mu_ = tau*H_max_diag;
	}

	// 执行迭代估计
	for (iter_ = 0; iter_ < n_iter_; ++iter_)
	{
		rho_ = 0;
		startIteration();

		// 尝试计算和更新，如果失败，则增加 mu
		n_trials_ = 0;
		do
		{
			// 初始变量
			ModelType new_model;
			double new_chi2 = -1;
			H_.setZero();
			//H_ = mu_ * Matrix<double,D,D>::Identity(D,D);
			Jres_.setZero();

			// 计算初始误差
			n_meas_ = 0;
			computeResiduals(model, true, false);

			// 添加阻尼项
			H_ += (H_.diagonal()*mu_).asDiagonal();

			// 添加先验估计
			if (have_prior_)
				applyPrior(model);

			// 计算线性系统问题
			if (solve())
			{
				// 更新模型
				update(model, new_model);

				// 根据新模型计算残差，与前残差进行对比
				n_meas_ = 0;
				new_chi2 = computeResiduals(new_model, false, false);
				rho_ = chi2_ - new_chi2;
			}
			else
			{
				// 矩阵是奇异的，不能被计算
				cout << "Matrix is close to singular!" << endl;
				cout << "H = " << H_ << endl;
				cout << "Jres = " << Jres_ << endl;
				rho_ = -1;
			}

			if (rho_ > 0)
			{
				// update decrased the error -> success
				model = new_model;
				chi2_ = new_chi2;
				stop_ = mvo::norm_max(x_) <= eps_;
				mu_ *= max(1. / 3., min(1. - pow(2 * rho_ - 1, 3), 2. / 3.));
				nu_ = 2.;
				if (verbose_)
				{
					cout << "It. " << iter_
						<< "\t Trial " << n_trials_
						<< "\t Success"
						<< "\t n_meas = " << n_meas_
						<< "\t new_chi2 = " << new_chi2
						<< "\t mu = " << mu_
						<< "\t nu = " << nu_
						<< endl;
				}
			}
			else
			{
				// update increased the error -> fail
				mu_ *= nu_;
				nu_ *= 2.;
				++n_trials_;
				if (n_trials_ >= n_trials_max_)
					stop_ = true;

				if (verbose_)
				{
					cout << "It. " << iter_
						<< "\t Trial " << n_trials_
						<< "\t Failure"
						<< "\t n_meas = " << n_meas_
						<< "\t new_chi2 = " << new_chi2
						<< "\t mu = " << mu_
						<< "\t nu = " << nu_
						<< endl;
				}
			}

			finishTrial();

		} while (!(rho_ > 0 || stop_));
		if (stop_)
			break;

		finishIteration();
	}
}


template <int D, typename T>
void mvo::NLLSSolver<D, T>::setRobustCostFunction(
	ScaleEstimatorType scale_estimator,
	WeightFunctionType weight_function)
{
	switch (scale_estimator)
	{
	case TDistScale:
		if (verbose_)
			printf("Using TDistribution Scale Estimator\n");
		scale_estimator_.reset(new TDistributionScaleEstimator());
		use_weights_ = true;
		break;
	case MADScale:
		if (verbose_)
			printf("Using MAD Scale Estimator\n");
		scale_estimator_.reset(new MADScaleEstimator());
		use_weights_ = true;
		break;
	case NormalScale:
		if (verbose_)
			printf("Using Normal Scale Estimator\n");
		scale_estimator_.reset(new NormalDistributionScaleEstimator());
		use_weights_ = true;
		break;
	default:
		if (verbose_)
			printf("Using Unit Scale Estimator\n");
		scale_estimator_.reset(new UnitScaleEstimator());
		use_weights_ = false;
	}

	switch (weight_function)
	{
	case TDistWeight:
		if (verbose_)
			printf("Using TDistribution Weight Function\n");
		weight_function_.reset(new TDistributionWeightFunction());
		break;
	case TukeyWeight:
		if (verbose_)
			printf("Using Tukey Weight Function\n");
		weight_function_.reset(new TukeyWeightFunction());
		break;
	case HuberWeight:
		if (verbose_)
			printf("Using Huber Weight Function\n");
		weight_function_.reset(new HuberWeightFunction());
		break;
	default:
		if (verbose_)
			printf("Using Unit Weight Function\n");
		weight_function_.reset(new UnitWeightFunction());
	}
}

template <int D, typename T>
void mvo::NLLSSolver<D, T>::setPrior(
	const T&  prior,
	const Eigen::Matrix<double, D, D>&  Information)
{
	have_prior_ = true;
	prior_ = prior;
	I_prior_ = Information;
}

template <int D, typename T>
void mvo::NLLSSolver<D, T>::reset()
{
	have_prior_ = false;
	chi2_ = 1e10;
	mu_ = mu_init_;
	nu_ = nu_init_;
	n_meas_ = 0;
	n_iter_ = n_iter_init_;
	iter_ = 0;
	stop_ = false;
}

template <int D, typename T>
inline const double& mvo::NLLSSolver<D, T>::getChi2() const
{
	return chi2_;
}

template <int D, typename T>
inline const mvo::Matrix<double, D, D>& mvo::NLLSSolver<D, T>::getInformationMatrix() const
{
	return H_;
}


#endif // OPENMVO_MATH_NLLS_SOLVER_IMPL_H_
