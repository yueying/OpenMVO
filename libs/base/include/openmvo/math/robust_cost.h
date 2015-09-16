#ifndef OPENMVO_MATH_ROBUST_COST_H_
#define OPENMVO_MATH_ROBUST_COST_H_

#include <vector>
#include <stdlib.h>
#include <memory>

namespace mvo {

	// 尺度估计接口
	class ScaleEstimator
	{
	public:
		virtual ~ScaleEstimator() {};
		virtual float compute(std::vector<float>& errors) const = 0;
	};
	typedef std::shared_ptr<ScaleEstimator> ScaleEstimatorPtr;

	class UnitScaleEstimator : public ScaleEstimator
	{
	public:
		UnitScaleEstimator() {}
		virtual ~UnitScaleEstimator() {}
		virtual float compute(std::vector<float>& errors) const { return 1.0f; };
	};

	// 通过拟合给定自由度的数据的t分布来估计scale
	class TDistributionScaleEstimator : public ScaleEstimator
	{
	public:
		TDistributionScaleEstimator(const float dof = DEFAULT_DOF);
		virtual ~TDistributionScaleEstimator() {};
		virtual float compute(std::vector<float>& errors) const;

		static const float DEFAULT_DOF;
		static const float INITIAL_SIGMA;
	protected:
		float dof_;
		float initial_sigma_;
	};

	// 通过计算绝对中值偏差来进行估计scale
	class MADScaleEstimator : public ScaleEstimator
	{
	public:
		MADScaleEstimator() {};
		virtual ~MADScaleEstimator() {};
		virtual float compute(std::vector<float>& errors) const;

	private:
		static const float NORMALIZER;;
	};

	// 通过计算标准变差来估计scale
	class NormalDistributionScaleEstimator : public ScaleEstimator
	{
	public:
		NormalDistributionScaleEstimator() {};
		virtual ~NormalDistributionScaleEstimator() {};
		virtual float compute(std::vector<float>& errors) const;
	private:
	};

	/**
	 * 权重函数接口，权重函数是一个对称鲁棒函数p(sqrt(t))的一阶导数.
	 * 误差被归一化到单位方差
	 *
	 * See:
	 *   "Lucas-Kanade 20 Years On: A Unifying Framework: Part 2" - Page 23, Equation (54)
	 */
	class WeightFunction
	{
	public:
		virtual ~WeightFunction() {};
		virtual float value(const float& x) const = 0;
		virtual void configure(const float& param) {};
	};
	typedef std::shared_ptr<WeightFunction> WeightFunctionPtr;

	class UnitWeightFunction : public WeightFunction
	{
	public:
		UnitWeightFunction() {};
		virtual ~UnitWeightFunction() {};
		virtual float value(const float& x) const { return 1.0f; };
	};

	/**
	 * Tukey's hard re-descending function.
	 *
	 * See:
	 *   http://en.wikipedia.org/wiki/Redescending_M-estimator
	 */
	class TukeyWeightFunction : public WeightFunction
	{
	public:
		TukeyWeightFunction(const float b = DEFAULT_B);
		virtual ~TukeyWeightFunction() {};
		virtual float value(const float& x) const;
		virtual void configure(const float& param);

		static const float DEFAULT_B;
	private:
		float b_square;
	};

	class TDistributionWeightFunction : public WeightFunction
	{
	public:
		TDistributionWeightFunction(const float dof = DEFAULT_DOF);
		virtual ~TDistributionWeightFunction() {};
		virtual float value(const float& x) const;
		virtual void configure(const float& param);

		static const float DEFAULT_DOF;
	private:
		float dof_;
		float normalizer_;
	};

	class HuberWeightFunction : public WeightFunction
	{
	public:
		HuberWeightFunction(const float k = DEFAULT_K);
		virtual ~HuberWeightFunction() {};
		virtual float value(const float& x) const;
		virtual void configure(const float& param);

		static const float DEFAULT_K;
	private:
		float k;
	};

} // namespace mvo

#endif // OPENMVO_MATH_ROBUST_COST_H_
