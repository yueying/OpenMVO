/*************************************************************************
 * 文件名： depth_filter
 *
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 * 时间： 2015/8/29
 *
 * 说明： 
 *************************************************************************/
#ifndef OPENMVO_MVO_DEPTH_FILTER_H_
#define OPENMVO_MVO_DEPTH_FILTER_H_
#include <mutex>
#include <thread>
#include <condition_variable>
#include <queue>
#include <Eigen/Core>
#include <openmvo/mvo/feature.h>
#include <openmvo/mvo/abstract_detector.h>
#include <openmvo/mvo/matcher.h>

namespace mvo
{
	///一个seed就是一个像素的概率性深度估计
	struct Seed
	{
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		static int batch_counter;    //!< 用于设置种子点对应帧的数目
		static int seed_counter;     //!< 用于设置种子点的唯一id
		int batch_id;                //!< batch_id是种子点被创建所对应的关键帧的id
		int id;                      //!< 种子ID,仅用来可视化显示
		Feature* ftr;                //!< 在关键帧上的特征，这些特征的深度需要被计算
		float a;                     //!< Beta分布的参数a: a越高，则内点的概率就越大
		float b;                     //!< Beta分布的参数b: b越高，则外点的概率就越大
		float mu;                    //!< 正态分布的均值
		float z_range;               //!< 可能深度的最大范围
		float sigma2;                //!< 正态分布的方差
		Seed(Feature* ftr, float depth_mean, float depth_min);
	};

	class DepthFilter
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		typedef std::unique_lock<std::mutex> lock_t;//独占锁
		typedef std::function<void(Point3D*, double)> callback_t;//绑定函数指针

		DepthFilter(
			DetectorPtr feature_detector,
			callback_t seed_converged_cb);

		virtual ~DepthFilter();

		/// 启动种子点更新线程
		void startThread();

		/// 停止正在运行的线程
		void stopThread();

		/// 添加帧到待处理的队列
		void addFrame(FramePtr frame);

		/// 添加新的关键帧到队列
		void addKeyframe(FramePtr frame, double depth_mean, double depth_min);

		void removeKeyframe(FramePtr frame);

		void reset();

		/// 贝叶斯框架下更新种子点，x表示测量，tau2表示测量的不确定性
		static void updateSeed(
			const float x,
			const float tau2,
			Seed* seed);

		/// 计算测量的不确定性
		static double computeTau(
			const SE3& T_ref_cur,
			const Vector3d& f,
			const double z,
			const double px_error_angle);
	protected:
		/// 从一个帧中初始化新的种子点列表
		void initializeSeeds(FramePtr frame);

		/// 根据一个新的测量帧更新所有的种子点
		virtual void updateSeeds(FramePtr frame);

		/// 当来了一个关键帧，则帧的队列进行清空
		void clearFrameQueue();

		/// 一个线程用于连续不断的更新种子点
		void updateSeedsLoop();
	public:
		/// Depth-filter配置参数
		struct Options
		{
			bool verbose;                               //!< 是否显示输出
			int max_n_kfs;                              //!< 我们维护种子点的关键帧的最大数目
			double sigma_i_sq;                          //!< 图像噪声
			double seed_convergence_sigma2_thresh;      //!< threshold on depth uncertainty for convergence.
			Options()
				: verbose(false),
				max_n_kfs(3),
				sigma_i_sq(5e-4),
				seed_convergence_sigma2_thresh(200.0)
			{}
		} options_;
	protected:
		DetectorPtr feature_detector_;        //!< 特征检测
		callback_t seed_converged_cb_;        //!< 绑定的回调函数
		bool seeds_updating_halt_;            //!< 值设为true，表明有新种子点生成，需要中断，以确保种子点列表更新
		std::list<Seed, aligned_allocator<Seed> > seeds_;//!< 种子点列表
		bool new_keyframe_set_;               //!< 是否有一个新的关键帧待处理
		double new_keyframe_min_depth_;       //!< 在新关键帧中的最小深度，用于新种子点的范围
		double new_keyframe_mean_depth_;      //!< 在新关键帧中的最大深度，用于新种子点的范围
		std::mutex seeds_mut_;                //!< 互斥量，用于对种子点的更新控制	
		std::thread *thread_;                 //!< 目前处理线程
		std::mutex frame_queue_mut_;
		std::condition_variable frame_queue_cond_;
		std::queue<FramePtr> frame_queue_;
		FramePtr new_keyframe_;               //!< 下一个关键帧用于提取新的种子点
		bool is_runing; //!<表示线程是否仍在执行
		Matcher matcher_;
	};
}

#endif // OPENMVO_MVO_DEPTH_FILTER_H_