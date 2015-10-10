/*************************************************************************
 * 文件名： config
 *
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 * 时间： 2015/8/14
 *
 * 说明： 全局配置文件 参考：rpg_svo(https://github.com/uzh-rpg/rpg_svo)
 *************************************************************************/
#ifndef OPENMVO_MVO_CONFIG_H_
#define OPENMVO_MVO_CONFIG_H_

#include <string>

namespace mvo {

	using std::string;

	/// 全局配置文件，采用单例模式
	class Config
	{
	public:
		static Config& getInstance();

		/// 跟踪文件的名称
		static string& traceName() { return getInstance().trace_name; }

		/// 跟踪文件保存到的目录
		static string& traceDir() { return getInstance().trace_dir; }

		/// 用于特征的金字塔层数
		static size_t& nPyrLevels() { return getInstance().n_pyr_levels; }

		/// 是否使用IMU获得相对旋转数据
		static bool& useImu() { return getInstance().use_imu; }

		/// 处理核心的关键帧数目，这个数目主要用于bundle adjustment.
		static size_t& coreNKfs() { return getInstance().core_n_kfs; }

		/// 初始地图的scale.
		static double& mapScale() { return getInstance().map_scale; }

		/// 特征一个单元格子的大小[px].
		static size_t& gridSize() { return getInstance().grid_size; }

		/// 初始化：开始两帧的最小差别
		static double& initMinDisparity() { return getInstance().init_min_disparity; }

		/// 初始值:跟踪特征的最小数目
		static size_t& initMinTracked() { return getInstance().init_min_tracked; }

		/// 初始值：RANSAC之后内点最小数目
		static size_t& initMinInliers() { return getInstance().init_min_inliers; }

		/// Lucas Kanade 跟踪的最大等级
		static size_t& kltMaxLevel() { return getInstance().klt_max_level; }

		/// Lucas Kanade 跟踪的最小等级
		static size_t& kltMinLevel() { return getInstance().klt_min_level; }

		/// 重投影的阈值 [px].
		static double& reprojThresh() { return getInstance().reproj_thresh; }

		/// 位姿优化之后重投影的阈值
		static double& poseOptimThresh() { return getInstance().poseoptim_thresh; }

		/// 局部bundle adjustment迭代次数
		static size_t& poseOptimNumIter() { return getInstance().poseoptim_num_iter; }

		/// 每次迭代进行优化的最大点的数目
		static size_t& structureOptimMaxPts() { return getInstance().structureoptim_max_pts; }

		/// 结构优化的最大迭代次数
		static size_t& structureOptimNumIter() { return getInstance().structureoptim_num_iter; }

		/// bundle adjustment之后的重投影阈值.
		static double& lobaThresh() { return getInstance().loba_thresh; }

		/// 局部bundle adjustment，Huber内核的阈值
		static double& lobaRobustHuberWidth() { return getInstance().loba_robust_huber_width; }

		/// 局部bundle adjustment迭代次数.
		static size_t& lobaNumIter() { return getInstance().loba_num_iter; }

		/// 两帧之间的最小距离，相对于地图的平均高度
		static double& kfSelectMinDist() { return getInstance().kfselect_mindist; }

		/// 选择最小的Harris角点score特征进行三角化
		static double& triangMinCornerScore() { return getInstance().triang_min_corner_score; }

		///重投影和三角化的时候计算亚像素，如果没有需要，则设置为0
		static size_t& subpixNIter() { return getInstance().subpix_n_iter; }

		/// 地图中限制的关键帧的数目，设置0为不限制，最小的关键帧数目为3
		static size_t& maxNKfs() { return getInstance().max_n_kfs; }

		/// 相机相对于IMU的延迟（单位毫秒）
		static double& imgImuDelay() { return getInstance().img_imu_delay; }

		/// 应该被跟踪的特征的最大数目
		static size_t& maxFts() { return getInstance().max_fts; }

		/// 如果跟踪特征的数量小于这个阈值，则说明这个跟踪质量不好
		static size_t& qualityMinFts() { return getInstance().quality_min_fts; }

		/// 对于一个帧，如果特征数在下降，则说明这个跟踪质量不好
		static int& qualityMaxFtsDrop() { return getInstance().quality_max_drop_fts; }

	private:
		Config();
		Config(Config const&);
		void operator=(Config const&);
		string trace_name;
		string trace_dir;
		size_t n_pyr_levels;
		bool use_imu;
		size_t core_n_kfs;
		double map_scale;
		size_t grid_size;
		double init_min_disparity;
		size_t init_min_tracked;
		size_t init_min_inliers;
		size_t klt_max_level;
		size_t klt_min_level;
		double reproj_thresh;
		double poseoptim_thresh;
		size_t poseoptim_num_iter;
		size_t structureoptim_max_pts;
		size_t structureoptim_num_iter;
		double loba_thresh;
		double loba_robust_huber_width;
		size_t loba_num_iter;
		double kfselect_mindist;
		double triang_min_corner_score;
		size_t triang_half_patch_size;
		size_t subpix_n_iter;
		size_t max_n_kfs;
		double img_imu_delay;
		size_t max_fts;
		size_t quality_min_fts;
		int quality_max_drop_fts;
	};

} // namespace mvo

#endif // OPENMVO_MVO_CONFIG_H_
