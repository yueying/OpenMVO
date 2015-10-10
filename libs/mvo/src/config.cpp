/*************************************************************************
 * 文件名： config
 *
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 * 时间： 2015/8/14
 *
 * 说明： 所有参数设置参考rpg_svo(https://github.com/uzh-rpg/rpg_svo)
 *************************************************************************/
#include <openmvo/mvo/config.h>

namespace mvo {

	Config::Config() :
		trace_name("mvo"),
		trace_dir("./tmp"),
		n_pyr_levels(3),
		use_imu(false),
		core_n_kfs(3),
		map_scale(1.0),
		grid_size(25),
		init_min_disparity(50.0),
		init_min_tracked(50),
		init_min_inliers(40),
		klt_max_level(4),
		klt_min_level(1),
		reproj_thresh(2.0),
		poseoptim_thresh(2.0),
		poseoptim_num_iter(10),
		structureoptim_max_pts(20),
		structureoptim_num_iter(5),
		loba_thresh(2.0),
		loba_robust_huber_width(1.0),
		loba_num_iter(0),
		kfselect_mindist(0.12),
		triang_min_corner_score(20.0),
		triang_half_patch_size(4),
		subpix_n_iter(10),
		max_n_kfs(0),
		img_imu_delay(0.0),
		max_fts(150),
		quality_min_fts(50),
		quality_max_drop_fts(40)
	{}

	Config& Config::getInstance()
	{
		static Config instance; // 第一次使用时初始化，并保证销毁
		return instance;
	}

} // namespace mvo

