/*************************************************************************
 * 文件名： map
 *
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 * 时间： 2015/8/17
 *
 * 说明： 地图定义，主要存储3d点和关键帧，参考rpg_svo(https://github.com/uzh-rpg/rpg_svo)
 *************************************************************************/
#include <openmvo/mvo/map.h>
#include <openmvo/mvo/feature.h>

namespace mvo
{
	Map::Map() {}

	Map::~Map()
	{
		keyframes_.clear();//清空所有关键帧
		std::for_each(points_.begin(), points_.end(), [&](Point3D*& pt){
			delete pt;
			pt = NULL;
		});
		points_.clear();
	}

	/// 获得有重叠视野的关键帧，就根据当前帧的特征对应的3D点，是否能投影到其它关键帧中，可以则
	/// 这个关键帧与当前帧有相关视野
	void Map::getCloseKeyframes(
		const FramePtr& frame,
		std::list< std::pair<FramePtr, double> >& close_kfs) const
	{
		for (auto kf : keyframes_)
		{
			// 检测当前帧与关键帧之间是否有重叠的视野，通过关键点(特征)来进行计算
			for (auto keypoint : kf->key_pts_)
			{
				if (keypoint == nullptr)
					continue;

				if (frame->isVisible(keypoint->point->pos_))// 判断目前帧的特征所对应的3d点是否在关键帧中可见
				{
					close_kfs.push_back(std::make_pair(
						kf, (frame->T_f_w_.translation() - kf->T_f_w_.translation()).norm()));
					break; // 这个关键帧跟目前帧有重叠的视野，则加入close_kfs
				}
			}
		}
	}

	void Map::addKeyframe(FramePtr new_keyframe)
	{
		keyframes_.push_back(new_keyframe);
	}

}