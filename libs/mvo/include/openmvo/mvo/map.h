/*************************************************************************
 * 文件名： map
 *
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 * 时间： 2015/8/16
 *
 * 说明： 地图定义，主要存储3d点和关键帧，参考rpg_svo(https://github.com/uzh-rpg/rpg_svo)
 *************************************************************************/
#ifndef OPENMVO_MVO_MAP_H_
#define OPENMVO_MVO_MAP_H_

#include <list>
#include <openmvo/utils/noncopyable.h>
#include <openmvo/mvo/frame.h>

namespace mvo
{
	class Point3D;

	class Map : public Noncopyable
	{
	public:
		Map();
		~Map();

		/// 得到跟目前帧有重叠视野的所有关键帧
		void getCloseKeyframes(const FramePtr& frame, std::list< std::pair<FramePtr, double> >& close_kfs) const;
		/// 往地图中添加一个新的关键帧
		void addKeyframe(FramePtr new_keyframe);
	public:
		std::list< FramePtr > keyframes_;          //!< 地图中存储的所有关键帧
		std::list< Point3D* >   points_;         //!< 存放3D点
	};
}


#endif // OPENMVO_MVO_MAP_H_