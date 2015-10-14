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
#include <mutex>
#include <openmvo/utils/noncopyable.h>
#include <openmvo/mvo/frame.h>

namespace mvo
{
	class Point3D;

	/// 容器用于对那些3D还没有分配到两个关键帧中
	class MapPointCandidates
	{
	public:
		typedef std::pair<Point3D*, Feature*> PointCandidate;//!< 候选点，pair由点和点对应的特征构成
		typedef std::list<PointCandidate> PointCandidateList;//!< 候选点list

		/// 通过depth-filter线程对候选点进行填充，通过这个互斥量控制对候选点的读取问题
		std::mutex mut_;

		/// 对种子点进行融合处理得到候选点
		/// 知道下一个关键帧，这些点用于重投影和姿态优化
		PointCandidateList candidates_;
		std::list< Point3D* > trash_points_;

		MapPointCandidates();
		~MapPointCandidates();

		/// 添加一个候选点
		void newCandidatePoint(Point3D* point, double depth_sigma2);

		/// 添加特征到帧，并从列表中删除候选
		void addCandidatePointToFrame(FramePtr frame);

		/// 在候选列表中删除一个候选点
		bool deleteCandidatePoint(Point3D* point);

		///删除属于这个帧的所有候选点
		void removeFrameCandidates(FramePtr frame);

		/// 对候选点的list重新设置，移除并删除所有的点
		void reset();

		void deleteCandidate(PointCandidate& c);

		void emptyTrash();
	};

	class Map : public Noncopyable
	{
	public:
		Map();
		~Map();
		/// 重新设置地图，删除所有的关键帧，重新设置帧和点的计数器
		void reset();

		/// 在地图中删除一个点，并且移除跟这个点相关的所有关键帧
		void safeDeletePoint(Point3D* pt);

		/// 移除点到垃圾队列
		void deletePoint(Point3D* pt);

		/// 移除帧到垃圾队列
		bool safeDeleteFrame(FramePtr frame);

		/// 移除点与帧之间的关联
		void removePtFrameRef(Frame* frame, Feature* ftr);

		/// 往地图中添加一个新的关键帧
		void addKeyframe(FramePtr new_keyframe);

		/// 得到跟目前帧有重叠视野的所有关键帧
		void getCloseKeyframes(const FramePtr& frame, std::list< std::pair<FramePtr, double> >& close_kfs) const;

		/// 返回与目前帧在空间上最靠近，有重叠视野的关键帧
		FramePtr getClosestKeyframe(const FramePtr& frame) const;

		/// 返回离目前位姿最远的关键帧，用于控制关键帧的数量
		FramePtr getFurthestKeyframe(const Vector3d& pos) const;

		/// 根据id得到关键帧
		bool getKeyframeById(const int id, FramePtr& frame) const;

		/// 对整幅地图进行变换，根据旋转R，平移t，和缩放s
		void transform(const Matrix3d& R, const Vector3d& t, const double& s);

		void emptyTrash();

		/// 返回最近插入地图中的关键帧
		inline FramePtr lastKeyframe() { return keyframes_.back(); }

		/// 返回在地图中关键帧的数目
		inline size_t size() const { return keyframes_.size(); }
	public:
		std::list< FramePtr > keyframes_;          //!< 地图中存储的所有关键帧
		//std::list< Point3D* >   points_;         //!< 存放3D点
		std::list< Point3D* > trash_points_;         //!< 存放删除掉的点
		MapPointCandidates point_candidates_;
	};
}


#endif // OPENMVO_MVO_MAP_H_