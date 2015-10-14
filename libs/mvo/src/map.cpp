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
	MapPointCandidates::MapPointCandidates()
	{}

	MapPointCandidates::~MapPointCandidates()
	{
		reset();
	}

	void MapPointCandidates::newCandidatePoint(Point3D* point, double depth_sigma2)
	{
		point->type_ = Point3D::TYPE_CANDIDATE;
		std::unique_lock<std::mutex> lock(mut_);
		candidates_.push_back(PointCandidate(point, point->obs_.front()));
	}

	void MapPointCandidates::addCandidatePointToFrame(FramePtr frame)
	{
		std::unique_lock<std::mutex> lock(mut_);
		PointCandidateList::iterator it = candidates_.begin();
		while (it != candidates_.end())
		{
			if (it->first->obs_.front()->frame == frame.get())
			{
				// 插入特征到当前帧中
				it->first->type_ = Point3D::TYPE_UNKNOWN;
				it->first->n_failed_reproj_ = 0;
				it->second->frame->addFeature(it->second);
				it = candidates_.erase(it);
			}
			else
				++it;
		}
	}

	bool MapPointCandidates::deleteCandidatePoint(Point3D* point)
	{
		std::unique_lock<std::mutex> lock(mut_);
		for (auto it = candidates_.begin(), ite = candidates_.end(); it != ite; ++it)
		{
			if (it->first == point)
			{
				deleteCandidate(*it);
				candidates_.erase(it);
				return true;
			}
		}
		return false;
	}

	void MapPointCandidates::removeFrameCandidates(FramePtr frame)
	{
		std::unique_lock<std::mutex> lock(mut_);
		auto it = candidates_.begin();
		while (it != candidates_.end())
		{
			if (it->second->frame == frame.get())
			{
				deleteCandidate(*it);
				it = candidates_.erase(it);
			}
			else
				++it;
		}
	}

	void MapPointCandidates::reset()
	{
		std::unique_lock<std::mutex> lock(mut_);
		std::for_each(candidates_.begin(), candidates_.end(), [&](PointCandidate& c){
			delete c.first;
			delete c.second;
		});
		candidates_.clear();
	}

	void MapPointCandidates::deleteCandidate(PointCandidate& c)
	{
		// 另外一帧可能仍然执行候选点，因此我们不能立即删除它，对这个候选点添加删除标识
		delete c.second; c.second = NULL;
		c.first->type_ = Point3D::TYPE_DELETED;
		trash_points_.push_back(c.first);
	}

	void MapPointCandidates::emptyTrash()
	{
		std::for_each(trash_points_.begin(), trash_points_.end(), [&](Point3D*& p){
			delete p; p = NULL;
		});
		trash_points_.clear();
	}

	Map::Map() {}

	Map::~Map()
	{
		reset();
	}

	void Map::reset()
	{
		keyframes_.clear();//清空所有关键帧
		point_candidates_.reset();//清空候选点
		emptyTrash();//对存储删除掉的点的容器进行清空
	}

	/// 在地图中删除一个点，并且移除跟这个点相关的所有关键帧
	bool Map::safeDeleteFrame(FramePtr frame)
	{
		bool found = false;
		for (auto it = keyframes_.begin(), ite = keyframes_.end(); it != ite; ++it)
		{
			if (*it == frame)
			{
				std::for_each((*it)->fts_.begin(), (*it)->fts_.end(), [&](Feature* ftr){
					removePtFrameRef(it->get(), ftr);
				});
				keyframes_.erase(it);
				found = true;
				break;
			}
		}

		point_candidates_.removeFrameCandidates(frame);

		if (found)
			return true;

		return false;
	}

	void Map::removePtFrameRef(Frame* frame, Feature* ftr)
	{
		if (ftr->point == NULL)
			return; // 点可能已经被删除
		Point3D* pt = ftr->point;
		ftr->point = NULL;
		if (pt->obs_.size() <= 2)
		{
			// If the references list of mappoint has only size=2, delete mappoint
			safeDeletePoint(pt);
			return;
		}
		pt->deleteFrameRef(frame);  // Remove reference from map_point
		frame->removeKeyPoint(ftr); // Check if mp was keyMp in keyframe
	}

	void Map::safeDeletePoint(Point3D* pt)
	{
		// 在所有关键帧中删除跟目前点有映射的点
		std::for_each(pt->obs_.begin(), pt->obs_.end(), [&](Feature* ftr){
			ftr->point = NULL;
			ftr->frame->removeKeyPoint(ftr);
		});
		pt->obs_.clear();

		// Delete mappoint
		deletePoint(pt);
	}

	void Map::deletePoint(Point3D* pt)
	{
		pt->type_ = Point3D::TYPE_DELETED;
		trash_points_.push_back(pt);
	}

	void Map::addKeyframe(FramePtr new_keyframe)
	{
		keyframes_.push_back(new_keyframe);
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
					close_kfs.push_back(
						std::make_pair(
						kf, (frame->T_f_w_.translation() - kf->T_f_w_.translation()).norm()));
					break; // 这个关键帧跟目前帧有重叠的视野，则加入close_kfs
				}
			}
		}
	}

	bool frameComparator(std::pair<FramePtr, double>  lhs, std::pair<FramePtr, double>  rhs)
	{
		return (lhs.second < rhs.second);
	}

	FramePtr Map::getClosestKeyframe(const FramePtr& frame) const
	{
		std::list< std::pair<FramePtr, double> > close_kfs;
		getCloseKeyframes(frame, close_kfs);//得到有重叠视野的所有关键帧
		if (close_kfs.empty())
		{
			return nullptr;
		}

		// 对所有有重叠关键帧根据靠近程度进行排序，使用boost的写法
		close_kfs.sort(frameComparator);

		// 判断可能第一个关键帧就是当前帧
		if (close_kfs.front().first != frame)
			return close_kfs.front().first;
		close_kfs.pop_front();
		return close_kfs.front().first;
	}

	FramePtr Map::getFurthestKeyframe(const Vector3d& pos) const
	{
		FramePtr furthest_kf;
		double maxdist = 0.0;
		for (auto it = keyframes_.begin(), ite = keyframes_.end(); it != ite; ++it)
		{
			double dist = ((*it)->pos() - pos).norm();
			if (dist > maxdist) {
				maxdist = dist;
				furthest_kf = *it;
			}
		}
		return furthest_kf;
	}

	bool Map::getKeyframeById(const int id, FramePtr& frame) const
	{
		bool found = false;
		for (auto it = keyframes_.begin(), ite = keyframes_.end(); it != ite; ++it)
			if ((*it)->id_ == id) {
			found = true;
			frame = *it;
			break;
			}
		return found;
	}

	void Map::transform(const Matrix3d& R, const Vector3d& t, const double& s)
	{
		for (auto it = keyframes_.begin(), ite = keyframes_.end(); it != ite; ++it)
		{
			Vector3d pos = s*R*(*it)->pos() + t;
			Matrix3d rot = R*(*it)->T_f_w_.rotation_matrix().inverse();
			(*it)->T_f_w_ = Sophus::SE3(rot, pos).inverse();
			for (auto ftr = (*it)->fts_.begin(); ftr != (*it)->fts_.end(); ++ftr)
			{
				if ((*ftr)->point == NULL)
					continue;
				if ((*ftr)->point->last_published_ts_ == -1000)
					continue;
				(*ftr)->point->last_published_ts_ = -1000;
				(*ftr)->point->pos_ = s*R*(*ftr)->point->pos_ + t;
			}
		}
	}

	void Map::emptyTrash()
	{
		std::for_each(trash_points_.begin(), trash_points_.end(), [&](Point3D*& pt){
			delete pt;
			pt = NULL;
		});
		trash_points_.clear();
		point_candidates_.emptyTrash();
	}

}