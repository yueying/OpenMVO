/*************************************************************************
 * 文件名： reprojector
 *
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 * 时间： 2015/9/17
 *
 * 说明： 
 *************************************************************************/
#include <openmvo/mvo/reprojector.h>

#include <openmvo/mvo/config.h>
#include <openmvo/mvo/map.h>
#include <openmvo/mvo/feature.h>

namespace mvo
{
	Reprojector::Reprojector(AbstractCamera* cam, Map& map) :
		map_(map)
	{
		initializeGrid(cam);
	}

	Reprojector::~Reprojector()
	{
		std::for_each(grid_.cells.begin(), grid_.cells.end(), [&](Cell* c){ delete c; });
	}

	void Reprojector::initializeGrid(AbstractCamera* cam)
	{
		grid_.cell_size = Config::gridSize();
		grid_.grid_n_cols = ceil(static_cast<double>(cam->width()) / grid_.cell_size);
		grid_.grid_n_rows = ceil(static_cast<double>(cam->height()) / grid_.cell_size);
		grid_.cells.resize(grid_.grid_n_cols*grid_.grid_n_rows);
		// 初始化单元格，在析构中删除
		std::for_each(grid_.cells.begin(), grid_.cells.end(), [&](Cell*& c){ c = new Cell; });
		grid_.cell_order.resize(grid_.cells.size());
		for (size_t i = 0; i < grid_.cells.size(); ++i)
			grid_.cell_order[i] = i;
		std::random_shuffle(grid_.cell_order.begin(), grid_.cell_order.end()); // 随机排列，一种策略
	}

	/// 重新设置grid，对每个grid中的候选点进行清空
	void Reprojector::resetGrid()
	{
		n_matches_ = 0;
		n_trials_ = 0;
		std::for_each(grid_.cells.begin(), grid_.cells.end(), [&](Cell* c){ c->clear(); });
	}

	bool compareDistance(std::pair<FramePtr, double>  &x, std::pair<FramePtr, double> &y)
	{
		if (x.second < y.second)
			return true;
		else
			return false;
	}

	void Reprojector::reprojectMap(
		FramePtr frame,
		std::vector< std::pair<FramePtr, std::size_t> >& overlap_kfs)
	{
		resetGrid();

		// 选出与目前帧有重叠视野的关键帧
		std::list< std::pair<FramePtr, double> > close_kfs;
		map_.getCloseKeyframes(frame, close_kfs);

		// 对靠近的关键帧根据靠近程度进行排序
		close_kfs.sort(compareDistance);

		// 对有有重叠部分的N个关键帧对应的mappoints进行重投影，我们只存储格子中特征点减少的
		size_t n = 0;
		overlap_kfs.reserve(options_.max_n_kfs);
		// 对最近的N个关键帧进行迭代，找到有重叠视野
		for (auto it_frame = close_kfs.begin(), ite_frame = close_kfs.end();
			it_frame != ite_frame && n < options_.max_n_kfs; ++it_frame, ++n)
		{
			FramePtr ref_frame = it_frame->first;
			overlap_kfs.push_back(std::pair<FramePtr, size_t>(ref_frame, 0));

			// 对这个参考帧观察到的点投影到当前帧中
			for (auto it_ftr = ref_frame->fts_.begin(), ite_ftr = ref_frame->fts_.end();
				it_ftr != ite_ftr; ++it_ftr)
			{
				// 检测这个特征是否有分配的mappoint
				if ((*it_ftr)->point == NULL)
					continue;

				// 确保我们只投影一次，不同帧上的特征会对应同一个3D点
				if ((*it_ftr)->point->last_projected_kf_id_ == frame->id_)
					continue;
				(*it_ftr)->point->last_projected_kf_id_ = frame->id_;
				if (reprojectPoint(frame, (*it_ftr)->point))
					overlap_kfs.back().second++;//相同观察点的数目
			}
		}

		// 现在遍历所有的单元格，选择一个点进行匹配
		for (size_t i = 0; i<grid_.cells.size(); ++i)
		{
			// 更倾向于寻找质量好的点
			if (reprojectCell(*grid_.cells.at(grid_.cell_order[i]), frame))
				++n_matches_;
			if (n_matches_ >(size_t) Config::maxFts())
				break;
		}

	}

	/// 重投影3D点，将投影在帧内的像素点，作为候选特征
	bool Reprojector::reprojectPoint(FramePtr frame, Point3D* point)
	{
		Vector2d px(frame->w2c(point->pos_));//将世界坐标的点转到像素
		if (frame->cam_->isInFrame(px.cast<int>(), 8)) // 判断像素是否在帧中，提供8px的边界
		{
			// 特征在哪个格子里
			const int k = static_cast<int>(px[1] / grid_.cell_size)*grid_.grid_n_cols
				+ static_cast<int>(px[0] / grid_.cell_size);
			grid_.cells.at(k)->push_back(Candidate(point, px));// 在格子中添加候选特征
			return true;
		}
		return false;
	}

	bool Reprojector::reprojectCell(Cell& cell, FramePtr frame)
	{
		Cell::iterator it = cell.begin();
		while (it != cell.end())
		{
			++n_trials_;//重投影单元格的次数

			bool found_match = true;
			if (options_.find_match_direct)
				found_match = matcher_.findMatchDirect(*it->pt, *frame, it->px);
			if (!found_match)
			{
				it->pt->n_failed_reproj_++;
				//TODO:对Map进行处理
				it = cell.erase(it);
				continue;
			}
			it->pt->n_succeeded_reproj_++;
			
			Feature* new_feature = new Feature(frame.get(), it->px, matcher_.search_level_);
			frame->addFeature(new_feature);

			// 这边我们添加点到特征的引用
			new_feature->point = it->pt;

			// 如果关键帧已经选择，也重投影其他点，则我们不需要检测这个点
			it = cell.erase(it);

			// 每个单元格中保证有一个点
			return true;
		}
		return false;
	}


}