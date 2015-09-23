/*************************************************************************
 * 文件名： reprojector
 *
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 * 时间： 2015/9/17
 *
 * 说明： 
 *************************************************************************/
#ifndef OPENMVO_MVO_REPROJECTOR_H_
#define OPENMVO_MVO_REPROJECTOR_H_

#include <vector>
#include <Eigen/Core>
#include <openmvo/mvo/frame.h>
#include <openmvo/mvo/matcher.h>

namespace mvo
{
	class AbstractCamera;
	class Map;
	class Point3D;

	/// 对地图中的点投影到图像中，找到一个特征（角点），我们不对所有的点进行匹配，只对一个
	/// 单元格子中匹配一个点，这样，我们可以对所有的匹配特征进行均匀分布，而且不用投影所有的点
	/// 这样可以更好的节约时间。
	class Reprojector
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		Reprojector(AbstractCamera* cam, Map& map);

		~Reprojector();

		/// 从地图中点投影到图像中，找到有重叠视野的关键帧，仅随这些mappoints进行投影
		void reprojectMap(
			FramePtr frame,
			std::vector< std::pair<FramePtr, std::size_t> >& overlap_kfs);

	public:
			/// 重投影的配置参数
		struct Options {
			size_t max_n_kfs;   //!< 以当前帧为参考，进行重投影的关键帧的最大数目
			bool find_match_direct;
			Options()
				: max_n_kfs(10),
				find_match_direct(true)
			{}
		} options_;

		size_t n_matches_;
		size_t n_trials_;

		

	private:

		/// candidate是一个3D点和对应的投影的2D像素，我们在图像中找与该点匹配的特征
		struct Candidate {
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
				Point3D* pt;       //!< 3D点
			Vector2d px;     //!< 投影的2D像素点
			Candidate(Point3D* pt, Vector2d& px) : pt(pt), px(px) {}
		};
		typedef std::list<Candidate, aligned_allocator<Candidate> > Cell;//!< 用于存放3D点和对应的投影2D像素坐标列表
		typedef std::vector<Cell*> CandidateGrid;

		/// grid用于存储一系列候选匹配.对于每一个grid单元格努力寻找一个匹配
		struct Grid
		{
			CandidateGrid cells;//!< 用于存放3D点和对应的投影2D像素坐标
			std::vector<int> cell_order;//!< 单元格的顺序编号
			int cell_size;//!< 单应格的大小
			int grid_n_cols;//!< 图像划分单元格的列数
			int grid_n_rows;//!< 图像划分单元格的行数
		};

		Grid grid_;//!< 图像划分为网格
		Matcher matcher_;
		Map& map_;//!< 存储3D点和关键帧的地图信息

		void initializeGrid(AbstractCamera* cam);
		/// 重新设置grid，对每个grid中的候选点进行清空
		void resetGrid();

		bool reprojectCell(Cell& cell, FramePtr frame);
		bool reprojectPoint(FramePtr frame, Point3D* point);
	};
}

#endif // OPENMVO_MVO_REPROJECTOR_H_