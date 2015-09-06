/*************************************************************************
 * 文件名： feature
 *
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 * 时间： 2015/8/2
 *
 * 说明： 给出特征定义，参考rpg_svo(https://github.com/uzh-rpg/rpg_svo)
 *************************************************************************/
#include <vector>
#include <Eigen/Core>
#include "openmvo/mvo/frame.h"

namespace mvo
{
	using namespace Eigen;

	/// 用于角点检测的临时容器，特征从这边开始初始化
	struct Corner
	{
		int x;        //!< 在图像中角点的x坐标
		int y;        //!< 在图像中角点的y坐标
		int level;    //!< 角点所在金字塔的等级
		float score;  //!< shi-tomasi 角点最小特征值
		float angle;  //!< 梯度特征，对应梯度值
		Corner(int x, int y, float score, int level, float angle) :
			x(x), y(y), level(level), score(score), angle(angle)
		{}
	};
	typedef std::vector<Corner> Corners;

	/**	特征考虑多尺度
	 */
	struct Feature
	{
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		///特征类型,目前先只考虑角点，后续考虑其它特征的时候，再进行添加
		enum FeatureType {
			CORNER//角点
		};
		FeatureType type;     //!< 特征类型，角点
		Frame* frame;         //!< 指针指向特征被检测到所对应的帧
		Vector2d px;          //!< 特征在金字塔等级为0时的像素坐标
		int level;            //!< 特征被提取时，图像金字塔的等级

		Feature(Frame* _frame, const Vector2d& _px, int _level) :
			type(CORNER),
			frame(_frame),
			px(_px),
			level(_level)
		{}

		~Feature(){}
	};
}
