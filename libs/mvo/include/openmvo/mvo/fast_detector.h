/*************************************************************************
 * 文件名： fast_detector
 *
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 * 时间： 2015/8/3
 *
 * 说明： fast特征检测参考rpg_svo(https://github.com/uzh-rpg/rpg_svo)
 *************************************************************************/
#include "openmvo/mvo/abstract_detector.h"
#include "openmvo/mvo/frame.h"

namespace mvo
{
	class FastDetector : public AbstractDetector
	{
	public:
		FastDetector(
			const int img_width,
			const int img_height,
			const int cell_size,
			const int n_pyr_levels);

		virtual ~FastDetector() {}

		virtual void detect(
			Frame* frame,
			const ImgPyr& img_pyr,
			const double detection_threshold,
			Features& fts);

	private:
		float shiTomasiScore(const cv::Mat& img, int u, int v);
	};
}
