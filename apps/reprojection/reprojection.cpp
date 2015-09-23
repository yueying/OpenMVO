/*************************************************************************
* 文件名： image_align
*
* 作者： 冯兵
* 邮件： fengbing123@gmail.com
* 时间： 2015/8/15
*
* 说明：
*************************************************************************/
#include <string>
#include <opencv2/opencv.hpp>
#include <openmvo/utils/cmd_line.h>
#include <openmvo/utils/timer.h>
#include <openmvo/mvo/pinhole_camera.h>
#include <openmvo/mvo/fast_detector.h>
#include <openmvo/mvo/frame.h>
#include <openmvo/mvo/initialization.h>
#include <openmvo/mvo/sparse_img_align.h>
#include <openmvo/mvo/map.h>
#include <openmvo/mvo/reprojector.h>

using namespace mvo;
using namespace std;

int main(int argc, char *argv[])
{
	CmdLine cmd;
	std::string first_frame_name;
	std::string second_frame_name;
	std::string third_frame_name;

	cmd.add(make_option('f', first_frame_name, "firstname"));
	cmd.add(make_option('s', second_frame_name, "secondname"));
	cmd.add(make_option('t', third_frame_name, "thirdname"));
	try {
		if (argc == 1) throw std::string("Invalid command line parameter.");
		cmd.process(argc, argv);
	}
	catch (const std::string& s) {
		std::cerr << "Feature detector \nUsage: " << argv[0] << "\n"
			<< "[-f|--firstname name]\n"
			<< "[-s|--secondname name]\n"
			<< "[-t|--thirdname name]\n"
			<< std::endl;

		std::cerr << s << std::endl;
		return EXIT_FAILURE;
	}
	cv::Mat first_img(cv::imread(first_frame_name, 0));
	cv::Mat second_img(cv::imread(second_frame_name, 0));
	cv::Mat third_img(cv::imread(third_frame_name, 0));
	assert(first_img.type() == CV_8UC1 && !first_img.empty());
	assert(second_img.type() == CV_8UC1 && !second_img.empty());
	assert(third_img.type() == CV_8UC1 && !third_img.empty());

	AbstractCamera* cam = new PinholeCamera(752, 480, 315.5, 315.5, 376.0, 240.0);

	FramePtr fisrt_frame(new Frame(cam, first_img, 0.0));
	FramePtr second_frame(new Frame(cam, second_img, 1.0));
	FramePtr third_frame(new Frame(cam, third_img, 1.0));

	Initialization init;
	init.addFirstFrame(fisrt_frame);
	init.addSecondFrame(second_frame);

	SparseImgAlign img_align(4, 1,
		30, SparseImgAlign::GaussNewton, false, false);
	size_t img_align_n_tracked = img_align.run(second_frame, third_frame);
	std::cout << "Img Align:\t Tracked = " << img_align_n_tracked << std::endl;
	mvo::Map map;
	fisrt_frame->setKeyframe();
	second_frame->setKeyframe();
	map.addKeyframe(fisrt_frame);
	map.addKeyframe(second_frame);
	Reprojector reprojector(cam,map);
	std::vector< std::pair<FramePtr, size_t> > overlap_kfs;
	reprojector.reprojectMap(third_frame, overlap_kfs);
	const size_t repr_n_new_references = reprojector.n_matches_;
	const size_t repr_n_mps = reprojector.n_trials_;
	std::cout << "Reprojection:\t Points = " << repr_n_mps << "\t \t Matches = " << repr_n_new_references<<std::endl;
	getchar();
	return 0;
}