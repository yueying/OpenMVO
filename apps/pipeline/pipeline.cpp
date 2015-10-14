#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <opencv2/opencv.hpp>

#include <sophus/se3.h>

#include <openmvo/mvo/config.h>
#include <openmvo/mvo/frame_handler_mono.h>
#include <openmvo/mvo/map.h>
#include <openmvo/mvo/frame.h>
#include <openmvo/utils/math_utils.h>
#include <openmvo/mvo/abstract_camera.h>
#include <openmvo/mvo/pinhole_camera.h>
#include <opencv2/core/eigen.hpp>

namespace mvo {
	using namespace Eigen;
	class BenchmarkNode
	{
		AbstractCamera* cam_;
		FrameHandlerMono* vo_;

	public:
		BenchmarkNode();
		~BenchmarkNode();
		void runFromFolder();
	};

	BenchmarkNode::BenchmarkNode()
	{
		cam_ = new PinholeCamera(752, 480, 315.5, 315.5, 376.0, 240.0);
		vo_ = new FrameHandlerMono(cam_);
		vo_->start();
	}

	BenchmarkNode::~BenchmarkNode()
	{
		delete vo_;
		delete cam_;
	}

	void BenchmarkNode::runFromFolder()
	{
		std::ofstream out("piptime.txt");
		for (int img_id = 3; img_id < 206; ++img_id)
		{
			// 导入图像
			std::stringstream ss;
			ss << "d:/dataset/sin2_tex2_h1_v8_d/img/frame_"
				<< std::setw(6) << std::setfill('0') << img_id << "_0.png";
			if (img_id == 2)
				std::cout << "reading image " << ss.str() << std::endl;
			cv::Mat img(cv::imread(ss.str().c_str(), 0));
			assert(!img.empty());

			// 处理帧
			vo_->addImage(img, 0.01*img_id);

			// 显示跟踪质量
			if (vo_->lastFrame() != NULL)
			{
				//out << "Frame-Id: " << vo_->lastFrame()->id_ << " \t"
				out << vo_->lastNumObservations() << " ";
				//	<< "Proc. Time: " << vo_->lastProcessingTime() * 1000 << "ms \n";
				
				Sophus::SE3 camera_pose = vo_->lastFrame()->T_f_w_.inverse();
				cv::Mat postion_vector = (cv::Mat_<double>(3, 1) << camera_pose.translation()[0],
					camera_pose.translation()[1],
					camera_pose.translation()[2]);
				cv::Mat rotMat(3, 3, CV_64FC1);
				Eigen::Matrix3d eigen_rotate = camera_pose.rotation_matrix();
				cv::eigen2cv(eigen_rotate, rotMat);
				cv::Mat rotate1 = (cv::Mat_<double>(3, 3) << 0, 1, 0,
					0, 0, -1,
					-1, 0, 0);
				cv::Mat rotate2 = (cv::Mat_<double>(3, 3) << 0, 0, -1,
					1, 0, 0,
					0, -1, 0);
				postion_vector = rotate2*(postion_vector);
				rotMat = rotate2*(rotMat)*rotate1;

				cv::Mat junk1(3, 3, CV_64FC1), junk2(3, 3, CV_64FC1);
				cv::Vec3d euler = cv::RQDecomp3x3(rotMat, junk1, junk2);

				out << postion_vector.at<double>(0, 0) << " "
					<< postion_vector.at<double>(1, 0) << " "
					<< postion_vector.at<double>(2, 0) << " "
					<< euler(0) << " "
					<< euler(1) << " "
					<< euler(2) << " "
				<< std::endl;

			}
		}
		out.close();
	}

} // namespace mvo

int main(int argc, char** argv)
{
	{
		mvo::BenchmarkNode benchmark;
		benchmark.runFromFolder();
	}
	printf("BenchmarkNode finished.\n");
	getchar();
	return 0;
}

