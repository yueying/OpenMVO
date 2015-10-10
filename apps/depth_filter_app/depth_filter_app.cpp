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
#include <openmvo/mvo/pose_optimizer.h>
#include <openmvo/mvo/structure_optimizer.h>
#include <openmvo/mvo/feature.h>
#include <openmvo/mvo/point3d.h>
#include <openmvo/mvo/depth_filter.h>
#include <openmvo/utils/file_reader.h>
#include <openmvo/mvo/blender_utils.h>
#include <openmvo/mvo/config.h>

using namespace mvo;
using namespace std;

struct ConvergedSeed 
{
	int x_, y_;
	double depth_, error_;
	ConvergedSeed(int x, int y, double depth, double error) :
		x_(x), y_(y), depth_(depth), error_(error)
	{}
};

class DepthFilterTest
{
public:
	DepthFilterTest();
	virtual ~DepthFilterTest();
	void depthFilterCB(mvo::Point3D* point, double depth_sigma2);
	void testReconstruction(std::string dataset_dir, std::string experiment_name);

public:
	std::list<ConvergedSeed> results_;
	std::vector<double> errors_;
	size_t n_converged_seeds_;
	mvo::PinholeCamera* cam_;
	mvo::DepthFilter* depth_filter_;
	mvo::FramePtr frame_ref_;
	mvo::FramePtr frame_cur_;
	cv::Mat depth_ref_;
};

DepthFilterTest::DepthFilterTest() :
n_converged_seeds_(0),
cam_(new mvo::PinholeCamera(752, 480, 315.5, 315.5, 376.0, 240.0)),
depth_filter_(NULL)
{
	errors_.reserve(1000);
}

DepthFilterTest::~DepthFilterTest()
{
	delete cam_;
}

void DepthFilterTest::depthFilterCB(mvo::Point3D* point, double depth_sigma2)
{
	double depth = (frame_ref_->pos() - point->pos_).norm();
	double error = fabs(depth - depth_ref_.at<float>(point->obs_.front()->px[1],
		point->obs_.front()->px[0]));
	results_.push_back(ConvergedSeed(
		point->obs_.front()->px[0], point->obs_.front()->px[1], depth, error));
	errors_.push_back(error);
	delete point->obs_.front();
	delete point;
	++n_converged_seeds_;
}

void DepthFilterTest::testReconstruction(
	std::string dataset_dir,
	std::string experiment_name)
{
	//读取图像名和姿态
	std::string file_name = dataset_dir + "/trajectory.txt";
	mvo::FileReader<mvo::blender_utils::file_format::ImageNameAndPose> sequence_file_reader(file_name);
	std::vector<mvo::blender_utils::file_format::ImageNameAndPose> sequence;
	sequence.reserve(10000);
	sequence_file_reader.skipComments();
	if (!sequence_file_reader.next())
		std::runtime_error("Failed to open sequence file");
	sequence_file_reader.readAllEntries(sequence);
	std::cout << "RUN EXPERIMENT: read " << sequence.size() << " dataset entries." << std::endl;

	// 构建depth filter
	DetectorPtr feature_detector(new mvo::FastDetector(
		cam_->width(), cam_->height(), mvo::Config::gridSize(), mvo::Config::nPyrLevels()));
	DepthFilter::callback_t depth_filter_cb = std::bind(&DepthFilterTest::depthFilterCB, this, std::placeholders::_1, std::placeholders::_2);
	depth_filter_ = new mvo::DepthFilter(feature_detector, depth_filter_cb);
	depth_filter_->options_.verbose = true;

	std::vector<mvo::blender_utils::file_format::ImageNameAndPose>::iterator it = sequence.begin();

	std::list<size_t> n_converged_per_iteration;
	// 遍历获取图像
	for (int i = 0; it != sequence.end() && i < 20; ++it, ++i)
	{
		std::string img_name(dataset_dir + "/img/" + (*it).image_name_ + "_0.png");
		printf("reading image: '%s'\n", img_name.c_str());
		cv::Mat img(cv::imread(img_name, 0));
		assert(!img.empty());

		Sophus::SE3 T_w_f(it->q_, it->t_);
		if (i == 0)
		{
			// 创建参考帧导入真实地图信息
			frame_ref_ = std::shared_ptr<mvo::Frame>(new mvo::Frame(cam_, img, 0.0));
			frame_ref_->T_f_w_ = T_w_f.inverse();//姿态变Rt
			depth_filter_->addKeyframe(frame_ref_, 2, 0.5);
			mvo::blender_utils::loadBlenderDepthmap(dataset_dir + "/depth/" + (*it).image_name_ + "_0.depth", *cam_, depth_ref_);
			continue;
		}

		n_converged_seeds_ = 0;
		frame_cur_ = std::shared_ptr<mvo::Frame>(new mvo::Frame(cam_, img, 0.0));
		frame_cur_->T_f_w_ = T_w_f.inverse();
		depth_filter_->addFrame(frame_cur_);
		n_converged_per_iteration.push_back(n_converged_seeds_);
	}
	//printf("Experiment '%s' took %f ms\n", experiment_name.c_str(), t.stop() * 1000);

	// compute mean, median and variance of error in converged area
	{
		printf("# converged:  \t %zu (ref: 287)\n", errors_.size());
		double sum_error = 0;
		std::for_each(errors_.begin(), errors_.end(), [&](double& e){sum_error += e; });
		printf("mean error:   \t %f cm (ref: 0.080357)\n", sum_error * 100 / errors_.size());
		std::vector<double>::iterator it = errors_.begin() + 0.5*errors_.size();
		std::nth_element(errors_.begin(), it, errors_.end());
		printf("50-percentile: \t %f cm (ref: 0.062042)\n", *it * 100);
		it = errors_.begin() + 0.8*errors_.size();
		std::nth_element(errors_.begin(), it, errors_.end());
		printf("80-percentile: \t %f cm (ref: 0.124526)\n", *it * 100);
		it = errors_.begin() + 0.95*errors_.size();
		std::nth_element(errors_.begin(), it, errors_.end());
		printf("95-percentile: \t %f cm (ref: 0.200417)\n", *it * 100);
	}

	// trace error
	std::string trace_name("./depth_filter_" + experiment_name + ".txt");
	std::ofstream ofs(trace_name.c_str());
	for (std::list<ConvergedSeed>::iterator i = results_.begin(); i != results_.end(); ++i)
		ofs << i->x_ << ", " << i->y_ << ", " << fabs(i->error_) << std::endl;
	ofs.close();

	// trace convergence rate
	trace_name = "./depth_filter_" + experiment_name + "_convergence.txt";
	ofs.open(trace_name.c_str());
	for (std::list<size_t>::iterator it = n_converged_per_iteration.begin();
		it != n_converged_per_iteration.end(); ++it)
		ofs << *it << std::endl;
	ofs.close();

	// write ply file for pointcloud visualization in Meshlab
	trace_name = "./depth_filter_" + experiment_name + ".ply";
	ofs.open(trace_name.c_str());
	ofs << "ply" << std::endl
		<< "format ascii 1.0" << std::endl
		<< "element vertex " << results_.size() << std::endl
		<< "property float x" << std::endl
		<< "property float y" << std::endl
		<< "property float z" << std::endl
		<< "property uchar blue" << std::endl
		<< "property uchar green" << std::endl
		<< "property uchar red" << std::endl
		<< "end_header" << std::endl;

	for (std::list<ConvergedSeed>::iterator i = results_.begin(); i != results_.end(); ++i)
	{
		//cv::Vec3b c = frame_ref_->img_pyr_[0].at<cv::Vec3b>(i->y_, i->x_);
		Eigen::Vector3d p = cam_->cam2world(i->x_, i->y_)*i->depth_;
		ofs << p[0] << " " << p[1] << " " << p[2] << " "
			//<< (int)c[0] << " " << (int)c[1] << " " << (int)c[2] << std::endl;
			<< "255 255 255" << std::endl;
	}
}

int main(int argc, char** argv)
{
	DepthFilterTest test;
	std::string dataset_dir("./sin2_tex2_h1_v8_d");
	std::string experiment_name("sin2_tex2_h1_v8_d");
	test.testReconstruction(dataset_dir, experiment_name);
	getchar();
	return 0;
}