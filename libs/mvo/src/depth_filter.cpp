/*************************************************************************
 * 文件名： depth_filter
 *
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 * 时间： 2015/8/29
 *
 * 说明： 
 *************************************************************************/
#include <openmvo/mvo/depth_filter.h>
#include <random>
#include <cmath>
#include <sophus/se3.h>
#include <openmvo/mvo/config.h>

namespace mvo
{
	using namespace Sophus;

	int Seed::batch_counter = 0;
	int Seed::seed_counter = 0;

	Seed::Seed(Feature* ftr, float depth_mean, float depth_min) :
		batch_id(batch_counter),
		id(seed_counter++),
		ftr(ftr),
		a(10),
		b(10),
		mu(1.0 / depth_mean),//???
		z_range(1.0 / depth_min),
		sigma2(z_range*z_range / 36)
	{}

	DepthFilter::DepthFilter(DetectorPtr feature_detector, callback_t seed_converged_cb) :
		feature_detector_(feature_detector),
		seed_converged_cb_(seed_converged_cb),
		seeds_updating_halt_(false),
		thread_(NULL),
		new_keyframe_set_(false),
		new_keyframe_min_depth_(0.0),
		new_keyframe_mean_depth_(0.0),
		is_runing(true)
	{}

	DepthFilter::~DepthFilter()
	{
		stopThread();
	}

	void DepthFilter::startThread()
	{
		thread_ = new std::thread(&DepthFilter::updateSeedsLoop, this);
	}

	void DepthFilter::stopThread()
	{
		if (thread_ != NULL)
		{
			seeds_updating_halt_ = true;
			thread_->detach();
			is_runing = false;
			thread_->join();
			thread_ = NULL;
		}
	}

	void DepthFilter::addFrame(FramePtr frame)
	{
		if (thread_ != NULL)
		{
			{
				lock_t lock(frame_queue_mut_);
				if (frame_queue_.size() > 2)
					frame_queue_.pop();
				frame_queue_.push(frame);
			}
			seeds_updating_halt_ = false;
			frame_queue_cond_.notify_one();// 通知帧序列已经添加帧了
		}
		else
			updateSeeds(frame);
	}

	void DepthFilter::addKeyframe(FramePtr frame, double depth_mean, double depth_min)
	{
		new_keyframe_min_depth_ = depth_min;
		new_keyframe_mean_depth_ = depth_mean;
		if (thread_ != NULL)
		{
			// 如果线程已经启动，将当期帧添加，设置通知表明有新帧加入
			new_keyframe_ = frame;
			new_keyframe_set_ = true;
			seeds_updating_halt_ = true;// 表明种子点需更新，禁止读取
			frame_queue_cond_.notify_one();// 通知有新的帧加入
		}
		else // 如果线程没有启动，则初始化种子点
			initializeSeeds(frame);
	}

	void DepthFilter::initializeSeeds(FramePtr frame)
	{
		Features new_features;
		// 将帧划分为格子，确定通过光流跟踪确定的特征是否在格子内，在，标识格子被占用
		// 以减小下一步特征检测的计算量
		feature_detector_->setExistingFeatures(frame->fts_);
		feature_detector_->detect(frame.get(), frame->img_pyr_,
			Config::triangMinCornerScore(), new_features);

		//对每个特征初始化一个新种子点
		seeds_updating_halt_ = true;
		lock_t lock(seeds_mut_); // 确保独立执行，updateSeeds函数暂停
		++Seed::batch_counter;
		std::for_each(new_features.begin(), new_features.end(), [&](Feature* ftr){
			seeds_.push_back(Seed(ftr, new_keyframe_mean_depth_, new_keyframe_min_depth_));
		});

		seeds_updating_halt_ = false;// 表明种子已经添加完成
	}

	void DepthFilter::updateSeedsLoop()
	{
		while (is_runing)
		{
			FramePtr frame;
			{
				lock_t lock(frame_queue_mut_);// 表明执行这一段是独占的
				// 等待是否有新帧加入
				while (frame_queue_.empty() && new_keyframe_set_ == false)
					frame_queue_cond_.wait(lock);
				if (new_keyframe_set_)// 有新帧加入之后
				{
					new_keyframe_set_ = false;// 设置是否有新帧加入的标识为false
					seeds_updating_halt_ = false;// 将种子点是否可更新标识设为false
					clearFrameQueue();// 找到关键帧，清除其它帧
					frame = new_keyframe_;// 设置新关键帧待处理
				}
				else
				{
					// 典型的生产者消费者模型， 先进先出，取出第一个元素 (back, push)->@@@@->(front, pop)
					frame = frame_queue_.front();
					frame_queue_.pop();
				}
			}
			updateSeeds(frame);
			if (frame->isKeyframe())//如果是关键帧，则初始化该帧的种子点
				initializeSeeds(frame);
		}
	}

	void DepthFilter::updateSeeds(FramePtr frame)
	{
		// 更新有限数目的种子点，因为我们没有时间对每帧中的所有种子点进行处理
		size_t n_updates = 0, n_failed_matches = 0, n_seeds = seeds_.size();
		lock_t lock(seeds_mut_);
		std::list<Seed>::iterator it = seeds_.begin();

		const double focal_length = frame->cam_->getFocalLength();// 得到焦距
		double px_noise = 1.0;//设置像素误差为1个像素
		double px_error_angle = atan(px_noise / (2.0*focal_length))*2.0; // 计算误差带来的角度变化误差

		while (it != seeds_.end())
		{
			// 表明种子点在添加
			if (seeds_updating_halt_)
				return;

			// 确保当前种子点对应的帧数不太大，以保证种子点较新
			if ((Seed::batch_counter - it->batch_id) > options_.max_n_kfs) {
				it = seeds_.erase(it);
				continue;
			}

			// 检测点是否在当前图像中可见
			SE3 T_ref_cur = it->ftr->frame->T_f_w_ * frame->T_f_w_.inverse();// 参考帧向当前帧的变换（姿态变换右乘）
			const Vector3d xyz_f(T_ref_cur.inverse()*(1.0 / it->mu * it->ftr->f));//1.0 / it->mu 也就是深度的均值
			if (xyz_f.z() < 0.0)  {
				++it; // 在相机后面
				continue;
			}
			if (!frame->cam_->isInFrame(frame->f2c(xyz_f).cast<int>())) {
				++it; // 点没有投影到相机中
				continue;
			}

			// 使用逆的深度坐标，这边为什么用逆呢
			float z_inv_min = it->mu + sqrt(it->sigma2);
			float z_inv_max = std::max(it->mu - sqrt(it->sigma2), 0.00000001f);
			double z;
			if (!matcher_.findEpipolarMatchDirect(
				*it->ftr->frame, *frame, *it->ftr, 1.0 / it->mu, 1.0 / z_inv_min, 1.0 / z_inv_max, z))
			{
				it->b++; // 如果没有发现匹配，则增加外点的概率
				++it;
				++n_failed_matches;
				continue;
			}

			// 计算测量的不确定性
			double tau = computeTau(T_ref_cur, it->ftr->f, z, px_error_angle);
			double tau_inverse = 0.5 * (1.0 / std::max(0.0000001, z - tau) - 1.0 / (z + tau));

			// 更新种子点
			updateSeed(1. / z, tau_inverse*tau_inverse, &*it);
			++n_updates;

			if (frame->isKeyframe())
			{
				// The feature detector should not initialize new seeds close to this location
				feature_detector_->setGridOccupancy(matcher_.px_cur_);
			}

			// if the seed has converged, we initialize a new candidate point and remove the seed
			if (sqrt(it->sigma2) < it->z_range / options_.seed_convergence_sigma2_thresh)
			{
				assert(it->ftr->point == NULL); // TODO this should not happen anymore
				Vector3d xyz_world(it->ftr->frame->T_f_w_.inverse() * (it->ftr->f * (1.0 / it->mu)));
				Point3D* point = new Point3D(xyz_world);
				point->obs_.push_front(it->ftr);
				it->ftr->point = point;			
				seed_converged_cb_(point, it->sigma2); // 添加到候选list			
				it = seeds_.erase(it);
			}
			else if (isnan(z_inv_min))
			{
				it = seeds_.erase(it);
			}
			else
				++it;
		}
	}

	void DepthFilter::clearFrameQueue()
	{
		while (!frame_queue_.empty())
			frame_queue_.pop();
	}

	void DepthFilter::updateSeed(const float x, const float tau2, Seed* seed)
	{
		float norm_scale = sqrt(seed->sigma2 + tau2);
		if (std::isnan(norm_scale))
			return;
		double pdf = (1.0 / (sqrt(2 * M_PI)*norm_scale) * exp(-1 * (x - seed->mu)*(x - seed->mu) / (2 * norm_scale*norm_scale)));
		float s2 = 1. / (1. / seed->sigma2 + 1. / tau2);
		float m = s2*(seed->mu / seed->sigma2 + x / tau2);
		float C1 = seed->a / (seed->a + seed->b) * pdf;
		float C2 = seed->b / (seed->a + seed->b) * 1. / seed->z_range;
		float normalization_constant = C1 + C2;
		C1 /= normalization_constant;
		C2 /= normalization_constant;
		float f = C1*(seed->a + 1.) / (seed->a + seed->b + 1.) + C2*seed->a / (seed->a + seed->b + 1.);
		float e = C1*(seed->a + 1.)*(seed->a + 2.) / ((seed->a + seed->b + 1.)*(seed->a + seed->b + 2.))
			+ C2*seed->a*(seed->a + 1.0f) / ((seed->a + seed->b + 1.0f)*(seed->a + seed->b + 2.0f));

		// update parameters
		float mu_new = C1*m + C2*seed->mu;
		seed->sigma2 = C1*(s2 + m*m) + C2*(seed->sigma2 + seed->mu*seed->mu) - mu_new*mu_new;
		seed->mu = mu_new;
		seed->a = (e - f) / (f - e / f);
		seed->b = seed->a*(1.0f - f) / f;
	}

	/// 计算测量的不确定性
	double DepthFilter::computeTau(
		const SE3& T_ref_cur,
		const Vector3d& f,
		const double z,
		const double px_error_angle)
	{
		Vector3d t(T_ref_cur.translation());
		Vector3d a = f*z - t;
		double t_norm = t.norm();
		double a_norm = a.norm();
		double alpha = acos(f.dot(t) / t_norm); // 点乘计算角度
		double beta = acos(a.dot(-t) / (t_norm*a_norm)); // 点乘计算角度
		double beta_plus = beta + px_error_angle;
		double gamma_plus = M_PI - alpha - beta_plus; // 三角之和180
		double z_plus = t_norm*sin(beta_plus) / sin(gamma_plus); // 正玄定理
		return (z_plus - z); // tau
	}
}