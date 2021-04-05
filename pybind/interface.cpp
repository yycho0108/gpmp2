#include <fstream>
#include <iostream>
#include <memory>

#include <Eigen/Core>

#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/PriorFactor.h>

#include "gpmp2/planner/BatchTrajOptimizer.h"
#include "gpmp2/planner/TrajUtils.h"

#include "gpmp2/dynamics/VehicleDynamicsFactorPose2.h"
#include "gpmp2/geometry/Pose2Vector.h"
#include "gpmp2/gp/GaussianProcessPriorPose2.h"
#include "gpmp2/kinematics/Pose2MobileBase.h"
#include "gpmp2/kinematics/Pose2MobileBaseModel.h"
#include "gpmp2/obstacle/ObstaclePlanarSDFFactorGPPose2MobileBase.h"
#include "gpmp2/obstacle/ObstaclePlanarSDFFactorPose2MobileBase.h"
#include "gpmp2/obstacle/PlanarSDF.h"
#include "gpmp2/planner/TrajUtils.h"

#include "interface.hpp"

class PlanarGPMP2::Impl {
 public:
  explicit Impl(const PlanarGPMP2Settings& opts);
  virtual ~Impl() = default;

  /**
   * @brief Set options. Triggers Init() with previous parameters if invoked
   * earlier.
   * @param opts
   */
  void SetOpts(const PlanarGPMP2Settings& opts);

  /**
   * @brief Initialize planning problem.

   * Internally, build expensive cache such as the SDF
   *
   * @param img_file Image for obstacle map
   * @param cell_size Metric conversion factor from image <-> world distances
   */
  void Init(const std::string& img_file, const float cell_size);

  /**
   * Plan with endpoint constraints (init/goal).
   */
  std::vector<float> Plan(const std::pair<float, float>& init_point,
                          const std::pair<float, float>& goal_point);

 private:
  PlanarGPMP2Settings opts_;

  std::shared_ptr<gpmp2::PlanarSDF> sdf_;
  std::shared_ptr<gpmp2::Pose2MobileBaseModel> robot_;

  // Cached noise model factors.
  gtsam::noiseModel::Base::shared_ptr Qc_model;
  gtsam::noiseModel::Base::shared_ptr pose_fix;
  gtsam::noiseModel::Base::shared_ptr vel_fix;

  // Saved Init() args
  std::string img_file_{""};
  float cell_size_{0.0F};
};

// Interfacing part
PlanarGPMP2::PlanarGPMP2(const PlanarGPMP2Settings& opts)
    : impl_(new Impl(opts)) {}

PlanarGPMP2::~PlanarGPMP2() {}

void PlanarGPMP2::SetOpts(const PlanarGPMP2Settings& opts) {
  return impl_->SetOpts(opts);
}

void PlanarGPMP2::Init(const std::string& img_file, const float cell_size) {
  return impl_->Init(img_file, cell_size);
}

std::vector<float> PlanarGPMP2::Plan(
    const std::pair<float, float>& init_point,
    const std::pair<float, float>& goal_point) {
  return impl_->Plan(init_point, goal_point);
}

PlanarGPMP2::Impl::Impl(const PlanarGPMP2Settings& opts) : opts_{opts} {
  this->SetOpts(opts);
}

void PlanarGPMP2::Impl::SetOpts(const PlanarGPMP2Settings& opts) {
  // Reset Robot ...
  gpmp2::BodySphereVector sphere_vec;
  sphere_vec.emplace_back(
      gpmp2::BodySphere{0, opts.radius, gtsam::Point3{0, 0, 0}});
  robot_ = std::make_shared<gpmp2::Pose2MobileBaseModel>(
      gpmp2::Pose2MobileBase{}, sphere_vec);

  // Initialize noise params...
  Eigen::MatrixXd Qc =
      opts.gp_sigma * Eigen::MatrixXd::Identity(robot_->dof(), robot_->dof());
  Qc_model = gtsam::noiseModel::Gaussian::Covariance(Qc);
  pose_fix = gtsam::noiseModel::Isotropic::Sigma(robot_->dof(), 0.0001);
  vel_fix = gtsam::noiseModel::Isotropic::Sigma(robot_->dof(), 0.0001);

  // Reinitialize if previous map was given
  // if (!img_file_.empty()) {
  //  Init(img_file_, cell_size_);
  //}
}

cv::Mat ComputeSdf(const cv::Mat& img, const float cell_size) {
  // NOTE(ycho): zero == obstacle in cv convention

  // Outer
  cv::Mat out(img.size(), CV_32FC1);
  cv::distanceTransform(255 - img, out, cv::DIST_L2, 3, CV_32FC1);

  // Inner
  cv::Mat out2;
  cv::distanceTransform(img, out2, cv::DIST_L2, 3, CV_32FC1);

  // Subtract complement + scale by `cell_size`
  out -= out2;
  out *= cell_size;

  return out;
}

void PlanarGPMP2::Impl::Init(const std::string& img_file,
                             const float cell_size) {
  // Save values...
  img_file_ = img_file;
  cell_size_ = cell_size;

  const cv::Mat& map_img_cv = cv::imread(img_file, cv::IMREAD_GRAYSCALE);
  const cv::Mat& sdf_cv = ComputeSdf(map_img_cv, cell_size);

  // Convert SDF to GTSAM matrix -> PlanarSDF
  gtsam::Matrix msdf;
  cv::cv2eigen(sdf_cv, msdf);
  sdf_ =
      std::make_shared<gpmp2::PlanarSDF>(gtsam::Point2{0, 0}, cell_size, msdf);
}

template <typename Value>
Value Lerp(const Value& a, const Value& b, const float w) {
  return (a * (1.0 - w)) + (b * w);
}

std::vector<float> PlanarGPMP2::Impl::Plan(
    const std::pair<float, float>& init_point,
    const std::pair<float, float>& goal_point) {
  // Define some derived parameters (can be cached, technically)
  const float dt = opts_.total_time_sec / opts_.total_time_step;
  const float idt = (1.0 / dt);
  const int plot_inter = opts_.check_inter;
  const int total_plot_step = opts_.total_time_step * (plot_inter + 1);
  const int total_check_step = (opts_.check_inter + 1) * opts_.total_time_step;

  // Convert Endpoints.
  auto point0 =
      sdf_->convertCelltoPoint2({init_point.first, init_point.second});
  auto point1 =
      sdf_->convertCelltoPoint2({goal_point.first, goal_point.second});

  // Initialize pose-vel
  gtsam::Pose2 pose0{point0.x(), point0.y(), 0};
  gtsam::Pose2 pose1{point1.x(), point1.y(), 0};
  Eigen::VectorXd vel0(3);
  vel0 << 0.0, 0.0, 0.0;
  Eigen::VectorXd vel1(3);
  vel1 << 0.0, 0.0, 0.0;

  Eigen::VectorXd avg_vel(3);

  // TODO(ycho): (pose1.theta() - pose0.theta()) is INCORRECT
  // for angle wrapping.
  avg_vel << idt * (pose1.x() - pose0.x()), idt * (pose1.y() - pose0.y()),
      idt * (pose1.theta() - pose0.theta());

  // Initialize Values
  gtsam::Values init_values{};
  for (int i = 0; i <= opts_.total_time_step; ++i) {
    auto key_pos = gtsam::symbol('x', i);
    auto key_vel = gtsam::symbol('v', i);

    const float alpha = static_cast<float>(i) / opts_.total_time_step;
    auto pose = gtsam::Pose2{Lerp(pose0.x(), pose1.x(), alpha),
                             Lerp(pose0.y(), pose1.y(), alpha),
                             Lerp(pose0.theta(), pose1.theta(), alpha)};

    auto vel = avg_vel;

    init_values.insert(key_pos, pose);
    init_values.insert(key_vel, vel);
  }

  // Build graph.
  auto graph = gtsam::NonlinearFactorGraph{};
  for (int i = 0; i <= opts_.total_time_step; ++i) {
    auto key_pos = gtsam::symbol('x', i);
    auto key_vel = gtsam::symbol('v', i);

    // start/end priors
    if (i == 0) {
      graph.add(gtsam::PriorFactor<gtsam::Pose2>(key_pos, pose0, pose_fix));
      graph.add(gtsam::PriorFactor<gtsam::Vector>(key_vel, vel0, vel_fix));
    } else if (i == opts_.total_time_step) {
      graph.add(gtsam::PriorFactor<gtsam::Pose2>(key_pos, pose1, pose_fix));
      graph.add(gtsam::PriorFactor<gtsam::Vector>(key_vel, vel1, vel_fix));
    }

    // cost factor
    graph.add(gpmp2::ObstaclePlanarSDFFactorPose2MobileBase(
        key_pos, *robot_, *sdf_, opts_.cost_sigma, opts_.epsilon_dist));

    // vehicle dynamics
    if (opts_.use_vehicle_dynamics) {
      graph.add(gpmp2::VehicleDynamicsFactorPose2(key_pos, key_vel,
                                                  opts_.dynamics_sigma));
    }

    // GP priors and cost factor
    if (i > 0) {
      auto key_pos1 = gtsam::symbol('x', i - 1);
      auto key_pos2 = gtsam::symbol('x', i);
      auto key_vel1 = gtsam::symbol('v', i - 1);
      auto key_vel2 = gtsam::symbol('v', i);
      graph.add(gpmp2::GaussianProcessPriorPose2(key_pos1, key_vel1, key_pos2,
                                                 key_vel2, dt, Qc_model));

      // GP cost factor
      for (int j = 1; j <= opts_.check_inter; ++j) {
        const float tau = j * (opts_.total_time_sec / total_check_step);
        graph.add(gpmp2::ObstaclePlanarSDFFactorGPPose2MobileBase(
            key_pos1, key_vel1, key_pos2, key_vel2, *robot_, *sdf_,
            opts_.cost_sigma, opts_.epsilon_dist, Qc_model, dt, tau));
      }
    }
  }

  // Optimize factor graph.
  auto params = gtsam::DoglegParams{};
  params.setVerbosity("SILENT");
  gtsam::DoglegOptimizer opt{graph, init_values, params};
  // auto params = gtsam::LevenbergMarquardtParams();
  // params.setVerbosity("ERROR");
  // gtsam::LevenbergMarquardtOptimizer opt{graph, init_values, params};
  opt.optimize();

  // Apply GP interpolation.
  const auto ires = gpmp2::interpolatePose2Traj(
      opt.values(), Qc_model, dt, plot_inter, 0, opts_.total_time_step);

  // Format output and return.
  // NOTE(ycho): Currently does NOT care about heading(theta) in pose2.
  std::vector<float> out;
  out.reserve(total_plot_step * 2);
  for (int i = 0; i <= total_plot_step; ++i) {
    // Retrieve key
    auto key_pos = gtsam::symbol('x', i);
    auto value = ires.at<gtsam::Pose2>(key_pos);

    // Convert key -> value
    gpmp2::PlanarSDF::float_index cell;
    try {
      cell = sdf_->convertPoint2toCell(gtsam::Point2(value.x(), value.y()));
    } catch (const gpmp2::SDFQueryOutOfRange& e) {
      continue;
    }

    // Output flattened (i,j)
    out.emplace_back(cell.get<0>());
    out.emplace_back(cell.get<1>());
  }

  return out;
}
