
#include <fmt/printf.h>
#include <boost/program_options.hpp>

#include <Eigen/Core>

#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/PriorFactor.h>

#include "gpmp2/dynamics/VehicleDynamicsFactorPose2.h"
#include "gpmp2/geometry/Pose2Vector.h"
#include "gpmp2/gp/GaussianProcessPriorPose2.h"
#include "gpmp2/kinematics/Pose2MobileBase.h"
#include "gpmp2/kinematics/Pose2MobileBaseModel.h"
#include "gpmp2/obstacle/ObstaclePlanarSDFFactorGPPose2MobileBase.h"
#include "gpmp2/obstacle/ObstaclePlanarSDFFactorPose2MobileBase.h"
#include "gpmp2/obstacle/PlanarSDF.h"
#include "gpmp2/planner/TrajUtils.h"

namespace po = boost::program_options;

cv::Mat LoadData() {
  return cv::imread("/home/jamiecho/Pictures/kaist-binary-map-ez.png",
                    cv::IMREAD_GRAYSCALE);
}

/**
 * @brief Compute signed distance functino to a wall. Might be inverted.
 *
 * @param img
 * @param out_ptr
 *
 * @return
 */
cv::Mat ComputeSdf(const cv::Mat& img, const float cell_size,
                   cv::Mat* const out_ptr) {
  // NOTE(ycho): zero == obstacle in cv convention
  cv::Mat out;
  if (out_ptr) {
    out = *out_ptr;
  } else {
    out.create(img.size(), CV_64FC1);
  }
  cv::distanceTransform(255 - img, out, cv::DIST_L2, 3, CV_64FC1);

  cv::Mat out2;
  cv::distanceTransform(img, out2, cv::DIST_L2, 3, CV_64FC1);
  out -= out2;

  out *= cell_size;

  return -out;
}

struct Settings {
  // just a fact of life
  float cell_size{0.666};

  // start/goal
  float i0{142};
  float j0{228};
  float i1{573};
  float j1{315};

  float total_time_sec = 2.0F;
  int total_time_step = 10;
  int check_inter = 5;

  // vehicle dynamics (specific for e.g. non-holonomic vehicles)
  bool use_vehicle_dynamics = true;
  float dynamics_sigma = 0.001;

  // smaller or bigger sigma?
  float cost_sigma = 0.01;

  float gp_sigma = 1.0;

  // distance from object that start non-zero cost
  // static constexpr const float epsilon_dist = 0.2;
  float epsilon_dist = 0.2;

  float radius = 1.0;
};

void Usage(const po::options_description& desc) {
  // clang-format off
  fmt::print(R"(Usage:
  rs_replay_app [options]
Description:
  Replay app for realsense record.
Options:
{}
  )",
             desc);
  // clang-format on
}

bool ParseArguments(int argc, char* argv[], Settings* const settings) {
  po::options_description desc("");

#define ADD_SETTINGS(x, y, z)                        \
  ((std::string(#x) + "," + std::string(y)).c_str(), \
   po::value(&settings->x)->default_value(settings->x), z)

  // clang-format off
  desc.add_options()
      ("help,h", "help")
      ADD_SETTINGS(total_time_step, "n", "Total time step nodes")
      ADD_SETTINGS(check_inter, "i", "Number of interpolation nodes")
      ADD_SETTINGS(total_time_sec, "t", "Total time in seconds, not sure")
      ADD_SETTINGS(cost_sigma, "s", "Cost Sigma")
      ADD_SETTINGS(epsilon_dist, "e", "Epsilon dist")
      ADD_SETTINGS(use_vehicle_dynamics, "d", "Vehicle dynamics")
      ADD_SETTINGS(dynamics_sigma, "q", "Vehicle dynamics sigma")
      ADD_SETTINGS(gp_sigma, "g", "GP sigma")
      ADD_SETTINGS(radius, "r", "robot radius")
      ;
  // clang-format on
#undef ADD_SETTINGS

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help")) {
    Usage(desc);
    return false;
  }
  return true;
}

template <typename Value>
Value Lerp(const Value& a, const Value& b, const float w) {
  return (a * (1.0 - w)) + (b * w);
}

cv::Mat PlotTrajectory(const gtsam::Values& res, const cv::Mat& vis0,
                       const gpmp2::PlanarSDF& sdf,
                       const gtsam::noiseModel::Base::shared_ptr& Qc_model,
                       const double delta_t, const int plot_inter,
                       const int total_time_step, const int total_plot_step) {
  const auto ires = gpmp2::interpolatePose2Traj(res, Qc_model, delta_t,
                                                plot_inter, 0, total_time_step);

  cv::Mat vis = vis0.clone();
  cv::Point prv{0, 0};
  for (int i = 0; i <= total_plot_step; ++i) {
    auto key_pos = gtsam::symbol('x', i);
    auto value = ires.at<gtsam::Pose2>(key_pos);

    gpmp2::PlanarSDF::float_index cell;
    try {
      cell = sdf.convertPoint2toCell(gtsam::Point2(value.x(), value.y()));
    } catch (const gpmp2::SDFQueryOutOfRange& e) {
      // std::cout << "OOB" << std::endl;
      // std::cout << value.x() << ',' << value.y() << std::endl;
      // break;
      continue;
    }
    // std::cout << cell.get<0>() << ',' << cell.get<1>() << std::endl;

    cv::Point cur{static_cast<int>(cell.get<1>()),
                  static_cast<int>(cell.get<0>())};
    if (i > 0) {
      cv::line(vis, prv, cur, 0.5, 2);
    }
    prv = cur;
  }
  cv::imshow("vis2", vis);
  while (1) {
    const int k = cv::waitKey(1);
    bool quit{false};
    switch (k) {
      case 'q':
      case 27:
        quit = true;
        break;
      default:
        break;
    }
    if (quit) {
      break;
    }
  }
  return vis;
}

int main(int argc, char* argv[]) {
  // args
  Settings opts{};
  if (!ParseArguments(argc, argv, &opts)) {
    return 0;
  }

  const float cell_size{opts.cell_size};  // 50m/75px
  const float i0{opts.i0};
  const float j0{opts.j0};
  const float i1{opts.i1};
  const float j1{opts.j1};

  const bool use_vehicle_dynamics{opts.use_vehicle_dynamics};
  const float dynamics_sigma{opts.dynamics_sigma};

  // Settings
  const float total_time_sec = opts.total_time_sec;
  const int total_time_step = opts.total_time_step;
  const int check_inter = opts.check_inter;
  const float delta_t = total_time_sec / total_time_step;
  const int total_check_step = (check_inter + 1) * total_time_step;
  // static constexpr const bool use_vehicle_dynamics = true;

  // smaller or bigger sigma?
  const float cost_sigma = opts.cost_sigma;

  // distance from object that start non-zero cost
  // static constexpr const float epsilon_dist = 0.2;
  const float epsilon_dist = opts.epsilon_dist;

  // NOTE(ycho): derived param for plotting
  const int plot_inter = opts.check_inter;
  const int total_plot_step = total_time_step * (plot_inter + 1);

  const cv::Mat& map_img = LoadData();
  const cv::Mat& cv_sdf = ComputeSdf(map_img, opts.cell_size, nullptr);

  cv::Mat vis;
  if (true) {
    cv::normalize(cv_sdf, vis, 0.0, 1.0, cv::NORM_MINMAX, CV_64FC1);
    // cv::imshow("cv_sdf", cv_sdf);
    cv::namedWindow("vis", cv::WINDOW_NORMAL);
    cv::imshow("vis", vis);
    // cv::waitKey(0);
  }

  // Convert SDF to gtsam matrix
  // 50m == 75px
  // cell_size = 50/75 = 0.66
  gtsam::Matrix msdf;
  cv::cv2eigen(cv_sdf, msdf);
  // where should origin be?
  gpmp2::PlanarSDF sdf{gtsam::Point2{0, 0}, cell_size, msdf};

  // Define robot
  gpmp2::BodySphereVector sphere_vec;
  sphere_vec.emplace_back(gpmp2::BodySphere(0, 1.0, gtsam::Point3{0, 0, 0}));
  gpmp2::Pose2MobileBaseModel robot(gpmp2::Pose2MobileBase{}, sphere_vec);

  // GP
  Eigen::MatrixXd Qc =
      opts.gp_sigma * Eigen::MatrixXd::Identity(robot.dof(), robot.dof());
  auto Qc_model = gtsam::noiseModel::Gaussian::Covariance(Qc);
  auto pose_fix = gtsam::noiseModel::Isotropic::Sigma(robot.dof(), 0.0001);
  auto vel_fix = gtsam::noiseModel::Isotropic::Sigma(robot.dof(), 0.0001);

  auto start_point = sdf.convertCelltoPoint2({i0, j0});
  auto end_point = sdf.convertCelltoPoint2({i1, j1});

  // TODO(ycho): pose {x,y} MIGHT be inverted .
  // point.y, point.x --> cell[0]=row, cell[1]=col
  // i.e. point.x == col, point.y == row
  auto start_pose = gtsam::Pose2{start_point.x(), start_point.y(), 0};
  Eigen::VectorXd start_vel(3);
  start_vel(0) = 0.0;
  start_vel(1) = 0.0;
  start_vel(2) = 0.0;
  // auto start_vel = Eigen::VectorXd{0, 0, 0};
  // auto end_pose = gtsam::Pose2{315 * cell_size, 573 * cell_size, 0};
  auto end_pose = gtsam::Pose2{end_point.x(), end_point.y(), 0};
  Eigen::VectorXd end_vel(3);
  end_vel(0) = 0.0;
  end_vel(1) = 0.0;
  end_vel(2) = 0.0;
  // auto end_vel = Eigen::VectorXd{0, 0, 0};

  // auto cell = sdf.convertPoint2toCell(gtsam::Point2{start_pose.x(),
  // start_pose.y()}); std::cout << cell.get<0>() << ',' << cell.get<1>() <<
  // std::endl; // ideally, (142,228)

  Eigen::VectorXd avg_vel(3);
  avg_vel(0) = (1.0 / delta_t) * (end_pose.x() - start_pose.x());
  avg_vel(1) = (1.0 / delta_t) * (end_pose.y() - start_pose.y());
  avg_vel(2) = (1.0 / delta_t) * (end_pose.theta() - start_pose.theta());

  static constexpr const float pause_time = 0.1;

  // Initialize
  auto init_values = gtsam::Values{};
  for (int i = 0; i <= total_time_step; ++i) {
    auto key_pos = gtsam::symbol('x', i);
    auto key_vel = gtsam::symbol('v', i);

    const float alpha = static_cast<float>(i) / total_time_step;
    auto pose = gtsam::Pose2{Lerp(start_pose.x(), end_pose.x(), alpha),
                             Lerp(start_pose.y(), end_pose.y(), alpha),
                             Lerp(start_pose.theta(), end_pose.theta(), alpha)};

    auto vel = avg_vel;

    init_values.insert(key_pos, pose);
    init_values.insert(key_vel, vel);
  }

  // Build
  auto graph = gtsam::NonlinearFactorGraph{};
  for (int i = 0; i <= total_time_step; ++i) {
    auto key_pos = gtsam::symbol('x', i);
    auto key_vel = gtsam::symbol('v', i);

    // start/end priors
    if (i == 0) {
      graph.add(
          gtsam::PriorFactor<gtsam::Pose2>(key_pos, start_pose, pose_fix));
      graph.add(gtsam::PriorFactor<gtsam::Vector>(key_vel, start_vel, vel_fix));
    } else if (i == total_time_step) {
      graph.add(gtsam::PriorFactor<gtsam::Pose2>(key_pos, end_pose, pose_fix));
      graph.add(gtsam::PriorFactor<gtsam::Vector>(key_vel, end_vel, vel_fix));
    }

    // cost factor
    graph.add(gpmp2::ObstaclePlanarSDFFactorPose2MobileBase(
        key_pos, robot, sdf, cost_sigma, epsilon_dist));

    // vehicle dynamics
    if (use_vehicle_dynamics) {
      graph.add(
          gpmp2::VehicleDynamicsFactorPose2(key_pos, key_vel, dynamics_sigma));
    }

    // GP priors and cost factor
    if (i > 0) {
      auto key_pos1 = gtsam::symbol('x', i - 1);
      auto key_pos2 = gtsam::symbol('x', i);
      auto key_vel1 = gtsam::symbol('v', i - 1);
      auto key_vel2 = gtsam::symbol('v', i);
      graph.add(gpmp2::GaussianProcessPriorPose2(key_pos1, key_vel1, key_pos2,
                                                 key_vel2, delta_t, Qc_model));

      // GP cost factor
      for (int j = 1; j <= check_inter; ++j) {
        const float tau = j * (total_time_sec / total_check_step);
        graph.add(gpmp2::ObstaclePlanarSDFFactorGPPose2MobileBase(
            key_pos1, key_vel1, key_pos2, key_vel2, robot, sdf, cost_sigma,
            epsilon_dist, Qc_model, delta_t, tau));
      }
    }
  }

  // Optimize
  auto params = gtsam::DoglegParams{};
  params.setVerbosity("ERROR");
  gtsam::DoglegOptimizer opt{graph, init_values, params};
  // auto params = gtsam::LevenbergMarquardtParams();
  // params.setVerbosity("ERROR");
  // gtsam::LevenbergMarquardtOptimizer opt{graph, init_values, params};

  map_img.convertTo(vis, CV_64FC1, 1.0 / 255.0);
  if (0) {
    opt.optimize();
  } else {
    for (int i = 0; i < 100; ++i) {
      opt.iterate();
      cv::Mat vis_i = PlotTrajectory(opt.values(), vis, sdf, Qc_model, delta_t, plot_inter,
                     total_time_step, total_plot_step);
      vis_i.convertTo(vis_i, CV_8UC1, 255.0, 0.0);
      cv::imwrite(fmt::format("/tmp/step-{:04d}.png", i), vis_i);
    }
  }
  return 0;
}
