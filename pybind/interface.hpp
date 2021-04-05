#pragma once

#include <memory>
#include <string>
#include <tuple>
#include <vector>

struct PlanarGPMP2Settings {
  // Trajectory support state parameters
  float total_time_sec = 2.0F;
  int total_time_step = 10;
  int check_inter = 5;

  // Whether to enable vehicle dynamics.
  // (specific for e.g. non-holonomic vehicles)
  bool use_vehicle_dynamics = true;
  float dynamics_sigma = 0.001;

  // Smaller or bigger sigma(s)?
  float cost_sigma = 0.01;
  float gp_sigma = 1.0;

  // distance from object that start non-zero cost
  // static constexpr const float epsilon_dist = 0.2;
  float epsilon_dist = 0.2;

  // Robot radius. For simplicity, the robot
  // is represented by a single hypersphere.
  float radius = 1.0;

  std::string verbosity = "SILENT";
};

class PlanarGPMP2 {
 public:
  explicit PlanarGPMP2(const PlanarGPMP2Settings& opts);
  virtual ~PlanarGPMP2();

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
  class Impl;
  std::unique_ptr<Impl> impl_;
};
