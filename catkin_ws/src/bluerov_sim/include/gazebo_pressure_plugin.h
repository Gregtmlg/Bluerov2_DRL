#pragma once
#include <ros/ros.h>
#include <sensor_msgs/FluidPressure.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math.hh>
#include <sdf/sdf.hh>

namespace gazebo {
static constexpr auto kDefaultPubRate = 50.0;
static constexpr auto kDefaultPressureTopic = "pressure";
static constexpr auto kDefaultPressureNoise = 100.0;

class PressurePlugin : public ModelPlugin {
 public:
  PressurePlugin();
  virtual ~PressurePlugin();

 protected:
  virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf);
  virtual void OnUpdate(const common::UpdateInfo &);
  void getSdfParams(sdf::ElementPtr sdf);

 private:
  std::string namespace_;
  physics::ModelPtr model_;
  physics::WorldPtr world_;
  event::ConnectionPtr update_connection_;
  std::string pressure_topic_;
  double pressure_noise_;

  ros::NodeHandle *node_handle_;
  ros::Publisher pressure_pub_;

  double pub_rate_;

  std::default_random_engine random_generator_;
  std::normal_distribution<double> standard_normal_distribution_;

  common::Time last_pub_time_;
  common::Time last_time_;

  double baro_rnd_y2_;
  bool baro_rnd_use_last_;
};
}  // namespace gazebo
