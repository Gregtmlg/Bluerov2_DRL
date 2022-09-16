#include <gazebo_pressure_plugin.h>

namespace gazebo {
GZ_REGISTER_MODEL_PLUGIN(PressurePlugin)

PressurePlugin::PressurePlugin()
    : ModelPlugin(), baro_rnd_y2_(0.0), baro_rnd_use_last_(false) {}

PressurePlugin::~PressurePlugin() { update_connection_->~Connection(); }

void PressurePlugin::getSdfParams(sdf::ElementPtr sdf) {
  namespace_.clear();
  if (sdf->HasElement("robotNamespace")) {
    namespace_ = sdf->GetElement("robotNamespace")->Get<std::string>();
  }
  if (sdf->HasElement("pubRate")) {
    pub_rate_ = sdf->GetElement("pubRate")->Get<double>();
  } else {
    pub_rate_ = kDefaultPubRate;
    gzwarn << "[pressure_plugin] Using default publication rate of "
           << pub_rate_ << "Hz\n";
  }
  if (sdf->HasElement("pressureTopic")) {
    pressure_topic_ = sdf->GetElement("pressureTopic")->Get<std::string>();
  } else {
    pressure_topic_ = kDefaultPressureTopic;
  }
  if (sdf->HasElement("noise")) {
    pressure_noise_ = sdf->GetElement("noise")->Get<double>();
  } else {
    pressure_noise_ = kDefaultPressureNoise;
  }
}

void PressurePlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf) {
  getSdfParams(sdf);
  model_ = model;
  world_ = model_->GetWorld();
  last_time_ = world_->SimTime();
  last_pub_time_ = world_->SimTime();

  if (!ros::isInitialized()) {
    ROS_FATAL_STREAM("Ros node for gazebo not initialized");
    return;
  }
  node_handle_ = new ros::NodeHandle(namespace_);

  update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&PressurePlugin::OnUpdate, this, _1));


  pressure_pub_ =
      node_handle_->advertise<sensor_msgs::FluidPressure>(namespace_ + "/" + pressure_topic_, 1);
}

void PressurePlugin::OnUpdate(const common::UpdateInfo &) {
  common::Time current_time = world_->SimTime();
  double dt = (current_time - last_pub_time_).Double();

  if (dt > 1.0 / pub_rate_) {
    sensor_msgs::FluidPressure msg;
    double z_height =
        model_->GetLink("pressure_sensor_link")->WorldPose().Pos().Z();

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    // pressure increases by 10 kPa/m water depth.
    // pressure decreases roughly 100 Pa/8m in air.
    double msl_pressure = 101325.0;

    if (z_height > 0)
      msg.fluid_pressure = msl_pressure - z_height * 12.5;
    else
      msg.fluid_pressure = msl_pressure - z_height * 10000;

    // generate Gaussian noise sequence using polar form of Box-Muller
    // transformation
    double x1, x2, w, y1;
    if (!baro_rnd_use_last_) {
      do {
        x1 = 2.0 * standard_normal_distribution_(random_generator_) - 1.0;
        x2 = 2.0 * standard_normal_distribution_(random_generator_) - 1.0;
        w = x1 * x1 + x2 * x2;
      } while (w >= 1.0);
      w = sqrt((-2.0 * log(w)) / w);
      // calculate two values - the second value can be used next time because
      // it is uncorrelated
      y1 = x1 * w;
      baro_rnd_y2_ = x2 * w;
      baro_rnd_use_last_ = true;
    } else {
      // no need to repeat the calculation - use the second value from last
      // update
      y1 = baro_rnd_y2_;
      baro_rnd_use_last_ = false;
    }

    // apply noise.
    double noise = pressure_noise_ * y1;
    msg.fluid_pressure += noise;

    pressure_pub_.publish(msg);
    last_pub_time_ = current_time;
  }
}
}  // namespace gazebo
