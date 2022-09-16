#include <mavros/mavros_plugin.h>
#include <mavros_msgs/MotorSetpoint.h>

namespace mavros {
namespace extra_plugins {
class SetpointMotorPlugin : public plugin::PluginBase {
 public:
  SetpointMotorPlugin() : PluginBase(), nh("~setpoint_motor"){};

  void initialize(UAS &uas_)
  {
    PluginBase::initialize(uas_);
    setpoint_sub =
        nh.subscribe("setpoint", 1, &SetpointMotorPlugin::setpoint_cb, this);
  };
  Subscriptions get_subscriptions() { return {}; }

 private:
  ros::NodeHandle nh;
  ros::Subscriber setpoint_sub;

  void setpoint_cb(const mavros_msgs::MotorSetpoint::ConstPtr &req) {
    mavlink::common::msg::SETPOINT_MOTOR target{};
    target.time_usec = req->header.stamp.toNSec() / 1000;
    for (int i = 0; i < 8; i++) {
      target.setpoint[i] = req->setpoint[i];
    }
    UAS_FCU(m_uas)->send_message_ignore_drop(target);
  }
};
}  // namespace extra_plugins
}  // namespace mavros
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::SetpointMotorPlugin,
                       mavros::plugin::PluginBase)