#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp" 
#include "gps_msgs/msg/gps_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "atlas_driver/interfaces/atlas_message_listener.hpp"
#include "atlas_driver/interfaces/atlas_message_event.hpp"
#include "atlas_driver/common/atlas_message_type.hpp"
#include "atlas_driver/atlas.hpp"

/*
 * Point One Nav Atlas Node publishes realtime GPSFix/IMU messages.
 */ 
class PointOneNavAtlasNode : public AtlasMessageListener, public rclcpp::Node {

public:
  PointOneNavAtlasNode() : Node("atlas_node"), gps(PointOneNavAtlas::getInstance()) {
    gps_fix_publisher_ = this->create_publisher<gps_msgs::msg::GPSFix>("/atlas/gps_fix", 1);
    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/atlas/imu", 1);
    timer_ = create_wall_timer(std::chrono::milliseconds(1), std::bind(&PointOneNavAtlasNode::serciveLoopCb, this));
    gps.initialize(this);
    gps.addAtlasMessageListener(*this);
  }

  /**
   * Callback function triggered by atlas receiving a complete message.
   * @param evt GPS/IMU data.
   * @return Nothing.
   */
  void receivedAtlasMessage(AtlasMessageEvent & evt) {
    std_msgs::msg::String message;
    if(evt.message_type == AtlasMessageType::GPS_FIX) {
      message.data = 
          "Position (LLA): " 
          + std::to_string(evt.gps_fix.latitude) + ", " 
          + std::to_string(evt.gps_fix.longitude) + ", " 
          + std::to_string(evt.gps_fix.altitude)
          + " (deg, deg, m)";
      gps_fix_publisher_->publish(evt.gps_fix);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    }
    else if(evt.message_type == AtlasMessageType::IMU) {
      message.data = 
          "Angular velocity (x, y, z): ("
          + std::to_string(evt.imu.angular_velocity.x) + ","
          + std::to_string(evt.imu.angular_velocity.y) + ","
          + std::to_string(evt.imu.angular_velocity.z) + ","
          + ")";
      imu_publisher_->publish(evt.imu);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    }
  }

private:
  PointOneNavAtlas & gps;
  rclcpp::Publisher<gps_msgs::msg::GPSFix>::SharedPtr gps_fix_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  /**
   * Initiate gps unit to read data.
   */
  void serciveLoopCb() {
    RCLCPP_INFO(this->get_logger(), "Service");
    timer_->cancel(); // one-time enrty into service loop
    gps.service();
  }
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointOneNavAtlasNode>());
  rclcpp::shutdown();
  return 0;
}