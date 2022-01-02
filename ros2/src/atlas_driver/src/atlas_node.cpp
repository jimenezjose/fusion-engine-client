#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp" 
#include "gps_msgs/msg/gps_fix.hpp"

#include "atlas_driver/interface/atlas_message_listener.hpp"
#include "atlas_driver/interface/atlas_message_event.hpp"
#include "atlas_driver/point_one_nav_atlas.hpp"
#include "atlas_driver/atlas_message_type.hpp"

class AtlasNode : public AtlasMessageListener, public rclcpp::Node {

public:
  AtlasNode() : Node("atlas_node"), gps(PointOneNavAtlas::getInstance()) {
    raw_msg_publisher_ = this->create_publisher<gps_msgs::msg::GPSFix>("/atlas/gps_fix", 1);
    timer_ = create_wall_timer(std::chrono::milliseconds(1), std::bind(&AtlasNode::serciveLoopCb, this));
    gps.initialize(this);
    gps.addAtlasMessageListener(*this);
  }

  /**
   * Callback function triggered by atlas receiving a complete message.
   */
  void receivedAtlasMessage(AtlasMessageEvent & evt) {
    std_msgs::msg::String message;

    if(evt.message_type == AtlasMessageType::GPS_FIX) {
      gps_msgs::msg::GPSFix gps_fix = evt.gps_fix;
      message.data = 
          "Position (LLA): " 
          + std::to_string(gps_fix.latitude) + ", " 
          + std::to_string(gps_fix.longitude) + ", " 
          + std::to_string(gps_fix.altitude)
          + " (deg, deg, m)";
      raw_msg_publisher_->publish(gps_fix);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    }
  }

private:
  PointOneNavAtlas & gps;
  rclcpp::Publisher<gps_msgs::msg::GPSFix>::SharedPtr raw_msg_publisher_;
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
  rclcpp::spin(std::make_shared<AtlasNode>());
  rclcpp::shutdown();
  return 0;
}