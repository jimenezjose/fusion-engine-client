#ifndef POINT_ONE_NAV_ATLAS_HPP
#define POINT_ONE_NAV_ATLAS_HPP

#include <vector>
#include <cstdio>

#include "point_one/fusion_engine/parsers/fusion_engine_framer.h"
#include "point_one/fusion_engine/messages/core.h"
#include "point_one/fusion_engine/messages/ros.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "gps_msgs/msg/gps_fix.hpp"

#include "atlas_driver/interface/atlas_message_listener.hpp"
#include "atlas_driver/interface/atlas_message_event.hpp"
#include "atlas_driver/interface/atlas_byte_frame_listener.hpp"
#include "atlas_driver/interface/atlas_byte_frame_event.hpp"
#include "atlas_driver/atlas_message_decoder.hpp"

using namespace point_one::fusion_engine::messages;
using namespace point_one::fusion_engine::messages::ros;
namespace point_one {
  namespace fusion_engine {
    void messageReceived(const messages::MessageHeader&, const void*);
  }
}

/*
 * Reads bit stream from the Point One Nav Atlas and notifies all event
 * listeners attached to this singelton object once a complete message has
 * been received.
 */
class PointOneNavAtlas : public AtlasByteFrameListener {

public:
  /**
   * Singleton object. Only one message parser is necessary.
   */
  static PointOneNavAtlas & getInstance() {
    static PointOneNavAtlas instance; // static method fields are instatiated once
    return instance;
  }

  /**
   * Initialize needed to set a ros envoronment for logging output.
   * @param node Link to ROS environment.
   * @return Nothing.
   */
  void initialize(rclcpp::Node * node) {
    decoder.initialize(node);
    this->node_ = node;
  }

  /**
   * Callback function for every new byte frame received from Atlas.
   * @note Inherited from AtlasByteFrameListener interface.
   * @param evt Wrapper that holds the byte frame data recieved.
   * @return Nothing.
   */
  void receivedAtlasByteFrame(AtlasByteFrameEvent & evt) {
    framer.OnData(evt.frame, evt.bytes_read);
  }

  /**
   * Callback function for every new parsed message received from Atlas.
   * @param header Metadata on payload.
   * @param payload_in Message received.
   * @return Nothing.
   */
  void messageReceived(const MessageHeader & header, const void * payload_in) {
    auto payload = static_cast<const uint8_t*>(payload_in);
    size_t message_size = sizeof(MessageHeader) + header.payload_size_bytes;

    if(header.message_type == MessageType::ROS_GPS_FIX) {
      auto & contents = *reinterpret_cast<const GPSFixMessage*>(payload); 
      AtlasMessageEvent evt( toGPSFix(contents) );
      fireAtlasMessageEvent(evt);
    } else if(header.message_type == MessageType::ROS_IMU) {
      auto & contents = *reinterpret_cast<const IMUMessage*>(payload);
    } 
  }
  
  /**
   * Main service to receive gps data from Atlas.
   * @return Nothing.
   */
  void service() {
    decoder.udp_service();
  }

  /**
   * Adds an event listener to be notified for every gps message received.
   * @param listener object to be notified for gps message received.
   * @return Nothing.
   */
  void addAtlasMessageListener(AtlasMessageListener & listener) {
    listenerList.push_back(&listener);
  }

private:
  point_one::fusion_engine::parsers::FusionEngineFramer framer;
  std::vector<AtlasMessageListener *> listenerList;
  AtlasMessageDecoder & decoder;
  rclcpp::Node * node_;

  /* only one instance will exist - singleton object. */
  PointOneNavAtlas() : framer(1024), decoder(AtlasMessageDecoder::getInstance()) {
    decoder.addByteFrameListener(*this);
    framer.SetMessageCallback(point_one::fusion_engine::messageReceived);
  }

  /**
   * Notifies all AtlasMessageListeners of a newly recieved gps message.
   * @param evt data sent to listeners.
   * @return Nothing.
   */
  void fireAtlasMessageEvent(AtlasMessageEvent evt) {
    for(AtlasMessageListener * listener : listenerList) {
      listener->receivedAtlasMessage(evt);
    }
  }

  /**
   * Helper method to translate atlas GPSFixMessage 
   * to ROS standard GPSFix.
   * @param contents Culprit gps data to be translated.
   * @return ROS standard message - GPSFix;
   */
  gps_msgs::msg::GPSFix toGPSFix(const GPSFixMessage & contents) {
    gps_msgs::msg::GPSFix gps_fix;
    gps_fix.latitude  = contents.latitude_deg;
    gps_fix.longitude = contents.longitude_deg;
    gps_fix.altitude  = contents.altitude_m;
    gps_fix.track     = contents.track_deg;
    gps_fix.speed     = contents.speed_mps;
    gps_fix.climb     = contents.climb_mps;
    gps_fix.pitch     = contents.pitch_deg;
    gps_fix.roll      = contents.roll_deg;
    gps_fix.dip       = contents.dip_deg;
    gps_fix.time      = 0;//contents.p1_time.seconds + (contents.p1_time.fraction_ns * 1e-9); // time since power-on
    gps_fix.gdop      = contents.gdop;
    gps_fix.hdop      = contents.hdop;
    gps_fix.vdop      = contents.vdop;
    gps_fix.tdop      = contents.tdop;
    gps_fix.err       = contents.err_3d_m;
    gps_fix.err_horz  = contents.err_horiz_m;
    gps_fix.err_vert  = contents.err_vert_m;
    gps_fix.err_speed = contents.err_speed_mps;
    gps_fix.err_climb = contents.err_climb_mps;
    gps_fix.err_time  = contents.err_time_sec;
    gps_fix.err_pitch = contents.err_pitch_deg;
    gps_fix.err_roll  = contents.err_roll_deg;
    gps_fix.err_dip   = contents.err_dip_deg;
    memcpy(&gps_fix.position_covariance, contents.position_covariance_m2, sizeof(contents.position_covariance_m2));
    gps_fix.position_covariance_type = contents.position_covariance_type;
    return gps_fix;
  }
};

// annoying
namespace point_one {
  namespace fusion_engine {
    void messageReceived(const messages::MessageHeader& header, const void* payload_in) {
      PointOneNavAtlas::getInstance().messageReceived(header, payload_in);
    }
  }
}

#endif