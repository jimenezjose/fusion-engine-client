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

#include "atlas_driver/interfaces/atlas_message_listener.hpp"
#include "atlas_driver/interfaces/atlas_message_event.hpp"
#include "atlas_driver/interfaces/atlas_byte_frame_listener.hpp"
#include "atlas_driver/interfaces/atlas_byte_frame_event.hpp"
#include "atlas_driver/atlas_receiver.hpp"
#include "atlas_driver/common/utils.hpp"

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
  void initialize(rclcpp::Node * node, int port) {
    recv.initialize(node, port);
    this->node_ = node;
  }

  /**
   * Adds an event listener to be notified for every gps message received.
   * @param listener object to be notified for gps message received.
   * @return Nothing.
   */
  void addAtlasMessageListener(AtlasMessageListener & listener) {
    listenerList.push_back(&listener);
  }

  /**
   * Callback function for every new parsed message received from Atlas.
   * @param header Metadata on payload.
   * @param payload_in Message received.
   * @return Nothing.
   */
  void messageReceived(const MessageHeader & header, const void * payload_in) {
    auto payload = static_cast<const uint8_t*>(payload_in);

    if(header.message_type == MessageType::ROS_GPS_FIX) {
      auto & contents = *reinterpret_cast<const GPSFixMessage*>(payload); 
      AtlasMessageEvent evt( AtlasUtils::toGPSFix(contents) );
      fireAtlasMessageEvent(evt);
    } 
    else if(header.message_type == MessageType::ROS_IMU) {
      auto & contents = *reinterpret_cast<const IMUMessage*>(payload);
      AtlasMessageEvent evt( AtlasUtils::toImu(contents) );
      fireAtlasMessageEvent(evt);
    } 
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
   * Main service to receive gps data from Atlas.
   * @return Nothing.
   */
  void service() {
    recv.udp_service();
  }

private:
  point_one::fusion_engine::parsers::FusionEngineFramer framer;
  std::vector<AtlasMessageListener *> listenerList;
  AtlasReceiver & recv;
  rclcpp::Node * node_;

  /* only one instance will exist - singleton object. */
  PointOneNavAtlas() : framer(1024), recv(AtlasReceiver::getInstance()) {
    recv.addByteFrameListener(*this);
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