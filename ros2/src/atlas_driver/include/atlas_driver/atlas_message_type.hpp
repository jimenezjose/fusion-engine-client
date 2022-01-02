#ifndef ATLAS_MESSAGE_TYPE_HPP
#define ATLAS_MESSAGE_TYPE_HPP

/**
 * @breif Atlas message type definitions.
 */
enum class AtlasMessageType {
  /** Standard GPSFix message. 
   *  @see http://docs.ros.org/api/gps_common/html/msg/GPSFix.html
   * */
  GPS_FIX = 0,
  /** Standard ROS IMU message. 
   * @see http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html
   * */
  IMU = 1,
};

#endif
