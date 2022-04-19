#ifndef ATLAS_RECEIVER_HPP
#define ATLAS_RECEIVER_HPP

#include <cerrno>
#include <cmath> // For lround()
#include <csignal> // For signal()
#include <cstdio> // For fprintf()
#include <cstring> // For memcpy()
#include <string> // For stoi() and strerror()
#include <netdb.h> // For gethostbyname() and hostent
#include <netinet/in.h> // For IPPROTO_* macros and htons()
#include <sys/socket.h> // For socket support.
#include <unistd.h> // For close()
#include <vector>
#include <arpa/inet.h>

#include "rclcpp/rclcpp.hpp"
#include "atlas_driver/interfaces/atlas_byte_frame_listener.hpp"
#include "atlas_driver/interfaces/atlas_byte_frame_event.hpp"

/*
 * Reads bit stream from the Point One Nav Atlas and notifies all event 
 * listeners attached to this singleton object once a raw data packet 
 * been receieved.
 */
class AtlasReceiver {
public:

  /**
   * Singleton object. Only one message parser is necessary.
   * @return static reference to single AtlasReceiver instance.
   */
  static AtlasReceiver & getInstance() {
    static AtlasReceiver instance; // static method field instatiated once
    return instance;
  }

  /**
   * Initialize needed to set a ros envoronment for logging output.
   * @param node link to ros environment.
   * @return Nothing.
   */
  void initialize(rclcpp::Node * node, int port) {
    node_ = node;
    port_ = port;
  }

  /**
   * Adds an event listener to be notified for every byte frames received.
   * @param listener listener to be notified for byte frames received.
   * @return Nothing.
   */
  void addByteFrameListener(AtlasByteFrameListener & listener) {
    listenerList.push_back(&listener);
  }

  // TODO: remove byte frame listener

  /**
   * Read udp input stream from the Point One Nav until this
   * system has shut down or this node is killed.
   * @return Nothing.
   */
  void udp_service() {
    uint8_t buffer[1024];
    size_t total_bytes_read = 0;
    struct sockaddr_storage their_addr; // address of recieved packet
    socklen_t addr_len = sizeof(their_addr);
    char their_ip[INET6_ADDRSTRLEN];

    open_udp_socket(); 

    try {
      while(rclcpp::ok()) {
        // continuously read input stream until system is shutdown / rclcpp::ok().
        ssize_t bytes_read = recvfrom(sock_, buffer, sizeof(buffer), 0, (struct sockaddr *)&their_addr, &addr_len);
        if(bytes_read < 0) {
          RCLCPP_INFO(node_->get_logger(), "Error reading from socket: %s (%d)", std::strerror(errno), errno);
          break;
        }   
        else if (bytes_read == 0) {
          RCLCPP_INFO(node_->get_logger(), "Socket closed remotely.");
          break;
        }
        inet_ntop(their_addr.ss_family, get_in_addr((struct sockaddr *)&their_addr), their_ip, sizeof(their_ip));
        total_bytes_read += bytes_read;
        // notify listeners
        fireAtlasByteFrameEvent(buffer, bytes_read, their_ip);
      }
      close(sock_);
      RCLCPP_INFO(node_->get_logger(), "Finished. %zu bytes read.", total_bytes_read);
    }
    catch(std::exception const & ex) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Decoder exception: " << ex.what());
    }
  }

private:
  int port_ = 0;
  int sock_ = 0;
  std::vector<AtlasByteFrameListener *> listenerList;
  rclcpp::Node * node_;

  /* private constructor for singleton design */
  AtlasReceiver() {}

  /**
   * Notifies all AtlasByteFrameListeners of a newly recieved byte frame.
   * @param frame Raw byte frame received.
   * @param bytes_read Size of byte frame.
   * @param frame_ip Frame source ip.
   * @return Nothing.
   */
  void fireAtlasByteFrameEvent(uint8_t * frame, size_t bytes_read, char frame_ip[INET6_ADDRSTRLEN]) {
    AtlasByteFrameEvent evt(frame, bytes_read, frame_ip);
    for(AtlasByteFrameListener * listener : listenerList) {
      listener->receivedAtlasByteFrame(evt);
    }
  }

  /**
   * Creates and binds to a UDP network socket.
   * @return status code 0 on success, non-zero otherwise.
   */
  int open_udp_socket() {
    // create UDP socket.
    sock_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if(sock_ < 0) {
      RCLCPP_INFO(node_->get_logger(), "Error creating socket");
      return 2;
    }   
    // bind socket to port.
    sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port_);
    addr.sin_addr.s_addr = INADDR_ANY; // any local address
    int ret = bind(sock_, (struct sockaddr *) &addr, sizeof(addr));
    if(ret < 0) {
      close(sock_);
      RCLCPP_INFO(node_->get_logger(), "Error binding");
      return 3;
    }
    RCLCPP_INFO(node_->get_logger(), "Opened port %d", port_);
    return 0;
  }

  /**
   * IPv4 and IPv5 address resolution.
   * @param sa socket address.
   * @return Resolved ip address.
   */
  void * get_in_addr(struct sockaddr * sa) {
    if (sa->sa_family == AF_INET) {
        return &(((struct sockaddr_in *)sa)->sin_addr);
    }
    return &(((struct sockaddr_in6 *)sa)->sin6_addr);
  }
};

#endif