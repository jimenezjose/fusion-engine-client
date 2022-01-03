#ifndef ATLAS_BYTE_FRAME_EVENT_HPP
#define ATLAS_BYTE_FRAME_EVENT_HPP

#include <cstring>
#include <arpa/inet.h>

/**
 * Data class that wraps a received byte frame into a generic object.
 */
class AtlasByteFrameEvent {
public:
  uint8_t * frame;
  size_t bytes_read;
  char frame_ip[INET6_ADDRSTRLEN]; // address of received packet

  AtlasByteFrameEvent(uint8_t * frame, size_t bytes_read, char frame_ip[INET6_ADDRSTRLEN]) 
      : frame{frame}, bytes_read{bytes_read} {
    
    memcpy(this->frame_ip, frame_ip, INET6_ADDRSTRLEN);
  }

};

#endif