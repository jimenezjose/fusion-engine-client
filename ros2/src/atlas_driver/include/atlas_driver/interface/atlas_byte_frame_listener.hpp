#ifndef ATLAS_BYTE_FRAME_LISTENER
#define ATLAS_BYTE_FRAME_LISTENER

#include "atlas_driver/interface/atlas_byte_frame_event.hpp"

/**
 * Abstract interface for clients listening to byte frames.
 */
class AtlasByteFrameListener {
public: 
  virtual void receivedAtlasByteFrame(AtlasByteFrameEvent & evt) = 0;
};

#endif