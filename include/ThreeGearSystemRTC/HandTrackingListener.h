#ifndef __HANDTRACKINGLISTENER_H__
#define __HANDTRACKINGLISTENER_H__

#include "HandTrackingMessage.h"

namespace HandTrackingClient
{
class HandTrackingListener
{
public:
    virtual ~HandTrackingListener() {}
    virtual void handleEvent(const HandTrackingMessage& message) = 0;

    // Called when the connection is dropped from the far end.
    // Note that it's not safe to do stuff like delete the client
    // object in response to this message, since it's still 'live.'
    virtual void handleConnectionClosed() = 0;
};
}

#endif // __HANDTRACKINGLISTENER_H__