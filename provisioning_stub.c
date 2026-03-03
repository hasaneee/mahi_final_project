#include "simplelink.h"

/* Some SDK versions do not define SL_EVENT_PROPAGATION_CONTINUE.
   Define it safely as enum value 0. */
#ifndef SL_EVENT_PROPAGATION_CONTINUE
#define SL_EVENT_PROPAGATION_CONTINUE ((_SlEventPropogationStatus_e)0)
#endif

_SlEventPropogationStatus_e sl_Provisioning_WlanEventHdl(SlWlanEvent_t *pWlanEvent)
{
    (void)pWlanEvent;
    return SL_EVENT_PROPAGATION_CONTINUE;
}

_SlEventPropogationStatus_e sl_Provisioning_NetAppEventHdl(SlNetAppEvent_t *pNetAppEvent)
{
    (void)pNetAppEvent;
    return SL_EVENT_PROPAGATION_CONTINUE;
}

_SlEventPropogationStatus_e sl_Provisioning_HttpServerEventHdl(SlHttpServerEvent_t *pHttpEvent,
                                                               SlHttpServerResponse_t *pHttpResponse)
{
    (void)pHttpEvent;
    (void)pHttpResponse;
    return SL_EVENT_PROPAGATION_CONTINUE;
}
