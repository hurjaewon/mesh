#ifndef MPDU_AGGREGATOR_H
#define MPDU_AGGREGATOR_H

#include "ns3/ptr.h"
#include "ns3/packet.h"
#include "ns3/object.h"

#include "ampdu-mpdu-delimiter.h"

#include <list>

namespace ns3 {

class WifiMacHeader;

/**
 * \brief Abstract class that concrete mpdu aggregators have to implement
 * \ingroup wifi
 */
class MpduAggregator : public Object
{
public:
  typedef std::list<std::pair<Ptr<Packet>, AmpduMpduDelimiter> > DeaggregatedMpdus;
  typedef std::list<std::pair<Ptr<Packet>, AmpduMpduDelimiter> >::const_iterator DeaggregatedMpdusCI;

  static TypeId GetTypeId (void);
  /* Adds <i>packet</i> to <i>aggregatedPacket</i>. In concrete aggregator's implementation is
   * specified how and if <i>packet</i> can be added to <i>aggregatedPacket</i>. If <i>packet</i>
   * can be added returns true, false otherwise.
   */
  virtual bool Aggregate (Ptr<const Packet> packet, Ptr<Packet> aggregatedPacket) = 0;

  virtual uint32_t GetMaxAvailableLength (Ptr<Packet> aggregatedPacket) = 0; 

  static DeaggregatedMpdus Deaggregate (Ptr<Packet> aggregatedPacket);
};

}  // namespace ns3

#endif /* MPDU_AGGREGATOR_H */

 
