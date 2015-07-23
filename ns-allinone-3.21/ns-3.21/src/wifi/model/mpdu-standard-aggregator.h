#ifndef MPDU_STANDARD_AGGREGATOR_H
#define MPDU_STANDARD_AGGREGATOR_H

#include "mpdu-aggregator.h"

namespace ns3 {

/**
 * \ingroup wifi
 * Standard MPDU aggregator
 *
 */
class MpduStandardAggregator : public MpduAggregator
{
public:
  static TypeId GetTypeId (void);
  MpduStandardAggregator ();
  ~MpduStandardAggregator ();
  /**
   * \param packet Packet we have to insert into <i>aggregatedPacket</i>.
   * \param aggregatedPacket Packet that will contain <i>packet</i>, if aggregation is possible,
   * \param src Source address of <i>packet</i>.
   * \param dest Destination address of <i>packet</i>.
   *
   * This method performs an MPDU aggregation.
   * Returns true if <i>packet</i> can be aggregated to <i>aggregatedPacket</i>, false otherwise.
   */
  virtual bool Aggregate (Ptr<const Packet> packet, Ptr<Packet> aggregatedPacket);
  
  virtual uint32_t GetMaxAvailableLength (Ptr<Packet> aggregatedPacket);
private:
  /*  Calculates how much padding must be added to the end of aggregated packet,
      after that a new packet is added.
      Each A-MPDU subframe is padded so that its length is multiple of 4 octets.
   */
  uint32_t CalculatePadding (Ptr<const Packet> packet);
  
  uint32_t m_maxAmpduLength;
};

}  // namespace ns3

#endif /* MPDU_STANDARD_AGGREGATOR_H */

