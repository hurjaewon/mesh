#include "ns3/log.h"

#include "mpdu-aggregator.h"
#include "wifi-mac-header.h"

NS_LOG_COMPONENT_DEFINE ("MpduAggregator");

namespace ns3 {

NS_OBJECT_ENSURE_REGISTERED (MpduAggregator);

TypeId
MpduAggregator::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::MpduAggregator")
    .SetParent<Object> ()
  ;
  return tid;
}

MpduAggregator::DeaggregatedMpdus
MpduAggregator::Deaggregate (Ptr<Packet> aggregatedPacket)
{
  NS_LOG_FUNCTION_NOARGS ();
  DeaggregatedMpdus set;

  AmpduMpduDelimiter hdr;
  Ptr<Packet> extractedMpdu = Create<Packet> ();
  uint32_t maxSize = aggregatedPacket->GetSize ();
  uint16_t extractedLength;
  uint32_t padding;
  uint32_t deserialized = 0;

	NS_LOG_DEBUG ("maxSize:  " << maxSize);
  while (deserialized < maxSize)
    {
      deserialized += aggregatedPacket->RemoveHeader (hdr);
			NS_LOG_DEBUG ("deserialized:  " << deserialized);
      extractedLength = hdr.GetLength ();
      extractedMpdu = aggregatedPacket->CreateFragment (0, static_cast<uint32_t> (extractedLength));
      aggregatedPacket->RemoveAtStart (extractedLength);
      deserialized += extractedLength;
			NS_LOG_DEBUG ("deserialized:  " << deserialized);

      padding = (4 - ((extractedLength + 4) % 4 )) % 4;

			NS_LOG_DEBUG ("padding:  " << padding);

      if (padding > 0 && deserialized < maxSize)
        {
          aggregatedPacket->RemoveAtStart (padding);
          deserialized += padding;
					NS_LOG_DEBUG ("deserialized:  " << deserialized);
        }

      std::pair<Ptr<Packet>, AmpduMpduDelimiter> packetHdr (extractedMpdu, hdr);
      set.push_back (packetHdr);
    }
  NS_LOG_INFO ("Deaggreated A-MPDU: extracted " << set.size () << " MPDUs");
  return set;
}

} // namespace ns3

