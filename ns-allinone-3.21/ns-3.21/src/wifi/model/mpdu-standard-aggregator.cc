#include "ns3/log.h"
#include "ns3/uinteger.h"

//#include "ampdu-mpdu-delimiter.h"
#include "mpdu-standard-aggregator.h"

NS_LOG_COMPONENT_DEFINE ("MpduStandardAggregator");

namespace ns3 {

NS_OBJECT_ENSURE_REGISTERED (MpduStandardAggregator);

TypeId
MpduStandardAggregator::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::MpduStandardAggregator")
    .SetParent<MpduAggregator> ()
    .AddConstructor<MpduStandardAggregator> ()
    .AddAttribute ("MaxAmpduSize", "Max length in byte of an A-MPDU",
                   UintegerValue (65535),
                   MakeUintegerAccessor (&MpduStandardAggregator::m_maxAmpduLength),
                   MakeUintegerChecker<uint32_t> ())
  ;
  return tid;
}

MpduStandardAggregator::MpduStandardAggregator ()
{
}

MpduStandardAggregator::~MpduStandardAggregator ()
{
}

bool
MpduStandardAggregator::Aggregate (Ptr<const Packet> packet, Ptr<Packet> aggregatedPacket)
{
  NS_LOG_FUNCTION (this);
  Ptr<Packet> currentPacket;
  AmpduMpduDelimiter currentHdr;

  uint32_t padding = CalculatePadding (aggregatedPacket);
  uint32_t actualSize = aggregatedPacket->GetSize ();

	NS_LOG_DEBUG("padding size: "<< padding <<
							 ", actual size"<< actualSize );

  if ((14 + packet->GetSize () + actualSize + padding) <= m_maxAmpduLength)
    {
      if (padding)
        {
          Ptr<Packet> pad = Create<Packet> (padding);
          aggregatedPacket->AddAtEnd (pad);
        }
      currentHdr.SetLength (packet->GetSize ());
      currentPacket = packet->Copy ();

      currentPacket->AddHeader (currentHdr);
      aggregatedPacket->AddAtEnd (currentPacket);
      return true;
    }
  return false;
}
//bjkim 110929 get maximum size of available length for ampdu
uint32_t
MpduStandardAggregator::GetMaxAvailableLength (Ptr<Packet> aggregatedPacket)
{
  NS_LOG_FUNCTION (this);
  uint32_t padding = CalculatePadding (aggregatedPacket);
  uint32_t actualSize = aggregatedPacket->GetSize ();

  NS_LOG_DEBUG("padding size: "<< padding <<
							 ", actual size: "<< actualSize );


	if (m_maxAmpduLength -14 -actualSize -padding > m_maxAmpduLength) 
		return 0;
	else 
	  return (m_maxAmpduLength - 14 - actualSize - padding);
  
}

uint32_t
MpduStandardAggregator::CalculatePadding (Ptr<const Packet> packet)
{
  return (4 - (packet->GetSize () % 4 )) % 4;
}


}  // namespace ns3

