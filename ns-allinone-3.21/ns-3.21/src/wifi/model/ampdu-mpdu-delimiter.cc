#include "ampdu-mpdu-delimiter.h"
#include "ns3/address-utils.h"

namespace ns3 {

NS_OBJECT_ENSURE_REGISTERED (AmpduMpduDelimiter);

TypeId
AmpduMpduDelimiter::GetTypeId ()
{
  static TypeId tid = TypeId ("ns3::AmpduMpduDelimiter")
    .SetParent<Header> ()
    .AddConstructor<AmpduMpduDelimiter> ()
  ;
  return tid;
}

TypeId
AmpduMpduDelimiter::GetInstanceTypeId (void) const
{
  return GetTypeId ();
}

AmpduMpduDelimiter::AmpduMpduDelimiter ()
  : m_length (0)
{
}

AmpduMpduDelimiter::~AmpduMpduDelimiter ()
{
}

uint32_t
AmpduMpduDelimiter::GetSerializedSize () const
{
 return 4;
}

void
AmpduMpduDelimiter::Serialize (Buffer::Iterator i) const
{
  i.WriteHtolsbU32 (m_length);
}

uint32_t
AmpduMpduDelimiter::Deserialize (Buffer::Iterator start)
{
  Buffer::Iterator i = start;
  m_length = i.ReadLsbtohU32();
  return i.GetDistanceFrom (start);
}

void
AmpduMpduDelimiter::Print (std::ostream &os) const
{
  os << "MPDU length = " << m_length;
}

void
AmpduMpduDelimiter::SetLength (uint16_t length)
{
  m_length = length;
}

uint16_t
AmpduMpduDelimiter::GetLength (void) const
{
  return m_length;
}

} // namespace ns3

