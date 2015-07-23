#include "ns3/assert.h"
#include "ns3/packet.h"
#include "ns3/boolean.h"

#include "ampdu-tag.h"

namespace ns3 {

TypeId 
AmpduTag::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::AmpduTag")
    .SetParent<Tag> ()
    .AddConstructor<AmpduTag> ()
    .AddAttribute ("Ampdu", "Bool that indicates this packet is A-MPDU", 
                   BooleanValue (false),
                   MakeBooleanAccessor (&AmpduTag::Get),
                   MakeBooleanChecker ())
  ;
  return tid;
}

TypeId 
AmpduTag::GetInstanceTypeId (void) const
{
  return GetTypeId ();
}

AmpduTag::AmpduTag ()
  : m_ampdu (0)
{
}

AmpduTag::AmpduTag (bool ampdu)
  : m_ampdu (ampdu)
{
}

uint32_t 
AmpduTag::GetSerializedSize (void) const
{
  return sizeof (bool);
}
void 
AmpduTag::Serialize (TagBuffer i) const
{
  i.WriteU8 ((uint8_t)m_ampdu);
}
void 
AmpduTag::Deserialize (TagBuffer i)
{
  m_ampdu = (bool)i.ReadU8 ();
}
void 
AmpduTag::Print (std::ostream &os) const
{
  os << "Ampdu =" << m_ampdu;
}
void 
AmpduTag::Set (bool ampdu)
{
  m_ampdu = ampdu;
}
bool 
AmpduTag::Get (void) const
{
  return m_ampdu;
}

} //namespace ns3
