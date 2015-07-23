#include "ns3/assert.h"
#include "ns3/packet.h"
#include "ns3/boolean.h"

#include "error-free-tag.h"

namespace ns3 {

TypeId 
ErrorFreeTag::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::ErrorFreeTag")
    .SetParent<Tag> ()
    .AddConstructor<ErrorFreeTag> ()
    .AddAttribute ("ErrorFree", "Bool that indicates this packet is error-free one", 
                   BooleanValue (false),
                   MakeBooleanAccessor (&ErrorFreeTag::Get),
                   MakeBooleanChecker ())
  ;
  return tid;
}
TypeId 
ErrorFreeTag::GetInstanceTypeId (void) const
{
  return GetTypeId ();
}

uint32_t 
ErrorFreeTag::GetSerializedSize (void) const
{
  return sizeof (bool);
}
void 
ErrorFreeTag::Serialize (TagBuffer i) const
{
  i.WriteU8 ((uint8_t)m_errorFree);
}
void 
ErrorFreeTag::Deserialize (TagBuffer i)
{
  m_errorFree = (bool)i.ReadU8 ();
}
void 
ErrorFreeTag::Print (std::ostream &os) const
{
  os << "ErrorFree =" << m_errorFree;
}
void 
ErrorFreeTag::Set (bool errorFree)
{
  m_errorFree = errorFree;
}
bool 
ErrorFreeTag::Get (void) const
{
  return m_errorFree;
}

}
