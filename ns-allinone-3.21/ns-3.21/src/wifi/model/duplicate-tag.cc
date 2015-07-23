#include "duplicate-tag.h"
#include "ns3/tag.h"
#include "ns3/uinteger.h"

namespace ns3 {

NS_OBJECT_ENSURE_REGISTERED (DuplicateTag);

TypeId
DuplicateTag::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::DuplicateTag")
    .SetParent<Tag> ()
    .AddConstructor<DuplicateTag> ()
    .AddAttribute ("duplicateBandwidth", "duplicate bandwidth",
                   UintegerValue (0),
                   MakeUintegerAccessor (&DuplicateTag::GetDuplicateBandwidth),
                   MakeUintegerChecker<uint16_t> ())
    .AddAttribute ("duplicateBandwidthResult", "duplicate bandwidth result",
                   UintegerValue (0),
                   MakeUintegerAccessor (&DuplicateTag::GetDuplicateBandwidthResult),
                   MakeUintegerChecker<uint16_t> ())
    .AddAttribute ("PacketType", "packetType",
                   UintegerValue (0),
                   MakeUintegerAccessor (&DuplicateTag::GetPacketType),
                   MakeUintegerChecker<uint8_t> ())
  ;
  return tid;
}

TypeId
DuplicateTag::GetInstanceTypeId (void) const
{
  return GetTypeId ();
}

DuplicateTag::DuplicateTag ()
  : m_db (0),
    m_dbr (0),
    m_pt (ACK)
{
}
DuplicateTag::DuplicateTag (uint16_t db, uint16_t dbr, PacketType pt )
  : m_db (db),
  m_dbr (dbr),
  m_pt (pt)
{
}
void
DuplicateTag::SetDuplicateBandwidth (uint16_t db)
{
  m_db = db;
}
void
DuplicateTag::SetDuplicateBandwidthResult (uint16_t dbr)
{
  m_dbr = dbr;
}
void
DuplicateTag::SetPacketType (PacketType pt)
{
  m_pt = pt;
}

uint32_t
DuplicateTag::GetSerializedSize (void) const
{
  return (2*sizeof(uint16_t)+sizeof(uint8_t));
}

void
DuplicateTag::Serialize (TagBuffer i) const
{
  i.WriteU16 (m_db);
  i.WriteU16 (m_dbr);
  i.WriteU8 (m_pt);
}

void
DuplicateTag::Deserialize (TagBuffer i)
{
  m_db = i.ReadU16 ();
  m_dbr = i.ReadU16 ();
  m_pt = (PacketType) i.ReadU8 ();
}

uint16_t
DuplicateTag::GetDuplicateBandwidth () const
{
  return m_db;
}
uint16_t
DuplicateTag::GetDuplicateBandwidthResult () const
{
  return m_dbr;
}
uint8_t
DuplicateTag::GetPacketType () const
{
  return m_pt;
}
void
DuplicateTag::Print (std::ostream &os) const
{
  os << "duplicateBandwidth=" << m_db << " result=" << m_dbr << " packetType=" << (PacketType) m_pt;
}
} // namespace ns3
