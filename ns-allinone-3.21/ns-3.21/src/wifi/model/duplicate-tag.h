#ifndef DUPLICATE_TAG_H
#define DUPLICATE_TAG_H

#include "ns3/packet.h"

namespace ns3 {

class Tag;

enum PacketType
{
  ACK = 0,
  BACK =1,
  RTS = 2,
  CTS = 3
};

class DuplicateTag : public Tag
{
public:
  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const;

  DuplicateTag ();
  DuplicateTag (uint16_t db, uint16_t db_result, PacketType p);
  void SetDuplicateBandwidth (uint16_t db);
  void SetDuplicateBandwidthResult (uint16_t db_result);
  void SetPacketType (PacketType p); 
  virtual void Serialize (TagBuffer i) const;
  virtual void Deserialize (TagBuffer i);
  virtual uint32_t GetSerializedSize () const;
  virtual void Print (std::ostream &os) const;

  uint16_t GetDuplicateBandwidth (void) const;
  uint16_t GetDuplicateBandwidthResult (void) const;
  uint8_t  GetPacketType (void) const;
private:
  uint16_t m_db;
  uint16_t m_dbr;
  PacketType m_pt;

};

} // namespace ns3

#endif /* DUPLICATE_TAG_H */
