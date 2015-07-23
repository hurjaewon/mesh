#ifndef AMPDU_MPDU_DELIMETER_H
#define AMPDU_MPDU_DELIMETER_H

#include "ns3/header.h"
#include "ns3/mac48-address.h"

namespace ns3 {

class AmpduMpduDelimiter : public Header
{
  public:
    AmpduMpduDelimiter ();
    virtual ~AmpduMpduDelimiter ();

    static TypeId GetTypeId (void);
    virtual TypeId GetInstanceTypeId (void) const;
    virtual void Print (std::ostream &os) const;
    virtual uint32_t GetSerializedSize (void) const;
    virtual void Serialize (Buffer::Iterator start) const;
    virtual uint32_t Deserialize (Buffer::Iterator start);

    void SetLength (uint16_t);
    uint16_t GetLength (void) const;

  private:
    uint16_t m_length;
};

} // namespace ns3

#endif /* AMPDU_MPDU_DELIMETER_H */
