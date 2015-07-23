#include <stdint.h>
#include <ostream>

#include "ns3/tag.h"

namespace ns3 {

class AmpduTag : public Tag
{
  public:
    AmpduTag ();
    AmpduTag (bool ampdu);
    static TypeId GetTypeId (void);
    virtual TypeId GetInstanceTypeId (void) const;
    virtual uint32_t GetSerializedSize (void) const;
    virtual void Serialize (TagBuffer i) const;
    virtual void Deserialize (TagBuffer i);
    virtual void Print (std::ostream &os) const;
    void Set (bool ampdu);
    bool Get (void) const;
  private:
    bool m_ampdu;
};

} // namespace ns3
