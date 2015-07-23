#ifndef ERROR_FREE_TAG_H
#define ERROR_FREE_TAG_H 

#include <stdint.h>
#include <ostream>

#include "ns3/tag.h"

namespace ns3 {

class ErrorFreeTag : public Tag
{
public:

  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const;

  virtual uint32_t GetSerializedSize (void) const;
  virtual void Serialize (TagBuffer i) const;
  virtual void Deserialize (TagBuffer i);
  virtual void Print (std::ostream &os) const;

  void Set (bool errorFree);
  bool Get (void) const;
private:
  bool m_errorFree;
};

} // namespace ns3

#endif /* ERROR_FREE_TAG_H */
