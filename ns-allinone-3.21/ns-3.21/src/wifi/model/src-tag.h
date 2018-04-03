#include <stdint.h>
#include <ostream>

#include "ns3/tag.h"

namespace ns3 {
	
	class SrcTag : public Tag
	{
		public:
			SrcTag ();
			SrcTag (uint32_t src);
			static TypeId GetTypeId (void);
			virtual TypeId GetInstanceTypeId (void) const;
			virtual uint32_t GetSerializedSize (void) const;
			virtual void Serialize (TagBuffer i) const;
			virtual void Deserialize (TagBuffer i);
			virtual void Print (std::ostream &os) const;
			void Set (uint32_t src);
			uint32_t Get (void) const;

		private:
			uint32_t m_src;
	};

} // namespace ns3
