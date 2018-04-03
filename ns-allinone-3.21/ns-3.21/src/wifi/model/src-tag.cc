#include "ns3/assert.h"
#include "ns3/packet.h"
#include "ns3/uinteger.h"

#include "src-tag.h"

namespace ns3 {

	TypeId SrcTag::GetTypeId (void)
	{
		static TypeId tid = TypeId ("ns3::SrcTag")
			.SetParent<Tag> ()
			.AddConstructor<SrcTag> ()
			.AddAttribute ("Src", "Node number of source",
											UintegerValue (0),
											MakeUintegerAccessor (&SrcTag::Get,
																						&SrcTag::Set),
											MakeUintegerChecker<uint32_t> ());
		return tid;
	}
	TypeId SrcTag::GetInstanceTypeId (void) const
	{
		return GetTypeId ();
	}
	SrcTag::SrcTag ()
		: m_src (0)
	{
	}
	SrcTag::SrcTag (uint32_t src)
		: m_src (src)
	{
	}
	uint32_t SrcTag::GetSerializedSize (void) const
	{
		return sizeof (uint32_t);
	}
	void SrcTag::Serialize (TagBuffer i) const
	{
		i.WriteU8 ((uint8_t)m_src);
	}
	void SrcTag::Deserialize (TagBuffer i)
	{
		m_src = (uint32_t)i.ReadU8 ();
	}
	void SrcTag::Print (std::ostream &os) const
	{
		os << "Src =" << m_src;
	}
	void SrcTag::Set (uint32_t src)
	{
		m_src = src;
	}

	uint32_t SrcTag::Get (void) const
	{
		return m_src;
	}

}
