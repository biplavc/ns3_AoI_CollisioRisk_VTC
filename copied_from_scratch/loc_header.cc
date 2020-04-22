
//#include "ns3/loc_header.h"
#include "ns3/nstime.h"
#include "ns3/header.h"
#include "ns3/vector.h"



void loc_header::SetLocation(const Vector &location)
{
	m_position = location;
}
const Vector loc_header::GetLocation() const // const Vector & GetPosition () const; the meaning of & here?
{
	return m_position;
}
static TypeId loc_header::GetTypeId(void)
{
	static TypeId tid = TypeId ("ns3::MyHeader")
	.SetParent<Header> ();
	return tid;
}
//virtual TypeId GetInstanceTypeId (void) const;
uint32_t loc_header::GetSerializedSize (void) const
{
	return (2*sizeof(double)); // x and y coordinates both are doubles
}
void loc_header::Serialize (Buffer:Iterator start) const // from Adeel cam header
{
	NS_LOG_FUNCTION (this << &start);
	Buffer::Iterator i = start;
	i.WriteHtonU16 (ceil(m_position.x*1000));
	i.WriteHtonU16 (ceil(m_position.y*1000));
	i.WriteHtonU16 (ceil(m_position.z*1000));
	
}
void loc_header::Deserialize (Buffer::Iterator start) const;   // from Adeel cam header
	NS_LOG_FUNCTION(this << &start);
	Buffer::Iterator i = start;
	m_position.x = i.ReadNtohU16()/1000;
	m_position.y = i.ReadNtohU16()/1000;
	m_position.z = i.ReadNtohU16()/1000;
