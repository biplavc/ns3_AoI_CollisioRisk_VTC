#include "ns3/bsm-application.h"
#include "ns3/wave-net-device.h"
#include "ns3/wave-helper.h"

#include "ns3/vector.h"
#include "ns3/string.h"
#include "ns3/socket.h"
#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/command-line.h"
#include "ns3/mobility-model.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/position-allocator.h"
#include "ns3/mobility-helper.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ipv4-interface-container.h"
#include <iostream>
#include "ns3/seq-ts-header.h"
#include "ns3/netanim-module.h"
#include "ns3/nstime.h"
#include "ns3/header.h"
#include <cmath>
#include <numeric>
#include "ns3/constant-velocity-mobility-model.h"
#include <cstdio>
#include <string>
#include <iterator>
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "loc_header.h"


namespace ns3 {


NS_LOG_COMPONENT_DEFINE ("Header Added");


// TypeId
// SeqTsHeader::GetTypeId (void)
// {
//   static TypeId tid = TypeId ("ns3::SeqTsHeader")
//     .SetParent<Header> ()
//     .SetGroupName("Applications")
//     .AddConstructor<SeqTsHeader> ()
//   ;
//   return tid;
// }

// TypeId
// SeqTsHeader::GetInstanceTypeId (void) const
// {
//   return GetTypeId ();
// }


loc_header::loc_header()
    {
        NS_LOG_FUNCTION (this);
    }
// https://groups.google.com/forum/#!searchin/ns-3-users/serialize$20location$20header|sort:date/ns-3-users/_V0IKIMBCOI/ajOHffpLAQAJ

TypeId loc_header::GetTypeId(void)
{
	static TypeId tid = TypeId ("ns3::loc_Header")
                        .SetParent<Header>()
                        .SetGroupName("Applications")
                        .AddConstructor<loc_header>();
	return tid;
}

TypeId loc_header::GetInstanceTypeId (void) const
{
  return GetTypeId ();
}

void loc_header::SetLocation(const Vector location)
{
	m_position = location;
}

void loc_header::SetSpeed(const Vector velocity)
{
	m_velocity = velocity;
}

const Vector loc_header::GetLocation() const // const Vector & GetPosition () const; the meaning of & here?
{
	return m_position;
}

const Vector loc_header::GetSpeed() const // const Vector & GetPosition () const; the meaning of & here?
{
	return m_velocity;
}


uint32_t loc_header::GetSerializedSize (void) const
{
	return (2*3*sizeof(double)+5*sizeof(uint32_t)); // x and y coordinates (3 each) both are doubles, IDs are uint32_t. Sender ID, MEssage ID, 2 IDs for x and y velocity signs
}

void loc_header::Serialize (Buffer::Iterator start) const // from Adeel cam header
{
	//NS_LOG_FUNCTION (this << &start);
	//Buffer::Iterator i = start;
start.WriteHtonU64 ((m_position.x*1000)); // doubles are 8 bytes
start.WriteHtonU64 ((m_position.y*1000));
start.WriteHtonU64 ((m_position.z*1000));

// velocities are written as positive values. While outputting, check flags

start.WriteHtonU64 ((abs(m_velocity.x)*1000)); // doubles are 8 bytes
start.WriteHtonU64 ((abs(m_velocity.y)*1000));
start.WriteHtonU64 ((abs(m_velocity.z)*1000));
start.WriteHtonU32 (SenderId); // 4 bytes uint32_t
start.WriteHtonU32(MID);
start.WriteHtonU32 (XvelID);
start.WriteHtonU32 (YvelID);
start.WriteHtonU32 (IntendedRx);

}

uint32_t loc_header::Deserialize (Buffer::Iterator start)   // from Adeel cam header
{
m_position.x = start.ReadNtohU64()/1000; // doubles are 8 bytes
m_position.y = start.ReadNtohU64()/1000;
m_position.z = start.ReadNtohU64()/1000;
m_velocity.x = start.ReadNtohU64()/1000; // doubles are 8 bytes
m_velocity.y = start.ReadNtohU64()/1000;
m_velocity.z = start.ReadNtohU64()/1000;
SenderId = start.ReadNtohU32();
MID = start.ReadNtohU32();
XvelID = start.ReadNtohU32();
YvelID = start.ReadNtohU32();
IntendedRx = start.ReadNtohU32();

return GetSerializedSize ();
}

void loc_header::Print(std::ostream &os) const 
{
  os << "Packet sent by Node "<<SenderId<<" from " << "(" << m_position.x << "," << m_position.y << "," << m_position.z <<std::endl; // ") with velocity " <<"("<<m_velocity.x << "," << m_velocity.y << "," << m_velocity.z <<")"<<std::endl;
}

void loc_header::SetSenderId(const uint32_t Id)
{
    SenderId = Id;
}

void loc_header::SetMsgId (const uint32_t MsgId)
{
    MID = MsgId;
}

void loc_header::SetXvelID(const uint32_t XID)
{
    XvelID = XID;
}

void loc_header::SetYvelID(const uint32_t YID)
{
    YvelID = YID;
}

void loc_header::SetIntendedRx(const uint32_t RxId)
{
    IntendedRx = RxId;
}

const uint32_t loc_header::GetXvelID() const
{
    return XvelID;
}

const uint32_t loc_header::GetYvelID() const
{
    return YvelID;
}

const uint32_t loc_header::GetMsgId() const
{
    return MID;
}

const uint32_t loc_header::GetIntendedRx() const
{
    return IntendedRx;
}


const uint32_t loc_header::GetSenderId() const
{
    return SenderId;
}
NS_OBJECT_ENSURE_REGISTERED (loc_header);

}



