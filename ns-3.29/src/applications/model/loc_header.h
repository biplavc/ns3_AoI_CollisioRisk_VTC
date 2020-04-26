#ifndef LOC_HEADER_H
#define LOC_HEADER_H

#include "ns3/header.h"
// #include "ns3/bsm-application.h"
#include "ns3/vector.h"
#include "ns3/string.h"
#include "ns3/socket.h"
#include "ns3/double.h"
#include "ns3/config.h"
#include "ns3/log.h"
#include "ns3/command-line.h"
#include "ns3/mobility-model.h"
#include <iostream>
#include "ns3/seq-ts-header.h"
#include "ns3/netanim-module.h"
#include "ns3/nstime.h"
#include <cmath>
#include <numeric>
#include <cstdio>
#include <string>
#include <iterator>
#include "ns3/queue-size.h"
#include "ns3/config-store-module.h"
#include "ns3/config.h"
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wave-bsm-helper.h"

//BIPLAV HEADER STARTS
namespace ns3
{

//   static TypeId GetTypeId (void);

//   virtual TypeId GetInstanceTypeId (void) const;

class loc_header : public Header 
{
	public:
    loc_header();
     //virtual ~loc_header();
    void SetLocation(const Vector location);
    void SetSpeed(const Vector velocity);
    const Vector GetLocation() const;
    const Vector GetSpeed() const;
    virtual TypeId GetInstanceTypeId (void) const;
    static TypeId GetTypeId (void);
    virtual uint32_t GetSerializedSize (void) const;
    virtual void Serialize (Buffer::Iterator start) const;
    virtual uint32_t Deserialize (Buffer::Iterator start); 
    virtual void Print (std::ostream &os) const;
    const uint32_t GetSenderId() const;
    void SetSenderId(const uint32_t Id);
    void SetMsgId (const uint32_t MsgId);
    const uint32_t GetMsgId() const;
    const uint32_t GetXvelID() const;
    const uint32_t GetYvelID() const;
    // const uint32_t loc_header::GetIntendedRx() const
    const uint32_t GetIntendedRx() const;
    void SetXvelID(const uint32_t XvelID); // 1 for positive x-vel and 0 for negative x-vel
    void SetYvelID(const uint32_t YvelID); // 1 for positive y-vel and 0 for negative y-vel
    //void loc_header::SetIntendedRx(RxId)
    void SetIntendedRx(const uint32_t RxId); // set the intended risky receiver
 
	private:
	Vector m_position; // 3 double here
    Vector m_velocity; // 3 double here
    uint32_t SenderId; // one double
    uint32_t MID; //MID is message ID, one double
    uint32_t XvelID; // 1 for positive values and 0 for negative values, one double
    uint32_t YvelID; // 1 for positive values and 0 for negative values, one double
    uint32_t IntendedRx; // the vehicle who is risky
};

// NS_OBJECT_ENSURE_REGISTERED (loc_header); needed in the .cc file

}
#endif /* LOC_HEADER_H*/
