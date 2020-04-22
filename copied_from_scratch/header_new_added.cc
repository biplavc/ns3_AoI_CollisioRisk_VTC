#include "ns3/vector.h"
#include "ns3/string.h"
#include "ns3/socket.h"
#include "ns3/double.h"
#include "ns3/config.h"
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
#include "ns3/ocb-wifi-mac.h"
#include "ns3/wifi-80211p-helper.h"
#include "ns3/wave-mac-helper.h"
#include "ns3/wifi-mac-queue.h"
#include "ns3/random-waypoint-mobility-model.h"
#include "ns3/netanim-module.h"
#include "ns3/nstime.h"
#include "ns3/header.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("Header&TagAdded");

class loc_header : public Header 
{
	public:
    loc_header();
     //virtual ~loc_header();
    void SetLocation(const Vector location);
    const Vector GetLocation() const;// const Vector & GetPosition () const; the meaning of & here?

    void SetSpeed(const Vector location);
    const Vector GetSpeed() const;

    void SetTxTime(const uint64_t TxTime);
    const uint64_t GetTxTime() const;

    static TypeId GetTypeId (void);
    TypeId GetInstanceTypeId () const;

    virtual uint32_t GetSerializedSize (void) const;
    virtual void Serialize (Buffer::Iterator start) const;
    virtual uint32_t Deserialize (Buffer::Iterator start); 

    virtual void Print (std::ostream &os) const;

    const uint32_t GetSenderId() const;
    void SetSenderId(const uint32_t Id);
			
	private:
		Vector m_position;
    Vector m_speed;
    uint32_t SenderId;
    int64_t TxTime; //instant when the packet was transmitted
};

NS_OBJECT_ENSURE_REGISTERED (loc_header);

loc_header::loc_header(){}
// why empty constructor needed?? 
// https://groups.google.com/forum/#!searchin/ns-3-users/serialize$20location$20header|sort:date/ns-3-users/_V0IKIMBCOI/ajOHffpLAQAJ

TypeId loc_header::GetInstanceTypeId () const
{
  NS_LOG_FUNCTION (this);
  return GetTypeId ();
}

void loc_header::SetLocation(const Vector location)
{
	m_position = location;
}

const Vector loc_header::GetLocation() const // const Vector & GetPosition () const; the meaning of & here?
{
	return m_position;
}

TypeId loc_header::GetTypeId(void)
{
	static TypeId tid = TypeId ("ns3::loc_Header").SetParent<Header>().AddConstructor<loc_header>();
	return tid;
}

uint32_t loc_header::GetSerializedSize (void) const
{
	return (6*sizeof(double)+1*sizeof(uint32_t)+(sizeof(int64_t))); // x and y coordinates both are doubles, ID is uint32_t, time is made to int64_t
}

void loc_header::Serialize (Buffer::Iterator start) const // from Adeel cam header
{
	//NS_LOG_FUNCTION (this << &start);
	//Buffer::Iterator i = start;
	start.WriteHtonU64 (ceil(m_position.x*1000)); //position doubles are 8 bytes
	start.WriteHtonU64 (ceil(m_position.y*1000));
	start.WriteHtonU64 (ceil(m_position.z*1000));

  start.WriteHtonU64 (ceil(m_position.x*1000)); //velocity doubles are 8 bytes
	start.WriteHtonU64 (ceil(m_position.y*1000));
	start.WriteHtonU64 (ceil(m_position.z*1000));

  start.WriteHtonU32 (SenderId);
  start.WriteHtonU64 (TxTime); //transmitted time, data type as int64_t
}

uint32_t loc_header::Deserialize (Buffer::Iterator start)   // from Adeel cam header
{
	m_position.x = start.ReadNtohU64()/1000; // doubles are 8 bytes
	m_position.y = start.ReadNtohU64()/1000;
	m_position.z = start.ReadNtohU64()/1000;

  m_speed.x = start.ReadNtohU64()/1000; // doubles are 8 bytes
	m_speed.y = start.ReadNtohU64()/1000;
	m_speed.z = start.ReadNtohU64()/1000;

  SenderId = start.ReadNtohU32();
  TxTime = start.ReadNtohU64(); // txtime converted to a double in the sending function
  return GetSerializedSize ();
}

void loc_header::SetSpeed(const Vector speed)
{
  m_speed = speed;
}

const Vector loc_header::GetSpeed() const
{
  return m_speed;
}


void loc_header::Print(std::ostream &os) const 
{
  os << "Packet sent by Node "<<SenderId<<" from " << "( " << m_position.x << ", " << m_position.y << ", " << m_position.z << " )" << std::endl;
}

void loc_header::SetSenderId(const uint32_t Id)
{
    SenderId = Id;
}


const uint32_t loc_header::GetSenderId() const
{
    return SenderId;
}

void loc_header::SetTxTime(const uint64_t txtime)
{
    TxTime = txtime;
}

const uint64_t loc_header::GetTxTime() const
{
    return TxTime;
}


void ReceivePacket (Ptr<Socket> socket)
{
  Ptr<Packet> packet;
  Time rcv;
  Time sqhd;
  Time total_delay;
  while (packet = socket->Recv ())
  {

    SeqTsHeader seqTsx; 
    packet->RemoveHeader(seqTsx);
    rcv = Simulator::Now();
    sqhd = seqTsx.GetTs();
    total_delay = (rcv - sqhd); //total delay calculation
    loc_header new_header;
    packet->RemoveHeader(new_header);
    Vector old_position = new_header.GetLocation();
    Vector old_speed = new_header.GetSpeed();
    uint32_t SenderId = new_header.GetSenderId();
    int64_t TxTime = new_header.GetTxTime();
    Ptr<Node> Sender_Node = NodeList::GetNode (SenderId);
    std::cout<<"Packet sent by node " <<SenderId<< " and received by node "<< socket->GetNode()->GetId()<<" at "<<rcv<<", Packet generated at "<<sqhd<<", sent at "<<TxTime<<" and overall delay for this packet is "<<total_delay<<std::endl;
    double xx; double yy; double zz;
    Ptr<MobilityModel> mob1 = Sender_Node->GetObject<MobilityModel>();
    xx = mob1->GetPosition().x; yy = mob1->GetPosition().y; zz = mob1->GetPosition().z;
    double xx_speed; double yy_speed; double zz_speed;
    xx_speed = mob1->GetVelocity().x; yy_speed = mob1->GetVelocity().y; zz_speed = mob1->GetVelocity().z;

    std::cout<<"Old position : "<<old_position.x<<", "<<old_position.y<<", "<<old_position.z<<std::endl;
    std::cout<<"Actual positions are : "<<xx<<", "<<yy<<", "<<zz<<"\n"<<std::endl;

    std::cout<<"Old Speed : "<<old_speed.x<<", "<<old_speed.y<<", "<<old_speed.z<<std::endl;
    std::cout<<"Actual Speed are : "<<xx_speed<<", "<<yy_speed<<", "<<zz_speed<<"\n"<<std::endl;



    // NS_LOG_UNCOND ("Received one packet!");
    
  }
}

static void GenerateTraffic (Ptr<Socket> socket, uint32_t pktSize,
                             uint32_t pktCount, Time pktInterval )
{
  if (pktCount > 0)
    {
      
    Ptr<Packet> p = Create<Packet> (pktSize-(12+24+4)); //12 bytes seqts, 24 bytes loc_header and 4 bytes tag
    loc_header new_header;
    Ptr<MobilityModel> mob = socket->GetNode()->GetObject<MobilityModel>(); //https://stackoverflow.com/questions/57290850/how-to-get-and-print-mobile-nodes-position-in-aodv-ns3
    double x_old = mob->GetPosition().x; double y_old = mob->GetPosition().y; double z_old = mob->GetPosition().z;
    Vector3D vect(x_old, y_old, z_old);
    new_header.SetLocation(vect);

    double x_old_speed = mob->GetVelocity().x; double y_old_speed = mob->GetVelocity().y; double z_old_speed = mob->GetVelocity().z;
    Vector3D vect1(x_old_speed, y_old_speed, z_old_speed);
    new_header.SetSpeed(vect1);


    new_header.SetSenderId(socket->GetNode()->GetId());
    new_header.SetTxTime(Simulator::Now().GetNanoSeconds());
    p->AddHeader(new_header);
    SeqTsHeader seqTs;
    seqTs.SetSeq (pktCount);
    p->AddHeader (seqTs); //confirm if size of seqTs is 12 or not??
    socket->Send(p); // means sent for Contention
    Simulator::Schedule (pktInterval, &GenerateTraffic, socket, pktSize, pktCount - 1, pktInterval);
    }
  else
    {
      socket->Close ();
    }
}

void EnqueueTrace(std::string context, Ptr<const WifiMacQueueItem> item) {
  Time entry_time = Simulator::Now();

}

void DequeueTrace(std::string context, Ptr<const WifiMacQueueItem> item) {
  //what to do when the packet is dequeued. You can do something like
  Time delay = Simulator::Now() - item->GetTimeStamp();
  //std::cout<<"Inside Deque Current Time = "<<Simulator::Now() << "," << "Packet Generation Time = "<< item->GetTimeStamp() << std::endl;
  //std::cout << "Queueing Delay : " << delay << std::endl;
}

int main (int argc, char *argv[])
{
  std::string phyMode ("OfdmRate3MbpsBW10MHz");
  uint32_t packetSize = 1000; // bytes
  uint32_t numPackets = 3;
  double interval = 0.0; // seconds
  bool verbose = false;

  CommandLine cmd;

  cmd.AddValue ("phyMode", "Wifi Phy mode", phyMode);
  cmd.AddValue ("packetSize", "size of application packet sent", packetSize);
  cmd.AddValue ("numPackets", "number of packets generated", numPackets);
  cmd.AddValue ("interval", "interval (seconds) between packets", interval);
  cmd.AddValue ("verbose", "turn on all WifiNetDevice log components", verbose);
  cmd.Parse (argc, argv);
  // Convert to time object
  Time interPacketInterval = Seconds (interval);


  NodeContainer c;
  c.Create (2);

  // The below set of helpers will help us to put together the wifi NICs we want
  YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
  Ptr<YansWifiChannel> channel = wifiChannel.Create ();
  wifiPhy.SetChannel (channel);
  wifiPhy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11);
  QosWaveMacHelper wifi80211pMac = QosWaveMacHelper::Default ();
  Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default ();
  if (verbose)
    {
      wifi80211p.EnableLogComponents ();      // Turn on all Wifi 802.11p logging
    }

  wifi80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                      "DataMode",StringValue (phyMode),
                                      "ControlMode",StringValue (phyMode));
  NetDeviceContainer devices = wifi80211p.Install (wifiPhy, wifi80211pMac, c);


  MobilityHelper mobility;

  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  positionAlloc->Add (Vector (0.0, 0.0, 0.0));
  positionAlloc->Add (Vector (5.0, 0.0, 0.0));
  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (c);


  InternetStackHelper internet;
  internet.Install (c);

  Ipv4AddressHelper ipv4;
  NS_LOG_INFO ("Assign IP Addresses.");
  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer i = ipv4.Assign (devices);

  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
  Ptr<Socket> recvSink = Socket::CreateSocket (c.Get (0), tid);
  InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), 80);
  recvSink->Bind (local);
  recvSink->SetRecvCallback (MakeCallback (&ReceivePacket));

  Ptr<Socket> source = Socket::CreateSocket (c.Get (1), tid);
  InetSocketAddress remote = InetSocketAddress (Ipv4Address ("255.255.255.255"), 80);
  source->SetAllowBroadcast (true);
  source->Connect (remote);

  // MAC tracing starts
  std::string enqueue_path= "/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/*/$ns3::OcbWifiMac/*/Queue/Enqueue";
  std::string dequeue_path= "/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/*/$ns3::OcbWifiMac/*/Queue/Dequeue";
  Config::Connect (enqueue_path, MakeCallback (&EnqueueTrace));
  Config::Connect (dequeue_path, MakeCallback (&DequeueTrace));  
  // MAC Tracing ends

  Simulator::ScheduleWithContext (source->GetNode ()->GetId (),
                                  Seconds (1.0), &GenerateTraffic,
                                  source, packetSize, numPackets, interPacketInterval);

  AnimationInterface anim("first.xml");

  Simulator::Run ();
  Simulator::Destroy ();

  return 0;
}