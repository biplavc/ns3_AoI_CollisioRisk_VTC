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
#include "ns3/traffic-control-helper.h"
#include "ns3/queue-size.h"
#include "ns3/nstime.h"
#include "ns3/header.h"
#include "ns3/config-store-module.h"


using namespace ns3;
int collisions_MacTxDrop=0;
int collisions_PhyTxDrop=0;
int collisions_PhyRxDrop=0;
int collisions_MacRxDrop=0;
Time enqueue_time;
Time dequeue_time;

void
MacTxDrop(Ptr<const Packet> p)
{
  // NS_LOG_INFO("Packet Drop");
//   MacTxDropCount++;
  std::cout<<"MacTxDrop"<<std::endl;
  collisions_MacTxDrop  = collisions_MacTxDrop + 1;
}

void
PhyTxDrop(Ptr<const Packet> p)
{
  // NS_LOG_INFO("Packet Drop");
//   PhyTxDropCount++;
  std::cout<<"PhyTxDrop"<<std::endl;
  collisions_PhyTxDrop  = collisions_PhyTxDrop + 1;
}
void
PhyRxDrop(Ptr<const Packet> p)
{
  // NS_LOG_INFO("Packet Drop");
//   PhyRxDropCount++;
  std::cout<<"PhyRxDrop"<<std::endl;
  collisions_PhyRxDrop  = collisions_PhyRxDrop + 1;
}
void
MacRxDrop(Ptr<const Packet> p)
{
  // NS_LOG_INFO("Packet Drop");
//   PhyRxDropCount++;
  std::cout<<"MacRxDrop"<<std::endl;
  collisions_MacRxDrop  = collisions_MacRxDrop + 1;
}


NS_LOG_COMPONENT_DEFINE ("Header&TagAdded");

class dq_header : public Header 
{
	public:
    dq_header();
     //virtual ~dq_header();
    static TypeId GetTypeId(void);
    TypeId GetInstanceTypeId () const;
    uint32_t GetSerializedSize (void) const;
    void Serialize (Buffer::Iterator start) const;
    uint32_t Deserialize (Buffer::Iterator start); 
    void Print(std::ostream &os) const; 
    const uint64_t GetDequeueTime() const;
    void SetDequeueTime(const uint64_t dqtime);
  
	private:
    uint64_t DequeueTime;
};

NS_OBJECT_ENSURE_REGISTERED (dq_header);

dq_header::dq_header(){}

TypeId dq_header::GetInstanceTypeId () const
{
    NS_LOG_FUNCTION (this);
    return GetTypeId ();
}

TypeId dq_header::GetTypeId(void)
{
	static TypeId tid = TypeId ("ns3::dq_header").SetParent<Header>().AddConstructor<dq_header>();
	return tid;
}

uint32_t dq_header::GetSerializedSize (void) const
{
	return (1*sizeof(double)); // x and y coordinates both are doubles, IDs are uint32_t
}

void dq_header::Serialize (Buffer::Iterator start) const // from Adeel cam header
{
	//NS_LOG_FUNCTION (this << &start);
	//Buffer::Iterator i = start;
    start.WriteHtonU64 (DequeueTime);
}

uint32_t dq_header::Deserialize (Buffer::Iterator start)   // from Adeel cam header
{
    DequeueTime = start.ReadNtohU64();
    return GetSerializedSize ();
}

void dq_header::Print(std::ostream &os) const 
{
    os << "Dequeue Time is  "<<DequeueTime<<" "<< std::endl;
}

void dq_header::SetDequeueTime(const uint64_t dqtime)
{
    DequeueTime = dqtime;
}

const uint64_t dq_header::GetDequeueTime() const
{
    return DequeueTime;
}


class loc_header : public Header 
{
	public:
    loc_header();
     //virtual ~loc_header();
    void SetLocation(const Vector location);
    const Vector GetLocation() const;// const Vector & GetPosition () const; the meaning of & here?

    void SetSpeed(const Vector location);
    const Vector GetSpeed() const;

    static TypeId GetTypeId (void);
    TypeId GetInstanceTypeId () const;
    virtual uint32_t GetSerializedSize (void) const;
    virtual void Serialize (Buffer::Iterator start) const;
    virtual uint32_t Deserialize (Buffer::Iterator start); 
    virtual void Print (std::ostream &os) const;
    const uint32_t GetSenderId() const;
    void SetSenderId(const uint32_t SId);
    const uint32_t GetMsgId() const;
    void SetMsgId(const uint32_t MId);
  
			
	private:
	Vector m_position;
    Vector m_speed;
    uint32_t SenderId;
    uint32_t MsgId;
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
	return (6*sizeof(double)+2*sizeof(uint32_t)); // x and y coordinates both are doubles, IDs are uint32_t
}

void loc_header::Serialize (Buffer::Iterator start) const // from Adeel cam header
{
	//NS_LOG_FUNCTION (this << &start);
	//Buffer::Iterator i = start;
	start.WriteHtonU64 (ceil(m_position.x*1000)); //position doubles are 8 bytes
	start.WriteHtonU64 (ceil(m_position.y*1000));
	start.WriteHtonU64 (ceil(m_position.z*1000));

    start.WriteHtonU64 (ceil(m_speed.x*1000)); //velocity doubles are 8 bytes
	start.WriteHtonU64 (ceil(m_speed.y*1000));
	start.WriteHtonU64 (ceil(m_speed.z*1000));

    start.WriteHtonU32 (SenderId);
    start.WriteHtonU32 (MsgId);
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
    MsgId = start.ReadNtohU32();
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
    os << "Packet "<<MsgId<<" sent by Node "<<SenderId<<std::endl; 
    //" from " << "( " << m_position.x << ", " << m_position.y << ", " << m_position.z << " )" << std::endl;
}

void loc_header::SetSenderId(const uint32_t SId)
{
    SenderId = SId;
}


const uint32_t loc_header::GetSenderId() const
{
    return SenderId;
}

void loc_header::SetMsgId(const uint32_t MId)
{
    MsgId = MId;
}

const uint32_t loc_header::GetMsgId() const
{
    return MsgId;
}

void ReceivePacket (Ptr<Socket> socket)
{
  Ptr<Packet> packet;
  Time rcv;
  Time sqhd;
  Time total_delay;
  while (packet = socket->Recv ())
  {  

    // dq_header n_header;
    // packet->RemoveHeader(n_header);
    // uint64_t dqtime = n_header.GetDequeueTime();
    SeqTsHeader seqTsx; 
    packet->RemoveHeader(seqTsx);
    rcv = Simulator::Now();
    sqhd = seqTsx.GetTs();
    total_delay = (rcv - sqhd); //total delay calculation
    loc_header new_header;
    packet->RemoveHeader(new_header);
    // Vector old_position = new_header.GetLocation();
    // Vector old_speed = new_header.GetSpeed();
    uint32_t SenderId = new_header.GetSenderId();
    uint32_t MsgId = new_header.GetMsgId();
    Ptr<Node> Sender_Node = NodeList::GetNode (SenderId);
    // std::cout<<"Packet " << MsgId <<" sent by node " <<SenderId<< " and received by node "<< socket->GetNode()->GetId()<<" at "<<rcv<<", Packet generated at "<<sqhd<<" and overall delay for this packet is "<<total_delay<<std::endl;
    std::cout<<"Packet " << MsgId <<" sent by node " <<SenderId<< " and received by node "<< socket->GetNode()->GetId()<<" at "<<rcv<<", Packet generated at "<<sqhd<<",  and overall delay for this packet is "<<total_delay<<std::endl;
    

    // double xx; double yy; double zz;
    Ptr<MobilityModel> mob1 = Sender_Node->GetObject<MobilityModel>();
    // xx = mob1->GetPosition().x; yy = mob1->GetPosition().y; zz = mob1->GetPosition().z;
    // double xx_speed; double yy_speed; double zz_speed;
    // xx_speed = mob1->GetVelocity().x; yy_speed = mob1->GetVelocity().y; zz_speed = mob1->GetVelocity().z;

    // std::cout<<"Old position : "<<old_position.x<<", "<<old_position.y<<", "<<old_position.z<<std::endl;
    // std::cout<<"Actual positions are : "<<xx<<", "<<yy<<", "<<zz<<"\n"<<std::endl;

    // std::cout<<"Old Speed : "<<old_speed.x<<", "<<old_speed.y<<", "<<old_speed.z<<std::endl;
    // std::cout<<"Actual Speed are : "<<xx_speed<<", "<<yy_speed<<", "<<zz_speed<<"\n"<<std::endl;
  }
}


  // Simulator::ScheduleWithContext (source->GetNode ()->GetId (),
  //                                 NanoSeconds (1000000000), &GenerateTraffic,
  //                                 source, packetSize, numPackets, interPacketInterval);



uint32_t msgcount = 0;
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
    new_header.SetMsgId(++msgcount);
    p->AddHeader(new_header);
    SeqTsHeader seqTs;
    seqTs.SetSeq (pktCount);
    p->AddHeader (seqTs); //confirm if size of seqTs is 12 or not??
    // std::cout<<"Packet about to be sent"<<std::endl;
    socket->Send(p); // means sent for Contention
    // std::cout<<"packet sent for contention"<<std::endl;
    Simulator::Schedule (pktInterval, &GenerateTraffic, socket, pktSize, pktCount - 1, pktInterval);
    }
  else
    {
      socket->Close ();
    }
}

void EnqueueTrace(std::string context, Ptr<const WifiMacQueueItem> item) 
{
  Time entry_time = Simulator::Now();
//   std::cout<<"Packet entered queue at "<<entry_time<<std::endl;
  // std::cout<<"Packet's generation time = "<<item->GetTimeStamp()<<std::endl;
  // packet enters queue directly after generation, no lag

}

// void AddDequeueTime(Ptr<Packet> p)
// {
//     uint64_t now_int = (Simulator::Now()).GetInteger();
//     dq_header new_header;
//     new_header.SetDequeueTime(now_int);
//     p->AddHeader (new_header);
// }

void DequeueTrace(std::string context, Ptr<const WifiMacQueueItem> item) 
{
    Time delay = Simulator::Now() - item->GetTimeStamp();

    Ptr<const Packet> p = item->GetPacket();
    SeqTsHeader n_header;
    // n_header
    p->PeekHeader(n_header);
    loc_header new_header;
    p->PeekHeader(new_header);
    new_header.Print(std::cout);
    std::cout<<" dequeued at "<<Simulator::Now()<<std::endl;
    // AddDequeueTime(p);
    // std::cout<<"New Header addeed"<<std::endl;
    // uint64_t now_int = (Simulator::Now()).GetInteger();
    // new_header.SetDequeueTime(now_int);
    // p->AddHeader (new_header);
//   std::cout<<"Inside Deque Current Time = "<<Simulator::Now() << "," << "Packet Generation Time = "<< item->GetTimeStamp() <<", Queueing Delay = "<<delay<<std::endl;
}

int main (int argc, char *argv[])
{
  // LogComponentEnable("Txop", LOG_ALL);
  // LogComponentEnable("Simulator", LOG_ALL);
  // LogComponentEnable("WifiMacQueue", LOG_ALL);
  // LogComponentEnable("Queue", LOG_ALL);
  // LogComponentEnable("Socket", LOG_ALL);
  // LogComponentEnable("WifiNetDevice", LOG_ALL);
  
  std::string phyMode ("OfdmRate3MbpsBW10MHz");
  uint32_t packetSize = 1000; // bytes
  uint32_t numPackets = 10;
  double interval = 10;
  bool verbose = false;

  CommandLine cmd;

  cmd.AddValue ("phyMode", "Wifi Phy mode", phyMode);
  cmd.AddValue ("packetSize", "size of application packet sent", packetSize);
  cmd.AddValue ("numPackets", "number of packets generated", numPackets);
  cmd.AddValue ("interval", "interval (seconds) between packets", interval);
  cmd.AddValue ("verbose", "turn on all WifiNetDevice log components", verbose);
  cmd.Parse (argc, argv);
  // Convert to time object
  Time interPacketInterval = NanoSeconds (interval);

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

  // change all the 5 queues to size 1 and DropOldest policy


  Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/$ns3::OcbWifiMac/Txop/Queue/MaxSize", StringValue ("5p"));
  Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/$ns3::OcbWifiMac/VO_Txop/Queue/MaxSize", StringValue ("5p"));
  Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/$ns3::OcbWifiMac/VI_Txop/Queue/MaxSize", StringValue ("5p"));
  Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/$ns3::OcbWifiMac/BE_Txop/Queue/MaxSize", StringValue ("5p"));
  Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/$ns3::OcbWifiMac/BK_Txop/Queue/MaxSize", StringValue ("5p"));

  // Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/$ns3::OcbWifiMac/Txop/Queue/DropPolicy", StringValue("DropOldest"));
  // Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/$ns3::OcbWifiMac/VO_Txop/Queue/DropPolicy", StringValue("DropOldest"));
  // Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/$ns3::OcbWifiMac/VI_Txop/Queue/DropPolicy", StringValue("DropOldest"));
  // Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/$ns3::OcbWifiMac/BE_Txop/Queue/DropPolicy", StringValue("DropOldest"));
  // Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/$ns3::OcbWifiMac/BK_Txop/Queue/DropPolicy", StringValue("DropOldest"));

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

  TrafficControlHelper tch;
  tch.Uninstall (devices);

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

//   Ptr<UniformRandomVariable> rv = CreateObject<UniformRandomVariable> ();
//   rv->SetAttribute ("Min", DoubleValue (0.0));
//   rv->SetAttribute ("Max", DoubleValue (10.0)); //on what basis is this max value taken??

  Simulator::ScheduleWithContext (source->GetNode ()->GetId (),
                                  NanoSeconds (1000000000), &GenerateTraffic,
                                  source, packetSize, numPackets, interPacketInterval);

  AnimationInterface anim("first.xml");

  // Config::SetDefault ("ns3::ConfigStore::Filename", StringValue ("output-attributes.txt"));
  // Config::SetDefault ("ns3::ConfigStore::FileFormat", StringValue ("RawText"));
  // Config::SetDefault ("ns3::ConfigStore::Mode", StringValue ("Save"));
  // ConfigStore outputConfig;
  // outputConfig.ConfigureAttributes ();

  Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacRxDrop", MakeCallback(&MacRxDrop));
  Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacTxDrop", MakeCallback(&MacTxDrop));
  Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyRxDrop", MakeCallback(&PhyRxDrop));
  Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyTxDrop", MakeCallback(&PhyTxDrop));
  // double TotalTime = 5.0;
  // Simulator::Stop (Seconds (TotalTime));

  Simulator::Run ();
  Simulator::Destroy ();
  std::cout<<collisions_MacTxDrop<<" "<<collisions_PhyTxDrop<<" "<<collisions_PhyRxDrop<<" "<<collisions_MacRxDrop<<std::endl;
  return 0;
}