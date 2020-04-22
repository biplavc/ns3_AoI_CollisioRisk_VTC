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

NS_LOG_COMPONENT_DEFINE ("HeaderAdded");

class loc_header : public Header 
{
	public:
    loc_header();
     //virtual ~loc_header();
    void SetLocation(const Vector location);
    const Vector GetLocation() const;// const Vector & GetPosition () const; the meaning of & here?
		static TypeId GetTypeId (void);
		TypeId GetInstanceTypeId () const;
		virtual uint32_t GetSerializedSize (void) const;
		virtual void Serialize (Buffer::Iterator start) const;
    virtual uint32_t Deserialize (Buffer::Iterator start); 
		//virtual void Print (std::ostream &os) const;
    virtual void Print (std::ostream &os) const;
			
	private:
		Vector m_position;
};

NS_OBJECT_ENSURE_REGISTERED (loc_header);

loc_header::loc_header()
{
} // why empty constructor needed?? 
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

//virtual TypeId GetInstanceTypeId (void) const;
uint32_t loc_header::GetSerializedSize (void) const
{
	return (3*sizeof(double)); // x and y coordinates both are doubles
}

void loc_header::Serialize (Buffer::Iterator start) const // from Adeel cam header
{
	//NS_LOG_FUNCTION (this << &start);
	//Buffer::Iterator i = start;
	start.WriteHtonU64 (ceil(m_position.x*1000)); // doubles are 8 bytes
	start.WriteHtonU64 (ceil(m_position.y*1000));
	start.WriteHtonU64 (ceil(m_position.z*1000));
}

//void Deserialize (Buffer::Iterator start)
uint32_t loc_header::Deserialize (Buffer::Iterator start)   // from Adeel cam header
{
	//NS_LOG_FUNCTION(this << &start);
	//Buffer::Iterator i = start;
	m_position.x = start.ReadNtohU64()/1000; // doubles are 8 bytes
	m_position.y = start.ReadNtohU64()/1000;
	m_position.z = start.ReadNtohU64()/1000;
    return GetSerializedSize ();
}

void loc_header::Print(std::ostream &os) const 
{
  os << "Position (x, y, z) " << "( " << m_position.x << ", " << m_position.y << ", " << m_position.z << " )" << std::endl;
    //os << "Position (x, y) " << "( " << m_position.x << ", " << m_position.y << " )" << std::endl;
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
      // Time rcv = Simulator::Now().GetNanoSeconds(); //time the packet was received
      rcv = Simulator::Now();
      sqhd = seqTsx.GetTs();
      // Time sqhd = seqTsx.GetTs().GetNanoSeconds(); //timestamp in the header, created when the packet was generated
      total_delay = (rcv - sqhd); //total delay calculation
      std::cout<<"Packet received at "<<rcv<<", Packet generated at "<<sqhd<<" and overall delay for this packet is "<<total_delay<<std::endl;
      loc_header new_header;
      packet->RemoveHeader(new_header);
      new_header.Print(std::cout);

      Ptr<MobilityModel> mob = socket->GetNode()->GetObject<MobilityModel>();
      double x_actual = mob->GetPosition().x; double y_actual = mob->GetPosition().y; double z_actual = mob->GetPosition().z;
      std::cout<<"actual positions are "<<x_actual<<", "<<y_actual<<", "<<z_actual<<std::endl;

      NS_LOG_UNCOND ("Received one packet!");
      std::cout<< "\n"<<std::endl;
    }
  //xx = rcv - senttime;
}

static void GenerateTraffic (Ptr<Socket> socket, uint32_t pktSize,
                             uint32_t pktCount, Time pktInterval )
{
  if (pktCount > 0)
    {
      
      Ptr<Packet> p = Create<Packet> (pktSize-(12+24)); //12 bytes seqts and 24 bytes loc_header
      
      loc_header new_header;
      Ptr<MobilityModel> mob = socket->GetNode()->GetObject<MobilityModel>(); //https://stackoverflow.com/questions/57290850/how-to-get-and-print-mobile-nodes-position-in-aodv-ns3
      
      double x_old = mob->GetPosition().x; double y_old = mob->GetPosition().y; double z_old = mob->GetPosition().z;
      Vector3D vect(x_old, y_old, z_old);
      new_header.SetLocation(vect);
      p->AddHeader(new_header);
      SeqTsHeader seqTs;
      seqTs.SetSeq (pktCount);
      p->AddHeader (seqTs); //confirm if size of seqTs is 12 or not??

      socket->Send(p); // means sent for Contention
      //std::cout<< "\nSending "<< pktCount  << " packet at time" << Simulator::Now() <<" ! \n"<<std::endl; // means packet generated
      Simulator::Schedule (pktInterval, &GenerateTraffic, socket, pktSize, pktCount - 1, pktInterval);
    }
  else
    {
      socket->Close ();
    }
}

void EnqueueTrace(std::string context, Ptr<const WifiMacQueueItem> item) {
  Time entry_time = Simulator::Now();
  //std::cout <<"Packet generated at "<< item->GetTimeStamp()<< " enters queue at  : " << entry_time << std::endl;
  //verified that packet enters queue as soon as they are generated
}

void DequeueTrace(std::string context, Ptr<const WifiMacQueueItem> item) {
  //what to do when the packet is dequeued. You can do something like
  Time delay = Simulator::Now() - item->GetTimeStamp();
  //std::cout<<"Inside Deque Current Time = "<<Simulator::Now() << "," << "Packet Generation Time = "<< item->GetTimeStamp() << std::endl;
  std::cout << "Queueing Delay : " << delay << std::endl;
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
  // ns-3 supports generate a pcap trace
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