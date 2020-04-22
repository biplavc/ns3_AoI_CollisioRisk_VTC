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

#include "ns3/ocb-wifi-mac.h"
#include "ns3/wifi-80211p-helper.h"
#include "ns3/wave-mac-helper.h"
#include "ns3/wave-bsm-helper.h"
#include "ns3/internet-module.h"

#include "ns3/tag.h"
#include "ns3/packet.h"
#include "ns3/uinteger.h"
#include <iostream>


#include "ns3/wifi-module.h"
#include "ns3/aodv-module.h"
#include "ns3/olsr-module.h"
#include "ns3/dsdv-module.h"
#include "ns3/dsr-module.h"
#include "ns3/applications-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("WifiSimpleOcb");

WaveBsmHelper m_waveBsmHelper;
int m_nodeSpeed;
int m_nodePause;

 class MyTag : public Tag
 {
 public:
  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const;
  virtual uint32_t GetSerializedSize (void) const;
  virtual void Serialize (TagBuffer i) const;
  virtual void Deserialize (TagBuffer i);
  virtual void Print (std::ostream &os) const;

  // these are our accessors to our tag structure
  /**
   * Set the tag value
   * \param value The tag value.
   */
  void SetSimpleValue (uint8_t value);
  /**
   * Get the tag value
   * \return the tag value.
   */
  uint32_t GetSimpleValue (void) const;
 private:
  uint32_t m_simpleValue;  //!< tag value
 };


 uint32_t m_phyTxPkts; ///< phy transmit packets
 uint32_t m_phyTxBytes; ///< phy transmit bytes

 //NS_LOG_COMPONENT_DEFINE ("WifiSimpleOcb");

 TypeId
 MyTag::GetTypeId (void)
 {
   static TypeId tid = TypeId ("ns3::MyTag")
     .SetParent<Tag> ()
     .AddConstructor<MyTag> ()
     .AddAttribute ("SimpleValue",
                    "A simple value",
                    EmptyAttributeValue (),
                    MakeUintegerAccessor (&MyTag::GetSimpleValue),
                    MakeUintegerChecker<uint32_t> ())
   ;
   return tid;
 }
 TypeId
 MyTag::GetInstanceTypeId (void) const
 {
   return GetTypeId ();
 }
 uint32_t
 MyTag::GetSerializedSize (void) const
 {
   return 4;
 }
 void
 MyTag::Serialize (TagBuffer i) const
 {
   i.WriteU32(m_simpleValue);
 }
 void
 MyTag::Deserialize (TagBuffer i)
 {
   m_simpleValue = i.ReadU32();
 }
 void
 MyTag::Print (std::ostream &os) const
 {
   os << "v=" << (uint32_t)m_simpleValue;
 }
 void
 MyTag::SetSimpleValue (uint8_t value)
 {
   m_simpleValue = value;
 }
 uint32_t
 MyTag::GetSimpleValue (void) const
 {
   return m_simpleValue;
 }

uint32_t MsgId = 0;
static void GenerateTraffic (Ptr<Socket> socket, uint32_t pktSize, uint32_t pktCount, Time pktInterval )
{
  //if (pktCount > 0)
   // {
	  Ptr<Packet> pkt = Create<Packet> (pktSize);
	  MyTag tag;
	  tag.SetSimpleValue (++MsgId);
	  pkt->AddPacketTag(tag);
	  Ptr<MobilityModel> mobility = socket->GetNode()->GetObject<MobilityModel>();
	  Vector pos = mobility->GetPosition();
      std::cout << Simulator::Now ().GetMilliSeconds() << " ,S," <<
      socket->GetNode()->GetId() << "," << tag.GetSimpleValue() << "," << pos.x << "," <<pos.y << "," <<pos.z << std::endl;
      socket->Send (pkt);
      Simulator::Schedule (pktInterval, &GenerateTraffic, socket, pktSize, pktCount - 1, pktInterval);
}

void ReceivePacket (Ptr<Socket> socket)
{
  Ptr<Packet> p;
  while ((p = socket->Recv ())) // Read a single packet from the socket.
    {
	  MyTag tag;
	  p->PeekPacketTag (tag);
	  Ptr<MobilityModel> mobility = socket->GetNode()->GetObject<MobilityModel>();
	  Vector pos = mobility->GetPosition();
      std::cout << Simulator::Now ().GetMilliSeconds() << " ,R ID," <<
      socket->GetNode()->GetId() << ", value" << tag.GetSimpleValue() << ",x" << pos.x << ",y" <<pos.y << ",z" <<pos.z << std::endl;
    }
}

void CourseChange (std::ostream *os, std::string context, Ptr<const MobilityModel> mobility)
{
  Vector pos = mobility->GetPosition (); // Get position
  std::cout << "/* in CourseChange */" << std::endl;
  Vector vel = mobility->GetVelocity (); // Get velocity

  pos.z = 1.5;

  int nodeId = mobility->GetObject<Node> ()->GetId ();
  double t = (Simulator::Now ()).GetSeconds ();
  if (t >= 1.0)
    {
      WaveBsmHelper::GetNodesMoving ()[nodeId] = 1;
    }

  NS_LOG_UNCOND ("Changing pos for node=" << nodeId << " at " << Simulator::Now () );

  // Prints position and velocities
  std::cout<< Simulator::Now () << nodeId << " POS: x=" << pos.x << ", y=" << pos.y
      << ", z=" << pos.z << "; VEL:" << vel.x << ", y=" << vel.y
      << ", z=" << vel.z << std::endl;
}


int main (int argc, char *argv[])
{
    std::string phyMode ("OfdmRate6MbpsBW10MHz");
    uint32_t packetSize = 1000; // bytes
    uint32_t numPackets = 10;
    double interval = 0.1; // seconds
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

    int num_nodes=3;

    NodeContainer nodes;
    nodes.Create (num_nodes);

    // The below set of helpers will help us to put together the wifi NICs we want
    YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();
    double m_TxGain=5.0;
    std::cout << "/* txgain */" << m_TxGain<<'\n';

    wifiPhy.Set ("TxGain", DoubleValue (m_TxGain) );
    //m_TxGain=wifiPhy.GetPowerDbm();
    //wifiPhy.GetTxGain();
    //wavePhy.Set ("RxGain", DoubleValue (m_RxGain) );
    //wavePhy.Set ("Frequency", UintegerValue (m_freq));
    //wavePhy.Set ("EnergyDetectionThreshold", DoubleValue (m_sensitivity));
    //wavePhy.Set ("CcaMode1Threshold",DoubleValue(m_cca_sensitivity));
    YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
    Ptr<YansWifiChannel> channel = wifiChannel.Create ();
    wifiPhy.SetChannel (channel);
    // ns-3 supports generate a pcap trace
    wifiPhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11);
    NqosWaveMacHelper wifi80211pMac = NqosWaveMacHelper::Default ();
    Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default ();
    if (verbose)
        {
        wifi80211p.EnableLogComponents ();      // Turn on all Wifi 802.11p logging
        }

    wifi80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                        "DataMode",StringValue (phyMode),
                                        "ControlMode",StringValue (phyMode));

    NetDeviceContainer devices = wifi80211p.Install (wifiPhy, wifi80211pMac, nodes);

    // Tracing
    wifiPhy.EnablePcap ("wave-simple-80211p", devices);

    MobilityHelper mobility;
    int64_t m_streamIndex=0;

  
    ObjectFactory pos;
    pos.SetTypeId ("ns3::RandomBoxPositionAllocator");
    pos.Set ("X", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=150.0]"));
    pos.Set ("Y", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=300.0]"));
    // we need antenna height uniform [1.0 .. 2.0] for loss model
    pos.Set ("Z", StringValue ("ns3::UniformRandomVariable[Min=1.0|Max=1.0]"));

    Ptr<PositionAllocator> taPositionAlloc = pos.Create ()->GetObject<PositionAllocator> (); //??
    m_streamIndex += taPositionAlloc->AssignStreams (m_streamIndex); //???
    m_nodeSpeed=20;
    std::stringstream ssSpeed;
    ssSpeed << "ns3::UniformRandomVariable[Min=0.0|Max=" << m_nodeSpeed << "]";
    std::cout << ssSpeed.str() << std::endl;
    m_nodePause=0;
    std::stringstream ssPause;
    ssPause << "ns3::ConstantRandomVariable[Constant=" << m_nodePause << "]";


    mobility.SetMobilityModel ("ns3::RandomWaypointMobilityModel",
                                    "Speed", StringValue (ssSpeed.str ()),
                                    "Pause", StringValue (ssPause.str ()),
                                    "PositionAllocator", PointerValue (taPositionAlloc));
    mobility.SetPositionAllocator (taPositionAlloc);
    mobility.Install (nodes);
    m_streamIndex += mobility.AssignStreams (nodes, m_streamIndex);
    std::ofstream m_os; 

    InternetStackHelper internet;
    internet.Install (nodes);

    int nSinks=num_nodes;
    double TotalTime = 2.0;
    Ipv4AddressHelper addressAdhoc;
    addressAdhoc.SetBase ("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer adhocInterfaces;
    adhocInterfaces = addressAdhoc.Assign (devices);
    TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");

    Ptr<UniformRandomVariable> rv = CreateObject<UniformRandomVariable> ();
    rv->SetAttribute ("Min", DoubleValue (0.0));
    rv->SetAttribute ("Max", DoubleValue (0.05));
    for (int i = 0; i < nSinks; i++)
        {
        //Ptr<Socket> sink = SetupPacketReceive (adhocInterfaces.GetAddress (i), nodes.Get (i));
        Ptr<Socket> recvSink = Socket::CreateSocket (nodes.Get (i), tid);
        InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), 80);
        recvSink->Bind (local);
        recvSink->SetRecvCallback (MakeCallback (&ReceivePacket));
        if(i==0)
        {
            Ptr<NetDevice> device_0 = devices.Get(i);
            Ptr<WifiNetDevice> wifiDevice_0 = DynamicCast<WifiNetDevice> (device_0);
            Ptr<YansWifiPhy> phy = DynamicCast<YansWifiPhy>(wifiDevice_0->GetPhy());
            phy->SetTxPowerStart(20);
            phy->SetTxPowerEnd(20);
        }
        Ptr<Socket> source = Socket::CreateSocket (nodes.Get (i), tid);
        InetSocketAddress remote = InetSocketAddress (Ipv4Address ("255.255.255.255"), 80);
        source->SetAllowBroadcast (true);
        source->Connect (remote);
        std::cout << "/* in main loop about to enter schedule with context */" << '\n';
        Simulator::ScheduleWithContext (source->GetNode ()->GetId (),
                                        Seconds (1.0+rv->GetValue()), &GenerateTraffic,
                                        source, packetSize, numPackets, interPacketInterval);
        }

    /*
    End of Avik's code
    */
    Simulator::Stop (Seconds (TotalTime));

    Simulator::Run ();
    Simulator::Destroy ();

    return 0;
}