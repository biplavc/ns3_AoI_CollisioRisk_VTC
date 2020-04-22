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
#include <cmath>
#include <numeric>
#include "ns3/constant-velocity-mobility-model.h"
#include <cstdio>
#include <string>
#include <iterator>



using namespace ns3;

int m_nodeSpeed;
int m_nodePause;

NS_LOG_COMPONENT_DEFINE ("Header Added");



class loc_header : public Header 
{
	public:
    loc_header();
     //virtual ~loc_header();
    void SetLocation(const Vector location);
    const Vector GetLocation() const;
    static TypeId GetTypeId (void);
    TypeId GetInstanceTypeId () const;
    virtual uint32_t GetSerializedSize (void) const;
    virtual void Serialize (Buffer::Iterator start) const;
    virtual uint32_t Deserialize (Buffer::Iterator start); 
    virtual void Print (std::ostream &os) const;
    const uint32_t GetSenderId() const;
    void SetSenderId(const uint32_t Id);
    void SetMsgId (const uint32_t MsgId);
    const uint32_t GetMsgId() const;
			
	private:
	Vector m_position;
    uint32_t SenderId;
    uint32_t MID; //MID is message ID
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
	return (3*sizeof(double)+2*sizeof(uint32_t)); // x and y coordinates both are doubles, IDs are uint32_t
}

void loc_header::Serialize (Buffer::Iterator start) const // from Adeel cam header
{
	//NS_LOG_FUNCTION (this << &start);
	//Buffer::Iterator i = start;
	start.WriteHtonU64 (ceil(m_position.x*1000)); // doubles are 8 bytes
	start.WriteHtonU64 (ceil(m_position.y*1000));
	start.WriteHtonU64 (ceil(m_position.z*1000));
    start.WriteHtonU32 (SenderId);
    start.WriteHtonU32(MID);
}

uint32_t loc_header::Deserialize (Buffer::Iterator start)   // from Adeel cam header
{
	m_position.x = start.ReadNtohU64()/1000; // doubles are 8 bytes
	m_position.y = start.ReadNtohU64()/1000;
	m_position.z = start.ReadNtohU64()/1000;
    SenderId = start.ReadNtohU32();
    MID = start.ReadNtohU32();
    return GetSerializedSize ();
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

void loc_header::SetMsgId (const uint32_t MsgId)
{
    MID = MsgId;
}

const uint32_t loc_header::GetMsgId() const
{
    return MID;
}

double x_error; double y_error; double z_error; double total_error;
int32_t total_received_packets = 0;
void ReceivePacket (Ptr<Socket> socket)
{
  Ptr<Packet> packet;
  Time rcv;
  Time sqhd;
  Time total_delay;
  while (packet = socket->Recv ())
  {
    total_received_packets++;
    SeqTsHeader seqTsx; 
    packet->RemoveHeader(seqTsx);
    rcv = Simulator::Now();
    sqhd = seqTsx.GetTs();
    total_delay = (rcv - sqhd); //total delay calculation
    loc_header new_header;
    packet->RemoveHeader(new_header);
    Vector old_position = new_header.GetLocation();
    uint32_t SenderId = new_header.GetSenderId();
    uint32_t MsgId = new_header.GetMsgId();
    Ptr<Node> Sender_Node = NodeList::GetNode (SenderId);
    //std::cout<<"Packet "<< MsgId<<" sent by node " <<SenderId<< " and received by node "<< socket->GetNode()->GetId()<<" at "<<rcv<<", Packet generated at "<<sqhd<<" and overall delay for this packet is "<<total_delay<<std::endl;
    double xx; double yy; double zz;
    Ptr<MobilityModel> mob1 = Sender_Node->GetObject<MobilityModel>();
    xx = mob1->GetPosition().x; yy = mob1->GetPosition().y; zz = mob1->GetPosition().z;
    // std::cout<<"Old position : "<<old_position.x<<", "<<old_position.y<<", "<<old_position.z<<std::endl;
    // std::cout<<"Actual positions are : "<<xx<<", "<<yy<<", "<<zz<<"\n"<<std::endl;
    x_error = old_position.x - xx; y_error = old_position.y - yy; z_error = old_position.z - zz; 
    total_error = sqrt(pow(x_error, 2) + pow(y_error, 2) + pow(z_error, 2));
    // int k = 0;
    // int filename_append = (k+1)*20;
    // std::stringstream ss;
    // ss << filename_append;
    // std::string str = ss.str();
    // std::string file_name ("output" + str + ".txt"); 
    // freopen(file_name.c_str(),"a",stdout); //https://stackoverflow.com/questions/21589353/cannot-convert-stdbasic-stringchar-to-const-char-for-argument-1-to-i
    std::cout<<MsgId<<" "<<SenderId<<" "<<socket->GetNode()->GetId()<<" "<<total_error<<" "<<sqhd.GetInteger()<<" "<<rcv.GetInteger()<<" "<<total_delay.GetInteger()<<"\n"<<std::endl;
    //std::cout<<MsgId<<" "<<SenderId<<" "<<socket->GetNode()->GetId()<<" "<<total_error<<" "<<total_delay<<"\n"<<std::endl;
    // NS_LOG_UNCOND ("Received one packet!");
  }
}

uint32_t MsgCount = 0; // will be used as MsgId to ID the packets from the tx side

int32_t total_transmitted_packets = 0; //won't gve the totl packets transmitted, will give the number of times this function was called = nodes*packets (200*3)
static void GenerateTraffic (Ptr<Socket> socket, uint32_t pktSize,
                             uint32_t pktCount, Time pktInterval )
{
  if (pktCount > 0)
    {
    total_transmitted_packets++;
    Ptr<Packet> p = Create<Packet> (pktSize-(12+24+8)); //12 bytes seqts, 24 bytes loc_header and 4+4 bytes IDs
    loc_header new_header;
    Ptr<MobilityModel> mob = socket->GetNode()->GetObject<MobilityModel>(); //https://stackoverflow.com/questions/57290850/how-to-get-and-print-mobile-nodes-position-in-aodv-ns3
    double x_old = mob->GetPosition().x; double y_old = mob->GetPosition().y; double z_old = mob->GetPosition().z;
    Vector3D vect(x_old, y_old, z_old);
    new_header.SetLocation(vect);
    new_header.SetSenderId(socket->GetNode()->GetId());
    new_header.SetMsgId(++MsgCount);
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
    MsgCount = 0;
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
  std::string phyMode ("OfdmRate6MbpsBW10MHz");
  uint32_t packetSize = 1000; // bytes
  uint32_t numPackets = 5; //packets to be sent per node
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

//   int arr[] = {20, 40, 60, 80, 100, 120, 140, 160, 180, 200}; //number of nodes in each scenario
  int arr[] = {20};
  int mm = (sizeof(arr)/sizeof(*arr));

  for (int m = 0; m < mm; m++)
  {

    NodeContainer c;
    int num_nodes = arr[m];
    c.Create (num_nodes);

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
    int64_t m_streamIndex=0;

    ObjectFactory pos;
    pos.SetTypeId ("ns3::RandomBoxPositionAllocator");
    pos.Set ("X", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=400.0]"));
    pos.Set ("Y", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=6.0]"));
    // we need antenna height uniform [1.0 .. 2.0] for loss model
    pos.Set ("Z", StringValue ("ns3::UniformRandomVariable[Min=1.0|Max=2.0]"));

    Ptr<PositionAllocator> taPositionAlloc = pos.Create ()->GetObject<PositionAllocator> ();
    m_streamIndex += taPositionAlloc->AssignStreams (m_streamIndex); //meaning?

    mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mobility.SetPositionAllocator (taPositionAlloc);
    mobility.Install (c);
    Ptr<NormalRandomVariable> var=CreateObject<NormalRandomVariable> ();;
    //Ptr<UniformRandomVariable> var = CreateObject<UniformRandomVariable> ();
    int64_t stream = 2;
    var->SetStream (stream);

    for(int i=0;i<num_nodes; i++)
    {
      Ptr<ConstantVelocityMobilityModel> mob = c.Get(i)-> GetObject<ConstantVelocityMobilityModel>();
      mob->SetVelocity(Vector(20 + sqrt(5)*var->GetValue (), 0.0, 0.0));
      //Vector posm = mob->GetPosition (); // Get position
      //Vector vel = mob->GetVelocity (); // Get velocity
      //std::cout << Simulator::Now () << " POS: x=" << posm.x << ", y=" << posm.y
      //  << ", z=" << posm.z << "; VEL:" << vel.x << ", y=" << vel.y
      //        << ", z=" << vel.z << std::endl;
    }

    m_streamIndex += mobility.AssignStreams (c, m_streamIndex);

    int nSinks = num_nodes;
    double TotalTime = 5.0;

    InternetStackHelper internet;
    internet.Install (c);

    Ipv4AddressHelper addressAdhoc;
    addressAdhoc.SetBase ("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer adhocInterfaces;
    adhocInterfaces = addressAdhoc.Assign (devices);
    TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");

    Ptr<UniformRandomVariable> rv = CreateObject<UniformRandomVariable> ();
    rv->SetAttribute ("Min", DoubleValue (0.0));
    rv->SetAttribute ("Max", DoubleValue (0.1)); //on what basis is this max value taken??
    for (int i = 0; i < nSinks; i++)
      {
        //Ptr<Socket> sink = SetupPacketReceive (adhocInterfaces.GetAddress (i), nodes.Get (i));
        Ptr<Socket> recvSink = Socket::CreateSocket (c.Get (i), tid);
        InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), 80);
        recvSink->Bind (local);
        recvSink->SetRecvCallback (MakeCallback (&ReceivePacket));
        /*if(i==0)
        { // point of this loop? sets the power level of 1 device ??
          Ptr<NetDevice> device_0 = devices.Get(i);
          Ptr<WifiNetDevice> wifiDevice_0 = DynamicCast<WifiNetDevice> (device_0);
          Ptr<YansWifiPhy> phy = DynamicCast<YansWifiPhy>(wifiDevice_0->GetPhy());
          phy->SetTxPowerStart(20); //min available tx power level in dBm
          phy->SetTxPowerEnd(20); //maximum available transmission power level (dBm)
        } */
        Ptr<Socket> source = Socket::CreateSocket (c.Get (i), tid);
        InetSocketAddress remote = InetSocketAddress (Ipv4Address ("255.255.255.255"), 80);
        source->SetAllowBroadcast (true);
        source->Connect (remote);

        std::string enqueue_path= "/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/*/$ns3::OcbWifiMac/*/Queue/Enqueue";
        std::string dequeue_path= "/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/*/$ns3::OcbWifiMac/*/Queue/Dequeue";
        Config::Connect (enqueue_path, MakeCallback (&EnqueueTrace));
        Config::Connect (dequeue_path, MakeCallback (&DequeueTrace));  
        //std::cout << "/* in main loop about to enter schedule with context */" << '\n';
        Simulator::ScheduleWithContext (source->GetNode ()->GetId (),
                                        Seconds (1.0+rv->GetValue()), &GenerateTraffic,
                                        source, packetSize, numPackets, interPacketInterval); //without rv, no packets delivered
      }

    // MAC tracing starts
    Simulator::Stop (Seconds (TotalTime));

    AnimationInterface anim("first.xml");

    int filename_append = arr[m];
    std::stringstream ss;
    ss << filename_append;
    std::string str = ss.str();
    std::string file_name ("GaussianXSpeed_X4K" + str + ".txt"); 
    freopen(file_name.c_str(),"a",stdout);

    Simulator::Run ();   
    Simulator::Destroy ();
  //double PDR = 1.000000*(total_received_packets)/((num_nodes-1)*numPackets*num_nodes);
  //std::cout<<"PDR = "<<PDR<<", Total Packets Received = " <<total_received_packets<<std::endl;
  }
  
  return 0;
}