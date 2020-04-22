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
#include "ns3/queue-size.h"
#include "ns3/config-store-module.h"
#include "ns3/config.h"
#include "ns3/traffic-control-helper.h"
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/ns2-mobility-helper.h"
#include "ns3/wave-bsm-helper.h"


  

using namespace ns3;

// Time::SetResolution (Time::NS);
// Time::SetResolution(Time::FS);

int collisions_MacTxDrop;
int collisions_PhyTxDrop;
int collisions_PhyRxDrop;
int collisions_MacRxDrop;

void
MacTxDrop(Ptr<const Packet> p)
{
  // NS_LOG_INFO("Packet Drop");
//   MacTxDropCount++;
  collisions_MacTxDrop  = collisions_MacTxDrop + 1;
}

void
PhyTxDrop(Ptr<const Packet> p)
{
  // NS_LOG_INFO("Packet Drop");
//   PhyTxDropCount++;
  collisions_PhyTxDrop  = collisions_PhyTxDrop + 1;
}
void
PhyRxDrop(Ptr<const Packet> p)
{
  // NS_LOG_INFO("Packet Drop");
//   PhyRxDropCount++;
  collisions_PhyRxDrop  = collisions_PhyRxDrop + 1;
}
void
MacRxDrop(Ptr<const Packet> p)
{
  // NS_LOG_INFO("Packet Drop");
//   PhyRxDropCount++;
  collisions_MacRxDrop  = collisions_MacRxDrop + 1;
}

NS_LOG_COMPONENT_DEFINE ("Header Added");



class loc_header : public Header 
{
	public:
    loc_header();
     //virtual ~loc_header();
    void SetLocation(const Vector location);
    void SetSpeed(const Vector velocity);
    const Vector GetLocation() const;
    const Vector GetSpeed() const;
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
    const uint32_t GetXvelID() const;
    const uint32_t GetYvelID() const;
    void SetXvelID(const uint32_t XvelID); // 1 for positive x-vel and 0 for negative x-vel
    void SetYvelID(const uint32_t YvelID); // 1 for positive y-vel and 0 for negative y-vel
 
	private:
	  Vector m_position; // 3 double here
    Vector m_velocity; // 3 double here
    uint32_t SenderId; // one double
    uint32_t MID; //MID is message ID, one double
    uint32_t XvelID; // 1 for positive values and 0 for negative values, one double
    uint32_t YvelID; // 1 for positive values and 0 for negative values, one double
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

TypeId loc_header::GetTypeId(void)
{
	static TypeId tid = TypeId ("ns3::loc_Header").SetParent<Header>().AddConstructor<loc_header>();
	return tid;
}

uint32_t loc_header::GetSerializedSize (void) const
{
	return (2*3*sizeof(double)+4*sizeof(uint32_t)); // x and y coordinates (3 each) both are doubles, IDs are uint32_t. Sender ID, MEssage ID, 2 IDs for x and y velocity signs
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
start.WriteHtonU32 (SenderId);
start.WriteHtonU32(MID);
start.WriteHtonU32 (XvelID);
start.WriteHtonU32 (YvelID);
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


const uint32_t loc_header::GetSenderId() const
{
    return SenderId;
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

double x_error; double y_error; double z_error; double total_error;
int32_t total_received_packets = 0;
void ReceivePacket (Ptr<Socket> socket)
{
  Ptr<Packet> packet;
  Time rcv;
  Time sqhd;
  Time total_delay;
  while ((packet = socket->Recv ()) && (Simulator::Now() > Seconds(360)))// start logging after 1865 seconds as in SUMO, 200 vehicles enter simulation after 173 seconds. For 50 vehicles, it is 173.
  {
    total_received_packets++;
    SeqTsHeader seqTsx; 
    packet->RemoveHeader(seqTsx);
    rcv = (Simulator::Now()); // time of reception
    sqhd = (seqTsx.GetTs()); //time stamp of when packet generated
    // std::cerr<<"rcv = "<<rcv<<", sqhd = "<<sqhd<<std::endl;
    total_delay = (rcv - sqhd); //total delay calculation
    loc_header new_header;
    packet->RemoveHeader(new_header);
    Vector old_position = new_header.GetLocation();
    Vector old_velocity = new_header.GetSpeed();
    // std::cout<<"the speeds are: "<<old_velocity<<std::endl;
    uint32_t SenderId = new_header.GetSenderId();
    uint32_t MsgId = new_header.GetMsgId();
    uint32_t x_flag = new_header.GetXvelID(); // 1 for positive and 0 for negative
    uint32_t y_flag = new_header.GetYvelID(); // 1 for positive and 0 for negative
    double xx1 = old_velocity.x; double yy1 = old_velocity.y; //sender old velocities = old_velocity.x; double yy1 = old_velocity.y; //sender old velocities
    if (x_flag == 0)
      xx1 = (-1.000)*xx1;
    if (y_flag == 0)
      yy1 = (-1.000)*yy1;

    Ptr<Node> Sender_Node = NodeList::GetNode (SenderId);
    //std::cout<<"Packet "<< MsgId<<" sent by node " <<SenderId<< " and received by node "<< socket->GetNode()->GetId()<<" at "<<rcv<<", Packet generated at "<<sqhd<<" and overall delay for this packet is "<<total_delay<<std::endl;
    double xx; double yy; //double zz; // will store positions

    Ptr<MobilityModel> mob2 = socket->GetNode()->GetObject<MobilityModel>();
    double xx2 = mob2->GetPosition().x; double yy2 = mob2->GetPosition().y; double xx3 = mob2->GetVelocity().x; double yy3 = mob2->GetVelocity().y; //receiver values
    Ptr<MobilityModel> mob1 = Sender_Node->GetObject<MobilityModel>();
    // IMPTT: the above command was giving trouble so in the next line, I am taking the current velocities
    // double xx1 = mob1->GetVelocity().x; double yy1 = mob1->GetVelocity().y; // double z_sender_vel = mob1->GetVelocity().z; // velocity at reception instant, wrong approach

    double old_x = old_position.x; double old_y = old_position.y; //sender old positions
    xx = mob1->GetPosition().x; yy = mob1->GetPosition().y; ////sender current position
    // total_error = sqrt(pow(x_error, 2)); // as velocity is only along x-axis
    double x_error = abs(abs(xx)-abs(old_x)); double y_error = abs(abs(yy)-abs(old_y)); 
    total_error = sqrt(x_error*x_error + y_error*y_error);
    // freopen(file_name.c_str(),"a",stdout); //https://stackoverflow.com/questions/21589353/cannot-convert-stdbasic-stringchar-to-const-char-for-argument-1-to-i
             // PId          TxId               RxId                        GenTime                   RecTime                Delay                 TxTruePos(X) TxHeadPos(X) TxTruePos(Y) TxHeadPos(Y) Error(m)   Tx_HeadVelX  Tx_HeadVelY  RecvPos(X) RecvPos(Y)
    std::cout<<MsgId<<" "<<SenderId<<" "<<socket->GetNode()->GetId()<<" "<<(sqhd).GetMilliSeconds()<<" "<<(rcv).GetMilliSeconds()<<" "<<(total_delay).GetMilliSeconds()<<" "<<xx<<" "<<old_x<<" "<<yy<<" "<<old_y<<" "<<x_error<<" "<<y_error<<" "<<total_error<<" "<<xx1<<" "<<yy1<<" "<<xx2<<" "<<yy2<<" "<<xx3<<" "<<yy3<<"\n"<<std::endl;
  }
}

uint32_t MsgCount = 0; // will be used as MsgId to ID the packets from the tx side

int32_t total_transmitted_packets = 0; //won't gve the totl packets transmitted, will give the number of times this function was called = nodes*packets (200*3)
static void GenerateTraffic (Ptr<Socket> socket, uint32_t pktSize, uint32_t pktCount, Time pktInterval )
{
  if (pktCount > 0)
    {
    total_transmitted_packets++;
    Ptr<Packet> p = Create<Packet> (pktSize-(12+24*2+8)); //12 bytes seqts, 24*2 bytes loc_header and 4+4 bytes IDs (MsgId and SenderId)
    loc_header new_header;

    
    // Ptr< ConstantVelocityMobilityModel > mob3 =  ns3::Ns2MobilityHelper::GetMobilityModel	(	std::string idString, const ObjectStore & store) const


    Ptr<MobilityModel> mob3 = socket->GetNode()->GetObject<MobilityModel>(); //https://stackoverflow.com/questions/57290850/how-to-get-and-print-mobile-nodes-position-in-aodv-ns3
    double x_old_pos = mob3->GetPosition().x; double y_old_pos = mob3->GetPosition().y; double z_old_pos = mob3->GetPosition().z;
    double x_old_vel = mob3->GetVelocity().x; double y_old_vel = mob3->GetVelocity().y; double z_old_vel = mob3->GetVelocity().z;

    // std::cout<<"At time "<<(Simulator::Now()).GetInteger()/100000<<" for node "<<socket->GetNode()->GetId()<<std::endl;
    // std::cout<<"positions are "<<x_old_pos<<","<<y_old_pos<<","<<z_old_pos<<std::endl;
    // std::cout<<"velocities are "<<x_old_vel<<","<<y_old_vel<<","<<z_old_vel<<std::endl;
    int x_flag; int y_flag;
    Vector vect1(x_old_pos, y_old_pos, z_old_pos);
    new_header.SetLocation(vect1);
    Vector vect2(abs(x_old_vel), abs(y_old_vel), abs(z_old_vel));
    if (x_old_vel < 0)
      x_flag = 0;
    else
      x_flag = 1; 
    if (y_old_vel < 0)
      y_flag = 0;
    else
      y_flag = 1; 
    new_header.SetSpeed(vect2);
    new_header.SetSenderId(socket->GetNode()->GetId());
    // std::cout<<"Sender is "<<socket->GetNode()->GetId()<<" and packet is generated from "<<x_old<<", "<<y_old<<", "<<z_old<<", "<<std::endl;
    new_header.SetMsgId(++MsgCount);
    new_header.SetXvelID(x_flag);
    new_header.SetYvelID(y_flag);
    p->AddHeader(new_header);
    SeqTsHeader seqTs;
    seqTs.SetSeq (pktCount);
    p->AddHeader (seqTs); //confirm if size of seqTs is 12 or not??
    socket->Send(p); // means sent for Contention
    Simulator::Schedule (pktInterval, &GenerateTraffic, socket, pktSize, pktCount - 1, pktInterval);
    // std::cout<<"At time "<<(Simulator::Now()).GetInteger()/1000000<<"ms for node "<<socket->GetNode()->GetId()<<std::endl;
    // std::cout<<"positions are "<<x_old_pos<<","<<y_old_pos<<","<<z_old_pos<<std::endl;
    // std::cout<<"velocities are "<<x_old_vel<<","<<y_old_vel<<","<<z_old_vel<<", flags are "<<x_flag<<", "<<y_flag<<std::endl;
    }
  else
    {
    socket->Close ();
    MsgCount = 0;
    }
}

//   void EnqueueTrace(std::string context, Ptr<const WifiMacQueueItem> item) 
// {
//     Time entry_time = Simulator::Now();
// }

// void DequeueTrace(std::string context, Ptr<const WifiMacQueueItem> item) {
//   Time delay = Simulator::Now() - item->GetTimeStamp();
  //std::cout<<"Inside Deque Current Time = "<<Simulator::Now() << "," << "Packet Generation Time = "<< item->GetTimeStamp() << std::endl;
  //std::cout << "Queueing Delay : " << delay << std::endl;
//}

int main (int argc, char *argv[])
{

  std::string phyMode ("OfdmRate6MbpsBW10MHz");
  uint32_t packetSize = 320; // bytes
  uint32_t numPackets = 9999999; // 300 for the other simulations
  // double interval = 0.1; // seconds
  bool verbose = false;

  CommandLine cmd;

//   ns3::Time::SetResolution(Time::NS);

  cmd.AddValue ("phyMode", "Wifi Phy mode", phyMode);
  cmd.AddValue ("packetSize", "size of application packet sent", packetSize);
  cmd.AddValue ("numPackets", "number of packets generated", numPackets);
  // cmd.AddValue ("interval", "interval (seconds) between packets", interval);
  cmd.AddValue ("verbose", "turn on all WifiNetDevice log components", verbose);
  cmd.Parse (argc, argv);
  // Convert to time object
  // Time interPacketInterval = Seconds (interval);

     
  // double intervals[] = {1.0, 0.2, 0.1, 0.06666666666666667, 0.05, 0.04, 0.03333333333, 0.02857142857142857, 0.025, 0.022222222222222223, 0.02, 0.01818181818181818, 0.016666666666666666, 0.015384615384615385, 0.014285714285714285, 0.013333333333333334, 0.0125};
  //intervals as per      1,   5,   10,            15,        20,   25,        30            35,              40,             45,         50,            55,                   60,                  65,                  70,                  75,            80 Hz

  // double intervals[] = {1.0, 0.2, 0.1, 0.05, 0.02, 0.01}; // 1,5,10,20,50,100 Hz
  double intervals[] = {1.0, 0.2, 0.1, 0.06666666666666667, 0.05, 0.04, 0.03333333333, 0.02857142857142857, 0.025, 0.022222222222222223, 0.02, 0.01818181818181818, 0.016666666666666666};
  int kk = (sizeof(intervals)/sizeof(intervals[0]));
  for (int yyy = 0; yyy <kk; yyy++) //loop for beacon rates
  {
    Time interPacketInterval = Seconds (intervals[yyy]);
    // int arr[] = {20,50,120,200}; // denotes the number of vehicles
    int arr[] = {50};
    int mm = (sizeof(arr)/sizeof(*arr)); //number of elements, i.e. number of different density scenarios
    for (int m = 0; m < mm; m++) // loop for number of vehicles
    {
        int num_nodes = arr[m];
        // std::string queue_size[] = { "1p", "5p", "10p", "100p"};
        std::string queue_size[] = { "1p", "100p"};
        int zt =  (sizeof(queue_size)/sizeof(*queue_size));
        for (int uu = 0; uu < zt; uu++) // loop for queue size

        {
          std::string queue_here = queue_size[uu];  
          collisions_MacTxDrop = 0;
          collisions_PhyTxDrop = 0;
          collisions_PhyRxDrop = 0;
          collisions_MacRxDrop = 0;

          NodeContainer c;
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


          Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/$ns3::OcbWifiMac/Txop/Queue/MaxSize", StringValue (queue_here));
          Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/$ns3::OcbWifiMac/VO_Txop/Queue/MaxSize", StringValue (queue_here));
          Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/$ns3::OcbWifiMac/VI_Txop/Queue/MaxSize", StringValue (queue_here));
          Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/$ns3::OcbWifiMac/BE_Txop/Queue/MaxSize", StringValue (queue_here));
          Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/$ns3::OcbWifiMac/BK_Txop/Queue/MaxSize", StringValue (queue_here));

          // Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/$ns3::OcbWifiMac/Txop/Queue/DropPolicy", StringValue("DropOldest"));
          // Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/$ns3::OcbWifiMac/VO_Txop/Queue/DropPolicy", StringValue("DropOldest"));
          // Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/$ns3::OcbWifiMac/VI_Txop/Queue/DropPolicy", StringValue("DropOldest"));
          // Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/$ns3::OcbWifiMac/BE_Txop/Queue/DropPolicy", StringValue("DropOldest"));
          // Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/$ns3::OcbWifiMac/BK_Txop/Queue/DropPolicy", StringValue("DropOldest"));


        //   MobilityHelper mobility;
        //   int64_t m_streamIndex=0;

        //   ObjectFactory pos;
        //   pos.SetTypeId ("ns3::RandomBoxPositionAllocator");
        //   pos.Set ("X", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=1000.0]"));
        //   pos.Set ("Y", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=12.0]"));
        //   // we need antenna height uniform [1.0 .. 2.0] for loss model
        //   pos.Set ("Z", StringValue ("ns3::UniformRandomVariable[Min=1.0|Max=2.0]"));

        //   Ptr<PositionAllocator> taPositionAlloc = pos.Create ()->GetObject<PositionAllocator> ();
        //   m_streamIndex += taPositionAlloc->AssignStreams (m_streamIndex); //meaning?

        //   mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
        //   mobility.SetPositionAllocator (taPositionAlloc);
        //   mobility.Install (c);


          // Ptr<NormalRandomVariable> var=CreateObject<NormalRandomVariable> ();
          // //Ptr<UniformRandomVariable> var = CreateObject<UniformRandomVariable> ();
          // int64_t stream = 2;
          // var->SetStream (stream);

        //   double ROAD_LENGTH_NUM = 1000;
        //   static int num_lanes = 3;
        //   int lane_pos[3] = {0,0,0};
        //   int curr_lane = 0;
        //   double lane_y[3]  = {2.0, 6.0, 10.0};

        //   double veh_spacing = ROAD_LENGTH_NUM*num_lanes/c.GetN();

          // File where the information of error and reception will be saved
          int filename_append = arr[m];
          std::stringstream ss_1;
          ss_1 << filename_append;
          std::string str_1 = ss_1.str();

          std::stringstream ss_2;
          ss_2 << float(1/intervals[yyy]);
          // ss_2 << int(intervals[yyy]);
          std::string str_2 = ss_2.str();

          std::string file_name ("EQ_" + queue_here + "_" + str_2 + "Hz_" + str_1 + "U.txt"); 
          // std::cout<<"PacketId SenderId ReceiverId TotalError(m) GenerationTime(ns) ReceivedTime(ns) Delay(ns) "<<std::endl;
          freopen(file_name.c_str(),"a",stdout);
          std::cout<<"PId TxId RxId GenTime RecTime Delay TxTruePos(X) TxHeadPos(X) TxTruePos(Y) TxHeadPos(Y) Error(X) Error(Y) Error(m) Tx_HeadVelX Tx_HeadVelY RecvPos(X) RecvPos(Y) RecvVel(X) RecvVel(Y)\n"<<std::endl;

          // File where the initial locations will be saved
          // int locname_append = arr[m];
          // std::stringstream ss_3;
          // ss_3 << locname_append;
          // std::string str_3 = ss_3.str();

          // std::stringstream ss_4;
          // ss_4 << float(1/intervals[yyy]);
          // // ss_2 << int(intervals[yyy]);
          // std::string str_4 = ss_4.str();

          std::string loc_name ("Drop_" + queue_here + "_" + str_2 + "Hz_" + str_1 + "U.txt"); 
          freopen(loc_name.c_str(),"a",stderr);
          // std::cerr<<"ID InitialX InitialY InitialZ "<<std::endl;

        //   for(unsigned int i = 0; i < c.GetN(); i++)
        //   {


            //   Ptr<ConstantVelocityMobilityModel> mob = c.Get(i)-> GetObject<ConstantVelocityMobilityModel>();
            //   Vector posm;
            //   double x = lane_pos[curr_lane];
            //   mob->SetPosition(Vector(x, lane_y[curr_lane], 1.0));
            //   Vector node_vel=Vector(25+std::sqrt(5)*var->GetValue (), 0.0, 0.0);
            //   //Vector node_vel=Vector(30+0.5*var->GetValue (), 0.0, 0.0);
            //   mob->SetVelocity(node_vel);
            //   // velocities[i]=node_vel;
            //   posm = mob->GetPosition (); // Get position
            //   Vector vel = mob->GetVelocity (); // Get velocity
            //   std::cerr << Simulator::Now () << " Setup Node: " << i << " POS: x=" << posm.x << ", y=" << posm.y << ", z=" << posm.z << "; VEL:" << vel.x << ", y=" << vel.y  << ", z=" << vel.z << std::endl;
            //   lane_pos[curr_lane] += veh_spacing;
            //   if (lane_pos[curr_lane] > ROAD_LENGTH_NUM)
            //   lane_pos[curr_lane]=0;
            //   curr_lane++;
            //   // std::cout<<Simulator::Now()<<" vehicle"<<i<<"'s position is ("<<posm.x<<", "<< posm.y <<", " <<posm.z<<"), speed = ("<<vel.x<<", "<< vel.y <<", " <<vel.z<<")"<<std::endl;
            //   if (curr_lane > (num_lanes-1))
            //   curr_lane = 0;
              
            //   // std::cerr<<i<<" "<<posm.x<<" "<<posm.y<<" "<<posm.z<<std::endl;


          Ns2MobilityHelper ns2 = Ns2MobilityHelper ( "/home/biplav/mobility_files/highway_veh_1.tcl");
          ns2.Install (); // configure movements for each node, while reading trace file
        // initially assume all nodes are not moving
          // WaveBsmHelper::GetNodesMoving ().resize (num_nodes, 0); // need for this ??
        //   }

        //   m_streamIndex += mobility.AssignStreams (c, m_streamIndex);

          int nSinks = num_nodes;
          double TotalTime = 365;

          InternetStackHelper internet;
          internet.Install (c);

          Ipv4AddressHelper addressAdhoc;
          addressAdhoc.SetBase ("10.1.1.0", "255.255.255.0");
          Ipv4InterfaceContainer adhocInterfaces;
          adhocInterfaces = addressAdhoc.Assign (devices);
          TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");

          TrafficControlHelper tch;
          tch.Uninstall (devices);

          Ptr<UniformRandomVariable> rv = CreateObject<UniformRandomVariable> ();
          rv->SetAttribute ("Min", DoubleValue (0.0));
          rv->SetAttribute ("Max", DoubleValue (intervals[yyy])); //on what basis is this max value taken??
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

              // std::string enqueue_path= "/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/*/$ns3::OcbWifiMac/*/Queue/Enqueue";
              // std::string dequeue_path= "/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/*/$ns3::OcbWifiMac/*/Queue/Dequeue";
              // Config::Connect (enqueue_path, MakeCallback (&EnqueueTrace));
              // Config::Connect (dequeue_path, MakeCallback (&DequeueTrace));  
              //std::cout << "/* in main loop about to enter schedule with context */" << '\n';
              Simulator::ScheduleWithContext (source->GetNode ()->GetId (), Seconds (353+rv->GetValue()), &GenerateTraffic, source, packetSize, numPackets, interPacketInterval); //without rv, no packets delivered
          }
          Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacTxDrop", MakeCallback(&MacRxDrop));
          Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacTxDrop", MakeCallback(&MacTxDrop));
          Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyRxDrop", MakeCallback(&PhyRxDrop));
          Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyTxDrop", MakeCallback(&PhyTxDrop));

          // MAC tracing starts
          Simulator::Stop (Seconds (TotalTime));

          // AnimationInterface anim("first.xml");



          // Config::SetDefault ("ns3::ConfigStore::Filename", StringValue ("output-attributes.txt"));
          // Config::SetDefault ("ns3::ConfigStore::FileFormat", StringValue ("RawText"));
          // Config::SetDefault ("ns3::ConfigStore::Mode", StringValue ("Save"));
          // ConfigStore outputConfig;
          // outputConfig.ConfigureAttributes ();

          Simulator::Run ();   
          Simulator::Destroy ();
          //double PDR = 1.000000*(total_received_packets)/((num_nodes-1)*numPackets*num_nodes);
          //std::cout<<"PDR = "<<PDR<<", Total Packets Received = " <<total_received_packets<<std::endl;
          std::cerr<<"collisions_MacTxDrop"<<" "<<"collisions_PhyTxDrop"<<" "<<"collisions_PhyRxDrop"<<" "<<"collisions_MacRxDrop"<<" "<<"MsgCount"<<std::endl;
          std::cerr<<collisions_MacTxDrop<<" "<<collisions_PhyTxDrop<<" "<<collisions_PhyRxDrop<<" "<<collisions_MacRxDrop<<" "<<MsgCount<<std::endl;
          // std::clog<<loc_name<<" is done "<<std::endl;
        }
      }
    
   
  }
  MsgCount = 0;
  return 0;
}