#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/wave-bsm-helper.h"
#include "ns3/wave-mac-helper.h"
#include "ns3/wave-helper.h"
#include "ns3/on-off-helper.h"

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
#include "ns3/traffic-control-helper.h"
#include "ns3/config-store-module.h"


using namespace ns3;

NS_LOG_COMPONENT_DEFINE("bsm");

WaveBsmHelper m_waveBsmHelper;

void CourseChange (std::ostream *os, std::string foo, Ptr<const MobilityModel> mobility)
{
  Vector pos = mobility->GetPosition (); // Get position
  Vector vel = mobility->GetVelocity (); // Get velocity

  pos.z = 0;

  /*for (uint32_t i=0; i<m_waveBsmHelper.GetNodesMoving ().size(); i++){
		m_waveBsmHelper.GetNodesMoving ()[i] = 1;
  }*/
  
  int nodeId = mobility->GetObject<Node> ()->GetId ();
  
  double t = (Simulator::Now ()).GetSeconds ();
  
  if (t >= 1.0)
    {
      WaveBsmHelper::GetNodesMoving ()[nodeId] = 1;
    }

  //NS_LOG_UNCOND ("Changing pos for node=" << nodeId << " at " << Simulator::Now () );

  //Prints position and velocities
  std::cout << (Simulator::Now()).GetSeconds () << " Id:"<< nodeId << " POS: x=" << pos.x << ", y=" << pos.y
      << ", z=" << pos.z << "; VEL:" << vel.x << ", y=" << vel.y
      << ", z=" << vel.z << std::endl;
}

int main(int argc, char** argv){
	std::vector <double> m_txSafetyRanges;
	
	Ipv4InterfaceContainer m_adhocTxInterfaces;
	
	NetDeviceContainer m_adhocTxDevices;
	
	// std::string traceFile = "mob.ns_movements";
	
	// int64_t m_streamIndex = 0;
	
	std::ofstream m_os;
	
	m_txSafetyRanges.resize (10, 0);
    m_txSafetyRanges[0] = 100.0;
    m_txSafetyRanges[1] = 150.0;
    m_txSafetyRanges[2] = 200.0;
    m_txSafetyRanges[3] = 250.0;
    m_txSafetyRanges[4] = 300.0;
    m_txSafetyRanges[5] = 350.0;
    m_txSafetyRanges[6] = 400.0;
    m_txSafetyRanges[7] = 450.0;
    m_txSafetyRanges[8] = 500.0;
    m_txSafetyRanges[9] = 550.0;
    
    //Configure nodes and devices.
    NodeContainer c;
    uint32_t num_nodes = 5;
    c.Create (num_nodes);
    //Install mobility model.
    std::cout.precision (2);
    std::cout.setf (std::ios::fixed);
	//   Ns2MobilityHelper ns2 = Ns2MobilityHelper (traceFile);
    // ns2.Install ();
    
    MobilityHelper mobility;
    int64_t m_streamIndex=0;

    ObjectFactory pos;
    pos.SetTypeId ("ns3::RandomBoxPositionAllocator");
    pos.Set ("X", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=1000.0]"));
    pos.Set ("Y", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=12.0]"));
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

    double ROAD_LENGTH_NUM = 1000;
    static int num_lanes = 3;
    int lane_pos[3] = {0,0,0};
    int curr_lane = 0;
    double lane_y[3]  = {2.0, 6.0, 10.0};

    double veh_spacing = ROAD_LENGTH_NUM*num_lanes/c.GetN();

    for(unsigned int i = 0; i < c.GetN(); i++)
        {
            Ptr<ConstantVelocityMobilityModel> mob = c.Get(i)-> GetObject<ConstantVelocityMobilityModel>();
            // Vector posm;
            double x = lane_pos[curr_lane];
            mob->SetPosition(Vector(x, lane_y[curr_lane], 1.0));
            Vector node_vel=Vector(25+std::sqrt(1)*var->GetValue (), 0.0, 0.0);
            //Vector node_vel=Vector(30+0.5*var->GetValue (), 0.0, 0.0);
            mob->SetVelocity(node_vel);
            // velocities[i]=node_vel;
            // posm = mob->GetPosition (); // Get position
            //Vector vel = mob->GetVelocity (); // Get velocity
            // std::cout << Simulator::Now () << " Setup Node: " << i << " POS: x=" << posm.x << ", y=" << posm.y << ", z=" << posm.z << "; VEL:" << vel.x << ", y=" << vel.y  << ", z=" << vel.z << std::endl;
            lane_pos[curr_lane] += veh_spacing;
            if (lane_pos[curr_lane] > ROAD_LENGTH_NUM)
            lane_pos[curr_lane]=0;
            curr_lane++;
            // std::cout<<Simulator::Now()<<" vehicle"<<i<<"'s position is ("<<posm.x<<", "<< posm.y <<", " <<posm.z<<"), speed = ("<<vel.x<<", "<< vel.y <<", " <<vel.z<<")"<<std::endl;
            if (curr_lane > (num_lanes-1))
            curr_lane = 0;
            
            // std::cerr<<i<<" "<<posm.x<<" "<<posm.y<<" "<<posm.z<<std::endl;
        }

    Config::Connect ("/NodeList/*/$ns3::MobilityModel/CourseChange",
                   MakeBoundCallback (&CourseChange, &m_os));
    
        //Internet Stack
	InternetStackHelper internet;
	Ipv4ListRoutingHelper list;
	internet.SetRoutingHelper (list);
    internet.Install (c);
    
    //Id adress
	Ipv4AddressHelper addressAdhoc;
    // we may have a lot of nodes, and want them all
    // in same subnet, to support broadcast
    addressAdhoc.SetBase ("10.1.0.0", "255.255.0.0");
    m_adhocTxInterfaces = addressAdhoc.Assign (m_adhocTxDevices);

    YansWifiChannelHelper wifiChannel;
    wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
    wifiChannel.AddPropagationLoss ("ns3::TwoRayGroundPropagationLossModel", "Frequency", DoubleValue (5.9e9), "HeightAboveZ", DoubleValue (1.5));
    
    Ptr<YansWifiChannel> channel = wifiChannel.Create ();
    YansWavePhyHelper wavePhy =  YansWavePhyHelper::Default ();
	
	wavePhy.SetChannel (channel);
	wavePhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11);
	
	// Set Tx Power
	wavePhy.Set ("TxPowerStart",DoubleValue (20));
	wavePhy.Set ("TxPowerEnd", DoubleValue (20));
    double txgain = 7.5;
    double rxgain = 7.5;
    wavePhy.Set ("TxGain", DoubleValue (txgain) );
    wavePhy.Set ("RxGain", DoubleValue (rxgain) );
	
	WaveHelper waveHelper = WaveHelper::Default ();

	waveHelper.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                      "DataMode",StringValue ("OfdmRate6MbpsBW10MHz"),
                                      "ControlMode",StringValue ("OfdmRate6MbpsBW10MHz"));
	
	QosWaveMacHelper waveMac = QosWaveMacHelper::Default ();
	
	m_adhocTxDevices = waveHelper.Install (wavePhy, waveMac, c);
	
    // Setup routing transmissions
    OnOffHelper onoff1 ("ns3::UdpSocketFactory",Address ());
	onoff1.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1.0]"));
	onoff1.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0.0]"));

	Ptr<UniformRandomVariable> var1 = CreateObject<UniformRandomVariable> ();
	int64_t stream1 = 2;	
	var1->SetStream (stream1);
	
	for (uint32_t i = 0; i < num_nodes; i++)
	{

      AddressValue remoteAddress (InetSocketAddress (m_adhocTxInterfaces.GetAddress (i), 9));
      onoff1.SetAttribute ("Remote", remoteAddress);

      ApplicationContainer temp = onoff1.Install (c.Get (i + 1));
      temp.Start (Seconds (var1->GetValue (1.0,2.0)));
      temp.Stop (Seconds (200));
    }
    
    //Install BSM System.
	m_waveBsmHelper.Install (m_adhocTxInterfaces,
                           Seconds (200),
                           200,
                           Seconds (0.1),
                           // GPS accuracy (i.e, clock drift), in number of ns
                           40,
                           m_txSafetyRanges,
                           0,
                           // tx max delay before transmit, in ms
                           MilliSeconds (10));
                           
    m_streamIndex += m_waveBsmHelper.AssignStreams (c, m_streamIndex);                    
    
    WaveBsmHelper::GetNodesMoving ().resize (3, 0);
    
    m_waveBsmHelper.GetWaveBsmStats ()->SetLogging (2);

    Config::SetDefault ("ns3::ConfigStore::Filename", StringValue ("output-attributes.txt"));
    Config::SetDefault ("ns3::ConfigStore::FileFormat", StringValue ("RawText"));
    Config::SetDefault ("ns3::ConfigStore::Mode", StringValue ("Save"));
    ConfigStore outputConfig;
    outputConfig.ConfigureAttributes ();

    
	Simulator::Stop(Seconds(200));
	Simulator::Run();
	Simulator::Destroy();
	
	double bsm_pdr1 = m_waveBsmHelper.GetWaveBsmStats ()->GetCumulativeBsmPdr (0);
	double bsm_pdr2 = m_waveBsmHelper.GetWaveBsmStats ()->GetCumulativeBsmPdr (1);
	double bsm_pdr3 = m_waveBsmHelper.GetWaveBsmStats ()->GetCumulativeBsmPdr (2);
	double bsm_pdr4 = m_waveBsmHelper.GetWaveBsmStats ()->GetCumulativeBsmPdr (3);
	double bsm_pdr5 = m_waveBsmHelper.GetWaveBsmStats ()->GetCumulativeBsmPdr (4);
	double bsm_pdr6 = m_waveBsmHelper.GetWaveBsmStats ()->GetCumulativeBsmPdr (5);
	double bsm_pdr7 = m_waveBsmHelper.GetWaveBsmStats ()->GetCumulativeBsmPdr (6);
	double bsm_pdr8 = m_waveBsmHelper.GetWaveBsmStats ()->GetCumulativeBsmPdr (7);
	double bsm_pdr9 = m_waveBsmHelper.GetWaveBsmStats ()->GetCumulativeBsmPdr (8);
	double bsm_pdr10 = m_waveBsmHelper.GetWaveBsmStats ()->GetCumulativeBsmPdr (9);
	
	std::cout << bsm_pdr1 << ", " << bsm_pdr2 << ", " << bsm_pdr3 << ", " <<
	bsm_pdr4 << ", " << bsm_pdr5 << ", " << bsm_pdr6 << ", " << bsm_pdr7 << 
	", " << bsm_pdr8 << ", " << bsm_pdr9 << ", " << bsm_pdr10 << std::endl;
}
