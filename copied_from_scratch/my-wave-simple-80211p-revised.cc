/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2005,2006,2007 INRIA
 * Copyright (c) 2013 Dalian University of Technology
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Mathieu Lacage <mathieu.lacage@sophia.inria.fr>
 * Author: Junling Bu <linlinjavaer@gmail.com>
 *
 */
/**
 * This example shows basic construction of an 802.11p node.  Two nodes
 * are constructed with 802.11p devices, and by default, one node sends a single
 * packet to another node (the number of packets and interval between
 * them can be configured by command-line arguments).  The example shows
 * typical usage of the helper classes for this mode of WiFi (where "OCB" refers
 * to "Outside the Context of a BSS")."
 */

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
#include "ns3/config-store-module.h"
#include "ns3/netanim-module.h"
#include "ns3/traffic-control-helper.h"
#include "ns3/queue-size.h"
#include "ns3/wave-bsm-stats.h"


using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("WifiSimpleOcb");

/*
 * In WAVE module, there is no net device class named like "Wifi80211pNetDevice",
 * instead, we need to use Wifi80211pHelper to create an object of
 * WifiNetDevice class.
 *
 * usage:
 *  NodeContainer nodes;
 *  NetDeviceContainer devices;
 *  nodes.Create (2);
 *  YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();
 *  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
 *  wifiPhy.SetChannel (wifiChannel.Create ());
 *  NqosWaveMacHelper wifi80211pMac = NqosWave80211pMacHelper::Default();
 *  Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default ();
 *  devices = wifi80211p.Install (wifiPhy, wifi80211pMac, nodes);
 *
 * The reason of not providing a 802.11p class is that most of modeling
 * 802.11p standard has been done in wifi module, so we only need a high
 * MAC class that enables OCB mode.
 */

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
      NS_LOG_UNCOND ("Received one packet !");
      // uint32_t queue_size;
      // Ptr<Queue<Packet> > receiver_device = socket->GetBoundNetDevice();
      // Ptr<WifiNetDevice> wifi_device = DynamicCast<WifiNetDevice> (receiver_device);
      // Ptr<WifiMac> wifi_mac = wifi_device->GetMac ();
      // PointerValue ptr;
      // Ptr<EdcaTxopN> edca;
      // wifi_mac->GetAttribute ("BE_EdcaTxopN", ptr);
      // edca = ptr.Get<EdcaTxopN> ();
    }
  //xx = rcv - senttime;
}

static void GenerateTraffic (Ptr<Socket> socket, uint32_t pktSize,
                             uint32_t pktCount, Time pktInterval )
{
  if (pktCount > 0)
    {
      SeqTsHeader seqTs;
      seqTs.SetSeq (pktCount);
      Ptr<Packet> p = Create<Packet> (pktSize-(12));
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
  //what to do when a packet is enqueued
  //You can print the context (what nodes, what channel, etc)
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

  std::string phyMode ("OfdmRate6MbpsBW10MHz");
  uint32_t packetSize = 1000; // bytes
  uint32_t numPackets = 10;
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
  // double txgain = 7.5;
  // double rxgain = 7.5;
  // wifiPhy.Set ("TxGain", DoubleValue (txgain) );
  // wifiPhy.Set ("RxGain", DoubleValue (rxgain) );
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
  Ptr<YansWifiChannel> channel = wifiChannel.Create ();
  wifiPhy.SetChannel (channel);
  // ns-3 supports generate a pcap trace
  wifiPhy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11);
  QosWaveMacHelper wifi80211pMac = QosWaveMacHelper::Default ();
  //Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/Txop/Queue/MaxPackets",  UintegerValue (1));
  //Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/MacEntities/*/$ns3::OcbWifiMac/*/Queue", UintegerValue (1));
  Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default ();
  //wifi80211p.SetQueue ("ns3::DropTailQueue", "MaxSize", StringValue ("1p"));

  if (verbose)
    {
      wifi80211p.EnableLogComponents ();      // Turn on all Wifi 802.11p logging
    }

  wifi80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                      "DataMode",StringValue (phyMode),
                                      "ControlMode",StringValue (phyMode));
  NetDeviceContainer devices = wifi80211p.Install (wifiPhy, wifi80211pMac, c);

  Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/$ns3::OcbWifiMac/Txop/Queue/MaxSize", StringValue ("1p"));
  Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/$ns3::OcbWifiMac/VO_Txop/Queue/MaxSize", StringValue ("1p"));
  Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/$ns3::OcbWifiMac/VI_Txop/Queue/MaxSize", StringValue ("1p"));
  Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/$ns3::OcbWifiMac/BE_Txop/Queue/MaxSize", StringValue ("1p"));
  Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/$ns3::OcbWifiMac/BK_Txop/Queue/MaxSize", StringValue ("1p"));

  Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/$ns3::OcbWifiMac/Txop/Queue/DropPolicy", StringValue("DropOldest"));
  Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/$ns3::OcbWifiMac/VO_Txop/Queue/DropPolicy", StringValue("DropOldest"));
  Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/$ns3::OcbWifiMac/VI_Txop/Queue/DropPolicy", StringValue("DropOldest"));
  Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/$ns3::OcbWifiMac/BE_Txop/Queue/DropPolicy", StringValue("DropOldest"));
  Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/$ns3::OcbWifiMac/BK_Txop/Queue/DropPolicy", StringValue("DropOldest"));

// QUEUE DONE

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


  Simulator::ScheduleWithContext (source->GetNode ()->GetId (),
                                  Seconds (1.0), &GenerateTraffic,
                                  source, packetSize, numPackets, interPacketInterval);

  AnimationInterface anim("first.xml");

  Config::SetDefault ("ns3::ConfigStore::Filename", StringValue ("output-attributes.txt"));
  Config::SetDefault ("ns3::ConfigStore::FileFormat", StringValue ("RawText"));
  Config::SetDefault ("ns3::ConfigStore::Mode", StringValue ("Save"));
  ConfigStore outputConfig;
  outputConfig.ConfigureAttributes ();

  Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacRxDrop", MakeCallback(&MacRxDrop));
  Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacTxDrop", MakeCallback(&MacTxDrop));
  Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyRxDrop", MakeCallback(&PhyRxDrop));
  Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyTxDrop", MakeCallback(&PhyTxDrop));


  Simulator::Run ();
  Simulator::Destroy ();
  std::cout<<collisions_MacTxDrop<<" "<<collisions_PhyTxDrop<<" "<<collisions_PhyRxDrop<<" "<<collisions_MacRxDrop<<std::endl;
  return 0;
}
