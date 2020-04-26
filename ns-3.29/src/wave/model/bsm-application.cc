/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2014 North Carolina State University
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
 * Author: Scott E. Carpenter <scarpen@ncsu.edu>
 *
 */

#include "ns3/bsm-application.h"
#include "ns3/log.h"
#include "ns3/wave-net-device.h"
#include "ns3/wave-mac-helper.h"
#include "ns3/wave-helper.h"
#include "ns3/mobility-model.h"
#include "ns3/mobility-helper.h"
#include "ns3/loc_header.h"


NS_LOG_COMPONENT_DEFINE ("BsmApplication");

namespace ns3 {

// (Arbitrary) port for establishing socket to transmit WAVE BSMs
int BsmApplication::wavePort = 9080;

NS_OBJECT_ENSURE_REGISTERED (BsmApplication);

TypeId
BsmApplication::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::BsmApplication")
    .SetParent<Application> ()
    .SetGroupName ("Wave")
    .AddConstructor<BsmApplication> ()
    ;

// std::string file_name ("VanetRouting_50U.txt"); //BIPLAV
// freopen(file_name.c_str(),"a",stdout);
// std::cout<<"PId TxId RxId RiskyVID GenTime RecTime Delay TxTruePos(X) TxHeadPos(X) TxTruePos(Y) TxHeadPos(Y) Error(X) Error(Y) Error(m) Tx_HeadVelX Tx_HeadVelY RecvPos(X) RecvPos(Y) RecvVel(X) RecvVel(Y)\n"<<std::endl;


  return tid;
}

BsmApplication::BsmApplication ()
  : m_waveBsmStats (0),
    m_txSafetyRangesSq (),
    m_TotalSimTime (Seconds (10)),
    m_wavePacketSize (200),
    m_numWavePackets (1),
    m_waveInterval (MilliSeconds (100)),
    m_gpsAccuracyNs (10000),
    m_adhocTxInterfaces (0),
    m_nodesMoving (0),
    m_unirv (0),
    m_nodeId (0),
    m_chAccessMode (0),
    m_txMaxDelay (MilliSeconds (10)),
    m_prevTxDelay (MilliSeconds (0))
{
  NS_LOG_FUNCTION (this);
}

static uint32_t MID = 0;


BsmApplication::~BsmApplication ()
{
  NS_LOG_FUNCTION (this);
}

void
BsmApplication::DoDispose (void)
{
  NS_LOG_FUNCTION (this);

  // chain up
  Application::DoDispose ();
}

// Application Methods
void BsmApplication::StartApplication () // Called at time specified by Start
{
  NS_LOG_FUNCTION (this);

  // setup generation of WAVE BSM messages
  Time waveInterPacketInterval = m_waveInterval;

  // BSMs are not transmitted for the first second
  Time startTime = Seconds (1.0);
  // total length of time transmitting WAVE packets
  Time totalTxTime = m_TotalSimTime - startTime;
  // total WAVE packets needing to be sent
  m_numWavePackets = (uint32_t) (totalTxTime.GetDouble () / m_waveInterval.GetDouble ());

  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");

  // every node broadcasts WAVE BSM to potentially all other nodes
  Ptr<Socket> recvSink = Socket::CreateSocket (GetNode (m_nodeId), tid);
  recvSink->SetRecvCallback (MakeCallback (&BsmApplication::ReceiveWavePacket, this));
  InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), wavePort);
  recvSink->BindToNetDevice (GetNetDevice (m_nodeId));
  recvSink->Bind (local);
  recvSink->SetAllowBroadcast (true);

  // dest is broadcast address
  InetSocketAddress remote = InetSocketAddress (Ipv4Address ("255.255.255.255"), wavePort);
  recvSink->Connect (remote);

  // Transmission start time for each BSM:
  // We assume that the start transmission time
  // for the first packet will be on a ns-3 time
  // "Second" boundary - e.g., 1.0 s.
  // However, the actual transmit time must reflect
  // additional effects of 1) clock drift and
  // 2) transmit delay requirements.
  // 1) Clock drift - clocks are not perfectly
  // synchronized across all nodes.  In a VANET
  // we assume all nodes sync to GPS time, which
  // itself is assumed  accurate to, say, 40-100 ns.
  // Thus, the start transmission time must be adjusted
  // by some value, t_drift.
  // 2) Transmit delay requirements - The US
  // minimum performance requirements for V2V
  // BSM transmission expect a random delay of
  // +/- 5 ms, to avoid simultaneous transmissions
  // by all vehicles congesting the channel.  Thus,
  // we need to adjust the start transmission time by
  // some value, t_tx_delay.
  // Therefore, the actual transmit time should be:
  // t_start = t_time + t_drift + t_tx_delay
  // t_drift is always added to t_time.
  // t_tx_delay is supposed to be +/- 5ms, but if we
  // allow negative numbers the time could drift to a value
  // BEFORE the interval start time (i.e., at 100 ms
  // boundaries, we do not want to drift into the
  // previous interval, such as at 95 ms.  Instead,
  // we always want to be at the 100 ms interval boundary,
  // plus [0..10] ms tx delay.
  // Thus, the average t_tx_delay will be
  // within the desired range of [0..10] ms of
  // (t_time + t_drift)

  // WAVE devices sync to GPS time
  // and all devices would like to begin broadcasting
  // their safety messages immediately at the start of
  // the CCH interval.  However, if all do so, then
  // significant collisions occur.  Thus, we assume there
  // is some GPS sync accuracy on GPS devices,
  // typically 40-100 ns.
  // Get a uniformly random number for GPS sync accuracy, in ns.
  Time tDrift = NanoSeconds (m_unirv->GetInteger (0, m_gpsAccuracyNs));

  // When transmitting at a default rate of 10 Hz,
  // the subsystem shall transmit every 100 ms +/-
  // a random value between 0 and 5 ms. [MPR-BSMTX-TXTIM-002]
  // Source: CAMP Vehicle Safety Communications 4 Consortium
  // On-board Minimum Performance Requirements
  // for V2V Safety Systems Version 1.0, December 17, 2014
  // max transmit delay (default 10ms)
  // get value for transmit delay, as number of ns
  uint32_t d_ns = static_cast<uint32_t> (m_txMaxDelay.GetInteger ());
  // convert random tx delay to ns-3 time
  // see note above regarding centering tx delay
  // offset by 5ms + a random value.
  Time txDelay = NanoSeconds (m_unirv->GetInteger (0, d_ns));
  m_prevTxDelay = txDelay;

  Time txTime = startTime + tDrift + txDelay;
  // schedule transmission of first packet
  Simulator::ScheduleWithContext (recvSink->GetNode ()->GetId (),
                                  txTime, &BsmApplication::GenerateWaveTraffic, this,
                                  recvSink, m_wavePacketSize, m_numWavePackets, waveInterPacketInterval, m_nodeId);
}

void BsmApplication::StopApplication () // Called at time specified by Stop
{
  NS_LOG_FUNCTION (this);
}

void
BsmApplication::Setup (Ipv4InterfaceContainer & i,
                       int nodeId,
                       Time totalTime,
                       uint32_t wavePacketSize, // bytes
                       Time waveInterval,
                       double gpsAccuracyNs,
                       std::vector <double> rangesSq,           // m ^2
                       Ptr<WaveBsmStats> waveBsmStats,
                       std::vector<int> * nodesMoving,
                       int chAccessMode,
                       Time txMaxDelay)
{
  NS_LOG_FUNCTION (this);

  m_unirv = CreateObject<UniformRandomVariable> ();

  m_TotalSimTime = totalTime;
  m_wavePacketSize = wavePacketSize;
  m_waveInterval = waveInterval;
  m_gpsAccuracyNs = gpsAccuracyNs;
  int size = rangesSq.size ();
  m_waveBsmStats = waveBsmStats;
  m_nodesMoving = nodesMoving;
  m_chAccessMode = chAccessMode;
  m_txSafetyRangesSq.clear ();
  m_txSafetyRangesSq.resize (size, 0);

  for (int index = 0; index < size; index++)
    {
      // stored as square of value, for optimization
      m_txSafetyRangesSq[index] = rangesSq[index];
    }

  m_adhocTxInterfaces = &i;
  m_nodeId = nodeId;
  m_txMaxDelay = txMaxDelay;
}

void
BsmApplication::GenerateWaveTraffic (Ptr<Socket> socket, uint32_t pktSize,
                                     uint32_t pktCount, Time pktInterval,
                                     uint32_t sendingNodeId)
{
  NS_LOG_FUNCTION (this);
  const uint32_t riskyNeighbor = 200; // remains constant here
  // more packets to send?
  if (pktCount > 0)
    {
      // for now, we cannot tell if each node has
      // started mobility.  so, as an optimization
      // only send if  this node is moving
      // if not, then skip
      int txNodeId = sendingNodeId;
      Ptr<Node> txNode = GetNode (txNodeId);
      Ptr<MobilityModel> txPosition = txNode->GetObject<MobilityModel> ();
      NS_ASSERT (txPosition != 0);

      int senderMoving = m_nodesMoving->at (txNodeId); //??
      if (senderMoving != 0)
        {
          // send it!
          // BIPLAV starts

          Ptr<Packet> p = Create<Packet> (pktSize-(12+24*2+20)); //12 bytes seqts, and in loc_header-> 24*2 bytes for 2 double vectors of length 3 and 5*4 bytes IDs (MsgId , SenderId , IntendedRx, sign flags)
          loc_header new_header;

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
          new_header.SetMsgId(++MID);

          // std::cout<<"Sender is "<<socket->GetNode()->GetId()<<" and packet is generated from "<<x_old<<", "<<y_old<<", "<<z_old<<", "<<std::endl;
          new_header.SetXvelID(x_flag);
          new_header.SetYvelID(y_flag);
          new_header.SetIntendedRx(riskyNeighbor); //BIPLAV , no check needed as it is 200 if no risky and 0-49 if some risky neighbor exists.
          p->AddHeader(new_header);
          SeqTsHeader seqTs;
          seqTs.SetSeq (pktCount);
          p->AddHeader (seqTs); //confirm if size of seqTs is 12 or not??
          socket->Send(p);
          // BIPLAV ends

          // count it
          m_waveBsmStats->IncTxPktCount ();
          m_waveBsmStats->IncTxByteCount (pktSize);
          int wavePktsSent = m_waveBsmStats->GetTxPktCount ();
          if ((m_waveBsmStats->GetLogging () != 0) && ((wavePktsSent % 1000) == 0))
            {
              NS_LOG_UNCOND ("Sending WAVE pkt # " << wavePktsSent );
            }

          // find other nodes within range that would be
          // expected to receive this broadbast
          int nRxNodes = m_adhocTxInterfaces->GetN ();
          for (int i = 0; i < nRxNodes; i++)
            {
              Ptr<Node> rxNode = GetNode (i);
              int rxNodeId = rxNode->GetId ();

              if (rxNodeId != txNodeId)
                {
                  Ptr<MobilityModel> rxPosition = rxNode->GetObject<MobilityModel> ();
                  NS_ASSERT (rxPosition != 0);
                  // confirm that the receiving node
                  // has also started moving in the scenario
                  // if it has not started moving, then
                  // it is not a candidate to receive a packet
                  int receiverMoving = m_nodesMoving->at (rxNodeId);
                  if (receiverMoving == 1)
                    {
                      double distSq = MobilityHelper::GetDistanceSquaredBetween (txNode, rxNode);
                      if (distSq > 0.0)
                        {
                          // dest node within range?
                          int rangeCount = m_txSafetyRangesSq.size ();
                          for (int index = 1; index <= rangeCount; index++)
                            {
                              if (distSq <= m_txSafetyRangesSq[index - 1])
                                {
                                  // we should expect dest node to receive broadcast pkt
                                  m_waveBsmStats->IncExpectedRxPktCount (index);
                                }
                            }
                        }
                    }
                }
            }
        }

      // every BSM must be scheduled with a tx time delay
      // of +/- (5) ms.  See comments in StartApplication().
      // we handle this as a tx delay of [0..10] ms
      // from the start of the pktInterval boundary
      uint32_t d_ns = static_cast<uint32_t> (m_txMaxDelay.GetInteger ());
      Time txDelay = NanoSeconds (m_unirv->GetInteger (0, d_ns));

      // do not want the tx delay to be cumulative, so
      // deduct the previous delay value.  thus we adjust
      // to schedule the next event at the next pktInterval,
      // plus some new [0..10] ms tx delay
      Time txTime = pktInterval - m_prevTxDelay + txDelay;
      m_prevTxDelay = txDelay;

      Simulator::ScheduleWithContext (socket->GetNode ()->GetId (),
                                      txTime, &BsmApplication::GenerateWaveTraffic, this,
                                      socket, pktSize, pktCount - 1, pktInterval,  socket->GetNode ()->GetId ());
    }
  else
    {
      socket->Close ();
    }
}

void BsmApplication::ReceiveWavePacket (Ptr<Socket> socket)
{
  NS_LOG_FUNCTION (this);

  Ptr<Packet> packet;
  Address senderAddr;
  while ((packet = socket->RecvFrom (senderAddr))) //sender's address can be any sender
    {
      Ptr<Node> rxNode = socket->GetNode ();

      if (InetSocketAddress::IsMatchingType (senderAddr))
        {
          InetSocketAddress addr = InetSocketAddress::ConvertFrom (senderAddr);
          int nodes = m_adhocTxInterfaces->GetN ();
          for (int i = 0; i < nodes; i++)
            {
              if (addr.GetIpv4 () == m_adhocTxInterfaces->GetAddress (i) )
                {
                  Ptr<Node> txNode = GetNode (i);
                  HandleReceivedBsmPacket (txNode, rxNode, packet, socket); //BIPLAV
                }
            }
        }
    }
}

// void BsmApplication::HandleReceivedBsmPacket (Ptr<Node> txNode,
//                                               Ptr<Node> rxNode)

void BsmApplication::HandleReceivedBsmPacket (Ptr<Node> txNode,
                                              Ptr<Node> rxNode,
                                              Ptr<Packet> packet,
                                              Ptr<Socket> socket) //biplav
{
  NS_LOG_FUNCTION (this);
  // BIPLAV starts
  // BIPLAV receiving starts
  Time rcv;
  Time sqhd;
  Time total_delay;
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
  uint32_t RiskyId = new_header.GetIntendedRx();
  double xx1 = old_velocity.x; double yy1 = old_velocity.y; //sender old velocities = old_velocity.x; double yy1 = old_velocity.y; //sender old velocities
  if (x_flag == 0)
    xx1 = (-1.000)*xx1;
  if (y_flag == 0)
    yy1 = (-1.000)*yy1;

  uint32_t RxId = socket->GetNode()->GetId();

  //BIPLAV

  Ptr<Node> Sender_Node = GetNode (SenderId);
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
  double total_error = sqrt(x_error*x_error + y_error*y_error);

  // freopen(file_name.c_str(),"a",stdout); //https://stackoverflow.com/questions/21589353/cannot-convert-stdbasic-stringchar-to-const-char-for-argument-1-to-i
            // PId          TxId        RxId       RiskyId               GenTime                   RecTime                          Delay   

  // std::string loc_name ("VanetRouting_50U.txt"); 
  // freopen(loc_name.c_str(),"a",stdout);

  std::cout<<MsgId<<" "<<SenderId<<" "<<RxId<<" "<<RiskyId<<" "<<(sqhd).GetMilliSeconds()<<" "<<(rcv).GetMilliSeconds()<<" "<<(total_delay).GetMilliSeconds()<<" "
  <<xx<<" "<<old_x<<" "<<yy<<" "<<old_y<<" "<<x_error<<" "<<y_error<<" "<<total_error<<" "<<xx1<<" "<<yy1<<" "<<xx2<<" "<<yy2<<" "<<xx3<<" "<<yy3<<"\n"<<std::endl;

  // BIPLAV ends

  m_waveBsmStats->IncRxPktCount ();

  Ptr<MobilityModel> rxPosition = rxNode->GetObject<MobilityModel> ();
  NS_ASSERT (rxPosition != 0);
  // confirm that the receiving node
  // has also started moving in the scenario
  // if it has not started moving, then
  // it is not a candidate to receive a packet
  int rxNodeId = rxNode->GetId ();
  int receiverMoving = m_nodesMoving->at (rxNodeId);
  if (receiverMoving == 1)
    {
      double rxDistSq = MobilityHelper::GetDistanceSquaredBetween (rxNode, txNode);
      if (rxDistSq > 0.0)
        {
          int rangeCount = m_txSafetyRangesSq.size ();
          for (int index = 1; index <= rangeCount; index++)
            {
              if (rxDistSq <= m_txSafetyRangesSq[index - 1])
                {
                  m_waveBsmStats->IncRxPktInRangeCount (index);
                }
            }
        }
    }
}

int64_t
BsmApplication::AssignStreams (int64_t streamIndex)
{
  NS_LOG_FUNCTION (this);

  NS_ASSERT (m_unirv);  // should be set by Setup() previously
  m_unirv->SetStream (streamIndex);

  return 1;
}

Ptr<Node>
BsmApplication::GetNode (int id)
{
  NS_LOG_FUNCTION (this);

  std::pair<Ptr<Ipv4>, uint32_t> interface = m_adhocTxInterfaces->Get (id);
  Ptr<Ipv4> pp = interface.first;
  Ptr<Node> node = pp->GetObject<Node> ();

  return node;
}

Ptr<NetDevice>
BsmApplication::GetNetDevice (int id)
{
  NS_LOG_FUNCTION (this);

  std::pair<Ptr<Ipv4>, uint32_t> interface = m_adhocTxInterfaces->Get (id);
  Ptr<Ipv4> pp = interface.first;
  Ptr<NetDevice> device = pp->GetObject<NetDevice> ();

  return device;
}

} // namespace ns3
