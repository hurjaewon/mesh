/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2006,2007 INRIA
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
 * Author: Mathieu Lacage, <mathieu.lacage@sophia.inria.fr>
 */
#include "ns3/packet.h"
#include "ns3/simulator.h"
#include "ns3/mobility-model.h"
#include "ns3/net-device.h"
#include "ns3/node.h"
#include "ns3/log.h"
#include "ns3/pointer.h"
#include "ns3/enum.h"
#include "ns3/object-factory.h"
#include "yans-wifi-channel.h"
#include "yans-wifi-phy.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/propagation-delay-model.h"
#include "ns3/duplicate-tag.h"
#include "ns3/boolean.h"
#include "ns3/ampdu-tag.h"

NS_LOG_COMPONENT_DEFINE ("YansWifiChannel");

namespace ns3 {

NS_OBJECT_ENSURE_REGISTERED (YansWifiChannel);

TypeId
YansWifiChannel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::YansWifiChannel")
    .SetParent<WifiChannel> ()
    .AddConstructor<YansWifiChannel> ()
    .AddAttribute ("PropagationLossModel", "A pointer to the propagation loss model attached to this channel.",
                   PointerValue (),
                   MakePointerAccessor (&YansWifiChannel::m_loss),
                   MakePointerChecker<PropagationLossModel> ())
    .AddAttribute ("PropagationDelayModel", "A pointer to the propagation delay model attached to this channel.",
                   PointerValue (),
                   MakePointerAccessor (&YansWifiChannel::m_delay),
                   MakePointerChecker<PropagationDelayModel> ())
    .AddAttribute ("CaudalLoss", "enable caudal loss or not",
                   BooleanValue (false),
                   MakeBooleanAccessor (&YansWifiChannel::m_caudal),
                   MakeBooleanChecker ())
  ;
  return tid;
}

YansWifiChannel::YansWifiChannel ()
{
}
YansWifiChannel::~YansWifiChannel ()
{
  NS_LOG_FUNCTION_NOARGS ();
  m_phyList.clear ();
}

void
YansWifiChannel::SetPropagationLossModel (Ptr<PropagationLossModel> loss)
{
  m_loss = loss;
}
void
YansWifiChannel::SetPropagationDelayModel (Ptr<PropagationDelayModel> delay)
{
  m_delay = delay;
}

//802.11ac channel bonding: useful function
  double 
YansWifiChannel::GetRxPowerDbm (double txPowerDbm, Ptr<MobilityModel> senderMobility, Ptr<MobilityModel> receiverMobility) 
{ 
  double rxPowerDbm = m_loss->CalcRxPower (txPowerDbm, senderMobility, receiverMobility); 
  return rxPowerDbm; 
}

void
YansWifiChannel::Send (Ptr<YansWifiPhy> sender, Ptr<const Packet> packet, double txPowerDbm,
    WifiTxVector txVector, WifiPreamble preamble) const
{
  //caudal loss
  AmpduTag tag;
  bool isAmpdu = packet->PeekPacketTag(tag);

  if(isAmpdu)
  {
    int k=0;
    MpduAggregator::DeaggregatedMpdus packets;
    Ptr<Packet> packet_copy = packet->Copy ();
    packets = MpduAggregator::Deaggregate(packet_copy);
    uint16_t noMpdus = packets.size();

    txVector.SetNumberMpdus(noMpdus);

    double* mpdu_us = new double [noMpdus];
    for(int i=0; i<noMpdus; i++)
      mpdu_us[i]=0;


    uint64_t txMode = txVector.GetMode().GetDataRate() * txVector.GetNss(); 
    Time ampduTx = Seconds ((double)packet->GetSize() * 8 / txMode);
    NS_LOG_DEBUG(mpdu_us << " " << txMode << " " << packet->GetSize() << " " << ampduTx);
    for (MpduAggregator::DeaggregatedMpdusCI i = packets.begin(); i != packets.end(); ++i)
    {
      if(k==0)
        mpdu_us[0]=0; 

      else
        mpdu_us[k] = mpdu_us[k-1] + (double)(*i).first->GetSize() * 8 / txMode;
      k++;
      NS_LOG_DEBUG(k-1 << "th time=" << mpdu_us[k-1] << " packetSize=" << (double)(*i).first->GetSize() * 8);
    }
    NS_LOG_DEBUG("1 send AMPDU, #ofMpdus=" << noMpdus << " TxMode=" << txMode << " NSS=" << (int)txVector.GetNss()); 
    // first value (mpdu_us[0]) shows size of this array
    mpdu_us[0] = noMpdus;

    //802.11ac channel bonding 
    enum ChannelBonding ch = DIFF_CHANNEL;
    Ptr<MobilityModel> senderMobility = sender->GetMobility ()->GetObject<MobilityModel> ();
    NS_ASSERT (senderMobility != 0);
    uint32_t j = 0;
    for (PhyList::const_iterator i = m_phyList.begin (); i != m_phyList.end (); i++, j++)
    {
      if (sender != (*i))
      {
        // For now don't account for inter channel interference
        ch = sender->OverlapCheck((*i)->GetChannelNumber(), (*i)->GetOperationalBandwidth(), (*i)->GetChannelNumberS20(), (*i)->GetChannelNumberS40_up(), (*i)->GetChannelNumberS40_down());  
        //          if ((*i)->GetChannelNumber () != sender->GetChannelNumber ())
        //            {
        //              continue;
        //            }
        if(ch == DIFF_CHANNEL)
        {
          continue;
        }

        Ptr<MobilityModel> receiverMobility = (*i)->GetMobility ()->GetObject<MobilityModel> ();
        Time delay = m_delay->GetDelay (senderMobility, receiverMobility);

        //11ac: mutiple_stream_tx_channel	
        uint8_t nss = txVector.GetNss ();
        double rxPowerDbm = 0;

        std::complex<double> * hvector = new std::complex<double> [(nss*nss)*noMpdus+1];
        hvector[0].real() = txPowerDbm;//m_loss->CalcRxPower (txPowerDbm, senderMobility, receiverMobility);
        hvector = m_loss->CalcRxPower (hvector, senderMobility, receiverMobility, nss, mpdu_us);
        NS_LOG_DEBUG(sender << " " << hvector << " " << (int)nss << " " << mpdu_us[0]);
        rxPowerDbm = hvector[0].real();
        txVector.SetCaudalLoss(m_caudal);
        txVector.SetChannelMatrix(hvector, nss, noMpdus);
        for (int i=0; i<(nss*nss)*noMpdus+1;i++)
        {
          NS_LOG_DEBUG("hvector["<<i<<"]="<<hvector[i]);
        }
        NS_LOG_DEBUG("set channel matrix in yanswifichannel to "<<sender << "  nss="<<(int)nss<<" nmpdus="<<noMpdus);
        delete [] hvector;

        NS_LOG_DEBUG ("propagation: txPower=" << txPowerDbm << "dbm, rxPower=" << rxPowerDbm << "dbm, " <<
            "distance=" << senderMobility->GetDistanceFrom (receiverMobility) << "m, delay=" << delay);
        Ptr<Packet> copy = packet->Copy ();
        Ptr<Object> dstNetDevice = m_phyList[j]->GetDevice ();
        uint32_t dstNode;
        uint32_t senderNode = sender->GetDevice ()->GetObject<NetDevice> ()->GetNode ()->GetId ();
        if (dstNetDevice == 0)
        {
          dstNode = 0xffffffff;
        }
        else
        {
          dstNode = dstNetDevice->GetObject<NetDevice> ()->GetNode ()->GetId ();
        }
        NS_LOG_DEBUG ("channelbonding -- " << "sender: " << senderNode << ", dst: " << dstNode  
            << ", channel number: " << sender->GetChannelNumber()
            << ", operation width: " << sender->GetOperationalBandwidth() 
            << ", current width: " << sender->GetCurrentWidth());
        Simulator::ScheduleWithContext (dstNode,
            delay, &YansWifiChannel::Receive, this,
            j, copy, rxPowerDbm, txVector, preamble, ch);
      }
    }
    delete [] mpdu_us;
  }
  else
  {
    //802.11ac channel bonding 
    enum ChannelBonding ch = DIFF_CHANNEL;
    Ptr<MobilityModel> senderMobility = sender->GetMobility ()->GetObject<MobilityModel> ();
    NS_ASSERT (senderMobility != 0);
    uint32_t j = 0;
    for (PhyList::const_iterator i = m_phyList.begin (); i != m_phyList.end (); i++, j++)
    {
      if (sender != (*i))
      {
        // For now don't account for inter channel interference
        ch = sender->OverlapCheck((*i)->GetChannelNumber(), (*i)->GetOperationalBandwidth(), (*i)->GetChannelNumberS20(), (*i)->GetChannelNumberS40_up(), (*i)->GetChannelNumberS40_down());  
        //          if ((*i)->GetChannelNumber () != sender->GetChannelNumber ())
        //            {
        //              continue;
        //            }
        if(ch == DIFF_CHANNEL)
        {
          continue;
        }

        Ptr<MobilityModel> receiverMobility = (*i)->GetMobility ()->GetObject<MobilityModel> ();
        Time delay = m_delay->GetDelay (senderMobility, receiverMobility);

        //11ac: mutiple_stream_tx_channel	
        uint8_t nss = txVector.GetNss ();
        double rxPowerDbm = 0;

        NS_LOG_DEBUG("caudal loss: " << m_caudal); 

        std::complex<double> * hvector = new std::complex<double> [nss*nss+1];
        hvector[0].real() = txPowerDbm;//m_loss->CalcRxPower (txPowerDbm, senderMobility, receiverMobility);
        hvector = m_loss->CalcRxPower (hvector, senderMobility, receiverMobility, nss, NULL);
        rxPowerDbm = hvector[0].real();
        txVector.SetCaudalLoss(m_caudal);
        txVector.SetChannelMatrix(hvector, nss, 1);
        delete [] hvector;

        NS_LOG_DEBUG("propagation: txPower=" << txPowerDbm << "dbm, rxPower=" << rxPowerDbm << "dbm, " <<
            "distance=" << senderMobility->GetDistanceFrom (receiverMobility) << "m, delay=" << delay);
        Ptr<Packet> copy = packet->Copy ();
        Ptr<Object> dstNetDevice = m_phyList[j]->GetDevice ();
        uint32_t dstNode;
        uint32_t senderNode = sender->GetDevice ()->GetObject<NetDevice> ()->GetNode ()->GetId ();
        if (dstNetDevice == 0)
        {
          dstNode = 0xffffffff;
        }
        else
        {
          dstNode = dstNetDevice->GetObject<NetDevice> ()->GetNode ()->GetId ();
        }
        NS_LOG_DEBUG ("channelbonding -- " << "sender: " << senderNode << ", dst: " << dstNode  
            << ", channel number: " << sender->GetChannelNumber()
            << ", operation width: " << sender->GetOperationalBandwidth() 
            << ", current width: " << sender->GetCurrentWidth());
        Simulator::ScheduleWithContext (dstNode,
            delay, &YansWifiChannel::Receive, this,
            j, copy, rxPowerDbm, txVector, preamble, ch);
      }
    }
  }

}

void
YansWifiChannel::Receive (uint32_t i, Ptr<Packet> packet, double rxPowerDbm,
                          WifiTxVector txVector, WifiPreamble preamble, enum ChannelBonding ch) const
{
  m_phyList[i]->StartReceivePacket (packet, rxPowerDbm, txVector, preamble, ch);
}

uint32_t
YansWifiChannel::GetNDevices (void) const
{
  return m_phyList.size ();
}
Ptr<NetDevice>
YansWifiChannel::GetDevice (uint32_t i) const
{
  return m_phyList[i]->GetDevice ()->GetObject<NetDevice> ();
}

void
YansWifiChannel::Add (Ptr<YansWifiPhy> phy)
{
  m_phyList.push_back (phy);
}

int64_t
YansWifiChannel::AssignStreams (int64_t stream)
{
  int64_t currentStream = stream;
  currentStream += m_loss->AssignStreams (stream);
  return (currentStream - stream);
}

//802.11ac: rate adapatation genie
Ptr<PropagationLossModel> YansWifiChannel::GetPropagationLossModel (void)
{
	return m_loss;
}

} // namespace ns3
