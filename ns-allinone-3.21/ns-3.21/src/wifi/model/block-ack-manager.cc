/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2009, 2010 MIRKO BANCHI
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
 * Author: Mirko Banchi <mk.banchi@gmail.com>
 */
#include "ns3/log.h"
#include "ns3/assert.h"
#include "ns3/simulator.h"
#include "ns3/fatal-error.h"

#include "block-ack-manager.h"
#include "mgt-headers.h"
#include "ctrl-headers.h"
#include "wifi-mac-header.h"
#include "edca-txop-n.h"
#include "mac-low.h"
#include "wifi-mac-queue.h"
#include "mac-tx-middle.h"

NS_LOG_COMPONENT_DEFINE ("BlockAckManager");

namespace ns3 {

BlockAckManager::Item::Item ()
{
  NS_LOG_FUNCTION (this);
}

BlockAckManager::Item::Item (Ptr<const Packet> packet, const WifiMacHeader &hdr, Time tStamp)
  : packet (packet),
    hdr (hdr),
    timestamp (tStamp)
{
  NS_LOG_FUNCTION (this << packet << hdr << tStamp);
}

Bar::Bar ()
{
  NS_LOG_FUNCTION (this);
}

Bar::Bar (Ptr<const Packet> bar, Mac48Address recipient, uint8_t tid, bool immediate)
  : bar (bar),
    recipient (recipient),
    tid (tid),
    immediate (immediate)
{
  NS_LOG_FUNCTION (this << bar << recipient << static_cast<uint32_t> (tid) << immediate);
}

BlockAckManager::BlockAckManager ()
{
  NS_LOG_FUNCTION (this);
}

BlockAckManager::~BlockAckManager ()
{
  NS_LOG_FUNCTION (this);
  m_queue = 0;
  m_agreements.clear ();
  m_retryPackets.clear ();
}

bool
BlockAckManager::ExistsAgreement (Mac48Address recipient, uint8_t tid) const
{
  NS_LOG_FUNCTION (this << recipient << static_cast<uint32_t> (tid));
  return (m_agreements.find (std::make_pair (recipient, tid)) != m_agreements.end ());
}

bool
BlockAckManager::ExistsAgreementInState (Mac48Address recipient, uint8_t tid,
                                         enum OriginatorBlockAckAgreement::State state) const
{
  NS_LOG_FUNCTION (this << recipient << static_cast<uint32_t> (tid) << state);
  AgreementsCI it;
  it = m_agreements.find (std::make_pair (recipient, tid));
  if (it != m_agreements.end ())
    {
      switch (state)
        {
        case OriginatorBlockAckAgreement::INACTIVE:
          return it->second.first.IsInactive ();
        case OriginatorBlockAckAgreement::ESTABLISHED:
          return it->second.first.IsEstablished ();
        case OriginatorBlockAckAgreement::PENDING:
          return it->second.first.IsPending ();
        case OriginatorBlockAckAgreement::UNSUCCESSFUL:
          return it->second.first.IsUnsuccessful ();
        default:
          NS_FATAL_ERROR ("Invalid state for block ack agreement");
        }
    }
  return false;
}

void
BlockAckManager::CreateAgreement (const MgtAddBaRequestHeader *reqHdr, Mac48Address recipient)
{
  NS_LOG_FUNCTION (this << reqHdr << recipient);
  std::pair<Mac48Address, uint8_t> key (recipient, reqHdr->GetTid ());
  OriginatorBlockAckAgreement agreement (recipient, reqHdr->GetTid ());
  agreement.SetStartingSequence (reqHdr->GetStartingSequence ());
  /* for now we assume that originator doesn't use this field. Use of this field
     is mandatory only for recipient */
  agreement.SetBufferSize (0);
  agreement.SetTimeout (reqHdr->GetTimeout ());
  agreement.SetAmsduSupport (reqHdr->IsAmsduSupported ());
  if (reqHdr->IsImmediateBlockAck ())
    {
      agreement.SetImmediateBlockAck ();
    }
  else
    {
      agreement.SetDelayedBlockAck ();
    }
  agreement.SetState (OriginatorBlockAckAgreement::PENDING);
  PacketQueue queue (0);
  std::pair<OriginatorBlockAckAgreement, PacketQueue> value (agreement, queue);
  m_agreements.insert (std::make_pair (key, value));
  m_blockPackets (recipient, reqHdr->GetTid ());
}

void
BlockAckManager::DestroyAgreement (Mac48Address recipient, uint8_t tid)
{
  NS_LOG_FUNCTION (this << recipient << static_cast<uint32_t> (tid));
  AgreementsI it = m_agreements.find (std::make_pair (recipient, tid));
  if (it != m_agreements.end ())
    {
      for (std::list<PacketQueueI>::iterator i = m_retryPackets.begin (); i != m_retryPackets.end ();)
        {
          if ((*i)->hdr.GetAddr1 () == recipient && (*i)->hdr.GetQosTid () == tid)
            {
              i = m_retryPackets.erase (i);
            }
          else
            {
              i++;
            }
        }
      m_agreements.erase (it);
      //remove scheduled bar
      for (std::list<Bar>::iterator i = m_bars.begin (); i != m_bars.end ();)
        {
          if (i->recipient == recipient && i->tid == tid)
            {
              i = m_bars.erase (i);
            }
          else
            {
              i++;
            }
        }
    }
}

void
BlockAckManager::UpdateAgreement (const MgtAddBaResponseHeader *respHdr, Mac48Address recipient)
{
  NS_LOG_FUNCTION (this << respHdr << recipient);
  uint8_t tid = respHdr->GetTid ();
  AgreementsI it = m_agreements.find (std::make_pair (recipient, tid));
  if (it != m_agreements.end ())
    {
      OriginatorBlockAckAgreement& agreement = it->second.first;
      agreement.SetBufferSize (respHdr->GetBufferSize () + 1);
      agreement.SetTimeout (respHdr->GetTimeout ());
      agreement.SetAmsduSupport (respHdr->IsAmsduSupported ());
      if (respHdr->IsImmediateBlockAck ())
        {
          agreement.SetImmediateBlockAck ();
        }
      else
        {
          agreement.SetDelayedBlockAck ();
        }
      agreement.SetState (OriginatorBlockAckAgreement::ESTABLISHED);
      if (agreement.GetTimeout () != 0)
        {
          Time timeout = MicroSeconds (1024 * agreement.GetTimeout ());
          agreement.m_inactivityEvent = Simulator::Schedule (timeout,
                                                             &BlockAckManager::InactivityTimeout,
                                                             this,
                                                             recipient, tid);
        }
    }
  m_unblockPackets (recipient, tid);
}

void
BlockAckManager::StorePacket (Ptr<const Packet> packet, const WifiMacHeader &hdr, Time tStamp)
{
  NS_LOG_FUNCTION (this << packet << hdr << tStamp);
  NS_ASSERT (hdr.IsQosData ());

  uint8_t tid = hdr.GetQosTid ();
  Mac48Address recipient = hdr.GetAddr1 ();

  Item item (packet, hdr, tStamp);
  AgreementsI it = m_agreements.find (std::make_pair (recipient, tid));
  NS_ASSERT (it != m_agreements.end ());

  //shbyeon
  AgreementsCI it2 = m_agreements.find (std::make_pair (recipient, tid));
  PacketQueueCI queueIt = (*it2).second.second.begin ();
	bool exist = false;
  while (queueIt != (*it2).second.second.end ())
  {
	  if(hdr.GetSequenceNumber() == (*queueIt).hdr.GetSequenceNumber ())
		{
			NS_LOG_DEBUG("the seq exist = " << hdr.GetSequenceNumber ());
			exist = true;
		}
    queueIt++;
 	}


	if(!exist)
    it->second.second.push_back (item);
}

//shbyeon store retry packet at the front
void
BlockAckManager::StorePacketRetryFront (const WifiMacHeader &hdr, Time tStamp)
{
  NS_LOG_FUNCTION (this);
  NS_ASSERT (hdr.IsQosData ());
	
	NS_LOG_DEBUG("seq="<<hdr.GetSequenceNumber ()<<
							", timeStamp: "<< tStamp);
  uint8_t tid = hdr.GetQosTid ();
  Mac48Address recipient = hdr.GetAddr1 ();
  AgreementsI it = m_agreements.find (std::make_pair (recipient, tid));
  PacketQueueI queueEnd = it->second.second.end ();
  for (PacketQueueI queueIt = it->second.second.begin (); queueIt != queueEnd;)
	{
		NS_LOG_DEBUG("Insert !! seq = "<<hdr.GetSequenceNumber () << "queue seq = " << (*queueIt).hdr.GetSequenceNumber ());
  	if ((*queueIt).hdr.GetSequenceNumber () == hdr.GetSequenceNumber ())
		{

			NS_LOG_DEBUG("Insert !! seq = "<<hdr.GetSequenceNumber ());
    	m_retryPackets.push_front (queueIt);
			break;
		}
		queueIt++;
	}
}

//shbyeon store retry packet at the back
void
BlockAckManager::StorePacketRetryBack (const WifiMacHeader &hdr, Time tStamp)
{
  NS_LOG_FUNCTION (this);
  NS_ASSERT (hdr.IsQosData ());
	
	NS_LOG_DEBUG("seq="<<hdr.GetSequenceNumber ()<<
							", timeStamp: "<< tStamp);
  uint8_t tid = hdr.GetQosTid ();
  Mac48Address recipient = hdr.GetAddr1 ();
  AgreementsI it = m_agreements.find (std::make_pair (recipient, tid));
  PacketQueueI queueEnd = it->second.second.end ();
  for (PacketQueueI queueIt = it->second.second.begin (); queueIt != queueEnd;)
	{
		NS_LOG_DEBUG("Insert !! seq = "<<hdr.GetSequenceNumber () << " queue seq = " << (*queueIt).hdr.GetSequenceNumber ());
  	if ((*queueIt).hdr.GetSequenceNumber () == hdr.GetSequenceNumber ())
		{

			NS_LOG_DEBUG("Insert !! seq = "<<hdr.GetSequenceNumber ());
    	m_retryPackets.push_back (queueIt);
			break;
		}
		queueIt++;
	}
}

Ptr<const Packet>
BlockAckManager::GetNextPacket (WifiMacHeader &hdr)
{
  NS_LOG_FUNCTION (this << &hdr);
  Ptr<const Packet> packet = 0;
  CleanupBuffers ();
  if (m_retryPackets.size () > 0)
    {
      PacketQueueI queueIt = m_retryPackets.front ();
      m_retryPackets.pop_front ();
      packet = queueIt->packet;
      hdr = queueIt->hdr;
      hdr.SetRetry ();
      NS_LOG_INFO ("Retry packet seq=" << hdr.GetSequenceNumber ());
      uint8_t tid = hdr.GetQosTid ();
      Mac48Address recipient = hdr.GetAddr1 ();

      if (ExistsAgreementInState (recipient, tid, OriginatorBlockAckAgreement::ESTABLISHED)
          || SwitchToBlockAckIfNeeded (recipient, tid, hdr.GetSequenceNumber ()))
      {
        if (GetImplicitBlockAckRequest ())
        {
          hdr.SetQosAckPolicy (WifiMacHeader::NORMAL_ACK);
        }
        else
        {
          hdr.SetQosAckPolicy (WifiMacHeader::BLOCK_ACK);
        }
      }
      else
      {
        /* From section 9.10.3 in IEEE802.11e standard:
         * In order to improve efficiency, originators using the Block Ack facility
         * may send MPDU frames with the Ack Policy subfield in QoS control frames
         * set to Normal Ack if only a few MPDUs are available for transmission.[...]
           * When there are sufficient number of MPDUs, the originator may switch back to
           * the use of Block Ack.
           */
          hdr.SetQosAckPolicy (WifiMacHeader::NORMAL_ACK);
          AgreementsI i = m_agreements.find (std::make_pair (recipient, tid));
          i->second.second.erase (queueIt);
        }
    }
  return packet;
}

//shbyeon ampdu integration
Ptr<const Packet>
BlockAckManager::GetNextPacketForAmpdu (WifiMacHeader &hdr, Mac48Address recipient, uint8_t tid, Time& tStamp, uint32_t maxAvailableLength)
{ 
  NS_LOG_FUNCTION (this);
  NS_LOG_DEBUG ("maxDelay=" << m_maxDelay.GetSeconds ());
  CleanupBuffers ();
  Ptr<const Packet> packet = 0;
  if (m_retryPackets.size () > 0)
  {
    std::list<PacketQueueI>::iterator it = m_retryPackets.begin ();
    while (it != m_retryPackets.end ())
    {
      if ((*it)->hdr.GetAddr1 () == recipient && (*it)->hdr.IsQosData () && (*it)->hdr.GetQosTid () == tid)
      {
        NS_LOG_DEBUG (" retry packets exist! retryQueueSize=" << m_retryPackets.size () <<
            ", nextPacketSize=" << ((*it)->packet)->GetSize () << ", maxAvailableLength=" <<
            maxAvailableLength);
        if (((*it)->packet)->GetSize () > maxAvailableLength)
        {
          NS_LOG_DEBUG ("exceeds maximum ampdu size! nextPacketSize: " << 
              ((*it)->packet)->GetSize () << ", maxAvailableLength: " << 
              maxAvailableLength);
          return 0;
        } 

        packet = (*it)->packet;
        hdr = (*it)->hdr;
        tStamp = (*it)->timestamp;

        m_retryPackets.erase (it);
        break;
      }
    }

    hdr.SetRetry ();
    NS_LOG_INFO ("Retry packet seq=" << hdr.GetSequenceNumber ());

    if (ExistsAgreementInState (recipient, tid, OriginatorBlockAckAgreement::ESTABLISHED)
        || SwitchToBlockAckIfNeeded (recipient, tid, hdr.GetSequenceNumber ()))
    {
		  if (GetImplicitBlockAckRequest ())
		  {
			  hdr.SetQosAckPolicy (WifiMacHeader::NORMAL_ACK);
		  }
		  else
		  {
			  hdr.SetQosAckPolicy (WifiMacHeader::BLOCK_ACK);
		  }
    }
    else
    {
      hdr.SetQosAckPolicy (WifiMacHeader::NORMAL_ACK);
      AgreementsI i = m_agreements.find (std::make_pair (recipient, tid));
      i->second.second.erase (*it);
    }
  }
  return packet;
}

bool
BlockAckManager::HasBar (struct Bar &bar)
{
  NS_LOG_FUNCTION (this << &bar);
  if (m_bars.size () > 0)
    {
      bar = m_bars.front ();
      m_bars.pop_front ();
      return true;
    }
  return false;
}

bool
BlockAckManager::HasPackets (void) const
{
  NS_LOG_FUNCTION (this);
  return (m_retryPackets.size () > 0 || m_bars.size () > 0);
}

uint32_t
BlockAckManager::GetNBufferedPackets (Mac48Address recipient, uint8_t tid) const
{
  NS_LOG_FUNCTION (this << recipient << static_cast<uint32_t> (tid));
  uint32_t nPackets = 0;
  if (ExistsAgreement (recipient, tid))
    {
      AgreementsCI it = m_agreements.find (std::make_pair (recipient, tid));
      PacketQueueCI queueIt = (*it).second.second.begin ();
      uint16_t currentSeq = 0;
      while (queueIt != (*it).second.second.end ())
        {
          currentSeq = (*queueIt).hdr.GetSequenceNumber ();
          nPackets++;
          /* a fragmented packet must be counted as one packet */
          while (queueIt != (*it).second.second.end () && (*queueIt).hdr.GetSequenceNumber () == currentSeq)
            {
              queueIt++;
            }
        }
      return nPackets;
    }
  return 0;
}

uint32_t
BlockAckManager::GetNRetryNeededPackets (Mac48Address recipient, uint8_t tid) const
{
  NS_LOG_FUNCTION (this << recipient << static_cast<uint32_t> (tid));
  uint32_t nPackets = 0;
  uint16_t currentSeq = 0;
  if (ExistsAgreement (recipient, tid))
    {
      std::list<PacketQueueI>::const_iterator it = m_retryPackets.begin ();
      while (it != m_retryPackets.end ())
        {
          if ((*it)->hdr.GetAddr1 () == recipient && (*it)->hdr.GetQosTid () == tid)
            {
              currentSeq = (*it)->hdr.GetSequenceNumber ();
              nPackets++;
              /* a fragmented packet must be counted as one packet */
              while (it != m_retryPackets.end () && (*it)->hdr.GetSequenceNumber () == currentSeq)
                {
                  it++;
                }
            }
        }
    }
  return nPackets;
}

//shbyeon set/get implicit blockackreq
void
BlockAckManager::SetImplicitBlockAckRequest (bool implicit)
{
    m_implicitBlockAckRequest = implicit;
}
bool
BlockAckManager::GetImplicitBlockAckRequest (void)
{
  return m_implicitBlockAckRequest;
  }

void
BlockAckManager::SetBlockAckThreshold (uint8_t nPackets)
{
  NS_LOG_FUNCTION (this << static_cast<uint32_t> (nPackets));
  m_blockAckThreshold = nPackets;
}

uint16_t
BlockAckManager::NotifyGotBlockAck (const CtrlBAckResponseHeader *blockAck, Mac48Address recipient, uint16_t results[], uint16_t results_mpdu[])
{
  NS_LOG_FUNCTION (this << blockAck << recipient);
  uint16_t sequenceFirstLost = 0;
  uint16_t resultsIdx = 0;
 
  if (!blockAck->IsMultiTid ())
    {
      uint8_t tid = blockAck->GetTidInfo ();
      if (ExistsAgreementInState (recipient, tid, OriginatorBlockAckAgreement::ESTABLISHED))
        {
          bool foundFirstLost = false;
          AgreementsI it = m_agreements.find (std::make_pair (recipient, tid));
          PacketQueueI queueEnd = it->second.second.end ();

          if (it->second.first.m_inactivityEvent.IsRunning ())
            {
              /* Upon reception of a block ack frame, the inactivity timer at the
                 originator must be reset.
                 For more details see section 11.5.3 in IEEE802.11e standard */
              it->second.first.m_inactivityEvent.Cancel ();
              Time timeout = MicroSeconds (1024 * it->second.first.GetTimeout ());
              it->second.first.m_inactivityEvent = Simulator::Schedule (timeout,
                                                                        &BlockAckManager::InactivityTimeout,
                                                                        this,
                                                                        recipient, tid);
            }
          if (blockAck->IsBasic ())
            {
              for (PacketQueueI queueIt = it->second.second.begin (); queueIt != queueEnd;)
                {
                  if (blockAck->IsFragmentReceived ((*queueIt).hdr.GetSequenceNumber (),
                                                    (*queueIt).hdr.GetFragmentNumber ()))
                    {
                      queueIt = it->second.second.erase (queueIt);
                    }
                  else
                    {
                      if (!foundFirstLost)
                        {
                          foundFirstLost = true;
                          sequenceFirstLost = (*queueIt).hdr.GetSequenceNumber ();
                          (*it).second.first.SetStartingSequence (sequenceFirstLost);
                        }
                      m_retryPackets.push_back (queueIt);
                      queueIt++;
                    }
                }
            }
          else if (blockAck->IsCompressed ())
            {
							//ohlee refresh m_retryPackets to rebuild it
              m_retryPackets.clear();
              for (PacketQueueI queueIt = it->second.second.begin (); queueIt != queueEnd;)
                {
                  resultsIdx++;
                  NS_LOG_DEBUG("recieved MPDU seq=" <<  (*queueIt).hdr.GetSequenceNumber () );
                  results_mpdu[resultsIdx-1]=(*queueIt).packet->GetSize();
                  if (blockAck->IsPacketReceived ((*queueIt).hdr.GetSequenceNumber ()))
                    {
                      results[resultsIdx-1]=1;
                      uint16_t currentSeq = (*queueIt).hdr.GetSequenceNumber ();
                      NS_LOG_DEBUG ("received ack, seq=" << currentSeq);
                      while (queueIt != queueEnd
                             && (*queueIt).hdr.GetSequenceNumber () == currentSeq)
                        {
                          queueIt = it->second.second.erase (queueIt);
                        }
                    }
                  else
                    {
                      if (!foundFirstLost)
                        {
                          foundFirstLost = true;
                          sequenceFirstLost = (*queueIt).hdr.GetSequenceNumber ();
                          (*it).second.first.SetStartingSequence (sequenceFirstLost);
						              NS_LOG_DEBUG ("firstlost, seq=" << sequenceFirstLost);
                        }
											NS_LOG_DEBUG("add to retry queue seq = " << (*queueIt).hdr.GetSequenceNumber ());
                      results[resultsIdx-1]=0; 
                      m_retryPackets.push_back (queueIt);
                      NS_LOG_DEBUG ("# of packets in the queue: " << 
                                  m_queue->GetNPacketsByTidAndAddress (tid, WifiMacHeader::ADDR1, recipient) << 
                                  " # of packets in the buffered queue: " << GetNBufferedPackets (recipient, tid) << 
                                  " # of packets in the retry queue: " << GetNRetryNeededPackets(recipient, tid)); 
                      queueIt++;
                    }
                }
            }
          uint16_t newSeq = m_txMiddle->GetNextSeqNumberByTidAndAddress (tid, recipient);
          if ((foundFirstLost && !SwitchToBlockAckIfNeeded (recipient, tid, sequenceFirstLost))
              || (!foundFirstLost && !SwitchToBlockAckIfNeeded (recipient, tid, newSeq)))
            {
              it->second.first.SetState (OriginatorBlockAckAgreement::INACTIVE);
            }
        }
        if(resultsIdx-GetNBufferedPackets (recipient, tid)>0) 
         resultsIdx-=GetNBufferedPackets (recipient, tid);
    }
  else
    {
      //NOT SUPPORTED FOR NOW
      NS_FATAL_ERROR ("Multi-tid block ack is not supported.");
    }
  return resultsIdx;
}

//ohlee recovery virtual collision
void
BlockAckManager::NotifyBlockAckTimeoutCancel (uint8_t tid, Mac48Address recipient)
{
	NS_LOG_FUNCTION(this);
	AgreementsI it = m_agreements.find (std::make_pair (recipient, tid));
  PacketQueueI queueEnd = it->second.second.end ();
	m_retryPackets.clear();
  for (PacketQueueI queueIt = it->second.second.begin (); queueIt != queueEnd;){
		(*queueIt).hdr.SetRetry();
 		m_retryPackets.push_back (queueIt);
    NS_LOG_DEBUG ("# of packets in the queue: " << m_queue->GetNPacketsByTidAndAddress (tid, WifiMacHeader::ADDR1, recipient)
			<< " # of packets in the buffered queue: " << GetNBufferedPackets (recipient, tid) 
			<< " # of packets in the retry queue: " << GetNRetryNeededPackets(recipient, tid)); 
    queueIt++;
	}
}

// ohlee recovery for rts/cts failer for AMPDU
void
BlockAckManager::NotifyRtsFailed (uint8_t tid, Mac48Address recipient)
{
	NS_LOG_FUNCTION(this);
	AgreementsI it = m_agreements.find (std::make_pair (recipient, tid));
  PacketQueueI queueEnd = it->second.second.end ();
	m_retryPackets.clear();
  for (PacketQueueI queueIt = it->second.second.begin (); queueIt != queueEnd;){
		// 120206 without set retry because there the packets are not retransmitted
		(*queueIt).hdr.SetRetry();
 		m_retryPackets.push_back (queueIt);
    NS_LOG_DEBUG ("# of packets in the queue: " << m_queue->GetNPacketsByTidAndAddress (tid, WifiMacHeader::ADDR1, recipient)
			<< " # of packets in the buffered queue: " << GetNBufferedPackets (recipient, tid) 
			<< " # of packets in the retry queue: " << GetNRetryNeededPackets(recipient, tid)); 
    queueIt++;
	}
}

//ohlee ampdu is enabled, but only single mpdu is transmitted
void
BlockAckManager::NotifyGotAck (uint8_t tid, uint16_t seqNumber, Mac48Address recipient)
{
  NS_LOG_FUNCTION (this);
  NS_LOG_DEBUG ("retryPacketSize=" << m_retryPackets.size ());
  NS_LOG_DEBUG ("# of packets in the buffered queue: " << GetNBufferedPackets (recipient, tid) 
			<< " # of packets in the retry queue: " << GetNRetryNeededPackets(recipient, tid)); 
	if (ExistsAgreementInState (recipient, tid, OriginatorBlockAckAgreement::ESTABLISHED))
	{
		uint16_t newSeq=0;
		AgreementsI it = m_agreements.find (std::make_pair (recipient, tid));
		PacketQueueI queueEnd = it->second.second.end ();

		if (it->second.first.m_inactivityEvent.IsRunning ())
		{
			/* Upon reception of a block ack frame, the inactivity timer at the
				 originator must be reset.
				 For more details see section 11.5.3 in IEEE802.11e standard */              
			it->second.first.m_inactivityEvent.Cancel ();
			Time timeout = MicroSeconds (1024 * it->second.first.GetTimeout ());
			it->second.first.m_inactivityEvent = Simulator::Schedule (timeout,
					&BlockAckManager::InactivityTimeout, this, recipient, tid);
		}

              
		NS_LOG_DEBUG ("received ack, seq=" << seqNumber);
		for (PacketQueueI queueIt = it->second.second.begin (); queueIt != queueEnd;)
		{
                      
			NS_LOG_DEBUG ("queued seq=" << (*queueIt).hdr.GetSequenceNumber ());
			if ((*queueIt).hdr.GetSequenceNumber () == seqNumber)
			{
				queueIt = it->second.second.erase (queueIt);
				NS_LOG_DEBUG ("erase seq=" << (*queueIt).hdr.GetSequenceNumber ());
  			NS_LOG_DEBUG ("retryPacketSize=" << m_retryPackets.size ());
    		NS_LOG_DEBUG ("# of packets in the buffered queue: " << GetNBufferedPackets (recipient, tid) 
											<< " # of packets in the retry queue: " << GetNRetryNeededPackets(recipient, tid)); 
        newSeq = m_txMiddle->GetNextSeqNumberByTidAndAddress (tid, recipient);
				NS_LOG_DEBUG ("nextSeq=" << newSeq);
			}
			else 
				queueIt++;
		}
		if (!SwitchToBlockAckIfNeeded (recipient, tid, newSeq))
		{
			it->second.first.SetState (OriginatorBlockAckAgreement::INACTIVE);
		}
	}
}


void
BlockAckManager::SetBlockAckType (enum BlockAckType bAckType)
{
  NS_LOG_FUNCTION (this << bAckType);
  m_blockAckType = bAckType;
}

//shbyeon schedule block ack request if block ack is missed!
void 
BlockAckManager::ScheduleBlockAckReq(Mac48Address recipient, uint8_t tid)
{
  NS_LOG_FUNCTION (this);
  Ptr<Packet> bar = 0;
	NS_LOG_DEBUG("bar.size: " << m_bars.size());
  AgreementsI it = m_agreements.find (std::make_pair (recipient, tid ));
  NS_ASSERT (it != m_agreements.end ());

  it->second.first.SetNeedBlockAckReq ();
	bar = ScheduleBlockAckReqIfNeeded (recipient, tid);
  if (bar != 0)
    {
			Bar request (bar, recipient, tid, it->second.first.IsImmediateBlockAck ());
	  	m_bars.push_back (request);
			NS_LOG_DEBUG("bar.size: " << m_bars.size());
		}
	else {
		NS_LOG_ERROR("bar should exist here!");
		exit(1);
	}
}

Ptr<Packet>
BlockAckManager::ScheduleBlockAckReqIfNeeded (Mac48Address recipient, uint8_t tid)
{
  /* This method checks if a BlockAckRequest frame should be send to the recipient station.
     Number of packets under block ack is specified in OriginatorBlockAckAgreement object but sometimes
     this number could be incorrect. In fact is possible that a block ack agreement exists for n
     packets but some of these packets are dropped due to MSDU lifetime expiration.
   */
  NS_LOG_FUNCTION (this << recipient << static_cast<uint32_t> (tid));
  AgreementsI it = m_agreements.find (std::make_pair (recipient, tid));
  NS_ASSERT (it != m_agreements.end ());
  NS_LOG_DEBUG ("# of packets in the queue: " << m_queue->GetNPacketsByTidAndAddress (tid, WifiMacHeader::ADDR1, recipient)
      << " # of packets in the buffered queue: " << GetNBufferedPackets (recipient, tid) 
      << " # of packets in the retry queue: " << GetNRetryNeededPackets(recipient, tid)); 
  NS_LOG_DEBUG ("BlockAckReqNeeded: " << (*it).second.first.IsBlockAckRequestNeeded ());

  if ((*it).second.first.IsBlockAckRequestNeeded ()
      || (GetNRetryNeededPackets (recipient, tid) == 0
          && m_queue->GetNPacketsByTidAndAddress (tid, WifiMacHeader::ADDR1, recipient) == 0))
    {
      OriginatorBlockAckAgreement &agreement = (*it).second.first;
	    NS_LOG_DEBUG ("CompleteExchange call!");
      agreement.CompleteExchange ();

      CtrlBAckRequestHeader reqHdr;
      if (m_blockAckType == BASIC_BLOCK_ACK || m_blockAckType == COMPRESSED_BLOCK_ACK)
        {
          reqHdr.SetType (m_blockAckType);
          reqHdr.SetTidInfo (agreement.GetTid ());
          reqHdr.SetStartingSequence (agreement.GetStartingSequence ());
        }
      else if (m_blockAckType == MULTI_TID_BLOCK_ACK)
        {
          NS_FATAL_ERROR ("Multi-tid block ack is not supported.");
        }
      else
        {
          NS_FATAL_ERROR ("Invalid block ack type.");
        }
      Ptr<Packet> bar = Create<Packet> ();
      bar->AddHeader (reqHdr);
      return bar;
    }
  return 0;
}

void
BlockAckManager::InactivityTimeout (Mac48Address recipient, uint8_t tid)
{
  NS_LOG_FUNCTION (this << recipient << static_cast<uint32_t> (tid));
  m_blockAckInactivityTimeout (recipient, tid, true);
}

void
BlockAckManager::NotifyAgreementEstablished (Mac48Address recipient, uint8_t tid, uint16_t startingSeq)
{
  NS_LOG_FUNCTION (this << recipient << static_cast<uint32_t> (tid) << startingSeq);
  AgreementsI it = m_agreements.find (std::make_pair (recipient, tid));
  NS_ASSERT (it != m_agreements.end ());

  it->second.first.SetState (OriginatorBlockAckAgreement::ESTABLISHED);
  it->second.first.SetStartingSequence (startingSeq);
}

void
BlockAckManager::NotifyAgreementUnsuccessful (Mac48Address recipient, uint8_t tid)
{
  NS_LOG_FUNCTION (this << recipient << static_cast<uint32_t> (tid));
  AgreementsI it = m_agreements.find (std::make_pair (recipient, tid));
  NS_ASSERT (it != m_agreements.end ());
  if (it != m_agreements.end ())
    {
      it->second.first.SetState (OriginatorBlockAckAgreement::UNSUCCESSFUL);
    }
}

void
BlockAckManager::NotifyMpduTransmission (Mac48Address recipient, uint8_t tid, uint16_t nextSeqNumber)
{
  NS_LOG_FUNCTION (this << recipient << static_cast<uint32_t> (tid) << nextSeqNumber);
  Ptr<Packet> bar = 0;
  AgreementsI it = m_agreements.find (std::make_pair (recipient, tid));
  NS_ASSERT (it != m_agreements.end ());

  uint16_t nextSeq;
  NS_LOG_DEBUG ("# of retry packets: " << GetNRetryNeededPackets (recipient, tid));
  if (GetNRetryNeededPackets (recipient, tid) > 0)
    {
      nextSeq = GetSeqNumOfNextRetryPacket (recipient, tid);
			NS_LOG_DEBUG ("retry nextSeq=" << nextSeq);
    }
  else
    {
      nextSeq = nextSeqNumber;
			NS_LOG_DEBUG ("noRetry nextSeq=" << nextSeq);
    }
  it->second.first.NotifyMpduTransmission (nextSeq);
  bar = ScheduleBlockAckReqIfNeeded (recipient, tid);
  if (bar != 0)
    {
      Bar request (bar, recipient, tid, it->second.first.IsImmediateBlockAck ());
      m_bars.push_back (request);
    }
}

//shbyeon last subframe of ampdu transmission (notification)
void
BlockAckManager::NotifyLastMpduTransmission (Mac48Address recipient, uint8_t tid)
{
  NS_LOG_FUNCTION (this);
  Ptr<Packet> bar = 0;
  AgreementsI it = m_agreements.find (std::make_pair (recipient, tid));
  NS_ASSERT (it != m_agreements.end ());
  
  it->second.first.SetNeedBlockAckReq ();
  bar = ScheduleBlockAckReqIfNeeded (recipient, tid);
  if (bar != 0)
    {
      Bar request (bar, recipient, tid, it->second.first.IsImmediateBlockAck ());
      m_bars.push_back (request);
    }
}

void
BlockAckManager::SetQueue (Ptr<WifiMacQueue> queue)
{
  NS_LOG_FUNCTION (this << queue);
  m_queue = queue;
}

bool
BlockAckManager::SwitchToBlockAckIfNeeded (Mac48Address recipient, uint8_t tid, uint16_t startingSeq)
{
  NS_LOG_FUNCTION (this << recipient << static_cast<uint32_t> (tid) << startingSeq);
  NS_ASSERT (!ExistsAgreementInState (recipient, tid, OriginatorBlockAckAgreement::PENDING));
  if (!ExistsAgreementInState (recipient, tid, OriginatorBlockAckAgreement::UNSUCCESSFUL) && ExistsAgreement (recipient, tid))
    {
      uint32_t packets = m_queue->GetNPacketsByTidAndAddress (tid, WifiMacHeader::ADDR1, recipient) +
        GetNBufferedPackets (recipient, tid);
      NS_LOG_DEBUG ("# of packets in the queue: " << m_queue->GetNPacketsByTidAndAddress (tid, WifiMacHeader::ADDR1, recipient)
          << " # of packets in the buffered queue: " << GetNBufferedPackets (recipient, tid) 
          << " # of packets in the retry queue: " << GetNRetryNeededPackets(recipient, tid)); 
      if (packets >= m_blockAckThreshold)
      {
        NotifyAgreementEstablished (recipient, tid, startingSeq);
        return true;
        }
    }
  return false;
}

void
BlockAckManager::TearDownBlockAck (Mac48Address recipient, uint8_t tid)
{
  NS_LOG_FUNCTION (this << recipient << static_cast<uint32_t> (tid));
  DestroyAgreement (recipient, tid);
}

bool
BlockAckManager::HasOtherFragments (uint16_t sequenceNumber) const
{
  NS_LOG_FUNCTION (this << sequenceNumber);
  bool retVal = false;
  if (m_retryPackets.size () > 0)
    {
      Item next = *(m_retryPackets.front ());
      if (next.hdr.GetSequenceNumber () == sequenceNumber)
        {
          retVal = true;
        }
    }
  return retVal;
}

uint32_t
BlockAckManager::GetNextPacketSize (void) const
{
  NS_LOG_FUNCTION (this);
  uint32_t size = 0;
  if (m_retryPackets.size () > 0)
    {
      Item next = *(m_retryPackets.front ());
      size = next.packet->GetSize ();
    }
  return size;
}

void
BlockAckManager::CleanupBuffers (void)
{
  NS_LOG_DEBUG ("retryPacketSize=" << m_retryPackets.size ());
  NS_LOG_DEBUG ("maxDelay: " << m_maxDelay.GetSeconds ());
  NS_LOG_FUNCTION (this);
  for (AgreementsI j = m_agreements.begin (); j != m_agreements.end (); j++)
    {
      if (j->second.second.empty ())
        {
          continue;
        }
      Time now = Simulator::Now ();
      PacketQueueI end = j->second.second.begin ();
      for (PacketQueueI i = j->second.second.begin (); i != j->second.second.end (); i++)
        {
          NS_LOG_DEBUG ("timestamp: " << (i->timestamp).GetSeconds () << ", maxDelay: " << 
				  m_maxDelay.GetSeconds () << ", now: " << now.GetSeconds ());
          if (i->timestamp + m_maxDelay > now)
            {
              end = i;
              break;
            }
          else
            {
							NS_LOG_DEBUG("i->timestamp + m_maxDelay < now!!!");
              /* remove retry packet iterator if it's present in retry queue */
              for (std::list<PacketQueueI>::iterator it = m_retryPackets.begin (); it != m_retryPackets.end ();)
                {
									NS_LOG_DEBUG("Seq: "<< (*it)->hdr.GetSequenceNumber());
									NS_LOG_DEBUG("Addr1: " << (*it)->hdr.GetAddr1());
									NS_LOG_DEBUG("GetQosTid: "<< (*it)->hdr.GetQosTid());
                  if ((*it)->hdr.GetAddr1 () == j->second.first.GetPeer ()
                      && (*it)->hdr.GetQosTid () == j->second.first.GetTid ()
                      && (*it)->hdr.GetSequenceNumber () == i->hdr.GetSequenceNumber ())
                    {
                      it = m_retryPackets.erase (it);
										  NS_LOG_DEBUG("erase-> " << (*it)->hdr.GetSequenceNumber () );
                    }
                  else
                    {
                      it++;
                    }
                }
            }
        }
      j->second.second.erase (j->second.second.begin (), end);
			NS_LOG_DEBUG("set startingsequence: " << end->hdr.GetSequenceNumber ());
      j->second.first.SetStartingSequence (end->hdr.GetSequenceNumber ());
    }
		NS_LOG_DEBUG("END");
}

void
BlockAckManager::SetMaxPacketDelay (Time maxDelay)
{
  NS_LOG_FUNCTION (this << maxDelay);
  m_maxDelay = maxDelay;
}

void
BlockAckManager::SetBlockAckInactivityCallback (Callback<void, Mac48Address, uint8_t, bool> callback)
{
  NS_LOG_FUNCTION (this << &callback);
  m_blockAckInactivityTimeout = callback;
}

void
BlockAckManager::SetBlockDestinationCallback (Callback<void, Mac48Address, uint8_t> callback)
{
  NS_LOG_FUNCTION (this << &callback);
  m_blockPackets = callback;
}

void
BlockAckManager::SetUnblockDestinationCallback (Callback<void, Mac48Address, uint8_t> callback)
{
  NS_LOG_FUNCTION (this << &callback);
  m_unblockPackets = callback;
}

void
BlockAckManager::SetTxMiddle (MacTxMiddle* txMiddle)
{
  NS_LOG_FUNCTION (this << txMiddle);
  m_txMiddle = txMiddle;
}

uint16_t
BlockAckManager::GetSeqNumOfNextRetryPacket (Mac48Address recipient, uint8_t tid) const
{
  NS_LOG_FUNCTION (this << recipient << static_cast<uint32_t> (tid));
  std::list<PacketQueueI>::const_iterator it = m_retryPackets.begin ();
  while (it != m_retryPackets.end ())
    {
      if ((*it)->hdr.GetAddr1 () == recipient && (*it)->hdr.GetQosTid () == tid)
        {
          return (*it)->hdr.GetSequenceNumber ();
        }
    }
  return 4096;
}

} // namespace ns3
