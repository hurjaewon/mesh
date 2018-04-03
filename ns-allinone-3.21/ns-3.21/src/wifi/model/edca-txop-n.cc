/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2006, 2009 INRIA
 * Copyright (c) 2009 MIRKO BANCHI
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
 * Author: Mirko Banchi <mk.banchi@gmail.com>
 */
#include "ns3/log.h"
#include "ns3/assert.h"
#include "ns3/boolean.h"
#include "ns3/pointer.h"

#include "edca-txop-n.h"
#include "mac-low.h"
#include "dcf-manager.h"
#include "mac-tx-middle.h"
#include "wifi-mac-trailer.h"
#include "wifi-mac.h"
#include "random-stream.h"
#include "wifi-mac-queue.h"
#include "msdu-aggregator.h"
#include "mgt-headers.h"
#include "qos-blocked-destinations.h"
#include "wifi-bonding.h"
#include "ns3/simulator.h" 

NS_LOG_COMPONENT_DEFINE ("EdcaTxopN");

#undef NS_LOG_APPEND_CONTEXT
#define NS_LOG_APPEND_CONTEXT if (m_low != 0) { std::clog << "[mac=" << m_low->GetAddress () << "] "; }

namespace ns3 {

class EdcaTxopN::Dcf : public DcfState
{
public:
  Dcf (EdcaTxopN * txop)
    : m_txop (txop)
  {
  }
private:
  virtual void DoNotifyAccessGranted (void)
  {
    m_txop->NotifyAccessGranted ();
  }
  virtual void DoNotifyInternalCollision (void)
  {
    m_txop->NotifyInternalCollision ();
  }
  virtual void DoNotifyCollision (void)
  {
    m_txop->NotifyCollision ();
  }
  virtual void DoNotifyChannelSwitching (void)
  {
    m_txop->NotifyChannelSwitching ();
  }
  EdcaTxopN *m_txop;
};

class EdcaTxopN::TransmissionListener : public MacLowTransmissionListener
{
public:
  TransmissionListener (EdcaTxopN * txop)
    : MacLowTransmissionListener (),
      m_txop (txop) {
  }

  virtual ~TransmissionListener () {}

  virtual void GotCts (double snr, WifiMode txMode)
  {
    m_txop->GotCts (snr, txMode);
  }
  virtual void MissedCts (void)
  {
    m_txop->MissedCts ();
  }
  virtual void GotAck (double snr, WifiMode txMode)
  {
    m_txop->GotAck (snr, txMode);
  }
  virtual void MissedAck (void)
  {
    m_txop->MissedAck ();
  }
  virtual void GotBlockAck (const CtrlBAckResponseHeader *blockAck, Mac48Address source)
  {
    m_txop->GotBlockAck (blockAck, source);
  }
  virtual void MissedBlockAck (void)
  {
    m_txop->MissedBlockAck ();
  }
  virtual void StartNext (void)
  {
    m_txop->StartNext ();
  }
  virtual void Cancel (void)
  {
    m_txop->Cancel ();
  }
  virtual void EndTxNoAck (void)
  {
    m_txop->EndTxNoAck ();
  }
  //shbyeon should be added since this is pure function
  virtual void NotifyCollision (void)
  {
    m_txop->NotifyCollision ();
  }

private:
  EdcaTxopN *m_txop;
};

class EdcaTxopN::BlockAckEventListener : public MacLowBlockAckEventListener
{
public:
  BlockAckEventListener (EdcaTxopN * txop)
    : MacLowBlockAckEventListener (),
      m_txop (txop) {
  }
  virtual ~BlockAckEventListener () {}

  virtual void BlockAckInactivityTimeout (Mac48Address address, uint8_t tid)
  {
    m_txop->SendDelbaFrame (address, tid, false);
  }

private:
  EdcaTxopN *m_txop;
};

NS_OBJECT_ENSURE_REGISTERED (EdcaTxopN);

TypeId
EdcaTxopN::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::EdcaTxopN")
    .SetParent (ns3::Dcf::GetTypeId ())
    .AddConstructor<EdcaTxopN> ()
    .AddAttribute ("BlockAckThreshold", "If number of packets in this queue reaches this value,\
                                         block ack mechanism is used. If this value is 0, block ack is never used.",
                   UintegerValue (0),
                   MakeUintegerAccessor (&EdcaTxopN::SetBlockAckThreshold,
                                         &EdcaTxopN::GetBlockAckThreshold),
                   MakeUintegerChecker<uint8_t> (0, 64))
    .AddAttribute ("BlockAckInactivityTimeout", "Represents max time (blocks of 1024 micro seconds) allowed for block ack\
                                                 inactivity. If this value isn't equal to 0 a timer start after that a\
                                                 block ack setup is completed and will be reset every time that a block\
                                                 ack frame is received. If this value is 0, block ack inactivity timeout won't be used.",
                   UintegerValue (0),
                   MakeUintegerAccessor (&EdcaTxopN::SetBlockAckInactivityTimeout),
                   MakeUintegerChecker<uint16_t> ())
    .AddAttribute ("Queue", "The WifiMacQueue object",
                   PointerValue (),
                   MakePointerAccessor (&EdcaTxopN::GetQueue),
                   MakePointerChecker<WifiMacQueue> ())
    .AddAttribute ("ImplcitBlockAckRequest", "enables implicit block ack request",
                   BooleanValue (false),
                   MakeBooleanAccessor (&EdcaTxopN::SetImplicitBlockAckRequest),
                   MakeBooleanChecker ())

  ;
  return tid;
}

EdcaTxopN::EdcaTxopN ()
  : m_manager (0),
    m_currentPacket (0),
    m_aggregator (0),
    m_blockAckType (COMPRESSED_BLOCK_ACK)
{
  NS_LOG_FUNCTION (this);
  m_transmissionListener = new EdcaTxopN::TransmissionListener (this);
  m_blockAckListener = new EdcaTxopN::BlockAckEventListener (this);
  m_dcf = new EdcaTxopN::Dcf (this);
  m_queue = CreateObject<WifiMacQueue> ();
  m_rng = new RealRandomStream ();
  m_qosBlockedDestinations = new QosBlockedDestinations ();
  m_baManager = new BlockAckManager ();
  m_baManager->SetQueue (m_queue);
  m_baManager->SetBlockAckType (m_blockAckType);
  m_baManager->SetBlockDestinationCallback (MakeCallback (&QosBlockedDestinations::Block, m_qosBlockedDestinations));
  m_baManager->SetUnblockDestinationCallback (MakeCallback (&QosBlockedDestinations::Unblock, m_qosBlockedDestinations));
  m_baManager->SetMaxPacketDelay (m_queue->GetMaxDelay ());
	//JWHUR
	to1 = 0, to2 = 0;
}

EdcaTxopN::~EdcaTxopN ()
{
  NS_LOG_FUNCTION (this);
}

void
EdcaTxopN::DoDispose (void)
{
  NS_LOG_FUNCTION (this);
  m_queue = 0;
  m_low = 0;
  m_stationManager = 0;
  delete m_transmissionListener;
  delete m_dcf;
  delete m_rng;
  delete m_qosBlockedDestinations;
  delete m_baManager;
  delete m_blockAckListener;
  m_transmissionListener = 0;
  m_dcf = 0;
  m_rng = 0;
  m_qosBlockedDestinations = 0;
  m_baManager = 0;
  m_blockAckListener = 0;
  m_txMiddle = 0;
  m_aggregator = 0;
	NS_LOG_UNCOND("EdcaTxopN to1: " << to1 << ", to2: " << to2);
}

void
EdcaTxopN::SetManager (DcfManager *manager)
{
  NS_LOG_FUNCTION (this << manager);
  m_manager = manager;
  m_manager->Add (m_dcf);
}

void
EdcaTxopN::SetTxOkCallback (TxOk callback)
{
  NS_LOG_FUNCTION (this << &callback);
  m_txOkCallback = callback;
}

void
EdcaTxopN::SetTxFailedCallback (TxFailed callback)
{
  NS_LOG_FUNCTION (this << &callback);
  m_txFailedCallback = callback;
}

void
EdcaTxopN::SetWifiRemoteStationManager (Ptr<WifiRemoteStationManager> remoteManager)
{
  NS_LOG_FUNCTION (this << remoteManager);
  m_stationManager = remoteManager;
}
void
EdcaTxopN::SetTypeOfStation (enum TypeOfStation type)
{
  NS_LOG_FUNCTION (this << static_cast<uint32_t> (type));
  m_typeOfStation = type;
}

enum TypeOfStation
EdcaTxopN::GetTypeOfStation (void) const
{
  NS_LOG_FUNCTION (this);
  return m_typeOfStation;
}

Ptr<WifiMacQueue >
EdcaTxopN::GetQueue () const
{
  NS_LOG_FUNCTION (this);
  return m_queue;
}

void
EdcaTxopN::SetMinCw (uint32_t minCw)
{
  NS_LOG_FUNCTION (this << minCw);
  m_dcf->SetCwMin (minCw);
}

void
EdcaTxopN::SetMaxCw (uint32_t maxCw)
{
  NS_LOG_FUNCTION (this << maxCw);
  m_dcf->SetCwMax (maxCw);
}

void
EdcaTxopN::SetAifsn (uint32_t aifsn)
{
  NS_LOG_FUNCTION (this << aifsn);
  m_dcf->SetAifsn (aifsn);
}

uint32_t
EdcaTxopN::GetMinCw (void) const
{
  NS_LOG_FUNCTION (this);
  return m_dcf->GetCwMin ();
}

uint32_t
EdcaTxopN::GetMaxCw (void) const
{
  NS_LOG_FUNCTION (this);
  return m_dcf->GetCwMax ();
}

uint32_t
EdcaTxopN::GetAifsn (void) const
{
  NS_LOG_FUNCTION (this);
  return m_dcf->GetAifsn ();
}

//shbyeon set max ppdu time
void
EdcaTxopN::SetMaxPpduTime (Time maxPpduTime){
	m_low->SetMaxPpduTime(maxPpduTime);
}

void
EdcaTxopN::SetTxMiddle (MacTxMiddle *txMiddle)
{
  NS_LOG_FUNCTION (this << txMiddle);
  m_txMiddle = txMiddle;
}

Ptr<MacLow>
EdcaTxopN::Low (void)
{
  NS_LOG_FUNCTION (this);
  return m_low;
}

//shbyeon get block ack manager pointer to use it in mac-low
BlockAckManager*
EdcaTxopN::GetBlockAckManager (void)
{
	return m_baManager;
}

void
EdcaTxopN::SetLow (Ptr<MacLow> low)
{
  NS_LOG_FUNCTION (this << low);
  m_low = low;
}

bool
EdcaTxopN::NeedsAccess (void) const
{
  NS_LOG_FUNCTION (this);
  return !m_queue->IsEmpty () || m_currentPacket != 0 || m_baManager->HasPackets ();
}

//shbyeon core function for aggregation
Ptr<Packet> 
EdcaTxopN::GetNextPacketForAmpdu (WifiMacHeader& hdr, uint32_t maxAvailableLength, Time maxAvailableDuration)
{
  NS_LOG_FUNCTION (this);
  NS_LOG_DEBUG ("maxAvailableLength=" << maxAvailableLength
      << " maxAvailableDuration=" << maxAvailableDuration.GetMicroSeconds () << " us");

  if (m_currentHdr.IsQosData()){
    if (!m_currentHdr.IsRetry ())
      m_baManager->StorePacket (m_currentPacket, m_currentHdr, m_currentPacketTimestamp);
    m_baManager->NotifyMpduTransmission (m_currentHdr.GetAddr1 (), m_currentHdr.GetQosTid (),
        m_txMiddle->GetNextSeqNumberByTidAndAddress (m_currentHdr.GetQosTid (),
          m_currentHdr.GetAddr1 ()));
  }

  //there is no  more packet or blockack buffer overflow...
  if (m_baManager->HasBar (m_currentBar))    
  {
    NS_LOG_DEBUG ("ImplicitBlockAckRequest ()" << GetImplicitBlockAckRequest() );
    if(!GetImplicitBlockAckRequest()){
      hdr = SendBlockAckRequestWithAmpdu (m_currentBar);
      return m_currentPacket->Copy();
    }
    else
    {
      NS_LOG_DEBUG ("there is no more packet to aggregate...");
      return 0;
    }
  }

  //kjyoon
  if (m_stationManager->IsSampling(m_currentHdr.GetAddr1(), &m_currentHdr)) {     // kjyoon
    NS_LOG_DEBUG ("IsSampling == true: Do not aggregate");
    return 0;
  }

  WifiMacHeader nextHdr;
  Ptr <const Packet> nextPacket = m_baManager->GetNextPacketForAmpdu (nextHdr,m_currentHdr.GetAddr1(),m_currentHdr.GetQosTid(), m_currentPacketTimestamp, maxAvailableLength);
  if (nextPacket == 0)
  {
    nextPacket = m_queue->PeekByTidAndAddress (&nextHdr, m_currentHdr.GetQosTid(), WifiMacHeader::ADDR1, m_currentHdr.GetAddr1 ());
    WifiTxVector txvector = m_low->GetDataTxVector (nextPacket, &nextHdr);
    Time blockAckReqDuration = m_low->CalculateTxDuration (20, txvector, WIFI_PREAMBLE_LONG);
		if(GetImplicitBlockAckRequest())
			blockAckReqDuration = Seconds (0.0);
    Time blockAckDuration = m_low->CalculateTxDuration (20, txvector, WIFI_PREAMBLE_LONG) + m_low->GetSifs ();
    Time dataDuration = m_low->CalculateTxDuration (m_low->GetSize (nextPacket, &nextHdr), txvector, WIFI_PREAMBLE_LONG);

    NS_LOG_DEBUG ("dataDuration=" << dataDuration.GetMicroSeconds () 
        << " us, blockAckReqDuration=" << blockAckReqDuration.GetMicroSeconds ()
        << " us, blockAckDuration=" << blockAckDuration.GetMicroSeconds () << " us");

    if (nextPacket == 0) 
    {
      NS_LOG_DEBUG ("no available packets in the queue");
      return 0;
    }

    WifiMacTrailer fcs;
    if (nextPacket->GetSize () + fcs.GetSerializedSize () + nextHdr.GetSerializedSize ()  > maxAvailableLength)
    {
      NS_LOG_DEBUG ("exceeds maximum ampdu size! nextPacketSize: " << nextPacket->GetSize () 
          << ", maxAvailableLength: " << maxAvailableLength);
      m_baManager->NotifyLastMpduTransmission (m_currentHdr.GetAddr1 (), m_currentHdr.GetQosTid ());

      if (m_baManager->HasBar (m_currentBar))
      {
        NS_LOG_DEBUG ("ImplicitBlockAckRequest ()" << GetImplicitBlockAckRequest() );
        if(!GetImplicitBlockAckRequest()){
          hdr = SendBlockAckRequestWithAmpdu (m_currentBar);
          return m_currentPacket->Copy();
        }
        else
        {
          NS_LOG_DEBUG ("there is no more packet to aggregated...");
          return 0;
        }
      }
      return 0;
    }

    else if (dataDuration > maxAvailableDuration)
    //caudal loss
    //else if (dataDuration + blockAckReqDuration + blockAckDuration > maxAvailableDuration)
    {
      NS_LOG_DEBUG ("exceeds maximum ppdu size! dataDuration=" 
          << dataDuration.GetMicroSeconds () 
          << " us, maxAvailableDuration: " << maxAvailableDuration.GetMicroSeconds ()
          << " us");
      m_baManager->NotifyLastMpduTransmission (m_currentHdr.GetAddr1 (), 
          m_currentHdr.GetQosTid ());

      if (m_baManager->HasBar (m_currentBar)) 
      {
        NS_LOG_DEBUG ("ImplicitBlockAckRequest ()" << GetImplicitBlockAckRequest() );
        if(!GetImplicitBlockAckRequest()){
          hdr = SendBlockAckRequestWithAmpdu (m_currentBar);
          return m_currentPacket->Copy();
        }
        else
        {
          NS_LOG_DEBUG ("ImplicitBlockAck is used");
          return 0;
        }
      }
      return 0;
    }

    if (m_currentHdr.IsQosData () && !m_currentHdr.GetAddr1 ().IsBroadcast ()
        && m_blockAckThreshold > 0
        && !m_baManager->ExistsAgreement (m_currentHdr.GetAddr1 (), m_currentHdr.GetQosTid ())
        && SetupBlockAckIfNeeded ())
    {
      return 0;
    }
    NS_LOG_DEBUG ("Peek new packet from queue");
    m_currentPacket = nextPacket;
    m_currentHdr = nextHdr;
    m_queue->Remove (m_currentPacket);
    NS_ASSERT (m_currentPacket != 0);

    uint16_t sequence = m_txMiddle->GetNextSequenceNumberfor (&m_currentHdr);
    m_currentHdr.SetSequenceNumber (sequence);
    m_currentHdr.SetFragmentNumber (0);
    m_currentHdr.SetNoMoreFragments ();
    m_currentHdr.SetNoRetry ();
    m_fragmentNumber = 0;
    NS_LOG_DEBUG ("dequeued size=" << m_currentPacket->GetSize () 
        << ", type="<< m_currentHdr.GetTypeString() 
        << ", to=" << m_currentHdr.GetAddr1 () 
        << ", seq=" << m_currentHdr.GetSequenceNumber () 
        << ", seqCon=" << m_currentHdr.GetSequenceControl ());
    if (m_currentHdr.IsQosData () && !m_currentHdr.GetAddr1 ().IsBroadcast ())
    {
      VerifyBlockAck ();
    }
    else {
      NS_LOG_DEBUG("What the hell?");
      exit(1);
    }
  } 
  else {
    NS_LOG_DEBUG ("Peek ba buffered packet");
    WifiTxVector txvector = m_low->GetDataTxVector (nextPacket, &nextHdr);
    Time blockAckReqDuration = m_low->CalculateTxDuration (20, txvector, WIFI_PREAMBLE_LONG);
		if(GetImplicitBlockAckRequest())
			blockAckReqDuration = Seconds (0.0);
    Time blockAckDuration = m_low->CalculateTxDuration (20, txvector, WIFI_PREAMBLE_LONG)+m_low->GetSifs () ;
    Time dataDuration = m_low->CalculateTxDuration (m_low->GetSize (nextPacket, &nextHdr), txvector, WIFI_PREAMBLE_LONG);
    //caudal loss
    //if (dataDuration + blockAckReqDuration + blockAckDuration > maxAvailableDuration)
    if(dataDuration > maxAvailableDuration)
    {
      NS_LOG_DEBUG ("exceeds maximum ppdu size! dataDuration=" 
          << dataDuration.GetMicroSeconds () 
          << " us, maxAvailableDuration: " << maxAvailableDuration.GetMicroSeconds ()
          << " us");
      m_baManager->StorePacketRetryFront ( nextHdr, m_currentPacketTimestamp);
      m_baManager->NotifyLastMpduTransmission (m_currentHdr.GetAddr1 (), 
          m_currentHdr.GetQosTid ());

      if (m_baManager->HasBar (m_currentBar)) //there is no more packet or blockack buffer overflow...
      {
        NS_LOG_DEBUG ("ImplicitBlockAckRequest ()" << GetImplicitBlockAckRequest() );
        if(!GetImplicitBlockAckRequest()){
          hdr = SendBlockAckRequestWithAmpdu (m_currentBar);
					return m_currentPacket->Copy();
				}
				else
				{
					NS_LOG_DEBUG ("implicitblockack is used");
          return 0;
        }
      }
      return 0;
    }
    else
      m_currentPacket = nextPacket;
    m_currentHdr= nextHdr;
  }

  WifiMacHeader peekedHdr;
  if (m_currentHdr.IsQosData ()
      && m_queue->PeekByTidAndAddress (&peekedHdr, m_currentHdr.GetQosTid (),
        WifiMacHeader::ADDR1, m_currentHdr.GetAddr1 ())
      && !m_currentHdr.GetAddr1 ().IsBroadcast ()
      && m_aggregator != 0 && !m_currentHdr.IsRetry ())
  {
    /* here is performed aggregation */
    Ptr<Packet> currentAggregatedPacket = Create<Packet> ();
    m_aggregator->Aggregate (m_currentPacket, currentAggregatedPacket,
        MapSrcAddressForAggregation (peekedHdr),
        MapDestAddressForAggregation (peekedHdr));
    bool aggregated = false;
    bool isAmsdu = false;
    Ptr<const Packet> peekedPacket = m_queue->PeekByTidAndAddress (&peekedHdr, m_currentHdr.GetQosTid (),
        WifiMacHeader::ADDR1,
        m_currentHdr.GetAddr1 ());
    while (peekedPacket != 0 && (currentAggregatedPacket->GetSize () + peekedPacket->GetSize () < maxAvailableLength)) //bjkim 111004 
    {
      aggregated = m_aggregator->Aggregate (peekedPacket, currentAggregatedPacket,
          MapSrcAddressForAggregation (peekedHdr),
          MapDestAddressForAggregation (peekedHdr));
      if (aggregated)
      {
        isAmsdu = true;
        m_queue->Remove (peekedPacket);
      }
      else
      {
        break;
      }
      peekedPacket = m_queue->PeekByTidAndAddress (&peekedHdr, m_currentHdr.GetQosTid (),
          WifiMacHeader::ADDR1, m_currentHdr.GetAddr1 ());
    }
    if (isAmsdu)
    {
      m_currentHdr.SetQosAmsdu ();
      m_currentHdr.SetAddr3 (m_low->GetBssid ());
      m_currentPacket = currentAggregatedPacket;
      currentAggregatedPacket = 0;
      NS_LOG_DEBUG ("tx unicast A-MSDU");
    }
  }
  NS_LOG_DEBUG ("return m_currentPacket");
  hdr = m_currentHdr;
  return m_currentPacket->Copy (); 
}

//802.11ac channel bonding
bool 
EdcaTxopN::SecondaryIdle (uint16_t bw)
{
	DynamicAccessFlag daf = m_low->NeedRestartBackoff();
  uint16_t bandwidth;
  uint16_t dynamicAccess = m_low->GetDynamicAccess();
  uint16_t operationalBandwidth = m_low->GetOperationalBandwidth();
	  
  switch (daf)
  {
    case PRIMARY_20:
      bandwidth = 20;
      break;
    case SECONDARY_20_IDLE:
      bandwidth = 40;
      break;
    case ALL_IDLE:
      bandwidth = 80;
      break;
    default:
      bandwidth = 20;
      break;
  }
	
	NS_LOG_DEBUG("tx bandwidth = " << bandwidth << " MHz\t access flag="<<daf);
  if(dynamicAccess == 0 && (operationalBandwidth > bandwidth) && bw > bandwidth)
  {
    NS_LOG_DEBUG(Simulator::Now() << " restart backoff (Secondary busy)");
    m_dcf->StartBackoffNow (m_rng->GetNext (0, m_dcf->GetCw ()));

    if(m_currentPacket != 0)
      RestartAccessIfNeeded();
    return false;
  }
	else
	{
		uint16_t txBandwidth;
		if(operationalBandwidth > bw)
			txBandwidth = bw;
		else
			txBandwidth = operationalBandwidth;

		if(m_currentPacket != 0)
		{
			if(bw >= bandwidth)
				txBandwidth = bandwidth;
			m_stationManager->SetCurrentBandwidth(m_currentHdr.GetAddr1 (), &m_currentHdr, txBandwidth);
		}

		NS_LOG_DEBUG("Tx operational="<<operationalBandwidth<<" Rx operational="<< bw << " availableBW=" << txBandwidth);
		return true;
	}
	return true;
}

void
EdcaTxopN::NotifyAccessGranted (void)
{
  NS_LOG_FUNCTION (this);
  NS_LOG_DEBUG("NotifyAccessGranted: " << Simulator::Now());
 

	if (m_currentPacket == 0)
    {
      if (m_queue->IsEmpty () && !m_baManager->HasPackets ())
        {
          NS_LOG_DEBUG ("queue is empty");
          return;
        }
      if (m_baManager->HasBar (m_currentBar))
        {
          SendBlockAckRequest (m_currentBar);
          return;
        }
      /* check if packets need retransmission are stored in BlockAckManager */
      m_currentPacket = m_baManager->GetNextPacket (m_currentHdr);
      if (m_currentPacket == 0)
        {
          if (m_queue->PeekFirstAvailable (&m_currentHdr, m_currentPacketTimestamp, m_qosBlockedDestinations) == 0)
            {
              NS_LOG_DEBUG ("no available packets in the queue");
              return;
            }
          if (m_currentHdr.IsQosData () && !m_currentHdr.GetAddr1 ().IsBroadcast ()
              && m_blockAckThreshold > 0
              && !m_baManager->ExistsAgreement (m_currentHdr.GetAddr1 (), m_currentHdr.GetQosTid ())
              && SetupBlockAckIfNeeded ())
            {
              return;
            }
          m_currentPacket = m_queue->DequeueFirstAvailable (&m_currentHdr, m_currentPacketTimestamp, m_qosBlockedDestinations);
          NS_ASSERT (m_currentPacket != 0);

          uint16_t sequence = m_txMiddle->GetNextSequenceNumberfor (&m_currentHdr);
          m_currentHdr.SetSequenceNumber (sequence);
          m_currentHdr.SetFragmentNumber (0);
          m_currentHdr.SetNoMoreFragments ();
          m_currentHdr.SetNoRetry ();
          m_fragmentNumber = 0;
          NS_LOG_DEBUG ("dequeued size=" << m_currentPacket->GetSize () <<
                        ", to=" << m_currentHdr.GetAddr1 () <<
                        ", seq=" << m_currentHdr.GetSequenceControl ());
          if (m_currentHdr.IsQosData () && !m_currentHdr.GetAddr1 ().IsBroadcast ())
            {
              VerifyBlockAck ();
            }
        }
    }
	
	//802.11ac channel bonding
	if(!SecondaryIdle (m_stationManager->GetRxOperationalBandwidth (m_currentHdr.GetAddr1(), &m_currentHdr)))
		return;

  MacLowTransmissionParameters params;
  params.DisableOverrideDurationId ();
  if (m_currentHdr.GetAddr1 ().IsGroup ())
    {
      params.DisableRts ();
      params.DisableAck ();
      params.DisableNextData ();
      m_low->StartTransmission (m_currentPacket,
                                &m_currentHdr,
                                params,
                                m_transmissionListener);

      NS_LOG_DEBUG ("tx broadcast");
    }
  else if (m_currentHdr.GetType () == WIFI_MAC_CTL_BACKREQ)
    {
  		SendBlockAckRequest (m_currentBar);
    }
  else
    {
      if (m_currentHdr.IsQosData () && m_currentHdr.IsQosBlockAck ())
        {
          params.DisableAck ();
        }
      else
        {
          params.EnableAck ();
        }
      if (NeedFragmentation () && ((m_currentHdr.IsQosData ()
                                    && !m_currentHdr.IsQosAmsdu ())
                                   || 
                                   (m_currentHdr.IsData ()
                                    && !m_currentHdr.IsQosData () && m_currentHdr.IsQosAmsdu ()))
          && (m_blockAckThreshold == 0
              || m_blockAckType == BASIC_BLOCK_ACK))
        {
          //With COMPRESSED_BLOCK_ACK fragmentation must be avoided.
          params.DisableRts ();
          WifiMacHeader hdr;
          Ptr<Packet> fragment = GetFragmentPacket (&hdr);
          if (IsLastFragment ())
            {
              NS_LOG_DEBUG ("fragmenting last fragment size=" << fragment->GetSize ());
              params.DisableNextData ();
            }
          else
            {
              NS_LOG_DEBUG ("fragmenting size=" << fragment->GetSize ());
              params.EnableNextData (GetNextFragmentSize ());
            }
          m_low->StartTransmission (fragment, &hdr, params,
                                    m_transmissionListener);
        }
      else
        {
          WifiMacHeader peekedHdr;
          if (m_currentHdr.IsQosData ()
              && m_queue->PeekByTidAndAddress (&peekedHdr, m_currentHdr.GetQosTid (),
                                               WifiMacHeader::ADDR1, m_currentHdr.GetAddr1 ())
              && !m_currentHdr.GetAddr1 ().IsBroadcast ()
              && m_aggregator != 0 && !m_currentHdr.IsRetry ())
            {
              /* here is performed aggregation */
              Ptr<Packet> currentAggregatedPacket = Create<Packet> ();
              m_aggregator->Aggregate (m_currentPacket, currentAggregatedPacket,
                                       MapSrcAddressForAggregation (peekedHdr),
                                       MapDestAddressForAggregation (peekedHdr));
              bool aggregated = false;
              bool isAmsdu = false;
              Ptr<const Packet> peekedPacket = m_queue->PeekByTidAndAddress (&peekedHdr, m_currentHdr.GetQosTid (),
                                                                             WifiMacHeader::ADDR1,
                                                                             m_currentHdr.GetAddr1 ());
              while (peekedPacket != 0)
                {
                  aggregated = m_aggregator->Aggregate (peekedPacket, currentAggregatedPacket,
                                                        MapSrcAddressForAggregation (peekedHdr),
                                                        MapDestAddressForAggregation (peekedHdr));
                  if (aggregated)
                    {
                      isAmsdu = true;
                      m_queue->Remove (peekedPacket);
                    }
                  else
                    {
                      break;
                    }
                  peekedPacket = m_queue->PeekByTidAndAddress (&peekedHdr, m_currentHdr.GetQosTid (),
                                                               WifiMacHeader::ADDR1, m_currentHdr.GetAddr1 ());
                }
              if (isAmsdu)
                {
                  m_currentHdr.SetQosAmsdu ();
                  m_currentHdr.SetAddr3 (m_low->GetBssid ());
                  m_currentPacket = currentAggregatedPacket;
                  currentAggregatedPacket = 0;
                  NS_LOG_DEBUG ("tx unicast A-MSDU");
                }
            }
          if (NeedRts ())
            {
              params.EnableRts ();
              NS_LOG_DEBUG ("tx unicast rts");
            }
          else
            {
              params.DisableRts ();
              NS_LOG_DEBUG ("tx unicast");
            }
          params.DisableNextData ();
          
          //shbyeon txop implementation
          if (m_currentHdr.IsQosData() && 
              m_stationManager->m_txop > 0 && 
              m_stationManager->GetTxop(m_currentHdr.GetAddr1(),&m_currentHdr) == MicroSeconds(0))
          {
            m_stationManager->SetTxop(m_currentHdr.GetAddr1(), &m_currentHdr, MicroSeconds(m_stationManager->m_txop));
            NS_LOG_DEBUG("Reset TXOPLIMIT to " << MicroSeconds(m_stationManager->m_txop));
          }

        	m_low->StartTransmission (m_currentPacket, &m_currentHdr,
              params, m_transmissionListener);
          CompleteTx ();
        }
    }
}

void EdcaTxopN::NotifyInternalCollision (void)
{
  NS_LOG_FUNCTION (this);
  NotifyCollision ();
}

void
EdcaTxopN::NotifyCollision (void)
{
  NS_LOG_FUNCTION (this);
  m_dcf->StartBackoffNow (m_rng->GetNext (0, m_dcf->GetCw ()));
  RestartAccessIfNeeded ();
}

void
EdcaTxopN::GotCts (double snr, WifiMode txMode)
{
  NS_LOG_FUNCTION (this << snr << txMode);
  NS_LOG_DEBUG ("got cts");
}

void
EdcaTxopN::MissedCts (void)
{
  NS_LOG_FUNCTION (this);
  NS_LOG_DEBUG ("missed cts");
  if (!NeedRtsRetransmission ())
    {
      NS_LOG_DEBUG ("Cts Fail");
      m_stationManager->ReportFinalRtsFailed (m_currentHdr.GetAddr1 (), &m_currentHdr);
      if (!m_txFailedCallback.IsNull ())
        {
          m_txFailedCallback (m_currentHdr);
        }
      // to reset the dcf.
      m_currentPacket = 0;
      m_dcf->ResetCw ();
    }
  else
    {
			//ohlee recovery for rts/cts failer for ampdu ...
			if (m_currentHdr.IsQosData ())
			{
				if (m_baManager->ExistsAgreement (m_currentHdr.GetAddr1 (), m_currentHdr.GetQosTid ()) && 
						m_baManager->GetNRetryNeededPackets (m_currentHdr.GetAddr1 (), m_currentHdr.GetQosTid ()) != 0 )
				{
					m_currentPacket = 0;
				}
			}
      m_dcf->UpdateFailedCw ();
    }
  m_dcf->StartBackoffNow (m_rng->GetNext (0, m_dcf->GetCw ()));
  RestartAccessIfNeeded ();
}

void
EdcaTxopN::NotifyChannelSwitching (void)
{
  NS_LOG_FUNCTION (this);
  m_queue->Flush ();
  m_currentPacket = 0;
}

void
EdcaTxopN::Queue (Ptr<const Packet> packet, const WifiMacHeader &hdr)
{
  NS_LOG_FUNCTION (this << packet << &hdr);
  WifiMacTrailer fcs;
  uint32_t fullPacketSize = hdr.GetSerializedSize () + packet->GetSize () + fcs.GetSerializedSize ();
  m_stationManager->PrepareForQueue (hdr.GetAddr1 (), &hdr,
                                     packet, fullPacketSize);
  m_queue->Enqueue (packet, hdr);
  StartAccessIfNeeded ();
}

void
EdcaTxopN::GotAck (double snr, WifiMode txMode)
{
  NS_LOG_FUNCTION (this << snr << txMode);

	if (m_currentHdr.IsQosData ())
	{
    //shbyeon ampdu is enabled, but only single mpdu was transmitted successfully.
		NS_LOG_DEBUG ("GotAck() seq=" << m_currentHdr.GetSequenceNumber ());
		m_baManager->NotifyGotAck (m_currentHdr.GetQosTid (), m_currentHdr.GetSequenceNumber (),		
				m_currentHdr.GetAddr1 ());
	}

  m_stationManager->UpdateStatistics(m_currentHdr.GetAddr1(), &m_currentHdr, 1, 1, m_stationManager->IsSampling(m_currentHdr.GetAddr1(), &m_currentHdr));

  if (!NeedFragmentation ()
      || IsLastFragment ()
      || m_currentHdr.IsQosAmsdu ())
    {
      NS_LOG_DEBUG ("got ack. tx done.");
      if (!m_txOkCallback.IsNull ())
        {
          m_txOkCallback (m_currentHdr);
        }

      if (m_currentHdr.IsAction ())
        {
          WifiActionHeader actionHdr;
          Ptr<Packet> p = m_currentPacket->Copy ();
          p->RemoveHeader (actionHdr);
          if (actionHdr.GetCategory () == WifiActionHeader::BLOCK_ACK
              && actionHdr.GetAction ().blockAck == WifiActionHeader::BLOCK_ACK_DELBA)
            {
              MgtDelBaHeader delBa;
              p->PeekHeader (delBa);
              if (delBa.IsByOriginator ())
                {
                  m_baManager->TearDownBlockAck (m_currentHdr.GetAddr1 (), delBa.GetTid ());
                }
              else
                {
                  m_low->DestroyBlockAckAgreement (m_currentHdr.GetAddr1 (), delBa.GetTid ());
                }
            }
        }
	    NS_LOG_DEBUG ("Remove m_currentPacket");
      m_currentPacket = 0;

      m_dcf->ResetCw ();
      //shbyeon txop implementation
      if(m_stationManager->GetTxop(m_currentHdr.GetAddr1(), &m_currentHdr) > 0)
      {
        m_dcf->StartBackoffNow (m_rng->GetNext (0,0));
        RestartAccessIfNeeded (true);
      }
      else
      {
        m_dcf->StartBackoffNow (m_rng->GetNext (0, m_dcf->GetCw ()));
        RestartAccessIfNeeded ();
      }
    }
  else
  {
    NS_LOG_DEBUG ("got ack. tx not done, size=" << m_currentPacket->GetSize ());
  }
}

void
EdcaTxopN::MissedAck (void)
{
  NS_LOG_FUNCTION (this);
  NS_LOG_DEBUG ("missed ack");
  m_stationManager->UpdateStatistics(m_currentHdr.GetAddr1(), &m_currentHdr, 1, 0, m_stationManager->IsSampling(m_currentHdr.GetAddr1(), &m_currentHdr));

  if (!NeedDataRetransmission ())
  {
    NS_LOG_DEBUG ("Ack Fail");
    m_stationManager->ReportFinalDataFailed (m_currentHdr.GetAddr1 (), &m_currentHdr);
    if (!m_txFailedCallback.IsNull ())
    {
      m_txFailedCallback (m_currentHdr);
    }
    // to reset the dcf.
    m_currentPacket = 0;
    m_dcf->ResetCw ();
  }
  else
  {
    if(m_currentHdr.IsQosData())
    {
      NS_LOG_DEBUG ("Retransmit");
      if (m_currentHdr.IsRetry ()) {
        if (m_baManager->ExistsAgreement (m_currentHdr.GetAddr1 (), m_currentHdr.GetQosTid ()))
        {
          NS_LOG_DEBUG ("# of packets in the buffered queue: " << m_baManager->GetNBufferedPackets (m_currentHdr.GetAddr1 (), m_currentHdr.GetQosTid ()) 
              << " # of packets in the retry queue: " << m_baManager->GetNRetryNeededPackets(m_currentHdr.GetAddr1 (), m_currentHdr.GetQosTid ())); 

          m_baManager->StorePacketRetryBack (m_currentHdr, m_currentPacketTimestamp);

          NS_LOG_DEBUG ("# of packets in the buffered queue: " << m_baManager->GetNBufferedPackets (m_currentHdr.GetAddr1 (), m_currentHdr.GetQosTid ()) 
              << " # of packets in the retry queue: " << m_baManager->GetNRetryNeededPackets(m_currentHdr.GetAddr1 (), m_currentHdr.GetQosTid ())); 
          m_currentPacket = 0;
        }
      }
      else {	
        m_currentHdr.SetRetry ();

        if (m_baManager->ExistsAgreement (m_currentHdr.GetAddr1 (), m_currentHdr.GetQosTid ()))
        {
          NS_LOG_DEBUG ("# of packets in the buffered queue: " << m_baManager->GetNBufferedPackets (m_currentHdr.GetAddr1 (), m_currentHdr.GetQosTid ()) 
              << " # of packets in the retry queue: " << m_baManager->GetNRetryNeededPackets(m_currentHdr.GetAddr1 (), m_currentHdr.GetQosTid ())); 

          m_baManager->StorePacket (m_currentPacket, m_currentHdr, m_currentPacketTimestamp);

          NS_LOG_DEBUG ("# of packets in the buffered queue: " << m_baManager->GetNBufferedPackets (m_currentHdr.GetAddr1 (), m_currentHdr.GetQosTid ()) 
              << " # of packets in the retry queue: " << m_baManager->GetNRetryNeededPackets(m_currentHdr.GetAddr1 (), m_currentHdr.GetQosTid ())); 

          m_baManager->StorePacketRetryBack (m_currentHdr, m_currentPacketTimestamp);

          NS_LOG_DEBUG ("# of packets in the buffered queue: " << m_baManager->GetNBufferedPackets (m_currentHdr.GetAddr1 (), m_currentHdr.GetQosTid ()) 
              << " # of packets in the retry queue: " << m_baManager->GetNRetryNeededPackets(m_currentHdr.GetAddr1 (), m_currentHdr.GetQosTid ())); 
          m_currentPacket = 0;
        }
      }
    }
    m_dcf->UpdateFailedCw ();
  }

  //shbyeon txop implementation
  if(m_stationManager->GetTxop(m_currentHdr.GetAddr1(), &m_currentHdr) > 0)
  {
    m_dcf->ResetCw ();
    m_dcf->StartBackoffNow (m_rng->GetNext (0,0));
    RestartAccessIfNeeded (true);
  }
  else
  {
    m_dcf->StartBackoffNow (m_rng->GetNext (0, m_dcf->GetCw ()));
    RestartAccessIfNeeded ();
  }
}

void
EdcaTxopN::MissedBlockAck (void)
{
  NS_LOG_FUNCTION (this);
  NS_LOG_DEBUG ("missed block ack");
  //should i report this to station addressed by ADDR1?
  NS_LOG_DEBUG ("Retransmit block ack request");
  m_currentHdr.SetRetry ();
  m_dcf->UpdateFailedCw ();

  //ohlee implicit blockack request (need to generate new blockack)
  if(!m_currentHdr.IsBlockAckReq ()){
      	m_baManager->ScheduleBlockAckReq(m_currentHdr.GetAddr1 (), m_currentHdr.GetQosTid ());
	    NS_LOG_DEBUG ("Remove m_currentPacket");
      	m_currentPacket = 0;
	}
  

  //shbyeon txop implementation
  if(m_stationManager->GetTxop(m_currentHdr.GetAddr1(), &m_currentHdr) > 0)
  {
    m_dcf->ResetCw ();
    m_dcf->StartBackoffNow (m_rng->GetNext (0,0));
    RestartAccessIfNeeded (true);
  }
  else
  {
    m_dcf->StartBackoffNow (m_rng->GetNext (0, m_dcf->GetCw ()));
    RestartAccessIfNeeded ();
  }
  
  /* 150623 by kjyoon 
   * These functions are used for updating MinstrelTable.
   */
  NS_LOG_DEBUG("n_mpdu: " << m_stationManager->GetNMpdus(m_currentHdr.GetAddr1(), &m_currentHdr));
  m_stationManager->UpdateMinstrelTable(m_currentHdr.GetAddr1(), &m_currentHdr, 0, (uint32_t) m_stationManager->GetNMpdus(m_currentHdr.GetAddr1(), &m_currentHdr));	// kjyoon

}

Ptr<MsduAggregator>
EdcaTxopN::GetMsduAggregator (void) const
{
  return m_aggregator;
}

void
EdcaTxopN::RestartAccessIfNeeded (void)
{
  NS_LOG_FUNCTION (this);
  if ((m_currentPacket != 0
       || !m_queue->IsEmpty () || m_baManager->HasPackets ())
      && !m_dcf->IsAccessRequested ())
    {
      m_manager->RequestAccess (m_dcf);
    }
}

//shbyeon txop implementation
void
EdcaTxopN::RestartAccessIfNeeded (bool txop)
{
  NS_LOG_FUNCTION (this);
  if ((m_currentPacket != 0
       || !m_queue->IsEmpty () || m_baManager->HasPackets ())
      && !m_dcf->IsAccessRequested ())
    {
      m_manager->RequestAccess (m_dcf, txop);
    }
}

void
EdcaTxopN::StartAccessIfNeeded (void)
{
  NS_LOG_FUNCTION (this);
  if (m_currentPacket == 0
      && (!m_queue->IsEmpty () || m_baManager->HasPackets ())
      && !m_dcf->IsAccessRequested ())
    {
      m_manager->RequestAccess (m_dcf);
    }
  NS_LOG_DEBUG ("m_queue->IsEmpty: " << m_queue->IsEmpty () << 
		  ", m_baManager->HasPackets: " << m_baManager->HasPackets ());
  NS_LOG_DEBUG ("m_dcf->IsAccessRequested: " << m_dcf->IsAccessRequested ());
}

bool
EdcaTxopN::NeedRts (void)
{
  NS_LOG_FUNCTION (this);
  return m_stationManager->NeedRts (m_currentHdr.GetAddr1 (), &m_currentHdr,
                                    m_currentPacket);
}

bool
EdcaTxopN::NeedRtsRetransmission (void)
{
  NS_LOG_FUNCTION (this);
  return m_stationManager->NeedRtsRetransmission (m_currentHdr.GetAddr1 (), &m_currentHdr,
                                                  m_currentPacket);
}

bool
EdcaTxopN::NeedDataRetransmission (void)
{
  NS_LOG_FUNCTION (this);
  //shbyeon add retransmission for ampdu
	if (m_currentHdr.IsQosData ()){
		if (m_baManager->ExistsAgreement (m_currentHdr.GetAddr1 (), m_currentHdr.GetQosTid ())){
			return 1;
		}
	}
  return m_stationManager->NeedDataRetransmission (m_currentHdr.GetAddr1 (), &m_currentHdr,
                                                   m_currentPacket);
}

void
EdcaTxopN::NextFragment (void)
{
  NS_LOG_FUNCTION (this);
  m_fragmentNumber++;
}

void
EdcaTxopN::StartNext (void)
{
  NS_LOG_FUNCTION (this);
  NS_LOG_DEBUG ("start next packet fragment");
  /* this callback is used only for fragments. */
  NextFragment ();
  WifiMacHeader hdr;
  Ptr<Packet> fragment = GetFragmentPacket (&hdr);
  MacLowTransmissionParameters params;
  params.EnableAck ();
  params.DisableRts ();
  params.DisableOverrideDurationId ();
  if (IsLastFragment ())
    {
      params.DisableNextData ();
    }
  else
    {
      params.EnableNextData (GetNextFragmentSize ());
    }
  Low ()->StartTransmission (fragment, &hdr, params, m_transmissionListener);
}

void
EdcaTxopN::Cancel (void)
{
  NS_LOG_FUNCTION (this);
  NS_LOG_DEBUG ("transmission cancelled");
}

void
EdcaTxopN::EndTxNoAck (void)
{
  NS_LOG_FUNCTION (this);
  NS_LOG_DEBUG ("a transmission that did not require an ACK just finished");
  m_currentPacket = 0;
  m_dcf->ResetCw ();
  m_dcf->StartBackoffNow (m_rng->GetNext (0, m_dcf->GetCw ()));
  StartAccessIfNeeded ();
}

bool
EdcaTxopN::NeedFragmentation (void) const
{
  NS_LOG_FUNCTION (this);
  return m_stationManager->NeedFragmentation (m_currentHdr.GetAddr1 (), &m_currentHdr,
                                              m_currentPacket);
}

uint32_t
EdcaTxopN::GetFragmentSize (void)
{
  NS_LOG_FUNCTION (this);
  return m_stationManager->GetFragmentSize (m_currentHdr.GetAddr1 (), &m_currentHdr,
                                            m_currentPacket, m_fragmentNumber);
}

uint32_t
EdcaTxopN::GetNextFragmentSize (void)
{
  NS_LOG_FUNCTION (this);
  return m_stationManager->GetFragmentSize (m_currentHdr.GetAddr1 (), &m_currentHdr,
                                            m_currentPacket, m_fragmentNumber + 1);
}

uint32_t
EdcaTxopN::GetFragmentOffset (void)
{
  NS_LOG_FUNCTION (this);
  return m_stationManager->GetFragmentOffset (m_currentHdr.GetAddr1 (), &m_currentHdr,
                                              m_currentPacket, m_fragmentNumber);
}


bool
EdcaTxopN::IsLastFragment (void) const
{
  NS_LOG_FUNCTION (this);
  return m_stationManager->IsLastFragment (m_currentHdr.GetAddr1 (), &m_currentHdr,
                                           m_currentPacket, m_fragmentNumber);
}

Ptr<Packet>
EdcaTxopN::GetFragmentPacket (WifiMacHeader *hdr)
{
  NS_LOG_FUNCTION (this << hdr);
  *hdr = m_currentHdr;
  hdr->SetFragmentNumber (m_fragmentNumber);
  uint32_t startOffset = GetFragmentOffset ();
  Ptr<Packet> fragment;
  if (IsLastFragment ())
    {
      hdr->SetNoMoreFragments ();
    }
  else
    {
      hdr->SetMoreFragments ();
    }
  fragment = m_currentPacket->CreateFragment (startOffset,
                                              GetFragmentSize ());
  return fragment;
}

void
EdcaTxopN::SetAccessCategory (enum AcIndex ac)
{
  NS_LOG_FUNCTION (this << static_cast<uint32_t> (ac));
  m_ac = ac;
}

//shbyeon get access category
enum
AcIndex EdcaTxopN::GetAccessCategory (void)
{
  return m_ac;
}

Mac48Address
EdcaTxopN::MapSrcAddressForAggregation (const WifiMacHeader &hdr)
{
  NS_LOG_FUNCTION (this << &hdr);
  Mac48Address retval;
  if (m_typeOfStation == STA || m_typeOfStation == ADHOC_STA)
    {
      retval = hdr.GetAddr2 ();
    }
  else
    {
      retval = hdr.GetAddr3 ();
    }
  return retval;
}

Mac48Address
EdcaTxopN::MapDestAddressForAggregation (const WifiMacHeader &hdr)
{
  NS_LOG_FUNCTION (this << &hdr);
  Mac48Address retval;
  if (m_typeOfStation == AP || m_typeOfStation == ADHOC_STA)
    {
      retval = hdr.GetAddr1 ();
    }
  else
    {
      retval = hdr.GetAddr3 ();
    }
  return retval;
}

void
EdcaTxopN::SetMsduAggregator (Ptr<MsduAggregator> aggr)
{
  NS_LOG_FUNCTION (this << aggr);
  m_aggregator = aggr;
}

void
EdcaTxopN::PushFront (Ptr<const Packet> packet, const WifiMacHeader &hdr)
{
  NS_LOG_FUNCTION (this << packet << &hdr);
  WifiMacTrailer fcs;
  uint32_t fullPacketSize = hdr.GetSerializedSize () + packet->GetSize () + fcs.GetSerializedSize ();
  m_stationManager->PrepareForQueue (hdr.GetAddr1 (), &hdr,
                                     packet, fullPacketSize);
  m_queue->PushFront (packet, hdr);
  StartAccessIfNeeded ();
}

void
EdcaTxopN::GotAddBaResponse (const MgtAddBaResponseHeader *respHdr, Mac48Address recipient)
{
  NS_LOG_FUNCTION (this << respHdr << recipient);
  NS_LOG_DEBUG ("received ADDBA response from " << recipient);
  uint8_t tid = respHdr->GetTid ();
  if (m_baManager->ExistsAgreementInState (recipient, tid, OriginatorBlockAckAgreement::PENDING))
    {
      if (respHdr->GetStatusCode ().IsSuccess ())
        {
          NS_LOG_DEBUG ("block ack agreement established with " << recipient);
          m_baManager->UpdateAgreement (respHdr, recipient);
        }
      else
        {
          NS_LOG_DEBUG ("discard ADDBA response" << recipient);
          m_baManager->NotifyAgreementUnsuccessful (recipient, tid);
        }
    }
  RestartAccessIfNeeded ();
}

void
EdcaTxopN::GotDelBaFrame (const MgtDelBaHeader *delBaHdr, Mac48Address recipient)
{
  NS_LOG_FUNCTION (this << delBaHdr << recipient);
  NS_LOG_DEBUG ("received DELBA frame from=" << recipient);
  m_baManager->TearDownBlockAck (recipient, delBaHdr->GetTid ());
}

void
EdcaTxopN::GotBlockAck (const CtrlBAckResponseHeader *blockAck, Mac48Address recipient)
{
  NS_LOG_FUNCTION (this << blockAck << recipient);
  NS_LOG_DEBUG ("got block ack from=" << recipient);
  uint16_t results[64];
  uint16_t results_MPDU[64];
  uint16_t n_success = 0;
  for(int j=0; j<64; j++)
  {
    results[j]=0;
    results_MPDU[j]=0;
  }
  uint16_t nframes=0;
  nframes = m_stationManager->GetNframes(m_currentHdr.GetAddr1(), &m_currentHdr);
  m_baManager->NotifyGotBlockAck (blockAck, recipient,results,results_MPDU);
  uint16_t results_[65];
  for(int j=0; j<65; j++)
    results_[j]=0;
  results_[0]=nframes;
  for(int i=1; i<nframes+1; i++) {
    results_[i]=results[i-1];
	  n_success+=results_[i];
  }
  NS_LOG_DEBUG ("Remove m_currentPacket, number of MPDUs in m_currentPacket=" << nframes << " n_suc=" << n_success);
	
  m_stationManager->UpdateStatistics(m_currentHdr.GetAddr1(), &m_currentHdr, nframes,n_success,m_stationManager->IsSampling(m_currentHdr.GetAddr1(), &m_currentHdr));

	m_currentPacket = 0;
  NS_LOG_DEBUG ("ResetCw!");
  m_dcf->ResetCw ();
  
  //shbyeon txop implementation
  if(m_stationManager->GetTxop(m_currentHdr.GetAddr1(), &m_currentHdr) > 0)
  {
    m_dcf->StartBackoffNow (m_rng->GetNext (0,0));
    RestartAccessIfNeeded (true);
  }
  else
  {
    m_dcf->StartBackoffNow (m_rng->GetNext (0, m_dcf->GetCw ()));
    RestartAccessIfNeeded ();
  }
}

void
EdcaTxopN::VerifyBlockAck (void)
{
  NS_LOG_FUNCTION (this);
  uint8_t tid = m_currentHdr.GetQosTid ();
  Mac48Address recipient = m_currentHdr.GetAddr1 ();
  uint16_t sequence = m_currentHdr.GetSequenceNumber ();
  if (m_baManager->ExistsAgreementInState (recipient, tid, OriginatorBlockAckAgreement::INACTIVE))
    {
      m_baManager->SwitchToBlockAckIfNeeded (recipient, tid, sequence);
    }
  if (m_baManager->ExistsAgreementInState (recipient, tid, OriginatorBlockAckAgreement::ESTABLISHED))
    {
  		NS_LOG_DEBUG ("set to blockack");
			if(GetImplicitBlockAckRequest()) {
					m_currentHdr.SetQosAckPolicy (WifiMacHeader::NORMAL_ACK);
			} else {
					m_currentHdr.SetQosAckPolicy (WifiMacHeader::BLOCK_ACK);
			}

    }
}

void
EdcaTxopN::CompleteTx (void)
{
  NS_LOG_FUNCTION (this);
  if (m_currentHdr.IsQosData () && m_currentHdr.IsQosBlockAck ())
    {
      if (!m_currentHdr.IsRetry ())
        {
          m_baManager->StorePacket (m_currentPacket, m_currentHdr, m_currentPacketTimestamp);
        }
      m_baManager->NotifyMpduTransmission (m_currentHdr.GetAddr1 (), m_currentHdr.GetQosTid (),
                                           m_txMiddle->GetNextSeqNumberByTidAndAddress (m_currentHdr.GetQosTid (),
                                                                                        m_currentHdr.GetAddr1 ()));
      if (!GetImplicitBlockAckRequest ())
      {
        NS_LOG_DEBUG ("Remove m_currentPacket");
        m_currentPacket = 0;
      }

      NS_LOG_DEBUG ("ResetCw!");
      m_dcf->ResetCw ();
      m_dcf->StartBackoffNow (m_rng->GetNext (0, m_dcf->GetCw ()));
      StartAccessIfNeeded ();
    }
}

bool
EdcaTxopN::SetupBlockAckIfNeeded ()
{
  NS_LOG_FUNCTION (this);
  uint8_t tid = m_currentHdr.GetQosTid ();
  Mac48Address recipient = m_currentHdr.GetAddr1 ();

  uint32_t packets = m_queue->GetNPacketsByTidAndAddress (tid, WifiMacHeader::ADDR1, recipient);

  if (packets >= m_blockAckThreshold)
    {
      /* Block ack setup */
      uint16_t startingSequence = m_txMiddle->GetNextSeqNumberByTidAndAddress (tid, recipient);
      SendAddBaRequest (recipient, tid, startingSequence, m_blockAckInactivityTimeout, true);
      return true;
    }
  return false;
}

void
EdcaTxopN::SendBlockAckRequest (const struct Bar &bar)
{
  NS_LOG_FUNCTION (this << &bar);
  NS_LOG_DEBUG("send blockackreq here");
  WifiMacHeader hdr;
  hdr.SetType (WIFI_MAC_CTL_BACKREQ);
  hdr.SetAddr1 (bar.recipient);
  hdr.SetAddr2 (m_low->GetAddress ());
  hdr.SetAddr3 (m_low->GetBssid ());
  hdr.SetDsNotTo ();
  hdr.SetDsNotFrom ();
  hdr.SetNoRetry ();
  hdr.SetNoMoreFragments ();

  m_currentPacket = bar.bar;
  m_currentHdr = hdr;

  MacLowTransmissionParameters params;
  params.DisableRts ();
  params.DisableNextData ();
  params.DisableOverrideDurationId ();
  if (bar.immediate)
    {
      if (m_blockAckType == BASIC_BLOCK_ACK)
        {
          params.EnableBasicBlockAck ();
        }
      else if (m_blockAckType == COMPRESSED_BLOCK_ACK)
        {
          params.EnableCompressedBlockAck ();
        }
      else if (m_blockAckType == MULTI_TID_BLOCK_ACK)
        {
          NS_FATAL_ERROR ("Multi-tid block ack is not supported");
        }
    }
  else
    {
      //Delayed block ack
      params.EnableAck ();
    }

	//802.11ac channel bonding
	if(!SecondaryIdle (m_stationManager->GetRxOperationalBandwidth (bar.recipient, &m_currentHdr)))
		return;

  m_low->StartTransmission (m_currentPacket, &m_currentHdr, params, m_transmissionListener);
}

WifiMacHeader 
EdcaTxopN::SendBlockAckRequestWithAmpdu (const struct Bar &bar)
{
  NS_LOG_FUNCTION (this);
  WifiMacHeader hdr;
  hdr.SetType (WIFI_MAC_CTL_BACKREQ);
  hdr.SetAddr1 (bar.recipient);
  hdr.SetAddr2 (m_low->GetAddress ());
  hdr.SetAddr3 (m_low->GetBssid ());
  hdr.SetDsNotTo ();
  hdr.SetDsNotFrom ();
  hdr.SetNoRetry ();
  hdr.SetNoMoreFragments ();

  m_currentPacket = bar.bar;
  m_currentHdr = hdr;
	
  MacLowTransmissionParameters params;
  params.DisableRts ();
  params.DisableNextData ();
  params.DisableOverrideDurationId ();
  if (bar.immediate)
    {
      if (m_blockAckType == BASIC_BLOCK_ACK)
        {
          params.EnableBasicBlockAck ();
        }
      else if (m_blockAckType == COMPRESSED_BLOCK_ACK)
        {
          params.EnableCompressedBlockAck ();
        }
      else if (m_blockAckType == MULTI_TID_BLOCK_ACK)
        {
          NS_FATAL_ERROR ("Multi-tid block ack is not supported");
        }
    }
  else
    {
      //Delayed block ack
      params.EnableAck ();
    }
	return hdr;
}

void
EdcaTxopN::CompleteConfig (void)
{
  NS_LOG_FUNCTION (this);
  m_baManager->SetTxMiddle (m_txMiddle);
  m_low->RegisterBlockAckListenerForAc (m_ac, m_blockAckListener);
  m_baManager->SetBlockAckInactivityCallback (MakeCallback (&EdcaTxopN::SendDelbaFrame, this));
}

void
EdcaTxopN::SetBlockAckThreshold (uint8_t threshold)
{
  NS_LOG_FUNCTION (this << static_cast<uint32_t> (threshold));
  m_blockAckThreshold = threshold;
  m_baManager->SetBlockAckThreshold (threshold);
}

void
EdcaTxopN::SetBlockAckInactivityTimeout (uint16_t timeout)
{
  NS_LOG_FUNCTION (this << timeout);
  m_blockAckInactivityTimeout = timeout;
}

uint8_t
EdcaTxopN::GetBlockAckThreshold (void) const
{
  NS_LOG_FUNCTION (this);
  return m_blockAckThreshold;
}
//shbyeon set/get implicit blockackreq
bool
EdcaTxopN::GetImplicitBlockAckRequest (void) const
{
  return m_implicitBlockAckRequest;
}
void
EdcaTxopN::SetImplicitBlockAckRequest (bool implicit) 
{
  m_implicitBlockAckRequest = implicit;
  m_baManager->SetImplicitBlockAckRequest (implicit);
}

void
EdcaTxopN::SendAddBaRequest (Mac48Address dest, uint8_t tid, uint16_t startSeq,
                             uint16_t timeout, bool immediateBAck)
{
  NS_LOG_FUNCTION (this << dest << static_cast<uint32_t> (tid) << startSeq << timeout << immediateBAck);
  WifiMacHeader hdr;
  hdr.SetAction ();
  hdr.SetAddr1 (dest);
  hdr.SetAddr2 (m_low->GetAddress ());
  hdr.SetAddr3 (m_low->GetAddress ());
  hdr.SetDsNotTo ();
  hdr.SetDsNotFrom ();

  WifiActionHeader actionHdr;
  WifiActionHeader::ActionValue action;
  action.blockAck = WifiActionHeader::BLOCK_ACK_ADDBA_REQUEST;
  actionHdr.SetAction (WifiActionHeader::BLOCK_ACK, action);
  NS_LOG_DEBUG ("sent ADDBA request to " << dest << "ActionHeader category: " << actionHdr.GetCategory());

  Ptr<Packet> packet = Create<Packet> ();
  /*Setting ADDBARequest header*/
  MgtAddBaRequestHeader reqHdr;
  reqHdr.SetAmsduSupport (true);
  if (immediateBAck)
    {
      reqHdr.SetImmediateBlockAck ();
    }
  else
    {
      reqHdr.SetDelayedBlockAck ();
    }
  reqHdr.SetTid (tid);
  /* For now we don't use buffer size field in the ADDBA request frame. The recipient
   * will choose how many packets it can receive under block ack.
   */
  reqHdr.SetBufferSize (0);
  reqHdr.SetTimeout (timeout);
  reqHdr.SetStartingSequence (startSeq);

  m_baManager->CreateAgreement (&reqHdr, dest);

  packet->AddHeader (reqHdr);
  packet->AddHeader (actionHdr);

  m_currentPacket = packet;
  m_currentHdr = hdr;

  uint16_t sequence = m_txMiddle->GetNextSequenceNumberfor (&m_currentHdr);
  m_currentHdr.SetSequenceNumber (sequence);
  m_currentHdr.SetFragmentNumber (0);
  m_currentHdr.SetNoMoreFragments ();
  m_currentHdr.SetNoRetry ();

  MacLowTransmissionParameters params;
  params.EnableAck ();
  params.DisableRts ();
  params.DisableNextData ();
  params.DisableOverrideDurationId ();
     
  m_low->StartTransmission (m_currentPacket, &m_currentHdr, params,
                            m_transmissionListener);
}

void
EdcaTxopN::SendDelbaFrame (Mac48Address addr, uint8_t tid, bool byOriginator)
{
  NS_LOG_FUNCTION (this << addr << static_cast<uint32_t> (tid) << byOriginator);
  WifiMacHeader hdr;
  hdr.SetAction ();
  hdr.SetAddr1 (addr);
  hdr.SetAddr2 (m_low->GetAddress ());
  hdr.SetAddr3 (m_low->GetAddress ());
  hdr.SetDsNotTo ();
  hdr.SetDsNotFrom ();

  MgtDelBaHeader delbaHdr;
  delbaHdr.SetTid (tid);
  if (byOriginator)
    {
      delbaHdr.SetByOriginator ();
    }
  else
    {
      delbaHdr.SetByRecipient ();
    }

  WifiActionHeader actionHdr;
  WifiActionHeader::ActionValue action;
  action.blockAck = WifiActionHeader::BLOCK_ACK_DELBA;
  actionHdr.SetAction (WifiActionHeader::BLOCK_ACK, action);

  Ptr<Packet> packet = Create<Packet> ();
  packet->AddHeader (delbaHdr);
  packet->AddHeader (actionHdr);

  PushFront (packet, hdr);
}

int64_t
EdcaTxopN::AssignStreams (int64_t stream)
{
  NS_LOG_FUNCTION (this << stream);
  m_rng->AssignStreams (stream);
  return 1;
}

void
EdcaTxopN::SetTxing (bool txing)
{
	m_manager->SetTxing(txing);
}

void
EdcaTxopN::DoInitialize ()
{
  NS_LOG_FUNCTION (this);
  NS_LOG_DEBUG ("ResetCw!   cwmin: " << GetMinCw() << " cwmax: " << GetMaxCw() << " aifsn: " << GetAifsn()); 
  m_dcf->ResetCw ();
  m_dcf->StartBackoffNow (m_rng->GetNext (0, m_dcf->GetCw ()));
  ns3::Dcf::DoInitialize ();
}
} // namespace ns3
