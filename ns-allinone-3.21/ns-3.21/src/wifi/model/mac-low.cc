/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2005,2006 INRIA
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

#include "ns3/assert.h"
#include "ns3/packet.h"
#include "ns3/simulator.h"
#include "ns3/tag.h"
#include "ns3/log.h"
#include "ns3/node.h"
#include "ns3/double.h"

#include "mac-low.h"
#include "wifi-phy.h"
#include "wifi-mac-trailer.h"
#include "qos-utils.h"
#include "edca-txop-n.h"
#include "snr-tag.h"

//shbyeon ampdu
#include "mpdu-aggregator.h"
#include "wifi-mac-queue.h"
#include "ns3/arp-header.h"
#include "ns3/ampdu-tag.h"
#include "wifi-mac-header.h"
#include "error-free-tag.h"
#include "wifi-bonding.h"
#include "ns3/duplicate-tag.h"

//jwhur
#include "yans-wifi-phy.h"


NS_LOG_COMPONENT_DEFINE ("MacLow");

#undef NS_LOG_APPEND_CONTEXT
#define NS_LOG_APPEND_CONTEXT std::clog << "[mac=" << m_self << "] "


namespace ns3{ 

    MacLowTransmissionListener::MacLowTransmissionListener ()
    {
    }
    MacLowTransmissionListener::~MacLowTransmissionListener ()
    {
    }
    void
        MacLowTransmissionListener::GotBlockAck (const CtrlBAckResponseHeader *blockAck,
                Mac48Address source)
        {
        }
    void
        MacLowTransmissionListener::MissedBlockAck (void)
        {
        }
    //shbyeon
    void
        MacLowTransmissionListener::NotifyCollision(void)
        {
        }

    MacLowDcfListener::MacLowDcfListener ()
    {
    }
    MacLowDcfListener::~MacLowDcfListener ()
    {
    }

    MacLowBlockAckEventListener::MacLowBlockAckEventListener ()
    {
    }
    MacLowBlockAckEventListener::~MacLowBlockAckEventListener ()
    {
    }

    MacLowTransmissionParameters::MacLowTransmissionParameters ()
        : m_nextSize (0),
        m_waitAck (ACK_NONE),
        m_sendRts (false),
        m_overrideDurationId (Seconds (0))
    {
    }
    void
        MacLowTransmissionParameters::EnableNextData (uint32_t size)
        {
            m_nextSize = size;
        }
    void
        MacLowTransmissionParameters::DisableNextData (void)
        {
            m_nextSize = 0;
        }
    void
        MacLowTransmissionParameters::EnableOverrideDurationId (Time durationId)
        {
            m_overrideDurationId = durationId;
        }
    void
        MacLowTransmissionParameters::DisableOverrideDurationId (void)
        {
            m_overrideDurationId = Seconds (0);
        }
    void
        MacLowTransmissionParameters::EnableSuperFastAck (void)
        {
            m_waitAck = ACK_SUPER_FAST;
        }
    void
        MacLowTransmissionParameters::EnableBasicBlockAck (void)
        {
            m_waitAck = BLOCK_ACK_BASIC;
        }
    void
        MacLowTransmissionParameters::EnableCompressedBlockAck (void)
        {
            m_waitAck = BLOCK_ACK_COMPRESSED;
        }
    void
        MacLowTransmissionParameters::EnableMultiTidBlockAck (void)
        {
            m_waitAck = BLOCK_ACK_MULTI_TID;
        }
    void
        MacLowTransmissionParameters::EnableFastAck (void)
        {
            m_waitAck = ACK_FAST;
        }
    void
        MacLowTransmissionParameters::EnableAck (void)
        {
            m_waitAck = ACK_NORMAL;
        }
    void
        MacLowTransmissionParameters::DisableAck (void)
        {
            m_waitAck = ACK_NONE;
        }
    void
        MacLowTransmissionParameters::EnableRts (void)
        {
            m_sendRts = true;
        }
    void
        MacLowTransmissionParameters::DisableRts (void)
        {
            m_sendRts = false;
        }
    bool
        MacLowTransmissionParameters::MustWaitAck (void) const
        {
            return (m_waitAck != ACK_NONE);
        }
    bool
        MacLowTransmissionParameters::MustWaitNormalAck (void) const
        {
            return (m_waitAck == ACK_NORMAL);
        }
    bool
        MacLowTransmissionParameters::MustWaitFastAck (void) const
        {
            return (m_waitAck == ACK_FAST);
        }
    bool
        MacLowTransmissionParameters::MustWaitSuperFastAck (void) const
        {
            return (m_waitAck == ACK_SUPER_FAST);
        }
    bool
        MacLowTransmissionParameters::MustWaitBasicBlockAck (void) const
        {
            return (m_waitAck == BLOCK_ACK_BASIC) ? true : false;
        }
    bool
        MacLowTransmissionParameters::MustWaitCompressedBlockAck (void) const
        {
            return (m_waitAck == BLOCK_ACK_COMPRESSED) ? true : false;
        }
    bool
        MacLowTransmissionParameters::MustWaitMultiTidBlockAck (void) const
        {
            return (m_waitAck == BLOCK_ACK_MULTI_TID) ? true : false;
        }
    bool
        MacLowTransmissionParameters::MustSendRts (void) const
        {
            return m_sendRts;
        }
    bool
        MacLowTransmissionParameters::HasDurationId (void) const
        {
            return (m_overrideDurationId != Seconds (0));
        }
    Time
        MacLowTransmissionParameters::GetDurationId (void) const
        {
            NS_ASSERT (m_overrideDurationId != Seconds (0));
            return m_overrideDurationId;
        }
    bool
        MacLowTransmissionParameters::HasNextPacket (void) const
        {
            return (m_nextSize != 0);
        }
    uint32_t
        MacLowTransmissionParameters::GetNextPacketSize (void) const
        {
            NS_ASSERT (HasNextPacket ());
            return m_nextSize;
        }

    std::ostream &operator << (std::ostream &os, const MacLowTransmissionParameters &params)
    {
        os << "["
            << "send rts=" << params.m_sendRts << ", "
            << "next size=" << params.m_nextSize << ", "
            << "dur=" << params.m_overrideDurationId << ", "
            << "ack=";
        switch (params.m_waitAck)
        {
            case MacLowTransmissionParameters::ACK_NONE:
                os << "none";
                break;
            case MacLowTransmissionParameters::ACK_NORMAL:
                os << "normal";
                break;
            case MacLowTransmissionParameters::ACK_FAST:
                os << "fast";
                break;
            case MacLowTransmissionParameters::ACK_SUPER_FAST:
                os << "super-fast";
                break;
            case MacLowTransmissionParameters::BLOCK_ACK_BASIC:
                os << "basic-block-ack";
                break;
            case MacLowTransmissionParameters::BLOCK_ACK_COMPRESSED:
                os << "compressed-block-ack";
                break;
            case MacLowTransmissionParameters::BLOCK_ACK_MULTI_TID:
                os << "multi-tid-block-ack";
                break;
        }
        os << "]";
        return os;
    }


    /**
     * Listener for PHY events. Forwards to MacLow
     */
    class PhyMacLowListener : public ns3::WifiPhyListener
    {
        public:
            /**
             * Create a PhyMacLowListener for the given MacLow.
             *
             * \param macLow
             */
            PhyMacLowListener (ns3::MacLow *macLow)
                : m_macLow (macLow)
            {
            }
            virtual ~PhyMacLowListener ()
            {
            }
            virtual void NotifyRxStart (Time duration)
            {
            }
            virtual void NotifyRxEndOk (void)
            {
            }
            virtual void NotifyRxEndError (void)
            {
            }
            virtual void NotifyTxStart (Time duration, double txPowerDbm)
            {
            }
            virtual void NotifyMaybeCcaBusyStart (Time duration)
            {
            }
            virtual void NotifySwitchingStart (Time duration)
            {
                m_macLow->NotifySwitchingStartNow (duration);
            }
            virtual void NotifySleep (void)
            {
                m_macLow->NotifySleepNow ();
            }
            virtual void NotifyWakeup (void)
            {
            }
        private:
            ns3::MacLow *m_macLow;
    };


    MacLow::MacLow ()
        : m_operationalBandwidth (80),
        m_da(0),
        m_normalAckTimeoutEvent (),
        m_fastAckTimeoutEvent (),
        m_superFastAckTimeoutEvent (),
        m_fastAckFailedTimeoutEvent (),
        m_blockAckTimeoutEvent (),
        m_ctsTimeoutEvent (),
        m_sendCtsEvent (),
        m_sendAckEvent (),
        m_sendDataEvent (),
        m_waitSifsEvent (),
        m_endTxNoAckEvent (),
        m_currentPacket (0),
        m_listener (0),
        m_phyMacLowListener (0),
        m_ctsToSelfSupported (false)
    {
        NS_LOG_FUNCTION (this);
        m_lastNavDuration = Seconds (0);
        m_lastNavStart = Seconds (0);
        m_promisc = false;
        //shbyeon set maxppdutime to 10 ms by default
        //but 802.11ac defines it as 5.484 ms
        //m_maxPpduTime = MilliSeconds (10);
        m_maxPpduTime = Seconds (0.005484);

        //JWHUR err rate calc.
        //err_count = 0;
        //rx_count = 0;
        //err_rate = 0;
        to1 = 0, to2 = 0;
    }

    MacLow::~MacLow ()
    {
        NS_LOG_FUNCTION (this);
    }

    void
        MacLow::SetupPhyMacLowListener (Ptr<WifiPhy> phy)
        {
            m_phyMacLowListener = new PhyMacLowListener (this);
            phy->RegisterListener (m_phyMacLowListener);
        }


    void
        MacLow::DoDispose (void)
        {
            //JWHUR err rate calc.
            //Ptr<YansWifiPhy> wifiPhy = DynamicCast<YansWifiPhy>(m_phy);
            //uint32_t nodeId = wifiPhy->nodeId;
            //err_rate = err_count / (rx_count + err_count);
            //std::cout << "NodeId: " << nodeId << ", err_rate: " << err_rate << "\n";

            NS_LOG_FUNCTION (this);
            m_normalAckTimeoutEvent.Cancel ();
            m_fastAckTimeoutEvent.Cancel ();
            m_superFastAckTimeoutEvent.Cancel ();
            m_fastAckFailedTimeoutEvent.Cancel ();
            m_blockAckTimeoutEvent.Cancel ();
            m_ctsTimeoutEvent.Cancel ();
            m_sendCtsEvent.Cancel ();
            m_sendAckEvent.Cancel ();
            m_sendDataEvent.Cancel ();
            m_waitSifsEvent.Cancel ();
            m_endTxNoAckEvent.Cancel ();
            m_waitRifsEvent.Cancel();
            m_phy = 0;
            m_stationManager = 0;
            if (m_phyMacLowListener != 0)
            {
                delete m_phyMacLowListener;
                m_phyMacLowListener = 0;
            }

        }

    //802.11ac channel bonding
    bool
        MacLow::RunningAckEvent(Time rxDuration, Ptr<Packet> packet)
        {
            bool waiting = false;
            bool sending = false;
            bool isData = false;

            WifiMacHeader hdr;
            packet->PeekHeader(hdr);
            if(!packet->PeekHeader(hdr))
                return false;

            bool IsQosData = hdr.IsQosData();
            AmpduTag ampdutag;
            bool isAmpdu= packet->PeekPacketTag (ampdutag);

            waiting = (m_normalAckTimeoutEvent.IsRunning() ||
                    m_blockAckTimeoutEvent.IsRunning() ||
                    m_ctsTimeoutEvent.IsRunning());

            isData = (IsQosData || isAmpdu);

            sending = m_sendAckEvent.IsRunning() ||
                m_sendCtsEvent.IsRunning() ||
                m_sendDataEvent.IsRunning() ||
                m_waitSifsEvent.IsRunning();

            NS_LOG_DEBUG(m_self << " " << rxDuration << " wating=" << waiting << " sending=" << sending << " isdata=" << isData);
            return ((waiting && isData) || sending);
        }


    void
        MacLow::CancelAllEvents (void)
        {
            NS_LOG_FUNCTION (this);
            bool oneRunning = false;
            if (m_normalAckTimeoutEvent.IsRunning ())
            {
                m_normalAckTimeoutEvent.Cancel ();
                oneRunning = true;
            }
            if (m_fastAckTimeoutEvent.IsRunning ())
            {
                m_fastAckTimeoutEvent.Cancel ();
                oneRunning = true;
            }
            if (m_superFastAckTimeoutEvent.IsRunning ())
            {
                m_superFastAckTimeoutEvent.Cancel ();
                oneRunning = true;
            }
            if (m_fastAckFailedTimeoutEvent.IsRunning ())
            {
                m_fastAckFailedTimeoutEvent.Cancel ();
                oneRunning = true;
            }
            if (m_blockAckTimeoutEvent.IsRunning ())
            {
                //ohlee recovery virtual collision
                if(m_currentHdr.IsQosData()){   // only for ampdu failure not blockack req
                    Ptr<EdcaTxopN> m_edca = m_edcas[QosUtilsMapTidToAc (m_currentHdr.GetQosTid ())];
                    m_edca->GetBlockAckManager ()->NotifyBlockAckTimeoutCancel (m_currentHdr.GetQosTid (), m_currentHdr.GetAddr1 ());
                }
                m_listener->NotifyCollision();
                m_blockAckTimeoutEvent.Cancel ();
                oneRunning = true;
            }
            if (m_ctsTimeoutEvent.IsRunning ())
            {
                m_ctsTimeoutEvent.Cancel ();
                oneRunning = true;
            }
            if (m_sendCtsEvent.IsRunning ())
            {
                m_sendCtsEvent.Cancel ();
                oneRunning = true;
            }
            if (m_sendAckEvent.IsRunning ())
            {
                m_sendAckEvent.Cancel ();
                oneRunning = true;
            }
            if (m_sendDataEvent.IsRunning ())
            {
                m_sendDataEvent.Cancel ();
                oneRunning = true;
            }
            if (m_waitSifsEvent.IsRunning ())
            {
                m_waitSifsEvent.Cancel ();
                oneRunning = true;
            }
            if (m_waitRifsEvent.IsRunning ())
            {
                m_waitRifsEvent.Cancel ();
                oneRunning = true;
            }
            if (m_endTxNoAckEvent.IsRunning ()) 
            {
                m_endTxNoAckEvent.Cancel ();
                oneRunning = true;
            }
            if (oneRunning && m_listener != 0)
            {
                m_listener->Cancel ();
                m_listener = 0;
            }
        }

    //ohlee - make edca pointer
    void
        MacLow::SetEdcas(EdcaQueues edcas)
        {
            NS_LOG_FUNCTION(this);
            m_edcas=edcas;
        }

    void
        MacLow::SetPhy (Ptr<WifiPhy> phy)
        {
            m_phy = phy;
            m_phy->SetReceiveOkCallback (MakeCallback (&MacLow::ReceiveOk, this));
            m_phy->SetReceiveErrorCallback (MakeCallback (&MacLow::ReceiveError, this));
            SetupPhyMacLowListener (phy);
            //802.11ac channel bonding
            SetOperationalBandwidth(m_phy->GetOperationalBandwidth());
            SetDynamicAccess(m_phy->GetDynamicAccess());
        }
    void
        MacLow::SetWifiRemoteStationManager (Ptr<WifiRemoteStationManager> manager)
        {
            m_stationManager = manager;
        }

    void
        MacLow::SetAddress (Mac48Address ad)
        {
            m_self = ad;
        }
    void
        MacLow::SetAckTimeout (Time ackTimeout)
        {
            m_ackTimeout = ackTimeout;
        }
    void
        MacLow::SetBasicBlockAckTimeout (Time blockAckTimeout)
        {
            m_basicBlockAckTimeout = blockAckTimeout;
        }
    void
        MacLow::SetCompressedBlockAckTimeout (Time blockAckTimeout)
        {
            m_compressedBlockAckTimeout = blockAckTimeout;
        }
    void
        MacLow::SetCtsToSelfSupported (bool enable)
        {
            m_ctsToSelfSupported = enable;
        }
    bool
        MacLow::GetCtsToSelfSupported () const
        {
            return m_ctsToSelfSupported;
        }
    void
        MacLow::SetCtsTimeout (Time ctsTimeout)
        {
            m_ctsTimeout = ctsTimeout;
        }
    void
        MacLow::SetSifs (Time sifs)
        {
            m_sifs = sifs;
        }
    void
        MacLow::SetSlotTime (Time slotTime)
        {
            m_slotTime = slotTime;
        }
    void
        MacLow::SetPifs (Time pifs)
        {
            m_pifs = pifs;
        }
    void
        MacLow::SetRifs (Time rifs)
        {
            m_rifs = rifs;
        }
    void
        MacLow::SetBssid (Mac48Address bssid)
        {
            m_bssid = bssid;
        }
    void
        MacLow::SetPromisc (void)
        {
            m_promisc = true;
        }
    Mac48Address
        MacLow::GetAddress (void) const
        {
            return m_self;
        }
    //shbyeon get max propagation delay (default)
    Time
        MacLow::GetDefaultMaxPropagationDelay (void) const
        {
            // 1000m
            return Seconds (1000.0 / 300000000.0);
        }

    Time
        MacLow::GetAckTimeout (void) const
        {
            return m_ackTimeout;
        }
    Time
        MacLow::GetBasicBlockAckTimeout () const
        {
            return m_basicBlockAckTimeout;
        }
    Time
        MacLow::GetCompressedBlockAckTimeout () const
        {
            return m_compressedBlockAckTimeout;
        }
    Time
        MacLow::GetCtsTimeout (void) const
        {
            return m_ctsTimeout;
        }
    Time
        MacLow::GetSifs (void) const
        {
            return m_sifs;
        }
    Time
        MacLow::GetRifs (void) const
        {
            return m_rifs;
        }
    Time
        MacLow::GetSlotTime (void) const
        {
            return m_slotTime;
        }
    Time
        MacLow::GetPifs (void) const
        {
            return m_pifs;
        }
    Mac48Address
        MacLow::GetBssid (void) const
        {
            return m_bssid;
        }
    bool
        MacLow::IsPromisc (void) const
        {
            return m_promisc;
        }

    void
        MacLow::SetRxCallback (Callback<void,Ptr<Packet>,const WifiMacHeader *> callback)
        {
            m_rxCallback = callback;
        }
    void
        MacLow::RegisterDcfListener (MacLowDcfListener *listener)
        {
            m_dcfListeners.push_back (listener);
        }


    void
        MacLow::StartTransmission (Ptr<const Packet> packet,
                const WifiMacHeader* hdr,
                MacLowTransmissionParameters params,
                MacLowTransmissionListener *listener)
        {
            NS_LOG_FUNCTION (this << packet << hdr << params << listener);
            /* m_currentPacket is not NULL because someone started
             * a transmission and was interrupted before one of:
             *   - ctsTimeout
             *   - sendDataAfterCTS
             * expired. This means that one of these timers is still
             * running. They are all cancelled below anyway by the
             * call to CancelAllEvents (because of at least one
             * of these two timer) which will trigger a call to the
             * previous listener's cancel method.
             *
             * This typically happens because the high-priority
             * QapScheduler has taken access to the channel from
             * one of the Edca of the QAP.
             */

            if (m_currentPacket != 0)
            {
                NS_LOG_DEBUG ("currentPacketType=" << m_currentHdr.GetTypeString () << ", size=" <<
                        m_currentPacket->GetSize () << ", Addr1=" << m_currentHdr.GetAddr1 () <<
                        ", seq=" << m_currentHdr.GetSequenceNumber ());
                NS_LOG_DEBUG ("newPacketType=" << hdr->GetTypeString () << ", size=" <<
                        packet->GetSize () << ", Addr1=" << hdr->GetAddr1 () <<
                        ", seq=" << hdr->GetSequenceNumber ());
            }
            NS_ASSERT(m_currentPacket == 0);

            m_currentPacket = packet->Copy ();
            m_currentHdr = *hdr;
            //JWHUR edca queue collision solved
            m_edcas[AC_VI]->SetTxing(true);
            m_edcas[AC_VO]->SetTxing(true);
            m_edcas[AC_BE]->SetTxing(true);
            m_edcas[AC_BK]->SetTxing(true);
            CancelAllEvents ();
            m_listener = listener;
            m_txParams = params;

            AmpduTag ampdu;
            ErrorFreeTag eft;
            m_currentPacket->RemovePacketTag(ampdu);
            m_currentPacket->RemovePacketTag(eft);

            //NS_ASSERT (m_phy->IsStateIdle ());
            NS_LOG_DEBUG ("packet type=" << m_currentHdr.GetTypeString () << 
                    ", startTx size=" << GetSize (m_currentPacket, &m_currentHdr) <<
                    ", to=" << m_currentHdr.GetAddr1 () << ", listener=" << m_listener <<
                    ", seq=" << m_currentHdr.GetSequenceNumber () << 
                    ", seqCon" << m_currentHdr.GetSequenceControl ());

            //shbyeon aggregate mpdus and send ampdu
            bool isAmpdu = false;

            if (hdr->IsQosData ()
                    && !hdr->GetAddr1 ().IsBroadcast ()){
                Ptr<MpduAggregator> m_aggregator = m_aggregators[QosUtilsMapTidToAc (hdr->GetQosTid ())];
                //jwhur
                if( m_aggregator !=0 )
                {
                    isAmpdu = AggregateMpdu(m_aggregator);	
                }
            }

            if (isAmpdu)
            {
                NS_LOG_DEBUG("ampdu transmission!");
            }
            else if (m_txParams.MustSendRts () || m_stationManager->NeedRts (m_currentHdr.GetAddr1 (), &m_currentHdr,m_currentPacket))
            {
                SendRtsForPacket ();
            }
            else
            {
                if (NeedCtsToSelf() && m_ctsToSelfSupported)
                {
                    SendCtsToSelf();
                }
                else
                {
                    SendDataPacket ();
                }
            }
            //JWHUR edca queue collision solved
            m_edcas[AC_VI]->SetTxing(false);
            m_edcas[AC_VO]->SetTxing(false);
            m_edcas[AC_BE]->SetTxing(false);
            m_edcas[AC_BK]->SetTxing(false);

            /* When this method completes, we have taken ownership of the medium. */
            NS_ASSERT (m_phy->IsStateTx ());
        }

    bool
        MacLow::NeedCtsToSelf (void)
        {
            WifiTxVector dataTxVector = GetDataTxVector (m_currentPacket, &m_currentHdr);
            return m_stationManager->NeedCtsToSelf (dataTxVector);
        }

    //shbyeon add txmode as an input
    void
        MacLow::ReceiveError (Ptr<const Packet> packet, double rxSnr, WifiMode txMode, bool rxOneMPDU)
        {
            NS_LOG_FUNCTION (this << packet << rxSnr);
            NS_LOG_DEBUG ("rx failed ");

            //JWHUR error rate calc.
            err_count += 1;

            Ptr<Packet> receivedPacket = packet->Copy();
            WifiMacHeader hdr;
            receivedPacket->RemoveHeader (hdr);

            NS_LOG_DEBUG ("recvErr " << hdr.GetTypeString () <<
                    ", from=" << hdr.GetAddr2 () <<
                    ", size=" << packet->GetSize () <<
                    ", duration=" << hdr.GetDuration () <<
                    ", seq=" << hdr.GetSequenceNumber () << 
                    ", seqCon=" << hdr.GetSequenceControl ());
            AmpduTag tag;
            bool lastMpdu = false;
            bool isAmpdu= packet->PeekPacketTag (tag);
            if(isAmpdu)
                lastMpdu = tag.Get ();

            if (m_txParams.MustWaitFastAck ())
            {
                NS_ASSERT (m_fastAckFailedTimeoutEvent.IsExpired ());
                m_fastAckFailedTimeoutEvent = Simulator::Schedule (GetSifs (),
                        &MacLow::FastAckFailedTimeout, this);
            }

            if (rxOneMPDU && lastMpdu && hdr.GetAddr1 () == m_self)
            {
                NS_LOG_DEBUG ("got last AMPDU subframe from " << hdr.GetAddr2 ());
                if (hdr.IsQosData ())
                {
                    if (hdr.IsQosAck ()) //if ACK policy is normal ACK //implicit BAR
                    {
                        NS_LOG_DEBUG ("implicit BAR");
                        uint8_t tid = hdr.GetQosTid ();
                        AgreementsI it = m_bAckAgreements.find (std::make_pair (hdr.GetAddr2 (), tid));
                        if (it != m_bAckAgreements.end ())
                        {
                            //Update block ack cache
                            BlockAckCachesI i = m_bAckCaches.find (std::make_pair (hdr.GetAddr2 (), tid));
                            NS_ASSERT (i != m_bAckCaches.end ());
                            (*i).second.UpdateWithBlockAckReq (it->second.first.GetStartingSequence ()); //modified to get Starting Sequence 
                            NS_ASSERT (m_sendAckEvent.IsExpired ());
                            ResetBlockAckInactivityTimerIfNeeded (it->second.first);
                            if ((*it).second.first.IsImmediateBlockAck ())
                            {
                                NS_LOG_DEBUG ("implicit blockAckRequest/sendImmediateBlockAck");
                                m_sendAckEvent = Simulator::Schedule (GetSifs (),
                                        &MacLow::SendBlockAckWithoutBlockAckRequest, this,
                                        hdr.GetAddr2 (),
                                        hdr.GetDuration (),
                                        txMode, 
                                        tid,
                                        it->second.first.GetStartingSequence ());
                            }
                            else
                            {
                                NS_FATAL_ERROR ("Delayed block ack not supported.");
                            }
                        }
                        else
                        {
                            NS_LOG_DEBUG ("There's not a valid agreement for this block ack request.");
                        }
                    }
                    else if (hdr.IsQosBlockAck ())
                    {
                        NS_LOG_DEBUG ("last AMPDU subframe/ack policy of block ack/not send block ack");
                        AgreementsI it = m_bAckAgreements.find (std::make_pair (hdr.GetAddr2 (), hdr.GetQosTid ()));
                        ResetBlockAckInactivityTimerIfNeeded (it->second.first);
                    }
                    return;
                }
            }
            return;
        }

    void
        MacLow::NotifySwitchingStartNow (Time duration)
        {
            NS_LOG_DEBUG ("switching channel. Cancelling MAC pending events");
            m_stationManager->Reset ();
            CancelAllEvents ();
            if (m_navCounterResetCtsMissed.IsRunning ())
            {
                m_navCounterResetCtsMissed.Cancel ();
            }
            m_lastNavStart = Simulator::Now ();
            m_lastNavDuration = Seconds (0);
            m_currentPacket = 0;
            m_listener = 0;
        }

    void
        MacLow::NotifySleepNow (void)
        {
            NS_LOG_DEBUG ("Device in sleep mode. Cancelling MAC pending events");
            m_stationManager->Reset ();
            CancelAllEvents ();
            if (m_navCounterResetCtsMissed.IsRunning ())
            {
                m_navCounterResetCtsMissed.Cancel ();
            }
            m_lastNavStart = Simulator::Now ();
            m_lastNavDuration = Seconds (0);
            m_currentPacket = 0;
            m_listener = 0;
        }

    void
        MacLow::ReceiveOk (Ptr<Packet> packet, double rxSnr, WifiMode txMode, WifiPreamble preamble)
        {
            NS_LOG_FUNCTION (this << packet << rxSnr << txMode << preamble);
            /* A packet is received from the PHY.
             * When we have handled this packet,
             * we handle any packet present in the
             * packet queue.
             */

            //JWHUR rx error calc.
            rx_count += 1;

            WifiMacHeader hdr;
            packet->RemoveHeader (hdr);

            bool isPrevNavZero = IsNavZero ();
            NS_LOG_DEBUG ("recvOk " << hdr.GetTypeString () <<
                    ", from=" << hdr.GetAddr2 () <<
                    ", size=" << packet->GetSize () <<
                    ", duration=" << hdr.GetDuration () <<
                    ", seq=" << hdr.GetSequenceNumber () << 
                    ", seqCon=" << hdr.GetSequenceControl ());
            NotifyNav (packet,hdr, txMode, preamble);

            AmpduTag ampdutag;
            bool lastMpdu = false;
            bool isAmpdu= packet->PeekPacketTag (ampdutag);
            if(isAmpdu)
                lastMpdu = ampdutag.Get ();

            if (hdr.IsRts ())
            {
                /* see section 9.2.5.7 802.11-1999
                 * A STA that is addressed by an RTS frame shall transmit a CTS frame after a SIFS
                 * period if the NAV at the STA receiving the RTS frame indicates that the medium is
                 * idle. If the NAV at the STA receiving the RTS indicates the medium is not idle,
                 * that STA shall not respond to the RTS frame.
                 */

                //802.11ac channel bonding
                DuplicateTag dtag;
                packet->RemovePacketTag(dtag);
                uint16_t txBandwidth = dtag.GetDuplicateBandwidth();
                uint16_t rxBandwidth = dtag.GetDuplicateBandwidthResult();
                NS_LOG_DEBUG("receive duplicate RTS, txBandwidth=" << txBandwidth << " rxBandwidth="<<rxBandwidth);
                NS_ASSERT(rxBandwidth != 0);

                if (isPrevNavZero
                        && hdr.GetAddr1 () == m_self)
                {
                    NS_LOG_DEBUG ("rx RTS from=" << hdr.GetAddr2 () << ", schedule CTS");
                    NS_ASSERT (m_sendCtsEvent.IsExpired ());
                    m_stationManager->ReportRxOk (hdr.GetAddr2 (), &hdr,
                            rxSnr, txMode);
                    m_sendCtsEvent = Simulator::Schedule (GetSifs (),
                            &MacLow::SendCtsAfterRts, this,
                            hdr.GetAddr2 (),
                            hdr.GetDuration (),
                            txMode,
                            rxSnr,
                            txBandwidth);
                }
                else
                {
                    NS_LOG_DEBUG ("rx RTS from=" << hdr.GetAddr2 () << ", cannot schedule CTS");
                }
            }
            else if (hdr.IsCts ()
                    && hdr.GetAddr1 () == m_self
                    && m_ctsTimeoutEvent.IsRunning ()
                    && m_currentPacket != 0)
            {
                //802.11ac channel bonding
                DuplicateTag dtag;
                packet->RemovePacketTag(dtag);
                uint16_t txBandwidth = dtag.GetDuplicateBandwidth();
                //		uint16_t rxBandwidth = dtag.GetDuplicateBandwidthResult();
                NS_LOG_DEBUG ("receive cts from=" << m_currentHdr.GetAddr1 ());
                SnrTag tag;
                packet->RemovePacketTag (tag);
                m_stationManager->ReportRxOk (m_currentHdr.GetAddr1 (), &m_currentHdr,
                        rxSnr, txMode);
                m_stationManager->ReportRtsOk (m_currentHdr.GetAddr1 (), &m_currentHdr,
                        rxSnr, txMode, tag.Get ());

                m_ctsTimeoutEvent.Cancel ();
                NotifyCtsTimeoutResetNow ();
                m_listener->GotCts (rxSnr, txMode);
                NS_ASSERT (m_sendDataEvent.IsExpired ());
                NS_LOG_DEBUG("current Ampdu?: "<<m_currentPacket->PeekPacketTag(ampdutag) << "("<<m_currentPacket <<")" );
                if(m_currentPacket->PeekPacketTag(ampdutag)){
                    m_sendDataEvent = Simulator::Schedule (GetSifs (),
                            &MacLow::SendAmpduAfterCts, this,
                            hdr.GetAddr1 (),
                            hdr.GetDuration (),
                            txMode, txBandwidth);

                } 
                else
                {
                    m_sendDataEvent = Simulator::Schedule (GetSifs (),
                            &MacLow::SendDataAfterCts, this,
                            hdr.GetAddr1 (),
                            hdr.GetDuration (),
                            txMode, txBandwidth);
                }
            }
            else if (hdr.IsAck ()
                    && hdr.GetAddr1 () == m_self
                    && (m_normalAckTimeoutEvent.IsRunning ()
                        || m_fastAckTimeoutEvent.IsRunning ()
                        || m_superFastAckTimeoutEvent.IsRunning ())
                    && m_txParams.MustWaitAck ())
            {
                DuplicateTag dtag;
                packet->RemovePacketTag (dtag);

                NS_LOG_DEBUG ("receive ack from=" << m_currentHdr.GetAddr1 ());
                SnrTag tag;
                packet->RemovePacketTag (tag);
                m_stationManager->ReportRxOk (m_currentHdr.GetAddr1 (), &m_currentHdr,
                        rxSnr, txMode);
                m_stationManager->ReportDataOk (m_currentHdr.GetAddr1 (), &m_currentHdr,
                        rxSnr, txMode, tag.Get ());
                bool gotAck = false;
                if (m_txParams.MustWaitNormalAck ()
                        && m_normalAckTimeoutEvent.IsRunning ())
                {
                    m_normalAckTimeoutEvent.Cancel ();
                    NotifyAckTimeoutResetNow ();
                    gotAck = true;
                    NS_LOG_DEBUG ("receive normal ack");
                }
                if (m_txParams.MustWaitFastAck ()
                        && m_fastAckTimeoutEvent.IsRunning ())
                {
                    m_fastAckTimeoutEvent.Cancel ();
                    NotifyAckTimeoutResetNow ();
                    gotAck = true;
                }
                if (gotAck)
                {
                    m_listener->GotAck (rxSnr, txMode);
                }

                if (m_txParams.HasNextPacket ())
                {
                    m_waitSifsEvent = Simulator::Schedule (GetSifs (),
                            &MacLow::WaitSifsAfterEndTx, this);
                }
            }
            else if (hdr.IsBlockAck () && hdr.GetAddr1 () == m_self
                    && (m_txParams.MustWaitBasicBlockAck () || m_txParams.MustWaitCompressedBlockAck ())
                    && m_blockAckTimeoutEvent.IsRunning ())
            {
                DuplicateTag dtag;
                packet->RemovePacketTag (dtag);
                NS_LOG_DEBUG ("1 got block ack from " << hdr.GetAddr2 () << " now " << Simulator::Now());
                CtrlBAckResponseHeader blockAck;
                packet->RemoveHeader (blockAck);
                m_blockAckTimeoutEvent.Cancel ();
                m_listener->GotBlockAck (&blockAck, hdr.GetAddr2 ());
                SnrTag tag;
                packet->RemovePacketTag (tag);
                m_stationManager->ReportDataOk (m_currentHdr.GetAddr1 (), &m_currentHdr,
                        rxSnr, txMode, tag.Get ());
            }
            else if (hdr.IsBlockAckReq () && hdr.GetAddr1 () == m_self)
            {
                CtrlBAckRequestHeader blockAckReq;
                packet->RemoveHeader (blockAckReq);
                if (!blockAckReq.IsMultiTid ())
                {
                    uint8_t tid = blockAckReq.GetTidInfo ();
                    AgreementsI it = m_bAckAgreements.find (std::make_pair (hdr.GetAddr2 (), tid));
                    if (it != m_bAckAgreements.end ())
                    {
                        //Update block ack cache
                        BlockAckCachesI i = m_bAckCaches.find (std::make_pair (hdr.GetAddr2 (), tid));
                        NS_ASSERT (i != m_bAckCaches.end ());
                        (*i).second.UpdateWithBlockAckReq (blockAckReq.GetStartingSequence ());

                        NS_ASSERT (m_sendAckEvent.IsExpired ());
                        /* See section 11.5.3 in IEEE802.11 for mean of this timer */
                        ResetBlockAckInactivityTimerIfNeeded (it->second.first);
                        if ((*it).second.first.IsImmediateBlockAck ())
                        {
                            NS_LOG_DEBUG ("rx blockAckRequest/sendImmediateBlockAck from=" << hdr.GetAddr2 ());
                            m_sendAckEvent = Simulator::Schedule (GetSifs (),
                                    &MacLow::SendBlockAckAfterBlockAckRequest, this,
                                    blockAckReq,
                                    hdr.GetAddr2 (),
                                    hdr.GetDuration (),
                                    txMode);
                        }
                        else
                        {
                            NS_FATAL_ERROR ("Delayed block ack not supported.");
                        }
                    }
                    else
                    {
                        NS_LOG_DEBUG ("There's not a valid agreement for this block ack request.");
                    }
                }
                else
                {
                    NS_FATAL_ERROR ("Multi-tid block ack is not supported.");
                }
            }
            else if (hdr.IsCtl ())
            {
                NS_LOG_DEBUG ("rx drop " << hdr.GetTypeString ());
            }

            //shbyeon last AMPDU subframe received
            else if (lastMpdu && hdr.GetAddr1 () == m_self)
            {
                NS_LOG_DEBUG ("got last AMPDU subframe from " << hdr.GetAddr2 ());
                m_stationManager->ReportRxOk (hdr.GetAddr2 (), &hdr,
                        rxSnr, txMode);
                if (hdr.IsQosData () && StoreMpduIfNeeded (packet, hdr))
                {
                    NS_LOG_DEBUG ("isAmpdu = " << isAmpdu);
                    if (hdr.IsQosAck ()) //if ACK policy is normal ACK //implicit BAR
                    {
                        NS_LOG_DEBUG ("implicit BAR");
                        uint8_t tid = hdr.GetQosTid ();
                        AgreementsI it = m_bAckAgreements.find (std::make_pair (hdr.GetAddr2 (), tid));
                        if (it != m_bAckAgreements.end ())
                        {
                            //Update block ack cache
                            BlockAckCachesI i = m_bAckCaches.find (std::make_pair (hdr.GetAddr2 (), tid));
                            NS_ASSERT (i != m_bAckCaches.end ());
                            (*i).second.UpdateWithBlockAckReq (it->second.first.GetStartingSequence ()); //modified to get Starting Sequence 
                            NS_ASSERT (m_sendAckEvent.IsExpired ());
                            ResetBlockAckInactivityTimerIfNeeded (it->second.first);
                            if ((*it).second.first.IsImmediateBlockAck ())
                            {
                                NS_LOG_DEBUG ("implicit blockAckRequest/sendImmediateBlockAck");
                                m_sendAckEvent = Simulator::Schedule (GetSifs (),
                                        &MacLow::SendBlockAckWithoutBlockAckRequest, this,
                                        hdr.GetAddr2 (),
                                        hdr.GetDuration (),
                                        txMode,
                                        tid,
                                        it->second.first.GetStartingSequence ());
                            }
                            else
                            {
                                NS_FATAL_ERROR ("Delayed block ack not supported.");
                            }
                        }
                        else
                        {
                            NS_LOG_DEBUG ("There's not a valid agreement for this block ack request.");
                        }
                    }
                    else if (hdr.IsQosBlockAck ())
                    {
                        NS_LOG_DEBUG ("last AMPDU subframe/ack policy of block ack/not send block ack");
                        AgreementsI it = m_bAckAgreements.find (std::make_pair (hdr.GetAddr2 (), hdr.GetQosTid ()));
                        ResetBlockAckInactivityTimerIfNeeded (it->second.first);
                    }
                    return;
                }
                else 
                {
                    NS_LOG_DEBUG ("Oops!!!");
                    exit(1);
                }
            }	

            else if (hdr.GetAddr1 () == m_self)
            {
                m_stationManager->ReportRxOk (hdr.GetAddr2 (), &hdr,
                        rxSnr, txMode);

                if (hdr.IsQosData () && StoreMpduIfNeeded (packet, hdr))
                {
                    /* From section 9.10.4 in IEEE802.11:
                       Upon the receipt of a QoS data frame from the originator for which
                       the Block Ack agreement exists, the recipient shall buffer the MSDU
                       regardless of the value of the Ack Policy subfield within the
                       QoS Control field of the QoS data frame. */
                    NS_LOG_DEBUG ("isAmpdu = " << isAmpdu);
                    if (!isAmpdu && hdr.IsQosAck ())
                    {
                        AgreementsI it = m_bAckAgreements.find (std::make_pair (hdr.GetAddr2 (), hdr.GetQosTid ()));
                        RxCompleteBufferedPacketsWithSmallerSequence (it->second.first.GetStartingSequenceControl (),
                                hdr.GetAddr2 (), hdr.GetQosTid ());
                        RxCompleteBufferedPacketsUntilFirstLost (hdr.GetAddr2 (), hdr.GetQosTid ());
                        NS_ASSERT (m_sendAckEvent.IsExpired ());
                        m_sendAckEvent = Simulator::Schedule (GetSifs (),
                                &MacLow::SendAckAfterData, this,
                                hdr.GetAddr2 (),
                                hdr.GetDuration (),
                                txMode,
                                rxSnr);
                    }
                    else if (isAmpdu || hdr.IsQosBlockAck ())
                    {
                        AgreementsI it = m_bAckAgreements.find (std::make_pair (hdr.GetAddr2 (), hdr.GetQosTid ()));
                        /* See section 11.5.3 in IEEE802.11 for mean of this timer */
                        ResetBlockAckInactivityTimerIfNeeded (it->second.first);
                    }
                    return;
                }
                else if (hdr.IsQosData () && hdr.IsQosBlockAck ())
                {
                    /* This happens if a packet with ack policy Block Ack is received and a block ack
                       agreement for that packet doesn't exist.

                       From section 11.5.3 in IEEE802.11e:
                       When a recipient does not have an active Block ack for a TID, but receives
                       data MPDUs with the Ack Policy subfield set to Block Ack, it shall discard
                       them and shall send a DELBA frame using the normal access
                       mechanisms. */
                    AcIndex ac = QosUtilsMapTidToAc (hdr.GetQosTid ());
                    m_edcaListeners[ac]->BlockAckInactivityTimeout (hdr.GetAddr2 (), hdr.GetQosTid ());
                    return;
                }
                else if (hdr.IsQosData () && hdr.IsQosNoAck ())
                {
                    NS_LOG_DEBUG ("rx unicast/noAck from=" << hdr.GetAddr2 ());
                }
                else if (hdr.IsData () || hdr.IsMgt ())
                {
                    NS_LOG_DEBUG ("rx unicast/sendAck from=" << hdr.GetAddr2 ());
                    NS_ASSERT (m_sendAckEvent.IsExpired ());
                    m_sendAckEvent = Simulator::Schedule (GetSifs (),
                            &MacLow::SendAckAfterData, this,
                            hdr.GetAddr2 (),
                            hdr.GetDuration (),
                            txMode,
                            rxSnr);
                }
                goto rxPacket;
            }
            else if (hdr.GetAddr1 ().IsGroup ())
            {
                if (hdr.IsData () || hdr.IsMgt ())
                {
                    NS_LOG_DEBUG ("rx group from=" << hdr.GetAddr2 ());
                    goto rxPacket;
                }
                else
                {
                    // DROP
                }
            }
            else if (m_promisc)
            {
                NS_ASSERT (hdr.GetAddr1 () != m_self);
                if (hdr.IsData ())
                {
                    goto rxPacket;
                }
            }
            else
            {
                //NS_LOG_DEBUG_VERBOSE ("rx not-for-me from %d", GetSource (packet));
            }
            return;
rxPacket:
            WifiMacTrailer fcs;
            packet->RemoveTrailer (fcs);
            m_rxCallback (packet, &hdr);
            return;
        }

    uint32_t
        MacLow::GetAckSize (void) const
        {
            WifiMacHeader ack;
            ack.SetType (WIFI_MAC_CTL_ACK);
            return ack.GetSize () + 4;
        }
    uint32_t
        MacLow::GetBlockAckSize (enum BlockAckType type) const
        {
            WifiMacHeader hdr;
            hdr.SetType (WIFI_MAC_CTL_BACKRESP);
            CtrlBAckResponseHeader blockAck;
            if (type == BASIC_BLOCK_ACK)
            {
                blockAck.SetType (BASIC_BLOCK_ACK);
            }
            else if (type == COMPRESSED_BLOCK_ACK)
            {
                blockAck.SetType (COMPRESSED_BLOCK_ACK);
            }
            else if (type == MULTI_TID_BLOCK_ACK)
            {
                //Not implemented
                NS_ASSERT (false);
            }
            return hdr.GetSize () + blockAck.GetSerializedSize () + 4;
        }
    uint32_t
        MacLow::GetRtsSize (void) const
        {
            WifiMacHeader rts;
            rts.SetType (WIFI_MAC_CTL_RTS);
            return rts.GetSize () + 4;
        }
    Time
        MacLow::GetAckDuration (Mac48Address to, WifiTxVector dataTxVector) const
        {
            WifiTxVector ackTxVector = GetAckTxVectorForData (to, dataTxVector.GetMode());
            return GetAckDuration (ackTxVector);
        }
    Time
        MacLow::GetAckDuration (WifiTxVector ackTxVector) const
        {
            WifiPreamble preamble;
            if (ackTxVector.GetMode().GetModulationClass () == WIFI_MOD_CLASS_HT)
                preamble= WIFI_PREAMBLE_HT_MF;
            else if (ackTxVector.GetMode().GetModulationClass () == WIFI_MOD_CLASS_VHT)
                preamble= WIFI_PREAMBLE_VHT;
            else
                preamble=WIFI_PREAMBLE_LONG;
            NS_LOG_DEBUG ("AckMode=" << ackTxVector.GetMode ());
            return m_phy->CalculateTxDuration (GetAckSize (), ackTxVector, preamble);
        }
    Time
        MacLow::GetBlockAckDuration (Mac48Address to, WifiTxVector blockAckReqTxVector, enum BlockAckType type) const
        {
            /*
             * For immediate BlockAck we should transmit the frame with the same WifiMode
             * as the BlockAckReq.
             *
             * from section 9.6 in IEEE802.11e:
             * The BlockAck control frame shall be sent at the same rate and modulation class as
             * the BlockAckReq frame if it is sent in response to a BlockAckReq frame.
             */
            WifiPreamble preamble;
            if (blockAckReqTxVector.GetMode().GetModulationClass () == WIFI_MOD_CLASS_VHT)
                preamble= WIFI_PREAMBLE_VHT;
            else if (blockAckReqTxVector.GetMode().GetModulationClass () == WIFI_MOD_CLASS_HT)
                preamble= WIFI_PREAMBLE_HT_MF;
            else
                preamble=WIFI_PREAMBLE_LONG;
            NS_LOG_DEBUG ("BlockAckMode=" << blockAckReqTxVector.GetMode ()<<"\tduration="<<m_phy->CalculateTxDuration (GetBlockAckSize (type), blockAckReqTxVector, preamble) << " blockackReqRate=" << blockAckReqTxVector.GetMode());
            return m_phy->CalculateTxDuration (GetBlockAckSize (type), blockAckReqTxVector, preamble);
        }
    Time
        MacLow::GetCtsDuration (Mac48Address to, WifiTxVector rtsTxVector) const
        {
            WifiTxVector ctsTxVector = GetCtsTxVectorForRts (to, rtsTxVector.GetMode());
            return GetCtsDuration (ctsTxVector);
        }

    Time
        MacLow::GetCtsDuration (WifiTxVector ctsTxVector) const
        {
            WifiPreamble preamble;
            if (ctsTxVector.GetMode().GetModulationClass () == WIFI_MOD_CLASS_VHT)
                preamble= WIFI_PREAMBLE_VHT;
            else if (ctsTxVector.GetMode().GetModulationClass () == WIFI_MOD_CLASS_HT)
                preamble= WIFI_PREAMBLE_HT_MF;
            else
                preamble=WIFI_PREAMBLE_LONG;
            return m_phy->CalculateTxDuration (GetCtsSize (), ctsTxVector, preamble);
        }
    uint32_t
        MacLow::GetCtsSize (void) const
        {
            WifiMacHeader cts;
            cts.SetType (WIFI_MAC_CTL_CTS);
            return cts.GetSize () + 4;
        }
    uint32_t
        MacLow::GetSize (Ptr<const Packet> packet, const WifiMacHeader *hdr) const
        {
            WifiMacTrailer fcs;
            return packet->GetSize () + hdr->GetSize () + fcs.GetSerializedSize ();
        }

    WifiTxVector
        MacLow::GetCtsToSelfTxVector (Ptr<const Packet> packet, const WifiMacHeader *hdr) const
        {
            return m_stationManager->GetCtsToSelfTxVector (hdr, packet);
        }

    WifiTxVector
        MacLow::GetRtsTxVector (Ptr<const Packet> packet, const WifiMacHeader *hdr) const
        {
            Mac48Address to = hdr->GetAddr1 ();
            return m_stationManager->GetRtsTxVector (to, hdr, packet);
        }
    WifiTxVector
        MacLow::GetDataTxVector (Ptr<const Packet> packet, const WifiMacHeader *hdr) const
        {
            Mac48Address to = hdr->GetAddr1 ();
            WifiMacTrailer fcs;
            uint32_t size =  packet->GetSize ()+ hdr->GetSize () + fcs.GetSerializedSize ();
            //size is not used in anything!! will not worry about aggregation
            return m_stationManager->GetDataTxVector (to, hdr, packet, size);
        }
    WifiTxVector
        MacLow::GetCtsTxVector (Mac48Address to, WifiMode rtsTxMode) const
        {
            return m_stationManager->GetCtsTxVector (to, rtsTxMode);
        }
    WifiTxVector
        MacLow::GetAckTxVector (Mac48Address to, WifiMode dataTxMode) const
        {
            return m_stationManager->GetAckTxVector (to, dataTxMode);
        }
    WifiTxVector
        MacLow::GetBlockAckTxVector (Mac48Address to, WifiMode dataTxMode) const
        {
            return m_stationManager->GetBlockAckTxVector (to, dataTxMode);
        }

    WifiTxVector
        MacLow::GetCtsTxVectorForRts (Mac48Address to, WifiMode rtsTxMode) const
        {
            return GetCtsTxVector (to, rtsTxMode);
        }
    WifiTxVector
        MacLow::GetAckTxVectorForData (Mac48Address to, WifiMode dataTxMode) const
        {
            return GetAckTxVector (to, dataTxMode);
        }


    Time
        MacLow::CalculateOverallTxTime (Ptr<const Packet> packet,
                const WifiMacHeader* hdr,
                const MacLowTransmissionParameters& params) const
        {
            WifiPreamble preamble;
            Time txTime = Seconds (0);
            if (params.MustSendRts ())
            {
                WifiTxVector rtsTxVector = GetRtsTxVector (packet, hdr);
                //standard says RTS packets can have GF format sec 9.6.0e.1 page 110 bullet b 2
                if (rtsTxVector.GetMode ().GetModulationClass () == WIFI_MOD_CLASS_VHT)
                {
                    preamble = WIFI_PREAMBLE_VHT;
                }
                else if (m_phy->GetGreenfield () && m_stationManager->GetGreenfieldSupported (m_currentHdr.GetAddr1 ()))
                {
                    preamble = WIFI_PREAMBLE_HT_GF;
                }
                else if (rtsTxVector.GetMode ().GetModulationClass () == WIFI_MOD_CLASS_HT)
                {
                    preamble = WIFI_PREAMBLE_HT_MF;
                }
                else
                {
                    preamble = WIFI_PREAMBLE_LONG;
                }
                txTime += m_phy->CalculateTxDuration (GetRtsSize (), rtsTxVector, preamble);
                txTime += GetCtsDuration (hdr->GetAddr1 (), rtsTxVector);
                txTime += Time (GetSifs () * 2);
            }
            WifiTxVector dataTxVector = GetDataTxVector (packet, hdr);
            //standard says RTS packets can have GF format sec 9.6.0e.1 page 110 bullet b 2
            if (dataTxVector.GetMode().GetModulationClass () == WIFI_MOD_CLASS_VHT)
                preamble= WIFI_PREAMBLE_VHT;
            else if ( m_phy->GetGreenfield()&& m_stationManager->GetGreenfieldSupported (m_currentHdr.GetAddr1 ()))
                preamble= WIFI_PREAMBLE_HT_GF;
            else if (dataTxVector.GetMode().GetModulationClass () == WIFI_MOD_CLASS_HT)
                preamble= WIFI_PREAMBLE_HT_MF;
            else
                preamble=WIFI_PREAMBLE_LONG;
            uint32_t dataSize = GetSize (packet, hdr);
            txTime += m_phy->CalculateTxDuration (dataSize, dataTxVector, preamble);
            if (params.MustWaitAck ())
            {
                txTime += GetSifs ();
                txTime += GetAckDuration (hdr->GetAddr1 (), dataTxVector);
            }
            return txTime;
        }

    Time
        MacLow::CalculateTransmissionTime (Ptr<const Packet> packet,
                const WifiMacHeader* hdr,
                const MacLowTransmissionParameters& params) const
        {
            Time txTime = CalculateOverallTxTime (packet, hdr, params);
            if (params.HasNextPacket ())
            {
                WifiTxVector dataTxVector = GetDataTxVector (packet, hdr);
                WifiPreamble preamble;
                //standard says RTS packets can have GF format sec 9.6.0e.1 page 110 bullet b 2
                if (dataTxVector.GetMode().GetModulationClass () == WIFI_MOD_CLASS_VHT)
                    preamble= WIFI_PREAMBLE_VHT;
                else if ( m_phy->GetGreenfield()&& m_stationManager->GetGreenfieldSupported (m_currentHdr.GetAddr1 ()))
                    preamble= WIFI_PREAMBLE_HT_GF;
                else if (dataTxVector.GetMode().GetModulationClass () == WIFI_MOD_CLASS_HT)
                    preamble= WIFI_PREAMBLE_HT_MF;
                else
                    preamble=WIFI_PREAMBLE_LONG;
                txTime += GetSifs ();
                txTime += m_phy->CalculateTxDuration (params.GetNextPacketSize (), dataTxVector, preamble);
            }
            return txTime;
        }

    //shbyeon use PHY CalculateTxDuration function in MacLow
    Time
        MacLow::CalculateTxDuration (uint32_t size, WifiTxVector txvector, WifiPreamble preamble)
        {
            NS_LOG_FUNCTION (this);
            return m_phy->CalculateTxDuration (size, txvector, preamble);
        }

    void
        MacLow::NotifyNav (Ptr<const Packet> packet,const WifiMacHeader &hdr, WifiMode txMode, WifiPreamble preamble)
        {
            NS_ASSERT (m_lastNavStart <= Simulator::Now ());
            Time duration = hdr.GetDuration ();

            if (hdr.IsCfpoll ()
                    && hdr.GetAddr2 () == m_bssid)
            {
                // see section 9.3.2.2 802.11-1999
                DoNavResetNow (duration);
                return;
            }
            /// \todo We should also handle CF_END specially here
            /// but we don't for now because we do not generate them.
            else if (hdr.GetAddr1 () != m_self)
            {
                // see section 9.2.5.4 802.11-1999
                bool navUpdated = DoNavStartNow (duration);
                if (hdr.IsRts () && navUpdated)
                {
                    /**
                     * A STA that used information from an RTS frame as the most recent basis to update its NAV setting
                     * is permitted to reset its NAV if no PHY-RXSTART.indication is detected from the PHY during a
                     * period with a duration of (2 * aSIFSTime) + (CTS_Time) + (2 * aSlotTime) starting at the
                     * PHY-RXEND.indication corresponding to the detection of the RTS frame. The “CTS_Time” shall
                     * be calculated using the length of the CTS frame and the data rate at which the RTS frame
                     * used for the most recent NAV update was received.
                     */
                    WifiMacHeader cts;
                    cts.SetType (WIFI_MAC_CTL_CTS);
                    WifiTxVector txVector=GetRtsTxVector (packet, &hdr);
                    Time navCounterResetCtsMissedDelay =
                        m_phy->CalculateTxDuration (cts.GetSerializedSize (), txVector, preamble) +
                        Time (2 * GetSifs ()) + Time (2 * GetSlotTime ());
                    m_navCounterResetCtsMissed = Simulator::Schedule (navCounterResetCtsMissedDelay,
                            &MacLow::NavCounterResetCtsMissed, this,
                            Simulator::Now ());
                }
            }
        }

    void
        MacLow::NavCounterResetCtsMissed (Time rtsEndRxTime)
        {
            if (m_phy->GetLastRxStartTime () < rtsEndRxTime)
            {
                DoNavResetNow (Seconds (0.0));
            }
        }

    void
        MacLow::DoNavResetNow (Time duration)
        {
            for (DcfListenersCI i = m_dcfListeners.begin (); i != m_dcfListeners.end (); i++)
            {
                (*i)->NavReset (duration);
            }
            m_lastNavStart = Simulator::Now ();
            m_lastNavStart = duration;
        }
    bool
        MacLow::DoNavStartNow (Time duration)
        {
            for (DcfListenersCI i = m_dcfListeners.begin (); i != m_dcfListeners.end (); i++)
            {
                (*i)->NavStart (duration);
            }
            Time newNavEnd = Simulator::Now () + duration;
            Time oldNavEnd = m_lastNavStart + m_lastNavDuration;
            if (newNavEnd > oldNavEnd)
            {
                m_lastNavStart = Simulator::Now ();
                m_lastNavDuration = duration;
                return true;
            }
            return false;
        }
    void
        MacLow::NotifyAckTimeoutStartNow (Time duration)
        {
            for (DcfListenersCI i = m_dcfListeners.begin (); i != m_dcfListeners.end (); i++)
            {
                (*i)->AckTimeoutStart (duration);
            }
        }
    void
        MacLow::NotifyAckTimeoutResetNow ()
        {
            for (DcfListenersCI i = m_dcfListeners.begin (); i != m_dcfListeners.end (); i++)
            {
                (*i)->AckTimeoutReset ();
            }
        }
    void
        MacLow::NotifyCtsTimeoutStartNow (Time duration)
        {
            for (DcfListenersCI i = m_dcfListeners.begin (); i != m_dcfListeners.end (); i++)
            {
                (*i)->CtsTimeoutStart (duration);
            }
        }
    void
        MacLow::NotifyCtsTimeoutResetNow ()
        {
            for (DcfListenersCI i = m_dcfListeners.begin (); i != m_dcfListeners.end (); i++)
            {
                (*i)->CtsTimeoutReset ();
            }
        }

    void
        MacLow::ForwardDown (Ptr<const Packet> packet, const WifiMacHeader* hdr,
                WifiTxVector txVector, WifiPreamble preamble)
        {
            //802.11ac channel bonding
            uint16_t currentWidth = m_stationManager->GetCurrentBandwidth(hdr->GetAddr1(), hdr);
            uint16_t dataBandwidth =  currentWidth;
            TxDuplicate duplicateBandwidth = NO_DUPLICATE;

            if(hdr->GetType() == WIFI_MAC_CTL_RTS)
            {
                //shbyeon: rts has a data packet to transmit, therefore
                //it can refer data packet's bandwidth (for duplicate)
                DuplicateTag dtag;
                if(packet->PeekPacketTag(dtag))
                {
                    dataBandwidth = dtag.GetDuplicateBandwidth();
                }
                else
                {
                    NS_LOG_DEBUG("rts should have duplicate tag");
                    NS_ASSERT(false);
                }
                NS_LOG_DEBUG(m_self << " Send rts" << " operatingBW="<<m_operationalBandwidth<<
                        " currentBW="<< currentWidth );
                switch (dataBandwidth)
                {
                    case 20:
                        duplicateBandwidth = NO_DUPLICATE;
                        break;
                    case 40:
                        duplicateBandwidth = DUPLICATE_40;
                        break;
                    case 80:
                        duplicateBandwidth = DUPLICATE_80;
                        break;
                    default:
                        duplicateBandwidth = NO_DUPLICATE;
                        break;
                }
            }
            else if (hdr->GetType() == WIFI_MAC_CTL_ACK || hdr->GetType() == WIFI_MAC_CTL_BACKRESP)
            {
                //shbyeon: ACK and BlockAck can refer received data's bandwidth
                DuplicateTag dtag;
                if(packet->PeekPacketTag(dtag))
                {
                    dataBandwidth = dtag.GetDuplicateBandwidth();
                }
                else
                {
                    NS_LOG_DEBUG("(b)ack should have duplicate tag");
                    NS_ASSERT(false);
                }
                switch (dataBandwidth)
                {
                    case 20:
                        duplicateBandwidth = NO_DUPLICATE;
                        break;
                    case 40:
                        duplicateBandwidth = DUPLICATE_40;
                        break;
                    case 80:
                        duplicateBandwidth = DUPLICATE_80;
                        break;
                    default:
                        duplicateBandwidth = NO_DUPLICATE;
                        break;
                }
                NS_LOG_DEBUG("Send ack/blockack" << " operatingBW="<<m_operationalBandwidth<<
                        " dataBW="<<dataBandwidth<<" currentBW="<<currentWidth<<" duplicateBW="<<duplicateBandwidth);
            }
            else if (hdr->GetType() == WIFI_MAC_CTL_CTS)
            {
                //shbyeon: CTS need to check the bandwidth of corresponding RTS
                DuplicateTag dtag;
                if(packet->PeekPacketTag(dtag))
                {
                    dataBandwidth = dtag.GetDuplicateBandwidth();
                }
                else
                {
                    NS_LOG_DEBUG("cts should have duplicate tag");
                    NS_ASSERT(false);
                }
                switch (dataBandwidth)
                {
                    case 20:
                        duplicateBandwidth = NO_DUPLICATE;
                        break;
                    case 40:
                        duplicateBandwidth = DUPLICATE_40;
                        break;
                    case 80:
                        duplicateBandwidth = DUPLICATE_80;
                        break;
                    default:
                        duplicateBandwidth = NO_DUPLICATE;
                        break;
                }
                NS_LOG_DEBUG("Send cts" << " operatingBW="<<m_operationalBandwidth<<
                        " dataBW="<<dataBandwidth<<" currentBW="<<currentWidth<<" duplicateBW="<<duplicateBandwidth);
            }
            else if (hdr->IsMgt() == true)
            {
                currentWidth = 20;
            }
            NS_LOG_FUNCTION (this << packet << hdr << txVector);
            NS_LOG_DEBUG ("send " << hdr->GetTypeString () <<
                    ", to=" << hdr->GetAddr1 () <<
                    ", size=" << packet->GetSize () <<
                    ", mode=" << txVector.GetMode() <<
                    ", nss=" << (int)txVector.GetNss() <<
                    ", duration=" << hdr->GetDuration () <<
                    ", seq=" << hdr->GetSequenceNumber () << 
                    ", bandwidth=" << currentWidth <<
                    ", seqCon=" << hdr->GetSequenceControl ());
            ErrorFreeTag tag;
            //shbyeon error free tag for mgmt/ctrl frames
            /*
               if (hdr->IsMgt () || hdr->IsBlockAck() || hdr->IsAck() || (hdr->IsQosData() && Simulator::Now().GetSeconds() < 1))
               tag.Set (true);
               else
               tag.Set (false);
               */
            tag.Set (false);
            packet->AddPacketTag (tag);
            m_phy->SendPacket (packet, txVector, preamble, currentWidth);
        }

    void
        MacLow::CtsTimeout (void)
        {
            NS_LOG_FUNCTION (this);
            NS_LOG_DEBUG ("cts timeout");
            /// \todo should check that there was no rx start before now.
            /// we should restart a new cts timeout now until the expected
            /// end of rx if there was a rx start before now.
            m_stationManager->ReportRtsFailed (m_currentHdr.GetAddr1 (), &m_currentHdr);

            //shbyeon recovery for rts/cts failer for ampdu
            AmpduTag tag;
            bool isAmpdu = m_currentPacket->PeekPacketTag(tag);
            if(isAmpdu)
            {
                NS_LOG_DEBUG ("AMPDU packet should be saved");
                Ptr<EdcaTxopN> m_edca = m_edcas[QosUtilsMapTidToAc (m_currentHdr.GetQosTid ())];
                m_edca->GetBlockAckManager ()->NotifyRtsFailed(m_currentHdr.GetQosTid (), m_currentHdr.GetAddr1 ());
            }

            m_currentPacket = 0;
            MacLowTransmissionListener *listener = m_listener;
            m_listener = 0;
            listener->MissedCts ();
        }
    void
        MacLow::NormalAckTimeout (void)
        {
            NS_LOG_FUNCTION (this);
            NS_LOG_DEBUG ("normal ack timeout");
            /// \todo should check that there was no rx start before now.
            /// we should restart a new ack timeout now until the expected
            /// end of rx if there was a rx start before now.
            m_stationManager->ReportDataFailed (m_currentHdr.GetAddr1 (), &m_currentHdr);
            MacLowTransmissionListener *listener = m_listener;
            m_listener = 0;
            listener->MissedAck ();
        }
    void
        MacLow::FastAckTimeout (void)
        {
            NS_LOG_FUNCTION (this);
            m_stationManager->ReportDataFailed (m_currentHdr.GetAddr1 (), &m_currentHdr);
            MacLowTransmissionListener *listener = m_listener;
            m_listener = 0;
            if (m_phy->IsStateIdle ())
            {
                NS_LOG_DEBUG ("fast Ack idle missed");
                listener->MissedAck ();
            }
            else
            {
                NS_LOG_DEBUG ("fast Ack ok");
            }
        }
    void
        MacLow::BlockAckTimeout (void)
        {
            NS_LOG_FUNCTION (this);
            NS_LOG_DEBUG ("block ack timeout");

            m_stationManager->ReportDataFailed (m_currentHdr.GetAddr1 (), &m_currentHdr);
            MacLowTransmissionListener *listener = m_listener;
            m_listener = 0;
            listener->MissedBlockAck ();
        }
    void
        MacLow::SuperFastAckTimeout ()
        {
            NS_LOG_FUNCTION (this);
            m_stationManager->ReportDataFailed (m_currentHdr.GetAddr1 (), &m_currentHdr);
            MacLowTransmissionListener *listener = m_listener;
            m_listener = 0;
            if (m_phy->IsStateIdle ())
            {
                NS_LOG_DEBUG ("super fast Ack failed");
                listener->MissedAck ();
            }
            else
            {
                NS_LOG_DEBUG ("super fast Ack ok");
                listener->GotAck (0.0, WifiMode ());
            }
        }

    void
        MacLow::SendRtsForPacket (void)
        {
            NS_LOG_FUNCTION (this);
            /* send an RTS for this packet. */
            WifiMacHeader rts;
            rts.SetType (WIFI_MAC_CTL_RTS);
            rts.SetDsNotFrom ();
            rts.SetDsNotTo ();
            rts.SetNoRetry ();
            rts.SetNoMoreFragments ();
            rts.SetAddr1 (m_currentHdr.GetAddr1 ());
            rts.SetAddr2 (m_self);
            WifiTxVector rtsTxVector = GetRtsTxVector (m_currentPacket, &m_currentHdr);
            Time duration = Seconds (0);



            WifiPreamble preamble;
            //standard says RTS packets can have GF format sec 9.6.0e.1 page 110 bullet b 2
            if (rtsTxVector.GetMode().GetModulationClass () == WIFI_MOD_CLASS_VHT)
                preamble= WIFI_PREAMBLE_VHT;
            else if ( m_phy->GetGreenfield()&& m_stationManager->GetGreenfieldSupported (m_currentHdr.GetAddr1 ()))
                preamble= WIFI_PREAMBLE_HT_GF;
            else if (rtsTxVector.GetMode().GetModulationClass () == WIFI_MOD_CLASS_HT)
                preamble= WIFI_PREAMBLE_HT_MF;
            else
                preamble=WIFI_PREAMBLE_LONG;

            if (m_txParams.HasDurationId ())
            {
                duration += m_txParams.GetDurationId ();
            }
            else
            {
                WifiTxVector dataTxVector = GetDataTxVector (m_currentPacket, &m_currentHdr);
                //shbyeon RTSCTS buf fix
                WifiPreamble dataPreamble;       
                if (dataTxVector.GetMode().GetModulationClass () == WIFI_MOD_CLASS_VHT)
                    dataPreamble= WIFI_PREAMBLE_VHT;
                else if (m_phy->GetGreenfield() && m_stationManager->GetGreenfieldSupported (m_currentHdr.GetAddr1 ()))
                    //In the future has to make sure that receiver has greenfield enabled
                    dataPreamble= WIFI_PREAMBLE_HT_GF;
                else if (dataTxVector.GetMode().GetModulationClass () == WIFI_MOD_CLASS_HT)
                    dataPreamble= WIFI_PREAMBLE_HT_MF;
                else
                    dataPreamble=WIFI_PREAMBLE_LONG;
                m_stationManager->SetPrevTxVector(m_currentHdr.GetAddr1(), &m_currentHdr, dataTxVector);
                duration += GetSifs ();
                duration += GetCtsDuration (m_currentHdr.GetAddr1 (), rtsTxVector);
                duration += GetSifs ();
                duration += m_phy->CalculateTxDuration (GetSize (m_currentPacket, &m_currentHdr),
                        dataTxVector, dataPreamble);
                NS_LOG_DEBUG("packetSize=" << GetSize(m_currentPacket, &m_currentHdr) << " dataRate=" << dataTxVector.GetMode() << " preamble=" << (int)dataPreamble);
                duration += GetSifs ();
                if(m_currentHdr.IsBlockAckReq())
                    duration += GetBlockAckDuration (m_currentHdr.GetAddr1 (), rtsTxVector, COMPRESSED_BLOCK_ACK);
                else
                    duration += GetAckDuration (m_currentHdr.GetAddr1 (), dataTxVector);
                NS_LOG_DEBUG("duration update, SIFS+CTS+SIFS+DATA+SIFS+(B)ACK=" << duration.GetMicroSeconds());
            }
            rts.SetDuration (duration);

            Time txDuration = m_phy->CalculateTxDuration (GetRtsSize (), rtsTxVector, preamble);
            Time timerDelay = txDuration + GetCtsTimeout ();

            NS_ASSERT (m_ctsTimeoutEvent.IsExpired ());
            NotifyCtsTimeoutStartNow (timerDelay);
            m_ctsTimeoutEvent = Simulator::Schedule (timerDelay, &MacLow::CtsTimeout, this);

            Ptr<Packet> packet = Create<Packet> ();
            packet->AddHeader (rts);
            WifiMacTrailer fcs;
            packet->AddTrailer (fcs);

            NS_LOG_DEBUG("rts transmission before sending a MPDU");
            DuplicateTag dtag(m_stationManager->GetCurrentBandwidth(m_currentHdr.GetAddr1(), &m_currentHdr), 0, RTS);
            packet->AddPacketTag (dtag);

            ForwardDown (packet, &rts, rtsTxVector,preamble);
        }

    //shbyeon rts setting for ampdu
    void
        MacLow::SendRtsForAmpdu (enum BlockAckType type)
        {
            NS_LOG_FUNCTION (this);
            AmpduTag tag;
            NS_LOG_DEBUG("current Ampdu?: "<<m_currentPacket->PeekPacketTag(tag) << "("<<m_currentPacket <<")" );
            NS_ASSERT(m_currentPacket->PeekPacketTag(tag));

            /* send an RTS for this packet. */
            WifiMacHeader rts;
            rts.SetType (WIFI_MAC_CTL_RTS);
            rts.SetDsNotFrom ();
            rts.SetDsNotTo ();
            rts.SetNoRetry ();
            rts.SetNoMoreFragments ();
            rts.SetAddr1 (m_currentHdr.GetAddr1 ());
            rts.SetAddr2 (m_self);
            WifiTxVector rtsTxVector = GetRtsTxVector (m_currentPacket, &m_currentHdr);
            Time duration = Seconds (0);
            WifiPreamble preamble;
            //standard says RTS packets can have GF format sec 9.6.0e.1 page 110 bullet b 2
            if (rtsTxVector.GetMode().GetModulationClass () == WIFI_MOD_CLASS_VHT)
                preamble= WIFI_PREAMBLE_VHT;
            else if ( m_phy->GetGreenfield()&& m_stationManager->GetGreenfieldSupported (m_currentHdr.GetAddr1 ()))
                preamble= WIFI_PREAMBLE_HT_GF;
            else if (rtsTxVector.GetMode().GetModulationClass () == WIFI_MOD_CLASS_HT)
                preamble= WIFI_PREAMBLE_HT_MF;
            else
                preamble=WIFI_PREAMBLE_LONG;

            if (m_txParams.HasDurationId ())
            {
                duration += m_txParams.GetDurationId ();
            }
            else
            {
                WifiTxVector dataTxVector = GetDataTxVector (m_currentPacket, &m_currentHdr);
                //shbyeon RTSCTS buf fix
                m_stationManager->SetPrevTxVector(m_currentHdr.GetAddr1(), &m_currentHdr, dataTxVector);
                duration += GetSifs ();
                duration += GetCtsDuration (m_currentHdr.GetAddr1 (), rtsTxVector);
                duration += GetSifs ();
                duration += m_phy->CalculateTxDuration (m_currentPacket->GetSize (),
                        dataTxVector, preamble);
                duration += GetSifs ();
                duration += GetBlockAckDuration (m_currentHdr.GetAddr1 (), rtsTxVector, type);
            }
            rts.SetDuration (duration);

            Time txDuration = m_phy->CalculateTxDuration (GetRtsSize (), rtsTxVector, preamble);
            Time timerDelay = txDuration + GetCtsTimeout ();

            NS_ASSERT (m_ctsTimeoutEvent.IsExpired ());
            NotifyCtsTimeoutStartNow (timerDelay);
            m_ctsTimeoutEvent = Simulator::Schedule (timerDelay, &MacLow::CtsTimeout, this);

            Ptr<Packet> packet = Create<Packet> ();
            packet->AddHeader (rts);
            WifiMacTrailer fcs;
            packet->AddTrailer (fcs);

            NS_LOG_DEBUG("rts transmission before sending AMPDU");
            DuplicateTag dtag(m_stationManager->GetCurrentBandwidth(m_currentHdr.GetAddr1(), &m_currentHdr), 0, RTS);
            packet->AddPacketTag (dtag);

            ForwardDown (packet, &rts, rtsTxVector, preamble);
        }

    //shbyeon ampdu tx timer
    void
        MacLow::StartAmpduTxTimers (WifiTxVector txVector)
        {
            NS_LOG_FUNCTION(this);
            WifiMode txMode = txVector.GetMode();

            WifiPreamble preamble;
            if (txVector.GetMode().GetModulationClass () == WIFI_MOD_CLASS_VHT)
                preamble= WIFI_PREAMBLE_VHT;
            else if (txVector.GetMode().GetModulationClass () == WIFI_MOD_CLASS_HT)
                preamble= WIFI_PREAMBLE_HT_MF;
            else
                preamble=WIFI_PREAMBLE_LONG;

            Time txDuration = m_phy->CalculateTxDuration (m_currentPacket->GetSize (), txVector, preamble);
            NS_LOG_DEBUG ("Size: " << m_currentPacket->GetSize () <<
                    ", txMode: " << txMode <<
                    ", txDuration: " << txDuration <<
                    ", duration: " <<  m_currentHdr.GetDuration ()); 

            if (m_txParams.MustWaitBasicBlockAck ())
            {
                SetBasicBlockAckTimeout (m_currentHdr.GetDuration ()+ MicroSeconds (GetDefaultMaxPropagationDelay ().GetMicroSeconds () * 2));
                NS_LOG_DEBUG ("BasicBlocAckTimeout: " << GetBasicBlockAckTimeout ());
                Time timerDelay = txDuration + GetBasicBlockAckTimeout ();
                NS_ASSERT (m_blockAckTimeoutEvent.IsExpired ());
                NotifyAckTimeoutStartNow (timerDelay);
                m_blockAckTimeoutEvent = Simulator::Schedule (timerDelay, &MacLow::BlockAckTimeout, this);
            }
            else if (m_txParams.MustWaitCompressedBlockAck ())
            {
                SetCompressedBlockAckTimeout (m_currentHdr.GetDuration () + MicroSeconds (GetDefaultMaxPropagationDelay ().GetMicroSeconds () * 2));
                Time timerDelay = txDuration + GetCompressedBlockAckTimeout ();
                NS_LOG_DEBUG ("If send BlockAckReq (ampdu), HdrDuration: " << m_currentHdr.GetDuration() << " propagationDelay: " << GetDefaultMaxPropagationDelay().GetMicroSeconds()*2 
                        << "  CompressedBlocAckTimeout: " << GetCompressedBlockAckTimeout () << ", txDuration: " << txDuration << ", timeout invoked at " << Simulator::Now () + timerDelay);
                NS_ASSERT (m_blockAckTimeoutEvent.IsExpired ());
                NotifyAckTimeoutStartNow (timerDelay);
                m_blockAckTimeoutEvent = Simulator::Schedule (timerDelay, &MacLow::BlockAckTimeout, this);
            }
        }

    void
        MacLow::StartDataTxTimers (WifiTxVector dataTxVector)
        {
            WifiPreamble preamble;

            //Since it is data then it can have format = GF
            if (dataTxVector.GetMode().GetModulationClass () == WIFI_MOD_CLASS_VHT)
                preamble= WIFI_PREAMBLE_VHT;
            else if (m_phy->GetGreenfield() && m_stationManager->GetGreenfieldSupported (m_currentHdr.GetAddr1 ()))
                preamble= WIFI_PREAMBLE_HT_GF;
            else if (dataTxVector.GetMode().GetModulationClass () == WIFI_MOD_CLASS_HT)
                preamble= WIFI_PREAMBLE_HT_MF;
            else
                preamble=WIFI_PREAMBLE_LONG;

            Time txDuration = m_phy->CalculateTxDuration (GetSize (m_currentPacket, &m_currentHdr), dataTxVector, preamble);

            NS_LOG_DEBUG("tx timer setup= " << dataTxVector.GetMode() << " packetsize= " << GetSize(m_currentPacket, &m_currentHdr) 
                    << " " << m_currentPacket->GetSize() << " duration=" << txDuration.GetMicroSeconds()); 

            if (m_txParams.MustWaitNormalAck ())
            {
                Time timerDelay = txDuration + GetAckTimeout ();
                NS_ASSERT (m_normalAckTimeoutEvent.IsExpired ());
                NotifyAckTimeoutStartNow (timerDelay);
                m_normalAckTimeoutEvent = Simulator::Schedule (timerDelay, &MacLow::NormalAckTimeout, this);
            }
            else if (m_txParams.MustWaitFastAck ())
            {
                Time timerDelay = txDuration + GetPifs ();
                NS_ASSERT (m_fastAckTimeoutEvent.IsExpired ());
                NotifyAckTimeoutStartNow (timerDelay);
                m_fastAckTimeoutEvent = Simulator::Schedule (timerDelay, &MacLow::FastAckTimeout, this);
            }
            else if (m_txParams.MustWaitSuperFastAck ())
            {
                Time timerDelay = txDuration + GetPifs ();
                NS_ASSERT (m_superFastAckTimeoutEvent.IsExpired ());
                NotifyAckTimeoutStartNow (timerDelay);
                m_superFastAckTimeoutEvent = Simulator::Schedule (timerDelay,
                        &MacLow::SuperFastAckTimeout, this);
            }
            else if (m_txParams.MustWaitBasicBlockAck ())
            {
                SetCompressedBlockAckTimeout (m_currentHdr.GetDuration () + MicroSeconds (GetDefaultMaxPropagationDelay ().GetMicroSeconds () * 2));
                Time timerDelay = txDuration + GetBasicBlockAckTimeout ();
                NS_ASSERT (m_blockAckTimeoutEvent.IsExpired ());
                m_blockAckTimeoutEvent = Simulator::Schedule (timerDelay, &MacLow::BlockAckTimeout, this);
            }
            else if (m_txParams.MustWaitCompressedBlockAck ())
            {
                //shbyeon
                SetCompressedBlockAckTimeout (m_currentHdr.GetDuration () + MicroSeconds (GetDefaultMaxPropagationDelay ().GetMicroSeconds () * 2));
                Time timerDelay = txDuration + GetCompressedBlockAckTimeout ();
                NS_ASSERT (m_blockAckTimeoutEvent.IsExpired ());
                NS_LOG_DEBUG ("If send BlockAckReq, HdrDuration: " << m_currentHdr.GetDuration ()<< " propagationDelay: " << GetDefaultMaxPropagationDelay().GetMicroSeconds()*2 
                        << "  CompressedBlocAckTimeout: " << GetCompressedBlockAckTimeout () << ", txDuration: " << txDuration << ", timeout invoked at " << Simulator::Now () + timerDelay);
                m_blockAckTimeoutEvent = Simulator::Schedule (timerDelay, &MacLow::BlockAckTimeout, this);
            }
            else if (m_txParams.HasNextPacket ())
            {
                if (m_stationManager->HasHtSupported())
                {
                    Time delay = txDuration + GetRifs ();
                    NS_ASSERT (m_waitRifsEvent.IsExpired ());
                    m_waitRifsEvent = Simulator::Schedule (delay, &MacLow::WaitSifsAfterEndTx, this); 
                }
                else
                {
                    Time delay = txDuration + GetSifs ();
                    NS_ASSERT (m_waitSifsEvent.IsExpired ());
                    m_waitSifsEvent = Simulator::Schedule (delay, &MacLow::WaitSifsAfterEndTx, this);
                }
            }
            else
            {
                // since we do not expect any timer to be triggered.
                Simulator::Schedule(txDuration, &MacLow::EndTxNoAck, this);
            }
        }

    void
        MacLow::SendDataPacket (void)
        {
            NS_LOG_FUNCTION (this);
            WifiTxVector dataTxVector;
            /* send this packet directly. No RTS is needed. */
            if(m_currentHdr.GetType() == WIFI_MAC_CTL_BACKREQ)
                dataTxVector = GetRtsTxVector (m_currentPacket, &m_currentHdr);
            else if (m_currentHdr.GetType() == WIFI_MAC_MGT_ASSOCIATION_REQUEST || m_currentHdr.GetType() == WIFI_MAC_MGT_ASSOCIATION_RESPONSE)
                dataTxVector = GetAckTxVector(m_currentHdr.GetAddr1(), m_phy->GetMode(0));
            else
                dataTxVector = GetDataTxVector (m_currentPacket, &m_currentHdr);

            WifiPreamble preamble;
            if (dataTxVector.GetMode().GetModulationClass () == WIFI_MOD_CLASS_VHT)
                preamble= WIFI_PREAMBLE_VHT;
            else if (m_phy->GetGreenfield() && m_stationManager->GetGreenfieldSupported (m_currentHdr.GetAddr1 ()))
                //In the future has to make sure that receiver has greenfield enabled
                preamble= WIFI_PREAMBLE_HT_GF;
            else if (dataTxVector.GetMode().GetModulationClass () == WIFI_MOD_CLASS_HT)
                preamble= WIFI_PREAMBLE_HT_MF;
            else
                preamble=WIFI_PREAMBLE_LONG;

            NS_LOG_DEBUG ("to=" << m_currentHdr.GetAddr1 () <<
                    ", DurationId=" << m_txParams.HasDurationId () << ", BasicBA=" <<
                    m_txParams.MustWaitBasicBlockAck () << ", CompBA=" << 
                    m_txParams.MustWaitCompressedBlockAck () << ", Ack=" <<
                    m_txParams.MustWaitAck () << ", NextPacket=" << 
                    m_txParams.HasNextPacket ());

            Time duration = Seconds (0.0);
            if (m_txParams.HasDurationId ())
            {
                duration += m_txParams.GetDurationId ();
            }
            else
            {
                if (m_txParams.MustWaitBasicBlockAck ())
                {
                    duration += GetSifs ();
                    duration += GetBlockAckDuration (m_currentHdr.GetAddr1 (), dataTxVector, BASIC_BLOCK_ACK);
                }
                else if (m_txParams.MustWaitCompressedBlockAck ())
                {
                    duration += GetSifs ();
                    duration += GetBlockAckDuration (m_currentHdr.GetAddr1 (), dataTxVector, COMPRESSED_BLOCK_ACK);
                }
                else if (m_txParams.MustWaitAck ())
                {
                    duration += GetSifs ();
                    duration += GetAckDuration (m_currentHdr.GetAddr1 (), dataTxVector);
                }
                if (m_txParams.HasNextPacket ())
                {
                    duration += GetSifs ();
                    duration += m_phy->CalculateTxDuration (m_txParams.GetNextPacketSize (),
                            dataTxVector, preamble);
                    if (m_txParams.MustWaitAck ())
                    {
                        duration += GetSifs ();
                        duration += GetAckDuration (m_currentHdr.GetAddr1 (), dataTxVector);
                    }
                }
            }
            m_currentHdr.SetDuration (duration);

            StartDataTxTimers (dataTxVector);
            m_currentPacket->AddHeader (m_currentHdr);
            WifiMacTrailer fcs;
            m_currentPacket->AddTrailer (fcs);

            ForwardDown (m_currentPacket, &m_currentHdr, dataTxVector,preamble);
            m_currentPacket = 0;
        }

    //shbyeon send ampdu
    void
        MacLow::SendAmpduPacket (void)
        {
            NS_LOG_FUNCTION (this);
            WifiTxVector dataTxVector = GetDataTxVector (m_currentPacket, &m_currentHdr);
            WifiMode dataTxMode = dataTxVector.GetMode();
            StartAmpduTxTimers (dataTxVector);
            NS_LOG_DEBUG ("txRate=" << dataTxMode.GetDataRate ());
            WifiPreamble preamble;       
            if (dataTxVector.GetMode().GetModulationClass () == WIFI_MOD_CLASS_VHT)
                preamble= WIFI_PREAMBLE_VHT;
            else if (dataTxVector.GetMode().GetModulationClass () == WIFI_MOD_CLASS_HT)
                preamble= WIFI_PREAMBLE_HT_MF;
            else
                preamble=WIFI_PREAMBLE_LONG;

            ForwardDown (m_currentPacket, &m_currentHdr, dataTxVector, preamble);
            m_currentPacket = 0;
        }

    //shbyeon send ampdu
    bool
        MacLow::AggregateMpdu(Ptr<MpduAggregator> aggregator)
        {
            NS_LOG_FUNCTION (this << aggregator);
            bool isAmpdu = false;

            Mac48Address recipient = m_currentHdr.GetAddr1 ();
            Mac48Address originator = m_currentHdr.GetAddr2 ();
            uint8_t tid = m_currentHdr.GetQosTid ();

            Ptr<EdcaTxopN> m_edca = m_edcas[QosUtilsMapTidToAc (m_currentHdr.GetQosTid ())];
            NS_LOG_DEBUG ("tid: " << (int)m_currentHdr.GetQosTid () <<
                    ", ac: "<< QosUtilsMapTidToAc (m_currentHdr.GetQosTid ()) <<
                    ", m_ac: "<< m_edca->GetAccessCategory ());
            Ptr<WifiMacQueue> m_queue = m_edca->GetQueue();

            NS_LOG_DEBUG ("# of packets in the queue: " << m_queue->GetNPacketsByTidAndAddress (tid, WifiMacHeader::ADDR1, recipient)
                    << " # of packets in the buffered queue: " << m_edca->GetBlockAckManager ()->GetNBufferedPackets (recipient, tid) 
                    << " # of packets in the retry queue: " << m_edca->GetBlockAckManager ()->GetNRetryNeededPackets(recipient, tid)); 
            if (m_queue->GetNPacketsByTidAndAddress (tid, WifiMacHeader::ADDR1, recipient) +
                    m_edca->GetBlockAckManager ()->GetNRetryNeededPackets (recipient, tid) == 0){
                NS_LOG_DEBUG("there is no packets to aggregate");
                return isAmpdu;
            }

            NS_LOG_DEBUG("there are packets to aggregate");

            WifiTxVector dataTxVector = GetDataTxVector (m_currentPacket, &m_currentHdr);
            WifiMode dataTxMode = dataTxVector.GetMode();

            Time duration = Seconds (0.0);
            duration += GetSifs ();
            //WifiTxVector blockAckTxVector = GetRtsTxVector (m_currentPacket, &m_currentHdr);
            WifiTxVector blockAckTxVector = GetBlockAckTxVector (originator, dataTxMode);
            duration += GetBlockAckDuration (m_currentHdr.GetAddr1 (), blockAckTxVector, m_edca->m_blockAckType);
            NS_LOG_DEBUG("duration: " << duration);
            m_currentHdr.SetDuration (duration);

            NS_LOG_DEBUG("Size without hdr: " << m_currentPacket->GetSize () );
            m_currentPacket->AddHeader (m_currentHdr);
            WifiMacTrailer fcs;
            m_currentPacket->AddTrailer (fcs);
            m_currentPacket->AddPacketTag (AmpduTag (false));

            NS_LOG_DEBUG("First Size with hdr/trailer: " << m_currentPacket->GetSize () );

            Ptr<Packet> currentAggregatedPacket = Create<Packet> ();
            aggregator->Aggregate (m_currentPacket, currentAggregatedPacket);


            NS_LOG_DEBUG("1st packet is aggregated size: " << currentAggregatedPacket->GetSize () );
            WifiPreamble preamble;       
            if (dataTxVector.GetMode().GetModulationClass () == WIFI_MOD_CLASS_VHT)
                preamble= WIFI_PREAMBLE_VHT;
            else if (dataTxVector.GetMode().GetModulationClass () == WIFI_MOD_CLASS_HT)
                preamble= WIFI_PREAMBLE_HT_MF;
            else
                preamble=WIFI_PREAMBLE_LONG;

            bool aggregated = false;

            WifiMacHeader nextHdr; 
            int32_t maxAvailableLength = aggregator->GetMaxAvailableLength (currentAggregatedPacket);

            //caudal loss
            Time maxAvailableDuration = Seconds(0);
            Time maxAvailableDuration_ref = Seconds(0);
            //shbyeon txop implementation
            if(m_stationManager->m_txop > 0)
            {
                maxAvailableDuration_ref = std::min(m_stationManager->GetTxop(m_currentHdr.GetAddr1(), &m_currentHdr), m_stationManager->GetAggrTime(m_currentHdr.GetAddr1(), &m_currentHdr));
                NS_LOG_DEBUG("current TXOP=" << m_stationManager->GetTxop(m_currentHdr.GetAddr1(), &m_currentHdr)
                        << " aggrTime=" << m_stationManager->GetAggrTime(m_currentHdr.GetAddr1(), &m_currentHdr)
                        << " maxDuration=" << maxAvailableDuration_ref);
            }
            else
                maxAvailableDuration_ref = m_stationManager->GetAggrTime(m_currentHdr.GetAddr1(), &m_currentHdr);
            maxAvailableDuration = maxAvailableDuration_ref
                - CalculateTxDuration (currentAggregatedPacket->GetSize (), dataTxVector, WIFI_PREAMBLE_LONG);
            NS_LOG_DEBUG ("maximum length for ampdu= " << maxAvailableLength 
                    << ", maximum duration for ampdu=" << maxAvailableDuration.GetMicroSeconds () 
                    << " us, m_maxPpduTime=" << m_maxPpduTime.GetMicroSeconds ()
                    << " us");	
            Ptr<Packet> nextPacket = m_edca->GetNextPacketForAmpdu(nextHdr, maxAvailableLength, 
                    maxAvailableDuration);

            int k = 1;
            while (nextPacket != 0 )
            {
                NS_LOG_DEBUG("nextPacket: "<< nextPacket <<
                        ", packet type: " << nextHdr.GetTypeString () <<
                        ", to=" << nextHdr.GetAddr1 () <<
                        ", seq=" << nextHdr.GetSequenceNumber ());

                if(nextHdr.IsBlockAckReq ())
                    break;

                nextHdr.SetDuration (duration);
                nextPacket->AddHeader (nextHdr);
                WifiMacTrailer fcs;
                nextPacket->AddTrailer (fcs);
                //JWHUR mesh ampdu
                AmpduTag tag;
                if (!nextPacket->PeekPacketTag(tag))
                    nextPacket->AddPacketTag (AmpduTag (false));

                NS_LOG_DEBUG("Nextpacket Size with hdr/trailer: " << nextPacket->GetSize () );
                aggregated = aggregator->Aggregate (nextPacket, currentAggregatedPacket);

                if (aggregated)
                {
                    k++;
                    NS_LOG_DEBUG(k << "th packet is aggregated size: " << currentAggregatedPacket->GetSize () );
                    isAmpdu = true;
                }
                else 
                {
                    NS_ASSERT("No aggregation!");
                    AmpduTag tag;
                    nextPacket-> RemovePacketTag(tag);
                    break;
                }
                maxAvailableLength = aggregator->GetMaxAvailableLength (currentAggregatedPacket);
                //shbyeon txop implementation
                maxAvailableDuration = maxAvailableDuration_ref - CalculateTxDuration (currentAggregatedPacket->GetSize (), dataTxVector, preamble);

                NS_LOG_DEBUG ("remaining length for ampdu= " << maxAvailableLength 
                        << ", remaining duration for ampdu=" << maxAvailableDuration.GetMicroSeconds () << " us");	
                nextPacket = m_edca->GetNextPacketForAmpdu(nextHdr, maxAvailableLength, 
                        maxAvailableDuration);

                NS_LOG_DEBUG("Aggregation size: " << k << ", aggregated packet size: " << currentAggregatedPacket->GetSize () );
            }

            if (nextPacket != 0 && nextHdr.IsBlockAckReq() && m_edca->GetImplicitBlockAckRequest ()){
                NS_LOG_DEBUG("nextPacket: "<< nextPacket <<
                        ", packet type: " << nextHdr.GetTypeString () <<
                        ", to=" << nextHdr.GetAddr1 () <<
                        ", seq=" << nextHdr.GetSequenceControl ());

                nextHdr.SetDuration (duration);
                nextPacket->AddHeader (nextHdr);
                WifiMacTrailer fcs;
                nextPacket->AddTrailer (fcs);
                NS_LOG_DEBUG("Nextpacket Size with hdr/trailer: " << nextPacket->GetSize () );

                aggregated = aggregator->Aggregate (nextPacket, currentAggregatedPacket);
                if (aggregated)
                {
                    k++;
                    NS_LOG_DEBUG(k << "th (last) packet is aggregated size: " << currentAggregatedPacket->GetSize () );
                    isAmpdu = true;
                }
            }

            //shbyeon txop implementation
            if(m_stationManager->m_txop>0)
            {
                Time txopConsumption = CalculateTxDuration(currentAggregatedPacket->GetSize(), dataTxVector, preamble);
                WifiTxVector rtsTxVector = GetRtsTxVector (m_currentPacket, &m_currentHdr);
                txopConsumption += GetSifs();
                txopConsumption += GetBlockAckDuration (m_currentHdr.GetAddr1 (), rtsTxVector,COMPRESSED_BLOCK_ACK);
                if(m_stationManager->NeedRts(m_currentHdr.GetAddr1(), &m_currentHdr, currentAggregatedPacket))
                {
                    txopConsumption += 2*GetCtsDuration (m_currentHdr.GetAddr1 (), rtsTxVector);
                    txopConsumption += 2*GetSifs(); 
                }
                //update txoplimit
                NS_LOG_DEBUG("original TXOPLIMIT=" << m_stationManager->GetTxop(m_currentHdr.GetAddr1(), &m_currentHdr) << " txopConsumption=" << txopConsumption);
                if(txopConsumption > m_stationManager->GetTxop(m_currentHdr.GetAddr1(), &m_currentHdr))
                    m_stationManager->SetTxop(m_currentHdr.GetAddr1(), &m_currentHdr, MicroSeconds(0));
                else
                    m_stationManager->SetTxop(m_currentHdr.GetAddr1(), &m_currentHdr, m_stationManager->GetTxop(m_currentHdr.GetAddr1(),&m_currentHdr)-txopConsumption);
                NS_LOG_DEBUG("new TXOPLIMIT=" << m_stationManager->GetTxop(m_currentHdr.GetAddr1(), &m_currentHdr));
            }


            if (isAmpdu)
            {    
                if (m_edca->m_blockAckType == BASIC_BLOCK_ACK)
                {
                    m_txParams.EnableBasicBlockAck ();
                }
                else if (m_edca->m_blockAckType == COMPRESSED_BLOCK_ACK)
                {
                    m_txParams.EnableCompressedBlockAck ();
                }
                else if (m_edca->m_blockAckType == MULTI_TID_BLOCK_ACK)
                {
                    NS_FATAL_ERROR ("Multi-tid block ack is not supported");
                }
                if (m_stationManager->NeedRtsForAmpdu (m_currentHdr.GetAddr1 (), &m_currentHdr,currentAggregatedPacket)) //skim11 NeedRts -> NeedRtsForAmpdu
                {
                    AmpduTag tag;
                    m_txParams.EnableRts ();
                    m_currentPacket = currentAggregatedPacket;
                    m_currentPacket->AddPacketTag (AmpduTag (false));
                    currentAggregatedPacket =0;
                    NS_LOG_DEBUG("current Ampdu?: "<<m_currentPacket->PeekPacketTag(tag) << "("<<m_currentPacket <<")" );
                    SendRtsForAmpdu (m_edca->m_blockAckType);
                    NS_LOG_DEBUG ("tx unicast rts");
                }
                else {
                    AmpduTag tag;
                    m_currentPacket = currentAggregatedPacket;
                    m_currentPacket->AddPacketTag (AmpduTag (false));
                    currentAggregatedPacket =0;
                    NS_LOG_DEBUG("current Ampdu?: "<<m_currentPacket->PeekPacketTag(tag) << "("<<m_currentPacket <<")" );
                    SendAmpduPacket ();
                    NS_LOG_DEBUG ("tx unicast A-MPDU");
                }
            }
            else {
                AmpduTag tag;
                m_currentPacket -> RemovePacketTag(tag);
                m_currentPacket->RemoveHeader (m_currentHdr); //bjkim 111029
                m_currentPacket->RemoveTrailer (fcs); //bjkim 111029
                NS_LOG_DEBUG("Ampdu failed");
            }

            /* 150623 by kjyoon 
             * n_mpdu is the number of total MPDUs of last A-MPDU frame.
             * This variable is used for updating MinstrelTable.
             */
            m_stationManager->SetNMpdus(m_currentHdr.GetAddr1(), &m_currentHdr, k);
            m_stationManager->UpdateSumAmpdu(m_currentHdr.GetAddr1(), &m_currentHdr, k);
            m_stationManager->SetNframes (m_currentHdr.GetAddr1 (), &m_currentHdr,k);
            return isAmpdu;
        }

    bool
        MacLow::IsNavZero (void) const
        {
            if (m_lastNavStart + m_lastNavDuration < Simulator::Now ())
            {
                return true;
            }
            else
            {
                return false;
            }
        }
    void
        MacLow::SendCtsToSelf (void)
        {
            WifiMacHeader cts;
            cts.SetType (WIFI_MAC_CTL_CTS);
            cts.SetDsNotFrom ();
            cts.SetDsNotTo ();
            cts.SetNoMoreFragments ();
            cts.SetNoRetry ();
            cts.SetAddr1 (m_self);

            WifiTxVector ctsTxVector = GetCtsToSelfTxVector (m_currentPacket, &m_currentHdr);
            WifiTxVector dataTxVector = GetDataTxVector (m_currentPacket, &m_currentHdr);

            WifiPreamble preamble;
            if (ctsTxVector.GetMode().GetModulationClass () == WIFI_MOD_CLASS_VHT)
                preamble= WIFI_PREAMBLE_VHT;
            else if (ctsTxVector.GetMode().GetModulationClass () == WIFI_MOD_CLASS_HT)
                preamble= WIFI_PREAMBLE_HT_MF;
            else
                preamble=WIFI_PREAMBLE_LONG;

            Time duration = Seconds (0);

            if (m_txParams.HasDurationId ())
            {
                duration += m_txParams.GetDurationId ();
            }
            else
            {
                duration += GetSifs ();
                duration += m_phy->CalculateTxDuration (GetSize (m_currentPacket,&m_currentHdr),
                        dataTxVector, preamble);
                if (m_txParams.MustWaitBasicBlockAck ())
                {

                    duration += GetSifs ();
                    duration += GetBlockAckDuration (m_currentHdr.GetAddr1 (), dataTxVector, BASIC_BLOCK_ACK);
                }
                else if (m_txParams.MustWaitCompressedBlockAck ())
                {
                    duration += GetSifs ();
                    duration += GetBlockAckDuration (m_currentHdr.GetAddr1 (), dataTxVector, COMPRESSED_BLOCK_ACK);
                }
                else if (m_txParams.MustWaitAck ())
                {
                    duration += GetSifs ();
                    duration += GetAckDuration (m_currentHdr.GetAddr1 (), dataTxVector);
                }
                if (m_txParams.HasNextPacket ())
                {
                    duration += GetSifs ();
                    duration += m_phy->CalculateTxDuration (m_txParams.GetNextPacketSize (),
                            dataTxVector, preamble);
                    if (m_txParams.MustWaitCompressedBlockAck ())
                    {
                        duration += GetSifs ();
                        duration += GetBlockAckDuration (m_currentHdr.GetAddr1 (), dataTxVector, COMPRESSED_BLOCK_ACK);
                    }
                    else if (m_txParams.MustWaitAck ())
                    {
                        duration += GetSifs ();
                        duration += GetAckDuration (m_currentHdr.GetAddr1 (), dataTxVector);
                    }
                }
            }

            cts.SetDuration (duration);

            Ptr<Packet> packet = Create<Packet> ();
            packet->AddHeader (cts);
            WifiMacTrailer fcs;
            packet->AddTrailer (fcs);

            //shbyeon add duplicate tag for ack/blockack
            DuplicateTag dtag(dataTxVector.GetMode().GetBandwidth()/1000000, 0, CTS);
            packet->AddPacketTag (dtag);

            ForwardDown (packet, &cts, ctsTxVector,preamble);

            Time txDuration = m_phy->CalculateTxDuration (GetCtsSize (), ctsTxVector, preamble);
            txDuration += GetSifs ();
            NS_ASSERT (m_sendDataEvent.IsExpired ());

            m_sendDataEvent = Simulator::Schedule (txDuration,
                    &MacLow::SendDataAfterCts, this,
                    cts.GetAddr1 (),
                    duration,
                    ctsTxVector.GetMode(), 20);
        }
    void
        MacLow::SendCtsAfterRts (Mac48Address source, Time duration, WifiMode rtsTxMode, double rtsSnr, uint16_t txBandwidth)
        {
            NS_LOG_FUNCTION (this << source << duration << rtsTxMode << rtsSnr);
            /* send a CTS when you receive a RTS
             * right after SIFS.
             */
            m_stationManager->SetCurrentBandwidth(m_currentHdr.GetAddr1 (), &m_currentHdr, txBandwidth);
            WifiTxVector ctsTxVector = GetCtsTxVectorForRts (source, rtsTxMode);
            WifiMacHeader cts;
            cts.SetType (WIFI_MAC_CTL_CTS);
            cts.SetDsNotFrom ();
            cts.SetDsNotTo ();
            cts.SetNoMoreFragments ();
            cts.SetNoRetry ();
            cts.SetAddr1 (source);
            duration -= GetCtsDuration (source, ctsTxVector);
            duration -= GetSifs ();
            NS_ASSERT (duration >= MicroSeconds (0));
            cts.SetDuration (duration);

            Ptr<Packet> packet = Create<Packet> ();
            packet->AddHeader (cts);
            WifiMacTrailer fcs;
            packet->AddTrailer (fcs);

            SnrTag tag;
            tag.Set (rtsSnr);
            packet->AddPacketTag (tag);

            WifiPreamble preamble;
            if (ctsTxVector.GetMode().GetModulationClass () == WIFI_MOD_CLASS_VHT)
                preamble= WIFI_PREAMBLE_VHT;
            else if (ctsTxVector.GetMode().GetModulationClass () == WIFI_MOD_CLASS_HT)
                preamble= WIFI_PREAMBLE_HT_MF;
            else
                preamble=WIFI_PREAMBLE_LONG;

            DuplicateTag dtag(txBandwidth, 0, CTS);
            packet->AddPacketTag(dtag);

            ForwardDown (packet, &cts, ctsTxVector,preamble);
        }

    void
        MacLow::SendDataAfterCts (Mac48Address source, Time duration, WifiMode txMode, uint16_t txBandwidth)
        {
            NS_LOG_FUNCTION (this);
            /* send the third step in a
             * RTS/CTS/DATA/ACK hanshake
             */
            NS_ASSERT (m_currentPacket != 0);
            m_stationManager->SetCurrentBandwidth(m_currentHdr.GetAddr1 (), &m_currentHdr, txBandwidth);
            //WifiTxVector dataTxVector = GetDataTxVector (m_currentPacket, &m_currentHdr);
            //shbyeon RTSCTS buf fix
            WifiTxVector dataTxVector = m_stationManager->GetPrevTxVector(m_currentHdr.GetAddr1(),&m_currentHdr);

            WifiPreamble preamble;       
            if (dataTxVector.GetMode().GetModulationClass () == WIFI_MOD_CLASS_VHT)
                preamble= WIFI_PREAMBLE_VHT;
            else if (m_phy->GetGreenfield() && m_stationManager->GetGreenfieldSupported (m_currentHdr.GetAddr1 ()))
                //In the future has to make sure that receiver has greenfield enabled
                preamble= WIFI_PREAMBLE_HT_GF;
            else if (dataTxVector.GetMode().GetModulationClass () == WIFI_MOD_CLASS_HT)
                preamble= WIFI_PREAMBLE_HT_MF;
            else
                preamble=WIFI_PREAMBLE_LONG;

            Time newDuration = Seconds (0);
            newDuration += GetSifs ();
            newDuration += GetAckDuration (m_currentHdr.GetAddr1 (), dataTxVector);
            Time txDuration = m_phy->CalculateTxDuration (GetSize (m_currentPacket, &m_currentHdr),
                    dataTxVector, preamble);
            NS_LOG_DEBUG("packetSize=" << GetSize(m_currentPacket, &m_currentHdr) << " dataRate=" << dataTxVector.GetMode() << " preamble=" << (int)preamble);
            NS_LOG_DEBUG("duration=" << duration.GetMicroSeconds() << " txDuration=" << txDuration.GetMicroSeconds());
            duration -= txDuration;
            duration -= GetSifs ();

            duration = std::max (duration, newDuration);
            NS_ASSERT (duration >= MicroSeconds (0));
            m_currentHdr.SetDuration (duration);

            StartDataTxTimers (dataTxVector);

            m_currentPacket->AddHeader (m_currentHdr);
            WifiMacTrailer fcs;
            m_currentPacket->AddTrailer (fcs);

            ForwardDown (m_currentPacket, &m_currentHdr, dataTxVector,preamble);
            m_currentPacket = 0;
        }

    //shbyeo nsend ampdu after cts
    void
        MacLow::SendAmpduAfterCts (Mac48Address source, Time duration, WifiMode txMode, uint16_t txBandwidth)
        {
            NS_LOG_FUNCTION (this);
            /* send the third step in a
             * RTS/CTS/DATA/ACK hanshake
             */
            NS_ASSERT (m_currentPacket != 0);

            m_stationManager->SetCurrentBandwidth(m_currentHdr.GetAddr1 (), &m_currentHdr, txBandwidth);
            //WifiTxVector dataTxVector = GetDataTxVector (m_currentPacket, &m_currentHdr);
            //shbyeon RTSCTS buf fix
            WifiTxVector dataTxVector = m_stationManager->GetPrevTxVector(m_currentHdr.GetAddr1(),&m_currentHdr);
            Ptr<EdcaTxopN> m_edca = m_edcas[QosUtilsMapTidToAc (m_currentHdr.GetQosTid ())];

            Time newDuration = Seconds (0);
            newDuration += GetSifs ();
            WifiTxVector blockAckTxVector = GetBlockAckTxVector(m_currentHdr.GetAddr1(), dataTxVector.GetMode());
            //WifiTxVector blockAckTxVector = GetRtsTxVector (m_currentPacket, &m_currentHdr);
            duration += GetBlockAckDuration (m_currentHdr.GetAddr1 (), blockAckTxVector, m_edca->m_blockAckType);
            newDuration += GetBlockAckDuration (m_currentHdr.GetAddr1 (), blockAckTxVector, m_edca->m_blockAckType);

            WifiPreamble preamble;
            if (dataTxVector.GetMode().GetModulationClass () == WIFI_MOD_CLASS_VHT)
                preamble= WIFI_PREAMBLE_VHT;
            else if (dataTxVector.GetMode().GetModulationClass () == WIFI_MOD_CLASS_HT)
                preamble= WIFI_PREAMBLE_HT_MF;
            else
                preamble=WIFI_PREAMBLE_LONG;


            Time txDuration = m_phy->CalculateTxDuration (m_currentPacket->GetSize (),
                    dataTxVector, preamble);
            duration -= txDuration;
            duration -= GetSifs ();

            NS_LOG_DEBUG("duration: " << duration <<
                    ", newDuration: " << newDuration);

            duration = std::max (duration, newDuration);
            NS_ASSERT (duration >= MicroSeconds (0));

            m_currentHdr.SetDuration (duration);

            StartAmpduTxTimers (dataTxVector);
            ForwardDown (m_currentPacket, &m_currentHdr, dataTxVector, preamble);
            m_currentPacket = 0;
        }


    void
        MacLow::WaitSifsAfterEndTx (void)
        {
            m_listener->StartNext ();
        }

    void 
        MacLow::EndTxNoAck (void)
        {
            MacLowTransmissionListener *listener = m_listener;
            m_listener = 0;
            listener->EndTxNoAck ();
        }

    void
        MacLow::FastAckFailedTimeout (void)
        {
            NS_LOG_FUNCTION (this);
            MacLowTransmissionListener *listener = m_listener;
            m_listener = 0;
            listener->MissedAck ();
            NS_LOG_DEBUG ("fast Ack busy but missed");
        }

    void
        MacLow::SendAckAfterData (Mac48Address source, Time duration, WifiMode dataTxMode, double dataSnr)
        {
            NS_LOG_FUNCTION (this);
            /* send an ACK when you receive
             * a packet after SIFS.
             */
            WifiTxVector ackTxVector = GetAckTxVector (source, dataTxMode);
            WifiMacHeader ack;
            ack.SetType (WIFI_MAC_CTL_ACK);
            ack.SetDsNotFrom ();
            ack.SetDsNotTo ();
            ack.SetNoRetry ();
            ack.SetNoMoreFragments ();
            ack.SetAddr1 (source);
            duration -= GetAckDuration (ackTxVector);
            duration -= GetSifs ();
            NS_ASSERT (duration >= MicroSeconds (0));
            ack.SetDuration (duration);
            NS_LOG_DEBUG ("duration=" << duration.GetMicroSeconds () << " us, Ackduration=" << 
                    GetAckDuration (ackTxVector).GetMicroSeconds () << "us, Sifs=" << GetSifs ().GetMicroSeconds () << " us");

            Ptr<Packet> packet = Create<Packet> ();
            packet->AddHeader (ack);
            WifiMacTrailer fcs;
            packet->AddTrailer (fcs);

            SnrTag tag;
            tag.Set (dataSnr);
            packet->AddPacketTag (tag);

            //since ACK is a control response it can't have Fomat =GF
            WifiPreamble preamble;
            if (ackTxVector.GetMode().GetModulationClass () == WIFI_MOD_CLASS_VHT)
                preamble= WIFI_PREAMBLE_VHT;
            else if (ackTxVector.GetMode().GetModulationClass () == WIFI_MOD_CLASS_HT)
                preamble= WIFI_PREAMBLE_HT_MF;
            else
                preamble=WIFI_PREAMBLE_LONG;

            //shbyeon add duplicate tag for ack/blockack
            DuplicateTag dtag(dataTxMode.GetBandwidth()/1000000, 0, ACK);
            packet->AddPacketTag (dtag);
            ForwardDown (packet, &ack, ackTxVector, preamble);
        }

    bool
        MacLow::StoreMpduIfNeeded (Ptr<Packet> packet, WifiMacHeader hdr)
        {
            AgreementsI it = m_bAckAgreements.find (std::make_pair (hdr.GetAddr2 (), hdr.GetQosTid ()));
            if (it != m_bAckAgreements.end ())
            {
                WifiMacTrailer fcs;
                packet->RemoveTrailer (fcs);
                BufferedPacket bufferedPacket (packet, hdr);

                uint16_t endSequence = ((*it).second.first.GetStartingSequence () + 2047) % 4096;
                uint16_t mappedSeqControl = QosUtilsMapSeqControlToUniqueInteger (hdr.GetSequenceControl (), endSequence);

                BufferedPacketI i = (*it).second.second.begin ();
                for (; i != (*it).second.second.end ()
                        && QosUtilsMapSeqControlToUniqueInteger ((*i).second.GetSequenceControl (), endSequence) < mappedSeqControl; i++)
                {
                    ;
                }
                (*it).second.second.insert (i, bufferedPacket);

                //Update block ack cache
                BlockAckCachesI j = m_bAckCaches.find (std::make_pair (hdr.GetAddr2 (), hdr.GetQosTid ()));
                NS_ASSERT (j != m_bAckCaches.end ());
                (*j).second.UpdateWithMpdu (&hdr);

                return true;
            }
            return false;
        }

    void
        MacLow::CreateBlockAckAgreement (const MgtAddBaResponseHeader *respHdr, Mac48Address originator,
                uint16_t startingSeq)
        {
            uint8_t tid = respHdr->GetTid ();
            BlockAckAgreement agreement (originator, tid);
            if (respHdr->IsImmediateBlockAck ())
            {
                agreement.SetImmediateBlockAck ();
            }
            else
            {
                agreement.SetDelayedBlockAck ();
            }
            agreement.SetAmsduSupport (respHdr->IsAmsduSupported ());
            agreement.SetBufferSize (respHdr->GetBufferSize () + 1);
            agreement.SetTimeout (respHdr->GetTimeout ());
            agreement.SetStartingSequence (startingSeq);

            std::list<BufferedPacket> buffer (0);
            AgreementKey key (originator, respHdr->GetTid ());
            AgreementValue value (agreement, buffer);
            m_bAckAgreements.insert (std::make_pair (key, value));

            BlockAckCache cache;
            cache.Init (startingSeq, respHdr->GetBufferSize () + 1);
            m_bAckCaches.insert (std::make_pair (key, cache));

            if (respHdr->GetTimeout () != 0)
            {
                AgreementsI it = m_bAckAgreements.find (std::make_pair (originator, respHdr->GetTid ()));
                Time timeout = MicroSeconds (1024 * agreement.GetTimeout ());

                AcIndex ac = QosUtilsMapTidToAc (agreement.GetTid ());

                it->second.first.m_inactivityEvent = Simulator::Schedule (timeout,
                        &MacLowBlockAckEventListener::BlockAckInactivityTimeout,
                        m_edcaListeners[ac],
                        originator, tid);
            }
        }

    void
        MacLow::DestroyBlockAckAgreement (Mac48Address originator, uint8_t tid)
        {
            AgreementsI it = m_bAckAgreements.find (std::make_pair (originator, tid));
            if (it != m_bAckAgreements.end ())
            {
                RxCompleteBufferedPacketsWithSmallerSequence (it->second.first.GetStartingSequenceControl (), originator, tid);
                RxCompleteBufferedPacketsUntilFirstLost (originator, tid);
                m_bAckAgreements.erase (it);

                BlockAckCachesI i = m_bAckCaches.find (std::make_pair (originator, tid));
                NS_ASSERT (i != m_bAckCaches.end ());
                m_bAckCaches.erase (i);
            }
        }

    void
        MacLow::RxCompleteBufferedPacketsWithSmallerSequence (uint16_t seq, Mac48Address originator, uint8_t tid)
        {
            NS_LOG_DEBUG("smaller sequence control");
            AgreementsI it = m_bAckAgreements.find (std::make_pair (originator, tid));
            if (it != m_bAckAgreements.end ())
            {
                uint16_t endSequence = ((*it).second.first.GetStartingSequence () + 2047) % 4096;
                //shbyeon ampdu bug
                uint16_t mappedStart = QosUtilsMapSeqControlToUniqueInteger (seq, endSequence);
                BufferedPacketI last = (*it).second.second.begin ();
                uint16_t guard = 0;
                if (last != (*it).second.second.end ())
                {
                    guard = (*it).second.second.begin ()->second.GetSequenceControl ();
                }
                NS_LOG_DEBUG("endSequence=" << endSequence << " mappedStart=" << mappedStart << " guard=" << guard);
                BufferedPacketI i = (*it).second.second.begin ();
                for (; i != (*it).second.second.end ()
                        //shbyeon ampdu bug
                        && QosUtilsMapSeqControlToUniqueInteger ((*i).second.GetSequenceControl (), endSequence) < mappedStart;)
                {
                    if (guard == (*i).second.GetSequenceControl ())
                    {
                        if (!(*i).second.IsMoreFragments ())
                        {
                            while (last != i)
                            {
                                m_rxCallback ((*last).first, &(*last).second);
                                last++;
                            }
                            m_rxCallback ((*last).first, &(*last).second);
                            last++;
                            /* go to next packet */
                            //while (i != (*it).second.second.end () && ((guard >> 4) & 0x0fff) == (*i).second.GetSequenceNumber ())
                            //shbyeon ampdu bug
                            while (i != (*it).second.second.end () && guard == (*i).second.GetSequenceControl ())
                            {
                                i++;
                            }
                            if (i != (*it).second.second.end ())
                            {
                                //shbyeon ampdu bug
                                guard = (*i).second.GetSequenceControl ();

                                last = i;
                            }
                        }
                        else
                        {
                            guard++;
                            NS_LOG_DEBUG("update guard by 1 = " << guard);
                        }
                    }
                    else
                    {
                        /* go to next packet */
                        while (i != (*it).second.second.end () && guard == (*i).second.GetSequenceControl ())
                        {
                            i++;
                        }
                        if (i != (*it).second.second.end ())
                        {
                            guard = (*i).second.GetSequenceControl ();
                            last = i;
                        }
                    }
                }
                NS_LOG_DEBUG("erase this packet");
                (*it).second.second.erase ((*it).second.second.begin (), i);
            }
        }

    void
        MacLow::RxCompleteBufferedPacketsUntilFirstLost (Mac48Address originator, uint8_t tid)
        {
            AgreementsI it = m_bAckAgreements.find (std::make_pair (originator, tid));
            if (it != m_bAckAgreements.end ())
            {
                //shbyeon ampdu bug
                uint16_t guard = (*it).second.first.GetStartingSequenceControl ();
                NS_LOG_DEBUG("ok, here before update " << (uint16_t) ((guard >> 4) & 0x0fff)); 

                BufferedPacketI lastComplete = (*it).second.second.begin ();
                BufferedPacketI i = (*it).second.second.begin ();
                for (; i != (*it).second.second.end () && guard == (*i).second.GetSequenceControl (); i++)
                {
                    if (!(*i).second.IsMoreFragments ())
                    {
                        while (lastComplete != i)
                        {
                            m_rxCallback ((*lastComplete).first, &(*lastComplete).second);
                            lastComplete++;
                        }
                        m_rxCallback ((*lastComplete).first, &(*lastComplete).second);
                        lastComplete++;
                    }
                    guard = (*i).second.IsMoreFragments () ? (guard + 1) : ((guard + 16) & 0xfff0);
                    NS_LOG_DEBUG("ok, here gaurd update! " << (uint16_t) ((guard >> 4) & 0x0fff) << " " << (*i).second.GetSequenceNumber()); 
                }
                NS_LOG_DEBUG("ok, here update to " << (uint16_t) ((guard >> 4) & 0x0fff)); 
                (*it).second.first.SetStartingSequenceControl(guard);
                /* All packets already forwarded to WifiMac must be removed from buffer:
                   [begin (), lastComplete) */
                (*it).second.second.erase ((*it).second.second.begin (), lastComplete);
            }
        }

    void
        MacLow::SendBlockAckResponse (const CtrlBAckResponseHeader* blockAck, Mac48Address originator, bool immediate,
                Time duration, WifiMode blockAckReqTxMode)
        {
            Ptr<Packet> packet = Create<Packet> ();
            packet->AddHeader (*blockAck);

            WifiMacHeader hdr;
            hdr.SetType (WIFI_MAC_CTL_BACKRESP);
            hdr.SetAddr1 (originator);
            hdr.SetAddr2 (GetAddress ());
            hdr.SetDsNotFrom ();
            hdr.SetDsNotTo ();
            hdr.SetNoRetry ();
            hdr.SetNoMoreFragments ();
            WifiTxVector blockAckTxVector = GetBlockAckTxVector (originator, blockAckReqTxMode);
            NS_LOG_DEBUG("blockAck duration="<<duration<<" mode=" << blockAckReqTxMode << " backMode=" << blockAckTxVector.GetMode());
            //WifiTxVector blockAckTxVector = GetRtsTxVector (m_currentPacket, &m_currentHdr);
            WifiTxVector blockAckReqTxVector;
            blockAckReqTxVector.SetMode(blockAckReqTxMode);
            blockAckReqTxVector.SetNss(1);
            blockAckReqTxVector.SetStbc(false);
            m_currentPacket = packet;
            m_currentHdr = hdr;
            if (immediate)
            {
                m_txParams.DisableAck ();
                duration -= GetSifs ();
                if (blockAck->IsBasic ())
                {
                    duration -= GetBlockAckDuration (originator, blockAckTxVector, BASIC_BLOCK_ACK);
                }
                else if (blockAck->IsCompressed ())
                {
                    duration -= GetBlockAckDuration (originator, blockAckTxVector, COMPRESSED_BLOCK_ACK);
                }
                else if (blockAck->IsMultiTid ())
                {
                    NS_FATAL_ERROR ("Multi-tid block ack is not supported.");
                }
            }
            else
            {
                m_txParams.EnableAck ();
                duration += GetSifs ();
                duration += GetAckDuration (originator, blockAckReqTxVector);
            }
            m_txParams.DisableNextData ();

            if (!immediate)
            {
                StartDataTxTimers (blockAckTxVector);
            }
            NS_LOG_DEBUG("blockack duration=" << duration);
            NS_ASSERT (duration >= MicroSeconds (0));
            hdr.SetDuration (duration);
            //here should be present a control about immediate or delayed block ack
            //for now we assume immediate
            packet->AddHeader (hdr);
            WifiMacTrailer fcs;
            packet->AddTrailer (fcs);
            WifiPreamble preamble;
            if (blockAckTxVector.GetMode().GetModulationClass () == WIFI_MOD_CLASS_VHT)
                preamble= WIFI_PREAMBLE_VHT;
            else if (blockAckTxVector.GetMode().GetModulationClass () == WIFI_MOD_CLASS_HT)
                preamble= WIFI_PREAMBLE_HT_MF;
            else
                preamble=WIFI_PREAMBLE_LONG;
            //shbyeon add duplicate tag for ack/blockack
            DuplicateTag dtag(blockAckReqTxMode.GetBandwidth()/1000000, 0, BACK);
            packet->AddPacketTag (dtag);
            ForwardDown (packet, &hdr, blockAckTxVector,preamble);
            m_currentPacket = 0;
        }

    void
        MacLow::SendBlockAckAfterBlockAckRequest (const CtrlBAckRequestHeader reqHdr, Mac48Address originator,
                Time duration, WifiMode blockAckReqTxMode)
        {
            NS_LOG_FUNCTION (this);
            CtrlBAckResponseHeader blockAck;
            uint8_t tid;
            bool immediate = false;
            if (!reqHdr.IsMultiTid ())
            {
                tid = reqHdr.GetTidInfo ();
                AgreementsI it = m_bAckAgreements.find (std::make_pair (originator, tid));
                if (it != m_bAckAgreements.end ())
                {
                    NS_LOG_DEBUG("blockack request makes seq=" << reqHdr.GetStartingSequence());
                    blockAck.SetStartingSequence (reqHdr.GetStartingSequence ());
                    blockAck.SetTidInfo (tid);
                    immediate = (*it).second.first.IsImmediateBlockAck ();
                    if (reqHdr.IsBasic ())
                    {
                        blockAck.SetType (BASIC_BLOCK_ACK);
                    }
                    else if (reqHdr.IsCompressed ())
                    {
                        blockAck.SetType (COMPRESSED_BLOCK_ACK);
                    }
                    BlockAckCachesI i = m_bAckCaches.find (std::make_pair (originator, tid));
                    NS_ASSERT (i != m_bAckCaches.end ());
                    (*i).second.FillBlockAckBitmap (&blockAck);
                    NS_LOG_DEBUG ("Got block Ack Req with seq " << reqHdr.GetStartingSequence ());
                    //shbyeon ampdu bug fix
                    if (!m_stationManager->HasHtSupported () && !m_stationManager->HasVhtSupported ())
                    {
                        /* All packets with smaller sequence than starting sequence control must be passed up to Wifimac
                         * See 9.10.3 in IEEE 802.11e standard.
                         */
                        RxCompleteBufferedPacketsWithSmallerSequence (reqHdr.GetStartingSequenceControl (), originator, tid);
                        RxCompleteBufferedPacketsUntilFirstLost (originator, tid);
                    }
                    else
                    {
                        if (!QosUtilsIsOldPacket ((*it).second.first.GetStartingSequence (), reqHdr.GetStartingSequence ()))
                        {
                            (*it).second.first.SetStartingSequence (reqHdr.GetStartingSequence ());
                            RxCompleteBufferedPacketsWithSmallerSequence (reqHdr.GetStartingSequenceControl (), originator, tid);
                            RxCompleteBufferedPacketsUntilFirstLost (originator, tid);
                        }
                    }

                    //RxCompleteBufferedPacketsWithSmallerSequence (reqHdr.GetStartingSequenceControl (), originator, tid);
                    //RxCompleteBufferedPacketsUntilFirstLost (originator, tid);
                }
                else
                {
                    NS_LOG_DEBUG ("there's not a valid block ack agreement with " << originator);
                }
            }
            else
            {
                NS_FATAL_ERROR ("Multi-tid block ack is not supported.");
            }

            SendBlockAckResponse (&blockAck, originator, immediate, duration, blockAckReqTxMode);
        }

    //shbyeon send block ack after reception of ampdu
    void
        MacLow::SendBlockAckWithoutBlockAckRequest (Mac48Address originator,
                Time duration, WifiMode AmpduTxMode,
                uint8_t tid, uint16_t startingSeq)
        {
            NS_LOG_FUNCTION (this);
            CtrlBAckResponseHeader blockAck;
            bool immediate = false;
            AgreementsI it = m_bAckAgreements.find (std::make_pair (originator, tid));
            if (it != m_bAckAgreements.end ())
            {
                NS_LOG_DEBUG ("bAckAggreements exist! startingSeq=" << startingSeq);
                blockAck.SetStartingSequence (startingSeq);
                blockAck.SetTidInfo (tid);
                immediate = (*it).second.first.IsImmediateBlockAck ();
                blockAck.SetType (COMPRESSED_BLOCK_ACK);
                BlockAckCachesI i = m_bAckCaches.find (std::make_pair (originator, tid));
                NS_ASSERT (i != m_bAckCaches.end ());
                (*i).second.FillBlockAckBitmap (&blockAck);

                BufferedPacketI ii = (*it).second.second.begin ();
                for (; ii != (*it).second.second.end () ; ii++)
                {
                    NS_LOG_DEBUG ("info: SequenceControl(Bpacket): " << (*ii).second.GetSequenceControl () <<  " SequenceNumber(Bpacket): " << (*ii).second.GetSequenceNumber () << " Bpacket Sizer: " << (*it).second.second.size());
                }
                RxCompleteBufferedPacketsWithSmallerSequence ((startingSeq <<4) & 0xfff0 , originator, tid);
                NS_LOG_FUNCTION ("RxCompleteBufferedPacketWithSmallerSequence..");

                ii = (*it).second.second.begin ();
                for (; ii != (*it).second.second.end () ; ii++)
                {
                    NS_LOG_DEBUG ("SequenceControl(Bpacket): " << (*ii).second.GetSequenceControl () <<  " SequenceNumber(Bpacket): " << (*ii).second.GetSequenceNumber () << " Bpacket Sizer: " << (*it).second.second.size());
                }

                RxCompleteBufferedPacketsUntilFirstLost (originator, tid);
                NS_LOG_DEBUG ("RxCompleteBufferedPacketUltilFirstLost..");
                ii = (*it).second.second.begin ();
                for (; ii != (*it).second.second.end () ; ii++)
                {
                    NS_LOG_FUNCTION ("SequenceControl(Bpacket): " << (*ii).second.GetSequenceControl () <<  " SequenceNumber(Bpacket): " << (*ii).second.GetSequenceNumber () << " Bpacket Sizer: " << (*it).second.second.size());
                }

            }
            else
            {
                NS_LOG_DEBUG ("there's not a valid block ack agreement with " << originator);
            }
            SendBlockAckResponse (&blockAck, originator, immediate, duration, AmpduTxMode);
        }


    void
        MacLow::ResetBlockAckInactivityTimerIfNeeded (BlockAckAgreement &agreement)
        {
            if (agreement.GetTimeout () != 0)
            {
                NS_ASSERT (agreement.m_inactivityEvent.IsRunning ());
                agreement.m_inactivityEvent.Cancel ();
                Time timeout = MicroSeconds (1024 * agreement.GetTimeout ());

                AcIndex ac = QosUtilsMapTidToAc (agreement.GetTid ());
                //std::map<AcIndex, MacLowTransmissionListener*>::iterator it = m_edcaListeners.find (ac);
                //NS_ASSERT (it != m_edcaListeners.end ());

                agreement.m_inactivityEvent = Simulator::Schedule (timeout,
                        &MacLowBlockAckEventListener::BlockAckInactivityTimeout,
                        m_edcaListeners[ac],
                        agreement.GetPeer (),
                        agreement.GetTid ());
            }
        }

    void
        MacLow::RegisterBlockAckListenerForAc (enum AcIndex ac, MacLowBlockAckEventListener *listener)
        {
            m_edcaListeners.insert (std::make_pair (ac, listener));
        }

    //ohlee - set ampdu
    void
        MacLow::SetMpduAggregator (Ptr<MpduAggregator> aggr, enum AcIndex ac)
        {
            NS_LOG_FUNCTION(this);
            m_aggregators.insert (std::make_pair (ac, aggr));
        }

    //shbyeon
    void
        MacLow::SetMaxPpduTime (Time maxPpduTime){
            m_maxPpduTime = maxPpduTime;
        }

    bool
        MacLow::noCurrentPacket (void) {
            if (m_currentPacket == 0)
                return true;
            else
                return false;
        }

    //802.11ac channel bonding
    DynamicAccessFlag
        MacLow::NeedRestartBackoff()
        {
            return m_phy->RestartBackoff(m_operationalBandwidth);
        }
    void
        MacLow::SetOperationalBandwidth (uint16_t bw)
        {
            m_operationalBandwidth = bw;
        }
    uint16_t
        MacLow::GetOperationalBandwidth (void)
        {
            return m_operationalBandwidth;
        }
    uint16_t
        MacLow::GetDynamicAccess (void)
        {
            return m_da;
        }
    void
        MacLow::SetDynamicAccess (uint16_t da)
        {
            m_da = da;
        }
} // namespace ns3
