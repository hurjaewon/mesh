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
#include "sta-wifi-mac.h"

#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/string.h"
#include "ns3/pointer.h"
#include "ns3/boolean.h"
#include "ns3/enum.h"
#include "ns3/trace-source-accessor.h"

#include "qos-tag.h"
#include "mac-low.h"
#include "dcf-manager.h"
#include "mac-rx-middle.h"
#include "mac-tx-middle.h"
#include "wifi-mac-header.h"
#include "msdu-aggregator.h"
#include "amsdu-subframe-header.h"
#include "mgt-headers.h"
#include "ht-capabilities.h"
#include "ns3/llc-snap-header.h"

NS_LOG_COMPONENT_DEFINE ("StaWifiMac");

namespace ns3 {

    class PhyStaMacListener : public ns3::WifiPhyListener
    {
        public:
            PhyStaMacListener (ns3::StaWifiMac *staMac)
                : m_staMac (staMac)
            {
            }
            virtual ~PhyStaMacListener ()
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
            virtual void NotifyTxStart (Time duration)
            {
            }
            virtual void NotifyTxStart (Time duration, double txPowerDbm)
            {
            }
            virtual void NotifySleep (void)
            {
            }
            virtual void NotifyWakeup (void)
            {
            }
            virtual void NotifyMaybeCcaBusyStart (Time duration)
            {
                m_staMac->NotifyCcaBusyOccurred ();
            }
            virtual void NotifySwitchingStart (Time duration)
            {
                m_staMac->NotifySwitchingStartNow (duration);
            }
        private:
            ns3::StaWifiMac *m_staMac;
    };

    /*
     * The state machine for this STA is:
     --------------                                          -----------
     | Associated |   <--------------------      ------->    | Refused |
     --------------                        \    /            -----------
     \                                   \  /
     \    -----------------     -----------------------------
     \-> | Beacon Missed | --> | Wait Association Response |
     -----------------     -----------------------------
     \                       ^
     \                      |
     \    -----------------------
     \-> | Wait Probe Response |
     -----------------------
     */


    NS_OBJECT_ENSURE_REGISTERED (StaWifiMac);

    TypeId
        StaWifiMac::GetTypeId (void)
        {
            static TypeId tid = TypeId ("ns3::StaWifiMac")
                .SetParent<RegularWifiMac> ()
                .AddConstructor<StaWifiMac> ()
                .AddAttribute ("ProbeRequestTimeout", "The interval between two consecutive probe request attempts.",
                        TimeValue (Seconds (0.05)),
                        MakeTimeAccessor (&StaWifiMac::m_probeRequestTimeout),
                        MakeTimeChecker ())
                .AddAttribute ("AssocRequestTimeout", "The interval between two consecutive assoc request attempts.",
                        TimeValue (Seconds (0.5)),
                        MakeTimeAccessor (&StaWifiMac::m_assocRequestTimeout),
                        MakeTimeChecker ())
                .AddAttribute ("MaxMissedBeacons",
                        "Number of beacons which much be consecutively missed before "
                        "we attempt to restart association.",
                        UintegerValue (10),
                        MakeUintegerAccessor (&StaWifiMac::m_maxMissedBeacons),
                        MakeUintegerChecker<uint32_t> ())
                .AddAttribute ("ActiveProbing", "If true, we send probe requests. If false, we don't. NOTE: if more than one STA in your simulation is using active probing, you should enable it at a different simulation time for each STA, otherwise all the STAs will start sending probes at the same time resulting in collisions. See bug 1060 for more info.",
                        BooleanValue (false),
                        MakeBooleanAccessor (&StaWifiMac::SetActiveProbing, &StaWifiMac::GetActiveProbing),
                        MakeBooleanChecker ())
                .AddAttribute ("ScanType",
                        "The type of scanning for a BSS.",
                        EnumValue (NOTSUPPORT),
                        MakeEnumAccessor (&StaWifiMac::m_scanType),
                        MakeEnumChecker (NOTSUPPORT, "NotSupport",
                            ACTIVE, "Active",
                            PASSIVE, "Passive"))
                .AddAttribute ("MaxScanningChannelNumber",
                        "Specifies maximum number of channels that are examined when scanning for a BSS.",
                        UintegerValue (11),
                        MakeUintegerAccessor (&StaWifiMac::m_maxChannelNumber),
                        MakeUintegerChecker<uint16_t> ())
                .AddAttribute ("MaxChannelTime",
                        "The maximum time to spend on each channel when scanning.",
                        TimeValue (Seconds (0.05)),
                        MakeTimeAccessor (&StaWifiMac::m_maxChannelTime),
                        MakeTimeChecker ())
                .AddAttribute ("MinChannelTime",
                        "The minimum time to spend on each channel when scanning.",
                        TimeValue (Seconds (0.02)),
                        MakeTimeAccessor (&StaWifiMac::m_minChannelTime),
                        MakeTimeChecker ())
                .AddTraceSource ("Assoc", "Associated with an access point.",
                        MakeTraceSourceAccessor (&StaWifiMac::m_assocLogger))
                .AddTraceSource ("DeAssoc", "Association with an access point lost.",
                        MakeTraceSourceAccessor (&StaWifiMac::m_deAssocLogger))
                ;
            return tid;
        }

    StaWifiMac::StaWifiMac ()
        : m_state (BEACON_MISSED),
        m_probeRequestEvent (),
        m_assocRequestEvent (),
        m_beaconWatchdogEnd (Seconds (0.0)),
        m_scanType (NOTSUPPORT),
        m_maxChannelTime (Seconds (0.0)),
        m_minChannelTime (Seconds (0.0)),
        m_maxChannelNumber (0),
        m_scanChannelNumber (0),
        m_bCcaBusyOccurred (false),
        m_scanResults (std::vector<ScanningEntry> ())
    {
        NS_LOG_FUNCTION (this);
        m_rxMiddle->SetForwardSnrCallback (MakeCallback (&StaWifiMac::SnrReceive, this));
        m_low->SetSnrRxCallback (MakeCallback (&MacRxMiddle::SnrReceive, m_rxMiddle));


        // Let the lower layers know that we are acting as a non-AP STA in
        // an infrastructure BSS.
        SetTypeOfStation (STA);
    }

    StaWifiMac::~StaWifiMac ()
    {
        NS_LOG_FUNCTION (this);
    }

    void
        StaWifiMac::SetWifiPhy (Ptr<WifiPhy> phy)
        {
            RegularWifiMac::SetWifiPhy (phy);
            SetupStaMacListener (phy);
        }

    void
        StaWifiMac::DoDispose ()
        {
            RegularWifiMac::DoDispose ();
            delete m_phyStaMacListener;
            m_phyStaMacListener = NULL;
        }

    void
        StaWifiMac::SetMaxMissedBeacons (uint32_t missed)
        {
            NS_LOG_FUNCTION (this << missed);
            m_maxMissedBeacons = missed;
        }

    void
        StaWifiMac::SetProbeRequestTimeout (Time timeout)
        {
            NS_LOG_FUNCTION (this << timeout);
            m_probeRequestTimeout = timeout;
        }

    void
        StaWifiMac::SetAssocRequestTimeout (Time timeout)
        {
            NS_LOG_FUNCTION (this << timeout);
            m_assocRequestTimeout = timeout;
        }

    void
        StaWifiMac::StartActiveAssociation (void)
        {
            NS_LOG_FUNCTION (this);
            TryToEnsureAssociated ();
        }

    void
        StaWifiMac::SetActiveProbing (bool enable)
        {
            NS_LOG_FUNCTION (this << enable);
            if (enable)
            {
                Simulator::ScheduleNow (&StaWifiMac::TryToEnsureAssociated, this);
            }
            else
            {
                m_probeRequestEvent.Cancel ();
            }
            m_activeProbing = enable;
        }

    bool StaWifiMac::GetActiveProbing (void) const
    {
        return m_activeProbing;
    }

    void
        StaWifiMac::SendProbeRequest (void)
        {
            NS_LOG_FUNCTION (this);
            WifiMacHeader hdr;
            hdr.SetProbeReq ();
            hdr.SetAddr1 (Mac48Address::GetBroadcast ());
            hdr.SetAddr2 (GetAddress ());
            hdr.SetAddr3 (Mac48Address::GetBroadcast ());
            hdr.SetDsNotFrom ();
            hdr.SetDsNotTo ();
            Ptr<Packet> packet = Create<Packet> ();
            MgtProbeRequestHeader probe;
            probe.SetSsid (GetSsid ());
            probe.SetSupportedRates (GetSupportedRates ());
            if (m_htSupported)
            {
                probe.SetHtCapabilities (GetHtCapabilities());
                hdr.SetNoOrder();
            }

            packet->AddHeader (probe);

            // The standard is not clear on the correct queue for management
            // frames if we are a QoS AP. The approach taken here is to always
            // use the DCF for these regardless of whether we have a QoS
            // association or not.
            m_dca->Queue (packet, hdr);

            if (m_probeRequestEvent.IsRunning ())
            {
                m_probeRequestEvent.Cancel ();
            }
            m_probeRequestEvent = Simulator::Schedule (m_probeRequestTimeout,
                    &StaWifiMac::ProbeRequestTimeout, this);
        }

    void
        StaWifiMac::SendAssociationRequest (void)
        {
            NS_LOG_FUNCTION (this << GetBssid ());
            WifiMacHeader hdr;
            hdr.SetAssocReq ();
            hdr.SetAddr1 (GetBssid ());
            hdr.SetAddr2 (GetAddress ());
            hdr.SetAddr3 (GetBssid ());
            hdr.SetDsNotFrom ();
            hdr.SetDsNotTo ();
            Ptr<Packet> packet = Create<Packet> ();
            MgtAssocRequestHeader assoc;
            assoc.SetSsid (GetSsid ());
            assoc.SetSupportedRates (GetSupportedRates ());
            if (m_htSupported)
            {
                assoc.SetHtCapabilities (GetHtCapabilities());
                hdr.SetNoOrder();
            }

            packet->AddHeader (assoc);

            // The standard is not clear on the correct queue for management
            // frames if we are a QoS AP. The approach taken here is to always
            // use the DCF for these regardless of whether we have a QoS
            // association or not.
            m_dca->Queue (packet, hdr);

            if (m_assocRequestEvent.IsRunning ())
            {
                m_assocRequestEvent.Cancel ();
            }
            m_assocRequestEvent = Simulator::Schedule (m_assocRequestTimeout,
                    &StaWifiMac::AssocRequestTimeout, this);
        }

    void
        StaWifiMac::TryToEnsureAssociated (void)
        {
            NS_LOG_FUNCTION (this);
            switch (m_state)
            {
                case ASSOCIATED:
                    return;
                    break;
                case WAIT_PROBE_RESP:
                    /* we have sent a probe request earlier so we
                       do not need to re-send a probe request immediately.
                       We just need to wait until probe-request-timeout
                       or until we get a probe response
                       */
                    break;
                case BEACON_MISSED:
                    /* we were associated but we missed a bunch of beacons
                     * so we should assume we are not associated anymore.
                     * We try to initiate a probe request now.
                     */
                    m_linkDown ();
                    if (m_activeProbing) 
                    {
                        SetState (WAIT_PROBE_RESP);
                        SendProbeRequest ();
                    }
                    break;
                case WAIT_ASSOC_RESP:
                    /* we have sent an assoc request so we do not need to
                       re-send an assoc request right now. We just need to
                       wait until either assoc-request-timeout or until
                       we get an assoc response.
                       */
                    break;
                case REFUSED:
                    /* we have sent an assoc request and received a negative
                       assoc resp. We wait until someone restarts an
                       association with a given ssid.
                       */
                    break;
                case SCANNING:
                    break;
            }
        }

    void
        StaWifiMac::AssocRequestTimeout (void)
        {
            NS_LOG_FUNCTION (this);
            SetState (WAIT_ASSOC_RESP);
            SendAssociationRequest ();
        }

    void
        StaWifiMac::ProbeRequestTimeout (void)
        {
            NS_LOG_FUNCTION (this);
            SetState (WAIT_PROBE_RESP);
            SendProbeRequest ();
        }

    void
        StaWifiMac::RunScanOrProbe (void)
        {
            NS_LOG_FUNCTION (this << GetBssid ());
            if (IsSupportScanning())
            {
                if (m_state != SCANNING)
                {
                    NS_LOG_DEBUG ("start scanning");
                    SetState (SCANNING);
                    ScanningStart ();
                }
            }
            else
            {
                NS_LOG_DEBUG ("beacon missed");
                SetState (BEACON_MISSED);
                TryToEnsureAssociated ();
            }
        }

    void
        StaWifiMac::MissedBeacons (void)
        {
            NS_LOG_FUNCTION (this);
            if (m_beaconWatchdogEnd > Simulator::Now ())
            {
                if (m_beaconWatchdog.IsRunning ())
                {
                    m_beaconWatchdog.Cancel ();
                }
                m_beaconWatchdog = Simulator::Schedule (m_beaconWatchdogEnd - Simulator::Now (),
                        &StaWifiMac::MissedBeacons, this);
                return;
            }
            // JWHUR enable handoff
            //            NS_LOG_DEBUG ("beacon missed");
            //            SetState (BEACON_MISSED);
            //            TryToEnsureAssociated ();
            RunScanOrProbe();
        }

    void
        StaWifiMac::RestartBeaconWatchdog (Time delay)
        {
            NS_LOG_FUNCTION (this << delay);
            m_beaconWatchdogEnd = std::max (Simulator::Now () + delay, m_beaconWatchdogEnd);
            if (Simulator::GetDelayLeft (m_beaconWatchdog) < delay
                    && m_beaconWatchdog.IsExpired ())
            {
                NS_LOG_DEBUG ("really restart watchdog.");
                m_beaconWatchdog = Simulator::Schedule (delay, &StaWifiMac::MissedBeacons, this);
            }
        }

    bool
        StaWifiMac::IsAssociated (void) const
        {
            return m_state == ASSOCIATED;
        }

    bool
        StaWifiMac::IsWaitAssocResp (void) const
        {
            return m_state == WAIT_ASSOC_RESP;
        }

    void
        StaWifiMac::Enqueue (Ptr<const Packet> packet, Mac48Address to)
        {
            NS_LOG_FUNCTION (this << packet << to);
            if (!IsAssociated ())
            {
                NotifyTxDrop (packet);
                // JWHUR enablge handoff
                //                TryToEnsureAssociated ();
                RunScanOrProbe();
                return;
            }
            WifiMacHeader hdr;

            // If we are not a QoS AP then we definitely want to use AC_BE to
            // transmit the packet. A TID of zero will map to AC_BE (through \c
            // QosUtilsMapTidToAc()), so we use that as our default here.
            uint8_t tid = 0;

            // For now, an AP that supports QoS does not support non-QoS
            // associations, and vice versa. In future the AP model should
            // support simultaneously associated QoS and non-QoS STAs, at which
            // point there will need to be per-association QoS state maintained
            // by the association state machine, and consulted here.
            if (m_qosSupported)
            {
                hdr.SetType (WIFI_MAC_QOSDATA);
                hdr.SetQosAckPolicy (WifiMacHeader::NORMAL_ACK);
                hdr.SetQosNoEosp ();
                hdr.SetQosNoAmsdu ();
                // Transmission of multiple frames in the same TXOP is not
                // supported for now
                hdr.SetQosTxopLimit (0);

                // Fill in the QoS control field in the MAC header
                tid = QosUtilsGetTidForPacket (packet);
                // Any value greater than 7 is invalid and likely indicates that
                // the packet had no QoS tag, so we revert to zero, which'll
                // mean that AC_BE is used.
                if (tid >= 7)
                {
                    tid = 0;
                }
                hdr.SetQosTid (tid);
            }
            else
            {
                hdr.SetTypeData ();
            }
            if (m_htSupported)
            {
                hdr.SetNoOrder();
            }

            hdr.SetAddr1 (GetBssid ());
            hdr.SetAddr2 (m_low->GetAddress ());
            hdr.SetAddr3 (to);
            hdr.SetDsNotFrom ();
            hdr.SetDsTo ();

            if (m_qosSupported)
            {
                // Sanity check that the TID is valid
                NS_ASSERT (tid < 8);
                m_edca[QosUtilsMapTidToAc (tid)]->Queue (packet, hdr);
            }
            else
            {
                m_dca->Queue (packet, hdr);
            }
        }

    void
        StaWifiMac::Receive (Ptr<Packet> packet, const WifiMacHeader *hdr)
        {
            NS_LOG_FUNCTION (this << packet << hdr);
            NS_ASSERT (!hdr->IsCtl ());
            if (hdr->GetAddr3 () == GetAddress ())
            {
                NS_LOG_LOGIC ("packet sent by us.");
                return;
            }
            else if (hdr->GetAddr1 () != GetAddress ()
                    && !hdr->GetAddr1 ().IsGroup ())
            {
                NS_LOG_LOGIC ("packet is not for us");
                NotifyRxDrop (packet);
                return;
            }
            else if (hdr->IsData ())
            {
                if (!IsAssociated ())
                {
                    NS_LOG_LOGIC ("Received data frame while not associated: ignore");
                    NotifyRxDrop (packet);
                    return;
                }
                if (!(hdr->IsFromDs () && !hdr->IsToDs ()))
                {
                    NS_LOG_LOGIC ("Received data frame not from the DS: ignore");
                    NotifyRxDrop (packet);
                    return;
                }
                if (hdr->GetAddr2 () != GetBssid ())
                {
                    NS_LOG_LOGIC ("Received data frame not from the BSS we are associated with: ignore");
                    NotifyRxDrop (packet);
                    return;
                }

                if (hdr->IsQosData ())
                {
                    if (hdr->IsQosAmsdu ())
                    {
                        NS_ASSERT (hdr->GetAddr3 () == GetBssid ());
                        DeaggregateAmsduAndForward (packet, hdr);
                        packet = 0;
                    }
                    else
                    {
                        ForwardUp (packet, hdr->GetAddr3 (), hdr->GetAddr1 ());
                    }
                }
                else
                {
                    ForwardUp (packet, hdr->GetAddr3 (), hdr->GetAddr1 ());
                }
                return;
            }
            else if (hdr->IsProbeReq ()
                    || hdr->IsAssocReq ())
            {
                // This is a frame aimed at an AP, so we can safely ignore it.
                NotifyRxDrop (packet);
                return;
            }
            else if (hdr->IsBeacon ())
            {
                MgtBeaconHeader beacon;
                packet->RemoveHeader (beacon);
                bool goodBeacon = false;
                if (GetSsid ().IsBroadcast ()
                        || beacon.GetSsid ().IsEqual (GetSsid ()))
                {
                    goodBeacon = true;
                }
                SupportedRates rates = beacon.GetSupportedRates ();
                for (uint32_t i = 0; i < m_phy->GetNBssMembershipSelectors (); i++)
                {
                    uint32_t selector = m_phy->GetBssMembershipSelector (i);
                    if (!rates.IsSupportedRate (selector))
                    {
                        goodBeacon = false;
                    }
                }
                if ((IsWaitAssocResp () || IsAssociated ()) && hdr->GetAddr3 () != GetBssid ())
                {
                    goodBeacon = false;
                }
                if (goodBeacon)
                {
                    Time delay = MicroSeconds (beacon.GetBeaconIntervalUs () * m_maxMissedBeacons);
                    RestartBeaconWatchdog (delay);
                    SetBssid (hdr->GetAddr3 ());
                }
                if (goodBeacon && m_state == BEACON_MISSED)
                {
                    SetState (WAIT_ASSOC_RESP);
                    SendAssociationRequest ();
                }
                return;
            }
            else if (hdr->IsProbeResp ())
            {
                if (m_state == WAIT_PROBE_RESP)
                {
                    MgtProbeResponseHeader probeResp;
                    packet->RemoveHeader (probeResp);
                    if (!probeResp.GetSsid ().IsEqual (GetSsid ()))
                    {
                        //not a probe resp for our ssid.
                        return;
                    }
                    SupportedRates rates = probeResp.GetSupportedRates ();
                    for (uint32_t i = 0; i < m_phy->GetNBssMembershipSelectors (); i++)
                    {
                        uint32_t selector = m_phy->GetBssMembershipSelector (i);
                        if (!rates.IsSupportedRate (selector))
                        {
                            return;
                        }
                    }
                    SetBssid (hdr->GetAddr3 ());
                    Time delay = MicroSeconds (probeResp.GetBeaconIntervalUs () * m_maxMissedBeacons);
                    RestartBeaconWatchdog (delay);
                    if (m_probeRequestEvent.IsRunning ())
                    {
                        m_probeRequestEvent.Cancel ();
                    }
                    SetState (WAIT_ASSOC_RESP);
                    SendAssociationRequest ();
                }
                return;
            }
            else if (hdr->IsAssocResp ())
            {
                if (m_state == WAIT_ASSOC_RESP)
                {
                    MgtAssocResponseHeader assocResp;
                    packet->RemoveHeader (assocResp);
                    if (m_assocRequestEvent.IsRunning ())
                    {
                        m_assocRequestEvent.Cancel ();
                    }
                    if (assocResp.GetStatusCode ().IsSuccess ())
                    {
                        SetState (ASSOCIATED);
                        NS_LOG_DEBUG ("assoc completed");
                        SupportedRates rates = assocResp.GetSupportedRates ();
                        if (m_htSupported)
                        {
                            HtCapabilities htcapabilities = assocResp.GetHtCapabilities ();		  
                            m_stationManager->AddStationHtCapabilities (hdr->GetAddr2 (),htcapabilities);
                        }

                        for (uint32_t i = 0; i < m_phy->GetNModes (); i++)
                        {
                            WifiMode mode = m_phy->GetMode (i);
                            if (rates.IsSupportedRate (mode.GetDataRate ()))
                            {
                                m_stationManager->AddSupportedMode (hdr->GetAddr2 (), mode);
                                if (rates.IsBasicRate (mode.GetDataRate ()))
                                {
                                    m_stationManager->AddBasicMode (mode);
                                }
                            }
                        }
                        if(m_htSupported)
                        {
                            HtCapabilities htcapabilities = assocResp.GetHtCapabilities ();
                            for (uint32_t i = 0; i < m_phy->GetNMcs(); i++)
                            {
                                uint8_t mcs=m_phy->GetMcs(i);
                                if (htcapabilities.IsSupportedMcs (mcs))
                                {
                                    m_stationManager->AddSupportedMcs (hdr->GetAddr2 (), mcs);
                                    //here should add a control to add basic MCS when it is implemented
                                }
                            }
                        }
                        if (!m_linkUp.IsNull ())
                        {
                            m_linkUp ();
                        }

                        //JWHUR enable handoff
                        char buf[] = "E\0ABCDEFGHIJKLMNOPQRSTUVXYZabcdefghijklmnopqrstuvxyz";
                        int64_t time_us = Simulator::Now().GetMicroSeconds();
                        memcpy(buf + 20, &time_us, sizeof(int64_t));
                        Ptr<Packet> packet = Create<Packet> ((const uint8_t*)buf, sizeof(buf));
                        Mac48Address broadcast = Mac48Address ("ff:ff:ff:ff:ff:ff");
                        LlcSnapHeader llc;

                        llc.SetType (0x800);
                        packet->AddHeader (llc);

                        Enqueue (packet, broadcast);
                    }
                    else
                    {
                        NS_LOG_DEBUG ("assoc refused");
                        SetState (REFUSED);
                    }
                }
                return;
            }

            // Invoke the receive handler of our parent class to deal with any
            // other frames. Specifically, this will handle Block Ack-related
            // Management Action frames.
            RegularWifiMac::Receive (packet, hdr);
        }

    SupportedRates
        StaWifiMac::GetSupportedRates (void) const
        {
            SupportedRates rates;
            if(m_htSupported)
            {
                for (uint32_t i = 0; i < m_phy->GetNBssMembershipSelectors(); i++)
                {
                    rates.SetBasicRate(m_phy->GetBssMembershipSelector(i));
                }
            }
            for (uint32_t i = 0; i < m_phy->GetNModes (); i++)
            {
                WifiMode mode = m_phy->GetMode (i);
                rates.AddSupportedRate (mode.GetDataRate ());
            }
            return rates;
        }
    HtCapabilities
        StaWifiMac::GetHtCapabilities (void) const
        {
            HtCapabilities capabilities;
            capabilities.SetHtSupported(1);
            capabilities.SetLdpc (m_phy->GetLdpc());
            capabilities.SetShortGuardInterval20 (m_phy->GetGuardInterval());
            capabilities.SetGreenfield (m_phy->GetGreenfield());
            for (uint8_t i =0 ; i < m_phy->GetNMcs();i++)
            {
                capabilities.SetRxMcsBitmask(m_phy->GetMcs(i));
            }
            capabilities.SetNRxAntenna(m_phy->GetNumberOfReceiveAntennas()); //11ac: multiple multiple_stream_tx_nss
            capabilities.SetOperationalBandwidth (m_phy->GetOperationalBandwidth()); //802.11ac channel bonding
            return capabilities;
        }
    void
        StaWifiMac::SetState (MacState value)
        {
            if (value == ASSOCIATED
                    && m_state != ASSOCIATED)
            {
                m_assocLogger (GetBssid ());
            }
            else if (value != ASSOCIATED
                    && m_state == ASSOCIATED)
            {
                m_deAssocLogger (GetBssid ());
            }
            m_state = value;
        }

    void
        StaWifiMac::SetupStaMacListener (Ptr<WifiPhy> phy)
        {
            m_phyStaMacListener = new PhyStaMacListener (this);
            phy->RegisterListener (m_phyStaMacListener);
        }

    bool
        StaWifiMac::IsSupportScanning (void) const
        {
            return m_scanType != NOTSUPPORT;
        }

    void
        StaWifiMac::NotifySwitchingStartNow (Time duration)
        {
            Simulator::Schedule (duration, 
                    &StaWifiMac::ScanningSwitchChannelEnd, this);
        }

    void
        StaWifiMac::NotifyCcaBusyOccurred ()
        {
            m_bCcaBusyOccurred = true;
        }

    void
        StaWifiMac::ScanningStart (void)
        {
            NS_LOG_FUNCTION (this);
            m_probeRequestEvent.Cancel ();
            m_beaconWatchdog.Cancel ();
            m_scanChannelNumber = 0;
            m_scanResults.clear ();
            m_low->EnableForwardSnr (true);
            m_bestAP = NULL;
            Simulator::ScheduleNow (&StaWifiMac::ScanningSwitchChannelStart, this);
        }

    void
        StaWifiMac::ScanningEnd (void)
        {
            NS_LOG_FUNCTION (this);
            m_low->EnableForwardSnr (false);

            SetState (BEACON_MISSED);

            uint32_t size = m_scanResults.size ();
            if (size == 0)
            {
                NS_LOG_LOGIC ("cant scan for any ap.");
                RunScanOrProbe ();
            }
            else
            {
                NS_LOG_DEBUG ("scan result: number of aps is " << m_scanResults.size ());
                m_bestAP = &m_scanResults[0];

                for (uint32_t i = 1; i < size; i++)
                {
                    if (m_bestAP->rxSnr < m_scanResults[i].rxSnr)
                    {
                        m_bestAP = &m_scanResults[i];
                    }
                }
                NS_LOG_DEBUG ("bestAP: " << m_bestAP->channelNumber << " " << m_bestAP->ssid << " " << m_bestAP->bssid);
                m_phy->SetChannelNumber (m_bestAP->channelNumber);
            }
        }

    void
        StaWifiMac::ScanningSwitchChannelStart(void)
        {
            if (m_probeRequestEvent.IsRunning ())
            {
                m_probeRequestEvent.Cancel ();
            }

            m_scanChannelNumber++;
            NS_LOG_DEBUG ("switch to channel number:" << m_scanChannelNumber);
            if(m_scanChannelNumber > m_maxChannelNumber)
            {
                ScanningEnd();
            }
            else
            {
                m_phy->SetChannelNumber (m_scanChannelNumber);
            }
        }

    void
        StaWifiMac::ScanningSwitchChannelEnd(void)
        {
            if (m_bestAP == NULL)
            {
                m_bCcaBusyOccurred = m_phy->IsStateCcaBusy ();

                if (m_scanType == ACTIVE)
                {
                    SetSsid(Ssid());
                    SendProbeRequest();
                    m_scanChannelEvent = Simulator::Schedule (m_minChannelTime,
                            &StaWifiMac::ScanningMinChannelTimeout, this);
                }
                else if (m_scanType == PASSIVE)
                {
                    m_scanChannelEvent = Simulator::Schedule (m_maxChannelTime,
                            &StaWifiMac::ScanningSwitchChannelStart, this);
                }
            }
            else
            {
                SetSsid (m_bestAP->ssid);
                SetState (WAIT_PROBE_RESP);
                SendProbeRequest();
            }
        }

    void
        StaWifiMac::ScanningMinChannelTimeout(void)
        {
            if (m_bCcaBusyOccurred && m_maxChannelTime > m_minChannelTime)
            {
                m_scanChannelEvent = Simulator::Schedule (m_maxChannelTime - m_minChannelTime,
                        &StaWifiMac::ScanningSwitchChannelStart, this);
            }
            else
            {
                Simulator::ScheduleNow (&StaWifiMac::ScanningSwitchChannelStart, this);
            }
        }

    void
        StaWifiMac::SnrReceive (Ptr<Packet> packet, const WifiMacHeader *hdr, double rxSnr)
        {
            NS_LOG_FUNCTION (this << packet << hdr << rxSnr << GetAddress () << hdr->GetAddr3 () << hdr->GetAddr1 ());
            NS_ASSERT (!hdr->IsCtl ());

            ScanningEntry entry;
            if (hdr->GetAddr3 () == GetAddress ())
            {
                NS_LOG_LOGIC ("packet sent by us.");
                return;
            }
            else if (hdr->GetAddr1 () != GetAddress ())
            {
                NS_LOG_LOGIC ("packet is not for us");
                NotifyRxDrop (packet);
                return;
            }
            else if (hdr->IsBeacon ())
            {
                MgtBeaconHeader beacon;
                packet->RemoveHeader (beacon);

                entry.ssid = beacon.GetSsid ();

            }
            else if (hdr->IsProbeResp ())
            {
                MgtProbeResponseHeader probeResp;
                packet->RemoveHeader (probeResp);

                entry.ssid = probeResp.GetSsid ();
            }
            else
            {
                NotifyRxDrop (packet);
                return;
            }

            entry.channelNumber = m_scanChannelNumber;
            entry.bssid = hdr->GetAddr3 ();
            entry.rxSnr = rxSnr;
            m_scanResults.push_back (entry);
        }

} // namespace ns3
