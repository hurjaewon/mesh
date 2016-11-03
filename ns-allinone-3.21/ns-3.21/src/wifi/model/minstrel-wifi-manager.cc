/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2009 Duy Nguyen
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
 * Author: Duy Nguyen <duy@soe.ucsc.edu>
 *
 * Some Comments:
 *
 * 1) Segment Size is declared for completeness but not used  because it has
 *    to do more with the requirement of the specific hardware.
 *
 * 2) By default, Minstrel applies the multi-rate retry(the core of Minstrel
 *    algorithm). Otherwise, please use ConstantRateWifiManager instead.
 *
 * http://linuxwireless.org/en/developers/Documentation/mac80211/RateControl/minstrel
 */

#include "minstrel-wifi-manager.h"
#include "wifi-phy.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include "ns3/uinteger.h"
#include "ns3/double.h"
#include "ns3/wifi-mac.h"
#include "ns3/assert.h"
#include <vector>

#define Min(a,b) ((a < b) ? a : b)

NS_LOG_COMPONENT_DEFINE ("MinstrelWifiManager");


namespace ns3 { 

/**
 * \brief hold per-remote-station state for Minstrel Wifi manager.
 *
 * This struct extends from WifiRemoteStation struct to hold additional
 * information required by the Minstrel Wifi manager
 */
struct MinstrelWifiRemoteStation : public WifiRemoteStation
{
  Time m_nextStatsUpdate;  ///< 10 times every second

  /**
   * To keep track of the current position in the our random sample table
   * going row by row from 1st column until the 10th column(Minstrel defines 10)
   * then we wrap back to the row 1 col 1.
   * note: there are many other ways to do this.
   */
  uint32_t m_waited;
  uint32_t cur_mpdu;
  uint32_t sum_mpdu;
  uint32_t sum_packet;
  uint32_t avg_ampduLen;
  uint32_t m_col[3], m_index[3];	// kjyoon
  uint32_t m_ngroup;				// number of supported groups	kjyoon
  uint32_t m_cs_group;				// current sampling group id	kjyoon
  uint32_t m_maxTpRate;  ///< the current throughput rate
  uint32_t m_group_maxTpRate;  // group id of maxTpRate		kjyoon
  uint32_t m_maxTpRate2;  ///< second highest throughput rate
  uint32_t m_group_maxTpRate2;  // group id of maxTpRate2	kjyoon
  uint32_t m_maxProbRate;  ///< rate with highest prob of success
  uint32_t m_group_maxProbRate;  // group id of maxProbRate	kjyoon

  uint32_t m_local_maxTpRate[3];		// rate with maximum throughput of each groups	kjyoon
  uint32_t m_local_maxTpRate2[3];		// rate with second highest throughput of each groups	kjyoon
  //	uint32_t m_local_maxProbRate[3];		// rate with maximum success probability of each groups	kjyoon

  int m_packetCount;  ///< total number of packets as of now
  int m_sampleCount;  ///< how many packets we have sample so far

  bool m_isSampling;  ///< a flag to indicate we are currently sampling
  uint32_t m_sampleRate;  ///< current sample rate
  uint32_t m_currentRate;  ///< current rate we are using
  uint32_t m_group_currentRate;  // kjyoon

  uint32_t m_shortRetry;  ///< short retries such as control packts
  uint32_t m_longRetry;  ///< long retries such as data packets
  uint32_t m_retry;  ///< total retries short + long
  uint32_t m_err;  ///< retry errors
  uint32_t m_txrate;  ///< current transmit rate
  uint32_t m_group_txrate;  // group id of txrate		kjyoon

  uint32_t m_sampleWait;	// kjyoon

  bool m_initialized;  ///< for initializing tables

  MinstrelRate m_minstrelTable;  ///< minstrel table
  MinstrelRate m_minstrelTable_Allgroup[3];	// kjyoon
  SampleRate m_sampleTable;  ///< sample table
  SampleRate m_sampleTable_Allgroup[3];  // kjyoon

  bool m_updatedByBlockAck;		// GotBlockAck function in edca-txop-n.cc kjyoon
};

NS_OBJECT_ENSURE_REGISTERED (MinstrelWifiManager);

  TypeId
MinstrelWifiManager::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::MinstrelWifiManager")
    .SetParent<WifiRemoteStationManager> ()
    .AddConstructor<MinstrelWifiManager> ()
    .AddAttribute ("UpdateStatistics",
        "The interval between updating statistics table ",
        TimeValue (Seconds (0.05)),
        MakeTimeAccessor (&MinstrelWifiManager::m_updateStats),
        MakeTimeChecker ())
    .AddAttribute ("LookAroundRate",
        "the percentage to try other rates",
        DoubleValue (10),
        MakeDoubleAccessor (&MinstrelWifiManager::m_lookAroundRate),
        MakeDoubleChecker<double> ())
    .AddAttribute ("EWMA",
        "EWMA level",
        DoubleValue (75),
        MakeDoubleAccessor (&MinstrelWifiManager::m_ewmaLevel),
        MakeDoubleChecker<double> ())
    .AddAttribute ("SampleColumn",
        "The number of columns used for sampling",
        DoubleValue (10),
        MakeDoubleAccessor (&MinstrelWifiManager::m_sampleCol),
        MakeDoubleChecker <double> ())
    .AddAttribute ("PacketLength",
        "The packet length used for calculating mode TxTime",
        DoubleValue (1000),
        MakeDoubleAccessor (&MinstrelWifiManager::m_pktLen),
        MakeDoubleChecker <double> ())
    ;
  return tid;
}

MinstrelWifiManager::MinstrelWifiManager ()
{
  m_uniformRandomVariable = CreateObject<UniformRandomVariable> ();

  m_nsupported = 0;
  //m_updatedByBlockAck = false;
}

MinstrelWifiManager::~MinstrelWifiManager ()
{
}

  void
MinstrelWifiManager::SetupPhy (Ptr<WifiPhy> phy)
{
  if (HasHtSupported())
  {	
    if (HasVhtSupported()) //11ac: vht_standard
    {
      uint32_t nModes = phy->GetNMcs();
      for (uint32_t i = 0; i < nModes; i++)
      {
        WifiMode mode = phy->AcMcsToWifiMode(phy->GetMcs (i), 20);
        WifiTxVector txVector;
        txVector.SetMode(mode);
        txVector.SetNss(1);
        AddCalcTxTime (mode, phy->CalculateTxDuration (m_pktLen, txVector, WIFI_PREAMBLE_HT_MF));

        mode = phy->AcMcsToWifiMode(phy->GetMcs (i), 40);
        txVector.SetMode(mode);
        AddCalcTxTime (mode, phy->CalculateTxDuration (m_pktLen, txVector, WIFI_PREAMBLE_HT_MF));
        mode = phy->AcMcsToWifiMode(phy->GetMcs (i), 80);
        txVector.SetMode(mode);
        AddCalcTxTime (mode, phy->CalculateTxDuration (m_pktLen, txVector, WIFI_PREAMBLE_HT_MF));
      }
    }
    else	
    {
      uint32_t nModes = phy->GetNMcs();
      for (uint32_t i = 0; i < nModes; i++)
      {
        WifiMode mode = phy->McsToWifiMode(phy->GetMcs (i));
        WifiTxVector txVector;
        txVector.SetMode(mode);
        txVector.SetNss(1);
        AddCalcTxTime (mode, phy->CalculateTxDuration (m_pktLen, txVector, WIFI_PREAMBLE_HT_MF));
      }
    }
  }
  else
  {
    uint32_t nModes = phy->GetNModes ();
    for (uint32_t i = 0; i < nModes; i++)
    {
      WifiMode mode = phy->GetMode (i);
      WifiTxVector txVector;
      txVector.SetMode(mode);
      AddCalcTxTime (mode, phy->CalculateTxDuration (m_pktLen, txVector, WIFI_PREAMBLE_LONG));
    }
  }
  WifiRemoteStationManager::SetupPhy (phy);
}

  int64_t
MinstrelWifiManager::AssignStreams (int64_t stream)
{
  NS_LOG_FUNCTION (this << stream);
  m_uniformRandomVariable->SetStream (stream);
  return 1;
}

Time
MinstrelWifiManager::GetCalcTxTime (WifiMode mode) const
{
  for (TxTime::const_iterator i = m_calcTxTime.begin (); i != m_calcTxTime.end (); i++)
  {
    if (mode == i->second)
    {
      return i->first;
    }
  }
  NS_ASSERT (false);
  return Seconds (0);
}

  void
MinstrelWifiManager::AddCalcTxTime (WifiMode mode, Time t)
{
  m_calcTxTime.push_back (std::make_pair (t, mode));
}

WifiRemoteStation *
MinstrelWifiManager::DoCreateStation (void) const
{
  MinstrelWifiRemoteStation *station = new MinstrelWifiRemoteStation ();

  station->m_nextStatsUpdate = Simulator::Now () + m_updateStats;
  station->m_waited = 0;	// kjyoon
  station->cur_mpdu = 0;	// kjyoon
  station->sum_mpdu = 0;	// kjyoon
  station->sum_packet = 0;
  station->avg_ampduLen= 0;
  station->m_col[0] = 0;	// kjyoon
  station->m_col[1] = 0;
  station->m_col[2] = 0;
  station->m_index[0] = 0;
  station->m_index[1] = 0;
  station->m_index[2] = 0;
  station->m_ngroup = 1;
  station->m_cs_group = 0;	// end kjyoon
  station->m_maxTpRate = 0;
  station->m_group_maxTpRate = 0;	// kjyoon
  station->m_maxTpRate2 = 0;
  station->m_maxProbRate = 0;
  station->m_packetCount = 0;
  station->m_sampleCount = 0;
  station->m_isSampling = false;
  station->m_sampleRate = 0;
  station->m_currentRate = 0;
  station->m_group_currentRate = 0;	// kjyoon
  station->m_shortRetry = 0;
  station->m_longRetry = 0;
  station->m_retry = 0;
  station->m_err = 0;
  station->m_txrate = 0;
  station->m_group_txrate = 0;		// kjyoon
  station->m_initialized = false;
  station->m_sampleWait = 16;		// kjyoon

  station->m_updatedByBlockAck = false;	// kjyoon
  station->m_local_maxTpRate[0] = 0;		
  station->m_local_maxTpRate[1] = 0;		
  station->m_local_maxTpRate[2] = 0;		
  station->m_local_maxTpRate2[0] = 0;		
  station->m_local_maxTpRate2[1] = 0;		
  station->m_local_maxTpRate2[2] = 0;		
  return station;
}

  void
MinstrelWifiManager::CheckInit (MinstrelWifiRemoteStation *station)
{
  //11ac: multiple multiple_stream_tx_ra
  if ((!HasHtSupported() &&!station->m_initialized && GetNSupported (station) > 1) || (HasHtSupported() &&!station->m_initialized && GetNMcsSupported (station) > 1))
  {
    // Note: we appear to be doing late initialization of the table
    // to make sure that the set of supported rates has been initialized
    // before we perform our own initialization.
    station->m_ngroup = Min (GetNumberOfReceiveAntennas (station),GetNumberOfTransmitAntennas());

    if (HasHtSupported())
      m_nsupported = GetNMcsSupported (station);
    else
      m_nsupported = GetNSupported (station);

    station->m_minstrelTable = MinstrelRate (m_nsupported);
    station->m_sampleTable = SampleRate (m_nsupported, std::vector<uint32_t> (m_sampleCol));
    for (uint32_t i_gr = 0; i_gr < station->m_ngroup; i_gr++) {
      station->m_minstrelTable_Allgroup [i_gr] = station->m_minstrelTable;	// kjyoon
      station->m_sampleTable_Allgroup [i_gr] = station->m_sampleTable;	// kjyoon
    }
    InitSampleTable (station);
    //	  PrintSampleTable (station);	// kjyoon
    RateInit (station);
    station->m_initialized = true;
  }
}

  void
MinstrelWifiManager::DoReportRxOk (WifiRemoteStation *st,
    double rxSnr, WifiMode txMode)
{
  MinstrelWifiRemoteStation *station = (MinstrelWifiRemoteStation *)st;
  NS_LOG_DEBUG ("DoReportRxOk m_txrate=" << station->m_txrate);
}

  void
MinstrelWifiManager::DoReportRtsFailed (WifiRemoteStation *st)
{
  MinstrelWifiRemoteStation *station = (MinstrelWifiRemoteStation *)st;
  NS_LOG_DEBUG ("DoReportRtsFailed m_txrate=" << station->m_txrate);

 // station->m_shortRetry++;//160415 skim11 : disable this (its wrong)
}

  void
MinstrelWifiManager::DoReportRtsOk (WifiRemoteStation *st, double ctsSnr, WifiMode ctsMode, double rtsSnr)
{
  NS_LOG_DEBUG ("self=" << st << " rts ok");
}

  void
MinstrelWifiManager::DoReportFinalRtsFailed (WifiRemoteStation *st)
{
  MinstrelWifiRemoteStation *station = (MinstrelWifiRemoteStation *)st;
  UpdateRetry (station);
  station->m_err++;
}

  void
MinstrelWifiManager::DoReportDataFailed (WifiRemoteStation *st)
{
  MinstrelWifiRemoteStation *station = (MinstrelWifiRemoteStation *)st;
  /**
   *
   * Retry Chain table is implemented here
   *
   * Try |         LOOKAROUND RATE              | NORMAL RATE
   *     | random < best    | random > best     |
   * --------------------------------------------------------------
   *  1  | Best throughput  | Random rate       | Best throughput
   *  2  | Random rate      | Best throughput   | Next best throughput
   *  3  | Best probability | Best probability  | Best probability
   *  4  | Lowest Baserate  | Lowest baserate   | Lowest baserate
   *
   * Note: For clarity, multiple blocks of if's and else's are used
   * After a failing 7 times, DoReportFinalDataFailed will be called
   */

  CheckInit (station);
  if (!station->m_initialized)
  {
    return;
  }
  

  station->m_longRetry++;

  NS_LOG_DEBUG ("DoReportDataFailed " << station << "\t rate " << station->m_txrate << "\tlongRetry \t" << station->m_longRetry);

  uint32_t org_group = station->m_group_txrate;	// kjyoon
  MinstrelRate tmp_minstrelTable;	// kjyoon
  tmp_minstrelTable = station->m_minstrelTable_Allgroup[station->m_group_txrate];		// kjyoon
  tmp_minstrelTable[station->m_txrate].numRateAttempt++;
  /// for normal rate, we're not currently sampling random rates
  if (!station->m_isSampling)
  {
    /// use best throughput rate
    if (station->m_longRetry < tmp_minstrelTable[station->m_txrate].adjustedRetryCount)
    {
      ;  ///<  there's still a few retries left
    }

    /// use second best throughput rate
    else if (station->m_longRetry <= (tmp_minstrelTable[station->m_txrate].adjustedRetryCount +
          tmp_minstrelTable[station->m_maxTpRate].adjustedRetryCount))
    {
      station->m_txrate = station->m_maxTpRate2;
      station->m_group_txrate = station->m_group_maxTpRate2;	// kjyoon
      NS_LOG_DEBUG("m_group_txrate update to maxTpRate2: " << station->m_group_txrate);	// kjyoon
    }

    /// use best probability rate
    else if (station->m_longRetry <= (tmp_minstrelTable[station->m_txrate].adjustedRetryCount +
          tmp_minstrelTable[station->m_maxTpRate2].adjustedRetryCount +
          tmp_minstrelTable[station->m_maxTpRate].adjustedRetryCount))
    {
      station->m_txrate = station->m_maxProbRate;
      station->m_group_txrate = station->m_group_maxProbRate;	// kjyoon
      NS_LOG_DEBUG("m_group_txrate update to maxProbRate: " << station->m_group_txrate);	// kjyoon
    }

    /// use lowest base rate
    else if (station->m_longRetry > (tmp_minstrelTable[station->m_txrate].adjustedRetryCount +
          tmp_minstrelTable[station->m_maxTpRate2].adjustedRetryCount +
          tmp_minstrelTable[station->m_maxTpRate].adjustedRetryCount))
    {
      station->m_txrate = 0;
      station->m_group_txrate = 0;		// kjyoon
      NS_LOG_DEBUG("m_group_txrate update to 0: " << station->m_group_txrate);	// kjyoon
    }
  }

  /// for look-around rate, we're currently sampling random rates
  else
  {
    /// current sampling rate is slower than the current best rate
    if (station->m_longRetry < tmp_minstrelTable[station->m_txrate].adjustedRetryCount)
    {
      ;    ///< keep using it
    }

    /// use the best rate
    else if (station->m_longRetry <= (tmp_minstrelTable[station->m_txrate].adjustedRetryCount +
          tmp_minstrelTable[station->m_sampleRate].adjustedRetryCount))
    {
      station->m_txrate = station->m_maxTpRate;
      station->m_group_txrate = station->m_group_maxTpRate;		// kjyoon
      NS_LOG_DEBUG("USE: txrate(group_txrate) of maxTpRate: " << station->m_txrate << "(" << station->m_group_txrate << ")");	// kjyoon
    }

    /// use the best probability rate
    else if (station->m_longRetry <= (tmp_minstrelTable[station->m_txrate].adjustedRetryCount +
          tmp_minstrelTable[station->m_maxTpRate].adjustedRetryCount +
          tmp_minstrelTable[station->m_sampleRate].adjustedRetryCount))
    {
      station->m_txrate = station->m_maxProbRate;
      station->m_group_txrate = station->m_group_maxProbRate;	// kjyoon
      NS_LOG_DEBUG("m_group_txrate update to maxProbRate: " << station->m_group_txrate);	// kjyoon
    }

    /// use the lowest base rate
    else if (station->m_longRetry > (tmp_minstrelTable[station->m_txrate].adjustedRetryCount +
          tmp_minstrelTable[station->m_maxTpRate].adjustedRetryCount +
          tmp_minstrelTable[station->m_sampleRate].adjustedRetryCount))
    {
      station->m_txrate = 0;
      station->m_group_txrate = 0;	// kjyoon
      NS_LOG_DEBUG("m_group_txrate update to 0: " << station->m_group_txrate);	// kjyoon
    }
  }
  station->m_minstrelTable_Allgroup[org_group] = tmp_minstrelTable;		// kjyoon
}
/* 150702 kjyoon
 * [Minstrel HT] Downgrade one group when attempt > 30 && success prob < 0.2
 */
void
MinstrelWifiManager::DowngradeRate (WifiRemoteStation *st)
{
  //	  return false;
  MinstrelWifiRemoteStation *station = (MinstrelWifiRemoteStation *) st;
  CheckInit (station);
  if (!station->m_initialized)
  {
    return;
  }
  MinstrelRate tmp_minstrelTable;
  tmp_minstrelTable = station->m_minstrelTable_Allgroup[station->m_group_maxTpRate];
  uint32_t tmp_success = tmp_minstrelTable[station->m_maxTpRate].numRateSuccess;
  uint32_t tmp_attempt = tmp_minstrelTable[station->m_maxTpRate].numRateAttempt;
  NS_LOG_DEBUG ("maxTp m_isSampling=" << station->m_isSampling);
  if ((!station->m_isSampling)&&(tmp_attempt > 30)&&(tmp_success*5 < tmp_attempt)) {
    if (station->m_group_maxTpRate > 0)
    {
      NS_LOG_DEBUG ("maxTp From: " << station->m_maxTpRate << "(" << station->m_group_maxTpRate << ")"); 
      station->m_group_maxTpRate--;
      station->m_maxTpRate = station->m_local_maxTpRate[station->m_group_maxTpRate];
      NS_LOG_DEBUG ("maxTp To: " << station->m_maxTpRate << "(" << station->m_group_maxTpRate << ")"); 
    }
    else 
      NS_LOG_DEBUG ("maxTp Already lowest group");
  }
  
  tmp_minstrelTable = station->m_minstrelTable_Allgroup[station->m_group_maxTpRate2];
  tmp_success = tmp_minstrelTable[station->m_maxTpRate2].numRateSuccess;
  tmp_attempt = tmp_minstrelTable[station->m_maxTpRate2].numRateAttempt;
  if ((!station->m_isSampling)&&(tmp_attempt > 30)&&(tmp_success*5 < tmp_attempt)) {
    if (station->m_group_maxTpRate2 > 0)
    {
      NS_LOG_DEBUG ("maxTp2 From: " << station->m_maxTpRate2 << "(" << station->m_group_maxTpRate2 << ")"); 
      station->m_group_maxTpRate2--;
      station->m_maxTpRate2 = station->m_local_maxTpRate2[station->m_group_maxTpRate2];
      NS_LOG_DEBUG ("maxTp2 To: " << station->m_maxTpRate2 << "(" << station->m_group_maxTpRate2 << ")"); 
    }
    else 
      NS_LOG_DEBUG ("maxTp2 already lowest group");
  }
}

/** 150625 added by kjyoon
 * Disable A-MPDU for sampling packet
 */
  bool
MinstrelWifiManager::DoIsSampling(WifiRemoteStation *st)
{
  MinstrelWifiRemoteStation *station = (MinstrelWifiRemoteStation *) st;
  return station->m_isSampling;
}
  void
MinstrelWifiManager::DoSetIsSampling(WifiRemoteStation *st, bool result)
{
  MinstrelWifiRemoteStation *station = (MinstrelWifiRemoteStation *) st;
  station->m_isSampling = result;
  return;
}

  void
MinstrelWifiManager::DoUpdateSumAmpdu(WifiRemoteStation *st, uint32_t mpdus)
{
  MinstrelWifiRemoteStation *station = (MinstrelWifiRemoteStation *) st;

  station->cur_mpdu = mpdus;
  if(!station->m_isSampling)
  {
    station->sum_mpdu += mpdus;
    station->sum_packet++;
  }
  //	  NS_LOG_DEBUG ("sum_mpdu: " << station->sum_mpdu << ", sum_packet: " << station->sum_packet);	// kjyoon
}

/**
 * 150623 added by kjyoon
 *
 * Update the number of success and attempt in one A-MPDU
 **/
  void 
MinstrelWifiManager::DoUpdateMinstrelTable(WifiRemoteStation *st, uint32_t success, uint32_t attempt)
{
  NS_LOG_DEBUG ("DoUpdateMinstrelTable: success=" << success << ", attempt=" << attempt);
  MinstrelWifiRemoteStation *station = (MinstrelWifiRemoteStation *) st;

  CheckInit (station);
  if (!station->m_initialized)
  {
    return;
  }
  uint32_t org_group = station->m_group_txrate;	// kjyoon
  MinstrelRate tmp_minstrelTable;	// kjyoon
  tmp_minstrelTable = station->m_minstrelTable_Allgroup[station->m_group_txrate];		// kjyoon
  tmp_minstrelTable[station->m_txrate].numRateSuccess += success;
  tmp_minstrelTable[station->m_txrate].numRateAttempt += attempt;
  NS_LOG_DEBUG ("m_minstrelTable_Allgroup[" << station->m_group_txrate  << "][" << station->m_txrate << "]: Success = " << tmp_minstrelTable[station->m_txrate].numRateSuccess << ", attempt = " << tmp_minstrelTable[station->m_txrate].numRateAttempt);		// kjyoon

  station->m_minstrelTable_Allgroup[org_group] = tmp_minstrelTable;		// kjyoon
  station->m_packetCount+=attempt;		//  += attempt?
  station->m_updatedByBlockAck = true;	// kjyoon

  
  if (m_nsupported >= 1)
  {
    station->m_txrate = FindRate (station);
  }

}

  void
MinstrelWifiManager::DoReportDataOk (WifiRemoteStation *st,
    double ackSnr, WifiMode ackMode, double dataSnr)
{
  MinstrelWifiRemoteStation *station = (MinstrelWifiRemoteStation *) st;

  if (station->m_updatedByBlockAck)	// kjyoon
  {
    station->m_updatedByBlockAck = false;
    return;
  }

  CheckInit (station);
  if (!station->m_initialized)
  {
    return;
  }

  uint32_t org_group = station->m_group_txrate;	// kjyoon
  MinstrelRate tmp_minstrelTable;	// kjyoon
  tmp_minstrelTable = station->m_minstrelTable_Allgroup[station->m_group_txrate];		// kjyoon
  tmp_minstrelTable[station->m_txrate].numRateSuccess++;
  tmp_minstrelTable[station->m_txrate].numRateAttempt++;

  UpdateRetry (station);

  //tmp_minstrelTable[station->m_txrate].numRateAttempt += station->m_retry;
  NS_LOG_DEBUG ("DoReportDataOk m_minstrelTable_Allgroup[" << station->m_group_txrate  << "][" << station->m_txrate << "]: Success = " << tmp_minstrelTable[station->m_txrate].numRateSuccess << ", attempt = " << tmp_minstrelTable[station->m_txrate].numRateAttempt);		// kjyoon
  station->m_minstrelTable_Allgroup[org_group] = tmp_minstrelTable;		// kjyoon
  station->m_packetCount++;

  
  if (m_nsupported >= 1)
  {
    station->m_txrate = FindRate (station);
  }
}

  void
MinstrelWifiManager::DoReportFinalDataFailed (WifiRemoteStation *st)
{
  MinstrelWifiRemoteStation *station = (MinstrelWifiRemoteStation *) st;
  NS_LOG_DEBUG ("DoReportFinalDataFailed m_txrate=" << station->m_txrate);

  UpdateRetry (station);

  uint32_t org_group = station->m_group_txrate;	// kjyoon
  MinstrelRate tmp_minstrelTable;	// kjyoon
  tmp_minstrelTable = station->m_minstrelTable_Allgroup[station->m_group_txrate];		// kjyoon
  tmp_minstrelTable[station->m_txrate].numRateAttempt += station->m_retry;
  station->m_minstrelTable_Allgroup[org_group] = tmp_minstrelTable;		// kjyoon
  station->m_err++;

  if (m_nsupported >= 1)
  {
    station->m_txrate = FindRate (station);
  }
}

  void
MinstrelWifiManager::UpdateRetry (MinstrelWifiRemoteStation *station)
{
  station->m_retry = station->m_shortRetry + station->m_longRetry;
  station->m_shortRetry = 0;
  station->m_longRetry = 0;
}

  WifiTxVector
MinstrelWifiManager::DoGetDataTxVector (WifiRemoteStation *st,
    uint32_t size)
{
  MinstrelWifiRemoteStation *station = (MinstrelWifiRemoteStation *) st;
  if (!station->m_initialized)
  {
    CheckInit (station);

    /// start the rate at half way
    station->m_txrate = m_nsupported / 2;
  }
  
  DowngradeRate(station);

  UpdateStats (station);
  if (HasHtSupported())
    return WifiTxVector (McsToWifiMode(GetMcsSupported (station, station->m_txrate)), GetDefaultTxPowerLevel (), GetLongRetryCount (station), GetShortGuardInterval (station), station->m_group_txrate+1, GetNumberOfTransmitAntennas (station), GetStbc (station));	// kjyoon
  else
    return WifiTxVector (GetSupported (station, station->m_txrate), GetDefaultTxPowerLevel (), GetLongRetryCount (station), GetShortGuardInterval (station), station->m_group_txrate+1, GetNumberOfTransmitAntennas (station), GetStbc (station));	// kjyoon
}

//11ac: multiple_stream_tx_ra
  WifiTxVector
MinstrelWifiManager::DoGetDataTxVector (WifiRemoteStation *st, uint32_t size, uint16_t bw)
{
  MinstrelWifiRemoteStation *station = (MinstrelWifiRemoteStation *) st;
  if (!station->m_initialized)
  {
    CheckInit (station);

    /// start the rate at half way
    station->m_txrate = m_nsupported / 2;
  }

  DowngradeRate(station);

  //shbyyeon 11ac
	WifiMode curMode;
  if(HasVhtSupported())
    curMode = AcMcsToWifiMode(GetMcsSupported (station, station->m_txrate), bw);
  else if (HasHtSupported())
    curMode = McsToWifiMode(GetMcsSupported (station, station->m_txrate));
  else
    curMode = GetSupported (station, station->m_txrate);

  UpdateStats (station);
  return WifiTxVector (curMode, GetDefaultTxPowerLevel (), GetLongRetryCount (station), GetShortGuardInterval (station), station->m_group_txrate+1, GetNumberOfTransmitAntennas (station), GetStbc (station));
}

  WifiTxVector
MinstrelWifiManager::DoGetRtsTxVector (WifiRemoteStation *st)
{
  MinstrelWifiRemoteStation *station = (MinstrelWifiRemoteStation *) st;
  NS_LOG_DEBUG ("DoGetRtsMode m_txrate=" << station->m_txrate);

  return WifiTxVector (GetSupported (station, 0), GetDefaultTxPowerLevel (), GetShortRetryCount (station), GetShortGuardInterval (station), Min (GetNumberOfReceiveAntennas (station),GetNumberOfTransmitAntennas()), GetNumberOfTransmitAntennas (station), GetStbc (station));
}
//11ac: multiple_stream_tx_ra
  WifiTxVector
MinstrelWifiManager::DoGetRtsTxVector (WifiRemoteStation *st, uint16_t bw)
{
  MinstrelWifiRemoteStation *station = (MinstrelWifiRemoteStation *) st;
  NS_LOG_DEBUG ("DoGetRtsMode m_txrate=" << station->m_txrate);

  if (HasVhtSupported())
    return WifiTxVector (AcMcsToWifiMode(GetMcsSupported (station, station->m_txrate), bw), GetDefaultTxPowerLevel (), GetLongRetryCount (station), GetShortGuardInterval (station), Min (GetNumberOfReceiveAntennas (station),GetNumberOfTransmitAntennas()), GetNumberOfTransmitAntennas (station), GetStbc (station));
  else if (HasHtSupported())
    return WifiTxVector (McsToWifiMode(GetMcsSupported (station, station->m_txrate)), GetDefaultTxPowerLevel (), GetLongRetryCount (station), GetShortGuardInterval (station), Min (GetNumberOfReceiveAntennas (station),GetNumberOfTransmitAntennas()), GetNumberOfTransmitAntennas (station), GetStbc (station));
  else
    return WifiTxVector (GetSupported (station, station->m_txrate), GetDefaultTxPowerLevel (), GetLongRetryCount (station), GetShortGuardInterval (station), Min (GetNumberOfReceiveAntennas (station),GetNumberOfTransmitAntennas()), GetNumberOfTransmitAntennas (station), GetStbc (station));
}

bool
MinstrelWifiManager::IsLowLatency (void) const
{
  return true;
}
  uint32_t
MinstrelWifiManager::GetNextSample (MinstrelWifiRemoteStation *station)
{
  station->m_cs_group++;
  uint32_t tmp_cs_group = station->m_cs_group;	// kjyoon	// Initial value tmp_cs_group=1 can be a problem?
  //  NS_LOG_DEBUG("Current sampling group: " << tmp_cs_group);
  if (station->m_cs_group >= station->m_ngroup)	// kjyoon
  {
    //	NS_LOG_DEBUG("Samping group is bigger than limit!!	kjyoon");  
    tmp_cs_group = 0;
    station->m_cs_group = 0;
  }
  uint32_t bitrate;
  SampleRate tmp_sampleTable;	// kjyoon
  tmp_sampleTable = station->m_sampleTable_Allgroup[tmp_cs_group];	// kjyoon
  bitrate = tmp_sampleTable[station->m_index[tmp_cs_group]][station->m_col[tmp_cs_group]];
  station->m_index[tmp_cs_group]++;

  /// bookeeping for m_index and m_col variables
  if (station->m_index[tmp_cs_group] > (m_nsupported - 2))
  {
    station->m_index[tmp_cs_group] = 0;
    station->m_col[tmp_cs_group]++;
    if (station->m_col[tmp_cs_group] >= m_sampleCol)
    {
      station->m_col[tmp_cs_group] = 0;
    }
  }
  // NS_LOG_DEBUG("Sampling MCS: " << bitrate << " Group: " << station->m_cs_group );	// kjyoon
  station->m_group_txrate = tmp_cs_group;	// kjyoon
  NS_LOG_DEBUG("m_group_txrate update to tmp_cs_group: " << station->m_group_txrate);	// kjyoon
	//shbyeon bw20 mcs9 bug fix
	if(station->m_group_txrate == 0 && bitrate == 9)
		bitrate = station->m_maxTpRate;
  return bitrate;
}

  uint32_t
MinstrelWifiManager::FindRate (MinstrelWifiRemoteStation *station)
{
  NS_LOG_DEBUG ("FindRate " << "packet=" << station->m_packetCount );

  if ((station->m_sampleCount + station->m_packetCount) == 0)
  {
    station->m_group_txrate = 0;	// kjyoon
    NS_LOG_DEBUG("FindRate! m_group_txrate update to 0: " << station->m_group_txrate);	// kjyoon
    return 0;
  }


  uint32_t idx;
  MinstrelRate tmp_minstrelTable;	// kjyoon

  /**
   * if we are below the target of look around rate percentage, look around
   * note: do it randomly by flipping a coin instead sampling
   * all at once until it reaches the look around rate
   */
  NS_LOG_DEBUG ("m_sampleWait: " << station->m_sampleWait << ", m_waited: " << station->m_waited << " cur_mpdu: " << station->cur_mpdu );
  if (station->m_sampleWait < station->m_waited) 
  {
    station->m_sampleWait = 16 + 2*station->avg_ampduLen;	// 16 + 2*avg_ampduLen
    station->m_waited = 0;

    idx = GetNextSample (station);
    tmp_minstrelTable = station->m_minstrelTable_Allgroup[station->m_cs_group];		// kjyoon

    /**
     * This if condition is used to make sure that we don't need to use
     * the sample rate it is the same as our current rate
     */
    if ((idx != station->m_maxTpRate || station->m_cs_group != station->m_group_maxTpRate) 
        && (idx != station->m_maxTpRate2 || station->m_cs_group != station->m_group_maxTpRate2)
        && (idx != station->m_maxProbRate || station->m_cs_group != station->m_group_maxProbRate)
        && (idx != station->m_txrate || station->m_cs_group != station->m_group_txrate))
    {
      PrintTable(station);
      NS_LOG_DEBUG ("sampling rate " << idx << "(" << station->m_cs_group << ") success prob: " << tmp_minstrelTable[idx].ewmaProb << ", threshold: " << 18000*0.95 << "(0.95)");
      if (tmp_minstrelTable[idx].ewmaProb < (uint32_t)(18000*0.95)) // 18000 is probability scale
      {
        /// set flag that we are currently sampling
        station->m_isSampling = true;

        NS_LOG_DEBUG ("m_isSampling = true");   // kjyoon
      }
      else
      {
        NS_LOG_DEBUG ("FindRate: [Exit sampling for enough success prob.]");
      }
    }
    else
    {
      station->m_isSampling = false;
      NS_LOG_DEBUG ("FindRate: m_isSampling = false");
      NS_LOG_DEBUG ("FindRate: [Exit sampling for same rate w/ best ...] sample idx: " << idx << "(" << station->m_cs_group << "), maxTpRate: " << station->m_maxTpRate << "(" << station->m_group_maxTpRate << "), maxTpRate2: " << station->m_maxTpRate2 << "(" << station->m_group_maxTpRate2 << "), maxProbRate: " << station->m_maxProbRate << "(" << station->m_group_maxProbRate << "), txrate: " << station->m_txrate << "(" << station->m_group_txrate << ")");	// kjyoon  Need modification later. In this case, we have to choose new sample rate. And if choosing number exceed avg_ampduLen, send A-MPDU with current rate.
    }

    if (station->m_isSampling == true)
    {
      /// start sample count
      station->m_sampleCount++;

      /// bookeeping for resetting stuff
      if (station->m_packetCount >= 10000)
      {
        station->m_sampleCount = 0;
        station->m_packetCount = 0;
      }

      /// error check
      if (idx >= m_nsupported)
      {
        NS_LOG_DEBUG ("ALERT!!! ERROR");
      }

      /// set the rate that we're currently sampling
      station->m_sampleRate = idx;
      station->m_group_txrate = station->m_cs_group;

      if (station->m_sampleRate == station->m_maxTpRate)
      {
        station->m_sampleRate = station->m_maxTpRate2;
        station->m_group_txrate = station->m_group_maxTpRate2;
      }
    } // end "if (station->m_isSampling == true)"

  }
  ///	continue using the best rate
  else
  {
    NS_LOG_DEBUG("m_waited update from " << station->m_waited << " to " << station->m_waited+station->cur_mpdu); 
    station->m_waited += station->cur_mpdu;	// kjyoon

    idx = station->m_maxTpRate;
    station->m_group_txrate = station->m_group_maxTpRate;	// kjyoon
    NS_LOG_DEBUG("CONTINUE: txrate(group_txrate) of maxTpRate: " << idx << "(" << station->m_group_txrate << ")");	// kjyoon
    station->m_isSampling = false;
    NS_LOG_DEBUG ("FindRate: m_isSampling = false");
  }

  NS_LOG_DEBUG ("Use rate=" << idx << " Group=" << station->m_group_txrate);
  //  NS_LOG_DEBUG ("FindRate " << "sample rate=" << idx << " Group=" << station->m_group_txrate);	// kjyoon

  return idx;
}

  void
MinstrelWifiManager::UpdateStats (MinstrelWifiRemoteStation *station)
{
  if (Simulator::Now () <  station->m_nextStatsUpdate)
  {
    return;
  }

  if (!station->m_initialized)
  {
    return;
  }
  NS_LOG_DEBUG ("Updating stats=" << this);

  station->m_nextStatsUpdate = Simulator::Now () + m_updateStats;

  Time txTime;
  uint32_t tempProb;

  station->avg_ampduLen = static_cast<uint32_t> (((station->avg_ampduLen * (100 - m_ewmaLevel)) + (m_ewmaLevel * station->sum_mpdu / station->sum_packet) ) / 100);	// kjyoon
  NS_LOG_DEBUG ("sum_mpdu: " << station->sum_mpdu << ", sum_packet: " << station->sum_packet << ", avg_ampduLen: " << station->avg_ampduLen);	// kjyoon
  station->sum_mpdu = 0;
  station->sum_packet = 0;

  MinstrelRate tmp_minstrelTable;	// kjyoon
  for (uint32_t i_gr = 0; i_gr < station->m_ngroup; i_gr++)
  {
    tmp_minstrelTable = station->m_minstrelTable_Allgroup[i_gr];		// kjyoon
    for (uint32_t i = 0; i < m_nsupported; i++)
    {

      /// calculate the perfect tx time for this rate
      txTime = tmp_minstrelTable[i].perfectTxTime;

      /// just for initialization
      if (txTime.GetMicroSeconds () == 0)
      {
        txTime = Seconds (1);
      }

      NS_LOG_DEBUG ("m_txrate[nss" << i_gr << "][rate" << i << "]=" << station->m_txrate <<
          "\t attempt=" << tmp_minstrelTable[i].numRateAttempt <<
          "\t success=" << tmp_minstrelTable[i].numRateSuccess);

      /// if we've attempted something
      if (tmp_minstrelTable[i].numRateAttempt)
      {
        /**
         * calculate the probability of success
         * assume probability scales from 0 to 18000
         */
        tempProb = (tmp_minstrelTable[i].numRateSuccess * 18000) / tmp_minstrelTable[i].numRateAttempt;

        /// bookeeping
        tmp_minstrelTable[i].successHist += tmp_minstrelTable[i].numRateSuccess;
        tmp_minstrelTable[i].attemptHist += tmp_minstrelTable[i].numRateAttempt;
        tmp_minstrelTable[i].prob = tempProb;

        /// ewma probability (cast for gcc 3.4 compatibility)
        tempProb = static_cast<uint32_t> (((tempProb * (100 - m_ewmaLevel)) + (tmp_minstrelTable[i].ewmaProb * m_ewmaLevel) ) / 100);

        tmp_minstrelTable[i].ewmaProb = tempProb;

        /// calculating throughput
        tmp_minstrelTable[i].throughput = tempProb * (1000000 / txTime.GetMicroSeconds ());

      }

      /// bookeeping
      tmp_minstrelTable[i].prevNumRateAttempt = tmp_minstrelTable[i].numRateAttempt;
      tmp_minstrelTable[i].prevNumRateSuccess = tmp_minstrelTable[i].numRateSuccess;
      tmp_minstrelTable[i].numRateSuccess = 0;
      tmp_minstrelTable[i].numRateAttempt = 0;

      /// Sample less often below 10% and  above 95% of success
      if ((tmp_minstrelTable[i].ewmaProb > 17100) || (tmp_minstrelTable[i].ewmaProb < 1800))
      {
        /**
         * retry count denotes the number of retries permitted for each rate
         * # retry_count/2
         */
        tmp_minstrelTable[i].adjustedRetryCount = tmp_minstrelTable[i].retryCount >> 1;
        if (tmp_minstrelTable[i].adjustedRetryCount > 2)
        {
          tmp_minstrelTable[i].adjustedRetryCount = 2;
        }
      }
      else
      {
        tmp_minstrelTable[i].adjustedRetryCount = tmp_minstrelTable[i].retryCount;
      }

      /// if it's 0 allow one retry limit
      if (tmp_minstrelTable[i].adjustedRetryCount == 0)
      {
        tmp_minstrelTable[i].adjustedRetryCount = 1;
      }
    }
    station->m_minstrelTable_Allgroup[i_gr] = tmp_minstrelTable;		// kjyoon
  }

  uint32_t max_prob = 0, index_max_prob = 0, max_tp = 0, index_max_tp = 0, index_max_tp2 = 0;
  uint32_t index_group_max_prob = 0, index_group_max_tp = 0, index_group_max_tp2 = 0;

  uint32_t local_max_tp = 0;		// for finding maxTpRate of each group	kjyoon

  /// go find max throughput, high probability succ
  for (uint32_t i_gr = 0; i_gr < station->m_ngroup; i_gr++)
  {
    tmp_minstrelTable = station->m_minstrelTable_Allgroup[i_gr];		// kjyoon
    local_max_tp = 0;
    for (uint32_t i = 0; i < m_nsupported; i++)
    {
      NS_LOG_DEBUG ("throughput[nss" << i_gr << "][rate" << i << "]= " << tmp_minstrelTable[i].throughput << ", ewma" << tmp_minstrelTable[i].ewmaProb);

      if (local_max_tp < tmp_minstrelTable[i].throughput)
      {
        station->m_local_maxTpRate[i_gr] = i;
        local_max_tp = tmp_minstrelTable[i].throughput;
      }
      if (max_tp < tmp_minstrelTable[i].throughput)
      {
        index_max_tp = i;
        index_group_max_tp = i_gr;	// kjyoon
        max_tp = tmp_minstrelTable[i].throughput;
      }

      if (tmp_minstrelTable[i].ewmaProb > 0.75) {
        if (tmp_minstrelTable[i].throughput > tmp_minstrelTable[index_max_prob].throughput) {
          index_max_prob = i;
          index_group_max_prob = i_gr;	// kjyoon
          max_prob = tmp_minstrelTable[i].ewmaProb;
        }
      }
      else {
        if (max_prob < tmp_minstrelTable[i].ewmaProb)
        {
          index_max_prob = i;
          index_group_max_prob = i_gr;	// kjyoon
          max_prob = tmp_minstrelTable[i].ewmaProb;
        }
      }
    }
    station->m_minstrelTable_Allgroup[i_gr] = tmp_minstrelTable;		// kjyoon
  }


  max_tp = 0;
  /// find the second highest max
  for (uint32_t i_gr = 0; i_gr < station->m_ngroup; i_gr++)
  {
    tmp_minstrelTable = station->m_minstrelTable_Allgroup[i_gr];		// kjyoon
    local_max_tp = 0;
    for (uint32_t i = 0; i < m_nsupported; i++)
    {
      if ((i != station->m_local_maxTpRate[i_gr]) && (local_max_tp < tmp_minstrelTable[i].throughput))
      {
        station->m_local_maxTpRate[i_gr] = i;
        local_max_tp = tmp_minstrelTable[i].throughput;
      }
      if (((i != index_max_tp) && (i_gr == index_group_max_tp)) && (max_tp < tmp_minstrelTable[i].throughput))
      //if (((i != index_max_tp) || (i_gr != index_group_max_tp)) && (max_tp < tmp_minstrelTable[i].throughput))
      {
        index_max_tp2 = i;
        index_group_max_tp2 = i_gr;	// kjyoon
        max_tp = tmp_minstrelTable[i].throughput;
      }
    }
    station->m_minstrelTable_Allgroup[i_gr] = tmp_minstrelTable;		// kjyoon
  }

  NS_LOG_DEBUG("maxTpRate: " << index_max_tp << "," << index_group_max_tp 
      << " maxTpRate2: " << index_max_tp2 << ","<< index_group_max_tp2	
      << " maxProbRate: " << index_max_prob << ","<< index_group_max_prob);	

  station->m_maxTpRate = index_max_tp;
  station->m_group_maxTpRate = index_group_max_tp;		// kjyoon
  station->m_maxTpRate2 = index_max_tp2;
  station->m_group_maxTpRate2 = index_group_max_tp2;		// kjyoon
  station->m_maxProbRate = index_max_prob;
  station->m_group_maxProbRate = index_group_max_prob;		// kjyoon
  station->m_currentRate = index_max_tp;
  station->m_group_currentRate = index_group_max_tp;		// kjyoon

  if (index_max_tp > station->m_txrate && index_group_max_tp >= station->m_group_txrate)
  {
    station->m_txrate = index_max_tp;
    station->m_group_txrate = index_group_max_tp;	// kjyoon
    NS_LOG_DEBUG("m_group_txrate update to max_tp: " << station->m_group_txrate);	// kjyoon
  }

  NS_LOG_DEBUG ("max tp=" << index_max_tp << "\nmax tp2=" << index_max_tp2 << "\nmax prob=" << index_max_prob);
  //  NS_LOG_DEBUG ("max tp=" << index_max_tp << "\nmax tp group=" << index_group_max_tp << "\nmax tp2=" << index_max_tp2 << "\nmax tp2 group=" << index_group_max_tp2 << "\nmax prob=" << index_max_prob << "\nmax prob group=" << index_group_max_prob);

  //PrintTable(station);	// kjyoon

  /// reset it
  RateInit (station);
}

  void
MinstrelWifiManager::RateInit (MinstrelWifiRemoteStation *station)
{
  NS_LOG_DEBUG ("RateInit=" << station);
  MinstrelRate tmp_minstrelTable;	// kjyoon
  int64_t tmp_Nss;	// kjyoon
  for (uint32_t i_gr = 0; i_gr < station->m_ngroup; i_gr++)
  {
    tmp_minstrelTable = station->m_minstrelTable_Allgroup[i_gr];		// kjyoon
    tmp_Nss = i_gr + 1;
    for (uint32_t i = 0; i < m_nsupported; i++)
    {
      tmp_minstrelTable[i].numRateAttempt = 0;
      tmp_minstrelTable[i].numRateSuccess = 0;
      tmp_minstrelTable[i].prob = 0;	// 18000 is probability scale
      tmp_minstrelTable[i].ewmaProb = 0;
      tmp_minstrelTable[i].prevNumRateAttempt = 0;
      tmp_minstrelTable[i].prevNumRateSuccess = 0;
      tmp_minstrelTable[i].successHist = 0;
      tmp_minstrelTable[i].attemptHist = 0;
      tmp_minstrelTable[i].throughput = 0;
      //11ac: multiple multiple_stream_tx_ra
      if (HasVhtSupported())
        tmp_minstrelTable[i].perfectTxTime = GetCalcTxTime (AcMcsToWifiMode(GetMcsSupported (station, i), GetCurrentBandwidth(station)));
      else if (HasHtSupported())	  
        tmp_minstrelTable[i].perfectTxTime = GetCalcTxTime (McsToWifiMode(GetMcsSupported (station, i)));
      else
        tmp_minstrelTable[i].perfectTxTime = GetCalcTxTime (GetSupported (station, i));
      tmp_minstrelTable[i].perfectTxTime = tmp_minstrelTable[i].perfectTxTime / tmp_Nss;	// kjyoon	// For simplification, we just divide perfectTxTime by # of spatial stream.
      tmp_minstrelTable[i].retryCount = 1;
      tmp_minstrelTable[i].adjustedRetryCount = 1;
    }
    station->m_minstrelTable_Allgroup[i_gr] = tmp_minstrelTable;		// kjyoon
  }
}

  void
MinstrelWifiManager::InitSampleTable (MinstrelWifiRemoteStation *station)
{
  NS_LOG_DEBUG ("InitSampleTable=" << this);

  station->m_col[0] = station->m_index[0] = 0;
  station->m_col[1] = station->m_index[1] = 0;
  station->m_col[2] = station->m_index[2] = 0;

  /// for off-seting to make rates fall between 0 and numrates
  uint32_t numSampleRates = m_nsupported;

  uint32_t newIndex;
  for (uint32_t i_gr = 0; i_gr < station->m_ngroup; i_gr++)
  {
    SampleRate tmp_sampleTable;	// kjyoon
    tmp_sampleTable = station->m_sampleTable_Allgroup[i_gr];	// kjyoon

    for (uint32_t col = 0; col < m_sampleCol; col++)
    {
      for (uint32_t i = 0; i < numSampleRates; i++ )
      {

        /**
         * The next two lines basically tries to generate a random number
         * between 0 and the number of available rates
         */
        int uv = m_uniformRandomVariable->GetInteger (0, numSampleRates);
        newIndex = (i + uv) % numSampleRates;

        /// this loop is used for filling in other uninitilized places
        while (tmp_sampleTable[newIndex][col] != 0)
        {
          newIndex = (newIndex + 1) % m_nsupported;
        }
        tmp_sampleTable[newIndex][col] = i;

      }
    }
    station->m_sampleTable_Allgroup[i_gr] = tmp_sampleTable ;	// kjyoon
  }
}

  void
MinstrelWifiManager::PrintSampleTable (MinstrelWifiRemoteStation *station)
{
  NS_LOG_DEBUG ("PrintSampleTable=" << station);

  uint32_t numSampleRates = m_nsupported;
  SampleRate tmp_sampleTable;	// kjyoon

  for (uint32_t i_gr = 0; i_gr < station->m_ngroup; i_gr++)	// kjyoon
  {
    tmp_sampleTable = station->m_sampleTable_Allgroup[i_gr];	// kjyoon
    std::cout << "[Group " << i_gr << "]\n";
    for (uint32_t i = 0; i < numSampleRates; i++)
    {
      for (uint32_t j = 0; j < m_sampleCol; j++)
      {
        std::cout << tmp_sampleTable[i][j] << "\t";
      }
      std::cout << std::endl;
    }
  }
}

  void
MinstrelWifiManager::PrintTable (MinstrelWifiRemoteStation *station)
{
  NS_LOG_DEBUG ("PrintTable=" << station);

  MinstrelRate tmp_minstrelTable;	// kjyoon

  for (uint32_t i_gr = 0; i_gr < station->m_ngroup; i_gr++)	// kjyoon
  {
    tmp_minstrelTable = station->m_minstrelTable_Allgroup[i_gr];	// kjyoon
    //		std::cout << "[Group " << i_gr << "]\n";
    NS_LOG_DEBUG ("[Group " << i_gr << "]");
    for (uint32_t i = 0; i < m_nsupported; i++)
    {
      //		  std::cout << "index(" << i << ") = " << tmp_minstrelTable[i].perfectTxTime << "\n";
      NS_LOG_DEBUG ("index(" << i << ") >> Throughput: " << tmp_minstrelTable[i].throughput << ", ewmaProb: " << tmp_minstrelTable[i].ewmaProb);
    }
  }
}

} // namespace ns3





