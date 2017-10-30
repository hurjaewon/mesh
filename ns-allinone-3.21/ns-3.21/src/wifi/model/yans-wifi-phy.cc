/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2005,2006 INRIA
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
 * Author: Ghada Badawy <gbadawy@gmail.com>
 */

#include "yans-wifi-phy.h"
#include "yans-wifi-channel.h"
#include "wifi-mode.h"
#include "wifi-preamble.h"
#include "wifi-phy-state-helper.h"
#include "error-rate-model.h"
#include "ns3/simulator.h"
#include "ns3/packet.h"
#include "ns3/assert.h"
#include "ns3/log.h"
#include "ns3/double.h"
#include "ns3/uinteger.h"
#include "ns3/enum.h"
#include "ns3/pointer.h"
#include "ns3/net-device.h"
#include "ns3/trace-source-accessor.h"
#include "ns3/boolean.h"
#include <cmath>
//shbyeon
#include "ns3/ampdu-tag.h"
#include "ampdu-mpdu-delimiter.h"
#include "ns3/error-free-tag.h"
#include "wifi-bonding.h"
#include "ns3/duplicate-tag.h"
#include "ns3/string.h" //11ac: vht_standard

NS_LOG_COMPONENT_DEFINE ("YansWifiPhy");

namespace ns3 {

NS_OBJECT_ENSURE_REGISTERED (YansWifiPhy);

TypeId
YansWifiPhy::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::YansWifiPhy")
    .SetParent<WifiPhy> ()
    .AddConstructor<YansWifiPhy> ()
    .AddAttribute ("EnergyDetectionThreshold",
                   "The energy of a received signal should be higher than "
                   "this threshold (dbm) to allow the PHY layer to detect the signal.",
                   DoubleValue (-72.0),
                   //DoubleValue (-96.0),
                   MakeDoubleAccessor (&YansWifiPhy::SetEdThreshold,
                                       &YansWifiPhy::GetEdThreshold),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("CcaMode1Threshold",
                   "The energy of a received signal should be higher than "
                   "this threshold (dbm) to allow the PHY layer to declare CCA BUSY state",
                   DoubleValue (-82.0),
                   //DoubleValue (-99.0),
                   MakeDoubleAccessor (&YansWifiPhy::SetCcaMode1Threshold,
                                       &YansWifiPhy::GetCcaMode1Threshold),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("TxGain",
                   "Transmission gain (dB).",
                   DoubleValue (1.0),
                   MakeDoubleAccessor (&YansWifiPhy::SetTxGain,
                                       &YansWifiPhy::GetTxGain),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("RxGain",
                   "Reception gain (dB).",
                   DoubleValue (1.0),
                   MakeDoubleAccessor (&YansWifiPhy::SetRxGain,
                                       &YansWifiPhy::GetRxGain),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("TxPowerLevels",
                   "Number of transmission power levels available between "
                   "TxPowerStart and TxPowerEnd included.",
                   UintegerValue (1),
                   MakeUintegerAccessor (&YansWifiPhy::m_nTxPower),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("TxPowerEnd",
                   "Maximum available transmission level (dbm).",
                   DoubleValue (16.0206),
                   MakeDoubleAccessor (&YansWifiPhy::SetTxPowerEnd,
                                       &YansWifiPhy::GetTxPowerEnd),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("TxPowerStart",
                   "Minimum available transmission level (dbm).",
                   DoubleValue (16.0206),
                   MakeDoubleAccessor (&YansWifiPhy::SetTxPowerStart,
                                       &YansWifiPhy::GetTxPowerStart),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("RxNoiseFigure",
                   "Loss (dB) in the Signal-to-Noise-Ratio due to non-idealities in the receiver."
                   " According to Wikipedia (http://en.wikipedia.org/wiki/Noise_figure), this is "
                   "\"the difference in decibels (dB) between"
                   " the noise output of the actual receiver to the noise output of an "
                   " ideal receiver with the same overall gain and bandwidth when the receivers "
                   " are connected to sources at the standard noise temperature T0 (usually 290 K)\"."
                   " For",
                   DoubleValue (7),
                   MakeDoubleAccessor (&YansWifiPhy::SetRxNoiseFigure,
                                       &YansWifiPhy::GetRxNoiseFigure),
                   MakeDoubleChecker<double> ())
//    .AddAttribute ("State", "The state of the PHY layer",
//                   PointerValue (),
//                   MakePointerAccessor (&YansWifiPhy::m_state[0]),
//                   MakePointerChecker<WifiPhyStateHelper> ())
    .AddAttribute ("ChannelSwitchDelay",
                   "Delay between two short frames transmitted on different frequencies.",
                   TimeValue (MicroSeconds (250)),
                   MakeTimeAccessor (&YansWifiPhy::m_channelSwitchDelay),
                   MakeTimeChecker ())
    //802.11ac channel bonding 
		.AddAttribute ("ChannelWidth",
                   "Channel bonding width",
                   UintegerValue (20),
                   MakeUintegerAccessor (&YansWifiPhy::SetOperationalBandwidth,
                                         &YansWifiPhy::GetOperationalBandwidth),
                   MakeUintegerChecker<uint16_t> ())
    .AddAttribute ("DynamicAccess",
                   "dynamic access",
                   UintegerValue (0),
                   MakeUintegerAccessor (&YansWifiPhy::SetDynamicAccess,
                                         &YansWifiPhy::GetDynamicAccess),
                   MakeUintegerChecker<uint16_t> ())
		////////////////////
    .AddAttribute ("ChannelNumber",
                   "Channel center frequency = Channel starting frequency + 5 MHz * nch",
                   UintegerValue (36),
                   MakeUintegerAccessor (&YansWifiPhy::SetChannelNumber,
                                         &YansWifiPhy::GetChannelNumber),
                   MakeUintegerChecker<uint16_t> ())
    .AddAttribute ("Frequency", "The operating frequency.",
                   UintegerValue (2407),
                   MakeUintegerAccessor (&YansWifiPhy::GetFrequency,
                                        &YansWifiPhy::SetFrequency),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("Transmitters", "The number of transmitters.",
                   UintegerValue (1),
                   MakeUintegerAccessor (&YansWifiPhy::GetNumberOfTransmitAntennas,
                                        &YansWifiPhy::SetNumberOfTransmitAntennas),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("Receivers", "The number of recievers.",
                   UintegerValue (1),
                   MakeUintegerAccessor (&YansWifiPhy::GetNumberOfReceiveAntennas,
                                        &YansWifiPhy::SetNumberOfReceiveAntennas),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("ShortGuardEnabled", "Whether or not short guard interval is enabled.",
                   BooleanValue (false),
                   MakeBooleanAccessor (&YansWifiPhy::GetGuardInterval,
                                        &YansWifiPhy::SetGuardInterval),
                   MakeBooleanChecker ())
    .AddAttribute ("LdpcEnabled", "Whether or not LDPC is enabled.",
                   BooleanValue (false),
                   MakeBooleanAccessor (&YansWifiPhy::GetLdpc,
                                        &YansWifiPhy::SetLdpc),
                   MakeBooleanChecker ())
    .AddAttribute ("STBCEnabled", "Whether or not STBC is enabled.",
                   BooleanValue (false),
                   MakeBooleanAccessor (&YansWifiPhy::GetStbc,
                                        &YansWifiPhy::SetStbc),
                   MakeBooleanChecker ())
    .AddAttribute ("GreenfieldEnabled", "Whether or not STBC is enabled.",
                   BooleanValue (false),
                   MakeBooleanAccessor (&YansWifiPhy::GetGreenfield,
                                        &YansWifiPhy::SetGreenfield),
                   MakeBooleanChecker ())
    .AddAttribute ("ChannelBonding", "Whether 20MHz or 40MHz.",
                   BooleanValue (false),
                   MakeBooleanAccessor (&YansWifiPhy::GetChannelBonding,
                                        &YansWifiPhy::SetChannelBonding),
                   MakeBooleanChecker ())
    .AddAttribute ("SecondCaptureCapability", "Whether or not second-capture effect is capable.",  //11ac: second_capture 
                   BooleanValue (true),
                   MakeBooleanAccessor (&YansWifiPhy::m_secondCaptureCapability),
                   MakeBooleanChecker ())               

    .AddTraceSource ("TxTime",
                     "transmit duration",
                     MakeTraceSourceAccessor (&YansWifiPhy::m_txTime))

  ;
  return tid;
}

YansWifiPhy::YansWifiPhy ()
 :  m_channelNumber (36),
		m_secondary20 (40),
    m_secondary40_up (44),
    m_secondary40_down (48),
    m_currentWidth(20),
    m_endRxEvent (),
    m_channelStartingFrequency (0)
{
  NS_LOG_FUNCTION (this);
  m_random = CreateObject<UniformRandomVariable> ();
  for(int j = 0; j < 4; j++)
    m_state[j] = CreateObject<WifiPhyStateHelper> ();
}

YansWifiPhy::~YansWifiPhy ()
{
  NS_LOG_FUNCTION (this);
}

void
YansWifiPhy::DoDispose (void)
{
  NS_LOG_FUNCTION (this);
  m_channel = 0;
  m_deviceRateSet.clear ();
  m_deviceMcsSet.clear();
  m_device = 0;
  m_mobility = 0;
  for(int j = 0; j < 4; j++)
    m_state[j] = 0;
}

void
YansWifiPhy::ConfigureStandard (enum WifiPhyStandard standard)
{
  NS_LOG_FUNCTION (this << standard);
  switch (standard)
    {
    case WIFI_PHY_STANDARD_80211a:
      Configure80211a ();
      break;
    case WIFI_PHY_STANDARD_80211b:
      Configure80211b ();
      break;
    case WIFI_PHY_STANDARD_80211g:
      Configure80211g ();
      break;
    case WIFI_PHY_STANDARD_80211_10MHZ:
      Configure80211_10Mhz ();
      break;
    case WIFI_PHY_STANDARD_80211_5MHZ:
      Configure80211_5Mhz ();
      break;
    case WIFI_PHY_STANDARD_holland:
      ConfigureHolland ();
      break;
    case WIFI_PHY_STANDARD_80211n_2_4GHZ:
      m_channelStartingFrequency=2407;
      Configure80211n ();
      break;
    case WIFI_PHY_STANDARD_80211n_5GHZ:
      m_channelStartingFrequency=5e3;
      Configure80211n ();
      break;
    //11ac: vht_standard	
    case WIFI_PHY_STANDARD_80211ac:
	Configure80211ac ();
	break;

    default:
      NS_ASSERT (false);
      break;
    }
}


void
YansWifiPhy::SetRxNoiseFigure (double noiseFigureDb)
{
  NS_LOG_FUNCTION (this << noiseFigureDb);
	//802.11ac channel bonding
  m_interference[0].SetNoiseFigure (DbToRatio (noiseFigureDb));
  m_interference[1].SetNoiseFigure (DbToRatio (noiseFigureDb));
  m_interference[2].SetNoiseFigure (DbToRatio (noiseFigureDb));
  m_interference[3].SetNoiseFigure (DbToRatio (noiseFigureDb));
}
void
YansWifiPhy::SetTxPowerStart (double start)
{
  NS_LOG_FUNCTION (this << start);
  m_txPowerBaseDbm = start;
}
void
YansWifiPhy::SetTxPowerEnd (double end)
{
  NS_LOG_FUNCTION (this << end);
  m_txPowerEndDbm = end;
}
void
YansWifiPhy::SetNTxPower (uint32_t n)
{
  NS_LOG_FUNCTION (this << n);
  m_nTxPower = n;
}
void
YansWifiPhy::SetTxGain (double gain)
{
  NS_LOG_FUNCTION (this << gain);
  m_txGainDb = gain;
}
void
YansWifiPhy::SetRxGain (double gain)
{
  NS_LOG_FUNCTION (this << gain);
  m_rxGainDb = gain;
}
void
YansWifiPhy::SetEdThreshold (double threshold)
{
  NS_LOG_FUNCTION (this << threshold);
  m_edThresholdW = DbmToW (threshold);
}
void
YansWifiPhy::SetCcaMode1Threshold (double threshold)
{
  NS_LOG_FUNCTION (this << threshold);
  m_ccaMode1ThresholdW = DbmToW (threshold);
}
void
YansWifiPhy::SetErrorRateModel (Ptr<ErrorRateModel> rate)
{
	//802.11ac channel bonding
  m_interference[0].SetErrorRateModel (rate);
  m_interference[1].SetErrorRateModel (rate);
  m_interference[2].SetErrorRateModel (rate);
  m_interference[3].SetErrorRateModel (rate);
}
void
YansWifiPhy::SetDevice (Ptr<Object> device)
{
  m_device = device;
}
void
YansWifiPhy::SetMobility (Ptr<Object> mobility)
{
  m_mobility = mobility;
}

double
YansWifiPhy::GetRxNoiseFigure (void) const
{
  return RatioToDb (m_interference[0].GetNoiseFigure ());
}
double
YansWifiPhy::GetTxPowerStart (void) const
{
  return m_txPowerBaseDbm;
}
double
YansWifiPhy::GetTxPowerEnd (void) const
{
  return m_txPowerEndDbm;
}
double
YansWifiPhy::GetTxGain (void) const
{
  return m_txGainDb;
}
double
YansWifiPhy::GetRxGain (void) const
{
  return m_rxGainDb;
}

double
YansWifiPhy::GetEdThreshold (void) const
{
  return WToDbm (m_edThresholdW);
}

double
YansWifiPhy::GetCcaMode1Threshold (void) const
{
  return WToDbm (m_ccaMode1ThresholdW);
}

Ptr<ErrorRateModel>
YansWifiPhy::GetErrorRateModel (void) const
{
  return m_interference[0].GetErrorRateModel ();
}
Ptr<Object>
YansWifiPhy::GetDevice (void) const
{
  return m_device;
}
Ptr<Object>
YansWifiPhy::GetMobility (void)
{
  return m_mobility;
}
//shbyeon 802.11ac genie
InterferenceHelper
YansWifiPhy::GetInterferenceHelper(void)
{
	return m_interference[0];
}
double
YansWifiPhy::CalculateSnr (WifiMode txMode, double ber) const
{
  return m_interference[0].GetErrorRateModel ()->CalculateSnr (txMode, ber);
}

Ptr<WifiChannel>
YansWifiPhy::GetChannel (void) const
{
  return m_channel;
}
void
YansWifiPhy::SetChannel (Ptr<YansWifiChannel> channel)
{
  m_channel = channel;
  m_channel->Add (this);
}

//802.11ac channel bonding 
DynamicAccessFlag
YansWifiPhy::RestartBackoff (uint16_t bw) const
{
  bool s20 = false;
  bool s40u = false;
  bool s40d = false;

  if(m_state[1]->GetState() == YansWifiPhy::IDLE)
  {
    s20 = m_state[1]->IdleForPifs();
  }
  if(m_state[2]->GetState() == YansWifiPhy::IDLE)
  {
    s40u = m_state[2]->IdleForPifs();
  }
  if(m_state[3]->GetState() == YansWifiPhy::IDLE)
  {
    s40d = m_state[3]->IdleForPifs();
  }
  
  NS_LOG_DEBUG(Simulator::Now() << " bandwidth=" << bw <<
      " 1" << s20 << s40u << s40d);

  if(bw==20)
    return PRIMARY_20;
  else if(bw==40 && s20)
    return SECONDARY_20_IDLE;
  else if(bw==40 && !s20)
    return PRIMARY_20;
  else if(bw==80 && !s20 && !s40u && !s40d)
    return PRIMARY_20;
  else if(bw==80 && s20 && !s40u && !s40d)
    return SECONDARY_20_IDLE;
  else if(bw==80 && s20 && s40u && !s40d)
    return SECONDARY_20_IDLE;
  else if(bw==80 && s20 && !s40u && s40d)
    return SECONDARY_20_IDLE;
  else if(bw==80 && s20 && s40u && s40d)
    return ALL_IDLE;
  else
    return PRIMARY_20;
}

Ptr<WifiPhyStateHelper> 
YansWifiPhy::GetPrimaryState(void) const
{
  return m_state[0];
}
void
YansWifiPhy::SetCurrentWidth(uint16_t ncw)
{
  m_currentWidth = ncw;
  return;
}
uint16_t
YansWifiPhy::GetCurrentWidth(void) const
{
  return m_currentWidth;
}
void
YansWifiPhy::SetOperationalBandwidth(uint16_t ncw)
{
  m_operationalBandwidth = ncw;
  return;
}
void
YansWifiPhy::SetDynamicAccess(uint16_t da)
{
  m_da = da;
  return;
}
uint16_t
YansWifiPhy::GetOperationalBandwidth (void) const
{
  return m_operationalBandwidth;
}
uint16_t
YansWifiPhy::GetDynamicAccess (void) const
{
  return m_da;
}
enum ChannelBonding
YansWifiPhy::OverlapCheck(uint16_t rxCh, uint16_t rxBw, uint16_t rxS20, uint16_t rxS40_up, uint16_t rxS40_down)
{
  uint16_t curCh = GetChannelNumber();
  uint16_t curBw = GetCurrentWidth();

  ////////operationg channel with of rx side

  if(curBw == 20)
  {
    if(rxCh == curCh)
      return RECV_OCC_20;
    else if(rxS20 == curCh)
      return NOCC_SECONDARY_20;
    else if(rxS40_up == curCh)
      return NOCC_SECONDARY_40_UP;
    else if(rxS40_down == curCh)
      return NOCC_SECONDARY_40_DOWN;
    else
      return DIFF_CHANNEL;
  }
  else if(curBw == 40 && rxBw == 20)
  {
    if(rxCh == curCh || rxS20 == curCh)
      return NO_RECV_OCC_20_40;
    else if(rxS40_up == curCh || rxS40_down == curCh)
      return NOCC_SECONDARY_40_ALL;
    else
      return DIFF_CHANNEL;
  }
  else if(curBw == 40 && rxBw > 20)
  {
    if(rxCh == curCh || rxS20 == curCh)
      return RECV_OCC_40;
    else if(rxS40_up == curCh || rxS40_down == curCh)
      return NOCC_SECONDARY_40_ALL;
    else
      return DIFF_CHANNEL;
  }
  else if(curBw == 80)
  {
    if(rxCh == curCh || rxS20 == curCh || rxS40_up == curCh || rxS40_down == curCh)
    {
      if(rxBw == 80)
        return RECV_OCC_80;
      else if(rxBw == 40)
        return NO_RECV_OCC_40;
      else
        return NO_RECV_OCC_20_80;
    }
    else 
      return DIFF_CHANNEL;
  }
  else
  {
    NS_LOG_DEBUG("current channel width seems to be wrong!");
    NS_ASSERT(false);
    return DIFF_CHANNEL;
  }
}
/////////////////////////////////////////////

void
YansWifiPhy::SetChannelNumber (uint16_t nch)
{
	int j = 0;
  if (Simulator::Now () == Seconds (0))
    {
      // this is not channel switch, this is initialization
      NS_LOG_DEBUG ("start at channel " << nch);
			m_channelNumber = nch;
			switch (nch)
			{
				case 36:
				case 52:
				case 100:
				case 116:
				case 149:
					m_secondary20 = nch + 4;
					m_secondary40_up = nch + 8;
					m_secondary40_down = nch + 12;
					break;
				case 40:
				case 56:
				case 104:
				case 120:
				case 153:
					m_secondary20 = nch - 4;
					m_secondary40_up = nch + 4;
					m_secondary40_down = nch + 8;
					break;
				case 44:
				case 60:
				case 108:
				case 124:
				case 157:
					m_secondary20 = nch + 4;
					m_secondary40_up = nch - 4;
					m_secondary40_down = nch - 8;
					break;
				case 48:
				case 64:
				case 112:
				case 128:
				case 161:
					m_secondary20 = nch - 4;
					m_secondary40_up = nch - 8;
					m_secondary40_down = nch - 12;
					break;
				default:
					NS_ASSERT (false);
					break;
			}
			NS_LOG_DEBUG ("primary channel: " << nch << ", secondary: " << 
					m_secondary20 << ", secondary 40: " << (m_secondary40_up + m_secondary40_down)/2 );
			return;
    }

  NS_ASSERT (!IsStateSwitching ());
  switch (m_state[0]->GetState ())
    {
    case YansWifiPhy::RX:
      NS_LOG_DEBUG ("drop packet because of channel switching while reception");
      m_endRxEvent.Cancel ();
      goto switchChannel;
      break;
    case YansWifiPhy::TX:
      NS_LOG_DEBUG ("channel switching postponed until end of current transmission");
      Simulator::Schedule (GetDelayUntilIdle (), &YansWifiPhy::SetChannelNumber, this, nch);
      break;
    case YansWifiPhy::CCA_BUSY:
    case YansWifiPhy::IDLE:
      goto switchChannel;
      break;
    case YansWifiPhy::SLEEP:
      NS_LOG_DEBUG ("channel switching ignored in sleep mode");
      break;
    default:
      NS_ASSERT (false);
      break;
    }

  return;

switchChannel:

  NS_LOG_DEBUG ("switching channel " << m_channelNumber << " -> " << nch);
  m_state[0]->SwitchToChannelSwitching (m_channelSwitchDelay);
  m_state[1]->SwitchToChannelSwitching (m_channelSwitchDelay);
  m_state[2]->SwitchToChannelSwitching (m_channelSwitchDelay);
  m_state[3]->SwitchToChannelSwitching (m_channelSwitchDelay);

  for (j=0; j<4; j++)
    m_interference[j].EraseEvents ();
  /*
   * Needed here to be able to correctly sensed the medium for the first
   * time after the switching. The actual switching is not performed until
   * after m_channelSwitchDelay. Packets received during the switching
   * state are added to the event list and are employed later to figure
   * out the state of the medium after the switching.
   */
	m_channelNumber = nch;
	switch (nch)
	{
		case 36:
		case 52:
		case 100:
		case 116:
		case 149:
			m_secondary20 = nch + 4;
			m_secondary40_up = nch + 8;
			m_secondary40_down = nch + 12;
			break;
		case 40:
		case 56:
		case 104:
		case 120:
		case 153:
			m_secondary20 = nch - 4;
			m_secondary40_up = nch + 4;
			m_secondary40_down = nch + 8;
			break;
		case 44:
		case 60:
		case 108:
		case 124:
		case 157:
			m_secondary20 = nch + 4;
			m_secondary40_up = nch - 4;
			m_secondary40_down = nch - 8;
			break;
		case 48:
		case 64:
		case 112:
		case 128:
		case 161:
			m_secondary20 = nch - 4;
			m_secondary40_up = nch - 8;
			m_secondary40_down = nch - 12;
			break;
		default:
			NS_ASSERT (false);
			break;
	}
	NS_LOG_DEBUG ("switch channel ==> primary channel: " << nch << ", secondary: " << 
			m_secondary20 << ", secondary 40: " << (m_secondary40_up + m_secondary40_down)/2 );
}

uint16_t
YansWifiPhy::GetChannelNumberS20 () const
{
  return m_secondary20;
}
uint16_t
YansWifiPhy::GetChannelNumberS40_up () const
{
  return m_secondary40_up;
}
uint16_t
YansWifiPhy::GetChannelNumberS40_down () const
{
  return m_secondary40_down;
}


uint16_t
YansWifiPhy::GetChannelNumber () const
{
  return m_channelNumber;
}

double
YansWifiPhy::GetChannelFrequencyMhz () const
{
  return m_channelStartingFrequency + 5 * GetChannelNumber ();
}

void
YansWifiPhy::SetSleepMode (void)
{
  NS_LOG_FUNCTION (this);
  switch (m_state[0]->GetState ())
    {
    case YansWifiPhy::TX:
      NS_LOG_DEBUG ("setting sleep mode postponed until end of current transmission");
      Simulator::Schedule (GetDelayUntilIdle (), &YansWifiPhy::SetSleepMode, this);
      break;
    case YansWifiPhy::RX:
      NS_LOG_DEBUG ("setting sleep mode postponed until end of current reception");
      Simulator::Schedule (GetDelayUntilIdle (), &YansWifiPhy::SetSleepMode, this);
      break;
    case YansWifiPhy::SWITCHING:
      NS_LOG_DEBUG ("setting sleep mode postponed until end of channel switching");
      Simulator::Schedule (GetDelayUntilIdle (), &YansWifiPhy::SetSleepMode, this);
      break;
    case YansWifiPhy::CCA_BUSY:
    case YansWifiPhy::IDLE:
      NS_LOG_DEBUG ("setting sleep mode");
      m_state[0]->SwitchToSleep ();
      m_state[1]->SwitchToSleep ();
      m_state[2]->SwitchToSleep ();
      m_state[3]->SwitchToSleep ();
      break;
    case YansWifiPhy::SLEEP:
      NS_LOG_DEBUG ("already in sleep mode");
      break;
    default:
      NS_ASSERT (false);
      break;
    }
}

void
YansWifiPhy::ResumeFromSleep (void)
{
  NS_LOG_FUNCTION (this);
  switch (m_state[0]->GetState ())
    {
    case YansWifiPhy::TX:
    case YansWifiPhy::RX:
    case YansWifiPhy::IDLE:
    case YansWifiPhy::CCA_BUSY:
    case YansWifiPhy::SWITCHING:
      NS_LOG_DEBUG ("not in sleep mode, there is nothing to resume");
      break;
    case YansWifiPhy::SLEEP:
      NS_LOG_DEBUG ("resuming from sleep mode");
      Time delayUntilCcaEnd = m_interference[0].GetEnergyDuration (m_ccaMode1ThresholdW);
      m_state[0]->SwitchFromSleep (delayUntilCcaEnd);
      m_state[1]->SwitchFromSleep (delayUntilCcaEnd);
      m_state[2]->SwitchFromSleep (delayUntilCcaEnd);
      m_state[3]->SwitchFromSleep (delayUntilCcaEnd);
      break;
    }
}

void
YansWifiPhy::SetReceiveOkCallback (RxOkCallback callback)
{
  m_state[0]->SetReceiveOkCallback (callback);
}
void
YansWifiPhy::SetReceiveErrorCallback (RxErrorCallback callback)
{
  m_state[0]->SetReceiveErrorCallback (callback);
}

void
YansWifiPhy::StartReceivePacket (Ptr<Packet> packet,
                                 double rxPowerDbm,
                                 WifiTxVector txVector,
                                 enum WifiPreamble preamble,
																 enum ChannelBonding ch)
{
  NS_LOG_FUNCTION (this << packet << rxPowerDbm << txVector.GetMode() << preamble << ch);
  rxPowerDbm += m_rxGainDb;
  double rxPowerW = DbmToW (rxPowerDbm);
  Time rxDuration = CalculateTxDuration (packet->GetSize (), txVector, preamble);
	NS_LOG_DEBUG ("rxDuration: " << rxDuration);
  WifiMode txMode = txVector.GetMode();

//160413 skim11 : channel bug fix
	double realRxPowerDbm = rxPowerDbm;
	double realRxPowerW = rxPowerW;
	uint8_t nss = txVector.GetNss();
	if (nss == 1){
		std::complex<double> **ch = new std::complex<double> *[nss];
		for (int i=0;i<nss;i++)
			ch[i] = new std::complex<double>[nss];
		txVector.GetChannelMatrix (ch);
		double a = ch[0][0].real();
		double b = ch[0][0].imag();
		double squareValue = (a*a + b*b);
		realRxPowerW = rxPowerW*squareValue/2; 
		realRxPowerDbm = WToDbm(realRxPowerW);
		for(int j=0; j<nss; j++)
		{
			delete [] ch[j];
		}
		delete [] ch; 
	}
		

	//802.11ac channel bonding
	bool currentWidth[4] = {0,};
  bool receivingTest = false;
  uint16_t bandWidth = 0;
  DuplicateTag dtag;
  bool ctrlFrame = (packet->PeekPacketTag(dtag));
  Time endRx = Simulator::Now () + rxDuration;
  Ptr<InterferenceHelper::Event> event[4];
  //considering only primary channel ocuppancy cases
  //if signal comes only to the secondary channels, then the bandwidth is zero and receivingTest alyways false
  switch(ch)
  {
    case RECV_OCC_20:
      receivingTest = true;
      bandWidth = 20;
      break;
    case NO_RECV_OCC_20_40:
      receivingTest = false;
      if(ctrlFrame)
        receivingTest = true;
      bandWidth = 20;
      break;
    case NO_RECV_OCC_20_80:
      receivingTest = false;
      bandWidth = 20;
      if(ctrlFrame)
        receivingTest = true;
      break;
    case NO_RECV_OCC_40:
      receivingTest = false;
      bandWidth = 40;
      if(ctrlFrame)
        receivingTest = true;
      break;
    case RECV_OCC_40:
      receivingTest = true;
      bandWidth = 40;
      break;
    case RECV_OCC_80:
      receivingTest = true;
      bandWidth = 80;
      break;
    default:
      receivingTest = false;
      break;
  }

	NS_LOG_DEBUG("what is event? " << event[0] << " " << event[1] << " " << event[2] << " " << event[3]);

	switch (m_state[0]->GetState ())
	{
		case YansWifiPhy::SWITCHING:
			switch(ch)
			{
				case RECV_OCC_20:
				case RECV_OCC_40:
				case RECV_OCC_80:
				case NO_RECV_OCC_20_40:
				case NO_RECV_OCC_20_80:
				case NO_RECV_OCC_40:
					event[0] = m_interference[0].Add (packet->GetSize (),
							txMode,
							preamble,
							rxDuration,
							rxPowerW,txVector);
					NS_LOG_DEBUG("primary channel add" << ", ch is " << ch);
					currentWidth[0]=1;
					break;

				default:
					break;
			}

			//shbyeon add secondary20 channel
			switch(ch)
			{
				case RECV_OCC_40:
				case RECV_OCC_80:
				case NO_RECV_OCC_40:
				case NOCC_SECONDARY_20:
					event[1] = m_interference[1].Add (packet->GetSize (),
							txMode,
							preamble,
							rxDuration,
							rxPowerW,txVector);
					NS_LOG_DEBUG("secondary 20 channel add" << ", ch is " << ch);
					currentWidth[1]=1;
					break;

				default:
					break;
			}

			//shbyeon add secondary40 channel
			switch(ch)
			{
				case RECV_OCC_80:
				case NOCC_SECONDARY_40_ALL:
				case NOCC_SECONDARY_40_UP:
					event[2] = m_interference[2].Add (packet->GetSize (),
							txMode,
							preamble,
							rxDuration,
							rxPowerW,txVector);
					NS_LOG_DEBUG("secondary 40 up channel add" << ", ch is " << ch);
					currentWidth[2]=1;
					break;
				default:
					break;
			}

			switch(ch)
			{
				case RECV_OCC_80:
				case NOCC_SECONDARY_40_ALL:
				case NOCC_SECONDARY_40_DOWN:
					event[3] = m_interference[3].Add (packet->GetSize (),
							txMode,
							preamble,
							rxDuration,
							rxPowerW,txVector);
					NS_LOG_DEBUG("secondary 40 down channel add" << ", ch is " << ch);
					currentWidth[3]=1;
					break;

				default:
					break;
			}
			NS_LOG_DEBUG ("drop packet because of channel switching");
			NotifyRxDrop (packet);
			/*
			 * Packets received on the upcoming channel are added to the event list
			 * during the switching state. This way the medium can be correctly sensed
			 * when the device listens to the channel for the first time after the
			 * switching e.g. after channel switching, the channel may be sensed as
			 * busy due to other devices' tramissions started before the end of
			 * the switching.
			 */
			if (endRx > Simulator::Now () + m_state[0]->GetDelayUntilIdle ())
			{
				// that packet will be noise _after_ the completion of the
				// channel switching.
				goto maybeCcaBusy;
			}
			break;
		case YansWifiPhy::RX:
			//11ac: second_capture 
			if (m_secondCaptureCapability && (realRxPowerW >= 10.0*m_prevRxPowerW))
			{
				if(GetMacLow()->RunningAckEvent(rxDuration, packet))
				{
					switch(ch)
					{
						case RECV_OCC_20:
						case RECV_OCC_40:
						case RECV_OCC_80:
						case NO_RECV_OCC_20_40:
						case NO_RECV_OCC_20_80:
						case NO_RECV_OCC_40:
							event[0] = m_interference[0].Add (packet->GetSize (),
									txMode,
									preamble,
									rxDuration,
									rxPowerW,txVector);
							NS_LOG_DEBUG("primary channel add" << ", ch is " << ch);
							currentWidth[0]=1;
							break;

						default:
							break;
					}

					//shbyeon add secondary20 channel
					switch(ch)
					{
						case RECV_OCC_40:
						case RECV_OCC_80:
						case NO_RECV_OCC_40:
						case NOCC_SECONDARY_20:
							event[1] = m_interference[1].Add (packet->GetSize (),
									txMode,
									preamble,
									rxDuration,
									rxPowerW,txVector);
							NS_LOG_DEBUG("secondary 20 channel add" << ", ch is " << ch);
							currentWidth[1]=1;
							break;

						default:
							break;
					}

					//shbyeon add secondary40 channel
					switch(ch)
					{
						case RECV_OCC_80:
						case NOCC_SECONDARY_40_ALL:
						case NOCC_SECONDARY_40_UP:
							event[2] = m_interference[2].Add (packet->GetSize (),
									txMode,
									preamble,
									rxDuration,
									rxPowerW,txVector);
							NS_LOG_DEBUG("secondary 40 up channel add" << ", ch is " << ch);
							currentWidth[2]=1;
							break;
						default:
							break;
					}

					switch(ch)
					{
						case RECV_OCC_80:
						case NOCC_SECONDARY_40_ALL:
						case NOCC_SECONDARY_40_DOWN:
							event[3] = m_interference[3].Add (packet->GetSize (),
									txMode,
									preamble,
									rxDuration,
									rxPowerW,txVector);
							NS_LOG_DEBUG("secondary 40 down channel add" << ", ch is " << ch);
							currentWidth[3]=1;
							break;

						default:
							break;
					}
					NS_LOG_DEBUG("drop packet because ack event is still running");
					NotifyRxDrop(packet);
					goto maybeCcaBusy;
				}

        NS_LOG_DEBUG("rxPower=" << WToDbm(rxPowerW) << " cca=" << GetCcaMode1Threshold());
				if (realRxPowerW > m_ccaMode1ThresholdW) //*bandWidth/20) // 170927 ywson: RxPowerW is already normalized to 20 MHz
				{
					if (IsModeSupported (txMode) || IsMcsSupported(txMode) || IsAcMcsSupported(txMode)) //11ac: vht_standard
					{
					  NS_LOG_DEBUG ("sync to captured signal (power=" <<WToDbm(realRxPowerW)<< 
							">" << WToDbm(m_prevRxPowerW) <<
							"+10). EndReceive="<<Simulator::Now().GetMicroSeconds() + rxDuration.GetMicroSeconds() <<
							"us, PrevEndReceive="<<m_prevEndRx.GetMicroSeconds() << "us");
						// sync to signal
						if(receivingTest)
						{
							m_endRxEvent.Cancel ();
							if(m_prevWidth[0])
								m_interference[0].NotifyRxEnd ();
							if(m_prevWidth[1])
								m_interference[1].NotifyRxEnd ();
							if(m_prevWidth[2] && m_prevWidth[3])
							{
								m_interference[2].NotifyRxEnd ();
								m_interference[3].NotifyRxEnd ();
							}
							switch(ch)
							{
								case RECV_OCC_20:
								case RECV_OCC_40:
								case RECV_OCC_80:
								case NO_RECV_OCC_20_40:
								case NO_RECV_OCC_20_80:
								case NO_RECV_OCC_40:
									event[0] = m_interference[0].Add (packet->GetSize (),
											txMode,
											preamble,
											rxDuration,
											rxPowerW,txVector);
									NS_LOG_DEBUG("primary channel add" << ", ch is " << ch);
									currentWidth[0]=1;
									break;

								default:
									break;
							}

							//shbyeon add secondary20 channel
							switch(ch)
							{
								case RECV_OCC_40:
								case RECV_OCC_80:
								case NO_RECV_OCC_40:
								case NOCC_SECONDARY_20:
									event[1] = m_interference[1].Add (packet->GetSize (),
											txMode,
											preamble,
											rxDuration,
											rxPowerW,txVector);
									NS_LOG_DEBUG("secondary 20 channel add" << ", ch is " << ch);
									currentWidth[1]=1;
									break;

								default:
									break;
							}

							//shbyeon add secondary40 channel
							switch(ch)
							{
								case RECV_OCC_80:
								case NOCC_SECONDARY_40_ALL:
								case NOCC_SECONDARY_40_UP:
									event[2] = m_interference[2].Add (packet->GetSize (),
											txMode,
											preamble,
											rxDuration,
											rxPowerW,txVector);
									NS_LOG_DEBUG("secondary 40 up channel add" << ", ch is " << ch);
									currentWidth[2]=1;
									break;
								default:
									break;
							}

							switch(ch)
							{
								case RECV_OCC_80:
								case NOCC_SECONDARY_40_ALL:
								case NOCC_SECONDARY_40_DOWN:
									event[3] = m_interference[3].Add (packet->GetSize (),
											txMode,
											preamble,
											rxDuration,
											rxPowerW,txVector);
									NS_LOG_DEBUG("secondary 40 down channel add" << ", ch is " << ch);
									currentWidth[3]=1;
									break;

								default:
									break;
							}
							NotifyRxDrop(m_prevPacket);
							if(m_prevBandwidth== 20)
							{
								NS_LOG_DEBUG(m_prevBandwidth << " MHz rx");
								NS_LOG_DEBUG("delete primary 20");

								//160326 skim11 : AMPDU capture
								AmpduTag prevTag;
								bool isPrevAmpdu = m_prevPacket->PeekPacketTag(prevTag);
								if (isPrevAmpdu){
									uint32_t k = 0;
									MpduAggregator::DeaggregatedMpdus packets; 
									packets = MpduAggregator::Deaggregate (m_prevPacket);
									for (MpduAggregator::DeaggregatedMpdusCI i = packets.begin ();
												i != packets.end (); ++i,++k)
									{ 
										AmpduTag tag;
										if(k==(packets.size() - 1)){
											NS_LOG_DEBUG("last aggregated packet -> tag set true");
											(*i).first->RemovePacketTag(tag);
											(*i).first->AddPacketTag(AmpduTag (true));
										}
										NotifyRxDrop ((*i).first);
										m_state[0]->SwitchFromRxEndSc ((*i).first, m_prevSnr, m_prevMode, 0); 		
									}
								 }
								else
									m_state[0]->SwitchFromRxEndSc(m_prevPacket, m_prevSnr, m_prevMode, false);
								NS_LOG_DEBUG("done");
							}
							if(m_prevBandwidth == 40)
							{
								NS_LOG_DEBUG(m_prevBandwidth << " MHz rx");
								NS_LOG_DEBUG("delete primary 20");
								m_state[0]->SwitchFromRxEndSc(m_prevPacket, m_prevSnr, m_prevMode, false);
								NS_LOG_DEBUG("delete secondary 20");
								m_state[1]->SwitchFromRxEndSc(m_prevPacket, m_prevSnr, m_prevMode, false);
								NS_LOG_DEBUG("done");
							}
							if(m_prevBandwidth == 80)
							{
								NS_LOG_DEBUG(m_prevBandwidth << " MHz rx");
								NS_LOG_DEBUG("delete primary 20");
								m_state[0]->SwitchFromRxEndSc(m_prevPacket, m_prevSnr, m_prevMode, false);
								NS_LOG_DEBUG("delete secondary 20");
								m_state[1]->SwitchFromRxEndSc(m_prevPacket, m_prevSnr, m_prevMode, false);
								NS_LOG_DEBUG("delete secondary 40u");
								m_state[2]->SwitchFromRxEndSc(m_prevPacket, m_prevSnr, m_prevMode, false);
								NS_LOG_DEBUG("delete secondary 40d");
								m_state[3]->SwitchFromRxEndSc(m_prevPacket, m_prevSnr, m_prevMode, false);
								NS_LOG_DEBUG("done");
							}


							NotifyRxBegin (packet);

							if(bandWidth == 20)
							{
								NS_LOG_DEBUG(bandWidth << " MHz rx");
								NS_LOG_DEBUG("add primary 20");
								m_state[0]->SwitchToRx (rxDuration);
								NS_LOG_DEBUG("done");
							}
							if(bandWidth == 40)
							{
								NS_LOG_DEBUG(bandWidth << " MHz rx");
								NS_LOG_DEBUG("add primary 20");
								m_state[0]->SwitchToRx (rxDuration);
								NS_LOG_DEBUG("add secondary 20");
								m_state[1]->SwitchToRx (rxDuration);
								NS_LOG_DEBUG("done");
							}
							if(bandWidth == 80)
							{
								NS_LOG_DEBUG(bandWidth << " MHz rx");
								NS_LOG_DEBUG("add primary 20");
								m_state[0]->SwitchToRx (rxDuration);
								NS_LOG_DEBUG("add secondary 20");
								m_state[1]->SwitchToRx (rxDuration);
								NS_LOG_DEBUG("add secondary 40u");
								m_state[2]->SwitchToRx (rxDuration);
								NS_LOG_DEBUG("add secondary 40d");
								m_state[3]->SwitchToRx (rxDuration);
								NS_LOG_DEBUG("done");
							}

							NS_ASSERT (m_endRxEvent.IsExpired ());
							if(currentWidth[0])
								m_interference[0].NotifyRxStart ();
							if(currentWidth[1])
								m_interference[1].NotifyRxStart ();
							if(currentWidth[2] && currentWidth[3])
							{
								m_interference[2].NotifyRxStart ();
								m_interference[3].NotifyRxStart ();
							}
							NS_LOG_DEBUG("used channel " << currentWidth[0] << " " << currentWidth[1] << " " <<
									currentWidth[2] << " " << currentWidth[3] <<", bw=" << bandWidth);


							AmpduTag tag;
							bool isAmpdu = packet->PeekPacketTag(tag);
							if(isAmpdu)
								m_endRxEvent = Simulator::Schedule (rxDuration, &YansWifiPhy::EndAmpduReceive, this,
										packet,
										event[0], event[1], event[2], event[3]);
							else
								m_endRxEvent = Simulator::Schedule (rxDuration, &YansWifiPhy::EndReceive, this,
										packet,
										event[0], event[1], event[2], event[3]);
							//11ac: second_capture 
							m_prevPacket = packet;
							m_prevSnr = m_interference[0].CalculateSnrPer(event[0]).snr;
							m_prevRxPowerW = realRxPowerW; //160413 skim11 : channel bug fix
							m_prevRxpowerDbm = realRxPowerDbm; //160413 skim11
							m_prevEndRx = endRx;
							m_prevMode = txMode;
							m_prevBandwidth = bandWidth;
							for (int i=0;i<4;i++)
								m_prevWidth[i] = currentWidth[i];
						}
						else
						{
							switch(ch)
							{
								case RECV_OCC_20:
								case RECV_OCC_40:
								case RECV_OCC_80:
								case NO_RECV_OCC_20_40:
								case NO_RECV_OCC_20_80:
								case NO_RECV_OCC_40:
									event[0] = m_interference[0].Add (packet->GetSize (),
											txMode,
											preamble,
											rxDuration,
											rxPowerW,txVector);
									NS_LOG_DEBUG("primary channel add" << ", ch is " << ch);
									currentWidth[0]=1;
									break;

								default:
									break;
							}

							//shbyeon add secondary20 channel
							switch(ch)
							{
								case RECV_OCC_40:
								case RECV_OCC_80:
								case NO_RECV_OCC_40:
								case NOCC_SECONDARY_20:
									event[1] = m_interference[1].Add (packet->GetSize (),
											txMode,
											preamble,
											rxDuration,
											rxPowerW,txVector);
									NS_LOG_DEBUG("secondary 20 channel add" << ", ch is " << ch);
									currentWidth[1]=1;
									break;

								default:
									break;
							}

							//shbyeon add secondary40 channel
							switch(ch)
							{
								case RECV_OCC_80:
								case NOCC_SECONDARY_40_ALL:
								case NOCC_SECONDARY_40_UP:
									event[2] = m_interference[2].Add (packet->GetSize (),
											txMode,
											preamble,
											rxDuration,
											rxPowerW,txVector);
									NS_LOG_DEBUG("secondary 40 up channel add" << ", ch is " << ch);
									currentWidth[2]=1;
									break;
								default:
									break;
							}

							switch(ch)
							{
								case RECV_OCC_80:
								case NOCC_SECONDARY_40_ALL:
								case NOCC_SECONDARY_40_DOWN:
									event[3] = m_interference[3].Add (packet->GetSize (),
											txMode,
											preamble,
											rxDuration,
											rxPowerW,txVector);
									NS_LOG_DEBUG("secondary 40 down channel add" << ", ch is " << ch);
									currentWidth[3]=1;
									break;

								default:
									break;
							}
							NS_LOG_DEBUG ("drop packet because it was sent using an channel not for primary channel, rxType=" << ch);
							NotifyRxDrop (packet);
							goto maybeCcaBusy;
						}
					}
					else
					{
						switch(ch)
						{
							case RECV_OCC_20:
							case RECV_OCC_40:
							case RECV_OCC_80:
							case NO_RECV_OCC_20_40:
							case NO_RECV_OCC_20_80:
							case NO_RECV_OCC_40:
								event[0] = m_interference[0].Add (packet->GetSize (),
										txMode,
										preamble,
										rxDuration,
										rxPowerW,txVector);
								NS_LOG_DEBUG("primary channel add" << ", ch is " << ch);
								currentWidth[0]=1;
								break;

							default:
								break;
						}

						//shbyeon add secondary20 channel
						switch(ch)
						{
							case RECV_OCC_40:
							case RECV_OCC_80:
							case NO_RECV_OCC_40:
							case NOCC_SECONDARY_20:
								event[1] = m_interference[1].Add (packet->GetSize (),
										txMode,
										preamble,
										rxDuration,
										rxPowerW,txVector);
								NS_LOG_DEBUG("secondary 20 channel add" << ", ch is " << ch);
								currentWidth[1]=1;
								break;

							default:
								break;
						}

						//shbyeon add secondary40 channel
						switch(ch)
						{
							case RECV_OCC_80:
							case NOCC_SECONDARY_40_ALL:
							case NOCC_SECONDARY_40_UP:
								event[2] = m_interference[2].Add (packet->GetSize (),
										txMode,
										preamble,
										rxDuration,
										rxPowerW,txVector);
								NS_LOG_DEBUG("secondary 40 up channel add" << ", ch is " << ch);
								currentWidth[2]=1;
								break;
							default:
								break;
						}

						switch(ch)
						{
							case RECV_OCC_80:
							case NOCC_SECONDARY_40_ALL:
							case NOCC_SECONDARY_40_DOWN:
								event[3] = m_interference[3].Add (packet->GetSize (),
										txMode,
										preamble,
										rxDuration,
										rxPowerW,txVector);
								NS_LOG_DEBUG("secondary 40 down channel add" << ", ch is " << ch);
								currentWidth[3]=1;
								break;

							default:
								break;
						}
						NS_LOG_DEBUG ("drop packet because it was sent using an unsupported mode (" << txMode << ")");
						NotifyRxDrop (packet);
						goto maybeCcaBusy;
					}
				}
				else
				{
					switch(ch)
					{
						case RECV_OCC_20:
						case RECV_OCC_40:
						case RECV_OCC_80:
						case NO_RECV_OCC_20_40:
						case NO_RECV_OCC_20_80:
						case NO_RECV_OCC_40:
							event[0] = m_interference[0].Add (packet->GetSize (),
									txMode,
									preamble,
									rxDuration,
									rxPowerW,txVector);
							NS_LOG_DEBUG("primary channel add" << ", ch is " << ch);
							currentWidth[0]=1;
							break;

						default:
							break;
					}

					//shbyeon add secondary20 channel
					switch(ch)
					{
						case RECV_OCC_40:
						case RECV_OCC_80:
						case NO_RECV_OCC_40:
						case NOCC_SECONDARY_20:
							event[1] = m_interference[1].Add (packet->GetSize (),
									txMode,
									preamble,
									rxDuration,
									rxPowerW,txVector);
							NS_LOG_DEBUG("secondary 20 channel add" << ", ch is " << ch);
							currentWidth[1]=1;
							break;

						default:
							break;
					}

					//shbyeon add secondary40 channel
					switch(ch)
					{
						case RECV_OCC_80:
						case NOCC_SECONDARY_40_ALL:
						case NOCC_SECONDARY_40_UP:
							event[2] = m_interference[2].Add (packet->GetSize (),
									txMode,
									preamble,
									rxDuration,
									rxPowerW,txVector);
							NS_LOG_DEBUG("secondary 40 up channel add" << ", ch is " << ch);
							currentWidth[2]=1;
							break;
						default:
							break;
					}

					switch(ch)
					{
						case RECV_OCC_80:
						case NOCC_SECONDARY_40_ALL:
						case NOCC_SECONDARY_40_DOWN:
							event[3] = m_interference[3].Add (packet->GetSize (),
									txMode,
									preamble,
									rxDuration,
									rxPowerW,txVector);
							NS_LOG_DEBUG("secondary 40 down channel add" << ", ch is " << ch);
							currentWidth[3]=1;
							break;

						default:
							break;
					}
					NS_LOG_DEBUG ("Already RX: drop packet because signal power too Small (" <<
							realRxPowerDbm << "<" << WToDbm(m_ccaMode1ThresholdW) << ")");
					NotifyRxDrop (packet);
					goto maybeCcaBusy;
				}
			}
			else
			{
				switch(ch)
				{
					case RECV_OCC_20:
					case RECV_OCC_40:
					case RECV_OCC_80:
					case NO_RECV_OCC_20_40:
					case NO_RECV_OCC_20_80:
					case NO_RECV_OCC_40:
						event[0] = m_interference[0].Add (packet->GetSize (),
								txMode,
								preamble,
								rxDuration,
								rxPowerW,txVector);
						NS_LOG_DEBUG("primary channel add" << ", ch is " << ch);
						currentWidth[0]=1;
						break;

					default:
						break;
				}

				//shbyeon add secondary20 channel
				switch(ch)
				{
					case RECV_OCC_40:
					case RECV_OCC_80:
					case NO_RECV_OCC_40:
					case NOCC_SECONDARY_20:
						event[1] = m_interference[1].Add (packet->GetSize (),
								txMode,
								preamble,
								rxDuration,
								rxPowerW,txVector);
						NS_LOG_DEBUG("secondary 20 channel add" << ", ch is " << ch);
						currentWidth[1]=1;
						break;

					default:
						break;
				}

				//shbyeon add secondary40 channel
				switch(ch)
				{
					case RECV_OCC_80:
					case NOCC_SECONDARY_40_ALL:
					case NOCC_SECONDARY_40_UP:
						event[2] = m_interference[2].Add (packet->GetSize (),
								txMode,
								preamble,
								rxDuration,
								rxPowerW,txVector);
						NS_LOG_DEBUG("secondary 40 up channel add" << ", ch is " << ch);
						currentWidth[2]=1;
						break;
					default:
						break;
				}

				switch(ch)
				{
					case RECV_OCC_80:
					case NOCC_SECONDARY_40_ALL:
					case NOCC_SECONDARY_40_DOWN:
						event[3] = m_interference[3].Add (packet->GetSize (),
								txMode,
								preamble,
								rxDuration,
								rxPowerW,txVector);
						NS_LOG_DEBUG("secondary 40 down channel add" << ", ch is " << ch);
						currentWidth[3]=1;
						break;

					default:
						break;
				}

				NS_LOG_DEBUG ("drop packet because already in Rx (power=" <<	WToDbm(realRxPowerW) << 
					"<"  << WToDbm(m_prevRxPowerW) <<
					"+10). PrevEndReceive=" << m_prevEndRx.GetMicroSeconds() << "us");
				NotifyRxDrop (packet);
				if (endRx > Simulator::Now () + m_state[0]->GetDelayUntilIdle ())
				{
					// that packet will be noise _after_ the reception of the
					// currently-received packet.
					goto maybeCcaBusy;
				}
			}
			break;
		case YansWifiPhy::TX:
			switch(ch)
			{
				case RECV_OCC_20:
				case RECV_OCC_40:
				case RECV_OCC_80:
				case NO_RECV_OCC_20_40:
				case NO_RECV_OCC_20_80:
				case NO_RECV_OCC_40:
					event[0] = m_interference[0].Add (packet->GetSize (),
							txMode,
							preamble,
							rxDuration,
							rxPowerW,txVector);
					NS_LOG_DEBUG("primary channel add" << ", ch is " << ch);
					currentWidth[0]=1;
					break;

				default:
					break;
			}

			//shbyeon add secondary20 channel
			switch(ch)
			{
				case RECV_OCC_40:
				case RECV_OCC_80:
				case NO_RECV_OCC_40:
				case NOCC_SECONDARY_20:
					event[1] = m_interference[1].Add (packet->GetSize (),
							txMode,
							preamble,
							rxDuration,
							rxPowerW,txVector);
					NS_LOG_DEBUG("secondary 20 channel add" << ", ch is " << ch);
					currentWidth[1]=1;
					break;

				default:
					break;
			}

			//shbyeon add secondary40 channel
			switch(ch)
			{
				case RECV_OCC_80:
				case NOCC_SECONDARY_40_ALL:
				case NOCC_SECONDARY_40_UP:
					event[2] = m_interference[2].Add (packet->GetSize (),
							txMode,
							preamble,
							rxDuration,
							rxPowerW,txVector);
					NS_LOG_DEBUG("secondary 40 up channel add" << ", ch is " << ch);
					currentWidth[2]=1;
					break;
				default:
					break;
			}

			switch(ch)
			{
				case RECV_OCC_80:
				case NOCC_SECONDARY_40_ALL:
				case NOCC_SECONDARY_40_DOWN:
					event[3] = m_interference[3].Add (packet->GetSize (),
							txMode,
							preamble,
							rxDuration,
							rxPowerW,txVector);
					NS_LOG_DEBUG("secondary 40 down channel add" << ", ch is " << ch);
					currentWidth[3]=1;
					break;

				default:
					break;
			}
			NS_LOG_DEBUG ("drop packet because already in Tx (power=" <<WToDbm(realRxPowerW) << ")" );
			NotifyRxDrop (packet);
			if (endRx > Simulator::Now () + m_state[0]->GetDelayUntilIdle ())
			{
				// that packet will be noise _after_ the transmission of the
				// currently-transmitted packet.
				goto maybeCcaBusy;
			}
			break;
		case YansWifiPhy::CCA_BUSY:
		case YansWifiPhy::IDLE:
			switch(ch)
			{
				case RECV_OCC_20:
				case RECV_OCC_40:
				case RECV_OCC_80:
				case NO_RECV_OCC_20_40:
				case NO_RECV_OCC_20_80:
				case NO_RECV_OCC_40:
					event[0] = m_interference[0].Add (packet->GetSize (),
							txMode,
							preamble,
							rxDuration,
							rxPowerW,txVector);
					NS_LOG_DEBUG("primary channel add" << ", ch is " << ch);
					currentWidth[0]=1;
					break;

				default:
					break;
			}

			//shbyeon add secondary20 channel
			switch(ch)
			{
				case RECV_OCC_40:
				case RECV_OCC_80:
				case NO_RECV_OCC_40:
				case NOCC_SECONDARY_20:
					event[1] = m_interference[1].Add (packet->GetSize (),
							txMode,
							preamble,
							rxDuration,
							rxPowerW,txVector);
					NS_LOG_DEBUG("secondary 20 channel add" << ", ch is " << ch);
					currentWidth[1]=1;
					break;

				default:
					break;
			}

			//shbyeon add secondary40 channel
			switch(ch)
			{
				case RECV_OCC_80:
				case NOCC_SECONDARY_40_ALL:
				case NOCC_SECONDARY_40_UP:
					event[2] = m_interference[2].Add (packet->GetSize (),
							txMode,
							preamble,
							rxDuration,
							rxPowerW,txVector);
					NS_LOG_DEBUG("secondary 40 up channel add" << ", ch is " << ch);
					currentWidth[2]=1;
					break;
				default:
					break;
			}

			switch(ch)
			{
				case RECV_OCC_80:
				case NOCC_SECONDARY_40_ALL:
				case NOCC_SECONDARY_40_DOWN:
					event[3] = m_interference[3].Add (packet->GetSize (),
							txMode,
							preamble,
							rxDuration,
							rxPowerW,txVector);
					NS_LOG_DEBUG("secondary 40 down channel add" << ", ch is " << ch);
					currentWidth[3]=1;
					break;

				default:
					break;
			}
			//802.11ac channel bonding: receiving data during ack event
			if(GetMacLow()->RunningAckEvent(rxDuration, packet))
			{
				NS_LOG_DEBUG("drop packet because ack event is still running");
				NotifyRxDrop(packet);
				goto maybeCcaBusy;
			}

      NS_LOG_DEBUG("rxPower=" << WToDbm(realRxPowerW) << " cca=" << GetCcaMode1Threshold());
			if (realRxPowerW > m_ccaMode1ThresholdW) //*bandWidth/20) // 180927 ywson: RxPower is already normalized to 20 MHz
			{
				if (IsModeSupported (txMode) || IsMcsSupported(txMode) || IsAcMcsSupported(txMode)) //11ac: vht_standard
				{
					NS_LOG_DEBUG ("sync to signal (power=" << WToDbm(realRxPowerW) << "), EndReceive at "<<Simulator::Now().GetMicroSeconds()+ rxDuration.GetMicroSeconds()<< "us");
					// sync to signal
					if(receivingTest)
					{
						NotifyRxBegin (packet);

						if(bandWidth == 20)
						{
							NS_LOG_DEBUG(bandWidth << " MHz rx");
							NS_LOG_DEBUG("add primary 20");
							m_state[0]->SwitchToRx (rxDuration);
							NS_LOG_DEBUG("done");
						}
						if(bandWidth == 40)
						{
							NS_LOG_DEBUG(bandWidth << " MHz rx");
							NS_LOG_DEBUG("add primary 20");
							m_state[0]->SwitchToRx (rxDuration);
							NS_LOG_DEBUG("add secondary 20");
							m_state[1]->SwitchToRx (rxDuration);
							NS_LOG_DEBUG("done");
						}
						if(bandWidth == 80)
						{
							NS_LOG_DEBUG(bandWidth << " MHz rx");
							NS_LOG_DEBUG("add primary 20");
							m_state[0]->SwitchToRx (rxDuration);
							NS_LOG_DEBUG("add secondary 20");
							m_state[1]->SwitchToRx (rxDuration);
							NS_LOG_DEBUG("add secondary 40u");
							m_state[2]->SwitchToRx (rxDuration);
							NS_LOG_DEBUG("add secondary 40d");
							m_state[3]->SwitchToRx (rxDuration);
							NS_LOG_DEBUG("done");
						}

						NS_ASSERT (m_endRxEvent.IsExpired ());
						if(currentWidth[0])
							m_interference[0].NotifyRxStart ();
						if(currentWidth[1])
							m_interference[1].NotifyRxStart ();
						if(currentWidth[2])
						{
							m_interference[2].NotifyRxStart ();
							m_interference[3].NotifyRxStart ();
						}
						NS_LOG_DEBUG("used channel " << currentWidth[0] << " " << currentWidth[1] << " " <<
								currentWidth[2] << " " << currentWidth[3] <<", bw=" << bandWidth);


						AmpduTag tag;
						bool isAmpdu = packet->PeekPacketTag(tag);
						if(isAmpdu)
							m_endRxEvent = Simulator::Schedule (rxDuration, &YansWifiPhy::EndAmpduReceive, this,
									packet,
									event[0], event[1], event[2], event[3]);
						else
							m_endRxEvent = Simulator::Schedule (rxDuration, &YansWifiPhy::EndReceive, this,
									packet,
									event[0], event[1], event[2], event[3]);
						//11ac: second_capture (shbyeon bug fix) 
						m_prevPacket = packet->Copy();
						m_prevSnr = m_interference[0].CalculateSnrPer(event[0]).snr;
						m_prevRxPowerW = realRxPowerW; //160413 skim11 : channel bug fix
						m_prevRxpowerDbm = realRxPowerDbm;//160413 skim11
						m_prevEndRx = endRx;
						m_prevMode = txMode;
						m_prevBandwidth = bandWidth;
						for (int i=0;i<4;i++)
							m_prevWidth[i] = currentWidth[i];
					}
					else
					{
						NS_LOG_DEBUG ("drop packet because it was sent using a channel not for primary channel, rxType=" << ch);
						NotifyRxDrop (packet);
						goto maybeCcaBusy;
					}
				}
				else
				{
					NS_LOG_DEBUG ("drop packet because it was sent using an unsupported mode (" << txMode << ")");
					NotifyRxDrop (packet);
					goto maybeCcaBusy;
				}
			}
			else
			{
				NS_LOG_DEBUG ("IDLE: drop packet because signal power too Small (" <<
						WToDbm(realRxPowerW) << "<" << WToDbm(m_ccaMode1ThresholdW) << ")");
				NotifyRxDrop (packet);
				goto maybeCcaBusy;
			}

			break;
		case YansWifiPhy::SLEEP:
			switch(ch)
			{
				case RECV_OCC_20:
				case RECV_OCC_40:
				case RECV_OCC_80:
				case NO_RECV_OCC_20_40:
				case NO_RECV_OCC_20_80:
				case NO_RECV_OCC_40:
					event[0] = m_interference[0].Add (packet->GetSize (),
							txMode,
							preamble,
							rxDuration,
							rxPowerW,txVector);
					NS_LOG_DEBUG("primary channel add" << ", ch is " << ch);
					currentWidth[0]=1;
					break;

				default:
					break;
			}

			//shbyeon add secondary20 channel
			switch(ch)
			{
				case RECV_OCC_40:
				case RECV_OCC_80:
				case NO_RECV_OCC_40:
				case NOCC_SECONDARY_20:
					event[1] = m_interference[1].Add (packet->GetSize (),
							txMode,
							preamble,
							rxDuration,
							rxPowerW,txVector);
					NS_LOG_DEBUG("secondary 20 channel add" << ", ch is " << ch);
					currentWidth[1]=1;
					break;

				default:
					break;
			}

			//shbyeon add secondary40 channel
			switch(ch)
			{
				case RECV_OCC_80:
				case NOCC_SECONDARY_40_ALL:
				case NOCC_SECONDARY_40_UP:
					event[2] = m_interference[2].Add (packet->GetSize (),
							txMode,
							preamble,
							rxDuration,
							rxPowerW,txVector);
					NS_LOG_DEBUG("secondary 40 up channel add" << ", ch is " << ch);
					currentWidth[2]=1;
					break;
				default:
					break;
			}

			switch(ch)
			{
				case RECV_OCC_80:
				case NOCC_SECONDARY_40_ALL:
				case NOCC_SECONDARY_40_DOWN:
					event[3] = m_interference[3].Add (packet->GetSize (),
							txMode,
							preamble,
							rxDuration,
							rxPowerW,txVector);
					NS_LOG_DEBUG("secondary 40 down channel add" << ", ch is " << ch);
					currentWidth[3]=1;
					break;

				default:
					break;
			}
			NS_LOG_DEBUG ("drop packet because in sleep mode");
			NotifyRxDrop (packet);
			break;
	}

	return;

maybeCcaBusy:
  // We are here because we have received the first bit of a packet and we are
  // not going to be able to synchronize on it
  // In this model, CCA becomes busy when the aggregation of all signals as
  // tracked by the InterferenceHelper class is higher than the CcaBusyThreshold
  
	double detectionThresholdW = FindBusyThreshold (ch, bandWidth);

	//802.11ac channel bonding
	Time delayUntilCcaEnd;
  switch (ch)
  {
    case RECV_OCC_20:
    case RECV_OCC_40:
    case RECV_OCC_80:
    case NO_RECV_OCC_20_40:
    case NO_RECV_OCC_20_80:
    case NO_RECV_OCC_40:
      delayUntilCcaEnd = m_interference[0].GetEnergyDuration (detectionThresholdW);
      if (!delayUntilCcaEnd.IsZero ())
      {
        m_state[0]->SwitchMaybeToCcaBusy (delayUntilCcaEnd);
        NS_LOG_DEBUG("primary maybecca busy, "<< delayUntilCcaEnd);
      }
      break;
    default:
      break;
  }

  switch(ch)
  {
    case RECV_OCC_40:
    case RECV_OCC_80:
    case NO_RECV_OCC_40:
    case NOCC_SECONDARY_20:
      delayUntilCcaEnd = m_interference[1].GetEnergyDuration (detectionThresholdW);
      if (!delayUntilCcaEnd.IsZero ())
      {
        m_state[1]->SwitchMaybeToCcaBusy (delayUntilCcaEnd);
        NS_LOG_DEBUG("secondary20 maybecca busy, "<< delayUntilCcaEnd);
      }
      break;
    default:
      break;
  }
  
  switch(ch)
  {
    case RECV_OCC_80:
    case NOCC_SECONDARY_40_ALL:
    case NOCC_SECONDARY_40_UP:
      delayUntilCcaEnd = m_interference[2].GetEnergyDuration (detectionThresholdW);
      if (!delayUntilCcaEnd.IsZero ())
      {
        m_state[2]->SwitchMaybeToCcaBusy (delayUntilCcaEnd);
        NS_LOG_DEBUG("secondary40_up maybecca busy, "<< delayUntilCcaEnd);
      }
      break;
    default:
      break;
  }
  
  switch(ch)
  {
    case RECV_OCC_80:
    case NOCC_SECONDARY_40_ALL:
    case NOCC_SECONDARY_40_DOWN:
      delayUntilCcaEnd = m_interference[3].GetEnergyDuration (detectionThresholdW);
      if (!delayUntilCcaEnd.IsZero ())
      {
        m_state[3]->SwitchMaybeToCcaBusy (delayUntilCcaEnd);
        NS_LOG_DEBUG("secondary40_down maybecca busy, "<< delayUntilCcaEnd);
      }

    default:
      break;
  }
    
}

//802.11ac channel bonding
double 
YansWifiPhy::FindBusyThreshold (enum ChannelBonding ch, uint16_t bw) const
{
  double ccaTh;
  switch (ch)
  {
    case RECV_OCC_20:
    case RECV_OCC_40:
    case RECV_OCC_80:
        ccaTh=m_edThresholdW*10; //-62 dBm
        break;
    default:
        ccaTh=m_edThresholdW; //-72 dBm
        break;
  }
  
  ccaTh=((double)bw/20)*ccaTh; //bandwidth scaling
  return ccaTh;
}

void
YansWifiPhy::SendPacket (Ptr<const Packet> packet, WifiTxVector txVector, WifiPreamble preamble, uint16_t currentWidth)
{
  NS_LOG_DEBUG (this << packet << txVector.GetMode() << preamble << (uint32_t)txVector.GetTxPowerLevel());
	SetCurrentWidth(currentWidth);
  /* Transmission can happen if:
   *  - we are syncing on a packet. It is the responsability of the
   *    MAC layer to avoid doing this but the PHY does nothing to
   *    prevent it.
   *  - we are idle
   */
  
	NS_ASSERT (!m_state[0]->IsStateTx ());
	NS_ASSERT (!m_state[0]->IsStateSwitching ());
  NS_ASSERT (!m_state[0]->IsStateTx () && !m_state[0]->IsStateSwitching ());

  if (m_state[0]->IsStateSleep ())
    {
      NS_LOG_DEBUG ("Dropping packet because in sleep mode");
      NotifyTxDrop (packet);
      return;
    }

  Time txDuration = CalculateTxDuration (packet->GetSize (), txVector, preamble);
  if (m_state[0]->IsStateRx () || m_state[1]->IsStateRx () || m_state[2]->IsStateRx () || m_state[3]->IsStateRx ())
    {
			m_endRxEvent.Cancel ();
			m_interference[0].NotifyRxEnd ();
			m_interference[1].NotifyRxEnd ();
			m_interference[2].NotifyRxEnd ();
			m_interference[3].NotifyRxEnd ();
		}
  NotifyTxBegin (packet);
	uint32_t dataRate500KbpsUnits = txVector.GetMode().GetDataRate () * txVector.GetNss() / 500000;
	bool isShortPreamble = (WIFI_PREAMBLE_SHORT == preamble);
	NotifyMonitorSniffTx (packet, (uint16_t)GetChannelFrequencyMhz (), GetChannelNumber (), dataRate500KbpsUnits, isShortPreamble, txVector.GetTxPowerLevel());

	//802.11ac channel bonding: bonding txpower loss per 20 MHz bw
	int bondingLoss = 0;
	switch(currentWidth)
	{
		case 20:
			bondingLoss = 0;
			break;
		case 40:
			bondingLoss = 3;
			break;
		case 80:
			bondingLoss = 6;
			break;
		default:
			NS_LOG_DEBUG("wrong bandwidth!");
			NS_ASSERT(false);
			break;
	}

	m_state[0]->SwitchToTx (txDuration, packet, GetPowerDbm (txVector.GetTxPowerLevel()), txVector, preamble);
	if(currentWidth==40)
		m_state[1]->SwitchToTx (txDuration, packet, GetPowerDbm (txVector.GetTxPowerLevel()), txVector, preamble);
	else if (currentWidth==80)
	{
		m_state[1]->SwitchToTx (txDuration, packet, GetPowerDbm (txVector.GetTxPowerLevel()), txVector, preamble);

		m_state[2]->SwitchToTx (txDuration, packet, GetPowerDbm (txVector.GetTxPowerLevel()), txVector, preamble);
		m_state[3]->SwitchToTx (txDuration, packet, GetPowerDbm (txVector.GetTxPowerLevel()), txVector, preamble);
	}
  
	m_channel->Send (this, packet, GetPowerDbm ( txVector.GetTxPowerLevel()) + m_txGainDb - bondingLoss, txVector, preamble);
}

uint32_t
YansWifiPhy::GetNModes (void) const
{
  return m_deviceRateSet.size ();
}
WifiMode
YansWifiPhy::GetMode (uint32_t mode) const
{
  return m_deviceRateSet[mode];
}
bool
YansWifiPhy::IsModeSupported (WifiMode mode) const
{
  for (uint32_t i = 0; i < GetNModes (); i++)
    {
      if (mode == GetMode (i))
        {
          return true;
        }
    }
  return false;
}
bool
YansWifiPhy::IsMcsSupported (WifiMode mode)
{
  for (uint32_t i = 0; i < GetNMcs (); i++)
    {
      if (mode == McsToWifiMode(GetMcs (i)))
        {
          return true;
        }
    }
  return false;
}
//11ac: vht_standard
bool
YansWifiPhy::IsAcMcsSupported (WifiMode mode)
{
  for (uint32_t i = 0; i < GetNMcs (); i++)
    {
      if (mode == AcMcsToWifiMode(GetMcs (i), 20) || mode == AcMcsToWifiMode(GetMcs (i), 40)||mode == AcMcsToWifiMode(GetMcs (i), 80))
        {
          return true;
        }
    }
  return false;
}

uint32_t
YansWifiPhy::GetNTxPower (void) const
{
  return m_nTxPower;
}

void
YansWifiPhy::Configure80211a (void)
{
  NS_LOG_FUNCTION (this);
  m_channelStartingFrequency = 5e3; // 5.000 GHz

  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate6Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate9Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate12Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate18Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate24Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate36Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate48Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate54Mbps ());
}


void
YansWifiPhy::Configure80211b (void)
{
  NS_LOG_FUNCTION (this);
  m_channelStartingFrequency = 2407; // 2.407 GHz

  m_deviceRateSet.push_back (WifiPhy::GetDsssRate1Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetDsssRate2Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetDsssRate5_5Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetDsssRate11Mbps ());
}

void
YansWifiPhy::Configure80211g (void)
{
  NS_LOG_FUNCTION (this);
  m_channelStartingFrequency = 2407; // 2.407 GHz

  m_deviceRateSet.push_back (WifiPhy::GetDsssRate1Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetDsssRate2Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetDsssRate5_5Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetErpOfdmRate6Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetErpOfdmRate9Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetDsssRate11Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetErpOfdmRate12Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetErpOfdmRate18Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetErpOfdmRate24Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetErpOfdmRate36Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetErpOfdmRate48Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetErpOfdmRate54Mbps ());
}

void
YansWifiPhy::Configure80211_10Mhz (void)
{
  NS_LOG_FUNCTION (this);
  m_channelStartingFrequency = 5e3; // 5.000 GHz, suppose 802.11a

  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate3MbpsBW10MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate4_5MbpsBW10MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate6MbpsBW10MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate9MbpsBW10MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate12MbpsBW10MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate18MbpsBW10MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate24MbpsBW10MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate27MbpsBW10MHz ());
}

void
YansWifiPhy::Configure80211_5Mhz (void)
{
  NS_LOG_FUNCTION (this);
  m_channelStartingFrequency = 5e3; // 5.000 GHz, suppose 802.11a

  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate1_5MbpsBW5MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate2_25MbpsBW5MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate3MbpsBW5MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate4_5MbpsBW5MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate6MbpsBW5MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate9MbpsBW5MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate12MbpsBW5MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate13_5MbpsBW5MHz ());
}

void
YansWifiPhy::ConfigureHolland (void)
{
  NS_LOG_FUNCTION (this);
  m_channelStartingFrequency = 5e3; // 5.000 GHz
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate6Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate12Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate18Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate36Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate54Mbps ());
}

void
YansWifiPhy::RegisterListener (WifiPhyListener *listener)
{
  m_state[0]->RegisterListener (listener);
}

bool
YansWifiPhy::IsStateCcaBusy (void)
{
  return m_state[0]->IsStateCcaBusy ();
}

bool
YansWifiPhy::IsStateIdle (void)
{
  return m_state[0]->IsStateIdle ();
}
bool
YansWifiPhy::IsStateBusy (void)
{
  return m_state[0]->IsStateBusy ();
}
bool
YansWifiPhy::IsStateRx (void)
{
  return m_state[0]->IsStateRx ();
}
bool
YansWifiPhy::IsStateTx (void)
{
  return m_state[0]->IsStateTx ();
}
bool
YansWifiPhy::IsStateSwitching (void)
{
  return m_state[0]->IsStateSwitching ();
}
bool
YansWifiPhy::IsStateSleep (void)
{
  return m_state[0]->IsStateSleep ();
}

Time
YansWifiPhy::GetStateDuration (void)
{
  return m_state[0]->GetStateDuration ();
}
Time
YansWifiPhy::GetDelayUntilIdle (void)
{
  return m_state[0]->GetDelayUntilIdle ();
}

Time
YansWifiPhy::GetLastRxStartTime (void) const
{
  return m_state[0]->GetLastRxStartTime ();
}

double
YansWifiPhy::DbToRatio (double dB) const
{
  double ratio = std::pow (10.0, dB / 10.0);
  return ratio;
}

double
YansWifiPhy::DbmToW (double dBm) const
{
  double mW = std::pow (10.0, dBm / 10.0);
  return mW / 1000.0;
}

double
YansWifiPhy::WToDbm (double w) const
{
  return 10.0 * std::log10 (w * 1000.0);
}

double
YansWifiPhy::RatioToDb (double ratio) const
{
  return 10.0 * std::log10 (ratio);
}

double
YansWifiPhy::GetEdThresholdW (void) const
{
  return m_edThresholdW;
}

double
YansWifiPhy::GetPowerDbm (uint8_t power) const
{
  NS_ASSERT (m_txPowerBaseDbm <= m_txPowerEndDbm);
  NS_ASSERT (m_nTxPower > 0);
  double dbm;
  if (m_nTxPower > 1)
    {
      dbm = m_txPowerBaseDbm + power * (m_txPowerEndDbm - m_txPowerBaseDbm) / (m_nTxPower - 1);
    }
  else
    {
      NS_ASSERT_MSG (m_txPowerBaseDbm == m_txPowerEndDbm, "cannot have TxPowerEnd != TxPowerStart with TxPowerLevels == 1");
      dbm = m_txPowerBaseDbm;
    }
  return dbm;
}

//802.11ac channel bonding
void
YansWifiPhy::EndReceive (Ptr<Packet> packet, Ptr<InterferenceHelper::Event> event0,
		Ptr<InterferenceHelper::Event> event1, Ptr<InterferenceHelper::Event> event2,
		Ptr<InterferenceHelper::Event> event3)
{
  NS_LOG_DEBUG("endreceive called");
  NS_LOG_FUNCTION (this << packet << event0 << event1 << event2 << event3);
  NS_ASSERT (IsStateRx ());
  NS_ASSERT (event0->GetEndTime () == Simulator::Now ());

  struct InterferenceHelper::SnrPer snrPer;
	struct InterferenceHelper::SnrPer snrPer0;
	struct InterferenceHelper::SnrPer snrPer1;
	struct InterferenceHelper::SnrPer snrPer2;
	struct InterferenceHelper::SnrPer snrPer3;
	if(event0)
		snrPer0 = m_interference[0].CalculateSnrPer (event0);
	else
		NS_ASSERT_MSG (false, "primary channel rx event should be called!");

	if(event1)
		snrPer1 = m_interference[1].CalculateSnrPer (event1);
	if(event2)
		snrPer2 = m_interference[2].CalculateSnrPer (event2);
	if(event3)
		snrPer3 = m_interference[3].CalculateSnrPer (event3);

	//shbyeon duplicate tx/rx protocol
  DuplicateTag dtag;
  bool duplicate = packet->RemovePacketTag(dtag);
  if(!duplicate)
  {
    uint16_t division = !!(event0) + !!(event1) + !!(event2) + !!(event3); 

    switch(division)
    {
      case 1:
        snrPer.snr = snrPer0.snr;
        snrPer.per = snrPer0.per;
        NS_LOG_DEBUG("primary=" << snrPer0.per);
        NS_LOG_DEBUG("primary=" << 10*log10(snrPer0.snr));
        break;
      case 2:
        snrPer.snr = (snrPer0.snr+snrPer1.snr)/2;
        snrPer.per = 1-(1-snrPer0.per)*(1-snrPer1.per);
        NS_LOG_DEBUG("primary=" << snrPer0.per << ", s20=" << snrPer1.per);
        NS_LOG_DEBUG("primary=" << 10*log10(snrPer0.snr) << ", s20=" << 10*log10(snrPer1.snr));
        break;
      case 4:
        snrPer.snr = (snrPer0.snr+snrPer1.snr+snrPer2.snr+snrPer3.snr)/4;
        snrPer.per = 1-(1-snrPer0.per)*(1-snrPer1.per)*(1-snrPer2.per)*(1-snrPer3.per);
        NS_LOG_DEBUG("primary=" << snrPer0.per << ", s20=" << snrPer1.per << ", s40u=" << snrPer2.per << ", s40d=" << snrPer3.per);
        NS_LOG_DEBUG("primary=" << 10*log10(snrPer0.snr) << ", s20=" << 10*log10(snrPer1.snr) << ", s40u=" << 10*log10(snrPer2.snr) << ", s40d=" << 10*log10(snrPer3.snr));
        break;
      default:
        NS_ASSERT_MSG (false, "bandwidth is wrong");
        break;
    }
    NS_LOG_DEBUG("ESNR=" << 10*log10(snrPer.snr) << " EPER=" << snrPer.per << " bw=" << division);

    m_interference[0].NotifyRxEnd ();
    if(event1)
      m_interference[1].NotifyRxEnd ();
    if(event2 || event3)
    {
      m_interference[2].NotifyRxEnd ();
      m_interference[3].NotifyRxEnd ();
    }

    NS_LOG_DEBUG ("mode=" << (event0->GetPayloadMode ().GetDataRate ()) <<
        ", snr=" << 10*log10(snrPer.snr) << ", per=" << snrPer.per << ", size=" << packet->GetSize ());
    ErrorFreeTag tag;
    packet->RemovePacketTag (tag);
    bool errorFree = tag.Get ();
    if (m_random->GetValue () > snrPer.per || errorFree)
    {
      NotifyRxEnd (packet);
      uint32_t dataRate500KbpsUnits = event0->GetPayloadMode ().GetDataRate () / 500000;
      bool isShortPreamble = (WIFI_PREAMBLE_SHORT == event0->GetPreambleType ());
      double signalDbm = RatioToDb (event0->GetRxPowerW ()) + 30;
      double noiseDbm = RatioToDb (event0->GetRxPowerW () / snrPer.snr) - GetRxNoiseFigure () + 30;
      NotifyMonitorSniffRx (packet, (uint16_t)GetChannelFrequencyMhz (), GetChannelNumber (), dataRate500KbpsUnits, isShortPreamble, signalDbm, noiseDbm);
      m_state[0]->SwitchFromRxEndOk (packet, snrPer.snr, event0->GetPayloadMode (), event0->GetPreambleType ());
      if(division>1)
        m_state[1]->SwitchFromRxEndOk (packet, snrPer.snr, event1->GetPayloadMode (), event1->GetPreambleType ());
      if(division>2)
      {
        m_state[2]->SwitchFromRxEndOk (packet, snrPer.snr, event2->GetPayloadMode (), event2->GetPreambleType ());
        m_state[3]->SwitchFromRxEndOk (packet, snrPer.snr, event3->GetPayloadMode (), event3->GetPreambleType ());
      }
    }
    else
    {
      /* failure. */
      NotifyRxDrop (packet);
      m_state[0]->SwitchFromRxEndError (packet, snrPer.snr, event0->GetPayloadMode (), false);
      if(division>1)
        m_state[1]->SwitchFromRxEndError (packet, snrPer.snr, event1->GetPayloadMode (), false);
      if(division>2)
      {
        m_state[2]->SwitchFromRxEndError (packet, snrPer.snr, event2->GetPayloadMode (), false);
        m_state[3]->SwitchFromRxEndError (packet, snrPer.snr, event3->GetPayloadMode (), false);
      }
    }
  }
  else
  {
    uint16_t division = !!(event0) + !!(event1) + !!(event2) + !!(event3); 
    uint16_t txBandwidth = dtag.GetDuplicateBandwidth();
    uint16_t rxBandwidth = 0;
    PacketType ptype = (PacketType) dtag.GetPacketType();
    bool txResult[4] = {0, };
    bool final_result;
    snrPer.snr = snrPer0.snr;
    snrPer.per = snrPer0.per;
    
    if(division == 1)
    {
      txResult[0]=!!(m_random->GetValue() > snrPer0.per);
      if(txResult[0])
        rxBandwidth = 20;
      else
        rxBandwidth = 0;
    }
    else if(division == 2)
    {
      txResult[0]=!!(m_random->GetValue() > snrPer0.per);
      txResult[1]=!!(m_random->GetValue() > snrPer1.per);
      if(!txResult[0])
        rxBandwidth = 0;
      else if(txResult[1])
        rxBandwidth = 40;
      else
        rxBandwidth = 20;
    }
    else if (division == 4)
    {
      txResult[0]=!!(m_random->GetValue() > snrPer0.per);
      txResult[1]=!!(m_random->GetValue() > snrPer1.per);
      txResult[2]=!!(m_random->GetValue() > snrPer2.per);
      txResult[3]=!!(m_random->GetValue() > snrPer3.per);
      if(!txResult[0])
        rxBandwidth = 0;
      else if(!txResult[1])
        rxBandwidth = 20;
      else if(txResult[2] && txResult[3])
        rxBandwidth = 80;
      else
        rxBandwidth = 40;
    }
    else
    {
      NS_ASSERT_MSG (false, "sending bandwidth is wrong!");
    }
    DuplicateTag newDtag(txBandwidth, rxBandwidth, ptype);
    packet->AddPacketTag(newDtag);
    if(dtag.GetPacketType() == ACK || dtag.GetPacketType() == BACK)
      final_result = txResult[0] || txResult[1] || txResult[2] || txResult[3];
    else
      final_result = txResult[0];
    NS_LOG_DEBUG("type=" << (PacketType) dtag.GetPacketType() << " txBW=" << txBandwidth << " rxBW=" << rxBandwidth <<
                 " result=" <<  txResult[0] << txResult[1] << txResult[2] << txResult[3] << " final=" <<final_result);
    NS_LOG_DEBUG("ESNR=" << 10*log10(snrPer.snr) << " EPER=" << snrPer.per << " bw=" << division);

    m_interference[0].NotifyRxEnd ();
    if(event1)
      m_interference[1].NotifyRxEnd ();
    if(event2 || event3)
    {
      m_interference[2].NotifyRxEnd ();
      m_interference[3].NotifyRxEnd ();
    }

    NS_LOG_DEBUG ("mode=" << (event0->GetPayloadMode ().GetDataRate ()) <<
        ", snr=" << 10*log10(snrPer.snr) << ", per=" << snrPer.per << ", size=" << packet->GetSize ());
 		ErrorFreeTag tag;
    packet->RemovePacketTag (tag);
    bool errorFree = tag.Get ();
    if (final_result || errorFree)
    {
      NotifyRxEnd (packet);
      uint32_t dataRate500KbpsUnits = event0->GetPayloadMode ().GetDataRate () / 500000;
      bool isShortPreamble = (WIFI_PREAMBLE_SHORT == event0->GetPreambleType ());
      double signalDbm = RatioToDb (event0->GetRxPowerW ()) + 30;
      double noiseDbm = RatioToDb (event0->GetRxPowerW () / snrPer.snr) - GetRxNoiseFigure () + 30;
      NotifyMonitorSniffRx (packet, (uint16_t)GetChannelFrequencyMhz (), GetChannelNumber (), dataRate500KbpsUnits, isShortPreamble, signalDbm, noiseDbm);
      m_state[0]->SwitchFromRxEndOk (packet, snrPer.snr, event0->GetPayloadMode (), event0->GetPreambleType ());
      if(division>1)
        m_state[1]->SwitchFromRxEndOk (packet, snrPer.snr, event1->GetPayloadMode (), event1->GetPreambleType ());
      if(division>2)
      {
        m_state[2]->SwitchFromRxEndOk (packet, snrPer.snr, event2->GetPayloadMode (), event2->GetPreambleType ());
        m_state[3]->SwitchFromRxEndOk (packet, snrPer.snr, event3->GetPayloadMode (), event3->GetPreambleType ());
      }
    }
    else
    {
      /* failure. */
      NotifyRxDrop (packet);
      m_state[0]->SwitchFromRxEndError (packet, snrPer.snr, event0->GetPayloadMode (), false);
      if(division>1)
        m_state[1]->SwitchFromRxEndError (packet, snrPer.snr, event1->GetPayloadMode (), false);
      if(division>2)
      {
        m_state[2]->SwitchFromRxEndError (packet, snrPer.snr, event2->GetPayloadMode (), false);
        m_state[3]->SwitchFromRxEndError (packet, snrPer.snr, event3->GetPayloadMode (), false);
      }
    }
  }
}	

//802.11ac channel bonding + ampdu
  void
YansWifiPhy::EndAmpduReceive (Ptr<Packet> packet, Ptr<InterferenceHelper::Event> event0,
		Ptr<InterferenceHelper::Event> event1,
		Ptr<InterferenceHelper::Event> event2,
		Ptr<InterferenceHelper::Event> event3)
{
  NS_LOG_FUNCTION (this << " receiving start");
  NS_ASSERT (IsStateRx ());
  NS_ASSERT (event0->GetEndTime () == Simulator::Now ());
  Ptr<Packet> ampdu0 = packet->Copy();
  Ptr<Packet> ampdu1 = packet->Copy();
  Ptr<Packet> ampdu2 = packet->Copy();
  Ptr<Packet> ampdu3 = packet->Copy();
  bool rxOneMPDU = false;
  
  MpduAggregator::DeaggregatedMpdus packets; 
  packets = MpduAggregator::Deaggregate (packet);

  NS_LOG_DEBUG("YansWifiPhy,  AmpduSize: " << packets.size() );
  NS_LOG_DEBUG ("# aggregated packets: " << packets.size ());	

	struct InterferenceHelper::SnrPer _plcpSnrPer1;
	struct InterferenceHelper::SnrPer _plcpSnrPer2;
	struct InterferenceHelper::SnrPer _plcpSnrPer3;
	struct InterferenceHelper::SnrPer _plcpSnrPer4;
  _plcpSnrPer1.snr=0;
  _plcpSnrPer1.per=0;
  _plcpSnrPer2.snr=0;
  _plcpSnrPer2.per=0;
  _plcpSnrPer3.snr=0;
  _plcpSnrPer3.per=0;
  _plcpSnrPer4.snr=0;
  _plcpSnrPer4.per=0;
  double plcpSnrPer = 0;
  if(event0)
	  _plcpSnrPer1 = m_interference[0].CalculatePlcpSnrPer(event0);
	else
		NS_ASSERT_MSG (false, "primary channel rx event should be called!");
  if(event1)
	  _plcpSnrPer2 = m_interference[1].CalculatePlcpSnrPer(event1);
  if(event2)
	  _plcpSnrPer3 = m_interference[2].CalculatePlcpSnrPer(event2);
  if(event3)
	  _plcpSnrPer4 = m_interference[3].CalculatePlcpSnrPer(event3);

  NS_LOG_DEBUG ("plcpSnrPer: " << _plcpSnrPer1.per << " " << _plcpSnrPer2.per << " " << _plcpSnrPer3.per << " " << _plcpSnrPer4.per);

	plcpSnrPer = 1-(1-_plcpSnrPer1.per)*(1-_plcpSnrPer2.per)*(1-_plcpSnrPer3.per)*(1-_plcpSnrPer4.per);

	SnrPers snrPers;
	SnrPers snrPers0;
	SnrPers snrPers1;
  SnrPers snrPers2;
  SnrPers snrPers3;
   
  uint16_t division = !!(event0) + !!(event1) + !!(event2) + !!(event3); 
  struct InterferenceHelper::SnrPer snrPer;

	bool plcpHeaderError = false;
	if( m_random->GetValue () <  plcpSnrPer){
		snrPer.per = 1.0;
		NS_LOG_DEBUG("plcp broken! " << plcpSnrPer);
		plcpHeaderError = true;
	}
  if(event0)
    snrPers0 = m_interference[0].CalculateAmpduSnrPer (event0, packet->GetSize(), packets.size(), ampdu0);
  if(event1)
    snrPers1 = m_interference[1].CalculateAmpduSnrPer (event1, packet->GetSize(), packets.size(), ampdu1);
  if(event2)
    snrPers2 = m_interference[2].CalculateAmpduSnrPer (event2, packet->GetSize(), packets.size(), ampdu2);
  if(event3)
    snrPers3 = m_interference[3].CalculateAmpduSnrPer (event3, packet->GetSize(), packets.size(), ampdu3);
   
  if(division > 0)
    m_interference[0].NotifyRxEnd ();
  if(division > 1)
    m_interference[1].NotifyRxEnd ();
  if(division > 2)
  {
    m_interference[2].NotifyRxEnd ();
    m_interference[3].NotifyRxEnd ();
  }

  YansWifiPhy::SnrPersCI bbb = snrPers1.begin(); bbb++;
  YansWifiPhy::SnrPersCI ccc = snrPers2.begin(); ccc++;
  YansWifiPhy::SnrPersCI ddd = snrPers3.begin(); ddd++;

  for(YansWifiPhy::SnrPersCI aaa = snrPers0.begin(); aaa !=snrPers0.end(); ++aaa)
  {
    switch(division)
    {
      case 1:
        snrPer.snr = (*aaa).snr;
        snrPer.per = (*aaa).per;
        NS_LOG_LOGIC("primary=" << (*aaa).per);
        NS_LOG_LOGIC("primary=" << 10*log10((*aaa).snr));
        snrPers.push_back(snrPer);
        break;
      case 2:
        snrPer.per = 1-(1-(*aaa).per)*(1-(*bbb).per);
        snrPer.snr = (*aaa).snr;
        NS_LOG_LOGIC("primary=" << (*aaa).per << ", s20=" << (*bbb).per);
        NS_LOG_LOGIC("primary=" << 10*log10((*aaa).snr) << ", s20=" << 10*log10((*bbb).snr));
        snrPers.push_back(snrPer);
        ++bbb;
        break;
      case 4:
        snrPer.per = 1-(1-(*aaa).per)*(1-(*bbb).per)*(1-(*ccc).per)*(1-(*ddd).per);
        snrPer.snr = (*aaa).snr;
        NS_LOG_LOGIC("primary=" << (*aaa).per << ", s20=" << (*bbb).per << ", s40_up=" << (*ccc).per << ", s40_down=" << (*ddd).per);
        NS_LOG_LOGIC("primary=" << 10*log10((*aaa).snr) << ", s20=" << 10*log10((*bbb).snr) << ", s40_up=" << 10*log10((*ccc).snr) << ", s40_down=" << 10*log10((*ddd).snr));
        snrPers.push_back(snrPer);
        ++bbb; ++ccc; ++ddd;
        break;
      default:
        NS_LOG_DEBUG("bandwidth is wrong");
        NS_ASSERT(false);
        break;
    }
    //find effective snr and packet error rate
    NS_LOG_DEBUG("ESNR=" << 10*log10(snrPer.snr) << " EPER=" << snrPer.per << " bw=" << division << " plcpSnrPer=" << plcpSnrPer);
  }
  NS_ASSERT(packets.size()+1 == snrPers.size());
	YansWifiPhy::SnrPersCI j = snrPers.begin ();
  j++;
  uint32_t k = 0;
  for (MpduAggregator::DeaggregatedMpdusCI i = packets.begin ();
      i != packets.end (); ++i,++j,++k)
  { 				
    AmpduTag tag;
    if(k==(packets.size() - 1)){
      NS_LOG_DEBUG("last aggregated packet -> tag set true");
      (*i).first->RemovePacketTag(tag);
      (*i).first->AddPacketTag(AmpduTag (true));
    }
    (*i).first->PeekPacketTag(tag);
    NS_LOG_DEBUG(k << " th packet mode=" << event0->GetPayloadMode ().GetDataRate () <<
        " ,StartTime=" << event0->GetStartTime () <<
        " ,EndTime=" << event0->GetEndTime () <<
        ", snr=" << 10*log10((*j).snr) << ", per=" << (*j).per << 
        ", size=" <<(*i).first->GetSize () << 
        ", ampduTag=" <<tag.Get ()); 

    if (m_random->GetValue () > (*j).per && !plcpHeaderError)
    {
      NotifyRxEnd ((*i).first);
      uint32_t dataRate500KbpsUnits = event0->GetPayloadMode ().GetDataRate () / 500000;
      bool isShortPreamble = (WIFI_PREAMBLE_SHORT == event0->GetPreambleType ());
      double signalDbm = RatioToDb (event0->GetRxPowerW ()) + 30;
      double noiseDbm = RatioToDb (event0->GetRxPowerW () / (*j).snr) - GetRxNoiseFigure () + 30;
      NotifyMonitorSniffRx ((*i).first, (uint16_t)GetChannelFrequencyMhz (), GetChannelNumber (), dataRate500KbpsUnits, isShortPreamble, signalDbm, noiseDbm);
      m_state[0]->SwitchFromRxEndOk ((*i).first, (*j).snr, event0->GetPayloadMode (), event0->GetPreambleType ());
      if(division>1)
        m_state[1]->SwitchFromRxEndOk ((*i).first, (*j).snr, event1->GetPayloadMode (), event1->GetPreambleType ());
      if(division>2)
      {
        m_state[2]->SwitchFromRxEndOk ((*i).first, (*j).snr, event2->GetPayloadMode (), event2->GetPreambleType ());
        m_state[3]->SwitchFromRxEndOk ((*i).first, (*j).snr, event3->GetPayloadMode (), event3->GetPreambleType ());
      }
      rxOneMPDU = true; 
    }
    else
    {
      /* failure. */
      NS_LOG_DEBUG("plcp header error? " << plcpHeaderError);
      NotifyRxDrop ((*i).first);
      m_state[0]->SwitchFromRxEndError ((*i).first, (*j).snr, event0->GetPayloadMode (), rxOneMPDU); 
      if(division>1)
        m_state[1]->SwitchFromRxEndError ((*i).first, (*j).snr, event1->GetPayloadMode (), rxOneMPDU);
      if(division>2)
      {
        m_state[2]->SwitchFromRxEndError ((*i).first, (*j).snr, event2->GetPayloadMode (), rxOneMPDU);
        m_state[3]->SwitchFromRxEndError ((*i).first, (*j).snr, event3->GetPayloadMode (), rxOneMPDU);

      }

    }
  }
}

int64_t
YansWifiPhy::AssignStreams (int64_t stream)
{
  NS_LOG_FUNCTION (this << stream);
  m_random->SetStream (stream);
  return 1;
}

void
YansWifiPhy::SetFrequency (uint32_t freq)
{
  m_channelStartingFrequency = freq;
}

void
YansWifiPhy::SetNumberOfTransmitAntennas (uint32_t tx)
{
  m_numberOfTransmitters = tx;
}
void
YansWifiPhy::SetNumberOfReceiveAntennas (uint32_t rx)
{
  m_numberOfReceivers = rx;
}

void
YansWifiPhy::SetLdpc (bool Ldpc)
{
  m_ldpc = Ldpc;
}

void
YansWifiPhy::SetStbc (bool stbc)
{
  m_stbc = stbc;
}

void
YansWifiPhy::SetGreenfield (bool greenfield)
{
  m_greenfield = greenfield;
}
bool
YansWifiPhy::GetGuardInterval (void) const
{
  return m_guardInterval;
}
void
YansWifiPhy::SetGuardInterval (bool guardInterval)
{
  m_guardInterval = guardInterval;
}

uint32_t
YansWifiPhy::GetFrequency (void) const
{
  return m_channelStartingFrequency;
}

uint32_t
YansWifiPhy::GetNumberOfTransmitAntennas (void) const
{
  return m_numberOfTransmitters;
}
uint32_t
YansWifiPhy::GetNumberOfReceiveAntennas (void) const
{
  return m_numberOfReceivers;
}

bool
YansWifiPhy::GetLdpc (void) const
{
  return m_ldpc;
}
bool
YansWifiPhy::GetStbc (void) const
{
  return m_stbc;
}

bool
YansWifiPhy::GetGreenfield (void) const
{
  return m_greenfield;
}

bool
YansWifiPhy::GetChannelBonding(void) const
{
  return m_channelBonding;
}

void
YansWifiPhy::SetChannelBonding(bool channelbonding) 
{
  m_channelBonding= channelbonding;
}

//11ac: vht_standard
void
YansWifiPhy::Configure80211ac (void)
{
  NS_LOG_FUNCTION (this);
  m_channelStartingFrequency = 5e3; // 5.000 GHz
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate6Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate9Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate12Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate18Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate24Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate36Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate48Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate54Mbps ());
  for (uint8_t i=0; i <10; i++)
    {
      m_deviceMcsSet.push_back(i);
    }
}
void
YansWifiPhy::Configure80211n (void)
{
  NS_LOG_FUNCTION (this);
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate6Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate9Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate12Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate18Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate24Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate36Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate48Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate54Mbps ());
  m_bssMembershipSelectorSet.push_back(HT_PHY);
  for (uint8_t i=0; i <8; i++)
    {
      m_deviceMcsSet.push_back(i);
    }

}
uint32_t
YansWifiPhy::GetNBssMembershipSelectors (void) const
{
  return  m_bssMembershipSelectorSet.size ();
}
uint32_t
YansWifiPhy::GetBssMembershipSelector (uint32_t selector) const
{
  return  m_bssMembershipSelectorSet[selector];
}
WifiModeList
YansWifiPhy::GetMembershipSelectorModes(uint32_t selector)
{
  uint32_t id=GetBssMembershipSelector(selector);
  WifiModeList supportedmodes;
  if (id == HT_PHY)
  {
    //mandatory MCS 0 to 7
     supportedmodes.push_back (WifiPhy::GetOfdmRate6_5MbpsBW20MHz ());
     supportedmodes.push_back (WifiPhy::GetOfdmRate13MbpsBW20MHz ());
     supportedmodes.push_back (WifiPhy::GetOfdmRate19_5MbpsBW20MHz ());
     supportedmodes.push_back (WifiPhy::GetOfdmRate26MbpsBW20MHz ());
     supportedmodes.push_back (WifiPhy::GetOfdmRate39MbpsBW20MHz ());
     supportedmodes.push_back (WifiPhy::GetOfdmRate52MbpsBW20MHz ());
     supportedmodes.push_back (WifiPhy::GetOfdmRate58_5MbpsBW20MHz ());
     supportedmodes.push_back (WifiPhy::GetOfdmRate65MbpsBW20MHz ());
  }
  return supportedmodes;
}
uint8_t
YansWifiPhy::GetNMcs (void) const
{
  return  m_deviceMcsSet.size ();
}
uint8_t
YansWifiPhy::GetMcs (uint8_t mcs) const
{
  return  m_deviceMcsSet[mcs];
}
uint32_t 
YansWifiPhy::WifiModeToMcs (WifiMode mode)
{
    uint32_t mcs = 0;
   if (mode.GetUniqueName() == "OfdmRate135MbpsBW40MHzShGi" || mode.GetUniqueName() == "OfdmRate65MbpsBW20MHzShGi" )
     {
             mcs=6;
     }
  else
    {
     switch (mode.GetDataRate())
       {
         case 6500000:
         case 7200000:
         case 13500000:
         case 15000000:
           mcs=0;
           break;
         case 13000000:
         case 14400000:
         case 27000000:
         case 30000000:
           mcs=1;
           break;
         case 19500000:
         case 21700000:
         case 40500000:
         case 45000000:
           mcs=2;
           break;
         case 26000000:
         case 28900000:
         case 54000000:
         case 60000000:
           mcs=3;
           break;
         case 39000000:
         case 43300000:
         case 81000000:
         case 90000000:        
           mcs=4;
           break;
         case 52000000:
         case 57800000:
         case 108000000:
         case 120000000:
           mcs=5;
           break; 
         case 58500000:
         case 121500000:
           mcs=6;
           break;
         case 65000000:
         case 72200000:
         case 135000000:
         case 150000000:
           mcs=7;
           break;     
       }
    }
  return mcs;
}
WifiMode
YansWifiPhy::McsToWifiMode (uint8_t mcs)
{
   WifiMode mode;
   switch (mcs)
     { 
       case 7:
          if (!GetGuardInterval() && !GetChannelBonding())
           {
              mode =  WifiPhy::GetOfdmRate65MbpsBW20MHz ();
            }
         else if(GetGuardInterval() && !GetChannelBonding())
            {
              mode = WifiPhy::GetOfdmRate72_2MbpsBW20MHz ();
            }
          else if (!GetGuardInterval() && GetChannelBonding())
            {
              mode = WifiPhy::GetOfdmRate135MbpsBW40MHz ();
            }
          else
            {
              mode = WifiPhy::GetOfdmRate150MbpsBW40MHz ();
            }
          break;
       case 6:
          if (!GetGuardInterval() && !GetChannelBonding())
           {
              mode = WifiPhy::GetOfdmRate58_5MbpsBW20MHz ();
 
            }
         else if(GetGuardInterval() && !GetChannelBonding())
            {
              mode =  WifiPhy::GetOfdmRate65MbpsBW20MHzShGi ();
       
            }
          else if (!GetGuardInterval() && GetChannelBonding())
            {
              mode = WifiPhy::GetOfdmRate121_5MbpsBW40MHz ();
     
            }
          else
            {
              mode= WifiPhy::GetOfdmRate135MbpsBW40MHzShGi ();
          
            }
          break;
       case 5:
          if (!GetGuardInterval() && !GetChannelBonding())
           {
              mode = WifiPhy::GetOfdmRate52MbpsBW20MHz ();
  
            }
         else if(GetGuardInterval() && !GetChannelBonding())
            {
              mode = WifiPhy::GetOfdmRate57_8MbpsBW20MHz ();
            }
          else if (!GetGuardInterval() && GetChannelBonding())
            {
              mode = WifiPhy::GetOfdmRate108MbpsBW40MHz ();
     
            }
          else
            {
              mode = WifiPhy::GetOfdmRate120MbpsBW40MHz ();
       
            }
          break;
       case 4:
          if (!GetGuardInterval() && !GetChannelBonding())
           {
              mode = WifiPhy::GetOfdmRate39MbpsBW20MHz ();
            }
         else if(GetGuardInterval() && !GetChannelBonding())
            {
              mode = WifiPhy::GetOfdmRate43_3MbpsBW20MHz ();
            }
          else if (!GetGuardInterval() && GetChannelBonding())
            {
              mode = WifiPhy::GetOfdmRate81MbpsBW40MHz ();
  
            }
          else
            {
              mode = WifiPhy::GetOfdmRate90MbpsBW40MHz ();
         
            }
          break;
       case 3:
          if (!GetGuardInterval() && !GetChannelBonding())
           {
              mode =  WifiPhy::GetOfdmRate26MbpsBW20MHz ();
  
            }
         else if(GetGuardInterval() && !GetChannelBonding())
            {
              mode = WifiPhy::GetOfdmRate28_9MbpsBW20MHz ();
      
            }
          else if (!GetGuardInterval() && GetChannelBonding())
            {
              mode = WifiPhy::GetOfdmRate54MbpsBW40MHz ();
     
            }
          else
            {
              mode = WifiPhy::GetOfdmRate60MbpsBW40MHz ();
            }
          break;
       case 2:
          if (!GetGuardInterval() && !GetChannelBonding())
           {
              mode = WifiPhy::GetOfdmRate19_5MbpsBW20MHz ();
 
            }
         else if(GetGuardInterval() && !GetChannelBonding())
            {
              mode = WifiPhy::GetOfdmRate21_7MbpsBW20MHz ();
     
            }
          else if (!GetGuardInterval() && GetChannelBonding())
            {
              mode =  WifiPhy::GetOfdmRate40_5MbpsBW40MHz ();
  
            }
          else
            {
              mode = WifiPhy::GetOfdmRate45MbpsBW40MHz ();
           
            }
          break;
       case 1:
          if (!GetGuardInterval() && !GetChannelBonding())
           {
            mode = WifiPhy::GetOfdmRate13MbpsBW20MHz ();
  
            }
         else if(GetGuardInterval() && !GetChannelBonding())
            {
              mode =  WifiPhy::GetOfdmRate14_4MbpsBW20MHz ();
            }
          else if (!GetGuardInterval() && GetChannelBonding())
            {
              mode = WifiPhy::GetOfdmRate27MbpsBW40MHz ();
     
            }
          else
            {
              mode = WifiPhy::GetOfdmRate30MbpsBW40MHz ();
            }
          break;
       case 0:
       default:
         if (!GetGuardInterval() && !GetChannelBonding())
           {
              mode = WifiPhy::GetOfdmRate6_5MbpsBW20MHz ();
              
            }
         else if(GetGuardInterval() && !GetChannelBonding())
            {
              mode = WifiPhy::GetOfdmRate7_2MbpsBW20MHz ();
            }
          else if (!GetGuardInterval() && GetChannelBonding())
            {
              mode = WifiPhy::GetOfdmRate13_5MbpsBW40MHz ();
 
            }
          else
            {
              mode = WifiPhy::GetOfdmRate15MbpsBW40MHz ();
            }
         break;
        }
 return mode;
}

//11ac: vht_standard
uint32_t 
YansWifiPhy::WifiModeToAcMcs (WifiMode mode)
{
    uint32_t mcs = 0;

  
     switch (mode.GetDataRate())
       {
         case 7200000:
         case 15000000:
         case 32500000:
           mcs=0;
           break;
         case 14400000:
         case 30000000:
         //case 65000000:
           mcs=1;
           break;
         case 21700000:
         case 45000000:
         case 97500000:
           mcs=2;
           break;
         case 28900000:
         case 60000000:
         case 130000000:
           mcs=3;
           break;
         case 43300000:
         case 90000000:
         case 195000000:        
           mcs=4;
           break;
         case 57800000:
         case 120000000:
         case 260000000:
           mcs=5;
           break; 
         case 65000000:
         case 135000000:
	  case 292500000:	 	
           mcs=6;
           break;
         case 72200000:
         case 150000000:
         case 325000000:
           mcs=7;
	    break;
	  case 86700000:
	  case 180000000:
	  case 390000000:	
	    mcs=8;	
           break;  
	  case 96300000:	   
	  case 200000000:	   
	  case 433300000:
	    mcs=9;	
           break;  
       
    	}
  return mcs;
}
//11ac: vht_standard
WifiMode
YansWifiPhy::AcMcsToWifiMode (uint8_t mcs, uint16_t bw)
{
   WifiMode mode;
   switch (mcs)
   	{
   	case 0:
		if (bw == 80)	{mode = WifiPhy::Get11acMcs0BW80MHz ();}
		else if (bw == 40) {mode = WifiPhy::Get11acMcs0BW40MHz ();}
		else {mode = WifiPhy::Get11acMcs0BW20MHz ();}
		break;
	case 1:
		if (bw == 80)	{mode = WifiPhy::Get11acMcs1BW80MHz ();}
		else if (bw == 40) {mode = WifiPhy::Get11acMcs1BW40MHz ();}
		else {mode = WifiPhy::Get11acMcs1BW20MHz ();}
		break;
	case 2:
		if (bw == 80)	{mode = WifiPhy::Get11acMcs2BW80MHz ();}
		else if (bw == 40) {mode = WifiPhy::Get11acMcs2BW40MHz ();}
		else {mode = WifiPhy::Get11acMcs2BW20MHz ();}
		break;
	case 3:
		if (bw == 80)	{mode = WifiPhy::Get11acMcs3BW80MHz ();}
		else if (bw == 40) {mode = WifiPhy::Get11acMcs3BW40MHz ();}
		else {mode = WifiPhy::Get11acMcs3BW20MHz ();}
		break;
	case 4:
		if (bw == 80)	{mode = WifiPhy::Get11acMcs4BW80MHz ();}
		else if (bw == 40) {mode = WifiPhy::Get11acMcs4BW40MHz ();}
		else {mode = WifiPhy::Get11acMcs4BW20MHz ();}
		break;
	case 5:
		if (bw == 80)	{mode = WifiPhy::Get11acMcs5BW80MHz ();}
		else if (bw == 40) {mode = WifiPhy::Get11acMcs5BW40MHz ();}
		else {mode = WifiPhy::Get11acMcs5BW20MHz ();}
		break;
	case 6:
		if (bw == 80)	{mode = WifiPhy::Get11acMcs6BW80MHz ();}
		else if (bw == 40) {mode = WifiPhy::Get11acMcs6BW40MHz ();}
		else {mode = WifiPhy::Get11acMcs6BW20MHz ();}
		break;
	case 7:
		if (bw == 80)	{mode = WifiPhy::Get11acMcs7BW80MHz ();}
		else if (bw == 40) {mode = WifiPhy::Get11acMcs7BW40MHz ();}
		else {mode = WifiPhy::Get11acMcs7BW20MHz ();}
		break;
	case 8:
		if (bw == 80)	{mode = WifiPhy::Get11acMcs8BW80MHz ();}
		else if (bw == 40) {mode = WifiPhy::Get11acMcs8BW40MHz ();}
		else {mode = WifiPhy::Get11acMcs8BW20MHz ();}
		break;
	case 9:
		if (bw == 80)	{mode = WifiPhy::Get11acMcs9BW80MHz ();}
		else if (bw == 40) {mode = WifiPhy::Get11acMcs9BW40MHz ();}
		else {mode = WifiPhy::Get11acMcs8BW20MHz ();}
		break;
   	}           
   return mode;
}

} // namespace ns3
