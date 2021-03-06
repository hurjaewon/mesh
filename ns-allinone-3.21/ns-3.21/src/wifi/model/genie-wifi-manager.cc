#include "genie-wifi-manager.h"
#include "wifi-phy.h"
#include "ns3/assert.h"
#include "ns3/double.h"

//////////////////////////ns3_lecture//////////////////////////////
#include <cmath>
#include "wifi-mac.h"
#include "yans-wifi-phy.h"
#include "yans-wifi-channel.h"
#include "ns3/string.h"
#include "ns3/log.h"
#include "ns3/object.h"
#include "ns3/mobility-model.h"
#include "ns3/propagation-loss-model.h"
#include "wifi-mac-header.h"
#include <vector>
#include <math.h>
#include "ns3/log.h"

#include "ns3/random-variable.h"
#define Min(a,b) ((a < b) ? a : b)

namespace ns3 {

struct GenieWifiRemoteStation : public WifiRemoteStation
{
  double m_lastSnr;
};

NS_OBJECT_ENSURE_REGISTERED (GenieWifiManager);
NS_LOG_COMPONENT_DEFINE("GenieWifiManager");
TypeId
GenieWifiManager::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::GenieWifiManager")
    .SetParent<WifiRemoteStationManager> ()
    .AddConstructor<GenieWifiManager> ()
    .AddAttribute ("BerThreshold",
                   "The maximum Bit Error Rate acceptable at any transmission mode",
                   DoubleValue (10e-6),
                   MakeDoubleAccessor (&GenieWifiManager::m_ber),
                   MakeDoubleChecker<double> ())
    //////////////////////////ns3_lecture//////////////////////////////
    .AddAttribute ("PerThreshold",
                   "The maximum Bit Error Rate acceptable at any transmission mode",
                   DoubleValue (10e-2),
                   MakeDoubleAccessor (&GenieWifiManager::m_per),
                   MakeDoubleChecker<double> ())
  ;
  return tid;
}

GenieWifiManager::GenieWifiManager ()
{
}
GenieWifiManager::~GenieWifiManager ()
{
}

void
GenieWifiManager::SetupPhy (Ptr<WifiPhy> phy)
{
  uint8_t nModes = phy->GetNModes ();
  for (uint8_t i = 0; i < nModes; i++)
    {
      WifiMode mode = phy->GetMode (i);
      AddModeSnrThreshold (mode, phy->CalculateSnr (mode, m_ber));
    }
  uint8_t nMcs = 0;
	if(HasHtSupported())
	{
		nMcs = phy->GetNMcs ();
		for (uint8_t i = 0; i < nMcs; i++)
    {
      WifiMode mode = phy->McsToWifiMode(phy->GetMcs (i));
      AddModeSnrThreshold (mode, phy->CalculateSnr (mode, m_ber));
    }
	}
	
	if(HasVhtSupported())
	{
		nMcs = phy->GetNMcs ();
		for (uint8_t i = 0; i < nMcs; i++)
    {
      WifiMode mode = phy->AcMcsToWifiMode(phy->GetMcs (i), 20);
      AddModeSnrThreshold (mode, phy->CalculateSnr (mode, m_ber));
      mode = phy->AcMcsToWifiMode(phy->GetMcs (i), 40);
      AddModeSnrThreshold (mode, phy->CalculateSnr (mode, m_ber));
      mode = phy->AcMcsToWifiMode(phy->GetMcs (i), 80);
      AddModeSnrThreshold (mode, phy->CalculateSnr (mode, m_ber));
    }
	}
  WifiRemoteStationManager::SetupPhy (phy);
}

double
GenieWifiManager::GetSnrThreshold (WifiMode mode) const
{
  for (Thresholds::const_iterator i = m_thresholds.begin (); i != m_thresholds.end (); i++)
    {
      if (mode == i->second)
        {
          return i->first;
        }
    }
  NS_ASSERT (false);
  return 0.0;
}

void
GenieWifiManager::AddModeSnrThreshold (WifiMode mode, double snr)
{
  for (Thresholds::const_iterator i = m_thresholds.begin (); i != m_thresholds.end (); i++)
    {
      if (mode == i->second)
        {
          return;
        }
    }
  m_thresholds.push_back (std::make_pair (snr,mode));
}

WifiRemoteStation *
GenieWifiManager::DoCreateStation (void) const
{
  GenieWifiRemoteStation *station = new GenieWifiRemoteStation ();
  station->m_lastSnr = 0.0;
  return station;
}


void
GenieWifiManager::DoReportRxOk (WifiRemoteStation *station,
                                double rxSnr, WifiMode txMode)
{
}
void
GenieWifiManager::DoReportRtsFailed (WifiRemoteStation *station)
{
}
void
GenieWifiManager::DoReportDataFailed (WifiRemoteStation *station)
{
}
void
GenieWifiManager::DoReportRtsOk (WifiRemoteStation *st,
                                 double ctsSnr, WifiMode ctsMode, double rtsSnr)
{
  GenieWifiRemoteStation *station = (GenieWifiRemoteStation *)st;
  station->m_lastSnr = rtsSnr;
}
void
GenieWifiManager::DoReportDataOk (WifiRemoteStation *st,
                                  double ackSnr, WifiMode ackMode, double dataSnr)
{
  GenieWifiRemoteStation *station = (GenieWifiRemoteStation *)st;
  station->m_lastSnr = dataSnr;
}
void
GenieWifiManager::DoReportFinalRtsFailed (WifiRemoteStation *station)
{
}
void
GenieWifiManager::DoReportFinalDataFailed (WifiRemoteStation *station)
{
}

WifiTxVector
GenieWifiManager::DoGetDataTxVector (WifiRemoteStation *st, uint32_t size, uint16_t bw)
{

	uint16_t bwLoss = bw/20;
	uint16_t txAntenna = Min (GetNumberOfReceiveAntennas (st),GetNumberOfTransmitAntennas());
  //////////////////////////ns3_lecture//////////////////////////////
  Ptr<WifiPhy> wifiPhy = GetWifiPhy();
  // Receiver MAC address and PHY instances
  Mac48Address receiverMac48Address = st->m_state->m_address;
  Ptr<WifiPhy> receiverWifiPhy;
  Ptr<YansWifiPhy> receiverYansWifiPhy;
  // Target instances attached to the channel
  Ptr<WifiNetDevice> testWifiNetDevice;
  Mac48Address testMac48Address;
  // Channel instance that this WifiManager is attached to
  Ptr<YansWifiChannel> channel;
  // Find the receiver PHY instances

  uint32_t nDevices = wifiPhy->GetChannel()->GetNDevices();
  for (uint32_t k = 0; k < nDevices; k++)
  {
    testWifiNetDevice = wifiPhy->GetChannel()->GetDevice(k)->GetObject<WifiNetDevice>();
    testMac48Address = testWifiNetDevice->GetMac()->GetAddress();
    if (testMac48Address == receiverMac48Address)
    {
      receiverWifiPhy = testWifiNetDevice->GetPhy();
      break;
    }
  }
  receiverYansWifiPhy = receiverWifiPhy->GetObject<YansWifiPhy>();
  channel = wifiPhy->GetChannel()->GetObject<YansWifiChannel>();
  // Mobility models for calculating pathloss
  Ptr<MobilityModel> receiverMobility = receiverYansWifiPhy->GetMobility()->GetObject<MobilityModel>();
  Ptr<MobilityModel> senderMobility = wifiPhy->GetObject<YansWifiPhy>()->GetMobility()->GetObject<MobilityModel>();

  // Noise figure >> Noise floor (ref: InterferenceHelper::CalculateSnr)
  double txPowerDbm = wifiPhy->GetObject<YansWifiPhy>()->GetPowerDbm(GetDefaultTxPowerLevel()) 
										+ wifiPhy->GetObject<YansWifiPhy>()->GetTxGain();
 
	//shbyeon: multiple stream processing
  WifiTxVector txVector (GetDefaultMode(), GetDefaultTxPowerLevel (), GetLongRetryCount (st), GetShortGuardInterval (st),
			txAntenna, GetNumberOfTransmitAntennas (st), GetStbc (st));
	txVector.SetBandwidth(bw);
	
	WifiMode results[txAntenna];

	for(uint16_t nss = 1; nss < txAntenna+1; nss++)
	{
		txVector.SetNss(nss);
		double maxThreshold = 0.0;
		if(nss == 1)
		{
			std::complex<double> * hvector = new std::complex<double> [nss*nss+1];
			hvector[0].real() = txPowerDbm;
			hvector = channel->GetPropagationLossModel()->CalcRxPower (hvector, senderMobility, receiverMobility, nss, NULL);
			double rxPower = hvector[0].real() + 1; //m_rxGain
			rxPower = pow(10.0, rxPower/10.0)/bwLoss/1000;

			WifiMode maxMode = GetDefaultMode ();
			uint32_t supported = 0;
			bool HT = false;
			bool VHT = false;
			if(HasHtSupported())
			{
				supported = GetNMcsSupported(st);
				HT = true;
				if(HasVhtSupported())
				{
					VHT = true;
					maxMode = AcMcsToWifiMode(GetDefaultMcs(), bw);
				}
				else
					maxMode = McsToWifiMode(GetDefaultMcs());
			}
			else
				supported = GetNSupported(st); 
			
      txVector.SetChannelMatrix(hvector, nss, 1);
      delete [] hvector;

			for (uint32_t i = 0; i < supported; i++)
			{
				WifiMode mode;
				if(VHT)
					mode = AcMcsToWifiMode(GetMcsSupported (st, i), bw);
				else if(HT)
					mode = McsToWifiMode(GetMcsSupported (st, i));
				else
					mode = GetSupported (st, i);
        
        txVector.SetMode(mode);
        m_currentSnr = receiverYansWifiPhy->GetInterferenceHelper().CalculateSnr (rxPower, 0, txVector);

				double threshold = GetSnrThreshold (mode);
				if (threshold > maxThreshold
						&& threshold < m_currentSnr)
				{
					maxThreshold = threshold;
					maxMode = mode;
				}
			}
			results[0] = maxMode;
			NS_LOG_DEBUG(nss << "-streams, " << "estimated snr=" << 10* std::log10(m_currentSnr));
		}
		else if(nss >= 2)
		{
			NS_ASSERT_MSG(HasHtSupported() || HasVhtSupported(), 
					"# of antenna should be smmaller than 2 for 11a");
			std::complex<double> * hvector = new std::complex<double> [nss*nss+1];
			hvector[0].real() = txPowerDbm;
			hvector = channel->GetPropagationLossModel()->CalcRxPower (hvector, senderMobility, receiverMobility, nss, NULL);
			double rxPower = hvector[0].real() + 1; //m_rxGain
			rxPower = pow(10.0, rxPower/10.0)/bwLoss/1000;
			WifiMode maxMode;
			if(HasVhtSupported())
				maxMode = AcMcsToWifiMode(GetDefaultMcs(), bw);
			else
				maxMode = McsToWifiMode(GetDefaultMcs());
			txVector.SetNss(nss);
			txVector.SetChannelMatrix(hvector, nss, 1);
      delete [] hvector;
      uint16_t mcsMax = GetNMcsSupported(st);	

      for (uint16_t mcs = 0; mcs < mcsMax; mcs++)
      {
        //802.11ac call interference helper 
        WifiMode mode;
        if(HasVhtSupported())
          mode = AcMcsToWifiMode(GetMcsSupported (st, mcs), bw);
        else
          mode = McsToWifiMode(GetMcsSupported (st, mcs));
        txVector.SetMode(mode);
        m_currentSnr = receiverYansWifiPhy->GetInterferenceHelper().CalculateSnr (rxPower, 0, txVector);
        double threshold = GetSnrThreshold (mode);
        if(threshold > maxThreshold && threshold < m_currentSnr)
        {
          maxThreshold = threshold;
          maxMode = mode;
        }
        results[nss-1] = maxMode;
      }

			NS_LOG_DEBUG(nss << "-streams, " << "estimated snr=" << 10* std::log10(m_currentSnr));
    }
	}
	WifiMode final = results[0];
	uint16_t final_idx = 0;
	for (uint16_t nss = 0; nss < txAntenna-1; nss++)
	{
		if(results[final_idx].GetDataRate()*(final_idx+1) < results[nss+1].GetDataRate()*(nss+2))
		{
			final = results[nss+1];
			final_idx = nss+1;
		}
	}
	txVector.SetNss(final_idx+1);
	txVector.SetMode(final);
  NS_LOG_DEBUG("final ss=" << final_idx+1 << " mode=" << final);
	return txVector;
}

WifiTxVector
GenieWifiManager::DoGetDataTxVector (WifiRemoteStation *st, uint32_t size)
{
	uint16_t bw = GetCurrentBandwidth(st);
	uint16_t bwLoss = bw/20;
	uint16_t txAntenna = Min (GetNumberOfReceiveAntennas (st),GetNumberOfTransmitAntennas());
  //////////////////////////ns3_lecture//////////////////////////////
  Ptr<WifiPhy> wifiPhy = GetWifiPhy();
  // Receiver MAC address and PHY instances
  Mac48Address receiverMac48Address = st->m_state->m_address;
  Ptr<WifiPhy> receiverWifiPhy;
  Ptr<YansWifiPhy> receiverYansWifiPhy;
  // Target instances attached to the channel
  Ptr<WifiNetDevice> testWifiNetDevice;
  Mac48Address testMac48Address;
  // Channel instance that this WifiManager is attached to
  Ptr<YansWifiChannel> channel;
  // Find the receiver PHY instances

  uint32_t nDevices = wifiPhy->GetChannel()->GetNDevices();
  for (uint32_t k = 0; k < nDevices; k++)
  {
    testWifiNetDevice = wifiPhy->GetChannel()->GetDevice(k)->GetObject<WifiNetDevice>();
    testMac48Address = testWifiNetDevice->GetMac()->GetAddress();
    if (testMac48Address == receiverMac48Address)
    {
      receiverWifiPhy = testWifiNetDevice->GetPhy();
      break;
    }
  }
  receiverYansWifiPhy = receiverWifiPhy->GetObject<YansWifiPhy>();
  channel = wifiPhy->GetChannel()->GetObject<YansWifiChannel>();
  // Mobility models for calculating pathloss
  Ptr<MobilityModel> receiverMobility = receiverYansWifiPhy->GetMobility()->GetObject<MobilityModel>();
  Ptr<MobilityModel> senderMobility = wifiPhy->GetObject<YansWifiPhy>()->GetMobility()->GetObject<MobilityModel>();

  // Noise figure >> Noise floor (ref: InterferenceHelper::CalculateSnr)
  double txPowerDbm = wifiPhy->GetObject<YansWifiPhy>()->GetPowerDbm(GetDefaultTxPowerLevel()) 
										+ wifiPhy->GetObject<YansWifiPhy>()->GetTxGain();
 
	//shbyeon: multiple stream processing
  WifiTxVector txVector (GetDefaultMode(), GetDefaultTxPowerLevel (), GetLongRetryCount (st), GetShortGuardInterval (st),
			txAntenna, GetNumberOfTransmitAntennas (st), GetStbc (st));
	txVector.SetBandwidth(bw);
	
	WifiMode results[txAntenna];

	for(uint16_t nss = 1; nss < txAntenna+1; nss++)
	{
		txVector.SetNss(nss);
		double maxThreshold = 0.0;
		if(nss == 1)
		{
			std::complex<double> * hvector = new std::complex<double> [nss*nss+1];
			hvector[0].real() = txPowerDbm;
			hvector = channel->GetPropagationLossModel()->CalcRxPower (hvector, senderMobility, receiverMobility, nss, NULL);
			double rxPower = hvector[0].real() + 1; //m_rxGain
			rxPower = pow(10.0, rxPower/10.0)/bwLoss/1000;

			WifiMode maxMode = GetDefaultMode ();
			uint32_t supported = 0;
			bool HT = false;
			bool VHT = false;
			if(HasHtSupported())
			{
				supported = GetNMcsSupported(st);
				HT = true;
				if(HasVhtSupported())
				{
					VHT = true;
					maxMode = AcMcsToWifiMode(GetDefaultMcs(), bw);
				}
				else
					maxMode = McsToWifiMode(GetDefaultMcs());
			}
			else
				supported = GetNSupported(st); 
			
      txVector.SetChannelMatrix(hvector, nss, 1);
      delete [] hvector;

			for (uint32_t i = 0; i < supported; i++)
			{
				WifiMode mode;
				if(VHT)
					mode = AcMcsToWifiMode(GetMcsSupported (st, i), bw);
				else if(HT)
					mode = McsToWifiMode(GetMcsSupported (st, i));
				else
					mode = GetSupported (st, i);
        
        txVector.SetMode(mode);
        m_currentSnr = receiverYansWifiPhy->GetInterferenceHelper().CalculateSnr (rxPower, 0, txVector);

				double threshold = GetSnrThreshold (mode);
				if (threshold > maxThreshold
						&& threshold < m_currentSnr)
				{
					maxThreshold = threshold;
					maxMode = mode;
				}
			}
			results[0] = maxMode;
			NS_LOG_DEBUG(nss << "-streams, " << "estimated snr=" << 10* std::log10(m_currentSnr));
		}
		else if(nss >= 2)
		{
			NS_ASSERT_MSG(HasHtSupported() || HasVhtSupported(), 
					"# of antenna should be smmaller than 2 for 11a");
			std::complex<double> * hvector = new std::complex<double> [nss*nss+1];
			hvector[0].real() = txPowerDbm;
			hvector = channel->GetPropagationLossModel()->CalcRxPower (hvector, senderMobility, receiverMobility, nss, NULL);
			double rxPower = hvector[0].real() + 1; //m_rxGain
			rxPower = pow(10.0, rxPower/10.0)/bwLoss/1000;
			WifiMode maxMode;
			if(HasVhtSupported())
				maxMode = AcMcsToWifiMode(GetDefaultMcs(), bw);
			else
				maxMode = McsToWifiMode(GetDefaultMcs());
			txVector.SetNss(nss);
			txVector.SetChannelMatrix(hvector, nss, 1);
      delete [] hvector;
      uint16_t mcsMax = GetNMcsSupported(st);	

      for (uint16_t mcs = 0; mcs < mcsMax; mcs++)
      {
        //802.11ac call interference helper 
        WifiMode mode;
        if(HasVhtSupported())
          mode = AcMcsToWifiMode(GetMcsSupported (st, mcs), bw);
        else
          mode = McsToWifiMode(GetMcsSupported (st, mcs));
        txVector.SetMode(mode);
        m_currentSnr = receiverYansWifiPhy->GetInterferenceHelper().CalculateSnr (rxPower, 0, txVector);
        double threshold = GetSnrThreshold (mode);
        if(threshold > maxThreshold && threshold < m_currentSnr)
        {
          maxThreshold = threshold;
          maxMode = mode;
        }
        results[nss-1] = maxMode;
      }

			NS_LOG_DEBUG(nss << "-streams, " << "estimated snr=" << 10* std::log10(m_currentSnr));
    }
	}
	WifiMode final = results[0];
	uint16_t final_idx = 0;
	for (uint16_t nss = 0; nss < txAntenna-1; nss++)
	{
		if(results[final_idx].GetDataRate()*(final_idx+1) < results[nss+1].GetDataRate()*(nss+2))
		{
			final = results[nss+1];
			final_idx = nss+1;
		}
	}
	txVector.SetNss(final_idx+1);
	txVector.SetMode(final);
  NS_LOG_DEBUG("final ss=" << final_idx+1);
	return txVector;
}


WifiTxVector
GenieWifiManager::DoGetRtsTxVector (WifiRemoteStation *st)
{
  GenieWifiRemoteStation *station = (GenieWifiRemoteStation *)st;
	uint16_t bw = GetCurrentBandwidth(st);
//	uint16_t txAntenna = Min (GetNumberOfReceiveAntennas (st),GetNumberOfTransmitAntennas());
  //////////////////////////ns3_lecture//////////////////////////////
  // Calculate RTS size
  Ptr<WifiPhy> wifiPhy = GetWifiPhy();
  WifiMacHeader rts;
  rts.SetType (WIFI_MAC_CTL_RTS);
  // Receiver MAC address and PHY instances
  Mac48Address receiverMac48Address = st->m_state->m_address;
  Ptr<WifiPhy> receiverWifiPhy;
  Ptr<YansWifiPhy> receiverYansWifiPhy;
  // Target instances attached to the channel
  Ptr<WifiNetDevice> testWifiNetDevice;
  Mac48Address testMac48Address;
  // Channel instance that this WifiManager is attached to
  Ptr<YansWifiChannel> channel;
  // Find the receiver PHY instances
  uint32_t nDevices = wifiPhy->GetChannel()->GetNDevices();
  for (uint32_t k = 0; k < nDevices; k++)
  {
    testWifiNetDevice = wifiPhy->GetChannel()->GetDevice(k)->GetObject<WifiNetDevice>();
    testMac48Address = testWifiNetDevice->GetMac()->GetAddress();
    if (testMac48Address == receiverMac48Address)
    {
      receiverWifiPhy = testWifiNetDevice->GetPhy();
      break;
    }
  }
  receiverYansWifiPhy = receiverWifiPhy->GetObject<YansWifiPhy>();
  channel = wifiPhy->GetChannel()->GetObject<YansWifiChannel>();
  // Mobility models for calculating pathloss
  Ptr<MobilityModel> receiverMobility = receiverYansWifiPhy->GetMobility()->GetObject<MobilityModel>();
  Ptr<MobilityModel> senderMobility = wifiPhy->GetObject<YansWifiPhy>()->GetMobility()->GetObject<MobilityModel>();
  // Calculate the RX power (ref: YansWifiChannel::GetRxPowerDbm(txPower, senderMobility, receiverMobility))

  // Noise figure >> Noise floor (ref: InterferenceHelper::CalculateSnr)
  double noiseFigure = pow(10.0, receiverYansWifiPhy->GetRxNoiseFigure()/10.0);
  double txPowerDbm = wifiPhy->GetObject<YansWifiPhy>()->GetPowerDbm(GetDefaultTxPowerLevel()) 
										+ wifiPhy->GetObject<YansWifiPhy>()->GetTxGain();
  double noiseFloorDbm = 10.0 * log10(1.3803e-23 * 290.0 * bw * 1000000 * noiseFigure * 1000.0);

  double rxPowerDbm = channel->GetPropagationLossModel()->CalcRxPower(txPowerDbm, 
                senderMobility, receiverMobility) + receiverYansWifiPhy->GetRxGain(); // Need to add the receiver (antenna) gain


  // Expected SNR (without interference) in scalar scale
  m_currentSnr = pow(10.0, (rxPowerDbm - noiseFloorDbm)/10.0);
  //////////////////////////ns3_lecture//////////////////////////////

  // We search within the Basic rate set the mode with the highest
  // snr threshold possible which is smaller than m_lastSnr to
  // ensure correct packet delivery.
  double maxThreshold = 0.0;
  WifiMode maxMode = GetDefaultMode ();
  for (uint32_t i = 0; i < GetNBasicModes (); i++)
    {
      WifiMode mode = GetBasicMode (i);
      double threshold = GetSnrThreshold (mode);
      if (threshold > maxThreshold
          && threshold < m_currentSnr)
        {
          maxThreshold = threshold;
          maxMode = mode;
        }
    }
  return WifiTxVector (maxMode, GetDefaultTxPowerLevel (), GetShortRetryCount (station), GetShortGuardInterval (station), Min (GetNumberOfReceiveAntennas (station),GetNumberOfTransmitAntennas()), GetNumberOfTransmitAntennas (station), GetStbc (station));
}

bool
GenieWifiManager::IsLowLatency (void) const
{
  return true;
}
} // namespace ns3
