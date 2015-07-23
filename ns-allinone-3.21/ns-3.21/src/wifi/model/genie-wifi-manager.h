#ifndef Genie_WIFI_MANAGER_H
#define Genie_WIFI_MANAGER_H

#include <stdint.h>
#include <vector>
#include "wifi-mode.h"
#include "wifi-remote-station-manager.h"

//////////////////////////ns3_lecture//////////////////////////////
#include "wifi-net-device.h"
#include "ns3/object.h"
#include "ns3/ptr.h"
#include "ns3/mac48-address.h"
#include "ns3/packet.h"

namespace ns3 {

class GenieWifiManager : public WifiRemoteStationManager
{
public:
  static TypeId GetTypeId (void);
  GenieWifiManager ();
  virtual ~GenieWifiManager ();

  virtual void SetupPhy (Ptr<WifiPhy> phy);

private:
  // overriden from base class
  virtual WifiRemoteStation* DoCreateStation (void) const;
  virtual void DoReportRxOk (WifiRemoteStation *station,
                             double rxSnr, WifiMode txMode);
  virtual void DoReportRtsFailed (WifiRemoteStation *station);
  virtual void DoReportDataFailed (WifiRemoteStation *station);
  virtual void DoReportRtsOk (WifiRemoteStation *station,
                              double ctsSnr, WifiMode ctsMode, double rtsSnr);
  virtual void DoReportDataOk (WifiRemoteStation *station,
                               double ackSnr, WifiMode ackMode, double dataSnr);
  virtual void DoReportFinalRtsFailed (WifiRemoteStation *station);
  virtual void DoReportFinalDataFailed (WifiRemoteStation *station);
  virtual WifiTxVector DoGetDataTxVector (WifiRemoteStation *station, uint32_t size);
  virtual WifiTxVector DoGetDataTxVector (WifiRemoteStation *station, uint32_t size, uint16_t bw);
  virtual WifiTxVector DoGetRtsTxVector (WifiRemoteStation *station);
  virtual bool IsLowLatency (void) const;

  // return the min snr needed to successfully transmit
  // data with this mode at the specified ber.
  double GetSnrThreshold (WifiMode mode) const;
  void AddModeSnrThreshold (WifiMode mode, double ber);

  typedef std::vector<std::pair<double,WifiMode> > Thresholds;

  double m_ber;
  Thresholds m_thresholds;
  double m_minSnr;
  double m_maxSnr;
  double m_per;
  double m_currentSnr;
};

} // namespace ns3

#endif /* Genie_WIFI_MANAGER_H */
