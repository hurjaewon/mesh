#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mesh-module.h"
#include "ns3/mobility-module.h"
#include "ns3/mesh-helper.h"
#include "ns3/bridge-helper.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/log.h"

#include <iostream>
#include <sstream>
#include <fstream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("TestMeshScript");

int
main (int argc, char *argv[])
{
  double m_totalTime = 10;
  uint16_t m_packetSize = 2000;
  double apGain = 5.0;
  double staGain = 0;
  double m_dist = 15;
  bool m_pcap = true;
  bool mac_ampdu = true;
  
  CommandLine cmd;
  cmd.AddValue ("ap_gain", "Tx power gain of mesh stations (dB)", apGain);
  cmd.AddValue ("sta_gain", "Tx power gain of 11n stations (dB)", staGain);
  cmd.AddValue ("dist", "Distance between ap and stations (m)", m_dist);
  cmd.Parse (argc, argv);

  NodeContainer apNodes, staNodes;
  NetDeviceContainer apDevices, staDevices;
  Ipv4InterfaceContainer apInterfaces, staInterfaces;
  WifiHelper wifi = WifiHelper::Default ();;

  apNodes.Create (1);
  staNodes.Create (2);  

  YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default();
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
	
  wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel"); 
  wifiChannel.AddPropagationLoss("ns3::JakesPropagationLossModel");

  int bandwidth = 20;

  wifiPhy.SetChannel (wifiChannel.Create());
  Config::SetDefault ("ns3::YansWifiPhy::ChannelWidth", UintegerValue(bandwidth));

  wifi.SetStandard(WIFI_PHY_STANDARD_80211n_5GHZ);

  wifi.SetRemoteStationManager ("ns3::MinstrelWifiManager");

  HtWifiMacHelper mac = HtWifiMacHelper::Default();
  Ssid ssid1 = Ssid ("Section1");
  
  if (mac_ampdu)
  {
    mac.SetImplicitBlockAckRequestForAc (AC_BE, true);
    mac.SetBlockAckThresholdForAc (AC_BE, 1);
 
    mac.SetMpduAggregatorForAc (AC_BE, "ns3::MpduStandardAggregator", "MaxAmpduSize", UintegerValue(65535));
    mac.SetMaxPpduTime (AC_BE, MilliSeconds (10));
 }

  wifiPhy.Set ("TxGain", DoubleValue (apGain));
  
  mac.SetType ("ns3::ApWifiMac", "Ssid", SsidValue (ssid1), "BeaconInterval", TimeValue (MicroSeconds (102400)), "BeaconGeneration", BooleanValue (true));
  apDevices = wifi.Install (wifiPhy, mac, apNodes.Get(0));
  
  wifiPhy.Set ("TxGain", DoubleValue (staGain));

  mac.SetType ("ns3::StaWifiMac", "Ssid", SsidValue (ssid1), "ActiveProbing", BooleanValue (false));
  staDevices = wifi.Install (wifiPhy, mac, staNodes);

  MobilityHelper apMobility;
  Ptr<ListPositionAllocator> apAlloc = CreateObject<ListPositionAllocator> ();
  apAlloc->Add (Vector (0.0, 0.0, 0.0));
  apMobility.SetPositionAllocator (apAlloc);
  apMobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  apMobility.Install (apNodes);

  MobilityHelper staMobility;
  Ptr<ListPositionAllocator> staAlloc = CreateObject<ListPositionAllocator> ();
  staAlloc->Add (Vector (m_dist, 0.0, 0.0));
  staAlloc->Add (Vector (m_dist, 4.5, 0.0));
  staMobility.SetPositionAllocator (staAlloc);
  staMobility.Install (staNodes);

  if (m_pcap)
    wifiPhy.EnablePcapAll (std::string ("mp-"));

  InternetStackHelper internetStack;
  internetStack.Install (apNodes);
  internetStack.Install (staNodes);

  Ipv4AddressHelper apAddress, staAddress;
  apAddress.SetBase ("192.168.1.0", "255.255.255.0");
  apInterfaces = apAddress.Assign (apDevices);
  staInterfaces = apAddress.Assign (staDevices);

  PacketSinkHelper sink1 ("ns3::TcpSocketFactory", InetSocketAddress (staInterfaces.GetAddress(0) , 9));
  ApplicationContainer apps_sink1 = sink1.Install (staNodes.Get(0));
  PacketSinkHelper sink2 ("ns3::TcpSocketFactory", InetSocketAddress (staInterfaces.GetAddress(1) , 9));
  ApplicationContainer apps_sink2 = sink2.Install (staNodes.Get(1));

  OnOffHelper onoff1 ("ns3::TcpSocketFactory", InetSocketAddress (staInterfaces.GetAddress(0), 9));
  onoff1.SetConstantRate (DataRate ("500Mb/s"), m_packetSize);
  onoff1.SetAttribute ("StartTime", TimeValue (Seconds (1.0)));
  onoff1.SetAttribute ("StopTime", TimeValue (Seconds (m_totalTime)));
  onoff1.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1000]"));
  ApplicationContainer apps_source1 = onoff1.Install (apNodes.Get(0));

  OnOffHelper onoff2 ("ns3::TcpSocketFactory", InetSocketAddress (staInterfaces.GetAddress(1), 9));
  onoff2.SetConstantRate (DataRate ("500Mb/s"), m_packetSize);
  onoff2.SetAttribute ("StopTime", TimeValue (Seconds (m_totalTime)));
  onoff2.SetAttribute ("StartTime", TimeValue (Seconds (1.0)));
  onoff2.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1000]"));
  ApplicationContainer apps_source2 = onoff2.Install (apNodes.Get(0));

  apps_sink1.Start(Seconds (0.0));
  apps_sink1.Stop (Seconds (m_totalTime));
  apps_sink2.Start(Seconds (0.0));
  apps_sink2.Stop (Seconds (m_totalTime));

  Simulator::Stop(Seconds (m_totalTime + 1));
  Simulator::Run ();
  Simulator::Destroy ();

  uint32_t StaBytesRecv1 = DynamicCast<PacketSink> (apps_sink1.Get (0))->GetTotalRx ();
  uint32_t StaBytesRecv2 = DynamicCast<PacketSink> (apps_sink2.Get (0))->GetTotalRx ();
  uint32_t totalBytesRecv = StaBytesRecv1 + StaBytesRecv2;
  double throughput = totalBytesRecv * 8 / ((m_totalTime - 1) * 1000000);
  std::cout << "Throughput: " << throughput << " Mbps" << "\n";
  std::cout << "TotalBytesRecv: " << totalBytesRecv << "\n";  

  return 0;
}

