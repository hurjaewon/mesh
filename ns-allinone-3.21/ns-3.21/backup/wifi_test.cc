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
#include <ns3/buildings-helper.h>
#include <ns3/hybrid-buildings-propagation-loss-model.h>

#include <iostream>
#include <sstream>
#include <fstream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("TestMeshScript");

int
main (int argc, char *argv[])
{
  double m_totalTime = 5;
  uint16_t m_packetSize = 2000;
  double apGain = 3.0;
  double staGain = 0;
  double ap_x = 0;
  double ap_y = 0;
  //double sta_x = 0;
  //double sta_y = 0;
	double apTxpower = 13.0206;
	double staTxpower = 13.0206;
  bool m_pcap = true;
  bool mac_ampdu = true;
  uint32_t seed = 10;

	/*
	double x_min = 0.0;
	double x_max = 100.0;
	double y_min = 0.0;
	double y_max = 100.0;
	double z_min = 0.0;
	double z_max = 5.0;
	*/
  
  CommandLine cmd;
  cmd.AddValue ("ap_gain", "Tx power gain of mesh stations (dB)", apGain);
  cmd.AddValue ("sta_gain", "Tx power gain of 11n stations (dB)", staGain);
  cmd.AddValue ("ap_txpower", "Tx power of ap stations (dB)", apTxpower);
  cmd.AddValue ("sta_txpower", "Tx power of 11n stations (dB)", staTxpower);
  cmd.AddValue ("ap_x", "x coordinate of AP (m)", ap_x);
  cmd.AddValue ("ap_y", "y coordinate of AP (m)", ap_y);
  //cmd.AddValue ("sta_x", "x coordinate of sta (m)", sta_x);
  //cmd.AddValue ("sta_y", "y coordinate of sta (m)", sta_y);
  cmd.AddValue ("seed", "random seed", seed);
  cmd.Parse (argc, argv);

  RngSeedManager::SetRun(seed);
  
	NodeContainer Nodes, apNodes, staNodes;
  NetDeviceContainer apDevices, staDevices;
  Ipv4InterfaceContainer apInterfaces, staInterfaces;
  WifiHelper wifi = WifiHelper::Default ();;

  apNodes.Create (1);
  staNodes.Create (5);
	Nodes.Add (apNodes);
	Nodes.Add (staNodes);
	//staNodes.Create (1);  

  YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default();
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
	
  wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel"); 
  //wifiChannel.ClearPropagationLoss ();
	//wifiChannel.AddPropagationLoss("ns3::HybridBuildingsPropagationLossModel");
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

  wifiPhy.Set ("TxGain", DoubleValue (apGain), "RxGain", DoubleValue (apGain), "TxPowerStart", DoubleValue (apTxpower), "TxPowerEnd", DoubleValue (apTxpower));
  
  mac.SetType ("ns3::ApWifiMac", "Ssid", SsidValue (ssid1), "BeaconInterval", TimeValue (MicroSeconds (102400)), "BeaconGeneration", BooleanValue (true));
  apDevices = wifi.Install (wifiPhy, mac, apNodes.Get(0));
  
  wifiPhy.Set ("TxGain", DoubleValue (staGain), "RxGain", DoubleValue (staGain), "TxPowerStart", DoubleValue (staTxpower), "TxPowerEnd", DoubleValue (staTxpower));

  mac.SetType ("ns3::StaWifiMac", "Ssid", SsidValue (ssid1), "ActiveProbing", BooleanValue (false));
  staDevices = wifi.Install (wifiPhy, mac, staNodes);

/*	
	Ptr<Building> building1 = CreateObject<Building> ();
	building1->SetBoundaries (Box (x_min, x_max, y_min, y_max, z_min, z_max));
	building1->SetBuildingType (Building::Residential);
	building1->SetNFloors (1);
	building1->SetNRoomsX (1);
  building1->SetNRoomsY (1);
 */

  MobilityHelper apMobility;
  Ptr<ListPositionAllocator> apAlloc = CreateObject<ListPositionAllocator> ();
  apAlloc->Add (Vector (ap_x, ap_y, 1.0));
  apMobility.SetPositionAllocator (apAlloc);
  apMobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  apMobility.Install (apNodes);

  MobilityHelper staMobility;
  Ptr<ListPositionAllocator> staAlloc = CreateObject<ListPositionAllocator> ();
  staAlloc->Add (Vector (3.0, 10.0, 1.0));
	staAlloc->Add (Vector (6.0, 3.3, 1.0));
	staAlloc->Add (Vector (12.0, 6.6, 1.0));
	staAlloc->Add (Vector (15.0, 0.0, 1.0));
	staAlloc->Add (Vector (18.0, 6.6, 1.0));
  //staAlloc->Add (Vector (sta_x, sta_y, 1.0));
  staMobility.SetPositionAllocator (staAlloc);
  staMobility.Install (staNodes);

	//BuildingsHelper::Install (Nodes);

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

  PacketSinkHelper sink3 ("ns3::TcpSocketFactory", InetSocketAddress (staInterfaces.GetAddress(2) , 9));
  ApplicationContainer apps_sink3 = sink3.Install (staNodes.Get(2));

  PacketSinkHelper sink4 ("ns3::TcpSocketFactory", InetSocketAddress (staInterfaces.GetAddress(3) , 9));
  ApplicationContainer apps_sink4 = sink4.Install (staNodes.Get(3));
  
	PacketSinkHelper sink5 ("ns3::TcpSocketFactory", InetSocketAddress (staInterfaces.GetAddress(4) , 9));
  ApplicationContainer apps_sink5 = sink5.Install (staNodes.Get(4));
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
  
  OnOffHelper onoff3 ("ns3::TcpSocketFactory", InetSocketAddress (staInterfaces.GetAddress(2), 9));
  onoff3.SetConstantRate (DataRate ("500Mb/s"), m_packetSize);
  onoff3.SetAttribute ("StopTime", TimeValue (Seconds (m_totalTime)));
  onoff3.SetAttribute ("StartTime", TimeValue (Seconds (1.0)));
  onoff3.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1000]"));
  ApplicationContainer apps_source3 = onoff3.Install (apNodes.Get(0));
	
	OnOffHelper onoff4 ("ns3::TcpSocketFactory", InetSocketAddress (staInterfaces.GetAddress(3), 9));
  onoff4.SetConstantRate (DataRate ("500Mb/s"), m_packetSize);
  onoff4.SetAttribute ("StopTime", TimeValue (Seconds (m_totalTime)));
  onoff3.SetAttribute ("StartTime", TimeValue (Seconds (1.0)));
  onoff3.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1000]"));
  ApplicationContainer apps_source4 = onoff4.Install (apNodes.Get(0));
	
	OnOffHelper onoff5 ("ns3::TcpSocketFactory", InetSocketAddress (staInterfaces.GetAddress(4), 9));
  onoff5.SetConstantRate (DataRate ("500Mb/s"), m_packetSize);
  onoff5.SetAttribute ("StopTime", TimeValue (Seconds (m_totalTime)));
  onoff5.SetAttribute ("StartTime", TimeValue (Seconds (1.0)));
  onoff5.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1000]"));
  ApplicationContainer apps_source5 = onoff5.Install (apNodes.Get(0));
	
	apps_sink1.Start(Seconds (0.0));
  apps_sink1.Stop (Seconds (m_totalTime));

	apps_sink2.Start(Seconds (0.0));
  apps_sink2.Stop (Seconds (m_totalTime));

	apps_sink3.Start(Seconds (0.0));
  apps_sink3.Stop (Seconds (m_totalTime));
	
	apps_sink4.Start(Seconds (0.0));
  apps_sink4.Stop (Seconds (m_totalTime));

	apps_sink5.Start(Seconds (0.0));
  apps_sink5.Stop (Seconds (m_totalTime));
	
	//BuildingsHelper::MakeMobilityModelConsistent ();

	std::cout << "( " << ap_x << ", " << ap_y << " )\tRSSI:";
  Simulator::Stop(Seconds (m_totalTime + 1));
  Simulator::Run ();
  Simulator::Destroy ();

  uint32_t StaBytesRecv1 = DynamicCast<PacketSink> (apps_sink1.Get (0))->GetTotalRx ();
  uint32_t StaBytesRecv2 = DynamicCast<PacketSink> (apps_sink2.Get (0))->GetTotalRx ();
  uint32_t StaBytesRecv3 = DynamicCast<PacketSink> (apps_sink3.Get (0))->GetTotalRx ();
  uint32_t StaBytesRecv4 = DynamicCast<PacketSink> (apps_sink4.Get (0))->GetTotalRx ();
  uint32_t StaBytesRecv5 = DynamicCast<PacketSink> (apps_sink5.Get (0))->GetTotalRx ();
  uint32_t totalBytesRecv = StaBytesRecv1 + StaBytesRecv2 + StaBytesRecv3 + StaBytesRecv4 + StaBytesRecv5;
  double throughput1 = StaBytesRecv1 * 8 / ((m_totalTime - 1) * 1000000);
  double throughput2 = StaBytesRecv2 * 8 / ((m_totalTime - 1) * 1000000);
  double throughput3 = StaBytesRecv3 * 8 / ((m_totalTime - 1) * 1000000);
  double throughput4 = StaBytesRecv4 * 8 / ((m_totalTime - 1) * 1000000);
  double throughput5 = StaBytesRecv5 * 8 / ((m_totalTime - 1) * 1000000);
  double throughput = totalBytesRecv * 8 / ((m_totalTime - 1) * 1000000);
  std::cout << "\tdBm,Throughput:\t" << throughput1 <<  "\t" << throughput2 << "\t" << throughput3 << "\t" <<  throughput4 << "\t" << throughput5 << "\t" << throughput << "\tMbps\n";

  return 0;
}

