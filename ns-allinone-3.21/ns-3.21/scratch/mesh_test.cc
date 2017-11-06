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
  double m_step = 10;
  double m_randomStart = 0.1;
//  double m_packetInterval = 0.1;
  double m_totalTime = 10;
  uint16_t m_packetSize = 2000;
  uint32_t m_nIfaces = 1;
  double meshGain = 3;
  double apGain = 3;
  double staGain = 0;
  double m_dist = 15;
  bool m_chan = false;
  bool m_pcap = true;
  bool mesh_ampdu = true;
  bool mac_ampdu = true;
  std::string m_stack = "ns3::Dot11sStack";
  std::string m_root = "ff:ff:ff:ff:ff:ff";
  
  CommandLine cmd;
  cmd.AddValue ("step", "Size of edge in our grid (meters)", m_step);
  cmd.AddValue ("mesh_gain", "Tx power gain of mesh stations (dB)", meshGain);
  cmd.AddValue ("ap_gain", "Tx power gain of ap stations (dB)", apGain);
  cmd.AddValue ("sta_gain", "Tx power gain of 11n stations (dB)", staGain);
  cmd.AddValue ("dist", "Distance between mesh root and stations (meters)", m_dist);
  cmd.Parse (argc, argv);

  NodeContainer meshnodes, stanodes;
  NetDeviceContainer meshDevices, apDevice1, apDevice2, brDevice1, brDevice2, staDevices;
  Ipv4InterfaceContainer meshinterfaces, apinterface1, apinterface2, stainterfaces;
  MeshHelper mesh;
  WifiHelper wifi = WifiHelper::Default ();;
  BridgeHelper bridge;

  meshnodes.Create (2);
  stanodes.Create (2);  

  YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default();
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
	
  wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel"); 
  wifiChannel.AddPropagationLoss("ns3::JakesPropagationLossModel");

  wifiPhy.Set("Transmitters", UintegerValue(2));
  wifiPhy.Set("Receivers", UintegerValue(2));

  int bandwidth = 20;

  wifiPhy.Set ("TxGain", DoubleValue (meshGain));
  wifiPhy.SetChannel (wifiChannel.Create());
  Config::SetDefault ("ns3::YansWifiPhy::ChannelWidth", UintegerValue(bandwidth));

  mesh = MeshHelper::Default();
  mesh.SetStandard(WIFI_PHY_STANDARD_80211n_5GHZ);
  mesh.SetRemoteStationManager ("ns3::MinstrelWifiManager");
  //mesh.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "DataMode", StringValue("OfdmRate65MbpsBW20MHz"), "ControlMode", StringValue("OfdmRate24Mbps"));

  if (!Mac48Address (m_root.c_str()).IsBroadcast ())
  {
    mesh.SetStackInstaller (m_stack, "Root", Mac48AddressValue (Mac48Address (m_root.c_str ())));
  }
  else
  {
    mesh.SetStackInstaller (m_stack);
  }
  if (m_chan)
  {
    mesh.SetSpreadInterfaceChannels (MeshHelper::SPREAD_CHANNELS);
  }
  else
  {
    mesh.SetSpreadInterfaceChannels (MeshHelper::ZERO_CHANNEL);
  }
  mesh.SetMacType("RandomStart", TimeValue (Seconds (m_randomStart)));
  mesh.SetNumberOfInterfaces (m_nIfaces);

  //jwhur ampdu
  if (mesh_ampdu)
  {
    mesh.SetImplicitBlockAckRequestForAc (AC_BE, true);
    mesh.SetBlockAckThresholdForAc (AC_BE, 1);
    mesh.SetMpduAggregatorForAc (AC_BE, "ns3::MpduStandardAggregator", "MaxAmpduSize", UintegerValue (65535));
    mesh.SetMaxPpduTime (AC_BE, MilliSeconds (10));
  }

  meshDevices = mesh.Install(wifiPhy, meshnodes);

  wifi.SetStandard(WIFI_PHY_STANDARD_80211n_5GHZ);


  //wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "DataMode", StringValue("OfdmRate65MbpsBW20MHz"), "ControlMode", StringValue("OfdmRate24Mbps"));
  wifi.SetRemoteStationManager ("ns3::MinstrelWifiManager");

  HtWifiMacHelper mac = HtWifiMacHelper::Default();
  Ssid ssid1 = Ssid ("Section1");
  Ssid ssid2 = Ssid ("Section2");
  if (mac_ampdu)
  {
    mac.SetImplicitBlockAckRequestForAc (AC_BE, true);
    mac.SetBlockAckThresholdForAc (AC_BE, 1);
 
    mac.SetMpduAggregatorForAc (AC_BE, "ns3::MpduStandardAggregator", "MaxAmpduSize", UintegerValue(65535));
    mac.SetMaxPpduTime (AC_BE, MilliSeconds (10));
 }

  wifiPhy.Set ("TxGain", DoubleValue (apGain));
  
  mac.SetType ("ns3::ApWifiMac", "Ssid", SsidValue (ssid1), "BeaconInterval", TimeValue (MicroSeconds (102400)), "BeaconGeneration", BooleanValue (true));
  apDevice1 = wifi.Install (wifiPhy, mac, meshnodes.Get(0));
  brDevice1 = bridge.Install (meshnodes.Get(0), NetDeviceContainer(apDevice1.Get(0), meshDevices.Get(0)));

  mac.SetType ("ns3::ApWifiMac", "Ssid", SsidValue (ssid2), "BeaconInterval", TimeValue (MicroSeconds (102400)), "BeaconGeneration", BooleanValue (true));
  apDevice2 = wifi.Install (wifiPhy, mac, meshnodes.Get(1));
  brDevice2 = bridge.Install (meshnodes.Get(1), NetDeviceContainer(apDevice2.Get(0), meshDevices.Get(1)));

  wifiPhy.Set ("TxGain", DoubleValue (staGain));
  
  mac.SetType ("ns3::StaWifiMac", "Ssid", SsidValue (ssid2), "ActiveProbing", BooleanValue (false));
  staDevices = wifi.Install (wifiPhy, mac, stanodes);

  MobilityHelper Meshmobility;
  Ptr<ListPositionAllocator> meshAlloc = CreateObject<ListPositionAllocator> ();
  meshAlloc->Add (Vector (0.0, 0.0, 0.0));
  meshAlloc->Add (Vector (m_step, 0.0, 0.0));
  Meshmobility.SetPositionAllocator (meshAlloc);
  Meshmobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  Meshmobility.Install (meshnodes);

  MobilityHelper Stamobility;
  Ptr<ListPositionAllocator> staAlloc = CreateObject<ListPositionAllocator> ();
  staAlloc->Add (Vector (m_dist, 0.0, 0.0));
  staAlloc->Add (Vector (m_dist, 4.5, 0.0));
  Stamobility.SetPositionAllocator (staAlloc);
  Stamobility.Install (stanodes);

  if (m_pcap)
    wifiPhy.EnablePcapAll (std::string ("mp-"));

  InternetStackHelper internetStack;
  internetStack.Install (meshnodes);
  internetStack.Install (stanodes);

  Ipv4AddressHelper meshaddress, apaddress1, apaddress2;
  meshaddress.SetBase ("10.1.1.0", "255.255.255.0");
  meshinterfaces = meshaddress.Assign (meshDevices);

  apaddress1.SetBase ("192.168.1.0", "255.255.255.0");
  apaddress2.SetBase ("192.168.2.0", "255.255.255.0");

  apinterface1 = apaddress1.Assign(apDevice1);
  apinterface2 = apaddress2.Assign(apDevice2);
  stainterfaces = apaddress2.Assign(staDevices);

  Ptr<Ipv4> ipv4sta1 = stanodes.Get(0)->GetObject<Ipv4> ();
  Ptr<Ipv4> ipv4sta2 = stanodes.Get(1)->GetObject<Ipv4> ();
  Ptr<Ipv4> ipv4mesh1 = meshnodes.Get(0)->GetObject<Ipv4> ();
  Ptr<Ipv4> ipv4mesh2 = meshnodes.Get(1)->GetObject<Ipv4> ();
 
  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  Ptr<Ipv4StaticRouting> staticRoutingSta1 = ipv4RoutingHelper.GetStaticRouting (ipv4sta1);
  staticRoutingSta1->AddHostRouteTo (Ipv4Address ("10.1.1.1"), Ipv4Address ("192.168.2.1"), 1);
  Ptr<Ipv4StaticRouting> staticRoutingSta2 = ipv4RoutingHelper.GetStaticRouting (ipv4sta2);
  staticRoutingSta2->AddHostRouteTo (Ipv4Address ("10.1.1.1"), Ipv4Address ("192.168.2.1"), 1);
  
  Ptr<Ipv4StaticRouting> staticRoutingMesh1 = ipv4RoutingHelper.GetStaticRouting (ipv4mesh1);
  staticRoutingMesh1->AddHostRouteTo (Ipv4Address ("192.168.2.2"), Ipv4Address ("10.1.1.2"), 1);  
  staticRoutingMesh1->AddHostRouteTo (Ipv4Address ("192.168.2.3"), Ipv4Address ("10.1.1.2"), 1);  

  PacketSinkHelper sink1 ("ns3::TcpSocketFactory", InetSocketAddress (stainterfaces.GetAddress(0) , 9));
  ApplicationContainer apps_sink1 = sink1.Install (stanodes.Get(0));
  PacketSinkHelper sink2 ("ns3::TcpSocketFactory", InetSocketAddress (stainterfaces.GetAddress(1) , 9));
  ApplicationContainer apps_sink2 = sink2.Install (stanodes.Get(1));

  OnOffHelper onoff1 ("ns3::TcpSocketFactory", InetSocketAddress (stainterfaces.GetAddress(0), 9));
  onoff1.SetConstantRate (DataRate ("500Mb/s"), m_packetSize);
  onoff1.SetAttribute ("StartTime", TimeValue (Seconds (1.0)));
  onoff1.SetAttribute ("StopTime", TimeValue (Seconds (m_totalTime)));
  onoff1.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1000]"));
  ApplicationContainer apps_source1 = onoff1.Install (meshnodes.Get(0));

  OnOffHelper onoff2 ("ns3::TcpSocketFactory", InetSocketAddress (stainterfaces.GetAddress(1), 9));
  onoff2.SetConstantRate (DataRate ("500Mb/s"), m_packetSize);
  onoff2.SetAttribute ("StopTime", TimeValue (Seconds (m_totalTime)));
  onoff2.SetAttribute ("StartTime", TimeValue (Seconds (1.0)));
  onoff2.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1000]"));
  ApplicationContainer apps_source2 = onoff2.Install (meshnodes.Get(0));

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
  
  return 0;
}

