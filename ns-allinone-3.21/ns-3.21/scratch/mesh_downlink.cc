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
  int m_xSize = 2;
  int m_ySize = 1;
  double m_step = 1;
  double m_randomStart = 0.1;
//  double m_packetInterval = 0.1;
  double m_totalTime = 10;
  uint16_t m_packetSize = 2000;
  uint32_t m_nIfaces = 1;
  bool m_chan = false;
  bool m_pcap = false;
  bool mesh_ampdu = false;
  bool mac_ampdu = false;
  std::string m_stack = "ns3::Dot11sStack";
  std::string m_root = "ff:ff:ff:ff:ff:ff";

  NodeContainer meshnodes, stanodes;
  NetDeviceContainer meshDevices, apDevice1, apDevice2, brDevice1, brDevice2, staDevices;
  Ipv4InterfaceContainer meshinterfaces, apinterface1, apinterface2, stainterfaces;
  MeshHelper mesh;
  WifiHelper wifi = WifiHelper::Default ();;
  BridgeHelper bridge;

  meshnodes.Create (m_ySize*m_xSize);
  stanodes.Create (1);  

  YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default();
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
	
  wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
	wifiChannel.AddPropagationLoss("ns3::Winner2PropagationLossModel");
  wifiChannel.AddPropagationLoss("ns3::JakesPropagationLossModel");

  //wifiPhy.Set("Transmitters", UintegerValue(2));
  //wifiPhy.Set("Receivers", UintegerValue(2));

  int bandwidth = 20;

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

  mac.SetType ("ns3::ApWifiMac", "Ssid", SsidValue (ssid1), "BeaconInterval", TimeValue (MicroSeconds (102400)), "BeaconGeneration", BooleanValue (true));
  apDevice1 = wifi.Install (wifiPhy, mac, meshnodes.Get(0));
  brDevice1 = bridge.Install (meshnodes.Get(0), NetDeviceContainer(apDevice1.Get(0), meshDevices.Get(0)));

  mac.SetType ("ns3::ApWifiMac", "Ssid", SsidValue (ssid2), "BeaconInterval", TimeValue (MicroSeconds (102400)), "BeaconGeneration", BooleanValue (true));
  apDevice2 = wifi.Install (wifiPhy, mac, meshnodes.Get(1));
  brDevice2 = bridge.Install (meshnodes.Get(1), NetDeviceContainer(apDevice2.Get(0), meshDevices.Get(1)));

  mac.SetType ("ns3::StaWifiMac", "Ssid", SsidValue (ssid2), "ActiveProbing", BooleanValue (false));
  staDevices = wifi.Install (wifiPhy, mac, stanodes);

  MobilityHelper mobility;
  mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
                                 "MinX", DoubleValue (0.0),
                                 "MinY", DoubleValue (0.0),
                                 "DeltaX", DoubleValue (1),
                                 "DeltaY", DoubleValue (1),
                                 "GridWidth", UintegerValue (m_xSize),
                                 "LayoutType", StringValue ("RowFirst"));
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (meshnodes);

  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  positionAlloc->Add (Vector (2 * m_step, 0.0, 0.0));
  mobility.SetPositionAllocator (positionAlloc);
  mobility.Install (stanodes);

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

  Ptr<Ipv4> ipv4sta = stanodes.Get(0)->GetObject<Ipv4> ();
  Ptr<Ipv4> ipv4mesh1 = meshnodes.Get(0)->GetObject<Ipv4> ();
  Ptr<Ipv4> ipv4mesh2 = meshnodes.Get(1)->GetObject<Ipv4> ();
 
  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  Ptr<Ipv4StaticRouting> staticRoutingSta = ipv4RoutingHelper.GetStaticRouting (ipv4sta);
  staticRoutingSta->AddHostRouteTo (Ipv4Address ("10.1.1.1"), Ipv4Address ("192.168.2.1"), 1);
  
  Ptr<Ipv4StaticRouting> staticRoutingMesh1 = ipv4RoutingHelper.GetStaticRouting (ipv4mesh1);
  staticRoutingMesh1->AddHostRouteTo (Ipv4Address ("192.168.2.2"), Ipv4Address ("10.1.1.2"), 1);  

  PacketSinkHelper sink ("ns3::UdpSocketFactory", InetSocketAddress (stainterfaces.GetAddress(0) , 9));
  ApplicationContainer apps_sink = sink.Install (stanodes.Get(0));

  OnOffHelper onoff ("ns3::UdpSocketFactory", InetSocketAddress (stainterfaces.GetAddress(0), 9));
  onoff.SetConstantRate (DataRate ("500Mb/s"), m_packetSize);
  onoff.SetAttribute ("StartTime", TimeValue (Seconds (1.0)));
  onoff.SetAttribute ("StopTime", TimeValue (Seconds (m_totalTime)));
  onoff.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1000]"));
  ApplicationContainer apps_source = onoff.Install (meshnodes.Get(0));

  apps_sink.Start(Seconds (0.0));
  apps_sink.Stop (Seconds (m_totalTime));

/*
  UdpServerHelper myServer(9);
  ApplicationContainer serverApp = myServer.Install (nodes.Get(0));
  serverApp.Start (Seconds (0.0));
  serverApp.Stop (Seconds (m_totalTime + 1));

  UdpClientHelper myClient (interfaces.GetAddress(0), 9);
  myClient.SetAttribute("MaxPackets", UintegerValue(m_totalTime * (1/m_packetInterval))); 
  myClient.SetAttribute("Interval", TimeValue(Seconds(m_packetInterval)));
  myClient.SetAttribute("PacketSize", UintegerValue (m_packetSize));

  ApplicationContainer clientApp = myClient.Install (nodes.Get(m_xSize * m_ySize - 1));
  clientApp.Start (Seconds (1.0));
  clientApp.Stop (Seconds (m_totalTime));
*/
  Simulator::Stop(Seconds (m_totalTime + 1));
  Simulator::Run ();
  Simulator::Destroy ();

  uint32_t totalBytesRecv = DynamicCast<PacketSink> (apps_sink.Get (0))->GetTotalRx ();
  double throughput = totalBytesRecv * 8 / ((m_totalTime - 1) * 1000000);
  std::cout << "Throughput: " << throughput << " Mbps" << "\n";
  std::cout << "TotalBytesRecv: " << totalBytesRecv << "\n";  

  unsigned n (0);
  for (NetDeviceContainer::Iterator i = meshDevices.Begin(); i != meshDevices.End(); ++i, ++n)
  {
    std::ostringstream os;
    os << "mp-report-" << n << ".xml";
    std::cerr << "Printing mesh point device #" << n << " diagnostics to " << os.str () << "\n";
    std::ofstream of;
    of.open (os.str ().c_str ());
    if (!of.is_open ())
    {
      std::cerr << "Error: Can't open file " << os.str () << "\n";
      break;
    }
    mesh.Report (*i, of);
    of.close ();
  }
  return 0;
}

