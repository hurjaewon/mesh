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
	//about mesh nodes
  double m_totalTime = 30.1;
	double m_randomStart = 0.1;
	uint32_t m_nIfaces = 1;
  uint16_t m_packetSize = 1460;
	bool m_chan = false;
	std::string m_stack = "ns3::Dot11sStack";
	std::string m_root = "ff:ff:ff:ff:ff:ff";
	
	//about stations
	double apGain = 3.0;
  double staGain = 0;
	double mpp_x = 0;
  double mpp_y = 0;
	double map_x = 0;
	double map_y = 0;
	double apTxpower = 13.0206;
	double staTxpower = 13.0206;
  bool m_pcap = true;
	bool mesh_ampdu = true;
  bool mac_ampdu = true;
  uint32_t seed = 10;

	CommandLine cmd;
  cmd.AddValue ("ap_gain", "Tx power gain of mesh stations (dB)", apGain);
  cmd.AddValue ("sta_gain", "Tx power gain of 11n stations (dB)", staGain);
  cmd.AddValue ("ap_txpower", "Tx power of ap stations (dB)", apTxpower);
  cmd.AddValue ("sta_txpower", "Tx power of 11n stations (dB)", staTxpower);
  cmd.AddValue ("mpp_x", "x coordinate of MPP (m)", mpp_x);
  cmd.AddValue ("mpp_y", "y coordinate of MPP (m)", mpp_y);
  cmd.AddValue ("map_x", "y coordinate of MAP (m)", map_x);
  cmd.AddValue ("map_y", "y coordinate of MAP (m)", map_y);
  cmd.AddValue ("seed", "random seed", seed);
  cmd.Parse (argc, argv);
	
	double mesh_pos[2][3] = {mpp_x, mpp_y, 1.0, map_x, map_y, 1.0};
	double sta_pos[5][3] = {3.0, 10.0, 1.0, 6.0, 3.3, 1.0, 12.0, 6.6, 1.0, 15.0, 0.0, 1.0, 18.0, 6.6, 1.0};

	RngSeedManager::SetRun(seed);
  
	NodeContainer Nodes, meshNodes, staNodes, staNodes1, staNodes2;
  NetDeviceContainer meshDevices, apDevice1, apDevice2, brDevice1, brDevice2, staDevices1, staDevices2;
  Ipv4InterfaceContainer meshInterfaces, apInterface1, apInterface2, staInterfaces1, staInterfaces2;
	ApplicationContainer apps_sink, apps_source;
	MeshHelper mesh;
  WifiHelper wifi = WifiHelper::Default ();;
	BridgeHelper bridge;

	meshNodes.Create (2);
	staNodes.Create (5);

	Nodes.Add (meshNodes);
	Nodes.Add (staNodes);

  YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default();
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
	
  wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel"); 
  wifiChannel.AddPropagationLoss("ns3::JakesPropagationLossModel");

  int bandwidth = 20;

  wifiPhy.SetChannel (wifiChannel.Create());
  Config::SetDefault ("ns3::YansWifiPhy::ChannelWidth", UintegerValue(bandwidth));

	mesh = MeshHelper::Default();
  mesh.SetStandard(WIFI_PHY_STANDARD_80211n_5GHZ);
  mesh.SetRemoteStationManager ("ns3::MinstrelWifiManager");

	//define mesh nodes
	if (!Mac48Address (m_root.c_str()).IsBroadcast())
	{
		mesh.SetStackInstaller (m_stack, "Root", Mac48AddressValue (Mac48Address (m_root.c_str())));
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

	if (mesh_ampdu)
	{
		mesh.SetImplicitBlockAckRequestForAc (AC_BE, true);
		mesh.SetBlockAckThresholdForAc (AC_BE, 1);
		mesh.SetMpduAggregatorForAc (AC_BE, "ns3::MpduStandardAggregator", "MaxAmpduSize", UintegerValue (65535));
		mesh.SetMaxPpduTime (AC_BE, MilliSeconds (10));
	}

  wifiPhy.Set ("TxGain", DoubleValue (apGain), "RxGain", DoubleValue (apGain), "TxPowerStart", DoubleValue (apTxpower), "TxPowerEnd", DoubleValue (apTxpower));
	meshDevices = mesh.Install (wifiPhy, meshNodes);

	wifi.SetStandard(WIFI_PHY_STANDARD_80211n_5GHZ);
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

  wifiPhy.Set ("TxGain", DoubleValue (apGain), "RxGain", DoubleValue (apGain), "TxPowerStart", DoubleValue (apTxpower), "TxPowerEnd", DoubleValue (apTxpower), "ChannelNumber", UintegerValue(40));
  mac.SetType ("ns3::ApWifiMac", "Ssid", SsidValue (ssid1), "BeaconInterval", TimeValue (MicroSeconds (102400)), "BeaconGeneration", BooleanValue (true));
  apDevice1 = wifi.Install (wifiPhy, mac, meshNodes.Get(0));
  brDevice1 = bridge.Install (meshNodes.Get(0), NetDeviceContainer (apDevice1.Get(0), meshDevices.Get(0)));

	wifiPhy.Set("ChannelNumber", UintegerValue(44));
	mac.SetType ("ns3::ApWifiMac", "Ssid", SsidValue (ssid2), "BeaconInterval", TimeValue (MicroSeconds (102400)), "BeaconGeneration", BooleanValue (true));
	apDevice2 = wifi.Install (wifiPhy, mac, meshNodes.Get(1));
  brDevice2 = bridge.Install (meshNodes.Get(1), NetDeviceContainer (apDevice2.Get(0), meshDevices.Get(1)));


  wifiPhy.Set ("TxGain", DoubleValue (staGain), "RxGain", DoubleValue (staGain), "TxPowerStart", DoubleValue (staTxpower), "TxPowerEnd", DoubleValue (staTxpower));

	//if station is close from MPP, set ssid ssid1, MAP, set ssid ssid2
	for (int i = 0; i < 5; i++)
	{
		double dist_mpp = (mesh_pos[0][0] - sta_pos[i][0]) * (mesh_pos[0][0] - sta_pos[i][0]) + (mesh_pos[0][1] - sta_pos[i][1]) * (mesh_pos[0][1] - sta_pos[i][1]);
		double dist_map = (mesh_pos[1][0] - sta_pos[i][0]) * (mesh_pos[1][0] - sta_pos[i][0]) + (mesh_pos[1][1] - sta_pos[i][1]) * (mesh_pos[1][1] - sta_pos[i][1]);
		if (dist_mpp < dist_map)//distance from MPP is shorter than distance from MAP
		{
			wifiPhy.Set("ChannelNumber", UintegerValue(40));
			mac.SetType ("ns3::StaWifiMac", "Ssid", SsidValue (ssid1), "ActiveProbing", BooleanValue (false));
			staDevices1.Add (wifi.Install (wifiPhy, mac, staNodes.Get(i)));
			staNodes1.Add(staNodes.Get(i));
		}
		else
		{
			wifiPhy.Set("ChannelNumber", UintegerValue(44));
			mac.SetType ("ns3::StaWifiMac", "Ssid", SsidValue (ssid2), "ActiveProbing", BooleanValue (false));
			staDevices2.Add (wifi.Install (wifiPhy, mac, staNodes.Get(i)));
			staNodes2.Add(staNodes.Get(i));
		}
	}

  MobilityHelper meshMobility;
  Ptr<ListPositionAllocator> meshAlloc = CreateObject<ListPositionAllocator> ();
	for (int i = 0; i < 2; i++)
	{
  	meshAlloc->Add (Vector (mesh_pos[i][0], mesh_pos[i][1], mesh_pos[i][2]));
	}
	meshMobility.SetPositionAllocator (meshAlloc);
  meshMobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  meshMobility.Install (meshNodes);

  MobilityHelper staMobility;
  Ptr<ListPositionAllocator> staAlloc = CreateObject<ListPositionAllocator> ();
  for (int i = 0; i < 5; i++)
	{
		staAlloc->Add (Vector (sta_pos[i][0], sta_pos[i][1], sta_pos[i][2]));
	}
  staMobility.SetPositionAllocator (staAlloc);
  staMobility.Install (staNodes);
	
  if (m_pcap)
    wifiPhy.EnablePcapAll (std::string ("mp-"));

  InternetStackHelper internetStack;
  internetStack.Install (meshNodes);
  internetStack.Install (staNodes);

  Ipv4AddressHelper meshAddress, apAddress1, apAddress2;
	meshAddress.SetBase ("10.1.1.0", "255.255.255.0");
	meshInterfaces = meshAddress.Assign (meshDevices);

  apAddress1.SetBase ("192.168.1.0", "255.255.255.0");
	apAddress2.SetBase ("192.168.2.0", "255.255.255.0");

  apInterface1 = apAddress1.Assign (apDevice1);
  staInterfaces1 = apAddress1.Assign (staDevices1);
  apInterface2 = apAddress2.Assign (apDevice2);
  staInterfaces2 = apAddress2.Assign (staDevices2);

	Ptr<Ipv4> ipv4mesh1 = meshNodes.Get(0)->GetObject<Ipv4> ();
	Ptr<Ipv4> ipv4mesh2 = meshNodes.Get(1)->GetObject<Ipv4> ();

	Ipv4StaticRoutingHelper ipv4RoutingHelper;
	Ptr<Ipv4StaticRouting> staticRoutingMesh1 = ipv4RoutingHelper.GetStaticRouting (ipv4mesh1);
	staticRoutingMesh1->AddNetworkRouteTo (Ipv4Address ("192.168.2.0"), Ipv4Mask("255.255.255.0"), Ipv4Address("10.1.1.2"), 1);

	Ptr <Ipv4> ipv4sta;
	Ptr <Ipv4StaticRouting> staticRoutingSta;

	for (int i = 0; i < (int)staNodes1.GetN(); i++)
	{
		ipv4sta = staNodes1.Get(i)->GetObject<Ipv4> ();
		staticRoutingSta = ipv4RoutingHelper.GetStaticRouting (ipv4sta);
		staticRoutingSta->AddHostRouteTo (Ipv4Address ("10.1.1.1"), Ipv4Address ("192.168.1.1"), 1);
	}
	for (int i = 0; i < (int)staNodes2.GetN(); i++)
	{
		ipv4sta = staNodes2.Get(i)->GetObject<Ipv4> ();
		staticRoutingSta = ipv4RoutingHelper.GetStaticRouting (ipv4sta);
		staticRoutingSta->AddHostRouteTo (Ipv4Address ("10.1.1.1"), Ipv4Address ("192.168.2.1"), 1);
	}

	for (int i = 0; i < (int)staNodes1.GetN(); i++)
	{
		PacketSinkHelper sink ("ns3::TcpSocketFactory", InetSocketAddress (staInterfaces1.GetAddress (i), 9));
		apps_sink.Add (sink.Install (staNodes1.Get(i)));
	}

	for (int i = 0; i < (int)staNodes2.GetN(); i++)
	{
		PacketSinkHelper sink ("ns3::TcpSocketFactory", InetSocketAddress (staInterfaces2.GetAddress (i), 9));
		apps_sink.Add (sink.Install (staNodes2.Get(i)));
	}

	for (int i = 0; i < (int)staNodes1.GetN(); i++)
	{
		OnOffHelper onoff ("ns3::TcpSocketFactory", InetSocketAddress (staInterfaces1.GetAddress (i), 9));;
		onoff.SetConstantRate (DataRate ("12Mb/s"), m_packetSize);
  	onoff.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1000]"));
		onoff.SetAttribute ("StartTime", TimeValue (Seconds (1.0)));
  	onoff.SetAttribute ("StopTime", TimeValue (Seconds (m_totalTime)));
		apps_source.Add (onoff.Install (meshNodes.Get(0)));
	}

	for (int i = 0; i < (int)staNodes2.GetN(); i++)
	{
		OnOffHelper onoff ("ns3::TcpSocketFactory", InetSocketAddress (staInterfaces2.GetAddress (i), 9));;
		onoff.SetConstantRate (DataRate ("500Mb/s"), m_packetSize);
  	onoff.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1000]"));
		onoff.SetAttribute ("StartTime", TimeValue (Seconds (1.0)));
  	onoff.SetAttribute ("StopTime", TimeValue (Seconds (m_totalTime)));
		apps_source.Add (onoff.Install (meshNodes.Get(0)));
	}

	apps_sink.Start(Seconds (1.0));
  apps_sink.Stop (Seconds (m_totalTime + 1));

	std::cout << "( " << map_x << ", " << map_y << " )\tRSSI:";
  Simulator::Stop(Seconds (m_totalTime + 1));
  Simulator::Run ();
  Simulator::Destroy ();

  uint32_t StaBytesRecv1 = DynamicCast<PacketSink> (apps_sink.Get (0))->GetTotalRx ();
  uint32_t StaBytesRecv2 = DynamicCast<PacketSink> (apps_sink.Get (1))->GetTotalRx ();
  uint32_t StaBytesRecv3 = DynamicCast<PacketSink> (apps_sink.Get (2))->GetTotalRx ();
  uint32_t StaBytesRecv4 = DynamicCast<PacketSink> (apps_sink.Get (3))->GetTotalRx ();
  uint32_t StaBytesRecv5 = DynamicCast<PacketSink> (apps_sink.Get (4))->GetTotalRx ();
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

