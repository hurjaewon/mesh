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
#include "ns3/dash-module.h"

#include <iostream>
#include <sstream>
#include <fstream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("TestMeshScript");

double
calcPl(double tx, double ty, double rx, double ry, int A1, int B1, int A2, int B2, double *wall_loss)
{
	double wall[15][4] = {0, 0, 0, 21.6, 3.6, 0, 3.6, 10.8, 3.6, 13.7, 3.6, 21.6, 9, 5.4, 9, 10.8, 9, 13.7, 9, 21.6, 11.7, 0, 11.7, 5.4, 19.8, 0, 19.8, 10.8, 19.8, 13.7, 19.8, 21.6, 3.6, 0, 19.8, 0, 3.6, 5.4, 19.8, 5.4, 3.6, 10.8, 19.8, 10.8, 3.6, 13.7, 19.8, 13.7, 0, 13.7, 3.6, 13.7, 0, 17.3, 3.6, 17.3, 0, 21.6, 19.8, 21.6};
	double its[15] = {0};

	double m_referenceDistance = 1, m_referenceFrequency = 2.4;
	double distance = std::sqrt(std::pow(tx-rx, 2) + std::pow(ty-ry, 2));
	if (distance <= 1)
	{
		return 0;
	}
	double w1x, w1y, w2x, w2y, compare1, compare2, coeff; //for inclination
	double openpl, wallpl;
	int num_wall = 15, num_intersect = 0;
	for (int i=0; i<num_wall; i++)
	{
		w1x = wall[i][0];
		w1y = wall[i][1];
		w2x = wall[i][2];
		w2y = wall[i][3];
		compare1 = std::abs((w2x - w1x) * (ry - ty) * tx - (w2y - w1y) * (rx - tx) * w1x + (rx - tx) * (w2x - w1x) * (w1y - ty));
		compare2 = std::abs((ry - ty) * (w2x - w1x) * w1y - (rx - tx) * (w2y - w1y) * ty + (ry - ty) * (w2y - w1y) * (tx - w1x));
		coeff =std::abs((ry - ty)*(w2x - w1x) - (w2y - w1y)*(rx - tx));
		if ((coeff * std::min(tx, rx) <= compare1) && (compare1 <= coeff * std::max(tx, rx))) {
			if ((coeff * std::min(w1x, w2x) <= compare1) && (compare1 <= coeff * std::max(w1x, w2x))){
				if ((coeff * std::min(ty, ry) <= compare2) && (compare2 <= coeff * std::max(ty, ry))) {
					if ((coeff * std::min(w1y, w2y) <= compare2) && (compare2 <= coeff * std::max(w1y, w2y))) {
						num_intersect++;
						its[i] = 1;
					}
				}
			}
		}
	}

	if (num_intersect == 0)
	{
		openpl = A1 + B1 * std::log10(distance / m_referenceDistance) + 20 * std::log10(m_referenceFrequency / 5);
	} else {
		openpl = A2 + B2 * std::log10(distance / m_referenceDistance) + 20 * std::log10(m_referenceFrequency / 5);
	}

	wallpl = 0;
	for (int i=0; i<num_wall; i++) {
		if (its[i] == 1)
			wallpl += wall_loss[i];
	}
	double pathLossDb = openpl + wallpl;
	return pathLossDb;
}


int
main (int argc, char *argv[])
{
	//about mesh nodes
  double m_totalTime = 60;
	double m_randomStart = 0.1;
	uint32_t m_nIfaces = 1;
 // uint16_t m_packetSize = 1460;
	bool m_chan = false;
	std::string m_stack = "ns3::Dot11sStack";
	std::string m_root = "ff:ff:ff:ff:ff:ff";
	char coeff_str[] = "180326";

  // for dash 
  double target_dt = 5.0;
  double stopTime = m_totalTime-10;
  //std::string linkRate = "30Mbps";
  std::string delay = "5ms";
  std::string protocol = "ns3::FdashClient";
  std::string window = "10s";
	
  //////////////
   
   
	//about stations
	double apGain = 0;
  double staGain = 0;
	double mpp_x = 4,  mpp_y = 5;
	double outlet[14][2] =	{
		18, 21,
		9.1, 5.5,	
		15, 14.5,
		13, 13, 
		8, 10.5,
		13, 10.5, 
		18, 10.5,
		3, 9, 
		7, 8,
		10, 6, 
		14, 6,
		4, 5,
		11, 5,
		9, 1
	};
	int map1_pos = 0, map2_pos = 1;
	double ap5Txpower = 15;
	double ap2_4Txpower = 13;
	double staTxpower = 13;
  bool m_pcap = false;
	bool mesh_ampdu = true;
  bool mac_ampdu = true;
  uint32_t seed = 10;
	int nsta = 3;
	int mnss = 2;
	double A1, B1, A2, B2;
	double wall_loss[15] = {0};

	//int dr[nsta] = {8, 10};
	std::string dataStr;
	std::string dumi = "Mb/s";

	CommandLine cmd;
  cmd.AddValue ("ap_gain", "Tx power gain of mesh stations (dB)", apGain);
  cmd.AddValue ("sta_gain", "Tx power gain of 11n stations (dB)", staGain);
  //cmd.AddValue ("ap_txpower", "Tx power of ap stations (dB)", apTxpower);
  cmd.AddValue ("sta_txpower", "Tx power of 11n stations (dB)", staTxpower);
	cmd.AddValue ("map1_pos", "Position of MAP1 (num)", map1_pos);
	cmd.AddValue ("map2_pos", "Position of MAP2 (num)", map2_pos);
  cmd.AddValue ("seed", "random seed", seed);
  cmd.AddValue ("simTime", "Simulation time", m_totalTime);
	cmd.AddValue ("coeff_str", "Coefficient string", coeff_str);
	cmd.Parse (argc, argv);


  LogComponentEnable("UdpClient", LOG_LEVEL_INFO);
  LogComponentEnable("UdpServer", LOG_LEVEL_INFO);

  std::ifstream File;
	double inputFromFile;
	//fileStr = "./rssi_cal/Result-" + coeff_str + ".txt"; 
	char fileStr[100] = "./rssi_cal/coef_results/Result-";
	strcat(fileStr, coeff_str);
	strcat(fileStr, ".txt");
	File.open(fileStr);

	File >> inputFromFile;
	Winner2PropagationLossModel::A1 = inputFromFile;
	A1 = inputFromFile;
	File >> inputFromFile;
	Winner2PropagationLossModel::B1 = inputFromFile;
	B1 = inputFromFile;
	File >> inputFromFile;
	Winner2PropagationLossModel::A2 = inputFromFile;
	A2 = inputFromFile;
	File >> inputFromFile;
	Winner2PropagationLossModel::B2 = inputFromFile;
	B2 = inputFromFile;

	int j = 0;
	while(File >> inputFromFile) {
		Winner2PropagationLossModel::wall_loss[j] = inputFromFile;
		wall_loss[j] = inputFromFile;
		j++;
	}
	File.close();

	double map1_x = outlet[map1_pos][0], map1_y = outlet[map1_pos][1];
	double map2_x = outlet[map2_pos][0], map2_y = outlet[map2_pos][1];
	double mesh_pos[3][3] = {mpp_x, mpp_y, 1.0, map1_x, map1_y, 1.0, map2_x, map2_y, 1.0};
	double sta_pos[nsta][3] = {
		10.6, 5.5,	1.0, 
		10.6, 11.8, 1.0};
	
	RngSeedManager::SetRun(seed);

	NodeContainer Nodes, meshNodes, staNodes, staNodes1, staNodes2, staNodes3;
  NetDeviceContainer meshDevices, apDevice1, apDevice2, apDevice3, brDevice1, brDevice2, brDevice3, staDevices1, staDevices2, staDevices3;
  Ipv4InterfaceContainer meshInterfaces, apInterface1, apInterface2, apInterface3, staInterfaces1, staInterfaces2, staInterfaces3;
	ApplicationContainer apps_sink, apps_source;
	MeshHelper mesh;
  WifiHelper wifi = WifiHelper::Default ();;
	BridgeHelper bridge;

	meshNodes.Create (3);
	staNodes.Create (nsta);

	Nodes.Add (meshNodes);
	Nodes.Add (staNodes);

  YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default();
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
	
  wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
	wifiChannel.AddPropagationLoss("ns3::Winner2PropagationLossModel", "Frequency", DoubleValue(5.15));
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

  wifiPhy.Set ("TxGain", DoubleValue (apGain), "RxGain", DoubleValue (apGain), "TxPowerStart", DoubleValue (ap5Txpower), "TxPowerEnd", DoubleValue (ap5Txpower), "Transmitters", UintegerValue(mnss), "Receivers", UintegerValue(mnss));
	meshDevices = mesh.Install (wifiPhy, meshNodes);

	wifi.SetStandard(WIFI_PHY_STANDARD_80211n_2_4GHZ);
	wifi.SetRemoteStationManager ("ns3::MinstrelWifiManager");

  HtWifiMacHelper mac = HtWifiMacHelper::Default();
  Ssid ssid1 = Ssid ("Section1");
  Ssid ssid2 = Ssid ("Section2");
	Ssid ssid3 = Ssid ("Section3");

  if (mac_ampdu)
  {
    mac.SetImplicitBlockAckRequestForAc (AC_BE, true);
    mac.SetBlockAckThresholdForAc (AC_BE, 1);
    mac.SetMpduAggregatorForAc (AC_BE, "ns3::MpduStandardAggregator", "MaxAmpduSize", UintegerValue(65535));
    mac.SetMaxPpduTime (AC_BE, MilliSeconds (10));
 }

	wifiChannel.ClearPropagationLoss();
	wifiChannel.AddPropagationLoss("ns3::Winner2PropagationLossModel", "Frequency", DoubleValue(2.4));
	wifiChannel.AddPropagationLoss("ns3::JakesPropagationLossModel");

	wifiPhy.SetChannel(wifiChannel.Create());

  wifiPhy.Set ("TxGain", DoubleValue (apGain), "RxGain", DoubleValue (apGain), "TxPowerStart", DoubleValue (ap2_4Txpower), "TxPowerEnd", DoubleValue (ap2_4Txpower), "ChannelNumber", UintegerValue(40), "Transmitters", UintegerValue(1), "Receivers", UintegerValue(1));
  mac.SetType ("ns3::ApWifiMac", "Ssid", SsidValue (ssid1), "BeaconInterval", TimeValue (MicroSeconds (102400)), "BeaconGeneration", BooleanValue (true));
  apDevice1 = wifi.Install (wifiPhy, mac, meshNodes.Get(0));
  brDevice1 = bridge.Install (meshNodes.Get(0), NetDeviceContainer (apDevice1.Get(0), meshDevices.Get(0)));

	wifiPhy.Set("ChannelNumber", UintegerValue(44), "Transmitters", UintegerValue(1), "Receivers", UintegerValue(1));
	mac.SetType ("ns3::ApWifiMac", "Ssid", SsidValue (ssid2));
	apDevice2 = wifi.Install (wifiPhy, mac, meshNodes.Get(1));
  brDevice2 = bridge.Install (meshNodes.Get(1), NetDeviceContainer (apDevice2.Get(0), meshDevices.Get(1)));

	wifiPhy.Set ("ChannelNumber", UintegerValue(48), "Transmitters", UintegerValue(1), "Receivers", UintegerValue(1));
	mac.SetType ("ns3::ApWifiMac", "Ssid", SsidValue (ssid3));
	apDevice3 = wifi.Install (wifiPhy, mac, meshNodes.Get(2));
	brDevice3 = bridge.Install (meshNodes.Get(2), NetDeviceContainer (apDevice3.Get(0), meshDevices.Get(2)));

  wifiPhy.Set ("TxGain", DoubleValue (staGain), "RxGain", DoubleValue (staGain), "TxPowerStart", DoubleValue (staTxpower), "TxPowerEnd", DoubleValue (staTxpower));

	//if station is close from MPP, set ssid ssid1, MAP, set ssid ssid2
	for (int i = 0; i < nsta; i++)
	{
		double pl_mpp = calcPl(mesh_pos[0][0], mesh_pos[0][1], sta_pos[i][0], sta_pos[i][1], A1, B1, A2, B2, wall_loss);
		double pl_map1 = calcPl(mesh_pos[1][0], mesh_pos[1][1], sta_pos[i][0], sta_pos[i][1], A1, B1, A2, B2, wall_loss);
		double pl_map2 = calcPl(mesh_pos[2][0], mesh_pos[2][1], sta_pos[i][0], sta_pos[i][1], A1, B1, A2, B2, wall_loss);

		if (pl_mpp < pl_map1) {
			if (pl_mpp < pl_map2) {
				wifiPhy.Set("ChannelNumber", UintegerValue(40));
				mac.SetType ("ns3::StaWifiMac", "Ssid", SsidValue (ssid1), "ActiveProbing", BooleanValue (false));
				staDevices1.Add (wifi.Install (wifiPhy, mac, staNodes.Get(i)));
				staNodes1.Add(staNodes.Get(i));
				std::cout << "mpp\t";
			} else {
				wifiPhy.Set("ChannelNumber", UintegerValue(48));
				mac.SetType ("ns3::StaWifiMac", "Ssid", SsidValue (ssid3), "ActiveProbing", BooleanValue (false));
				staDevices3.Add (wifi.Install (wifiPhy, mac, staNodes.Get(i)));
				staNodes3.Add (staNodes.Get(i));
				std::cout << "map2\t";
			}
		} else {
			if (pl_map1 < pl_map2) {
				wifiPhy.Set("ChannelNumber", UintegerValue(44));
				mac.SetType ("ns3::StaWifiMac", "Ssid", SsidValue (ssid2), "ActiveProbing", BooleanValue (false));
				staDevices2.Add (wifi.Install (wifiPhy, mac, staNodes.Get(i)));
				staNodes2.Add(staNodes.Get(i));
				std::cout << "map1\t";
			} else {
				wifiPhy.Set("ChannelNumber", UintegerValue(48));
				mac.SetType ("ns3::StaWifiMac", "Ssid", SsidValue (ssid3), "ActiveProbing", BooleanValue (false));
				staDevices3.Add (wifi.Install (wifiPhy, mac, staNodes.Get(i)));
				staNodes3.Add (staNodes.Get(i));
				std::cout << "map2\t";
			}
		}
	}

  MobilityHelper meshMobility;
  Ptr<ListPositionAllocator> meshAlloc = CreateObject<ListPositionAllocator> ();
	for (int i = 0; i < 3; i++)
	{
  	meshAlloc->Add (Vector (mesh_pos[i][0], mesh_pos[i][1], mesh_pos[i][2]));
	}
	meshMobility.SetPositionAllocator (meshAlloc);
  meshMobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  meshMobility.Install (meshNodes);

  MobilityHelper staMobility;
  Ptr<ListPositionAllocator> staAlloc = CreateObject<ListPositionAllocator> ();
  for (int i = 0; i < nsta; i++)
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

  Ipv4AddressHelper meshAddress, apAddress1, apAddress2, apAddress3;
	meshAddress.SetBase ("10.1.1.0", "255.255.255.0");
	meshInterfaces = meshAddress.Assign (meshDevices);

  apAddress1.SetBase ("192.168.1.0", "255.255.255.0");
	apAddress2.SetBase ("192.168.2.0", "255.255.255.0");
	apAddress3.SetBase ("192.168.3.0", "255.255.255.0");

  apInterface1 = apAddress1.Assign (apDevice1);
  staInterfaces1 = apAddress1.Assign (staDevices1);
  apInterface2 = apAddress2.Assign (apDevice2);
  staInterfaces2 = apAddress2.Assign (staDevices2);
	apInterface3 = apAddress3.Assign (apDevice3);
	staInterfaces3 = apAddress3.Assign (staDevices3);

	Ptr<Ipv4> ipv4mesh1 = meshNodes.Get(0)->GetObject<Ipv4> ();

	Ipv4StaticRoutingHelper ipv4RoutingHelper;
	Ptr<Ipv4StaticRouting> staticRoutingMesh1 = ipv4RoutingHelper.GetStaticRouting (ipv4mesh1);
	staticRoutingMesh1->AddNetworkRouteTo (Ipv4Address ("192.168.2.0"), Ipv4Mask("255.255.255.0"), Ipv4Address("10.1.1.2"), 1);
	staticRoutingMesh1->AddNetworkRouteTo (Ipv4Address ("192.168.3.0"), Ipv4Mask("255.255.255.0"), Ipv4Address("10.1.1.3"), 1);

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
	for (int i = 0; i < (int)staNodes3.GetN(); i++)
	{
		ipv4sta = staNodes3.Get(i)->GetObject<Ipv4> ();
		staticRoutingSta = ipv4RoutingHelper.GetStaticRouting (ipv4sta);
		staticRoutingSta->AddHostRouteTo (Ipv4Address ("10.1.1.1"), Ipv4Address ("192.168.3.1"), 1);
	}

  uint16_t port = 80;

	for (int i = 0; i < (int)staNodes.GetN(); i++)
	{ 
    Ipv4Address addr = ipv4mesh1->GetAddress (1, 0).GetLocal();
    DashClientHelper client("ns3::TcpSocketFactory",InetSocketAddress(addr, port),protocol);
    client.SetAttribute("VideoId", UintegerValue(i+1)); // VideoId should be positive
    client.SetAttribute("TargetDt", TimeValue(Seconds(target_dt)));
    client.SetAttribute("window", TimeValue(Time(window)));
    client.SetAttribute("MaxRate", UintegerValue(5000001));
    ApplicationContainer clientApp = client.Install(staNodes.Get(i)); // Node 0 is the server
    clientApp.Start(Seconds(0.25+i));
    clientApp.Stop(Seconds(stopTime));
	}

  //ap dash server setup

  DashServerHelper server("ns3::TcpSocketFactory",InetSocketAddress(Ipv4Address::GetAny(), port));
  ApplicationContainer serverApps = server.Install(meshNodes.Get(0));
  serverApps.Start(Seconds(0.0));
  serverApps.Stop(Seconds(stopTime + 5.0));

  Simulator::Stop(Seconds (m_totalTime + 1));
  Simulator::Run ();
  Simulator::Destroy ();
  /*
	std::cout << map1_pos << "\t" << map2_pos << "\t";
	uint32_t StaBytesRecv[nsta];
	uint32_t totalBytesRecv = 0;
	double throughput[nsta];
	double sumThp = 0;
	for (int i = 0; i < nsta; i++)
	{
		StaBytesRecv[i] = DynamicCast<PacketSink> (apps_sink.Get (i))->GetTotalRx ();
		totalBytesRecv += StaBytesRecv[i];
		throughput[i] = StaBytesRecv[i] * 8 / ((m_totalTime - 1) * 1000000);
		sumThp += throughput[i];
	}
  std::cout << "\t";
	for (int i = 0; i < nsta; i++)
		std::cout << throughput[i] << "\t";
	std::cout << sumThp << "\n";
  */
  return 0;
}




