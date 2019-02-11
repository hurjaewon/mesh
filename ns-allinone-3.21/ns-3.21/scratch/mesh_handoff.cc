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

NS_LOG_COMPONENT_DEFINE ("MeshHanOffScript");

static void PrintPositions (Ptr<Node> staNode)
{
    Ptr<NetDevice> staNetDevice = staNode->GetDevice(0);
    Ptr<WifiMac> staMac = DynamicCast<WifiNetDevice> (staNetDevice)->GetMac();
    Mac48Address staBssid = staMac->GetBssid();

    std::cout << "t = " << Simulator::Now().GetMicroSeconds()/1000000.0 << " " << staBssid << std::endl;
    Ptr<MobilityModel> mob = staNode->GetObject<MobilityModel> ();
    Vector pos = mob->GetPosition ();
    std::cout << "pos: " << pos.x << ", " << pos.y << std::endl;
    Simulator::Schedule (Seconds (1), (&PrintPositions), staNode);
}

int
main (int argc, char *argv[])
{
    double m_randomStart = 0.1;
    uint32_t m_nIfaces = 1;
    bool m_chan = false;
    std::string m_stack = "ns3::Dot11sStack";
    std::string m_root = "ff:ff:ff:ff:ff:ff";

    double apGain = 0;
    double staGain = 0;
    double mpp_x = 0, mpp_y = 0, map_x = 100, map_y = 0;
    double ap5Txpower = 15;
    double ap2_4Txpower = 13;
    double staTxpower = 13;

    bool mesh_ampdu = false;
    bool mac_ampdu = false;
    uint32_t seed = 10;
    int nap = 2;
    int nsta = 1;
    int mnss = 2;

    RngSeedManager::SetRun(seed);

    NodeContainer Nodes, apNodes, staNodes;
    NetDeviceContainer serverDevice, meshDevices, apDevices, staDevices, brDevices;
    Ipv4InterfaceContainer serverInterface, staInterface;
    InternetStackHelper internetStack;
    ApplicationContainer apps_sink, apps_source;

    MeshHelper mesh;
    WifiHelper wifi = WifiHelper::Default ();
    BridgeHelper bridge;

    apNodes.Create(nap);
    staNodes.Create(nsta);

    Nodes.Add(apNodes);
    Nodes.Add(staNodes);

    YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default();
    YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();

    wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    wifiChannel.AddPropagationLoss("ns3::LogDistancePropagationLossModel");
    wifiChannel.AddPropagationLoss("ns3::JakesPropagationLossModel");

    int bandwidth = 20;

    wifiPhy.SetChannel (wifiChannel.Create());
    Config::SetDefault ("ns3::YansWifiPhy::ChannelWidth", UintegerValue(bandwidth));

    mesh = MeshHelper::Default();
    mesh.SetStandard(WIFI_PHY_STANDARD_80211n_5GHZ);
    mesh.SetRemoteStationManager ("ns3::MinstrelWifiManager");

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
    meshDevices = mesh.Install (wifiPhy, apNodes);
    serverDevice.Add(meshDevices.Get(0));

    wifi.SetStandard (WIFI_PHY_STANDARD_80211n_2_4GHZ);
    wifi.SetRemoteStationManager ("ns3::MinstrelWifiManager");

    HtWifiMacHelper mac = HtWifiMacHelper::Default();
    Ssid ssid = Ssid ("MWNL_301");

    if (mac_ampdu)
    {
        mac.SetImplicitBlockAckRequestForAc (AC_BE, true);
        mac.SetBlockAckThresholdForAc (AC_BE, 1);
        mac.SetMpduAggregatorForAc (AC_BE, "ns3::MpduStandardAggregator", "MaxAmpduSize", UintegerValue(65535));
        mac.SetMaxPpduTime (AC_BE, MilliSeconds (10));
    }

    wifiChannel.ClearPropagationLoss();
    wifiChannel.AddPropagationLoss("ns3::LogDistancePropagationLossModel");
    wifiChannel.AddPropagationLoss("ns3::JakesPropagationLossModel");

    wifiPhy.SetChannel (wifiChannel.Create());

    wifiPhy.Set ("TxGain", DoubleValue (apGain), "RxGain", DoubleValue (apGain), "TxPowerStart", DoubleValue (ap2_4Txpower), "TxPowerEnd", DoubleValue (ap2_4Txpower), "ChannelNumber", UintegerValue(1), "Transmitters", UintegerValue(1), "Receivers", UintegerValue(1));
    mac.SetType ("ns3::ApWifiMac", "Ssid", SsidValue (ssid), "BeaconInterval", TimeValue (MicroSeconds (102400)), "BeaconGeneration", BooleanValue (true));
    apDevices = wifi.Install (wifiPhy, mac, apNodes.Get(0));
    brDevices = bridge.Install (apNodes.Get(0), NetDeviceContainer (apDevices.Get(0), meshDevices.Get(0)));

    wifiPhy.Set ("TxGain", DoubleValue (apGain), "RxGain", DoubleValue (apGain), "TxPowerStart", DoubleValue (ap2_4Txpower), "TxPowerEnd", DoubleValue (ap2_4Txpower), "ChannelNumber", UintegerValue(5), "Transmitters", UintegerValue(1), "Receivers", UintegerValue(1));
    apDevices.Add (wifi.Install (wifiPhy, mac, apNodes.Get(1)));
    brDevices.Add (bridge.Install (apNodes.Get(1), NetDeviceContainer (apDevices.Get(1), meshDevices.Get(1))));

    wifiPhy.Set ("ChannelNumber", UintegerValue(1), "TxGain", DoubleValue (staGain), "RxGain", DoubleValue (staGain), "TxPowerStart", DoubleValue (staTxpower), "TxPowerEnd", DoubleValue (staTxpower));
    mac.SetType ("ns3::StaWifiMac", "Ssid", SsidValue (ssid), "ScanType", EnumValue (StaWifiMac::ACTIVE), "ActiveProbing", BooleanValue (true));
    staDevices = wifi.Install (wifiPhy, mac, staNodes);

    MobilityHelper apMobility, staMobility;
    Ptr<ListPositionAllocator> apAlloc = CreateObject<ListPositionAllocator> ();

    apAlloc->Add (Vector (mpp_x, mpp_y, 1));
    apAlloc->Add (Vector (map_x, map_y, 1));
    apMobility.SetPositionAllocator (apAlloc);
    apMobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    apMobility.Install (apNodes);

    staMobility.SetMobilityModel ("ns3::ConstantVelocityMobilityModel");
    staMobility.Install (staNodes);
    Ptr<ConstantVelocityMobilityModel> staMob = staNodes.Get(0)->GetObject<ConstantVelocityMobilityModel> ();
    staMob->SetPosition (Vector (0.0, 0.0, 0.0));
    staMob->SetVelocity (Vector (10.0, 0.0, 0.0));

    internetStack.Install (apNodes);
    internetStack.Install (staNodes);
    Ipv4AddressHelper ip;
    ip.SetBase ("192.168.0.0", "255.255.255.0");
    serverInterface = ip.Assign (meshDevices.Get(0));
    staInterface = ip.Assign (staDevices);

    Simulator::Schedule (Seconds (0), (&PrintPositions), staNodes.Get(0));

    UdpServerHelper srv(9);
    ApplicationContainer srv_apps = srv.Install (staNodes.Get(0));
    srv_apps.Start (Seconds (0.5));
    srv_apps.Stop (Seconds (15.0));

    UdpClientHelper client(staInterface.GetAddress (0), 9);
    client.SetAttribute ("MaxPackets", UintegerValue (64707202));
    client.SetAttribute ("Interval", TimeValue (Time ("0.01")));
    client.SetAttribute ("PacketSize", UintegerValue (1450));
    ApplicationContainer cln_apps = client.Install (apNodes.Get(0));
    cln_apps.Start (Seconds (0.5));
    cln_apps.Stop (Seconds (60.0));

    Simulator::Stop (Seconds (60.0));
    Simulator::Run ();
    Simulator::Destroy ();
}

    


