#include "ns3/core-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/wifi-module.h"
#include "ns3/wifi-mode.h"
#include "ns3/mobility-module.h"
#include "ns3/csma-module.h"
#include "ns3/internet-module.h"
#include <ostream>
#include "stdio.h"
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <string>
using namespace ns3;
NS_LOG_COMPONENT_DEFINE ("Caudal");

void 
PrintRecordResult(char* fn, int dist, double thpt, double aggr, double sfer, int iter, int ampdu, 
    int sd0, int sd1, int sd2, int sd3, int sd4, int sd5, int sd6, int sd7, int sd8, int sd9, int sd10, int sd11, int sd12, int sd13, int sd14, int sd15, int sd16, int sd17,
    int fd0, int fd1, int fd2, int fd3, int fd4, int fd5, int fd6, int fd7, int fd8, int fd9, int fd10, int fd11, int fd12, int fd13, int fd14, int fd15, int fd16, int fd17)
{
		std::ofstream out (fn);
		out << std::fixed;
		out << std::showpoint;
		out.precision(6);
		out << "DISTANCE\t" << dist <<
				"\tTHPT\t"<< thpt << 
				"\tAGGRTIME\t" << aggr << 
				"\tSFER\t" << sfer << 
				"\tITER\t" << iter << 
				"\tAMPDU\t" << ampdu << 
				"\tSMCS0\t" << sd0 << 
				"\tSMCS1\t" << sd1 << 
				"\tSMCS2\t" << sd2 << 
				"\tSMCS3\t" << sd3 << 
				"\tSMCS4\t" << sd4 << 
				"\tSMCS5\t" << sd5 << 
				"\tSMCS6\t" << sd6 << 
				"\tSMCS7\t" << sd7 << 
				"\tSMCS8\t" << sd8 << 
				"\tSMCS9\t" << sd9 << 
				"\tSMCS10\t" << sd10 << 
				"\tSMCS11\t" << sd11 << 
				"\tSMCS12\t" << sd12 << 
				"\tSMCS13\t" << sd13 << 
				"\tSMCS14\t" << sd14 << 
				"\tSMCS15\t" << sd15 << 
				"\tSMCS16\t" << sd16 << 
				"\tSMCS17\t" << sd17 << 
				"\tFMCS0\t" << fd0 << 
				"\tFMCS1\t" << fd1 << 
				"\tRFCS2\t" << fd2 << 
				"\tFMCS3\t" << fd3 << 
				"\tFMCS4\t" << fd4 << 
				"\tFMCS5\t" << fd5 << 
				"\tFMCS6\t" << fd6 << 
				"\tFMCS7\t" << fd7 << 
				"\tFMCS8\t" << fd8 << 
				"\tFMCS9\t" << fd9 << 
				"\tFMCS10\t" << fd10 << 
				"\tFMCS11\t" << fd11 << 
				"\tFMCS12\t" << fd12 << 
				"\tFMCS13\t" << fd13 << 
				"\tFMCS14\t" << fd14 << 
				"\tFMCS15\t" << fd15 << 
				"\tFMCS16\t" << fd16 << 
				"\tFMCS17\t" << fd17 << 
				std::endl;
		out.close();
}


int tx_tot=0;
int tx_ampdu=0;
int tx_suc=0;
double aggr_time=0;
//[stream][bandwidth][mcs]
int ***smcsLog;
int ***fmcsLog;
double inst_thpt[20];
int inst_idx=0;
static void
CaudalLoss_ (Mac48Address addr, uint16_t totalTx, uint16_t Nsuc, double aggrTime, int mcs, int antenna, uint16_t bw)
{
  if(Simulator::Now().GetSeconds() > 5)
  {
    if(Simulator::Now().GetSeconds() > 5.5 + inst_idx*0.5)
      inst_idx++;
    inst_thpt[inst_idx] += Nsuc;

    NS_LOG_DEBUG(Simulator::Now().GetSeconds() <<" tx\t" << totalTx << " success\t" << Nsuc << " aggr\t" << aggrTime
        << " mcs\t" << mcs << " antenna\t" << antenna << " bw\t" << bw);
    tx_ampdu++;
    tx_tot += totalTx;
    tx_suc += Nsuc;
    aggr_time += aggrTime;

    int bwIdx=0;
    switch(bw){
      case 40:
        bwIdx=1;
      case 80:
        bwIdx=2;
      default:
        bwIdx=0;
    }
    smcsLog[antenna-1][bwIdx][mcs]+=Nsuc;
    fmcsLog[antenna-1][bwIdx][mcs]+=totalTx-Nsuc;
  }
}

std::string 
IEEE80211acMode (int index, int bw)
{
	switch(index){
	case 0:
		if(bw==80)
			return "11acMcs0BW80MHz";
		else if(bw==40)
			return "11acMcs0BW40MHz";
		else
			return "11acMcs0BW20MHz";
	case 1:
		if(bw==80)
			return "11acMcs1BW80MHz";
		else if(bw==40)
			return "11acMcs1BW40MHz";
		else
			return "11acMcs1BW20MHz";
	case 2:
		if(bw==80)
			return "11acMcs2BW80MHz";
		else if(bw==40)
			return "11acMcs2BW40MHz";
		else
			return "11acMcs2BW20MHz";
	case 3:
		if(bw==80)
			return "11acMcs3BW80MHz";
		else if(bw==40)
			return "11acMcs3BW40MHz";
		else
			return "11acMcs3BW20MHz";
	case 4:
		if(bw==80)
			return "11acMcs4BW80MHz";
		else if(bw==40)
			return "11acMcs4BW40MHz";
		else
			return "11acMcs4BW20MHz";
	case 5:
		if(bw==80)
			return "11acMcs5BW80MHz";
		else if(bw==40)
			return "11acMcs5BW40MHz";
		else
			return "11acMcs5BW20MHz";
	case 6:
		if(bw==80)
			return "11acMcs6BW80MHz";
		else if(bw==40)
			return "11acMcs6BW40MHz";
		else
			return "11acMcs6BW20MHz";
	case 7:
		if(bw==80)
			return "11acMcs7BW80MHz";
		else if(bw==40)
			return "11acMcs7BW40MHz";
		else
			return "11acMcs7BW20MHz";
	case 8:
		if(bw==80)
			return "11acMcs8BW80MHz";
		else if(bw==40)
			return "11acMcs8BW40MHz";
		else
			return "11acMcs8BW20MHz";
	case 9:
		if(bw==80)
			return "11acMcs9BW80MHz";
		else if(bw==40)
			return "11acMcs9BW40MHz";
		else
			return "11acMcs9BW20MHz";
	default:
		return "11acMcs0BW20MHz";
	}
}

std::string 
	IEEE80211nAckMode (int index )
{
	switch(index){
	case 0:
	default:
		return "OfdmRate6Mbps";
	case 1:
	case 2:
		return "OfdmRate12Mbps";
	case 3:
	case 4:
	case 5:
	case 6:
	case 7:
		return "OfdmRate24Mbps";
	}
}

int main (int argc, char *argv[])
{
  smcsLog = new int ** [4];
  fmcsLog = new int ** [4];
  for(int z=0; z<4; z++)
  {
    smcsLog[z] = new int * [3];
    fmcsLog[z] = new int * [3];
    for(int j=0; j<3; j++)
    {
      smcsLog[z][j] = new int [10];
      fmcsLog[z][j] = new int [10];
      for(int i=0; i<10; i++)
      {
        smcsLog[z][j][i] = 0;
        fmcsLog[z][j][i] = 0;
      }
    }
  }
  int version = 0;
  bool vht=true;
  int nsta=1;
	uint32_t pktSize = 1470;
	uint32_t thre = 2000000;
	double dist = 5;
	int nss = 1;
	double dopplerVel=0.1;
	bool pcap =0;
  int mcs = 7;
	int bandwidth = 20;
	bool ampdu = 1;
	int iter = 1;
	int sim_time= 5;
	int ra = 2;
  bool caudal = false;

	CommandLine cmd;
	cmd.AddValue ("pktSize", "Size of UDP packet", pktSize);
	cmd.AddValue ("ver", "version", version);
	cmd.AddValue ("thre", "RTC/CTS threshold", thre);
	cmd.AddValue ("Dist", "Distance btw AP and STA", dist);
	cmd.AddValue ("Nss", "number of spatial streams", nss);
	cmd.AddValue ("Mcs", "Mcs index for tx", mcs);
	cmd.AddValue ("Dvel", "Doppler velocity", dopplerVel);
	cmd.AddValue ("Bandwidth", "Bandwidth", bandwidth);
	cmd.AddValue ("Pcap", "Pcap", pcap);
	cmd.AddValue ("Nsta", "number of stations", nsta);
	cmd.AddValue ("Ampdu", "Enable Ampdu", ampdu);
	cmd.AddValue ("Iter", "iteration number", iter);
	cmd.AddValue ("SimTime", "Simulation Time", sim_time);
	cmd.AddValue ("Ra", "Rate control", ra);
	cmd.AddValue ("Caudal", "Caudal loss", caudal);
	cmd.AddValue ("Vht", "vht", vht);
	cmd.Parse (argc,argv);

  NS_LOG_UNCOND("Iter:" << iter << " Dist:" << dist << " Bandwidth:" << bandwidth << " SimulationTime:" << sim_time); 
	SeedManager::SetRun(iter);	
 
	Time EndSimulationTime = Seconds( 5+sim_time );

	NodeContainer wifiApNode;
	wifiApNode.Create (1);
	
	NodeContainer wifiStaNodes;
	wifiStaNodes.Create (nsta);

	NodeContainer nodes (wifiApNode, wifiStaNodes);

  

	double dopplerFreq = dopplerVel * 50 / 3;
	YansWifiChannelHelper channel;
	channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel"); 
	channel.AddPropagationLoss("ns3::JakesPropagationLossModel");
  Config::SetDefault ("ns3::JakesProcess::DopplerFrequencyHz", DoubleValue(dopplerFreq));  
  Config::SetDefault ("ns3::YansWifiChannel::CaudalLoss", BooleanValue(caudal));  
	channel.AddPropagationLoss("ns3::TgaxPropagationLossModel");
	
	YansWifiPhyHelper phy = YansWifiPhyHelper::Default ();
	phy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11_RADIO);
	phy.SetErrorRateModel ("ns3::YansErrorRateModel");
	phy.SetChannel (channel.Create ());
  Config::SetDefault ("ns3::YansWifiPhy::ChannelWidth", UintegerValue(bandwidth));
  //Config::SetDefault ("ns3::YansWifiPhy::AntennaCorrelation", BooleanValue (true));

	
	VhtWifiMacHelper mac = VhtWifiMacHelper::Default ();
	if(!vht)
		HtWifiMacHelper mac = HtWifiMacHelper::Default ();

	Ssid ssid = Ssid ("ns-3-ssid");
	mac.SetType ("ns3::ApWifiMac",
			"Ssid", SsidValue (ssid));

	WifiHelper wifi = WifiHelper::Default ();
	wifi.SetStandard (WIFI_PHY_STANDARD_80211ac);
	std::string dataMode (IEEE80211acMode(mcs, bandwidth));
	std::string basicMode (IEEE80211nAckMode(mcs));


	if(ra==1)
  {
  	wifi.SetRemoteStationManager ("ns3::GenieWifiManager", "RtsCtsThreshold", UintegerValue (thre), "MaxSlrc", UintegerValue(7) );
	}
  else if(ra==2)
  {
  	wifi.SetRemoteStationManager ("ns3::MinstrelWifiManager", "RtsCtsThreshold", UintegerValue (thre), "MaxSlrc", UintegerValue(1000) );
  }
  else
  {
	wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "DataMode", StringValue (dataMode), "ControlMode", StringValue (basicMode),"MaxSlrc",UintegerValue(7),"RtsCtsThreshold", UintegerValue (thre));
	}


  phy.Set("Transmitters", UintegerValue(nss));
  phy.Set("Receivers", UintegerValue(nss));

	if (ampdu){
		mac.SetImplicitBlockAckRequestForAc (AC_BE,true);
		mac.SetBlockAckThresholdForAc (AC_BE, 1);
	  if(vht)	
    {
      mac.SetMpduAggregatorForAc (AC_BE, "ns3::MpduStandardAggregator", "MaxAmpduSize", UintegerValue (1048575));
      mac.SetMaxPpduTime (AC_BE, MilliSeconds (5.484));
    }
    else
    {
      mac.SetMpduAggregatorForAc (AC_BE, "ns3::MpduStandardAggregator", "MaxAmpduSize", UintegerValue (65535));
      mac.SetMaxPpduTime (AC_BE, MilliSeconds (10));
    }
	}


	NetDeviceContainer apDevices = wifi.Install (phy, mac, wifiApNode);
	
	mac.SetType ("ns3::StaWifiMac",
			"Ssid", SsidValue (ssid),
			"ActiveProbing", BooleanValue (false));


	NetDeviceContainer staDevices = wifi.Install (phy, mac, wifiStaNodes);

	
	MobilityHelper Stamobility;
	Stamobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
	Ptr<ListPositionAllocator> positionAllocSta[nsta] ;
	for (int k=0;k<nsta;k++){
		positionAllocSta[k] = CreateObject<ListPositionAllocator> ();
		positionAllocSta[k]->Add (Vector (1+dist*k, 0.0, 0.0));
		Stamobility.SetPositionAllocator (positionAllocSta[k]);
		Stamobility.Install (wifiStaNodes.Get(k));
	}

	MobilityHelper Apmobility;
	Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
	positionAlloc->Add (Vector (0.0, 0.0, 0.0));
	Apmobility.SetPositionAllocator (positionAlloc);
	Apmobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
	Apmobility.Install (wifiApNode);

	InternetStackHelper stack;
	stack.Install (nodes);

	Ipv4AddressHelper address;
	address.SetBase ("10.1.1.0", "255.255.255.0");
	Ipv4InterfaceContainer wifiApInterface = address.Assign (apDevices);
	Ipv4InterfaceContainer wifiStaInterfaces = address.Assign (staDevices);

	Ipv4GlobalRoutingHelper::PopulateRoutingTables ();
	
	uint16_t port = 8080;

	ApplicationContainer app, sinkapp;
	ApplicationContainer pingApps;

	UdpEchoClientHelper pingHelper(Ipv4Address(), 9);
	pingHelper.SetAttribute("MaxPackets", UintegerValue(1));
	pingHelper.SetAttribute("Interval", TimeValue(Seconds(0.1)));
	pingHelper.SetAttribute("PacketSize", UintegerValue(10));


	OnOffHelper onoff ("ns3::UdpSocketFactory", Address (InetSocketAddress (wifiStaInterfaces.GetAddress(0), port)));
	Config::SetDefault("ns3::UdpSocket::RcvBufSize", UintegerValue(64000));
	onoff.SetAttribute ("OnTime",StringValue ("ns3::ConstantRandomVariable[Constant=1000]") );
	onoff.SetAttribute ("OffTime",StringValue ("ns3::ConstantRandomVariable[Constant=0]") );
	onoff.SetAttribute("MaxBytes", UintegerValue(0));
	onoff.SetAttribute ("PacketSize", UintegerValue(pktSize));
	onoff.SetAttribute ("DataRate", StringValue ("900Mbps"));

	for (int k=0;k<nsta;k++){

		PacketSinkHelper udpsink ("ns3::UdpSocketFactory", Address (InetSocketAddress (Ipv4Address::GetAny (), port)));
		udpsink.SetAttribute("StartMeasureTime", TimeValue(Seconds(5.0)));
		sinkapp.Add( udpsink.Install (wifiStaNodes.Get (k)));
		
		onoff.SetAttribute("Remote", AddressValue(InetSocketAddress(wifiStaInterfaces.GetAddress(k), port)));
		app.Add (onoff.Install(wifiApNode.Get(0)));
	}

  UniformVariable x(0,1);
	for (int j=0; j<nsta; j++)
	{
		pingHelper.SetAttribute("RemoteAddress", AddressValue(wifiStaInterfaces.GetAddress(j)));
		pingHelper.SetAttribute("StartTime", TimeValue(Seconds(3.0 * x.GetValue())));
		pingApps.Add(pingHelper.Install(wifiApNode.Get(0)));
	}

	for (uint32_t i=0; i < app.GetN(); i++)
  {
    UniformVariable app_start(4,5);
    double app_Start = app_start.GetValue();
		app.Get(i)->SetStartTime(Time(Seconds(app_Start)));
    NS_LOG_DEBUG(i+1 << "th application start time=" << app_Start);
  }
	app.Stop(EndSimulationTime);
	
	sinkapp.Start (Seconds (1.0));
  sinkapp.Stop(EndSimulationTime);
	
	if(pcap==true)
	{
			std::ostringstream oss2;
			oss2 << "pcap/multicelludp-cell"<< 0;
			phy.EnablePcap (oss2.str(), apDevices.Get(0), false);
		
	}

  Config::ConnectWithoutContext ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/Caudal", MakeCallback (&CaudalLoss_));

	Simulator::Stop (EndSimulationTime);
  Simulator::Run ();



	double tot_bytes;
	double tot_thruput;
	double sta_bytes[nsta];
	double sta_thruput[nsta];

	tot_bytes = 0;
	tot_thruput = 0;

	for (int j=0; j<nsta; j++)
	{
		Ptr<PacketSink> sink = DynamicCast<PacketSink>(sinkapp.Get(j));
		sta_bytes[j] = sink->GetRecvBytes();
		tot_bytes += sta_bytes[j];
		sta_thruput[j] = sta_bytes[j]*8/(EndSimulationTime.GetSeconds()-5.0)/1000000;
		tot_thruput += sta_thruput[j];
		NS_LOG_UNCOND("sta bytes of node "<<j<<": " <<  sta_thruput[j] << " Mbps\t" );
  }

  NS_LOG_UNCOND("SUCCESS");
  for(int z=0; z<nss; z++)
  {
    int j = 0;
    NS_LOG_UNCOND("SS:" << z+1 <<" BW"<<20<< "\t" << smcsLog[z][j][0]<< "\t" << smcsLog[z][j][1]<< "\t" << smcsLog[z][j][2] 
        << "\t" << smcsLog[z][j][3] << "\t" << smcsLog[z][j][4]<< "\t" << smcsLog[z][j][5]
        << "\t" << smcsLog[z][j][6] << "\t" << smcsLog[z][j][7]<< "\t" << smcsLog[z][j][8]<< "\t" << smcsLog[z][j][9]);
  }
  NS_LOG_UNCOND("FAILURE");
  for(int z=0; z<nss; z++)
  {
    int j = 0;
    NS_LOG_UNCOND("SS:" << z+1 <<" BW"<<20<< "\t" << fmcsLog[z][j][0]<< "\t" << fmcsLog[z][j][1]<< "\t" << fmcsLog[z][j][2] 
        << "\t" << fmcsLog[z][j][3] << "\t" << fmcsLog[z][j][4]<< "\t" << fmcsLog[z][j][5]
        << "\t" << fmcsLog[z][j][6] << "\t" << fmcsLog[z][j][7]<< "\t" << fmcsLog[z][j][8]<< "\t" << fmcsLog[z][j][9]);
  }
  double sfer = (double)(tx_tot-tx_suc)/tx_tot*100;
  double aggrTime = aggr_time/tx_ampdu;
  NS_LOG_UNCOND ("Nss\t"<<nss<<"\tDist\t"<<dist<<"\tTotalTput\t" << tot_thruput 
          << "\tSFER\t" <<  sfer << "\tAGGR\t" << aggrTime << "\tTOTAL\t" << tx_tot  << "\tSUCCESS\t" << tx_suc);

  double total_inst = 0;
  for(int i=0; i<10; i++)
    total_inst+=inst_thpt[i];
  NS_LOG_UNCOND("# OF MPDU " << total_inst);

  char fn[255];
  sprintf(fn,"/home/shbyeon/data/Dist_%d-Caudal_%d-Dvel_%.1f-Iter_%d.txt",(int)dist,caudal,dopplerVel,iter);
  PrintRecordResult (fn,dist,tot_thruput,aggrTime,sfer,iter,tx_ampdu,
      smcsLog[0][0][0],smcsLog[0][0][1],smcsLog[0][0][2],smcsLog[0][0][3],smcsLog[0][0][4],
      smcsLog[0][0][5],smcsLog[0][0][6],smcsLog[0][0][7],smcsLog[0][0][8],
      smcsLog[1][0][0],smcsLog[1][0][1],smcsLog[1][0][2],smcsLog[1][0][3],smcsLog[1][0][4],
      smcsLog[1][0][5],smcsLog[1][0][6],smcsLog[1][0][7],smcsLog[1][0][8],
      fmcsLog[0][0][0],fmcsLog[0][0][1],fmcsLog[0][0][2],fmcsLog[0][0][3],fmcsLog[0][0][4],
      fmcsLog[0][0][5],fmcsLog[0][0][6],fmcsLog[0][0][7],fmcsLog[0][0][8],
      fmcsLog[1][0][0],fmcsLog[1][0][1],fmcsLog[1][0][2],fmcsLog[1][0][3],fmcsLog[1][0][4],
      fmcsLog[1][0][5],fmcsLog[1][0][6],fmcsLog[1][0][7],fmcsLog[1][0][8]);

  for(int z=0; z<4; z++)
  {
    for(int j=0; j<3; j++)
      delete [] smcsLog[z][j];
    delete [] smcsLog[z];
  }
  delete [] smcsLog;
  for(int z=0; z<4; z++)
  {
    for(int j=0; j<3; j++)
      delete [] fmcsLog[z][j];
    delete [] fmcsLog[z];
  }
  delete [] fmcsLog;
  
  Simulator::Destroy ();
}

