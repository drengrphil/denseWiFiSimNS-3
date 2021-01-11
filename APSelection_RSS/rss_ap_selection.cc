/*
 Phillip and Amr
 Electrical and Computer Engineering
 Queen's University, Kingston, ON
 CISC 825 Final Project

 Interference-aware AP selection in Dense IEEE 802.11 Networks

 Standard used: 802.11n Wifi Network
 MCS value (0 to 7)
 Channel Width (20 or 40 MHz)
 Guard Interval (long or short)
 PHY bitrate
 IP Packet Size: 1500 bytes
 Packets are classified as BestEffort Access Class (AC_BE), no QoS

 This program simulates a Dense WiFi network where Stations selection APs that offer
 best SINR in the uplink considering interference rather using the RSS based scheme
 which does not consider interference, but select APs based on the level of RSS.
*/

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/internet-module.h"
/*For Network Animator*/
#include "ns3/netanim-module.h"
#include<iostream>
#include<fstream>
#include "ns3/propagation-loss-model.h"
#include<algorithm>
#include<vector>
#include<ctime>
#include<cstdlib>
#include<iostream>

using namespace ns3;
using namespace std;

NS_LOG_COMPONENT_DEFINE ("cisc825-wifi-network");

/*Function to determine locations of APs and STAs
 * And calculate distances between each AP and STA
 * */
//---------------------Distance between STAs and APs----------------------
double disAPSTA(Ptr<Node> apNode, Ptr<Node> staNode) {
	Ptr<MobilityModel> apfixMobility = apNode->GetObject<MobilityModel>();
	Ptr<MobilityModel> staMobility = staNode->GetObject<MobilityModel>();
	double disbtwAPnSTA = staMobility->GetDistanceFrom(apfixMobility);
	//Return distance between AP and STA
	return disbtwAPnSTA;
}

/* Function to determine RSS between AP_j and STA_i  in Down-link
 * Using Log Distance Propagation Model
 * */
//-------------------Down-link PathLoss for RSS between APs and STAs------------------------------
double RssValueSTAAPDL(Ptr<Node> apnode, Ptr<Node> stanode) {
	Ptr<LogDistancePropagationLossModel> rssmodel = CreateObject<
			LogDistancePropagationLossModel>();
	//Transmit Power of APs
	double txPower_APdBm = +20.0; // 20dBm, 100mW
	//Get AP location on the network area
	Ptr<MobilityModel> apfixMobility = apnode->GetObject<MobilityModel>();
	//Get STA location on the network area
	Ptr<MobilityModel> staMobility = stanode->GetObject<MobilityModel>();
	//Compute the RSS for AP to STA Links
	double rxPowerDbm_DL = rssmodel->CalcRxPower(txPower_APdBm, apfixMobility,
			staMobility);
	return rxPowerDbm_DL;
}

/* Function to determine RSS between AP_j and STA_i  in Up-link
 * Using Log Distance Propagation Model
 * */
//-------------------Up-link PathLoss for RSS between APs and STAs--------------------------------
double RssValueSTAAPUL(Ptr<Node> apnode, Ptr<Node> stanode) {

	Ptr<LogDistancePropagationLossModel> rssmodel1 = CreateObject<
			LogDistancePropagationLossModel>();
	//Transmit Power of STAs
	double txPower_STAdBm = +12; //12dBm, 15.85mW
	//Get AP location on the network area
	Ptr<MobilityModel> apfixMobility = apnode->GetObject<MobilityModel>();
	//Get STA location on the network area
	Ptr<MobilityModel> staMobility = stanode->GetObject<MobilityModel>();
	//Compute the RSS for STA to AP Links
	double rxPowerDbm_UL = rssmodel1->CalcRxPower(txPower_STAdBm, staMobility,
			apfixMobility);
	return rxPowerDbm_UL;
}

/*Generate Random packet sizes
 * min packet size = 500 bytes
 * max packaet size = 1500
 * */

int payLoadSizeGenerator(int minPktSize, int maxPktSize){
	//srand(time(0));
	//return (rand() % (maxPktSize - minPktSize + 1) + minPktSize);
	Ptr<UniformRandomVariable> ra = CreateObject<UniformRandomVariable>();
	return ra->GetInteger(minPktSize, maxPktSize);
}


//Number of Access Points
uint32_t numAPs = 15;
//Number of Stations
uint32_t numSTAs = 300;
//Association variable x_ij = 1 if STA_i associates with STA_j, x_ij = 0 otherwise
int x_ij = 0; //Binary variable indicating association

int main (int argc, char *argv[])
{

	//Association matrix for Down-link
	int Xij_UL[numSTAs][numAPs]; //Contains all x_ijs
	//Association matrix for Down-link
	int Xij_DL[numSTAs][numAPs];	//Contains all x_ijs
	//Array of Distances between APs and STAs
	double STA2AP_dis[numSTAs][numAPs];
	//Array of RSS
	double RSS_ULdBm[numSTAs][numAPs];	//Up-link
	double RSS_DLdBm[numSTAs][numAPs];	//Down-link
	//Simulation Time (s)
	double simulationTime = 5; //seconds
	//Distance between STA and AP
	double distance = 0.0; //meters
	//Frequency
	double freqBand = 2.4; //whether 2.4 or 5.0 GHz
	//Number of STAs per BSS
	std::vector<int> totalUser(numAPs);
	//Initialize to zero
	std::fill(totalUser.begin(), totalUser.end(), 0);
	//Keep Record of packet sent by STAs
	std::vector<int>packetSizes(numSTAs);
	//Initialize to zeros
	std::fill(packetSizes.begin(), packetSizes.end(), 0);
	//Stations
	NodeContainer wifiStaNode;
	wifiStaNode.Create(numSTAs);
	//APs
	NodeContainer wifiApNode;
	wifiApNode.Create(numAPs);
	//Command Line Paramters
	  CommandLine cmd;
	  //Set IEEE 802.11 Network Frequency
	  cmd.AddValue ("frequency", "IEEE 802.11n supports both 5GHz or 2.4GHz", freqBand);
	  //Run simulation for different simulation time
	  cmd.AddValue ("simulationTime", "Simulation time in seconds", simulationTime);
	  cmd.Parse (argc,argv);

	//-----------------------------------------Mobility------------------------------------------------------.
	MobilityHelper apfixMobility, staMobility;

	//-----------Mobility and Locations for APs
	apfixMobility.SetPositionAllocator("ns3::RandomRectanglePositionAllocator",
			"X", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=300.0]"),
			"Y", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=300.0]"));
	apfixMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
	//Install Mobility on APs
	apfixMobility.Install(wifiApNode);

	//-------------Mobility and Locations for STAs----------------------------------------------------------
	staMobility.SetPositionAllocator("ns3::RandomRectanglePositionAllocator",
			"X", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=300.0]"),
			"Y", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=300.0]"));
	staMobility.SetMobilityModel("ns3::RandomWalk2dMobilityModel", "Mode",
			StringValue("Time"), "Time", StringValue("5s"), "Bounds",
			RectangleValue(Rectangle(0, 300, 0, 300)));
	//Install Mobility on STAs
	staMobility.Install(wifiStaNode);


	//--------------------------------------Distance between STAs and APs-----------------------------------
	for (int k = 0; k < numSTAs; k++) {
		for (int kk = 0; kk < numAPs; kk++) {
			//Call distance calculating function
			double STA2APdistance = disAPSTA(wifiApNode.Get(kk),
					wifiStaNode.Get(k));
			//Store distances in an array for use later
			STA2AP_dis[k][kk] = STA2APdistance;
			std::ostringstream oss;
			oss << "Distance between AP " << kk << " STA " << k << " "
					<< STA2APdistance;
			NS_LOG_UNCOND(oss.str());
			//Store distances between each AP and STAs in a text file
			ofstream fm;
			//Check if file is opened
			fm.open("DistanceSTAAP.txt", ofstream::app);
			if (fm.is_open()) {
				//Store distances to file
				fm << " " << STA2APdistance;
				fm.close();
			} else {
				//Throw Error Exception
				cout << "Unable to store distances in file";
			}
		}
	}

	//-----------------------------------Received Signal Strength Computation-----------------------------------

	//Up-link RSS
	cout << "------Up-link RSS-----------------" << endl;
	for (int jk = 0; jk < numSTAs; jk++) {
		for (int kj = 0; kj < numAPs; kj++) {
			//Uplink RSS
			double RSS_UL = RssValueSTAAPUL(wifiApNode.Get(kj),
					wifiStaNode.Get(jk));
			RSS_ULdBm[jk][kj] = RSS_UL;			//Store RSS values
			cout << "RSS from AP" << kj << " at STA " << jk << " : " << RSS_UL
					<< endl;
		}
	}

	//Down-link RSS
	cout << "------Down-link RSS-----------------" << endl;
	for (int jkk = 0; jkk < numSTAs; jkk++) {
		for (int kkj = 0; kkj < numAPs; kkj++) {
			//Downlink RSS
			double RSS_DL = RssValueSTAAPDL(wifiApNode.Get(kkj),
					wifiStaNode.Get(jkk));
			RSS_DLdBm[jkk][kkj] = RSS_DL;		//Store RSS values
			//std::ostringstream oss2, oss3;
			cout << "RSS from AP" << kkj << " at STA " << jkk << " : " << RSS_DL
					<< endl;
		}
	}

	/*Determine RSS Association; Default in 802.11 standard
	 * This type of AP association is Default in current 802.11 standards
	 *
	 * */
	//int countuser = 0;
	/*Association based on Up-link RSS*/
	for (int ck = 0; ck < numSTAs; ck++) {
		//Select AP with max RSS
		double tempRss_max = RSS_ULdBm[ck][0];
		int tempind = 0; //Stores index of max RSS
		for (int kc = 0; kc < numAPs; kc++) {
			if (RSS_ULdBm[ck][kc] > tempRss_max) {
				tempRss_max = RSS_ULdBm[ck][kc];
				Xij_DL[ck][tempind] = 0;
				tempind = kc;
			} else {
				Xij_UL[ck][kc] = 0;
			}
		}
		//STA_i associates with AP_j
		Xij_UL[ck][tempind] = 1;
	}

	/*-------------------------Association based on best up-link RSS----------------------*/
	cout << "------Up-link Association-----------------" << endl;
	for (int ti = 0; ti < numSTAs; ti++) {
		for (int it = 0; it < numAPs; it++) {
			if (Xij_UL[ti][it] == 1) {
				//Count Number of Users that associates with each AP
				totalUser[it] = totalUser[it] + 1;
				cout << "STA " << ti << " associates with AP " << it
						<< " with RSS " << RSS_ULdBm[ti][it] << endl;
				ofstream fm, fd;
				//Check if file is opened
				fm.open("SSF_UL_RSS_Assoc.txt", ofstream::app);
				fd.open("UL_AssociationDistance.txt", ofstream::app);
				if (fm.is_open() && fd.is_open()) {
					//Store distances to file
					fm << " " << RSS_ULdBm[ti][it];
					fd << " " << STA2AP_dis[ti][it];
					fm.close();
					fd.close();
				} else {
					//Throw Error Exception
					cout << "Unable to store distances in file";
				}
			}
		}
	}

	/*------------------------------Association based on DL-link RSS----------------------*/
	for (int cck = 0; cck < numSTAs; cck++) {
		//Select AP with max RSS
		double tempRss = RSS_DLdBm[cck][0];
		int tempin = 0;
		for (int kcc = 0; kcc < numAPs; kcc++) {
			if (RSS_DLdBm[cck][kcc] > tempRss) {
				tempRss = RSS_DLdBm[cck][kcc];
				Xij_DL[cck][tempin] = 0;
				tempin = kcc;
			} else {
				//No Association
				Xij_DL[cck][kcc] = 0;
			}
		}
		//STA_i associates with AP_j
		Xij_DL[cck][tempin] = 1;
	}

	//Print Down-link Associations to screen
	cout << "------Down-link Association-----------------" << endl;
	for (int tii = 0; tii < numSTAs; tii++) {
		for (int iit = 0; iit < numAPs; iit++) {
			//Check if association exist between APj and STAi
			if (Xij_DL[tii][iit] == 1) {
				cout << "STA " << tii << " associates with AP " << iit
						<< " with RSS " << RSS_DLdBm[tii][iit] << endl;
				ofstream fml, fdl;
				//Check if file is opened
				fdl.open("DL_AssociationDistance.txt", ofstream::app);
				fml.open("SSF_DL_RSS_Assoc.txt", ofstream::app);
				if (fml.is_open() && fdl.is_open()) {
					//Store distances to file
					fml << " " << RSS_DLdBm[tii][iit];
					fdl << " " << STA2AP_dis[tii][iit];
					fml.close();
					fdl.close();
				} else {
					//Throw Error Exception
					cout << "Unable to store distances in file";
				}
			}
		}
	}

	//Count users in each BSS after association
	cout << "---------------Number of Users per BSS or AP-----------" << endl;
	for (int sd = 0; sd < numAPs; sd++) {
		cout << "Number of Users with AP-" << sd << " is " << totalUser[sd]<< endl;
		ofstream nm;
		//File store number of STAs associated with a typical AP
		nm.open("STAsperAP.txt", ofstream::app);
		if(nm.is_open()){
			//Write to file
			nm <<  " " << totalUser[sd];
			nm.close();
		} else {
			//Throw Error Exception
			cout << "Failed to store number of STAs per AP";
		}
	}
	cout << "---------------/////////////////////////////////----------------" << endl;
	//ofstream sim;
	std::cout <<"STA" << "\t\t" << "AP" << "\t\t" <<"Packet Size" << "\t\t" << "MCS value" << "\t\t" << "Channel width" << "\t\t" << "short GI" << "\t\t" << "Throughput" << '\n';

	//Iterate through the APs
	for(int ji = 0; ji < numSTAs; ji++){
		//Iterate through the STAs
		for(int ij = 0; ij < numAPs; ij++){
			//Check if there is an association between between STA_i and AP_j
			if (Xij_UL[ji][ij] == 1) {
				//Store BSS ID
				ofstream bs;
				bs.open("BPSK_BSS_ID.txt", ofstream::app);//Store BSS ID for each STA
				bs << " " << ij;
				bs.close();
				//Get the distance between STA_i and AP_j
				distance = STA2AP_dis[ji][ij];
				//Modulation and Coding Schemes
				//i ranges from 0 to 7; 0 is for BPSK while 1 is QPSK
				for (int i = 0; i < 1; i++)
				{
					//Channel Width - Could be 20 MHz or 40 MHz
					for (int j = 20; j <= 20; )
					{
						//IEEE 802.11n Guard Interval; 0 or 1
						for (int k = 0; k < 1; k++)
						{
							//IP Packet Size in bytes
							//Call the Packet Generator function
							int minPktSi = 500; //Minimum Packet Size
							int maxPktSi = 1400; //Maximum Packet Size
							int payLoadSize;
							payLoadSize = payLoadSizeGenerator(minPktSi, maxPktSi);
							//Save Transmitted Packet Size to file and vector
							packetSizes[ji] = payLoadSize; //Payload size changes for each user randomly

							ofstream pktsi;
							//Create file for Packet Size
							pktsi.open("BPSK_PacketSizeSent.txt", ofstream::app);
							if(pktsi.is_open()){
								pktsi << " " << payLoadSize;
								//Close the file
								pktsi.close();
							} else {
								//Throw error exception
								cout << "File does not exist";
							}
							NodeContainer staNode; //Inner Node Container
							staNode.Create (1);
							//AP Node container
							NodeContainer apNodes;
							apNodes.Create (1);
							//Wifi Helper
							YansWifiChannelHelper channel = YansWifiChannelHelper::Default ();
							YansWifiPhyHelper phy = YansWifiPhyHelper::Default ();
							phy.SetChannel (channel.Create ());
							// Set guard interval
							phy.Set ("ShortGuardEnabled", BooleanValue (k));
							//Call Wifi Mac Class
							WifiMacHelper mac;
							WifiHelper wifi;
							//Set Wifi Stanard 802.11n on 2.4GHz ISM band
							wifi.SetStandard (WIFI_PHY_STANDARD_80211n_2_4GHZ);
							Config::SetDefault ("ns3::LogDistancePropagationLossModel::ReferenceLoss", DoubleValue (10.046));

							std::ostringstream oss;
							oss << "HtMcs" << i;
							wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager","DataMode", StringValue (oss.str ()),
									"ControlMode", StringValue (oss.str ()));

							Ssid ssid = Ssid ("cisc825-80211nWifi");
							//ns3::StaWifiMac class implements an active probing and association state
									//machine that handles automatic re-association whenever too many beacons are missed
							mac.SetType ("ns3::StaWifiMac",
									"Ssid", SsidValue (ssid),
									"ActiveProbing", BooleanValue (false));
							NetDeviceContainer staDevice;
							staDevice = wifi.Install (phy, mac, staNode);
							//ns3::ApWifiMac implements an AP that generates periodic beacons, and that accepts every attempt to associate.
							mac.SetType ("ns3::ApWifiMac",
									"Ssid", SsidValue (ssid));
							//Device Containers
							NetDeviceContainer apDevice;
							apDevice = wifi.Install (phy, mac, apNodes);
							// Set channel width
							Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/ChannelWidth", UintegerValue (j));
							// mobility.
							MobilityHelper mobility;
							Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
							//Place node in position
							positionAlloc->Add (Vector (0.0, 0.0, 0.0));
							positionAlloc->Add (Vector (distance, 0.0, 0.0));
							mobility.SetPositionAllocator (positionAlloc);
							//Define Mobility Model
							mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
							//Install Mobility on Nodes
							mobility.Install (apNodes);
							mobility.Install (staNode);

							/* Internet stack*/
							InternetStackHelper stack;
							stack.Install (apNodes);
							stack.Install (staNode);

							//Set IP Addresses
							Ipv4AddressHelper address;
							//Address base
							/*
							 * Define IP Address Class
							 * Mask address is set to 255.255.0.0 to have more addresses for large number of stations
							 * Also IP address starts from 192.168.0.0 to accommodate more users.
							 * */
							address.SetBase ("192.168.1.0", "255.255.255.0");
							//Interfaces for IP Address
							Ipv4InterfaceContainer staNodeInterface;
							Ipv4InterfaceContainer apNodeInterface;
							//Assign IP Addresses to Nodes
							staNodeInterface = address.Assign (staDevice);
							apNodeInterface = address.Assign (apDevice);

							/*
							 * Setting applications
							 * We are mainly sending UDP Packet
							 * */
							ApplicationContainer serverApp;
								//UDP Packet flow
								UdpServerHelper myServer (9);
								serverApp = myServer.Install (staNode.Get (0));
								serverApp.Start (Seconds (0.0));
								serverApp.Stop (Seconds (simulationTime + 1));

								UdpClientHelper myClient (staNodeInterface.GetAddress (0), 9);
								myClient.SetAttribute ("MaxPackets", UintegerValue (4294967295u));
								myClient.SetAttribute ("Interval", TimeValue (Time ("0.00001"))); //packets/s
								myClient.SetAttribute ("PacketSize", UintegerValue (payLoadSize));

								ApplicationContainer clientApp = myClient.Install (apNodes.Get (0));
								clientApp.Start (Seconds (1.0));
								clientApp.Stop (Seconds (simulationTime + 1));

							Ipv4GlobalRoutingHelper::PopulateRoutingTables ();

							//----------------------------------------Network Animation------------------------------------
							AnimationInterface anim("cisc825-apselectionscheme.xml");

							//Run Simulator
							Simulator::Stop (Seconds (simulationTime + 1));
							Simulator::Run ();
							Simulator::Destroy ();

							//Calculate End-to-End Throughput
							double throughput = 0;
								uint32_t totalPacketsThrough = DynamicCast<UdpServer> (serverApp.Get (0))->GetReceived ();
								throughput = totalPacketsThrough * payLoadSize * 8 / (simulationTime * 1000000.0); //Mbit/s

							//Write throughput to file for analysis
							ofstream trput, sim;
							//Check or create file
							trput.open("BPSK_Throughput_RSS_VaryPkt.txt", ofstream::app);
							sim.open("BPSK_Simulation_Traces_VaryPkt.txt", ofstream::app);
							std::cout << ji << "\t\t" << ij << "\t\t" << payLoadSize << " bytes\t\t" << i << "\t\t\t" << j << " MHz\t\t\t" << k
									<< "\t\t\t" << throughput << " Mbps" << std::endl;
							//Check if file is open
							if(trput.is_open() & sim.is_open()){
								//Write throughput to file
								trput << " " << throughput;
								sim << ji << "\t\t" << ij << "\t\t" << payLoadSize << " bytes\t\t" << i << "\t\t\t" << j << " MHz\t\t\t" << k
										<< "\t\t\t" << throughput << " Mbps" <<endl;
								trput.close();
							} else {
								//Throw Error Exception
								cout << "Error Opening the throughput file" <<endl;
							}
						}
						j *= 2;
					}
				}
			}
		}
	}
	return 0;
}
