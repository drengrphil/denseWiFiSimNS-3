/*
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
#include "ns3/wifi-net-device.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/yans-wifi-phy.h"
#include "ns3/error-rate-model.h"
#include "ns3/yans-error-rate-model.h"
#include "ns3/ptr.h"
#include "ns3/mobility-model.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/propagation-delay-model.h"
#include "ns3/vector.h"
#include "ns3/packet.h"
#include "ns3/simulator.h"
#include "ns3/nstime.h"
#include "ns3/command-line.h"
#include "ns3/wifi-tx-vector.h"

using namespace ns3;

/*Function to enable concurrent transmissions
 * and capture sources of interference for a typical STA_i
 * */
//First Generate random number to determining active APs at time t
int activeSTAs(int minU, int maxU){
	Ptr<UniformRandomVariable> actv = CreateObject<UniformRandomVariable>();
	return actv->GetInteger(minU, maxU);
}

//Second, Generate indexes of active STAs
int actvSTAind(){
	Ptr<UniformRandomVariable> actind = CreateObject<UniformRandomVariable>();
	return actind->GetInteger(0, 299);
}

/*Generate Random packet sizes
 * min packet size = 500 bytes and max packaet size = 1500
 * */
int payLoadSizeGenerator(int minPktSize, int maxPktSize){
	//srand(time(0));
	//return (rand() % (maxPktSize - minPktSize + 1) + minPktSize);
	Ptr<UniformRandomVariable> ra = CreateObject<UniformRandomVariable>();
	return ra->GetInteger(minPktSize, maxPktSize);
}

/*
 * Declare inputs for the interference measurement
 * using struct
 * */
class InterferenceMeasurement
{
public:
  struct Input
  {
    Input ();
    //Time interval for scheduled event 
    Time interval;
    //Position of desired transmitter
    double pos_DesiredSignal; //Get the X position from our mobility model
    //Position of interferer 
    double pos_InterfSignal; //get the X position from our mobility model
    std::string txModeA; //Set the Wifi Mode to be used for desired signal
    std::string txModeB; //Set the Wifi Mode of interferer node
    /*Set transmit power of 
     * */
    uint32_t txPowerLevelA; 
    uint32_t txPowerLevelB;
    /*Get Packet Size
     * */
    uint32_t packetSizeA;
    uint32_t packetSizeB;
    /*Define Wifi Standard to support
     * */
    enum WifiPhyStandard standard;
    enum WifiPreamble preamble;
  };
  //Interference Experiment
  InterferenceMeasurement ();
  void Run (struct InterferenceMeasurement::Input input);

private:
  /*Send Methods for desired signal and interferer
   * */
  void DesiredSend (void) const;
  void InterfererSend (void) const;
  Ptr<YansWifiPhy> m_txA;
  Ptr<YansWifiPhy> m_txB;
  struct Input m_input;
};

/*Method for allowing desired node transmit
 * */
void
InterferenceMeasurement::DesiredSend (void) const
{
  Ptr<Packet> p = Create<Packet> (m_input.packetSizeA);
  WifiTxVector txVector;
  txVector.SetTxPowerLevel (m_input.txPowerLevelA);
  txVector.SetMode (WifiMode (m_input.txModeA));
  m_txA->SendPacket (p, txVector, m_input.preamble);
}

/*Method for allowing interfering nodes transmit
 * */
void
InterferenceMeasurement::InterfererSend (void) const
{
  Ptr<Packet> p = Create<Packet> (m_input.packetSizeB);
  WifiTxVector txVector;
  txVector.SetTxPowerLevel (m_input.txPowerLevelB);
  txVector.SetMode (WifiMode (m_input.txModeB));
  m_txB->SendPacket (p, txVector, m_input.preamble);
}

InterferenceMeasurement::InterferenceMeasurement ()
{
}
InterferenceMeasurement::Input::Input ()
  : interval (MicroSeconds (0)),
    pos_DesiredSignal (-50),
    pos_InterfSignal (40),
	/* Set the constant Rate
	 * */
    txModeA ("OfdmRate54Mbps"),
    txModeB ("OfdmRate54Mbps"),
	//Transmit Power
    txPowerLevelA (0),
    txPowerLevelB (0),
	//Set Packet Size
    packetSizeA (1500),
    packetSizeB (1500),
	//Predefined the 802.11n Standard
    standard (WIFI_PHY_STANDARD_80211n_2_4GHZ),
	//Length of Preambale
    preamble (WIFI_PREAMBLE_LONG)
{
}

void
InterferenceMeasurement::Run (struct InterferenceMeasurement::Input input)
{
  m_input = input;

  //Number of active transmissions from STAs
  int numActive = activeSTAs(1, 300);
  std::vector<int>indexactivSTA(numActive);
  //Index of Active Users
  for(int k = 0; k < numActive; k++){
	  indexactivSTA[k] = actvSTAind();
  }
  /* Distance between receiver AP and interfering node
   * */


/* Compute interference range
 * */
  double Interfrrange = std::max (std::abs (input.pos_DesiredSignal), input.pos_InterfSignal);
  /*Use range propagation model to calculate power between
   * Desired node and interfering node
   * */
  Config::SetDefault ("ns3::RangePropagationLossModel::MaxRange", DoubleValue (Interfrrange));
  /* Set channel using YansWifiChannel
   * */
  Ptr<YansWifiChannel> channel = CreateObject<YansWifiChannel> ();
  channel->SetPropagationDelayModel (CreateObject<ConstantSpeedPropagationDelayModel> ());
  /*  Obtain path-loss
   * */
  Ptr<RangePropagationLossModel> pathloss_range = CreateObject<RangePropagationLossModel> ();
  //Install propagation loss model on channel
  channel->SetPropagationLossModel (pathloss_range);
  //Mobility Model
  Ptr<MobilityModel> pos_desiredNode = CreateObject<ConstantPositionMobilityModel> ();
  /* Set position of desired signal
   * */
  pos_desiredNode->SetPosition (Vector (input.pos_DesiredSignal, 0.0, 0.0));
  /* Set position of interfering node
   * */
  Ptr<MobilityModel> pos_interfrNode = CreateObject<ConstantPositionMobilityModel> ();
  pos_interfrNode->SetPosition (Vector (input.pos_InterfSignal, 0.0, 0.0));
  //Position of receiver AP
  Ptr<MobilityModel> pos_rxAP = CreateObject<ConstantPositionMobilityModel> ();
  pos_rxAP->SetPosition (Vector (0.0, 0.0, 0.0));

  m_txA = CreateObject<YansWifiPhy> ();
  m_txB = CreateObject<YansWifiPhy> ();
  Ptr<YansWifiPhy> rx = CreateObject<YansWifiPhy> ();
  // Error rate model
  Ptr<ErrorRateModel> error = CreateObject<YansErrorRateModel> ();
  m_txA->SetErrorRateModel (error);
  m_txB->SetErrorRateModel (error);
  rx->SetErrorRateModel (error);
  //Channel
  m_txA->SetChannel (channel);
  m_txB->SetChannel (channel);
  rx->SetChannel (channel);
  m_txA->SetMobility (pos_desiredNode);
  m_txB->SetMobility (pos_interfrNode);
  rx->SetMobility (pos_rxAP);

  /* Install WiFi standard on the transmitter
   * and the receiver
   * */
  m_txA->ConfigureStandard (input.standard);
  m_txB->ConfigureStandard (input.standard);
  /*Install Wifi Standard on the receiver
   * */
  rx->ConfigureStandard (input.standard);

  /*Schedule Transmission or Simulation
   * */
  Simulator::Schedule (Seconds (0), &InterferenceMeasurement::DesiredSend, this);
  Simulator::Schedule (Seconds (0) + input.interval, &InterferenceMeasurement::InterfererSend, this);
  /*Run Simulator
   * */
  Simulator::Run ();
  Simulator::Destroy ();

  //uint32_t sINR =
}


int main (int argc, char *argv[])
{
  InterferenceMeasurement::Input input;
  /*Define the WiFi standard to support
   * In this simulation, we chose 802.11n over 2.4GHz
   * */
  std::string str_standard = "WIFI_PHY_STANDARD_80211n_2_4GHZ";
  /*Preamble
   * */
  std::string str_preamble = "WIFI_PREAMBLE_LONG";
  /*Delay-time difference between transmission of
   * Desired signal and the interfering signal
   * */
  double delay = 0;

  /*Output traces of InterferenceHelper log
     */
  LogComponentEnable ("InterferenceHelper", LOG_LEVEL_ALL);

  /*With the helper of delay
   * Time interval is defined for start of each frame
   * */
  input.interval = MicroSeconds (delay);

  /* Set the WiFi Standard
   * */
  input.standard = WIFI_PHY_STANDARD_80211n_2_4GHZ;
  //Call the interference Class
  InterferenceMeasurement experiment;
  experiment.Run (input);

  return 0;
}
