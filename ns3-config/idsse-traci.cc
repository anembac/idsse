/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/* 
 * Based on the ezCar2x template for writing ns-3 scripts (credits to Karsten Roscher, Fraunhofer Institut).
 * Copyright (c) [2022] Thomas Rosenstatter, RISE Research Institutes of Sweden, All rights reserved.
 * Author: Thomas Rosenstatter
 */
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>

#include <boost/lexical_cast.hpp>
#include <boost/optional/optional_io.hpp>

#include "ns3/core-module.h"
#include "ns3/ezc2x-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/wifi-module.h"
#include "ns3/netanim-module.h"
#include "ns3/traci-vehicle.h"
#include "ns3/traci-module.h"
#include "ns3/lifecycle-module.h"
#include "ns3/synapse-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/trace-helper.h"

// #include <ezC2X/facility/cam/CaBasicService.hpp>
// #include <ezC2X/facility/cam/Cam.pb.h>
// #include <ezC2X/facility/denm/DenBasicService.hpp>
// #include <ezC2X/facility/denm/Denm.pb.h>

NS_LOG_COMPONENT_DEFINE("idsse-traci");

using namespace ns3;

/*
 * How-to add more command line parameters that modify the used configuration:
 *
 * 1.) Replace the specific value in the config file with a placeholder enclosed in
       '@@', e.g. @@TRIGGER_STATION@@
 * 2.) define a variable of the proper type in the 'Configuration' below
*  3.) map variable to command line option in function 'ConfigureCmdLine'
*  4.) add mapping to the 'UpdateCommonProperties' below, where the second
       parameter is the string used in (1) and the third is the variable defined in (2)
*/


struct Configuration
{
  bool verbose = true; 	// enable/disable logging
  std::string animate = "";	// animate and write to this file
  std::string ezRootDir = "/home/anders/kurser/thesis/include"; 	// root directory of the ezCar2X installation

  double runtime = 50.0;		// unit seconds [s]
  double activate = 0.0; 		// time for earliest activation of nodes [s]

  std::uint32_t numOfNodes = 5; 		// number of nodes
  std::uint32_t numOfAttackers =1;
  double equipRate = 1.0; 		// fraction of vehicles to equip

  bool deterministicChannel = 0;               // deterministic channel (enabled deterministic channel means disabled fading)
  double txPower = 23;		// transmission power in dB

  std::string configPath = "/home/anders/kurser/thesis/sim/ns-allinone-3.35/ns-3.35/scratch/idsse-traci.xml";	// configuration file to load for the nodes

  //std::uint64_t triggerStation = 0; // Station id sending maneuver request
  std::uint32_t triggerStart = 10000; // Offset time (ms) to trigger maneuver
  //std::uint32_t triggerInterval = 4000; // Interval time (ms) to repeat maneuver trigger

  bool isPeriodic = false;

  void
  ConfigureCmdLine(CommandLine& cmd)
  {
    cmd.AddValue ("verbose", "Tell application to log if true", verbose);
    cmd.AddValue ("ezc2x-root", "Root directory of the ezCar2X installation", ezRootDir);
    cmd.AddValue ("runtime", "Runtime in seconds", runtime);
    cmd.AddValue ("activate", "Earliest activation time for nodes in seconds", activate);

    cmd.AddValue ("nodes", "Number of nodes", numOfNodes);
    cmd.AddValue ("equip", "Equipment rate", equipRate);
    cmd.AddValue ("animate", "Write netanim to this file", animate);

    cmd.AddValue ("deterministic-channel", "Deterministic channel with disabled fading", deterministicChannel);
    cmd.AddValue ("txpower", "TX power per packet", txPower);
    cmd.AddValue ("config", "Configuration file for the ezCar2X stack on the nodes", configPath);

    // cmd.AddValue ("trigger-station", "Station id sending maneuver request", triggerStation);
    // cmd.AddValue ("trigger-start", "Offset time (ms) to trigger maneuver", triggerStart);
    // cmd.AddValue ("trigger-interval", "Interval time (ms) to repeat maneuver trigger", triggerInterval);

    cmd.AddValue ("periodic-trigger", "Triggers maneuvers in given intervals", isPeriodic);
  }
};


// Update properties based on configuration parameters
void
UpdateCommonProperties (boost::property_tree::ptree& properties, Configuration const& config)
{
  ReplacePropertyValue(properties, "@@EZC2X_ROOT_DIR@@", config.ezRootDir);
  // ReplacePropertyValue(properties, "@@TRIGGER_STATION@@", config.triggerStation);
  ReplacePropertyValue(properties, "@@TRIGGER_START@@", config.triggerStart);
  // ReplacePropertyValue(properties, "@@TRIGGER_INTERVAL@@", config.triggerInterval);

  ReplacePropertyValue(properties, "@@NUMBER_NODES@@", config.numOfNodes);
  ReplacePropertyValue(properties, "@@PERIODIC_BASED_TRIGGER@@", config.isPeriodic);

}

/*
 * Main function of the simulation.
 */
int
main (int argc, char *argv[])
{
  EnableSynapseComponents();

  NS_LOG_INFO("Simulation preparation.");

  // random stream index manually assigned
  int64_t stream = 0;

  /*
   * SUMO specific options
   */

  std::string traciHost ("127.0.0.1"); // "localhost" or "127.0.0.1"
  int traciPort = 12345;
  double erasureRate = 0.0;
  SumoConfig sumoConfig;

  /*
   * Prepare simulation.
   */

  Configuration config;

  // Parse command line options.
  CommandLine cmd;
  config.ConfigureCmdLine(cmd);
  sumoConfig.SetupCommandLine(cmd);

  cmd.AddValue ("host", "Hostname of the TraCI server", traciHost);
  cmd.AddValue ("port", "Port of the TraCI server", traciPort);
  cmd.AddValue ("loss", "Packet loss rate for each communication link", erasureRate);

  cmd.Parse (argc, argv);

  // enable logging from ezCar2X
  SetupEzC2XLogging ();

  // Enable component logging and initialize Ns3ConsoleSink (both optional).
  if (config.verbose)
    {
      LogComponentEnable ("idsse-traci", LogLevel (LOG_LEVEL_ALL | LOG_PREFIX_FUNC | LOG_PREFIX_TIME));
      
      LogComponentEnable ("EzC2XHelper", LogLevel (LOG_LEVEL_ALL | LOG_PREFIX_FUNC | LOG_PREFIX_TIME));
      LogComponentEnable ("EzC2XApplicationHelper", LogLevel (LOG_LEVEL_ALL | LOG_PREFIX_FUNC | LOG_PREFIX_TIME));
      LogComponentEnable ("EzC2XInstaller", LogLevel (LOG_LEVEL_ALL | LOG_PREFIX_FUNC | LOG_PREFIX_TIME));
      LogComponentEnable ("EzC2XApplicationInstaller", LogLevel (LOG_LEVEL_ALL | LOG_PREFIX_FUNC | LOG_PREFIX_TIME));

      // LogComponent "EzC2X" sets same log level for all ezCar2X components. Note (intentionally) switched debug and info levels:
      // LOG_LEVEL_ERROR (ns-3)-> Error (ezCar2X)
      // LOG_LEVEL_WARN        -> Warning
      // LOG_LEVEL_DEBUG       -> Info
      // LOG_LEVEL_INFO        -> Debug
      // LOG_LEVEL_LOGIC       -> Trace
      LogComponentEnable ("EzC2X", LogLevel (LOG_LEVEL_DEBUG | LOG_PREFIX_TIME));
      LogComponentEnable ("SumoInterface", LogLevel (LOG_LEVEL_WARN | LOG_PREFIX_TIME));
    }

  /*
   * Parse configuration file for the ezCar2X node.
   *
   * The properties from the file are used to configure the ezCar2X stack
   * instance on all nodes. See the provided 'config.xml' for details.
   */

  // Read XML properties.
  boost::property_tree::ptree properties;
  try
  {
      properties = ReadXmlProperties (config.configPath);
  }
  catch (std::exception const& e)
  {
      std::cout << "Fatal error, reading XML properties from '"
	  << config.configPath << ": " << e.what() << std::endl;

      return EXIT_FAILURE;
    }

  // Update properties with configuration options
  UpdateCommonProperties(properties, config);
  // Initialize TraCI helper.
  TraCiHelper traci;

  // Connect to TraCI server.
  NS_LOG_INFO ("Connecting to TraCI server at " << traciHost << ":" << traciPort);
  if (!traci.Connect (traciHost, traciPort))
    {
      std::cout << "Failed to connect to TraCI server @" << traciHost << ":" << traciPort << std::endl;
      return EXIT_FAILURE;
    }

  // if any run arguments were specified, we assume to run SUMO
  // via the SumoLauncher.py script
  if (sumoConfig.HasRunArguments ())
    {
      if (!traci.LaunchSumo(sumoConfig))
	{
	  std::cout << "Failed to launch SUMO" << std::endl;
	  return EXIT_FAILURE;
	}
    }

  /*
   * Setup simulation.
   */

  // Create node container for all other vehicles
  NodeContainer normalNodes;
  normalNodes.Create (config.numOfNodes-config.numOfAttackers);

  NodeContainer attackNodes;
  attackNodes.Create (config.numOfAttackers);

  NodeContainer vehicleNodes (normalNodes, attackNodes);
  // Install life cycle management on all vehicles
  NodeLifecycleHelper nlcHelper;
  nlcHelper.Install (vehicleNodes);

  // install mobility model
  traci.InstallMobilityModel (vehicleNodes);

  /*
   * First we setup the position and movement of all nodes.
   */

  /*
   * Now lets create the network. We start with the wireless channel and
   * attached wireless devices. We use the ItsG5Helper provided in the
   * its-g5 module for that.
   *
   * If required, configure the channel first before installing the devices.
   */
  ItsG5Helper itsg5Helper;
  ItsG5ChannelHelper channelHelper (ItsG5ChannelHelper::NakagamiHighwayDefault);

  // deterministic channel (disabled fading)
  channelHelper.SetDeterministic (config.deterministicChannel);

  // set custom channel
  Ptr<YansWifiChannel> wifiChannel = channelHelper.Create ();
  if (erasureRate > 0.0) // set constant loss rate, if specified in command line parameters
    {
      // TODO: a better approach would be to use the AddPreFadingLossExtension method of ItsG5ChannelHelper
      //       as this avoid re-generating the PropagationModel, requires erasureRate to be setable as an attribute
      Ptr<PropagationLossModel> innerPropagationModel = channelHelper.CreatePropagationLossModel ();
      Ptr<EzC2XErasureModel> erasureModel = Create<EzC2XErasureModel> ();
      erasureModel->SetErasureRate(erasureRate);
      erasureModel->SetNext(innerPropagationModel);
      wifiChannel->SetPropagationLossModel (erasureModel);

    }

  itsg5Helper.SetChannel(wifiChannel);

  // Set parameters from configuration
  itsg5Helper.SetTxPower (config.txPower);

  // Create WifiNetDevices and connect with the default channel.
  NetDeviceContainer devices = itsg5Helper.Install (vehicleNodes);
  // Enable lifecycle management of wifi devices
  // (will be switched to another channel if not active)
  ItsG5LifecycleHelper wifiLifecycle;
  wifiLifecycle.AddDevices (devices);
    /*
   * The remaining C2X stack is provided by ezCar2X.
   *
   * We will install it next using the EzC2XHelper and the
   * configuration provided under the node "ezC2X" in the
   * config file.
   */

  // Install ezC2X installation handlers
  EzC2XHelper ezHelper;
  ezHelper.SetErrorHandler ([](Ptr<Node> n, std::string const& error)
			    {
			      std::cout << "Failed to install and run ezCar2X on node " << n->GetId () << ": " << error << std::endl;
			      exit(EXIT_FAILURE);
			    });
  // std::out << prop?>;
  ezHelper.Install (vehicleNodes, properties.get_child ("ezC2X.Framework"));
  // properties.get_child("ezC2X.Framework")

/*

  Normal Vehicles get "Applications" installed

*/
  EzC2XApplicationHelper appHelper;
  appHelper.SetErrorHandler ([](Ptr<Node> n, std::string const& error)
		    {
		      std::cout << "Failed to install and run application(s) on node " << n->GetId () << ": " << error << std::endl;
		      exit(EXIT_FAILURE);
		    });
  appHelper.Install (vehicleNodes, properties.get_child("ezC2X.idsse"));
  // create node manager that sets the life cycle accordingly
  Ptr<traci::AcceptPartial> acceptor = Create<traci::AcceptPartial> (config.equipRate);
  stream += acceptor->AssignStreams (stream);

  /*
   * Create node manager for normal vehicles based on the provided
   * equipment rate.
   */

  Ptr<TraCiNodeLifecycleManager> nodeManager = Create<TraCiNodeLifecycleManager> (acceptor);
  nodeManager->AddNodes (normalNodes);
  nodeManager->SetRandomActivationDelay (
      CreateObjectWithAttributes<UniformRandomVariable> (
	  "Min", DoubleValue (0), "Max", DoubleValue (0.001)));

  nodeManager->SetEarliestActivationTime (Seconds (config.activate));

  // Add vehicle handler.
  traci.AddHandler (nodeManager, 0);

  /*
   * Create node manager for **attack** vehicles based on the provided
   * equipment rate.
   */

  Ptr<TraCiNodeLifecycleManager> attackNodeManager = Create<TraCiNodeLifecycleManager> (acceptor);
  attackNodeManager->AddNodes (attackNodes);
  attackNodeManager->SetRandomActivationDelay (
      CreateObjectWithAttributes<UniformRandomVariable> (
	  "Min", DoubleValue (0), "Max", DoubleValue (0.001)));

  // attackNodeManager->SetEarliestActivationTime (Seconds (config.activate));
  attackNodeManager->SetEarliestActivationTime (Seconds (5));

  // Add vehicle handler.
  traci.AddHandler (attackNodeManager, 0);

  TraCiGuiHelper guihelperNormal;
  TraCiGuiHelper guihelperAttack;
  traci::Color normalColor(0,155,133,255);
  traci::Color attackColor(255,0,0,255);
  // guihelperNormal.SetActiveColor(normalColor);
  // guihelperNormal.EnableActiveColor(normalNodes);
  // guihelperAttack.SetActiveColor(attackColor);
  // guihelperAttack.EnableActiveColor(attackNodes);


  // traci::Vehicle v1 = traci::Vehicle(traci.GetTraCi(),"1");
  // std::cout << "Getting vehicle speed " << v1.GetSpeed()<<std::endl;

  // if (vehicleNodes.GetN() > 1)
  // {
  //   // Ptr<Node> a = vehicleNodes.Get(0);
  //   for (NodeContainer::Iterator it = vehicleNodes.Begin (); it != vehicleNodes.End (); ++it)
  //   {
  //     v1.SetNode((*it));
  //     v1.setSpeed(0);

  //     std::cout << "Getting vehicle speed " << v1.GetSpeed()<<std::endl;
  //     ns3::traci::Color attackColor;
  //     attackColor.green = ns3::traci::UByte(255);
  //     v1.SetColor(attackColor);
	//     std::cout << std::endl;
  //   }
  // }




  /*
   * Do simulation.
   */
  Simulator::Stop (Seconds (config.runtime));

  // Toggle animation if file name is provided.
  std::unique_ptr<AnimationInterface> anim;
  if (!config.animate.empty())
    {
      anim.reset(new AnimationInterface(config.animate));
      anim->EnablePacketMetadata(true);
    }

  YansWifiPhyHelper wifiphyh = itsg5Helper.GetPhyHelper();
  // Record/log everything on physical layer as PCAP file
  wifiphyh.EnablePcapAll(std::string("idsse"));

  NS_LOG_INFO("Simulation started.");

  Simulator::Run ();
  NS_LOG_INFO("Simulation will be destroyed.");
  Simulator::Destroy ();

  traci.Disconnect();

  NS_LOG_INFO("Simulation finished.");

  return 0;
}
